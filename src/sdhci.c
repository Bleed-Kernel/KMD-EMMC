#include "sdhci.h"
#include <user/errno.h>
#include <ACPI/acpi_hpet.h>
#include <drivers/serial/serial.h>
#include <ansii.h>
#include <string.h>

// the sd host controller, polled and pio only. everything waits by spinning on the hpet, this
// all runs during the boot probe or under the kernel lock

#define CMD_TIMEOUT_US      1000000
#define DATA_TIMEOUT_US     2000000
#define RESET_TIMEOUT_US    100000

// mmio helpers
static inline uint32_t rd32(sdhci_t *h, uint32_t off) { return *(volatile uint32_t *)(uintptr_t)(h->mmio + off); }
static inline uint16_t rd16(sdhci_t *h, uint32_t off) { return *(volatile uint16_t *)(uintptr_t)(h->mmio + off); }
static inline uint8_t  rd8 (sdhci_t *h, uint32_t off) { return *(volatile uint8_t  *)(uintptr_t)(h->mmio + off); }
static inline void wr32(sdhci_t *h, uint32_t off, uint32_t v) { *(volatile uint32_t *)(uintptr_t)(h->mmio + off) = v; }
static inline void wr16(sdhci_t *h, uint32_t off, uint16_t v) { *(volatile uint16_t *)(uintptr_t)(h->mmio + off) = v; }
static inline void wr8 (sdhci_t *h, uint32_t off, uint8_t v)  { *(volatile uint8_t  *)(uintptr_t)(h->mmio + off) = v; }

uint64_t sdhci_now_us(void) {
    return hpet_get_femtoseconds() / 1000000000ull;
}

// waits for any of bits in the interrupt status (or an error), the status is left for the caller
static int wait_int(sdhci_t *h, uint32_t bits, uint64_t timeout_us, uint32_t *status) {
    uint64_t end = sdhci_now_us() + timeout_us;
    for (;;) {
        uint32_t s = rd32(h, SDHCI_INT_STATUS);
        if (s & (bits | INT_ERRORS)) {
            *status = s;
            return (s & INT_ERRORS) ? -EIO : 0;
        }
        if (sdhci_now_us() > end) {
            *status = s;
            return -ETIMEDOUT;
        }
        asm volatile("pause");
    }
}

static int reset(sdhci_t *h, uint8_t what) {
    wr8(h, SDHCI_SOFTWARE_RESET, what);
    uint64_t end = sdhci_now_us() + RESET_TIMEOUT_US;
    while (rd8(h, SDHCI_SOFTWARE_RESET) & what) {
        if (sdhci_now_us() > end) {
            serial_printf(LOG_ERROR "sdhci: reset 0x%02x never finished\n", what);
            return -ETIMEDOUT;
        }
    }
    return 0;
}

// after an error the cmd and data state machines have to be reset or nothing else goes
void sdhci_reset_lines(sdhci_t *h) {
    reset(h, RESET_CMD);
    reset(h, RESET_DATA);
    wr32(h, SDHCI_INT_STATUS, 0xFFFFFFFF);
}

int sdhci_cmd(sdhci_t *h, uint8_t index, uint32_t arg, uint16_t flags, uint16_t mode, uint32_t resp[4]) {
    uint32_t inhibit = PS_CMD_INHIBIT;
    if ((flags & CMD_DATA) || (flags & 3) == CMD_RESP_48_BUSY)
        inhibit |= PS_DAT_INHIBIT;
    uint64_t end = sdhci_now_us() + CMD_TIMEOUT_US;
    while (rd32(h, SDHCI_PRESENT_STATE) & inhibit) {
        if (sdhci_now_us() > end) {
            serial_printf(LOG_ERROR "sdhci: CMD%u stuck behind inhibit (state 0x%08x)\n", index,
                          rd32(h, SDHCI_PRESENT_STATE));
            sdhci_reset_lines(h);
            return -ETIMEDOUT;
        }
    }

    wr32(h, SDHCI_INT_STATUS, 0xFFFFFFFF);
    wr32(h, SDHCI_ARGUMENT, arg);
    wr16(h, SDHCI_TRANSFER_MODE, mode);
    wr16(h, SDHCI_COMMAND, (uint16_t)((index << 8) | flags));

    uint32_t s;
    int r = wait_int(h, INT_CMD_DONE, CMD_TIMEOUT_US, &s);
    if (r < 0) {
        // no answer at all is normal while probing (an sd card gets no CMD1), so that ones quiet
        if (!(s & INT_CMD_TIMEOUT))
            serial_printf(LOG_ERROR "sdhci: CMD%u arg 0x%08x failed, status 0x%08x\n", index, arg, s);
        sdhci_reset_lines(h);
        return (s & INT_CMD_TIMEOUT) ? -ETIMEDOUT : r;
    }
    wr32(h, SDHCI_INT_STATUS, INT_CMD_DONE);

    if (resp) {
        if ((flags & 3) == CMD_RESP_136) {
            for (int i = 0; i < 4; i++)
                resp[i] = rd32(h, SDHCI_RESPONSE + i * 4);
        } else {
            resp[0] = rd32(h, SDHCI_RESPONSE);
        }
    }

    // r1b holds dat0 low while the card is busy, transfer complete says its done
    if ((flags & 3) == CMD_RESP_48_BUSY && !(flags & CMD_DATA)) {
        r = wait_int(h, INT_XFER_DONE, DATA_TIMEOUT_US, &s);
        // some hosts never say for a short busy, dat0 back high means the card is done anyway
        if (r == -ETIMEDOUT && (rd32(h, SDHCI_PRESENT_STATE) & PS_DAT0))
            r = 0;
        if (r < 0) {
            serial_printf(LOG_ERROR "sdhci: CMD%u busy never ended, status 0x%08x\n", index, s);
            sdhci_reset_lines(h);
            return r;
        }
        wr32(h, SDHCI_INT_STATUS, INT_XFER_DONE);
    }
    return 0;
}

int sdhci_data(sdhci_t *h, uint8_t index, uint32_t arg, uint16_t flags, bool read, void *buf, uint32_t blocks) {
    if (!blocks || blocks > 0xFFFF)
        return -EINVAL;

    // 512k sdma boundary bits, unused by pio but some controllers want them set
    wr16(h, SDHCI_BLOCK_SIZE, (uint16_t)((7u << 12) | SDHCI_BLOCK));
    wr16(h, SDHCI_BLOCK_COUNT, (uint16_t)blocks);

    uint16_t mode = TM_BLKCNT_EN | (read ? TM_READ : 0);
    if (blocks > 1)
        mode |= TM_MULTI | TM_AUTO_CMD12;

    uint32_t resp[4];
    int r = sdhci_cmd(h, index, arg, flags | CMD_DATA, mode, resp);
    if (r < 0)
        return r;

    uint32_t *p    = buf;
    uint32_t  want = read ? INT_BUF_READ : INT_BUF_WRITE;
    for (uint32_t b = 0; b < blocks; b++) {
        uint32_t s;
        r = wait_int(h, want, DATA_TIMEOUT_US, &s);
        if (r < 0) {
            serial_printf(LOG_ERROR "sdhci: CMD%u block %u of %u never ready, status 0x%08x\n",
                          index, b, blocks, s);
            sdhci_reset_lines(h);
            return r;
        }
        wr32(h, SDHCI_INT_STATUS, want);

        if (read) {
            for (int i = 0; i < SDHCI_BLOCK / 4; i++)
                *p++ = rd32(h, SDHCI_BUFFER);
        } else {
            for (int i = 0; i < SDHCI_BLOCK / 4; i++)
                wr32(h, SDHCI_BUFFER, *p++);
        }
    }

    // a write only completes once the card has stopped being busy with it
    uint32_t s;
    r = wait_int(h, INT_XFER_DONE, DATA_TIMEOUT_US, &s);
    if (r < 0) {
        serial_printf(LOG_ERROR "sdhci: CMD%u transfer never completed, status 0x%08x\n", index, s);
        sdhci_reset_lines(h);
        return r;
    }
    wr32(h, SDHCI_INT_STATUS, INT_XFER_DONE);
    return 0;
}

int sdhci_set_clock(sdhci_t *h, uint32_t hz) {
    wr16(h, SDHCI_CLOCK_CONTROL, 0);
    if (!hz)
        return 0;

    uint16_t reg;
    uint32_t actual;
    if (h->version >= SDHCI_SPEC_300) {
        // 10 bit divider, the clock is base / (2 * div), 0 means base
        uint32_t div = 0;
        if (h->base_hz > hz) {
            div = (h->base_hz + 2 * hz - 1) / (2 * hz);
            if (div > 1023)
                div = 1023;
        }
        actual = div ? h->base_hz / (2 * div) : h->base_hz;
        reg    = (uint16_t)(((div & 0xFF) << 8) | (((div >> 8) & 3) << 6));
    } else {
        // 8 bit and a power of two
        uint32_t div = 1;
        while (div < 256 && h->base_hz / div > hz)
            div <<= 1;
        actual = h->base_hz / div;
        reg    = (uint16_t)((div >> 1) << 8);
    }

    wr16(h, SDHCI_CLOCK_CONTROL, reg | CLK_INT_EN);
    uint64_t end = sdhci_now_us() + 20000;
    while (!(rd16(h, SDHCI_CLOCK_CONTROL) & CLK_INT_STABLE)) {
        if (sdhci_now_us() > end) {
            serial_printf(LOG_ERROR "sdhci: internal clock never stable for %u hz\n", hz);
            return -ETIMEDOUT;
        }
    }
    wr16(h, SDHCI_CLOCK_CONTROL, reg | CLK_INT_EN | CLK_CARD_EN);
    h->clock_hz = actual;
    serial_printf(LOG_INFO "sdhci: card clock %u khz (asked %u, base %u)\n", actual / 1000, hz / 1000, h->base_hz / 1000);
    return 0;
}

void sdhci_set_bus(sdhci_t *h, int width, bool high_speed) {
    uint8_t hc = rd8(h, SDHCI_HOST_CONTROL) & ~(HC_4BIT | HC_8BIT | HC_HIGH_SPEED);
    if (width == 8)
        hc |= HC_8BIT;
    else if (width == 4)
        hc |= HC_4BIT;
    if (high_speed)
        hc |= HC_HIGH_SPEED;
    wr8(h, SDHCI_HOST_CONTROL, hc);
}

int sdhci_init(sdhci_t *h, uint64_t mmio) {
    memset(h, 0, sizeof(*h));
    h->mmio    = mmio;
    h->version = (uint8_t)(rd16(h, SDHCI_HOST_VERSION) & 0xFF);
    h->caps    = rd32(h, SDHCI_CAPABILITIES);

    uint32_t base_mhz = (h->caps >> CAP_BASE_CLOCK_SHIFT) & (h->version >= SDHCI_SPEC_300 ? 0xFF : 0x3F);
    h->base_hz = base_mhz * 1000000;
    serial_printf(LOG_INFO "sdhci: spec %u.00, caps 0x%08x 0x%08x, base clock %u mhz, %s%s%s%s%s\n",
                  h->version + 1, h->caps, rd32(h, SDHCI_CAPABILITIES_1), base_mhz,
                  h->caps & CAP_8BIT ? "8 bit " : "", h->caps & CAP_HIGH_SPEED ? "high speed " : "",
                  h->caps & CAP_330 ? "3.3v " : "", h->caps & CAP_300 ? "3.0v " : "", h->caps & CAP_180 ? "1.8v" : "");
    if (!base_mhz) {
        // only the spec 3 vendor specific way left, nobody here does that
        serial_printf(LOG_ERROR "sdhci: no base clock in the capabilities\n");
        return -ENODEV;
    }

    if (reset(h, RESET_ALL) < 0)
        return -EIO;

    // polled, so every status bit on and no interrupts out
    wr32(h, SDHCI_INT_ENABLE, INT_WANTED);
    wr32(h, SDHCI_SIGNAL_ENABLE, 0);
    wr8(h, SDHCI_TIMEOUT_CONTROL, 0x0E);
    // presets off, intel glk has them broken (linux SDHCI_QUIRK2_PRESET_VALUE_BROKEN)
    if (h->version >= SDHCI_SPEC_300)
        wr16(h, SDHCI_HOST_CONTROL2, rd16(h, SDHCI_HOST_CONTROL2) & ~(1u << 15));

    // the highest voltage it can do, an emmc says in its ocr what it takes
    uint8_t pwr = h->caps & CAP_330 ? PWR_330 : h->caps & CAP_300 ? PWR_300 : PWR_180;
    wr8(h, SDHCI_POWER_CONTROL, 0);
    wr8(h, SDHCI_POWER_CONTROL, pwr);
    wr8(h, SDHCI_POWER_CONTROL, pwr | PWR_ON);
    hpet_busy_wait_ms(10);

    if (pwr == PWR_180 && h->version >= SDHCI_SPEC_300) {
        wr16(h, SDHCI_HOST_CONTROL2, rd16(h, SDHCI_HOST_CONTROL2) | HC2_SIGNAL_180);
        hpet_busy_wait_ms(5);
        serial_printf(LOG_INFO "sdhci: 1.8v signalling, host control 2 0x%04x\n", rd16(h, SDHCI_HOST_CONTROL2));
    }
    serial_printf(LOG_INFO "sdhci: bus power %s, present state 0x%08x\n",
                  pwr == PWR_330 ? "3.3v" : pwr == PWR_300 ? "3.0v" : "1.8v", rd32(h, SDHCI_PRESENT_STATE));

    sdhci_set_bus(h, 1, false);
    return sdhci_set_clock(h, 400000);
}
