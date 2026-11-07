#include "emmc.h"
#include <user/errno.h>
#include <ACPI/acpi_hpet.h>
#include <mm/kalloc.h>
#include <drivers/serial/serial.h>
#include <ansii.h>
#include <string.h>

#define OP_COND_TIMEOUT_US  1000000

static int status(emmc_t *card, uint32_t *out) {
    uint32_t resp[4];
    int r = sdhci_cmd(&card->host, MMC_SEND_STATUS, EMMC_RCA << 16, RESP_R1, 0, resp);
    if (r == 0)
        *out = resp[0];
    return r;
}

// one ext_csd byte, the card is busy until its applied and says if it took it in its status
static int mmc_switch(emmc_t *card, uint8_t index, uint8_t value) {
    uint32_t arg = (3u << 24) | ((uint32_t)index << 16) | ((uint32_t)value << 8);
    int r = sdhci_cmd(&card->host, MMC_SWITCH, arg, RESP_R1B, 0, NULL);
    if (r < 0) {
        serial_printf(LOG_ERROR "emmc: switch ext_csd[%u] = %u failed (%d)\n", index, value, r);
        return r;
    }
    uint32_t st = 0;
    if (status(card, &st) < 0 || (st & STATUS_SWITCH_ERROR)) {
        serial_printf(LOG_ERROR "emmc: switch ext_csd[%u] = %u refused, status 0x%08x\n", index, value, st);
        return -EIO;
    }
    return 0;
}

static int read_ext_csd(emmc_t *card, uint8_t *ext) {
    return sdhci_data(&card->host, MMC_SEND_EXT_CSD, 0, RESP_R1, true, ext, 1);
}

// after changing the bus, ext_csd has to read back the same as it did before
static bool ext_csd_matches(emmc_t *card, const uint8_t *before, uint8_t *scratch) {
    if (read_ext_csd(card, scratch) < 0)
        return false;
    return !memcmp(scratch + EXT_CSD_SEC_COUNT, before + EXT_CSD_SEC_COUNT, 4) &&
           scratch[EXT_CSD_REV] == before[EXT_CSD_REV] && scratch[EXT_CSD_CARD_TYPE] == before[EXT_CSD_CARD_TYPE];
}

// 8 then 4 then 1 bit, whatever the host can do and still reads ext_csd right
static void pick_bus_width(emmc_t *card, uint8_t *ext, uint8_t *scratch) {
    static const struct { int width; uint8_t value; } tries[] = { { 8, 2 }, { 4, 1 } };

    for (int i = 0; i < 2; i++) {
        if (tries[i].width == 8 && !(card->host.caps & CAP_8BIT))
            continue;
        if (mmc_switch(card, EXT_CSD_BUS_WIDTH, tries[i].value) < 0)
            continue;
        sdhci_set_bus(&card->host, tries[i].width, false);
        if (ext_csd_matches(card, ext, scratch)) {
            card->bus_width = tries[i].width;
            serial_printf(LOG_INFO "emmc: %d bit bus\n", tries[i].width);
            return;
        }
        serial_printf(LOG_WARN "emmc: %d bit bus didnt read back right, trying narrower\n", tries[i].width);
        sdhci_set_bus(&card->host, 1, false);
        mmc_switch(card, EXT_CSD_BUS_WIDTH, 0);
    }
    card->bus_width = 1;
    serial_printf(LOG_INFO "emmc: 1 bit bus\n");
}

static void pick_speed(emmc_t *card, uint8_t *ext, uint8_t *scratch) {
    bool hs52 = (ext[EXT_CSD_CARD_TYPE] & CARD_TYPE_HS_52) && (card->host.caps & CAP_HIGH_SPEED);
    if (!hs52 || mmc_switch(card, EXT_CSD_HS_TIMING, 1) < 0) {
        serial_printf(LOG_INFO "emmc: staying at 26 mhz (card type 0x%02x)\n", ext[EXT_CSD_CARD_TYPE]);
        return;
    }

    sdhci_set_bus(&card->host, card->bus_width, true);
    sdhci_set_clock(&card->host, 52000000);
    if (ext_csd_matches(card, ext, scratch)) {
        serial_printf(LOG_INFO "emmc: high speed, 52 mhz\n");
        return;
    }

    serial_printf(LOG_WARN "emmc: 52 mhz didnt read back right, back to 26\n");
    sdhci_set_clock(&card->host, 26000000);
    sdhci_set_bus(&card->host, card->bus_width, false);
    mmc_switch(card, EXT_CSD_HS_TIMING, 0);
}

// cid bytes big end first, the host drops the crc byte and shifts the rest down 8
static void cid_bytes(const uint32_t r[4], uint8_t out[16]) {
    for (int i = 0; i < 15; i++) {
        int bit  = 120 - 8 * i - 8;         // bits below the crc
        int word = bit / 32;
        out[i]   = (uint8_t)(r[word] >> (bit % 32));
    }
    out[15] = 0;
}

int emmc_card_init(emmc_t *card) {
    sdhci_t *h = &card->host;
    uint32_t resp[4];

    sdhci_cmd(h, MMC_GO_IDLE, 0, RESP_NONE, 0, NULL);
    hpet_busy_wait_ms(1);

    // op cond until the card isnt busy, an sd card or an empty slot never answers this
    uint32_t ocr = 0;
    uint64_t end = sdhci_now_us() + OP_COND_TIMEOUT_US;
    int tries = 0;
    for (;;) {
        int r = sdhci_cmd(h, MMC_SEND_OP_COND, OCR_SECTOR_MODE | OCR_VOLTAGES, RESP_R3, 0, resp);
        if (r < 0) {
            serial_printf(LOG_INFO "emmc: no answer to CMD1 (%d), theres no emmc on this slot\n", r);
            return -ENODEV;
        }
        tries++;
        ocr = resp[0];
        if (ocr & OCR_BUSY)
            break;
        if (sdhci_now_us() > end) {
            serial_printf(LOG_ERROR "emmc: still powering up after %d tries, ocr 0x%08x\n", tries, ocr);
            return -ETIMEDOUT;
        }
        hpet_busy_wait_ms(1);
    }
    card->sector_mode = (ocr & OCR_SECTOR_MODE) != 0;
    serial_printf(LOG_INFO "emmc: powered up after %d tries, ocr 0x%08x, %s addressing\n",
                  tries, ocr, card->sector_mode ? "sector" : "byte");

    if (sdhci_cmd(h, MMC_ALL_SEND_CID, 0, RESP_R2, 0, resp) < 0)
        return -EIO;
    uint8_t cid[16];
    cid_bytes(resp, cid);
    card->mid = cid[0];
    for (int i = 0; i < 6; i++)
        card->name[i] = (cid[3 + i] >= 0x20 && cid[3 + i] < 0x7F) ? (char)cid[3 + i] : '?';
    card->name[6] = '\0';
    serial_printf(LOG_INFO "emmc: cid %08x %08x %08x %08x, maker 0x%02x, name %s\n",
                  resp[3], resp[2], resp[1], resp[0], card->mid, card->name);

    // emmc takes the address its given, unlike sd which picks one
    if (sdhci_cmd(h, MMC_SET_RELATIVE_ADDR, EMMC_RCA << 16, RESP_R1, 0, resp) < 0)
        return -EIO;
    if (sdhci_cmd(h, MMC_SEND_CSD, EMMC_RCA << 16, RESP_R2, 0, resp) < 0)
        return -EIO;
    serial_printf(LOG_INFO "emmc: csd %08x %08x %08x %08x\n", resp[3], resp[2], resp[1], resp[0]);

    if (sdhci_cmd(h, MMC_SELECT_CARD, EMMC_RCA << 16, RESP_R1B, 0, NULL) < 0)
        return -EIO;

    sdhci_set_clock(h, 26000000);

    uint8_t *ext     = kmalloc(512);
    uint8_t *scratch = kmalloc(512);
    if (!ext || !scratch) {
        kfree(ext);
        kfree(scratch);
        return -ENOMEM;
    }

    int r = read_ext_csd(card, ext);
    if (r < 0) {
        serial_printf(LOG_ERROR "emmc: cant read ext_csd (%d)\n", r);
        kfree(ext);
        kfree(scratch);
        return r;
    }
    card->ext_csd_rev = ext[EXT_CSD_REV];
    card->sectors = (uint64_t)ext[EXT_CSD_SEC_COUNT] | ((uint64_t)ext[EXT_CSD_SEC_COUNT + 1] << 8) |
                    ((uint64_t)ext[EXT_CSD_SEC_COUNT + 2] << 16) | ((uint64_t)ext[EXT_CSD_SEC_COUNT + 3] << 24);
    serial_printf(LOG_INFO "emmc: ext_csd rev %u, card type 0x%02x, %llu sectors (%llu mb), boot partitions %u kb each\n",
                  card->ext_csd_rev, ext[EXT_CSD_CARD_TYPE], card->sectors, card->sectors / 2048,
                  ext[EXT_CSD_BOOT_MULT] * 128);

    pick_bus_width(card, ext, scratch);
    pick_speed(card, ext, scratch);
    kfree(ext);
    kfree(scratch);

    // only byte addressed cards care, but its harmless on the rest
    sdhci_cmd(h, MMC_SET_BLOCKLEN, SDHCI_BLOCK, RESP_R1, 0, NULL);

    uint32_t st = 0;
    if (status(card, &st) == 0)
        serial_printf(LOG_INFO "emmc: ready, status 0x%08x (state %u)\n", st, STATUS_STATE(st));
    return 0;
}

static int emmc_rw(emmc_t *card, bool read, uint64_t lba, uint32_t count, void *buf) {
    if (!card->present)
        return -ENODEV;
    if (lba + count > card->sectors)
        return -EINVAL;

    uint8_t *p = buf;
    while (count) {
        uint32_t n = count > EMMC_MAX_BLOCKS ? EMMC_MAX_BLOCKS : count;
        uint32_t addr = card->sector_mode ? (uint32_t)lba : (uint32_t)(lba * SDHCI_BLOCK);
        uint8_t cmd = read ? (n > 1 ? MMC_READ_MULTIPLE : MMC_READ_SINGLE)
                           : (n > 1 ? MMC_WRITE_MULTIPLE : MMC_WRITE_SINGLE);

        spinlock_acquire(&card->host.lock);
        int r = sdhci_data(&card->host, cmd, addr, RESP_R1, read, p, n);
        spinlock_release(&card->host.lock);
        if (r < 0) {
            serial_printf(LOG_ERROR "emmc: %s of %u blocks at %llu failed (%d)\n", read ? "read" : "write", n, lba, r);
            return -EIO;
        }
        p     += (size_t)n * SDHCI_BLOCK;
        lba   += n;
        count -= n;
    }
    return 0;
}

int emmc_read(emmc_t *card, uint64_t lba, uint32_t count, void *buf) {
    return emmc_rw(card, true, lba, count, buf);
}

int emmc_write(emmc_t *card, uint64_t lba, uint32_t count, const void *buf) {
    if (!card->writable)
        return -EROFS;
    return emmc_rw(card, false, lba, count, (void *)buf);
}
