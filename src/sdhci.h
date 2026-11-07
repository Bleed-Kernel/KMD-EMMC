#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <mm/spinlock.h>

// sd host controller registers (sd host controller simplified spec 3.0), offsets from bar 0
#define SDHCI_BLOCK_SIZE        0x04
#define SDHCI_BLOCK_COUNT       0x06
#define SDHCI_ARGUMENT          0x08
#define SDHCI_TRANSFER_MODE     0x0C
#define SDHCI_COMMAND           0x0E    // 16, writing it starts the command
#define SDHCI_RESPONSE          0x10    // 4 x 32
#define SDHCI_BUFFER            0x20    // pio data port
#define SDHCI_PRESENT_STATE     0x24
#define SDHCI_HOST_CONTROL      0x28    // 8
#define SDHCI_POWER_CONTROL     0x29    // 8
#define SDHCI_CLOCK_CONTROL     0x2C
#define SDHCI_TIMEOUT_CONTROL   0x2E    // 8
#define SDHCI_SOFTWARE_RESET    0x2F    // 8
#define SDHCI_INT_STATUS        0x30    // normal in the low 16, errors in the high 16
#define SDHCI_INT_ENABLE        0x34
#define SDHCI_SIGNAL_ENABLE     0x38
#define SDHCI_HOST_CONTROL2     0x3E    // 16
#define SDHCI_CAPABILITIES      0x40
#define SDHCI_CAPABILITIES_1    0x44
#define SDHCI_HOST_VERSION      0xFE    // 16, spec version in the low byte

#define SDHCI_SPEC_200          1
#define SDHCI_SPEC_300          2

// present state
#define PS_CMD_INHIBIT          (1u << 0)
#define PS_DAT_INHIBIT          (1u << 1)
#define PS_CARD_INSERTED        (1u << 16)
#define PS_DAT0                 (1u << 20)

// host control
#define HC_4BIT                 (1u << 1)
#define HC_HIGH_SPEED           (1u << 2)
#define HC_8BIT                 (1u << 5)

// host control 2
#define HC2_SIGNAL_180          (1u << 3)

// power control
#define PWR_ON                  (1u << 0)
#define PWR_180                 (5u << 1)
#define PWR_300                 (6u << 1)
#define PWR_330                 (7u << 1)

// clock control
#define CLK_INT_EN              (1u << 0)
#define CLK_INT_STABLE          (1u << 1)
#define CLK_CARD_EN             (1u << 2)

// software reset
#define RESET_ALL               (1u << 0)
#define RESET_CMD               (1u << 1)
#define RESET_DATA              (1u << 2)

// interrupt status
#define INT_CMD_DONE            (1u << 0)
#define INT_XFER_DONE           (1u << 1)
#define INT_BUF_WRITE           (1u << 4)
#define INT_BUF_READ            (1u << 5)
#define INT_ERROR               (1u << 15)
#define INT_CMD_TIMEOUT         (1u << 16)
#define INT_ERRORS              (0x3FFu << 16)
#define INT_WANTED              (INT_CMD_DONE | INT_XFER_DONE | INT_BUF_WRITE | INT_BUF_READ | INT_ERRORS)

// transfer mode
#define TM_BLKCNT_EN            (1u << 1)
#define TM_AUTO_CMD12           (1u << 2)
#define TM_READ                 (1u << 4)
#define TM_MULTI                (1u << 5)

// command register flags, the index goes in bits 13:8
#define CMD_RESP_NONE           0x00
#define CMD_RESP_136            0x01
#define CMD_RESP_48             0x02
#define CMD_RESP_48_BUSY        0x03
#define CMD_CRC                 0x08
#define CMD_INDEX               0x10
#define CMD_DATA                0x20

#define RESP_NONE               CMD_RESP_NONE
#define RESP_R1                 (CMD_RESP_48 | CMD_CRC | CMD_INDEX)
#define RESP_R1B                (CMD_RESP_48_BUSY | CMD_CRC | CMD_INDEX)
#define RESP_R2                 (CMD_RESP_136 | CMD_CRC)
#define RESP_R3                 CMD_RESP_48

// capabilities
#define CAP_BASE_CLOCK_SHIFT    8
#define CAP_8BIT                (1u << 18)
#define CAP_HIGH_SPEED          (1u << 21)
#define CAP_330                 (1u << 24)
#define CAP_300                 (1u << 25)
#define CAP_180                 (1u << 26)

#define SDHCI_BLOCK             512

typedef struct {
    uint64_t   mmio;            // virtual
    uint8_t    version;         // SDHCI_SPEC_
    uint32_t   caps;
    uint32_t   base_hz;
    uint32_t   clock_hz;        // what the card clock is running at
    spinlock_t lock;
} sdhci_t;

// a command out and its response back, flags is RESP_ plus CMD_DATA when a data phase follows
int  sdhci_cmd(sdhci_t *h, uint8_t index, uint32_t arg, uint16_t flags, uint16_t mode, uint32_t resp[4]);

// a command with blocks of data, read into or written from buf by pio
int  sdhci_data(sdhci_t *h, uint8_t index, uint32_t arg, uint16_t flags, bool read, void *buf, uint32_t blocks);

int  sdhci_init(sdhci_t *h, uint64_t mmio);
int  sdhci_set_clock(sdhci_t *h, uint32_t hz);
void sdhci_set_bus(sdhci_t *h, int width, bool high_speed);
void sdhci_reset_lines(sdhci_t *h);

// microseconds since boot, for timeouts
uint64_t sdhci_now_us(void);
