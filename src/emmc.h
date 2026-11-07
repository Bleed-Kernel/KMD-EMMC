#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sdhci.h"

// emmc commands (jedec jesd84)
#define MMC_GO_IDLE             0
#define MMC_SEND_OP_COND        1
#define MMC_ALL_SEND_CID        2
#define MMC_SET_RELATIVE_ADDR   3
#define MMC_SWITCH              6
#define MMC_SELECT_CARD         7
#define MMC_SEND_EXT_CSD        8
#define MMC_SEND_CSD            9
#define MMC_SEND_STATUS         13
#define MMC_SET_BLOCKLEN        16
#define MMC_READ_SINGLE         17
#define MMC_READ_MULTIPLE       18
#define MMC_WRITE_SINGLE        24
#define MMC_WRITE_MULTIPLE      25

#define OCR_BUSY                (1u << 31)      // set once power up is done
#define OCR_SECTOR_MODE         (1u << 30)      // addresses are blocks, not bytes (over 2G)
#define OCR_VOLTAGES            0x00FF8080u     // 2.7-3.6v and 1.7-1.95v

// ext_csd bytes
#define EXT_CSD_BUS_WIDTH       183
#define EXT_CSD_HS_TIMING       185
#define EXT_CSD_REV             192
#define EXT_CSD_CARD_TYPE       196
#define EXT_CSD_SEC_COUNT       212     // 4 bytes little endian
#define EXT_CSD_BOOT_MULT       226     // boot partition size in 128k

#define CARD_TYPE_HS_26         (1u << 0)
#define CARD_TYPE_HS_52         (1u << 1)

#define STATUS_SWITCH_ERROR     (1u << 7)
#define STATUS_STATE(s)         (((s) >> 9) & 0xF)

#define EMMC_RCA                1
#define EMMC_MAX_BLOCKS         256     // per command, pio so theres no point going bigger

typedef struct {
    sdhci_t  host;
    bool     present;
    bool     sector_mode;
    bool     writable;          // emmc=rw on the kernel command line
    uint64_t sectors;
    int      bus_width;
    char     name[8];
    uint8_t  mid;
    uint8_t  ext_csd_rev;
} emmc_t;

// takes the card from power on to selected and ready for block io, 0 if theres an emmc there
int  emmc_card_init(emmc_t *card);

int  emmc_read(emmc_t *card, uint64_t lba, uint32_t count, void *buf);
int  emmc_write(emmc_t *card, uint64_t lba, uint32_t count, const void *buf);
void emmc_register(emmc_t *card, int index);
