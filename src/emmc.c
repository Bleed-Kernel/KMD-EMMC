#include "emmc.h"
#include <kmd/kmd.h>
#include <user/errno.h>
#include <mm/paging.h>
#include <kernel/bootargs.h>
#include <drivers/serial/serial.h>
#include <ansii.h>
#include <string.h>
#include <boot/bootloader_interface/generic_bootloader.h>

// emmc behind an sd host controller (pci class 08.05), the gemini lake chromebooks have theirs
// at 8086:31cc. sd cards on the same kind of controller arent handled, CMD1 finds no emmc there.
// read only unless the kernel command line has emmc=rw, it usually has an os on it

#define EMMC_MAX_CARDS  2
#define SDHCI_MMIO_SIZE 0x1000

static emmc_t cards[EMMC_MAX_CARDS];
static int    card_count;

static int emmc_probe(kmd_device_t *dev) {
    pci_device_t *pci = &dev->pci;
    serial_printf(LOG_INFO "emmc: sd host controller " PCI_SPEC " %04x:%04x prog if 0x%02x\n",
                  PCI_SPEC_ARGS(pci), pci->vendor_id, pci->device_id, pci->prog_if);
    if (card_count == EMMC_MAX_CARDS)
        return -EBUSY;

    uint16_t cmd = pci_read16(pci->bus, pci->device, pci->function, PCI_COMMAND);
    pci_write16(pci->bus, pci->device, pci->function, PCI_COMMAND, cmd | PCI_CMD_MEMORY | PCI_CMD_BUSMASTER);

    // a 64 bit bar has the top half in the next one
    uint64_t bar = pci->bar[0] & ~0xFull;
    if ((pci->bar[0] & 0x6) == 0x4)
        bar |= (uint64_t)pci->bar[1] << 32;
    if (!bar || (pci->bar[0] & 1)) {
        serial_printf(LOG_ERROR "emmc: bar 0 isnt memory (0x%08x)\n", pci->bar[0]);
        return -ENODEV;
    }
    if (paging_map_mmio(bar, SDHCI_MMIO_SIZE) < 0) {
        serial_printf(LOG_ERROR "emmc: cant map bar 0 at 0x%llx\n", bar);
        return -EIO;
    }

    emmc_t *card = &cards[card_count];
    memset(card, 0, sizeof(*card));
    if (sdhci_init(&card->host, bar + g_gbi.hhdm_offset) < 0)
        return -EIO;
    int r = emmc_card_init(card);
    if (r < 0) {
        // power off so the slot is left like we found it, near enough
        sdhci_set_clock(&card->host, 0);
        return r;
    }

    const char *rw = bootargs_get("emmc");
    card->writable = rw && !strcmp(rw, "rw");
    card->present  = true;
    serial_printf(LOG_OK "emmc: %s, %llu mb, %d bit at %u khz, %s\n", card->name, card->sectors / 2048,
                  card->bus_width, card->host.clock_hz / 1000,
                  card->writable ? "writable (emmc=rw)" : "read only, boot with emmc=rw to write to it");

    emmc_register(card, card_count);
    card_count++;
    return 0;
}

static const kmd_pci_match_t emmc_ids[] = {
    KMD_PCI_CLASS(0x08, 0x05, KMD_ANY),     // sd host controller
    KMD_PCI_END,
};

KMD_DRIVER(emmc_kmd) = {
    .description = "eMMC on SD host controllers",
    .pci         = emmc_ids,
    .probe       = emmc_probe,
};
