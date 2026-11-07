#include "emmc.h"
#include <user/errno.h>
#include <devices/devices.h>
#include <fs/vfs.h>
#include <fs/partition.h>
#include <mm/kalloc.h>
#include <drivers/serial/serial.h>
#include <ansii.h>
#include <string.h>
#include <stdio.h>

// /dev/mmc0 for the whole card and /dev/mmc0p<n> for its partitions, like nvme doe

typedef struct {
    emmc_t  *card;
    uint64_t lba_start;
    uint64_t sector_count;
} emmc_blk_t;

static long blk_io(INode_t *inode, void *buf, size_t count, size_t offset, bool read) {
    emmc_blk_t *blk = inode->internal_data;
    if (!blk || count == 0)
        return 0;
    if (!read && !blk->card->writable)
        return -EROFS;

    size_t max = (size_t)blk->sector_count * SDHCI_BLOCK;
    if (offset >= max)
        return 0;
    if (count > max - offset)
        count = max - offset;

    uint8_t *tmp = kmalloc(SDHCI_BLOCK);
    if (!tmp)
        return -ENOMEM;

    size_t total = 0;
    while (total < count) {
        uint64_t lba  = blk->lba_start + (offset + total) / SDHCI_BLOCK;
        size_t   skip = (offset + total) % SDHCI_BLOCK;
        size_t   left = count - total;
        uint8_t *at   = (uint8_t *)buf + total;
        int      r;

        // whole sectors straight in and out of the callers buffer
        if (skip == 0 && left >= SDHCI_BLOCK) {
            uint32_t n = (uint32_t)(left / SDHCI_BLOCK);
            if (n > EMMC_MAX_BLOCKS)
                n = EMMC_MAX_BLOCKS;
            r = read ? emmc_read(blk->card, lba, n, at) : emmc_write(blk->card, lba, n, at);
            if (r < 0)
                break;
            total += (size_t)n * SDHCI_BLOCK;
            continue;
        }

        // a piece of one, through tmp
        size_t chunk = SDHCI_BLOCK - skip;
        if (chunk > left)
            chunk = left;
        if (emmc_read(blk->card, lba, 1, tmp) < 0)
            break;
        if (read) {
            memcpy(at, tmp + skip, chunk);
        } else {
            memcpy(tmp + skip, at, chunk);
            if (emmc_write(blk->card, lba, 1, tmp) < 0)
                break;
        }
        total += chunk;
    }

    kfree(tmp);
    return total ? (long)total : -EIO;
}

static long blk_read(INode_t *inode, void *buf, size_t count, size_t offset) {
    return blk_io(inode, buf, count, offset, true);
}

static long blk_write(INode_t *inode, const void *buf, size_t count, size_t offset) {
    return blk_io(inode, (void *)buf, count, offset, false);
}

static size_t blk_size(INode_t *inode) {
    emmc_blk_t *blk = inode->internal_data;
    return blk ? (size_t)blk->sector_count * SDHCI_BLOCK : 0;
}

static INodeOps_t emmc_blk_ops = {
    .read  = blk_read,
    .write = blk_write,
    .size  = blk_size,
};

static void add_device(emmc_t *card, char *name, uint64_t start, uint64_t count) {
    emmc_blk_t *blk   = kmalloc(sizeof(*blk));
    INode_t    *inode = kmalloc(sizeof(*inode));
    if (!blk || !inode) {
        kfree(blk);
        kfree(inode);
        return;
    }
    blk->card         = card;
    blk->lba_start    = start;
    blk->sector_count = count;

    memset(inode, 0, sizeof(*inode));
    inode->type          = INODE_DEVICE;
    inode->ops           = &emmc_blk_ops;
    inode->internal_data = blk;
    inode->shared        = 1;

    if (device_register(inode, name) < 0) {
        serial_printf(LOG_WARN "emmc: couldnt register /dev/%s\n", name);
        kfree(inode);
        kfree(blk);
        return;
    }
    serial_printf(LOG_OK "emmc: /dev/%s, %llu sectors from %llu\n", name, count, start);
}

static void part_cb(void *drive, int drive_idx, int part_idx, uint64_t start, uint64_t count, uint8_t type) {
    (void)type;
    char name[16];
    snprintf(name, sizeof(name), "mmc%dp%d", drive_idx, part_idx + 1);
    add_device(drive, name, start, count);
}

static int part_read(void *drive, uint64_t lba, uint16_t count, void *buf) {
    return emmc_read(drive, lba, count, buf);
}

void emmc_register(emmc_t *card, int index) {
    char name[16];
    snprintf(name, sizeof(name), "mmc%d", index);
    add_device(card, name, 0, card->sectors);
    partition_probe(card, index, part_read, part_cb);
}
