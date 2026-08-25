// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 Whimory FTL read-only block devices + host partition aliases.
 *
 * Hardware: N31 is NAND-only (no SPI/NOR utility flash from nano4G onward).
 * Whimory "FPart" (PPNFPart) manages special blocks under FTL — it is NOT the
 * host MBR/name table. Host-visible slices (classic + N5/N6/N7 family) are:
 *
 *   firmware  — IMG1 / MSE (osos, rsrc, disk, gpfw, …) or "[hi]" style
 *   ipod      — FAT32 user volume (Windows D:\, iPod_Control, n31os)
 *
 * This module:
 *   /dev/s5l8740-ftl       — whole FTL LBA space (4096 B sectors)
 *   /dev/s5l8740-firmware  — firmware slice if discovered / forced
 *   /dev/s5l8740-ipod      — user FAT slice (or whole disk if superfloppy)
 *   /dev/s5l8740-rsrc      — optional resource FS inside firmware
 *
 * Low-level NAND I/O: fmss-s5l8740.ko
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/errno.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stdarg.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "fmss-s5l8740-api.h"

#define FTL_DISK_NAME		"s5l8740-ftl"
#define FTL_VALIDATE_HEX	128
#define FPART_MAX_DISKS		4
#define FPART_SCAN_LBAS		4096u

enum fpart_kind {
	FPART_WHOLE = 0,
	FPART_FIRMWARE,
	FPART_IPOD,
	FPART_RSRC,
};

struct fpart_slice {
	const char *name;
	enum fpart_kind kind;
	u64 start_lba;	/* 4096-byte FTL sectors */
	u64 nsectors;
	bool present;
	struct gendisk *disk;
};

static u64 ftl_capacity = FMSS_FTL_DEFAULT_CAPACITY;
module_param(ftl_capacity, ullong, 0644);
MODULE_PARM_DESC(ftl_capacity,
		 "FTL logical sector count (4096 B; N31 default ~3856968)");

static unsigned int ftl_map_max_lpn;
module_param(ftl_map_max_lpn, uint, 0644);

static bool ftl_auto_map;
module_param(ftl_auto_map, bool, 0644);

/* Manual overrides (4K LBAs). 0 = auto / unused. */
static unsigned long fw_start_lba;
module_param(fw_start_lba, ulong, 0644);
MODULE_PARM_DESC(fw_start_lba, "Firmware slice start (4K LBA, default 0)");

static unsigned long fw_nsectors;
module_param(fw_nsectors, ulong, 0644);
MODULE_PARM_DESC(fw_nsectors,
		 "Firmware slice size in 4K sectors (0=auto from scan/FWPartSize)");

static unsigned long ipod_start_lba;
module_param(ipod_start_lba, ulong, 0644);
MODULE_PARM_DESC(ipod_start_lba, "User FAT start 4K LBA (0=auto)");

static unsigned long ipod_nsectors;
module_param(ipod_nsectors, ulong, 0644);
MODULE_PARM_DESC(ipod_nsectors, "User FAT size 4K sectors (0=to end of FTL)");

static bool fpart_auto_scan = true;
module_param(fpart_auto_scan, bool, 0644);
MODULE_PARM_DESC(fpart_auto_scan, "Scan FTL for MBR/[hi]/FAT and create named disks");

static struct gendisk *ftl_disk;
static struct platform_device *ftl_pdev;
static char fpart_status[PAGE_SIZE];
static unsigned int fpart_status_len;

static struct fpart_slice slices[FPART_MAX_DISKS] = {
	{ .name = "s5l8740-firmware", .kind = FPART_FIRMWARE },
	{ .name = "s5l8740-ipod", .kind = FPART_IPOD },
	{ .name = "s5l8740-rsrc", .kind = FPART_RSRC },
};

static bool is_apple_fat_bpb(const u8 *s)
{
	if (s[0] != 0xeb && s[0] != 0xe9)
		return false;
	/* LE bytes/sector 512 or 4096 */
	{
		u16 bps = s[11] | (s[12] << 8);

		if (bps != 512 && bps != 4096)
			return false;
	}
	if (s[3] == '*' && s[4] == 'U' && s[5] == 'O')
		return true;
	if (s[0x52] == 'F' && s[0x53] == 'A' && s[0x54] == '3')
		return true;
	if (s[510] == 0x55 && s[511] == 0xaa)
		return true;
	return false;
}

static bool is_hi_firmware_hdr(const u8 *s)
{
	/* Classic firmware volume header: "[hi]" at +0x100 (LE magic). */
	return s[0x100] == '[' && s[0x101] == 'h' &&
	       s[0x102] == 'i' && s[0x103] == ']';
}

static bool is_mbr(const u8 *s)
{
	return s[510] == 0x55 && s[511] == 0xaa &&
	       (s[0x1be + 4] != 0 || s[0x1ce + 4] != 0 ||
		s[0x1de + 4] != 0 || s[0x1ee + 4] != 0);
}

static bool looks_img1_dir(const u8 *s)
{
	/* Loose: FourCC-ish names used in Apple MSE (LE dword text). */
	static const char *const tags[] = {
		"osos", "soso", "rsrc", "crsr", "disk", "ksid",
		"gpfw", "wfpg", "appl", NULL
	};
	unsigned int i, t;

	for (i = 0; i + 40 <= 512; i += 40) {
		for (t = 0; tags[t]; t++) {
			if (!memcmp(s + i + 4, tags[t], 4) ||
			    !memcmp(s + i, tags[t], 4))
				return true;
		}
	}
	return false;
}

static int ftl_read_lba(u64 lba, u8 *buf)
{
	if (lba >= ftl_capacity)
		return -ERANGE;
	return fmss_ftl_read_sector(lba, buf);
}

static void fpart_status_reset(void)
{
	fpart_status_len = 0;
}

static void fpart_status_printf(const char *fmt, ...)
{
	va_list ap;
	int n;

	if (fpart_status_len >= sizeof(fpart_status) - 1)
		return;
	va_start(ap, fmt);
	n = vscnprintf(fpart_status + fpart_status_len,
		       sizeof(fpart_status) - fpart_status_len, fmt, ap);
	va_end(ap);
	if (n > 0)
		fpart_status_len += n;
}

static void fpart_clear_slices(void)
{
	int i;

	for (i = 0; i < FPART_MAX_DISKS; i++) {
		if (slices[i].disk) {
			del_gendisk(slices[i].disk);
			put_disk(slices[i].disk);
			slices[i].disk = NULL;
		}
		slices[i].present = false;
		slices[i].start_lba = 0;
		slices[i].nsectors = 0;
	}
}

static void ftl_submit_bio_range(struct bio *bio, u64 start_lba, u64 nsectors)
{
	struct bio_vec bvec;
	struct bvec_iter iter;
	u8 *secbuf;
	u64 pos;
	int ret = 0;

	if (bio_op(bio) != REQ_OP_READ) {
		bio_io_error(bio);
		return;
	}

	secbuf = kmalloc(FMSS_FTL_SECTOR_SIZE, GFP_NOIO);
	if (!secbuf) {
		bio_io_error(bio);
		return;
	}

	pos = (u64)bio->bi_iter.bi_sector << 9;

	bio_for_each_segment(bvec, bio, iter) {
		unsigned long seg_done = 0;

		while (seg_done < bvec.bv_len) {
			u64 byte = pos + seg_done;
			u64 lsec = start_lba + byte / FMSS_FTL_SECTOR_SIZE;
			unsigned int off = byte % FMSS_FTL_SECTOR_SIZE;
			unsigned int chunk = min_t(unsigned int,
						   FMSS_FTL_SECTOR_SIZE - off,
						   bvec.bv_len - seg_done);

			if (byte / FMSS_FTL_SECTOR_SIZE >= nsectors ||
			    lsec >= ftl_capacity) {
				ret = -EIO;
				goto out;
			}

			ret = fmss_ftl_read_sector(lsec, secbuf);
			if (ret)
				goto out;

			{
				void *page_addr = kmap_local_page(bvec.bv_page);

				memcpy(page_addr + bvec.bv_offset + seg_done,
				       secbuf + off, chunk);
				kunmap_local(page_addr);
			}
			seg_done += chunk;
		}
		pos += bvec.bv_len;
	}

out:
	kfree(secbuf);
	if (ret)
		bio_io_error(bio);
	else
		bio_endio(bio);
}

static void ftl_submit_bio(struct bio *bio)
{
	ftl_submit_bio_range(bio, 0, ftl_capacity);
}

static void fpart_submit_bio(struct bio *bio)
{
	struct fpart_slice *sl = bio->bi_bdev->bd_disk->private_data;

	if (!sl || !sl->present) {
		bio_io_error(bio);
		return;
	}
	ftl_submit_bio_range(bio, sl->start_lba, sl->nsectors);
}

static const struct block_device_operations ftl_bd_ops = {
	.owner		= THIS_MODULE,
	.submit_bio	= ftl_submit_bio,
};

static const struct block_device_operations fpart_bd_ops = {
	.owner		= THIS_MODULE,
	.submit_bio	= fpart_submit_bio,
};

static int fpart_register_slice(struct fpart_slice *sl)
{
	struct queue_limits lim = {
		.logical_block_size = FMSS_FTL_SECTOR_SIZE,
		.physical_block_size = FMSS_FTL_SECTOR_SIZE,
	};
	struct gendisk *disk;
	int ret;

	if (!sl->present || !sl->nsectors)
		return 0;

	disk = blk_alloc_disk(&lim, NUMA_NO_NODE);
	if (IS_ERR(disk))
		return PTR_ERR(disk);

	disk->first_minor = 0;
	disk->flags = GENHD_FL_NO_PART;
	disk->fops = &fpart_bd_ops;
	disk->private_data = sl;
	snprintf(disk->disk_name, DISK_NAME_LEN, "%s", sl->name);
	set_capacity(disk, sl->nsectors * (FMSS_FTL_SECTOR_SIZE / 512));

	ret = add_disk(disk);
	if (ret) {
		put_disk(disk);
		return ret;
	}
	sl->disk = disk;
	dev_info(&ftl_pdev->dev,
		 "/dev/%s start_lba=%llu nsectors=%llu (%llu MiB)\n",
		 sl->name, sl->start_lba, sl->nsectors,
		 (sl->nsectors * FMSS_FTL_SECTOR_SIZE) >> 20);
	return 0;
}

static struct fpart_slice *fpart_by_kind(enum fpart_kind k)
{
	int i;

	for (i = 0; i < FPART_MAX_DISKS; i++)
		if (slices[i].kind == k)
			return &slices[i];
	return NULL;
}

static void fpart_set(enum fpart_kind k, u64 start, u64 nsec)
{
	struct fpart_slice *sl = fpart_by_kind(k);

	if (!sl || !nsec || start >= ftl_capacity)
		return;
	if (start + nsec > ftl_capacity)
		nsec = ftl_capacity - start;
	sl->start_lba = start;
	sl->nsectors = nsec;
	sl->present = true;
}

/*
 * Discover host partitions inside the FTL LBA space.
 * N31: FTL capacity often already equals the user FAT (WMR_Partition).
 * Still probe for classic MBR / [hi] / second FAT so firmware can be split out.
 */
static int fpart_scan_ex(unsigned int scan_n)
{
	u8 *sec;
	u64 i, fat0 = ~0ULL, fat1 = ~0ULL, hi_lba = ~0ULL;
	u64 mbr_fat_start = ~0ULL, mbr_fat_size = 0;
	u64 mbr_other_start = ~0ULL, mbr_other_size = 0;
	bool have_mbr = false, l0_fat = false;
	int ret;

	fpart_clear_slices();
	fpart_status_reset();

	sec = kmalloc(FMSS_FTL_SECTOR_SIZE, GFP_KERNEL);
	if (!sec)
		return -ENOMEM;

	if (!scan_n)
		scan_n = 64;
	if (scan_n > FPART_SCAN_LBAS)
		scan_n = FPART_SCAN_LBAS;
	if (scan_n > ftl_capacity)
		scan_n = (unsigned int)ftl_capacity;

	ret = ftl_read_lba(0, sec);
	if (ret) {
		fpart_status_printf("LBA0 read failed %d\n", ret);
		kfree(sec);
		return ret;
	}

	l0_fat = is_apple_fat_bpb(sec);
	if (is_hi_firmware_hdr(sec)) {
		hi_lba = 0;
		fpart_status_printf("LBA0: [hi] firmware volume header\n");
	}
	if (is_mbr(sec)) {
		unsigned int p;

		have_mbr = true;
		fpart_status_printf("LBA0: MBR partition table\n");
		for (p = 0; p < 4; p++) {
			const u8 *e = sec + 0x1be + p * 16;
			u8 type = e[4];
			u32 start512 = e[8] | (e[9] << 8) | (e[10] << 16) |
				       (e[11] << 24);
			u32 size512 = e[12] | (e[13] << 8) | (e[14] << 16) |
				      (e[15] << 24);
			u64 start4k = (u64)start512 / 8;
			u64 size4k = (u64)size512 / 8;

			if (!type || !size512)
				continue;
			fpart_status_printf(
				"  mbr[%u] type=0x%02x start4k=%llu size4k=%llu\n",
				p, type, start4k, size4k);
			if (type == 0x0b || type == 0x0c || type == 0x1b ||
			    type == 0x1c) {
				mbr_fat_start = start4k;
				mbr_fat_size = size4k;
			} else if (mbr_other_start == ~0ULL) {
				mbr_other_start = start4k;
				mbr_other_size = size4k;
			}
		}
	}
	if (l0_fat)
		fpart_status_printf("LBA0: Apple/FAT BPB (superfloppy or volume)\n");
	if (looks_img1_dir(sec))
		fpart_status_printf("LBA0: possible IMG1/MSE directory tags\n");

	if (fw_nsectors)
		fpart_set(FPART_FIRMWARE, fw_start_lba, fw_nsectors);
	if (ipod_start_lba || ipod_nsectors) {
		u64 st = ipod_start_lba;
		u64 ns = ipod_nsectors ? ipod_nsectors : (ftl_capacity - st);

		fpart_set(FPART_IPOD, st, ns);
	}

	if (!fpart_by_kind(FPART_IPOD)->present) {
		if (have_mbr && mbr_fat_start != ~0ULL && mbr_fat_size) {
			fpart_set(FPART_IPOD, mbr_fat_start, mbr_fat_size);
			if (!fpart_by_kind(FPART_FIRMWARE)->present &&
			    mbr_other_start != ~0ULL)
				fpart_set(FPART_FIRMWARE, mbr_other_start,
					  mbr_other_size);
		} else if (l0_fat) {
			fpart_set(FPART_IPOD, 0, ftl_capacity);
			fpart_status_printf(
				"layout: superfloppy — FTL == ipod userdata\n");
		}
	}

	for (i = 1; i < scan_n; i++) {
		if (ftl_read_lba(i, sec))
			continue;
		if (hi_lba == ~0ULL && is_hi_firmware_hdr(sec)) {
			hi_lba = i;
			fpart_status_printf("LBA%llu: [hi] firmware header\n", i);
		}
		if (is_apple_fat_bpb(sec)) {
			if (fat0 == ~0ULL) {
				fat0 = i;
				fpart_status_printf("LBA%llu: FAT BPB #1\n", i);
			} else if (fat1 == ~0ULL && i > fat0 + 8) {
				fat1 = i;
				fpart_status_printf("LBA%llu: FAT BPB #2\n", i);
				break;
			}
		}
		if (i == 7 && is_hi_firmware_hdr(sec))
			fpart_status_printf("LBA7: [hi] (classic 512-LBA 63)\n");
	}

	if (!fpart_by_kind(FPART_FIRMWARE)->present && hi_lba != ~0ULL) {
		u64 fw_end = (fat0 != ~0ULL && fat0 > hi_lba) ? fat0 :
			     (ftl_capacity / 32);

		if (fw_end > hi_lba)
			fpart_set(FPART_FIRMWARE, hi_lba, fw_end - hi_lba);
	}

	if (!fpart_by_kind(FPART_IPOD)->present && fat0 != ~0ULL) {
		u64 ns = (fat1 != ~0ULL) ? (fat1 - fat0) : (ftl_capacity - fat0);

		fpart_set(FPART_IPOD, fat0, ns);
	}

	if (!fpart_by_kind(FPART_IPOD)->present) {
		fpart_set(FPART_IPOD, 0, ftl_capacity);
		fpart_status_printf(
			"fallback: ipod = whole FTL (no separate FAT found)\n");
	}

	{
		struct fpart_slice *fw = fpart_by_kind(FPART_FIRMWARE);
		struct fpart_slice *ipod = fpart_by_kind(FPART_IPOD);

		if (fw->present && fat0 != ~0ULL &&
		    fat0 >= fw->start_lba &&
		    fat0 < fw->start_lba + fw->nsectors &&
		    fat0 != ipod->start_lba) {
			u64 rsrc_n = fw->start_lba + fw->nsectors - fat0;

			if (ipod->present && ipod->start_lba > fat0)
				rsrc_n = ipod->start_lba - fat0;
			fpart_set(FPART_RSRC, fat0, rsrc_n);
		}
	}

	fpart_status_printf(
		"scanned_lbas=%u summary: fw=%d@%llu+%llu ipod=%d@%llu+%llu rsrc=%d@%llu+%llu\n",
		scan_n,
		fpart_by_kind(FPART_FIRMWARE)->present,
		fpart_by_kind(FPART_FIRMWARE)->start_lba,
		fpart_by_kind(FPART_FIRMWARE)->nsectors,
		fpart_by_kind(FPART_IPOD)->present,
		fpart_by_kind(FPART_IPOD)->start_lba,
		fpart_by_kind(FPART_IPOD)->nsectors,
		fpart_by_kind(FPART_RSRC)->present,
		fpart_by_kind(FPART_RSRC)->start_lba,
		fpart_by_kind(FPART_RSRC)->nsectors);

	kfree(sec);

	ret = 0;
	ret |= fpart_register_slice(fpart_by_kind(FPART_FIRMWARE));
	ret |= fpart_register_slice(fpart_by_kind(FPART_IPOD));
	ret |= fpart_register_slice(fpart_by_kind(FPART_RSRC));
	return ret < 0 ? ret : 0;
}

static int fpart_scan(void)
{
	return fpart_scan_ex(64);
}

static ssize_t validate_sector_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	u64 sector;
	u8 *secbuf;
	unsigned int i, n;
	int ret;

	if (kstrtoull(buf, 0, &sector))
		return -EINVAL;
	if (sector >= ftl_capacity)
		return -ERANGE;

	secbuf = kmalloc(FMSS_FTL_SECTOR_SIZE, GFP_KERNEL);
	if (!secbuf)
		return -ENOMEM;

	ret = fmss_ftl_read_sector(sector, secbuf);
	if (ret) {
		kfree(secbuf);
		dev_warn(dev, "validate sector %llu failed: %d\n", sector, ret);
		return ret;
	}

	n = min_t(unsigned int, FTL_VALIDATE_HEX, FMSS_FTL_SECTOR_SIZE);
	{
		/* One line — bare printk("%02x") becomes a dmesg line each. */
		char hex[FTL_VALIDATE_HEX * 3 + 4];
		unsigned int pos = 0;

		for (i = 0; i < n && pos + 3 < sizeof(hex); i++)
			pos += scnprintf(hex + pos, sizeof(hex) - pos, "%02x",
					 secbuf[i]);
		dev_info(dev, "sector %llu ok head[%u]: %s\n", sector, n, hex);
	}
	kfree(secbuf);
	return count;
}
static DEVICE_ATTR_WO(validate_sector);

static ssize_t map_build_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int max_lpn = ftl_map_max_lpn;
	int ret;

	if (buf[0] && buf[0] != '\n' && kstrtouint(buf, 0, &max_lpn))
		return -EINVAL;
	if (!max_lpn)
		max_lpn = (unsigned int)(ftl_capacity / FMSS_FTL_SECTORS_PER_LPN) + 64;

	ret = fmss_ftl_build_map(max_lpn);
	if (ret)
		return ret;

	dev_info(dev, "LPN map built max=%u entries=%u\n",
		 max_lpn, fmss_ftl_lpn_count());
	return count;
}
static DEVICE_ATTR_WO(map_build);

static ssize_t fpart_scan_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int n = 64;
	int ret;

	if (buf[0] && buf[0] != '\n' && kstrtouint(buf, 0, &n))
		return -EINVAL;
	ret = fpart_scan_ex(n);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fpart_scan);

static ssize_t fpart_status_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	if (!fpart_status_len)
		return sysfs_emit(buf, "(no fpart_scan yet)\n");
	return sysfs_emit(buf, "%.*s", (int)fpart_status_len, fpart_status);
}
static DEVICE_ATTR_RO(fpart_status);

static ssize_t lpn_count_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return sysfs_emit(buf, "%u\n", fmss_ftl_lpn_count());
}
static DEVICE_ATTR_RO(lpn_count);

static ssize_t capacity_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sysfs_emit(buf, "%llu\n", ftl_capacity);
}
static DEVICE_ATTR_RO(capacity);

static struct attribute *ftl_attrs[] = {
	&dev_attr_validate_sector.attr,
	&dev_attr_map_build.attr,
	&dev_attr_fpart_scan.attr,
	&dev_attr_fpart_status.attr,
	&dev_attr_lpn_count.attr,
	&dev_attr_capacity.attr,
	NULL,
};

static const struct attribute_group ftl_attr_group = {
	.attrs = ftl_attrs,
};

static int ftl_register_disk(void)
{
	struct queue_limits lim = {
		.logical_block_size = FMSS_FTL_SECTOR_SIZE,
		.physical_block_size = FMSS_FTL_SECTOR_SIZE,
	};
	int ret;

	ftl_disk = blk_alloc_disk(&lim, NUMA_NO_NODE);
	if (IS_ERR(ftl_disk))
		return PTR_ERR(ftl_disk);

	ftl_disk->first_minor = 0;
	ftl_disk->flags = GENHD_FL_NO_PART;
	ftl_disk->fops = &ftl_bd_ops;
	snprintf(ftl_disk->disk_name, DISK_NAME_LEN, "%s", FTL_DISK_NAME);
	set_capacity(ftl_disk, ftl_capacity * (FMSS_FTL_SECTOR_SIZE / 512));

	ret = add_disk(ftl_disk);
	if (ret) {
		put_disk(ftl_disk);
		ftl_disk = NULL;
	}
	return ret;
}

static void ftl_unregister_disk(void)
{
	fpart_clear_slices();
	if (ftl_disk) {
		del_gendisk(ftl_disk);
		put_disk(ftl_disk);
		ftl_disk = NULL;
	}
}

static int __init ftl_init(void)
{
	unsigned int max_lpn;
	int ret;

	if (!fmss_ftl_present()) {
		pr_err("s5l8740-ftl: load fmss-s5l8740.ko first\n");
		return -ENODEV;
	}

	ret = ftl_register_disk();
	if (ret)
		return ret;

	ftl_pdev = platform_device_register_simple("s5l8740-ftl", -1, NULL, 0);
	if (IS_ERR(ftl_pdev)) {
		ret = PTR_ERR(ftl_pdev);
		ftl_unregister_disk();
		return ret;
	}

	ret = sysfs_create_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
	if (ret) {
		platform_device_unregister(ftl_pdev);
		ftl_unregister_disk();
		return ret;
	}

	if (ftl_auto_map) {
		max_lpn = ftl_map_max_lpn;
		if (!max_lpn)
			max_lpn = (unsigned int)(ftl_capacity /
						 FMSS_FTL_SECTORS_PER_LPN) + 64;
		ret = fmss_ftl_build_map(max_lpn);
		if (ret)
			dev_warn(&ftl_pdev->dev, "auto map_build failed (%d)\n",
				 ret);
	}

	if (fpart_auto_scan) {
		ret = fpart_scan();
		if (ret)
			dev_warn(&ftl_pdev->dev, "fpart_scan failed (%d)\n", ret);
	}

	dev_info(&ftl_pdev->dev,
		 "NAND-only Whimory FTL /dev/%s (%llu x %uB); named slices via fpart_scan\n",
		 FTL_DISK_NAME, ftl_capacity, FMSS_FTL_SECTOR_SIZE);
	return 0;
}

static void __exit ftl_exit(void)
{
	if (ftl_pdev) {
		sysfs_remove_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
		platform_device_unregister(ftl_pdev);
		ftl_pdev = NULL;
	}
	ftl_unregister_disk();
}

module_init(ftl_init);
module_exit(ftl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5L8740 Whimory FTL RO disks (ftl/firmware/ipod/rsrc)");
MODULE_AUTHOR("n31");
MODULE_SOFTDEP("pre: fmss_s5l8740");
