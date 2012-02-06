/*
 * U-Boot command for OneNAND device
 */

#include <common.h>

#if defined(CONFIG_S3C64XX) || defined(CONFIG_S5PC1XX)

/*
 *
 * New NAND support
 *
 */
#include <common.h>
#include <command.h>
#include <watchdog.h>
#include <malloc.h>
#include <asm/byteorder.h>

#ifdef CONFIG_SHOW_BOOT_PROGRESS
# include <status_led.h>
# define SHOW_BOOT_PROGRESS(arg)	show_boot_progress(arg)
#else
# define SHOW_BOOT_PROGRESS(arg)
#endif

#if defined(CONFIG_S3C6400) || defined(CONFIG_S3C6410) || defined(CONFIG_S3C6430) || defined(CONFIG_S5PC100)
//#include <s3c_onenand.h>
#include <linux/mtd/s3c_onenand.h>
#else
#include <onenand.h>
#endif

extern int onenand_curr_device;
extern struct mtd_info onenand_info[];       /* info for OneNAND chips */
extern void onenand_print_device_info(int device, int version);

static int onenand_dump_oob(struct mtd_info *mtd, ulong off)
{
	int i;
	u_int8_t *p;
	u_int8_t *buf;
	struct mtd_oob_ops ops = {
		.mode		= MTD_OOB_PLACE,
		//.mode		= MTD_OOB_AUTO,
		.datbuf		= NULL,
	};

	buf = malloc(mtd->oobsize);
	memset(buf, 0xff, mtd->oobsize);

	ops.oobbuf = buf;
	if (ops.mode == MTD_OOB_AUTO)
		ops.ooblen = mtd->oobavail;
	else
		ops.ooblen = mtd->oobsize;

	mtd->read_oob(mtd, off, &ops);

	puts("OOB:\n");

	p = ops.oobbuf;

	i = mtd->oobsize >> 3;
	
	while (i--) {
		printf( "\t%02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
		p += 8;
	}

	free(buf);
	
	return 0;
}

static int onenand_dump(struct mtd_info *mtd, ulong off)
{
	int i;
	u_int8_t *p, *buf;
	size_t len = mtd->writesize;

	buf = malloc(len);
	memset(buf, 0xff, len);

	mtd->read(mtd, off, len, &len, (u_char *)buf);

	i = mtd->writesize >> 4;
	p = buf;

	printf("Page %08x dump:\n", (unsigned int) off);

	while (i--) {
		printf( "\t%02x %02x %02x %02x %02x %02x %02x %02x"
			"  %02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
			p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		p += 16;
	}

	onenand_dump_oob(mtd, off);

	free(buf);

	return 0;
}

static int onenand_block_read(struct mtd_info *mtd,
			loff_t from, size_t len,	size_t *retlen, u_char *buf)
{
	struct onenand_chip *this = mtd->priv;
	int blocks = (int) len >> this->erase_shift;
	int blocksize = (1 << this->erase_shift);
	loff_t ofs = from;
	size_t _retlen = 0, _tmp;
	int ret;

	if (ofs & (mtd->erasesize - 1)) {
		printk(" ERROR: starting address %x is not a block start address\n",
				(unsigned int) ofs);
		return 1;
	}

	if (len & (mtd->erasesize - 1)) {
		printk(" ERROR: length is not block aligned(0x%x)\n", (unsigned int) len);
		return 1;
	}

	printk("Main area read (%d blocks):\n", blocks);

	while (blocks) {
		if ((ofs & (mtd->erasesize - 1)) == 0) {
			ret = mtd->block_isbad(mtd, ofs);
			if (ret) {
				printk("Bad blocks %d at 0x%x\n",
				       (u32)(ofs >> this->erase_shift), (u32)ofs);
				ofs += blocksize;
				continue;
			}
		}
		printf("the blocksize is %d ...\n", blocksize);
		ret = mtd->read(mtd, ofs, blocksize, &_tmp, (u_char *)buf);
		if (ret) {
			printk("Read failed 0x%x, %d\n", (u32)ofs, ret);
			ofs += blocksize;
			continue;
		}
		ofs += blocksize;
		buf += blocksize;
		blocks--;
		_retlen += _tmp;
	}

	*retlen = _retlen;

	return 0;
}

static int onenand_block_write(struct mtd_info *mtd,
			loff_t to, size_t len, size_t *retlen, const u_char * buf)
{
	struct onenand_chip *this = mtd->priv;
	int blocks = (int) len >> this->erase_shift;
	int blocksize = (1 << this->erase_shift);
	loff_t ofs = to;
	size_t _retlen = 0, _tmp;
	int ret;

	if (ofs & (mtd->erasesize - 1)) {
		printk(" ERROR: starting address %x is not a block start address\n",
				(unsigned int) ofs);
		return 1;
	}

	if (len & (mtd->erasesize - 1)) {
		printk(" ERROR: length is not block aligned(0x%x)\n", (unsigned int) len);
		return 1;
	}

	printk("Main area write (%d blocks):\n", blocks);

	while (blocks) {
		if ((ofs & (mtd->erasesize - 1)) == 0) {
			ret = mtd->block_isbad(mtd, ofs);
			if (ret) {
				printk("Bad blocks %d at 0x%x is skipped.\n",
				       (u32)(ofs >> this->erase_shift), (u32)ofs);
				ofs += blocksize;
				//udelay(1000);
				continue;
			}
		}

		ret = mtd->write(mtd, ofs, blocksize, &_tmp, buf);
		if (ret) {
			printk("Write failed 0x%x, %d", (u32)ofs, ret);
			ofs += blocksize;
			continue;
		}

		ofs += blocksize;
		buf += blocksize;
		blocks--;
		_retlen += _tmp;
	}

	*retlen = _retlen;

	return 0;
}

static int onenand_write_yaffs2(struct mtd_info *mtd,
			loff_t to, size_t len, size_t *retlen, const u_char * buf)
{
	struct onenand_chip *this = mtd->priv;
	loff_t ofs = to;
	struct mtd_oob_ops ops = {
		.retlen		= 0,
	};
	size_t _retlen = 0;
	size_t written = 0;
	int ret;

	if (ofs & (mtd->erasesize - 1)) {
		printk(" ERROR starting address %x is not a block start address\n",
				(unsigned int) ofs);
		return 1;
	}

	printk("Yaffs2 write:\n");

	ops.mode = MTD_OOB_AUTO;
	//ops.mode = MTD_OOB_PLACE;
	ops.len = mtd->writesize;
	//ops.ooblen = mtd->oobsize;
	ops.ooblen = mtd->oobavail;
	while (written < len) {
		if ((ofs & (mtd->erasesize - 1)) == 0) {
			ret = mtd->block_isbad(mtd, ofs);
			if (ret) {
				printk("Bad blocks %d at 0x%x is skipped.\n",
				       (int) ofs >> this->erase_shift, (u32)ofs);

				ofs += mtd->erasesize;
				continue;
			}
			printk("\rWriting data at 0x%08x -- %3d%% complete.", (u32)ofs, ((written*4)/(len/25))+1);
		}

		ops.datbuf = (u_int8_t*)buf;
		ops.oobbuf = (u_int8_t*)buf + mtd->writesize;

		ret = mtd->write_oob(mtd, ofs, &ops);
		if (ret) {
			printk("ERROR: Write failed 0x%x, %d", (u32)ofs, ret);
			return 1;
		}

		buf += mtd->writesize + mtd->oobsize;
		written += mtd->writesize + mtd->oobsize;
		_retlen += ops.retlen;
		ofs += mtd->writesize;
	}

	printk("\n");
	*retlen = _retlen;

	return 0;
}

static int onenand_erase(struct mtd_info *mtd, ulong off, ulong size)
{
    struct erase_info instr;

    instr.mtd = mtd;
    instr.addr = off;
    instr.len = size;
    instr.callback = 0;

    return mtd->erase(mtd, &instr);
}

/* ------------------------------------------------------------------------- */

static void
arg_off_size(int argc, char *argv[], ulong *off, size_t *size, size_t totsize)
{
	*off = 0;
	*size = 0;

#if defined(CONFIG_JFFS2_NAND) && defined(CFG_JFFS_CUSTOM_PART)
	if (argc >= 1 && strcmp(argv[0], "partition") == 0) {
		int part_num;
		struct part_info *part;
		const char *partstr;

		if (argc >= 2)
			partstr = argv[1];
		else
			partstr = getenv("partition");

		if (partstr)
			part_num = (int)simple_strtoul(partstr, NULL, 10);
		else
			part_num = 0;

		part = jffs2_part_info(part_num);
		if (part == NULL) {
			printf("\nInvalid partition %d\n", part_num);
			return;
		}
		*size = part->size;
		*off = (ulong)part->offset;
	} else
#endif
	{
		if (argc >= 1)
			*off = (ulong)simple_strtoul(argv[0], NULL, 16);
		else
			*off = 0;

		if (argc >= 2)
			*size = (ulong)simple_strtoul(argv[1], NULL, 16);
		else
			*size = totsize - *off;

	}

}

int do_onenand(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int i, dev, ret = 1;
	ulong addr, off;
	size_t size, retlen=0;
	char *cmd, *s;
	struct mtd_info *onenand;

	/* at least two arguments please */
	if (argc < 2)
		goto usage;

	cmd = argv[1];

	if (strcmp(cmd, "info") == 0) {

		putc('\n');
		for (i = 0; i < CFG_MAX_ONENAND_DEVICE; i++) {
			struct onenand_chip *this = (onenand_info[i].priv);
			printf("Device %d: (%04x) ", i, this->device_id);
			/* org: onenand_print_device_info(this->device_id); */
			onenand_print_device_info(this->device_id, 0);
		}
		return 0;
	}

	if (strcmp(cmd, "device") == 0) {
		if (argc < 3) {
			if ((onenand_curr_device < 0) ||
					(onenand_curr_device >= CFG_MAX_ONENAND_DEVICE))
				puts("\nno devices available\n");
			else {
				struct onenand_chip *this = (onenand_info[onenand_curr_device].priv);
				printf("Device %d: (%04x) ", onenand_curr_device, this->device_id);
				/* org: onenand_print_device_info(this->device_id); */
				onenand_print_device_info(this->device_id, 0);
			}
			return 0;
		}
		dev = (int)simple_strtoul(argv[2], NULL, 10);
		if (dev < 0 || dev >= CFG_MAX_ONENAND_DEVICE || !onenand_info[dev].name) {
			puts("No such device\n");
			return 1;
		}
		printf("Device %d: %s", dev, onenand_info[dev].name);
		puts("... is now current device\n");
		onenand_curr_device = dev;
		return 0;
	}

	if (strcmp(cmd, "bad") != 0 && strcmp(cmd, "scrub") != 0 && strcmp(cmd, "erase") != 0 &&
	    strncmp(cmd, "dump", 4) != 0 && strncmp(cmd, "writeoob", 8) != 0 &&
	    strncmp(cmd, "read", 4) != 0 && strncmp(cmd, "write", 5) != 0 &&
	    strncmp(cmd, "lock", 4) != 0)
		goto usage;

	/* the following commands operate on the current device */
	if (onenand_curr_device < 0 ||
			onenand_curr_device >= CFG_MAX_ONENAND_DEVICE ||
			!onenand_info[onenand_curr_device].name) {
		puts("\nno devices available\n");
		return 1;
	}
	onenand = &onenand_info[onenand_curr_device];

	if (strcmp(cmd, "bad") == 0) {
		printf("\nDevice %d bad blocks:\n", onenand_curr_device);
		for (off = 0; off < onenand->size; off += onenand->erasesize)
			if (onenand->block_isbad(onenand, off))
				printf("  %08x\n", (unsigned int) off);
		return 0;
	}

	if (strcmp(cmd, "scrub") == 0) {
		struct onenand_chip *this = (onenand->priv);
		unsigned int options_backup = this->options;

		printf("\nOneNAND scrub: device %d\n", onenand_curr_device);

		this->options &= ~ONENAND_CHECK_BAD;
		for (off = 0; off < onenand->size; off += onenand->erasesize) {
			if (onenand->block_isbad(onenand, off)) {
				printf("  %08x - ", (unsigned int) off);
				if (ret = onenand_erase(onenand, off, onenand->erasesize)) {
					printf("FAILED!!!\n");
				}
				printf("OK\n");
			}
		}
		this->options = options_backup;

		return ret == 0 ? 0 : 1;
	}

	if (strcmp(cmd, "erase") == 0) {
		arg_off_size(argc - 2, argv + 2, &off, &size, onenand->size);
		
		if (off == 0 && size == 0)
			return 1;		

		printf("\nOneNAND erase: device %d offset 0x%x, size 0x%x ",
		       onenand_curr_device, (unsigned int) off, size);

		ret = onenand_erase(onenand, off, size);

		printf("%s\n", ret ? "ERROR" : "OK");

		return ret == 0 ? 0 : 1;
	}

	if (strncmp(cmd, "dump", 4) == 0) {
		if (argc < 3)
			goto usage;

		s = strchr(cmd, '.');
		off = (int)simple_strtoul(argv[2], NULL, 16);

		if (s != NULL && strcmp(s, ".oob") == 0)
			ret = onenand_dump_oob(onenand, off);
		else
			ret = onenand_dump(onenand, off);

		return ret == 0 ? 1 : 0;

	}

	/* read write */
	if (strncmp(cmd, "read", 4) == 0 || strncmp(cmd, "write", 5) == 0) {
		u8 yaffs_write = 0;
		if (argc < 4)
			goto usage;
		s = strchr(cmd, '.');
		if (s != NULL) {
			if (strcmp(s, ".yaffs2") == 0)
				yaffs_write = 1;
		}
		addr = (ulong)simple_strtoul(argv[2], NULL, 16);

		arg_off_size(argc - 3, argv + 3, &off, &size, onenand->size);
		if (off == 0 && size == 0)
			return 1;

		i = strncmp(cmd, "read", 4) == 0;	/* 1 = read, 0 = write */
		printf("\nOneNAND %s: device %d offset %x, size %x  addr %x... \n",
		       i ? "read" : "write", onenand_curr_device, (unsigned int) off, size, (unsigned int) addr);

		if (i) {
			ret = onenand_block_read(onenand, off, size, &retlen, (u_char *)addr);
		}
		else {
			if(!yaffs_write) {
				ret = onenand_block_write(onenand, off, size, &retlen, (u_char *)addr);
			}else {
				ret = onenand_write_yaffs2(onenand, off, size, &retlen, (u_char *)addr);
							}
						}
		printf("%d bytes %s: %s\n", retlen,
		       i ? "read" : "written", ret ? "ERROR" : "OK");

		return ret == 0 ? 0 : 1;
	}

#if 0
	if (strncmp(cmd, "lock", 4) == 0) {
		int lock = 0;
		if (argc != 5)
			goto usage;

		if (strcmp(argv[2], "off") == 0)
			lock = 0;
		else if (strcmp(argv[2], "on") == 0)
			lock = 1;
		else
			goto usage;

		arg_off_size(argc - 3, argv + 3, &off, &size, onenand->size);
		if (off == 0 && size == 0)
			return 1;
		printf("\nOneNAND %slock: device %d offset 0x%x, size 0x%x ",
		       lock ? "":"un", onenand_curr_device, (unsigned int) off, size);

		if (lock) {
			ret = onenand_lock(onenand, off, size);
		}
		else {
			ret = onenand_unlock(onenand, off, size);
		}
		printf(": %s\n", ret ? "ERROR":"OK");
		return ret == 0 ? 0 : 1;
	}
#endif

usage:
	printf("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(onenand, 5, 0, do_onenand,
	"onenand - OneNAND sub-system\n",
	"info                  - show available OneNAND devices\n"
	"onenand device [dev]     - show or set current device\n"
	"onenand read     - addr off size\n"
	"onenand write[.yaffs2]    - addr off size - read/write `size' bytes starting\n"
	"    at offset `off' to/from memory address `addr'\n"
	"onenand erase [off size] - erase `size' bytes from\n"
	"    offset `off' (entire device if not specified)\n"
	"onenand bad - show bad blocks\n"
	"onenand dump[.oob] off - dump page\n"
	"onenand scrub - really clean NAND erasing bad blocks (UNSAFE)\n"
	"onenand markbad off - mark bad block at offset (UNSAFE)\n");

#if 0
int do_onenandboot(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	char *boot_device = NULL;
	char *ep;
	int dev;
	int r;
	unsigned int addr, cnt, offset = 0;
	image_header_t *hdr;
	struct mtd_info *onenand;

	switch (argc) {
	case 1:
		addr = CFG_LOAD_ADDR;
		boot_device = getenv("bootdevice");
		break;
	case 2:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = getenv("bootdevice");
		break;
	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = argv[2];
		break;
	case 4:
		addr = simple_strtoul(argv[1], NULL, 16);
		boot_device = argv[2];
		offset = simple_strtoul(argv[3], NULL, 16);
		break;
	default:
		printf("Usage:\n%s\n", cmdtp->usage);
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	if (!boot_device) {
		puts("\n** No boot device **\n");
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	dev = simple_strtoul(boot_device, &ep, 16);

	if (dev < 0 || dev >= CFG_MAX_ONENAND_DEVICE || !onenand_info[dev].name) {
		printf("\n** Device %d not available\n", dev);
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	onenand = &onenand_info[dev];
	printf("\nLoading from device %d: %s (offset 0x%lx)\n",
	       dev, onenand->name, (ulong) offset);

	cnt = onenand->writesize;
	r = onenand_read(onenand, offset, &cnt, (u_char *) addr);
	if (r) {
		printf("** Read error on %d\n", dev);
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	hdr = (image_header_t *) addr;

	if (ntohl(hdr->ih_magic) != IH_MAGIC) {
		printf("\n** Bad Magic Number 0x%x **\n", hdr->ih_magic);
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	print_image_hdr(hdr);

	cnt = (ntohl(hdr->ih_size) + sizeof (image_header_t));

	r = onenand_read(onenand, offset, &cnt, (u_char *) addr);
	if (r) {
		printf("** Read error on %d\n", dev);
		SHOW_BOOT_PROGRESS(-1);
		return 1;
	}

	/* Loading ok, update default load address */

	load_addr = addr;

	/* Check if we should attempt an auto-start */
	if (((ep = getenv("autostart")) != NULL) && (strcmp(ep, "yes") == 0)) {
		char *local_args[2];
		extern int do_bootm(cmd_tbl_t *, int, int, char *[]);

		local_args[0] = argv[0];
		local_args[1] = NULL;

		printf("Automatic boot of image at addr 0x%08lx ...\n", (ulong) addr);

		do_bootm(cmdtp, 0, 1, local_args);
		return 1;
	}
	return 0;
}

U_BOOT_CMD(oneboot, 4, 1, do_onenandboot,
	"oneboot - boot from NAND device\n", "loadAddr dev\n");
#endif

#endif	/* end of CONFIG_S5PC1XX */

