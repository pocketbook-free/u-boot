/*
 * fat.c
 *
 * R/O (V)FAT 12/16/32 filesystem implementation by Marcus Sundberg
 *
 * 2002-07-28 - rjones@nexus-tech.net - ported to ppcboot v1.1.6
 * 2003-03-10 - kharris@nexus-tech.net - ported to uboot
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 *			  \fat\fat.c
 *				Update Logs
 *		1. add the function file_fat_copy_sdcard and file_fat_copy_sdram for copy image from ex-card.
 *		2. add the accordingly code for update in function file_fat_read.
 *						James J.Weng	2010.4.14
 *     
 */ 
 
 
#include <common.h>
#include <config.h>
#include <fat.h>
#include <asm/byteorder.h>
#include <part.h>

#ifdef AUTO_UPDATE
//add by James,for flag the image ex-card exist.
#include <movi.h>
int flag_kernel = 0;  
int flag_rootfs = 0;
int flag_uboot = 0;
int flag_long_name = 0; //for long_name entry
///////////////////////////
#endif
#if defined(CONFIG_CMD_FAT)

/*
 * Convert a string to lowercase.
 */
static void
downcase(char *str)
{
	while (*str != '\0') {
		TOLOWER(*str);
		str++;
	}
}

static  block_dev_desc_t *cur_dev = NULL;
static unsigned long part_offset = 0;
static int cur_part = 1;

#define DOS_PART_TBL_OFFSET	0x1be
#define DOS_PART_MAGIC_OFFSET	0x1fe
#define DOS_FS_TYPE_OFFSET	0x36
#define DOS_FS32_TYPE_OFFSET	0x52
int disk_read (__u32 startblock, __u32 getsize, __u8 * bufptr)
{
	startblock += part_offset;
	if (cur_dev == NULL)
		return -1;
	if (cur_dev->block_read) {
		return cur_dev->block_read (cur_dev->dev
			, startblock, getsize, (unsigned long *)bufptr);
	}
	return -1;
}


int
fat_register_device(block_dev_desc_t *dev_desc, int part_no)
{
	unsigned char buffer[SECTOR_SIZE];
	disk_partition_t info;

	if (!dev_desc->block_read)
		return -1;
	cur_dev = dev_desc;
	/* check if we have a MBR (on floppies we have only a PBR) */
	if (dev_desc->block_read (dev_desc->dev, 0, 1, (ulong *) buffer) != 1) {
		printf ("** Can't read from device %d **\n", dev_desc->dev);
		return -1;
	}
	if (buffer[DOS_PART_MAGIC_OFFSET] != 0x55 ||
		buffer[DOS_PART_MAGIC_OFFSET + 1] != 0xaa) {
		/* no signature found */
		return -1;
	}
	if(!strncmp((char *)&buffer[DOS_FS_TYPE_OFFSET],"FAT",3)) {
		/* ok, we assume we are on a PBR only */
		cur_part = 1;
		part_offset=0;
	}
	else if(!strncmp((char *)&buffer[DOS_FS32_TYPE_OFFSET],"FAT32",5)) {
		/* ok, we assume we are on a PBR only */
		cur_part = 1;
		part_offset=0;
	}
#if (defined(CONFIG_CMD_IDE) || \
     defined(CONFIG_CMD_SCSI) || \
     defined(CONFIG_CMD_USB) || \
     defined(CONFIG_MMC) || \
     defined(CONFIG_SYSTEMACE) )
	/* First we assume, there is a MBR */
	else {
	if (!get_partition_info (dev_desc, part_no, &info)) {
		printf("1\n");
		part_offset = info.start;
		cur_part = part_no;
	} else {
		printf("3\n");
		printf ("** Partition %d not valid on device %d **\n",
				part_no, dev_desc->dev);
		return -1;
	}
	}

#else
	if (!strncmp((char *)&buffer[DOS_FS_TYPE_OFFSET],"FAT",3)) {
		printf("4\n");
		/* ok, we assume we are on a PBR only */
		cur_part = 1;
		part_offset = 0;
		info.start = part_offset;
	} else {
		/* FIXME we need to determine the start block of the
		 * partition where the DOS FS resides. This can be done
		 * by using the get_partition_info routine. For this
		 * purpose the libpart must be included.
		 */
		 printf("5\n");
		part_offset = 32;
		cur_part = 1;
	}
#endif
	return 0;
}


/*
 * Get the first occurence of a directory delimiter ('/' or '\') in a string.
 * Return index into string if found, -1 otherwise.
 */
static int
dirdelim(char *str)
{
	char *start = str;

	while (*str != '\0') {
		if (ISDIRDELIM(*str)) return str - start;
		str++;
	}
	return -1;
}


/*
 * Match volume_info fs_type strings.
 * Return 0 on match, -1 otherwise.
 */
static int
compare_sign(char *str1, char *str2)
{
	char *end = str1+SIGNLEN;

	while (str1 != end) {
		if (*str1 != *str2) {
			return -1;
		}
		str1++;
		str2++;
	}

	return 0;
}


/*
 * Extract zero terminated short name from a directory entry.
 */
static void get_name (dir_entry *dirent, char *s_name)
{
	char *ptr;

	memcpy (s_name, dirent->name, 8);
	s_name[8] = '\0';
	ptr = s_name;
	while (*ptr && *ptr != ' ')
		ptr++;
	if (dirent->ext[0] && dirent->ext[0] != ' ') {
		*ptr = '.';
		ptr++;
		memcpy (ptr, dirent->ext, 3);
		ptr[3] = '\0';
		while (*ptr && *ptr != ' ')
			ptr++;
	}
	*ptr = '\0';
	if (*s_name == DELETED_FLAG)
		*s_name = '\0';
	else if (*s_name == aRING)
		*s_name = 'å';
	downcase (s_name);
}

/*
 * Get the entry at index 'entry' in a FAT (12/16/32) table.
 * On failure 0x00 is returned.
 */
static __u32
get_fatent(fsdata *mydata, __u32 entry)
{
	__u32 bufnum;
	__u32 offset;
	__u32 ret = 0x00;

	switch (mydata->fatsize) {
	case 32:
		bufnum = entry / FAT32BUFSIZE;
		offset = entry - bufnum * FAT32BUFSIZE;
		break;
	case 16:
		bufnum = entry / FAT16BUFSIZE;
		offset = entry - bufnum * FAT16BUFSIZE;
		break;
	case 12:
		bufnum = entry / FAT12BUFSIZE;
		offset = entry - bufnum * FAT12BUFSIZE;
		break;

	default:
		/* Unsupported FAT size */
		return ret;
	}

	/* Read a new block of FAT entries into the cache. */
	if (bufnum != mydata->fatbufnum) {
		int getsize = FATBUFSIZE/FS_BLOCK_SIZE;
		__u8 *bufptr = mydata->fatbuf;
		__u32 fatlength = mydata->fatlength;
		__u32 startblock = bufnum * FATBUFBLOCKS;

		fatlength *= SECTOR_SIZE;	/* We want it in bytes now */
		startblock += mydata->fat_sect;	/* Offset from start of disk */

		if (getsize > fatlength) getsize = fatlength;
		if (disk_read(startblock, getsize, bufptr) < 0) {
			FAT_DPRINT("Error reading FAT blocks\n");
			return ret;
		}
		mydata->fatbufnum = bufnum;
	}

	/* Get the actual entry from the table */
	switch (mydata->fatsize) {
	case 32:
		ret = FAT2CPU32(((__u32*)mydata->fatbuf)[offset]);
		break;
	case 16:
		ret = FAT2CPU16(((__u16*)mydata->fatbuf)[offset]);
		break;
	case 12: {
		__u32 off16 = (offset*3)/4;
		__u16 val1, val2;

		switch (offset & 0x3) {
		case 0:
			ret = FAT2CPU16(((__u16*)mydata->fatbuf)[off16]);
			ret &= 0xfff;
			break;
		case 1:
			val1 = FAT2CPU16(((__u16*)mydata->fatbuf)[off16]);
			val1 &= 0xf000;
			val2 = FAT2CPU16(((__u16*)mydata->fatbuf)[off16+1]);
			val2 &= 0x00ff;
			ret = (val2 << 4) | (val1 >> 12);
			break;
		case 2:
			val1 = FAT2CPU16(((__u16*)mydata->fatbuf)[off16]);
			val1 &= 0xff00;
			val2 = FAT2CPU16(((__u16*)mydata->fatbuf)[off16+1]);
			val2 &= 0x000f;
			ret = (val2 << 8) | (val1 >> 8);
			break;
		case 3:
			ret = FAT2CPU16(((__u16*)mydata->fatbuf)[off16]);;
			ret = (ret & 0xfff0) >> 4;
			break;
		default:
			break;
		}
	}
	break;
	}
	FAT_DPRINT("ret: %d, offset: %d\n", ret, offset);

	return ret;
}


/*
 * Read at most 'size' bytes from the specified cluster into 'buffer'.
 * Return 0 on success, -1 otherwise.
 */
static int
get_cluster(fsdata *mydata, __u32 clustnum, __u8 *buffer, unsigned long size)
{
	int idx = 0;
	__u32 startsect;

	if (clustnum > 0) {
		startsect = mydata->data_begin + clustnum*mydata->clust_size;
	} else {
		startsect = mydata->rootdir_sect;
	}

	FAT_DPRINT("gc - clustnum: %d, startsect: %d\n", clustnum, startsect);
	if (disk_read(startsect, size/FS_BLOCK_SIZE , buffer) < 0) {
		FAT_DPRINT("Error reading data\n");
		return -1;
	}
	if(size % FS_BLOCK_SIZE) {
		__u8 tmpbuf[FS_BLOCK_SIZE];
		idx= size/FS_BLOCK_SIZE;
		if (disk_read(startsect + idx, 1, tmpbuf) < 0) {
			FAT_DPRINT("Error reading data\n");
			return -1;
		}
		buffer += idx*FS_BLOCK_SIZE;

		memcpy(buffer, tmpbuf, size % FS_BLOCK_SIZE);
		return 0;
	}

	return 0;
}


/*
 * Read at most 'maxsize' bytes from the file associated with 'dentptr'
 * into 'buffer'.
 * Return the number of bytes read or -1 on fatal errors.
 */
static long
get_contents(fsdata *mydata, dir_entry *dentptr, __u8 *buffer,
	     unsigned long maxsize)
{
	unsigned long filesize = FAT2CPU32(dentptr->size), gotsize = 0;
	unsigned int bytesperclust = mydata->clust_size * SECTOR_SIZE;
	__u32 curclust = START(dentptr);
	__u32 endclust, newclust;
	unsigned long actsize;

	FAT_DPRINT("Filesize: %ld bytes\n", filesize);

	if (maxsize > 0 && filesize > maxsize) filesize = maxsize;

	FAT_DPRINT("Reading: %ld bytes\n", filesize);

	actsize=bytesperclust;
	endclust=curclust;
	do {
		/* search for consecutive clusters */
		while(actsize < filesize) {
			newclust = get_fatent(mydata, endclust);
			if((newclust -1)!=endclust)
				goto getit;
			if (CHECK_CLUST(newclust, mydata->fatsize)) {
				FAT_DPRINT("curclust: 0x%x\n", newclust);
				FAT_DPRINT("Invalid FAT entry\n");
				return gotsize;
			}
			endclust=newclust;
			actsize+= bytesperclust;
		}
		/* actsize >= file size */
		actsize -= bytesperclust;
		/* get remaining clusters */
		if (get_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			FAT_ERROR("Error reading cluster\n");
			return -1;
		}
		/* get remaining bytes */
		gotsize += (int)actsize;
		filesize -= actsize;
		buffer += actsize;
		actsize= filesize;
		if (get_cluster(mydata, endclust, buffer, (int)actsize) != 0) {
			FAT_ERROR("Error reading cluster\n");
			return -1;
		}
		gotsize+=actsize;
		return gotsize;
getit:
		if (get_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			FAT_ERROR("Error reading cluster\n");
			return -1;
		}
		gotsize += (int)actsize;
		filesize -= actsize;
		buffer += actsize;
		curclust = get_fatent(mydata, endclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			FAT_DPRINT("curclust: 0x%x\n", curclust);
			FAT_ERROR("Invalid FAT entry\n");
			return gotsize;
		}
		actsize=bytesperclust;
		endclust=curclust;
	} while (1);
}


#ifdef CONFIG_SUPPORT_VFAT
/*
 * Extract the file name information from 'slotptr' into 'l_name',
 * starting at l_name[*idx].
 * Return 1 if terminator (zero byte) is found, 0 otherwise.
 */
static int
slot2str(dir_slot *slotptr, char *l_name, int *idx)
{
	int j;

	for (j = 0; j <= 8; j += 2) {
		l_name[*idx] = slotptr->name0_4[j];
		if (l_name[*idx] == 0x00) return 1;
		(*idx)++;
	}
	for (j = 0; j <= 10; j += 2) {
		l_name[*idx] = slotptr->name5_10[j];
		if (l_name[*idx] == 0x00) return 1;
		(*idx)++;
	}
	for (j = 0; j <= 2; j += 2) {
		l_name[*idx] = slotptr->name11_12[j];
		if (l_name[*idx] == 0x00) return 1;
		(*idx)++;
	}

	return 0;
}


/*
 * Extract the full long filename starting at 'retdent' (which is really
 * a slot) into 'l_name'. If successful also copy the real directory entry
 * into 'retdent'
 * Return 0 on success, -1 otherwise.
 */
__u8	 get_vfatname_block[MAX_CLUSTSIZE];
static int
get_vfatname(fsdata *mydata, int curclust, __u8 *cluster,
	     dir_entry *retdent, char *l_name)
{
	dir_entry *realdent;
	dir_slot  *slotptr = (dir_slot*) retdent;
	__u8	  *nextclust = cluster + mydata->clust_size * SECTOR_SIZE;
	__u8	   counter = (slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff;
	int idx = 0;

	while ((__u8*)slotptr < nextclust) {
		if (counter == 0) break;
		if (((slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff) != counter)
			return -1;
		slotptr++;
		counter--;
	}

	if ((__u8*)slotptr >= nextclust) {
		dir_slot *slotptr2;

		slotptr--;
		curclust = get_fatent(mydata, curclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			FAT_DPRINT("curclust: 0x%x\n", curclust);
			FAT_ERROR("Invalid FAT entry\n");
			return -1;
		}
		if (get_cluster(mydata, curclust, get_vfatname_block,
				mydata->clust_size * SECTOR_SIZE) != 0) {
			FAT_DPRINT("Error: reading directory block\n");
			return -1;
		}
		slotptr2 = (dir_slot*) get_vfatname_block;
		while (slotptr2->id > 0x01) {
			slotptr2++;
		}
		/* Save the real directory entry */
		realdent = (dir_entry*)slotptr2 + 1;
		while ((__u8*)slotptr2 >= get_vfatname_block) {
			slot2str(slotptr2, l_name, &idx);
			slotptr2--;
		}
	} else {
		/* Save the real directory entry */
		realdent = (dir_entry*)slotptr;
	}

	do {
		slotptr--;
		if (slot2str(slotptr, l_name, &idx)) break;
	} while (!(slotptr->id & LAST_LONG_ENTRY_MASK));

	l_name[idx] = '\0';
	if (*l_name == DELETED_FLAG) *l_name = '\0';
	else if (*l_name == aRING) *l_name = 'å';
	downcase(l_name);

	/* Return the real directory entry */
	memcpy(retdent, realdent, sizeof(dir_entry));

	return 0;
}


/* Calculate short name checksum */
static __u8
mkcksum(const char *str)
{
	int i;
	__u8 ret = 0;

	for (i = 0; i < 11; i++) {
		ret = (((ret&1)<<7)|((ret&0xfe)>>1)) + str[i];
	}

	return ret;
}
#endif


/*
 * Get the directory entry associated with 'filename' from the directory
 * starting at 'startsect'
 */
__u8 get_dentfromdir_block[MAX_CLUSTSIZE];
static dir_entry *get_dentfromdir (fsdata * mydata, int startsect,
				   char *filename, dir_entry * retdent,
				   int dols)
{
    __u16 prevcksum = 0xffff;
    __u32 curclust = START (retdent);
    int files = 0, dirs = 0;

    FAT_DPRINT ("get_dentfromdir: %s\n", filename);
    while (1) {
	dir_entry *dentptr;
	int i;

	if (get_cluster (mydata, curclust, get_dentfromdir_block,
		 mydata->clust_size * SECTOR_SIZE) != 0) {
	    FAT_DPRINT ("Error: reading directory block\n");
	    return NULL;
	}
	dentptr = (dir_entry *) get_dentfromdir_block;
	for (i = 0; i < DIRENTSPERCLUST; i++) {
	    char s_name[14], l_name[256];

	    l_name[0] = '\0';
	    if (dentptr->name[0] == DELETED_FLAG) {
		    dentptr++;
		    continue;
	    }
	    if ((dentptr->attr & ATTR_VOLUME)) {
#ifdef CONFIG_SUPPORT_VFAT
		if ((dentptr->attr & ATTR_VFAT) &&
		    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {
		    prevcksum = ((dir_slot *) dentptr)
			    ->alias_checksum;
		    get_vfatname (mydata, curclust, get_dentfromdir_block,
				  dentptr, l_name);
		    if (dols) {
			int isdir = (dentptr->attr & ATTR_DIR);
			char dirc;
			int doit = 0;

			if (isdir) {
			    dirs++;
			    dirc = '/';
			    doit = 1;
			} else {
			    dirc = ' ';
			    if (l_name[0] != 0) {
				files++;
				doit = 1;
			    }
			}
			if (doit) {
			    if (dirc == ' ') {
				printf (" %8ld   %s%c\n",
					(long) FAT2CPU32 (dentptr->size),
					l_name, dirc);
			    } else {
				printf ("            %s%c\n", l_name, dirc);
			    }
			}
			dentptr++;
			continue;
		    }
		    FAT_DPRINT ("vfatname: |%s|\n", l_name);
		} else
#endif
		{
		    /* Volume label or VFAT entry */
		    dentptr++;
		    continue;
		}
	    }
	    if (dentptr->name[0] == 0) {
		if (dols) {
		    printf ("\n%d file(s), %d dir(s)\n\n", files, dirs);
		}
		FAT_DPRINT ("Dentname == NULL - %d\n", i);
		return NULL;
	    }
#ifdef CONFIG_SUPPORT_VFAT
	    if (dols && mkcksum (dentptr->name) == prevcksum) {
		dentptr++;
		continue;
	    }
#endif
	    get_name (dentptr, s_name);
	    if (dols) {
		int isdir = (dentptr->attr & ATTR_DIR);
		char dirc;
		int doit = 0;

		if (isdir) {
		    dirs++;
		    dirc = '/';
		    doit = 1;
		} else {
		    dirc = ' ';
		    if (s_name[0] != 0) {
			files++;
			doit = 1;
		    }
		}
		if (doit) {
		    if (dirc == ' ') {
			printf (" %8ld   %s%c\n",
				(long) FAT2CPU32 (dentptr->size), s_name,
				dirc);
		    } else {
			printf ("            %s%c\n", s_name, dirc);
		    }
		}
		dentptr++;
		continue;
	    }
	    if (strcmp (filename, s_name) && strcmp (filename, l_name)) {
		FAT_DPRINT ("Mismatch: |%s|%s|\n", s_name, l_name);
		dentptr++;
		continue;
	    }
	    memcpy (retdent, dentptr, sizeof (dir_entry));

	    FAT_DPRINT ("DentName: %s", s_name);
	    FAT_DPRINT (", start: 0x%x", START (dentptr));
	    FAT_DPRINT (", size:  0x%x %s\n",
			FAT2CPU32 (dentptr->size),
			(dentptr->attr & ATTR_DIR) ? "(DIR)" : "");

	    return retdent;
	}
	curclust = get_fatent (mydata, curclust);
	if (CHECK_CLUST(curclust, mydata->fatsize)) {
	    FAT_DPRINT ("curclust: 0x%x\n", curclust);
	    FAT_ERROR ("Invalid FAT entry\n");
	    return NULL;
	}
    }

    return NULL;
}


/*
 * Read boot sector and volume info from a FAT filesystem
 */
static int
read_bootsectandvi(boot_sector *bs, volume_info *volinfo, int *fatsize)
{
	__u8 block[FS_BLOCK_SIZE];
	volume_info *vistart;

	if (disk_read(0, 1, block) < 0) {
		FAT_DPRINT("Error: reading block\n");
		return -1;
	}

	memcpy(bs, block, sizeof(boot_sector));
	bs->reserved	= FAT2CPU16(bs->reserved);
	bs->fat_length	= FAT2CPU16(bs->fat_length);
	bs->secs_track	= FAT2CPU16(bs->secs_track);
	bs->heads	= FAT2CPU16(bs->heads);
#if 0 /* UNUSED */
	bs->hidden	= FAT2CPU32(bs->hidden);
#endif
	bs->total_sect	= FAT2CPU32(bs->total_sect);

	/* FAT32 entries */
	if (bs->fat_length == 0) {
		/* Assume FAT32 */
		bs->fat32_length = FAT2CPU32(bs->fat32_length);
		bs->flags	 = FAT2CPU16(bs->flags);
		bs->root_cluster = FAT2CPU32(bs->root_cluster);
		bs->info_sector  = FAT2CPU16(bs->info_sector);
		bs->backup_boot  = FAT2CPU16(bs->backup_boot);
		vistart = (volume_info*) (block + sizeof(boot_sector));
		*fatsize = 32;
	} else {
		vistart = (volume_info*) &(bs->fat32_length);
		*fatsize = 0;
	}
	memcpy(volinfo, vistart, sizeof(volume_info));

	/* Terminate fs_type string. Writing past the end of vistart
	   is ok - it's just the buffer. */
	vistart->fs_type[8] = '\0';

	if (*fatsize == 32) {
		if (compare_sign(FAT32_SIGN, vistart->fs_type) == 0) {
			printf("the fat32 has been signed (*fatsize = 32)...\n");
			return 0;
		}
	} else {
		if (compare_sign(FAT12_SIGN, vistart->fs_type) == 0) {
			*fatsize = 12;
			return 0;
		}
		if (compare_sign(FAT16_SIGN, vistart->fs_type) == 0) {
			printf("the fat16 has been signed(*fatsize = 16) ...\n");
			*fatsize = 16;
			return 0;
		}
	}

	FAT_DPRINT("Error: broken fs_type sign\n");
	return -1;
}


__u8 do_fat_read_block[MAX_CLUSTSIZE];  /* Block buffer */
__u8 do_fat_read_block_src[MAX_CLUSTSIZE];  //add by James

long
do_fat_read (const char *filename, void *buffer, unsigned long maxsize,
	     int dols)
{
#if CONFIG_NIOS /* NIOS CPU cannot access big automatic arrays */
    static
#endif
    char fnamecopy[2048];
    boot_sector bs;
    volume_info volinfo;
    fsdata datablock;
    fsdata *mydata = &datablock;
    dir_entry *dentptr;
    __u16 prevcksum = 0xffff;
    char *subname = "";
    int rootdir_size, cursect;
    int idx, isdir = 0;
    int files = 0, dirs = 0;
    long ret = 0;
    int firsttime;
	/////////////////////////////////////
	int addr_src;
	int addr_offset = 0;
	
    if (read_bootsectandvi (&bs, &volinfo, &mydata->fatsize)) {
	FAT_DPRINT ("Error: reading boot sector\n");
	return -1;
    }
    if (mydata->fatsize == 32) {
	mydata->fatlength = bs.fat32_length;
    } else {
	mydata->fatlength = bs.fat_length;
    }
    mydata->fat_sect = bs.reserved;
    cursect = mydata->rootdir_sect
	    = mydata->fat_sect + mydata->fatlength * bs.fats;
	printf("the cursect is %d ...\n", cursect);
    mydata->clust_size = bs.cluster_size;
    if (mydata->fatsize == 32) {
	rootdir_size = mydata->clust_size;
	mydata->data_begin = mydata->rootdir_sect   /* + rootdir_size *//* Start sector of root directory */
		- (mydata->clust_size * 2);
    } else {
	rootdir_size = ((bs.dir_entries[1] * (int) 256 + bs.dir_entries[0])/* Number of root directory entries */
			* sizeof (dir_entry)) / SECTOR_SIZE;
	mydata->data_begin = mydata->rootdir_sect + rootdir_size
		- (mydata->clust_size * 2);
    }
    mydata->fatbufnum = -1;

    FAT_DPRINT ("FAT%d, fatlength: %d\n", mydata->fatsize,
		mydata->fatlength);
    FAT_DPRINT ("Rootdir begins at sector: %d, offset: %x, size: %d\n"
		"Data begins at: %d\n",
		mydata->rootdir_sect, mydata->rootdir_sect * SECTOR_SIZE,
		rootdir_size, mydata->data_begin);
    FAT_DPRINT ("Cluster size: %d\n", mydata->clust_size);

    /* "cwd" is always the root... */
    while (ISDIRDELIM (*filename))  //#define ISDIRDELIM(c)   ((c) == '/' || (c) == '\\')
	filename++;
    /* Make a copy of the filename and convert it to lowercase */
    strcpy (fnamecopy, filename);
    downcase (fnamecopy);
    if (*fnamecopy == '\0') {
	if (!dols)
	    return -1;
	dols = LS_ROOT;
    } else if ((idx = dirdelim (fnamecopy)) >= 0) {
	isdir = 1;
	fnamecopy[idx] = '\0';
	subname = fnamecopy + idx + 1;
	/* Handle multiple delimiters */
	while (ISDIRDELIM (*subname))
	    subname++;
    } else if (dols) {
	isdir = 1;
    }
	
    while (1) {
	int i;

	if (disk_read (cursect, mydata->clust_size, do_fat_read_block) < 0) {
	    FAT_DPRINT ("Error: reading rootdir block\n");
	    return -1;
	}
	
	// add by James/////////////////////////////
	if (disk_read (cursect, mydata->clust_size, do_fat_read_block_src) < 0) {
	    FAT_DPRINT ("Error: reading rootdir block\n");
	    return -1;
	}
	////////////////////////////////////////////
	dentptr = (dir_entry *) do_fat_read_block;
	addr_src = &dentptr;//add by James
	
	printf("the DIRENTSPERBLOCK is %d ...\n", DIRENTSPERBLOCK);
	for (i = 0; i < DIRENTSPERBLOCK; i++) {   //DIRENTSPERBLOCK = (FS_BLOCK_SIZE/sizeof(dir_entry))
	    char s_name[14], l_name[256];

	    l_name[0] = '\0';
	    if ((dentptr->attr & ATTR_VOLUME)) {//perhaps volume or long_name file item
#ifdef CONFIG_SUPPORT_VFAT
		if ((dentptr->attr & ATTR_VFAT) &&
		    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {//if it a long_name file
		    prevcksum = ((dir_slot *) dentptr)->alias_checksum;
		    get_vfatname (mydata, 0, do_fat_read_block, dentptr, l_name);//get fatname
			printf("try it ...\n");
		    if (dols == LS_ROOT) {
			int isdir = (dentptr->attr & ATTR_DIR);
			char dirc;
			int doit = 0;

			if (isdir) {
			    dirs++;
			    dirc = '/';
			    doit = 1;
			} else {
			    dirc = ' ';
			    if (l_name[0] != 0) {
				files++;
				doit = 1;
			    }
			}
			if (doit) {
			    if (dirc == ' ') {
				printf (" %8ld   %s%c\n",
					(long) FAT2CPU32 (dentptr->size),
					l_name, dirc);
				//add by James
#ifdef AUTO_UPDATE
				flag_long_name = 1;
 				file_fat_copy_sdram(l_name,	
									addr_src,	
									addr_offset,
									cursect,
									mydata->clust_size,
									do_fat_read_block_src,
									dentptr,
									mydata->fatsize,
									prevcksum,
									flag_long_name
									); 
				printf("The l_name is %s ...\n",l_name);
#endif
			    } else {
				printf ("            %s%c\n", l_name, dirc);
			    }
			}
			dentptr++;
			addr_offset += 32;
			continue;
		    }
		    FAT_DPRINT ("Rootvfatname: |%s|\n", l_name);
		} else
#endif
		{
		    /* Volume label or VFAT entry */
		    dentptr++;
			addr_offset += 32;
		    continue;
		}
	    } else if (dentptr->name[0] == 0) {
		FAT_DPRINT ("RootDentname == NULL - %d\n", i);
		if (dols == LS_ROOT) {
		    printf ("\n%d file(s), %d dir(s)\n\n", files, dirs);
#ifdef AUTO_UPDATE
			file_fat_copy_onenand();
#endif
		    return 0;
		}
		return -1;
	    }
#ifdef CONFIG_SUPPORT_VFAT
	    else if (dols == LS_ROOT
		     && mkcksum (dentptr->name) == prevcksum) {
		dentptr++;
		addr_offset += 32;
		continue;
	    }
#endif
	    get_name (dentptr, s_name);
		printf("the s_name is ...%s\n", s_name);
	    if (dols == LS_ROOT) {
		int isdir = (dentptr->attr & ATTR_DIR);
		char dirc;
		int doit = 0;

		if (isdir) {
		    dirc = '/';
		    if (s_name[0] != 0) {
			dirs++;
			doit = 1;
		    }
		} else {
		    dirc = ' ';
		    if (s_name[0] != 0) {
			files++;
			doit = 1;
		    }
		}
		if (doit) {
		    if (dirc == ' ') {
			printf (" %8ld   %s%c***\n",
				(long) FAT2CPU32 (dentptr->size), s_name,
				dirc);
			//add by James
#ifdef AUTO_UPDATE
			file_fat_copy_sdram(s_name,
								addr_src,
								addr_offset,
								cursect,
								mydata->clust_size,
								do_fat_read_block_src,
								dentptr,
								mydata->fatsize,
								NULL,
								0);
#endif
		    } else {
			printf ("            %s%c\n", s_name, dirc);
		    }
		}
		dentptr++;
		addr_offset += 32;
		continue;
	    }
	    if (strcmp (fnamecopy, s_name) && strcmp (fnamecopy, l_name)) {
		FAT_DPRINT ("RootMismatch: |%s|%s|\n", s_name, l_name);
		dentptr++;
		addr_offset += 32;
		continue;
	    }
	    if (isdir && !(dentptr->attr & ATTR_DIR))
		return -1;

	    FAT_DPRINT ("RootName: %s", s_name);
	    FAT_DPRINT (", start: 0x%x", START (dentptr));
	    FAT_DPRINT (", size:  0x%x %s\n",
			FAT2CPU32 (dentptr->size), isdir ? "(DIR)" : "");

	    goto rootdir_done;  /* We got a match */
	}
	cursect++; //while a sector is end and copy a new sector, we must update buf and set add_offset is 0
	addr_offset = 0;
    }
  rootdir_done:

    firsttime = 1;
    while (isdir) {
	int startsect = mydata->data_begin
		+ START (dentptr) * mydata->clust_size;
	dir_entry dent;
	char *nextname = NULL;

	dent = *dentptr;
	dentptr = &dent;

	idx = dirdelim (subname);
	if (idx >= 0) {
	    subname[idx] = '\0';
	    nextname = subname + idx + 1;
	    /* Handle multiple delimiters */
	    while (ISDIRDELIM (*nextname))
		nextname++;
	    if (dols && *nextname == '\0')
		firsttime = 0;
	} else {
	    if (dols && firsttime) {
		firsttime = 0;
	    } else {
		isdir = 0;
	    }
	}

	if (get_dentfromdir (mydata, startsect, subname, dentptr,
			     isdir ? 0 : dols) == NULL) {
	    if (dols && !isdir)
		return 0;
	    return -1;
	}

	if (idx >= 0) {
	    if (!(dentptr->attr & ATTR_DIR))
		return -1;
	    subname = nextname;
	}
    }
    ret = get_contents (mydata, dentptr, buffer, maxsize);
    FAT_DPRINT ("Size: %d, got: %ld\n", FAT2CPU32 (dentptr->size), ret);

    return ret;
}


int
file_fat_detectfs(void)
{
	boot_sector	bs;
	volume_info	volinfo;
	int		fatsize;
	char	vol_label[12];

	if(cur_dev==NULL) {
		printf("No current device\n");
		return 1;
	}
#if defined(CONFIG_CMD_IDE) || \
    defined(CONFIG_CMD_SCSI) || \
    defined(CONFIG_CMD_USB) || \
    defined(CONFIG_MMC)
	printf("Interface:  ");
	switch(cur_dev->if_type) {
		case IF_TYPE_IDE :	printf("IDE"); break;
		case IF_TYPE_SCSI :	printf("SCSI"); break;
		case IF_TYPE_ATAPI :	printf("ATAPI"); break;
		case IF_TYPE_USB :	printf("USB"); break;
		case IF_TYPE_DOC :	printf("DOC"); break;
		case IF_TYPE_MMC :	printf("MMC"); break;
		default :		printf("Unknown");
	}
	printf("\n  Device %d: ",cur_dev->dev);
	dev_print(cur_dev);
#endif
	if(read_bootsectandvi(&bs, &volinfo, &fatsize)) {
		printf("\nNo valid FAT fs found\n");
		return 1;
	}
	memcpy (vol_label, volinfo.volume_label, 11);
	vol_label[11] = '\0';
	volinfo.fs_type[5]='\0';
	printf("Partition %d: Filesystem: %s \"%s\"\n"
			,cur_part,volinfo.fs_type,vol_label);
	return 0;
}


int
file_fat_ls(const char *dir)
{
	return do_fat_read(dir, NULL, 0, LS_YES);
}


long
file_fat_read(const char *filename, void *buffer, unsigned long maxsize)
{
	printf("reading %s\n",filename);
	return do_fat_read(filename, buffer, maxsize, LS_NO);
}
#ifdef AUTO_UPDATE
int file_fat_copy_sdram(char *name,
						int addr_src,
						int offset,
						int cursect,
						int clust_size,
						__u8 do_fat_read_block_src[MAX_CLUSTSIZE],
						dir_entry *dentptr,
						int fatsize,
						int prevcksum,
						int flag_long_name
						)
{
	//////////////////////////////////////////////
	// int index = 0;
	 int tmp_addr;
	 int i = 0;
	 ulong dst;
	 int offset_tmp;
	 int get_fat_label_flag = 0; //when use the file_name,we must get the s_name struct dir_entry for start_cluster 
	// int tmp_addr1;
	// int tmp = 0;
	////////////////////////////////////////////////
	char *s;	

 	if(!strcmp("u-boot.bin",name))
	{
		printf("Found u-boot, begin  copy it to mmc! ...\n");
		//copy file to sdram
  		s = getenv("mmcucmd");
		run_command(s,0);
		
		offset_tmp = offset;
		//while((do_fat_read_block_src[offset_tmp] & ~LAST_LONG_ENTRY_MASK) != 0x01){
		if(flag_long_name) {
			while(do_fat_read_block_src[offset_tmp + 0xB] != 0x20){	//attr != 0x20
				printf("************************the dentptr->attr is 0x%02x ...\n", do_fat_read_block_src[offset_tmp]);
				do_fat_read_block_src[offset_tmp] = 0xE5;//we must set the first byte = 0xE5 fot flag it had been cleared.
				offset_tmp += 32;
				get_fat_label_flag++;
			}
		}
		do_fat_read_block_src[offset_tmp] = 0xE5;
		dentptr = dentptr + get_fat_label_flag;
		printf("************************the dentptr->start is 0x%04x ...\n", dentptr->start);
		 
		
		flag_uboot = 1;
		
		printf("The offset is : %d\n",offset);
		if(fatsize == 16)
		{
			delete_FAT_label(dentptr->start);		
		} 
		else if(fatsize == 32)
		{
			if(delete_FAT32_label(dentptr->start, dentptr->starthi) != 0)
			{
				printf("run failed ...\n");
				return -1;
			}
		}

		
		int tp = 0;
		int i = 0;
		//display ex-card dirc
 		while(tp <= 200)
		{
			if(15 != i || 0 == i)
			{
				printf(" 0x%02x	",do_fat_read_block_src[tp]);
				i++;
			} else {
				printf(" 0x%02x	\n",do_fat_read_block_src[tp]);
				i = 0;
			}
			tp++;
		} 
		printf("\nThe do_fat_read_block address is 0x%08x ...\n",do_fat_read_block_src);
		dst = virt_to_phys(do_fat_read_block_src);
		printf("The dest is 0x%08x ...\n",dst);
		printf("The cursect is %d , clust_size is %d ...\n",cursect,clust_size);
		movi_write((uint)dst,(uint)cursect,(uint)clust_size); 
	} 
	if(!strcmp("zimage",name))
	{
		printf("Found zImage, begin  copy it to mmc! ...\n");
		s = getenv("mmckcmd");
		run_command(s,0);
		flag_kernel = 1;
	}
		
	//printf("The l_name is %s ...\n",name);
	if(!strcmp("rootfs",name))
	{
		printf("Found rootfs, begin  copy it to mmc! ...\n");
		s = getenv("mmcfcmd");
		run_command(s,0);
		flag_rootfs = 1;
	}
	
	flag_long_name = 0;
	return 0;
}
	
int file_fat_copy_sdcard(void)
{
	char *s;
	//init sd0
	printf("Now init mmc 0 ...\n");
/*  s = getenv("initmmc0cmd");
	run_command(s,0);  */
	//add int for debug
/*  	int movi_c = 0;
	movi_set_capacity();
	movi_set_ofs(MOVI_TOTAL_BLKCNT);
	movi_init();  */

	if(flag_kernel)
	{
		s = getenv("wkcmd");
		run_command(s,0);
		printf("End of copy kernel to mmc 0 ...\n");
	}
			
	if(flag_rootfs)
	{
		s = getenv("wfcmd");
		run_command(s,0);
		printf("End of copy rootfs to mmc 0 ...\n");
	}
		
	if(flag_uboot)
	{
		s = getenv("wucmd");
		run_command(s,0);
		printf("End of copy u-boot to mmc 0 ...now reset...\n");
			
		s = getenv("reset");
		run_command(s,0);
	}
		
	return 0;
}
int file_fat_copy_onenand(void)
{
	char *s;
	if(flag_kernel)
	{
		s = getenv("wkcmd");
		run_command(s,0);
		printf("End of copy kernel to mmc 0 ...\n");
	}
			
	if(flag_rootfs)
	{
		s = getenv("wfcmd");
		run_command(s,0);
		printf("End of copy rootfs to mmc 0 ...\n");
	}
		
	if(flag_uboot)
	{
		s = getenv("onenand_clear_uboot_cmd");
		run_command(s,0);
		s = getenv("onenand_load_uboot_cmd");
		run_command(s,0);
		printf("End of copy u-boot to onenand ...now reset...\n");
			
		s = getenv("reset");
		run_command(s,0);
	}
}

int delete_FAT_label(int start_addr)
{
	int i;
	int flag = 0;
	ulong dst;
	__u32 second_addr;	//for fat label cluster
	
	char *fat_label = (char *)malloc(FAT_LABEL_LEN*512*sizeof(char));
	disk_read(FAT_LABEL1, FAT_LABEL_LEN, fat_label);
	printf("The fat_label[0] is  0x%02x ,address is 0x%08x ...\n", fat_label[0], fat_label);
	printf("The fat_label[1] is  0x%02x,address is 0x%08x ...\n", fat_label[1], (fat_label+1));
	printf("The fat_label[2] is 0x%02x ,address is 0x%08x ...\n", fat_label[2], (fat_label+2)); 

	printf("the dentptr->start is 0x%04x ...\n", SWAP16(start_addr));
	second_addr = start_addr + 1;	//fat label first cluster = start addr + 1
	printf("the second_addr is 0x%04x ...\n", second_addr);
	for(i = 4; i <= FAT_LABEL_LEN*512; i = i + 2) //delete fat label 1&2
	{
		if((fat_label[i+1] == ((0xFF00 & second_addr) >> 8)) && (fat_label[i] == (0x00FF & second_addr)))
		{//scan the fat_label find the cluster information and clear it
			flag ++;
			while((fat_label[i] != 0xFF) || (fat_label[i+1] != 0xFF) )
				{
					fat_label[i] = 0x00;
					i++;
				}
				fat_label[i] = 0x0;
				fat_label[i+1] = 0x0;
				fat_label[i+2] = 0x0;
		}
	}
	dst = virt_to_phys(fat_label);
	movi_write((uint)dst,(uint)FAT_LABEL1,(uint)FAT_LABEL_LEN);
	movi_write((uint)dst,(uint)FAT_LABEL2,(uint)FAT_LABEL_LEN);	
	printf("the flag is %d ...\n", flag);
	free(fat_label);
	return 0;
}

int delete_FAT32_label(int start_addr, int starthi_addr)
{
	int i;
	int flag = 0;
	ulong dst;
	__u32 second_addr;	//for fat label cluster
	

	char *fat_label = (char *)malloc(FAT32_LABEL_LEN*512*sizeof(char));
	if(fat_label == 0)
	{
		printf("memory malloc failure ...\n");
		return -1;
	}
	disk_read(FAT32_LABEL1, FAT32_LABEL_LEN, fat_label);
	printf("The fat_label[0] is  0x%02x ,address is 0x%08x ...\n", fat_label[0], fat_label);
	printf("The fat_label[1] is  0x%02x,address is 0x%08x ...\n", fat_label[1], (fat_label+1));
	printf("The fat_label[2] is 0x%02x ,address is 0x%08x ...\n", fat_label[2], (fat_label+2)); 

	printf("the dentptr->start is 0x%08x ...\n", SWAP16(start_addr));
	second_addr = starthi_addr << 16 | start_addr + 1;	//fat label first cluster = start addr + 1
	printf("the second_addr is 0x%08x ...\n", second_addr);
	for(i = 4; i <= FAT32_LABEL_LEN*512; i = i + 4) //delete fat label 1&2
	{
		if((fat_label[i] == (0x000000FF & second_addr))
									&& (fat_label[i+1] == ((0x0000FF00 & second_addr) >> 8))
									&& (fat_label[i+2] == ((0x00FF0000 & second_addr) >>16))
									&& (fat_label[i+3] == ((0xFF000000 & second_addr) >>24)))
		{//scan the fat_label find the cluster information and clear it
			flag ++;
			while((fat_label[i] != 0xFF) 
							|| (fat_label[i+1] != 0xFF) 
							|| (fat_label[i+2] != 0xFF) 
							|| (fat_label[i+3] != 0x0F))
				{
					fat_label[i] = 0x00;
					i++;
				}
				fat_label[i] = 0x0;
				fat_label[i+1] = 0x0;
				fat_label[i+2] = 0x0;
				fat_label[i+3] = 0x0;
		}
	}
	printf("the addr of fat_label is 0x%08x ...\n", fat_label);
	dst = virt_to_phys(fat_label);
	movi_write((uint)dst,(uint)FAT32_LABEL1,(uint)FAT32_LABEL_LEN);
	movi_write((uint)dst,(uint)FAT32_LABEL2,(uint)FAT32_LABEL_LEN);	
	printf("the flag is %d ...\n", flag);
	free(fat_label);
	return 0;
}
#endif //endif auto_update
#endif
