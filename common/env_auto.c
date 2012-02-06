/*
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.

 * (C) Copyright 2000-2006
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2001 Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Andreas Heppel <aheppel@sysgo.de>

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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
 
/*
 *			  \common\env_auto.c
 *				Update Logs
 *		remove about the nand's parameters and function
 *						James J.Weng	2010.5.8
 *     
 */

/* #define DEBUG */

#include <common.h>

#if defined(CFG_ENV_IS_IN_AUTO) /* Environment is in Nand Flash */

#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <s3c_onenand.h>
#include <movi.h>
#include <regs.h>

#if defined(CONFIG_CMD_ENV) || \
	defined(CONFIG_CMD_NAND) || \
	defined(CONFIG_CMD_MOVINAND) || \
	defined(CONFIG_CMD_ONENAND)
#define CMD_SAVEENV
#endif
#include <linux/mtd/mtd.h>
#define ONENAND_ENV_SIZE 0x20000

extern int onenand_curr_device;
extern struct mtd_info onenand_info[];       /* info for OneNAND chips */
extern void onenand_print_device_info(int device, int version);


/* info for NAND chips, defined in drivers/nand/nand.c */
//extern nand_info_t nand_info[];

/* references to names in env_common.c */
extern uchar default_environment[];
extern int default_environment_size;

char * env_name_spec = "SMDK bootable device";

#ifdef ENV_IS_EMBEDDED
extern uchar environment[];
env_t *env_ptr = (env_t *)(&environment[0]);
#else /* ! ENV_IS_EMBEDDED */
env_t *env_ptr = 0;
#endif /* ENV_IS_EMBEDDED */


/* local functions */
#if !defined(ENV_IS_EMBEDDED)
static void use_default(void);
#endif

DECLARE_GLOBAL_DATA_PTR;

uchar env_get_char_spec (int index)
{
	return ( *((uchar *)(gd->env_addr + index)) );
}


/* this is called before nand_init()
 * so we can't read Nand to validate env data.
 * Mark it OK for now. env_relocate() in env_common.c
 * will call our relocate function which will does
 * the real validation.
 *
 * When using a NAND boot image (like sequoia_nand), the environment
 * can be embedded or attached to the U-Boot image in NAND flash. This way
 * the SPL loads not only the U-Boot image from NAND but also the
 * environment.
 */
int env_init(void)
{
#if defined(ENV_IS_EMBEDDED)
	ulong total;
	int crc1_ok = 0, crc2_ok = 0;
	env_t *tmp_env1, *tmp_env2;

	total = CFG_ENV_SIZE;

	tmp_env1 = env_ptr;
	tmp_env2 = (env_t *)((ulong)env_ptr + CFG_ENV_SIZE);

	crc1_ok = (crc32(0, tmp_env1->data, ENV_SIZE) == tmp_env1->crc);
	crc2_ok = (crc32(0, tmp_env2->data, ENV_SIZE) == tmp_env2->crc);

	if (!crc1_ok && !crc2_ok)
		gd->env_valid = 0;
	else if(crc1_ok && !crc2_ok)
		gd->env_valid = 1;
	else if(!crc1_ok && crc2_ok)
		gd->env_valid = 2;
	else {
		/* both ok - check serial */
		if(tmp_env1->flags == 255 && tmp_env2->flags == 0)
			gd->env_valid = 2;
		else if(tmp_env2->flags == 255 && tmp_env1->flags == 0)
			gd->env_valid = 1;
		else if(tmp_env1->flags > tmp_env2->flags)
			gd->env_valid = 1;
		else if(tmp_env2->flags > tmp_env1->flags)
			gd->env_valid = 2;
		else /* flags are equal - almost impossible */
			gd->env_valid = 1;
	}

	if (gd->env_valid == 1)
		env_ptr = tmp_env1;
	else if (gd->env_valid == 2)
		env_ptr = tmp_env2;
#else /* ENV_IS_EMBEDDED */
	gd->env_addr  = (ulong)&default_environment[0];
	gd->env_valid = 1;
#endif /* ENV_IS_EMBEDDED */

	return (0);
}

#ifdef CMD_SAVEENV

int saveenv_movinand(void)
{
        movi_write_env(virt_to_phys((ulong)env_ptr));
        puts("done\n");

        return 1;
}

int saveenv_onenand(void)
{
        size_t total;
        int ret = 0, i;
        u32 erasebase;
        u32 eraselength;
        u32 eraseblock;
        u32 erasesize = onenand_info[0].erasesize;
        uint8_t *data;
		struct mtd_info *onenand;

        puts("Erasing Onenand...\n");
		
			/* the following commands operate on the current device */
		if (onenand_curr_device < 0 ||
				onenand_curr_device >= CFG_MAX_ONENAND_DEVICE ||
				!onenand_info[onenand_curr_device].name) {
			puts("\nno devices available\n");
			return 1;
		}
		onenand = &onenand_info[onenand_curr_device];
        /* If the value of CFG_ENV_OFFSET is not a NAND block boundary, the
         * NAND erase operation will fail. So first check if the CFG_ENV_OFFSET
         * is equal to a NAND block boundary
         */
        if ((CFG_ENV_OFFSET % (erasesize - 1)) != 0 ) {
                /* CFG_ENV_OFFSET is not equal to block boundary address. So, read
                 * the read the NAND block (in which ENV has to be stored), and
                 * copy the ENV data into the copied block data.
                 */

                /* Step 1: Find out the starting address of the NAND block to
                 * be erased. Also allocate memory whose size is equal to tbe
                 * NAND block size (NAND erasesize).
                 */
                eraseblock = CFG_ENV_OFFSET / erasesize;
                erasebase = eraseblock * erasesize;
				printf("the baseaddr is 0x%08x ...\n", erasebase);
                data = (uint8_t*)malloc(erasesize);
                if (data == NULL) {
                        printf("Could not save enviroment variables\n");
                        return 1;
                }

                /* Step 2: Read the NAND block into which the ENV data has
                 * to be copied
                 */
                total = erasesize;
				printf("the total is 0x%08x ...\n", total);
		for (i = 0; i < CFG_MAX_NAND_DEVICE; i++) {
			if (onenand_scan(&onenand_info[i], 1) == 0) {
				ret = onenand_read(onenand, erasebase, &total, data);
			} else {
				printf("no devices available\n");
				return 1;
			}
		}
                if (ret || total != erasesize) {
                        printf("Could not save enviroment variables %d\n",ret);
                        return 1;
                }
				
                /* Step 3: Copy the ENV data into the local copy of the block
                 * contents.
                 */
                memcpy((data + (CFG_ENV_OFFSET - erasebase)), (void*)env_ptr, CFG_ENV_SIZE);
        } else {
                /* CFG_ENV_OFFSET is equal to a NAND block boundary. So
                 * no special care is required when erasing and writing NAND
                 * block
                 */
                data = env_ptr;
                erasebase = CFG_ENV_OFFSET;
                erasesize = CFG_ENV_SIZE;
        }

        /* Erase the NAND block which will hold the ENV data */
        if (onenand_erase(onenand, erasebase, erasesize))
                return 1;

        puts("Writing to onenand... \n");
        total = erasesize;

        /* Write the ENV data to the NAND block */
		printf("the baseaddr is 0x%08x ...\n", erasebase);
		printf("the baseaddr is 0x%08x ...\n", total);
        ret = onenand_write(onenand, erasebase, &total, (u_char*)data);
        if (ret || total != erasesize) {
                printf("Could not save enviroment variables\n");
                return 1;
        }

         if ((CFG_ENV_OFFSET % (erasesize - 1)) != 0 )
                free(data);

        puts("Saved enviroment variables\n");
        return ret;
        return 1;
}

int saveenv_nor(void)
{
        printf("Now, NOR does not support the saveenv command!! \n But will be implement.\n");
        return 1;
}
int saveenv(void)
{
#if !defined(CONFIG_SMDK6440)
#if defined(CONFIG_S5PC100) || defined(CONFIG_S5PC110) || defined(CONFIG_S5P6442)

        if (INF_REG3_REG == 2);
        else if (INF_REG3_REG == 3)
                saveenv_movinand();
        else if (INF_REG3_REG == 1)
                saveenv_onenand();
        else if (INF_REG3_REG == 4)
                saveenv_nor();
        else
                printf("Unknown boot device\n");
#else   // others
        if (INF_REG3_REG == 2 || INF_REG3_REG == 3);
        else if (INF_REG3_REG == 4 || INF_REG3_REG == 5 || INF_REG3_REG == 6);
        else if (INF_REG3_REG == 0 || INF_REG3_REG == 7)
                saveenv_movinand();
        else if (INF_REG3_REG == 1)
                saveenv_onenand();
        else
                printf("Unknown boot device\n");

#endif
#else	// CONFIG_SMDK6440
        if (INF_REG3_REG == 3);
        else if (INF_REG3_REG == 4 || INF_REG3_REG == 5 || INF_REG3_REG == 6);
        else if (INF_REG3_REG == 0 || INF_REG3_REG == 1 || INF_REG3_REG == 7)
                saveenv_movinand();
        else
                printf("Unknown boot device\n");
#endif

        return 0;
}
#endif /* CMD_SAVEENV */


void env_relocate_spec_movinand(void)
{
#if !defined(ENV_IS_EMBEDDED)
	uint *magic = (uint*)(PHYS_SDRAM_1);

	if ((0x24564236 != magic[0]) || (0x20764316 != magic[1])) {
		movi_read_env(virt_to_phys((ulong)env_ptr));
	}
	
	if (crc32(0, env_ptr->data, ENV_SIZE) != env_ptr->crc)
		return use_default();
#endif /* ! ENV_IS_EMBEDDED */
}

void env_relocate_spec_onenand(void)
{
	struct mtd_info *onenand;

	/* the following commands operate on the current device */
	if (onenand_curr_device < 0 ||
		onenand_curr_device >= CFG_MAX_ONENAND_DEVICE ||
		!onenand_info[onenand_curr_device].name) {
			puts("\nno devices available\n");
			return 1;
	}
		onenand = &onenand_info[onenand_curr_device];
#if !defined(ENV_IS_EMBEDDED)
	size_t total;
	int ret, i;
	u_char *data;

	data = (u_char*)malloc(CFG_ENV_SIZE);
	total = CFG_ENV_SIZE;
	for (i = 0; i < CFG_MAX_NAND_DEVICE; i++) {
		if (onenand_scan(&onenand_info[i], 1) == 0) {
			ret = onenand_read(onenand, CFG_ENV_OFFSET, &total, (u_char*)data);
			env_ptr = data;
			if (ret || total != CFG_ENV_SIZE)
				return use_default();
			if (crc32(0, env_ptr->data, ENV_SIZE) != env_ptr->crc)
				return use_default();
		} else {
			printf("no devices available\n");
			return use_default();
		}
	}
/*	
*/
#endif /* ! ENV_IS_EMBEDDED */
}

void env_relocate_spec_nor(void)
{
	use_default();
}
void env_relocate_spec(void)
{
#if defined(CONFIG_SMDKC100) | defined(CONFIG_SMDKC110) | defined(CONFIG_S5P6442)
	if (INF_REG3_REG == 1)
		env_relocate_spec_onenand();
	else if (INF_REG3_REG == 2);
	else if (INF_REG3_REG == 3)
		env_relocate_spec_movinand();
	else if (INF_REG3_REG == 4)
		env_relocate_spec_nor();
	else
		use_default();
#elif !defined(CONFIG_SMDK6440)
	if (INF_REG3_REG >= 2 && INF_REG3_REG <= 6);
	else if (INF_REG3_REG == 0 || INF_REG3_REG == 7)
		env_relocate_spec_movinand();
	else if (INF_REG3_REG == 1)
		env_relocate_spec_onenand();
	else
		printf("Unknown boot device\n");
#else
	if (INF_REG3_REG >= 3 && INF_REG3_REG <= 6);
	else if (INF_REG3_REG == 0 || INF_REG3_REG == 1 || INF_REG3_REG == 7)
		env_relocate_spec_movinand();
#endif
}

#if !defined(ENV_IS_EMBEDDED)
static void use_default()
{
	puts("*** Warning - using default environment\n\n");

	if (default_environment_size > CFG_ENV_SIZE) {
		puts("*** Error - default environment is too large\n\n");
		return;
	}

	memset (env_ptr, 0, sizeof(env_t));
	memcpy (env_ptr->data,
			default_environment,
			default_environment_size);
	env_ptr->crc = crc32(0, env_ptr->data, ENV_SIZE);
	gd->env_valid = 1;

}
#endif

#endif /* CFG_ENV_IS_IN_NAND */
