#include <common.h>
#include <movi.h>
#include <asm/io.h>
#include <regs.h>

uint movi_hc = 0;

void movi_set_capacity(void)
{
#if defined(CONFIG_S3C6400)
	if (MOVI_HIGH_CAPACITY == 2)
#else
	if (MOVI_HIGH_CAPACITY & 0x1)
#endif
		movi_hc = 1;
}

int movi_set_ofs(uint last)
{
	int changed = 0;

	if (ofsinfo.last != last) {
		printf("The last is %d\n",last);
		ofsinfo.last 	= last - (eFUSE_SIZE / MOVI_BLKSIZE);
		printf("The ofsinfo.last is %d\n",ofsinfo.last);
		ofsinfo.bl1	= ofsinfo.last - MOVI_BL1_BLKCNT;
		printf("The ofsinfo.bl1 is %d\n",ofsinfo.bl1);
		ofsinfo.env	= ofsinfo.bl1 - MOVI_ENV_BLKCNT;
		printf("The ofsinfo.env is %d\n",ofsinfo.env);
		ofsinfo.bl2	= ofsinfo.bl1 - (MOVI_BL2_BLKCNT + MOVI_ENV_BLKCNT);
		printf("The ofsinfo.bl2 is %d\n",ofsinfo.bl2);
		ofsinfo.kernel	= ofsinfo.bl2 - MOVI_ZIMAGE_BLKCNT;
		printf("The ofsinfo.kernel is %d\n",ofsinfo.kernel);
		ofsinfo.rootfs	= ofsinfo.kernel - MOVI_ROOTFS_BLKCNT;
		printf("The ofsinfo.rootfs is %d\n",ofsinfo.rootfs);
		changed = 1;
	}

	return changed;
}

int movi_init(void)
{
	hsmmc_set_gpio();
	hsmmc_set_base();
	hsmmc_reset();

	if (hsmmc_init()) {
		printf("\nCard Initialization failed.\n");
		return -1;
	}

	return 1;
}

void movi_write_env(ulong addr)
{
	movi_write((uint)addr, ofsinfo.env, MOVI_ENV_BLKCNT);
}

void movi_read_env(ulong addr)
{
	movi_read((uint)addr, ofsinfo.env, MOVI_ENV_BLKCNT);
}

void movi_bl2_copy(void)
{
#if defined(CONFIG_S3C6400)
	CopyMovitoMem(MOVI_BL2_POS, MOVI_BL2_BLKCNT, (uint *)BL2_BASE, CONFIG_SYS_CLK_FREQ, MOVI_INIT_REQUIRED);
#else
	int movi_ch;

	if (INF_REG3_REG == 0)
		movi_ch = 0;
	else
		movi_ch = 1;

	writel(readl(ELFIN_HSMMC_BASE + (movi_ch * 0x100000) + HM_CONTROL4) | (0x3 << 16), ELFIN_HSMMC_BASE + (movi_ch * 0x100000) + HM_CONTROL4);
	CopyMovitoMem(movi_ch, MOVI_BL2_POS, MOVI_BL2_BLKCNT, (uint *)BL2_BASE, MOVI_INIT_REQUIRED);
#endif
}

