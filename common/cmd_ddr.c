#include <common.h>
#include <command.h>
#include <s3c6410.h>

#ifdef CONFIG_DDR_PAR
int do_ddr (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	printf("the DMC_DDR_REFRESH_PRD is  0x%08x ...\n", DMC_DDR_REFRESH_PRD);
	printf("the DMC_DDR_CAS_LATENCY is  0x%08x ...\n", DMC_DDR_CAS_LATENCY);
	printf("the DMC_DDR_t_DQSS is  0x%08x ...\n", DMC_DDR_t_DQSS);
	printf("the DMC_DDR_t_MRD is  0x%08x ...\n", DMC_DDR_t_MRD);
	printf("the DMC_DDR_t_RAS is  0x%08x ...\n", DMC_DDR_t_RAS);
	printf("the DMC_DDR_t_RC is  0x%08x ...\n", DMC_DDR_t_RC);
	printf("the DMC_DDR_t_RCD is  0x%08x ...\n", DMC_DDR_t_RCD);
	printf("the DMC_DDR_schedule_RCD is  0x%08x ...\n", DMC_DDR_schedule_RCD);
	printf("the DMC_DDR_t_RFC is  0x%08x ...\n", DMC_DDR_t_RFC);
	printf("the DMC_DDR_schedule_RFC is  0x%08x ...\n", DMC_DDR_schedule_RFC);
	printf("the DMC_DDR_schedule_RP is  0x%08x ...\n", DMC_DDR_schedule_RP);
	printf("the DMC_DDR_t_RP is  0x%08x ...\n", DMC_DDR_t_RP);
	printf("the DMC_DDR_t_RRD is  0x%08x ...\n", DMC_DDR_t_RRD);
	printf("the DMC_DDR_t_WR is  0x%08x ...\n", DMC_DDR_t_WR);
	printf("the DMC_DDR_t_WTR is  0x%08x ...\n", DMC_DDR_t_WTR);
	printf("the DMC_DDR_t_XP is  0x%08x ...\n", DMC_DDR_t_XP);
	printf("the DMC_DDR_t_XSR is  0x%08x ...\n", DMC_DDR_t_XSR);
	printf("the DMC_DDR_t_ESR is  0x%08x ...\n", DMC_DDR_t_ESR);
	printf("the DMC1_MEM_CFG is  0x%08x ...\n", DMC1_MEM_CFG);
	printf("the DMC1_MEM_CFG2 is  0x%08x ...\n", DMC1_MEM_CFG2);
	printf("the DMC1_CHIP0_CFG is  0x%08x ...\n", DMC1_CHIP0_CFG);
	printf("the DMC_mDDR_EMR0 is 0x%08x ...\n", DMC_mDDR_EMR0);
//	printf("the DMC1_CHIP1_CFG is 0x%08x ...\n", DMC1_CHIP1_CFG);
	printf("the DMC_mDDR_EMR1 is 0x%08x ...\n", DMC_mDDR_EMR1);
	return 0;
}

U_BOOT_CMD(
	ddr,	1,	0,	do_ddr,
	"ddr - display part of ddr info\n",
	NULL
);
#endif