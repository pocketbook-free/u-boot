/*
 *	This file is for the eink display in u-boot.
 *
 *
 *      Copyright (C) 2010 James J.Weng
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.
 */

#include <common.h>
#include <asm-arm/io.h>
#include <regs.h>
#include "second2_logo.h"
//#include "yuliang.h"

#define EINK_DEBUG 1

unsigned long gpcon;
unsigned long gpdat;
unsigned long sromdata;

static int wfm_fvsn = 0;
static int wfm_luts = 0;
static int wfm_mc   = 0;
static int wfm_trc  = 0;
static int wfm_eb   = 0;
static int wfm_sb   = 0;
static int wfm_wmta = 0;

#define bsaddr ((volatile unsigned long *)(0x10000000))
#define imgaddr ((volatile unsigned long *)(0x55000000))

#define BS97_INIT_HSIZE         1200
#define BS97_INIT_VSIZE         828
#define BS97_INIT_FSLEN         0
#define BS97_INIT_FBLEN         4
#define BS97_INIT_FELEN         4
#define BS97_INIT_LSLEN         4
#define BS97_INIT_LBLEN         10
#define BS97_INIT_LELEN         74
#define BS97_INIT_PIXCLKDIV     3
#define BS97_INIT_SDRV_CFG      (100 | (1 << 8) | (1 << 9))
#define BS97_INIT_GDRV_CFG      0x2
#define BS97_INIT_LUTIDXFMT     (4 | (1 << 7))


//reset pin gpp9
#define HIGH_RESET_PIN	gpdat = __raw_readl(GPPDAT);gpdat =(gpdat & ~(1<<9));__raw_writel(gpdat|(0x1<<9),GPPDAT);
#define LOW_RESET_PIN	gpdat = __raw_readl(GPPDAT);gpdat =(gpdat & ~(1<<9));__raw_writel(gpdat,GPPDAT);


//HDC pin gpo2
#define HIGH_HDC_PIN    gpdat = __raw_readl(GPODAT);gpdat =(gpdat & ~(1<<2));__raw_writel(gpdat|(0x1<<2),GPODAT);
#define LOW_HDC_PIN    gpdat = __raw_readl(GPODAT);gpdat =(gpdat & ~(1<<2));__raw_writel(gpdat,GPODAT);

#define EINKFB_SUCCESS          0
#define mdelay(x) udelay(x * 1000)
//#define mdelay(x) udelay(x * 2500)

#define EPD_CMD_DELAY 10
//#define EPD_CMD_DELAY 60


void epd_send_command(unsigned short command, unsigned short data)
{
    unsigned short value = 0;	
    udelay(EPD_CMD_DELAY);
    
    while(!value)
	{
        value =(__raw_readl(GPPDAT))&(0x1<<2);
	}
	
	LOW_HDC_PIN;
	
	__raw_writew(command,bsaddr);
	
    //udelay(1);
    udelay(3);
	HIGH_HDC_PIN;
	 	
	__raw_writew(data,bsaddr);
	
}

void epd_write_command(unsigned short command)
{

    unsigned short value = 0;	
    udelay(EPD_CMD_DELAY);
    
    while(!value)
		{	
			value =(__raw_readl(GPPDAT))&(0x1<<2);
		}
			
		LOW_HDC_PIN;
		udelay(10);
		__raw_writew(command,bsaddr);
    
		//udelay(1);
		udelay(3);
		HIGH_HDC_PIN;

}	

void epd_write_data(unsigned short data)
{
	__raw_writew(data,bsaddr);
	udelay(EPD_CMD_DELAY);
}

void write_epson_reg(unsigned short command, unsigned short addr, unsigned short data)
{
	unsigned short value = 0;    
    while(!value)
	{
        value = (__raw_readl(GPPDAT))&(0x1<<2);
	}
	
    epd_write_command(command); 
        udelay(5); 
	__raw_writew(addr,bsaddr);
        udelay(5); 
	__raw_writew(data,bsaddr);
        udelay(5); 
}

unsigned short epd_read_epson_reg(unsigned short command, unsigned short addr)
{
	
	volatile unsigned short reg;
	unsigned short value = 0;	
		
	udelay(EPD_CMD_DELAY);
	
	//while(!GET_HRDY_STATUS)	
	while(!value)
	{
		value =(__raw_readl(GPPDAT))&(0x1<<2);
	}
		
	epd_write_command(command);
  udelay(EPD_CMD_DELAY);
	__raw_writew(addr,bsaddr);
  udelay(EPD_CMD_DELAY);
	reg = __raw_readw(bsaddr);
  udelay(EPD_CMD_DELAY);
	
	return reg;
}

void bs_cmd_get_wfm_info(void)
{
	if(EINK_DEBUG)
	{
		u16   a = epd_read_epson_reg(0x10, 0x354),
			  b = epd_read_epson_reg(0x10, 0x356),
			  c = epd_read_epson_reg(0x10, 0x358),
              d = epd_read_epson_reg(0x10, 0x35C),
              e = epd_read_epson_reg(0x10, 0x35E);

		wfm_fvsn  = a & 0xFF;
		wfm_luts  = (a >> 8) & 0xFF;
		wfm_trc   = (b >> 8) & 0xFF;
		wfm_mc    = b & 0xFF;
		wfm_sb    = (c >> 8) & 0xFF;
		wfm_eb    = c & 0xFF;
		wfm_wmta  = d | (e << 16);
		printf("wfm: fvsn=%d luts=%d mc=%d trc=%d eb=0x%02x sb=0x%02x wmta=%d\n",
				wfm_fvsn, wfm_luts, wfm_mc, wfm_trc, wfm_eb, wfm_sb, wfm_wmta);
	}
}


void init_sys_run(void)
{
	printf("init_sys_run() ...\n");
	epd_write_command(0x06);
	printf("1\n");
	mdelay(100);
 	epd_write_command(0x30);	//read wfm_info
	printf("2\n");
/* 	*bsaddr = 0x0886;	
	*bsaddr = 0x0; */
	__raw_writew(0x0886, bsaddr);
	__raw_writew(0x0, bsaddr);
	printf("3\n");
	epd_write_command(0x37);
	printf("4\n");
	epd_write_command(0x28);
	printf("5\n"); 
}

void init_dspe_cfg(void)
{
	printf("init_dspe_cfg() ...\n");
	epd_write_command(0x09);
	printf("1...\n");
	epd_write_data(BS97_INIT_HSIZE);
	printf("2...\n");
	epd_write_data(BS97_INIT_VSIZE);
	printf("3...\n");
	epd_write_data(BS97_INIT_SDRV_CFG);
	printf("4...\n");
	epd_write_data(BS97_INIT_GDRV_CFG);
	printf("5...\n");
	epd_write_data(BS97_INIT_LUTIDXFMT);
	printf("6...\n");
	
}

void init_dspe_tmg(void)
{
	int iba = BS97_INIT_HSIZE * BS97_INIT_VSIZE * 2;
	printf("init_dspe_tmg() ...\n");
	//ipu_adc_write_cmd(CMD,0x0a);
	epd_write_command(0x0a);
	printf("1...\n");
	epd_write_data(BS97_INIT_FSLEN);
	printf("2...\n");
	epd_write_data((BS97_INIT_FELEN << 8) | BS97_INIT_FBLEN);
	printf("3...\n");
	epd_write_data(BS97_INIT_LSLEN);
	printf("4...\n");
	epd_write_data((BS97_INIT_LELEN << 8) | BS97_INIT_LBLEN);
	printf("5...\n");
	epd_write_data(BS97_INIT_PIXCLKDIV);
	
	write_epson_reg(0x0011, 0x310, (iba & 0xFFFF));
    write_epson_reg(0x0011, 0x312, ((iba >> 16) & 0xFFFF));
}

void init_controller(void)
{
		init_sys_run();
		init_dspe_cfg();
		init_dspe_tmg();
		printf("end of init controller ...\n");
		int vsize = epd_read_epson_reg(0x10,0x300),
			vsync = epd_read_epson_reg(0x10,0x302),
			vblen = epd_read_epson_reg(0x10,0x304),
			velen = (vblen >> 8) & 0xFF,
			hsize = epd_read_epson_reg(0x10,0x306),
			hsync = epd_read_epson_reg(0x10,0x308),
			hblen = epd_read_epson_reg(0x10,0x30A),
			helen = (hblen >> 8) & 0xFF;
		
			vblen &= 0xFF;
			hblen &= 0xFF;
	if( EINK_DEBUG )
	{
			printf("disp_timings: vsize=%d vsync=%d vblen=%d velen=%d\n", vsize, vsync, vblen, velen);
			printf("disp_timings: hsize=%d hsync=%d hblen=%d helen=%d\n", hsize, hsync, hblen, helen);
	}
}

 void epd_show_logo(unsigned short cmd)
{		
	int i, j = 0;	
	//char image[120000];
	char *image = malloc(496800 * sizeof(char));
	char *s;
	unsigned short value = 0;
 	//display black
   	value = 0x2 << 4;
	epd_send_command(0x20,value);	
	write_epson_reg(0x11,0x140,0x0020);
	
	//Host Memory Access Port Register[0154h]
	value = (0x154);
	epd_send_command(0x11,value);	
	
	for(i=0;i<242800;i++)
	{
		__raw_writew(0xFFFF,bsaddr);
	}

	//LD_IMG_END
	epd_write_command(0x23);

	//UPD_FULL[0334h]
 	value = (0x4000);
	epd_send_command(0x33,value); 
	//WAIT_DSPE_TRG
	epd_write_command(0x28);
    
  //WAIT_DSPE_FREND
    epd_write_command(0x29);
	for(i = 0; i <3; i++)
		mdelay(2000);    
 	epd_write_command(0x37);
	epd_write_command(0x28);
 //end display black	
/*  	setenv("copynand", "nand read 55000000 200000 3c000");
	s = getenv ("copynand");
	run_command (s, 0);
	
	image = yuliang;
	image = (volatile char *)0x55000000; */

  	image = gImage_second2_logo1;
 		epd_send_command(0x0B,(0x2 << 4));	
	
  	//INIT_ROTMODE[0140h] Rotation 90
		epd_send_command(0x0B,(0x1 << 8));	 
		epd_send_command(0x20,(0x2 << 4));	
		//udelay(1);
		udelay(3);
	
		//Host Memory Access Port Register[0154h]
  		epd_send_command(0x11,(0x154));	
				
    		for(i = 0; i <= 1199; i++)
			for(j = 413; j > 0; j=j-2)
				{
					__raw_writew((uint32_t)((image[i * 414 + j]| image[i * 414 + j - 1] << 8)), bsaddr);
				}    
/*  		    for(i=0;i<828*1200/4;i++) {
				 __raw_writew((uint32_t)((((image[i*2]) & 0x0f << 4)
																 | ((image[i*2] &0xf0) >> 4)
																 | ((image[i*2+1] &0x0f)<< 12)
																 | ((image[i*2+1] &0xf0)<< 4)							
																 )),bsaddr);//DCBA
															}  */
															
  			for(i=0;i<248400;i++)
			{
				__raw_writew(0xFFFF,bsaddr);
			}  
	
/*       		for(i=0;i<248400/1;i++)
		{
				if((i%(828*2))==0)
					{
					if(((i/(828*2))%2)==0)
						value = 0xFFFF;
					else
						value = 0x0000;
					}
				
				if(value==0)
					value = 0xF0F0;
				else
					value = 0x0000;
					
			for(j=0;j<1;j++)
			{
				__raw_writew(value,bsaddr);
			}
		}    */
  		//LD_IMG_END
		epd_write_command(0x23);

		//UPD_FULL[0334h]
		epd_send_command(0x33,(0x4200));	
	
		//WAIT_DSPE_TRG
		epd_write_command(0x28);
    
  	//WAIT_DSPE_FREND
  	epd_write_command(0x29);     
		
} 

int epd_update_area(int xstart, int ystart, int width, int length, char * buf)
{
	int i;
	
	epd_write_command(0x20);
	epd_write_data(0xf<<8);
	
	//Host Memory Access Port Register[0154h]
	epd_send_command(0x11,(0x154));	
	
	//for test
	for(i=0; i<248400; i++)
		{
			__raw_writew(0xffff,bsaddr);
		}
	
	epd_write_command(0x23);
	
	epd_write_command(0x36);
	epd_write_data(0x1<<4);
	epd_write_data(xstart);
	epd_write_data(ystart);
	epd_write_data(width);
	epd_write_data(length);
	
	epd_write_command(0x28);
	
	  	//WAIT_DSPE_FREND
  	epd_write_command(0x29);  
	return 0;
}

int eink_init(void)
{
	printf("begin eink_init...\n");
	
	int result = 0;
	u32 reg; 
/* 	reg = readl(MEM_SYS_CFG);
	if(EINK_DEBUG)
		printf("MEM_SYS_CFG is %x before modify\n",reg);
	
	reg = readl(MEM_SYS_CFG) & (~0x0000003f);
	writel(reg | 0x00000008, MEM_SYS_CFG); 

	reg = readl(MEM_SYS_CFG);
	if(EINK_DEBUG)
		printf("MEM_SYS_CFG is %x after modified\n",reg); */
		
	// GPC7  EINK_3.3V_EN config output
		gpcon = __raw_readl(GPCCON);
		gpcon = gpcon & ~(0xF << 28);
		__raw_writel(gpcon | (0x1 << 28), GPCCON);		
	//GPC7 EINK_3.3V_EN set low
		gpdat = __raw_readl(GPCDAT);
		gpdat =(gpdat & ~(1<<7));
		__raw_writel(gpdat,GPCDAT);
		
    //epd power pin gpc6; gpc6 output 1, enable EPD power
    gpcon = __raw_readl(GPCCON);
    gpcon  = (gpcon & ~(3<<24));
     __raw_writel(gpcon|(0x1<<24), GPCCON);
 
    gpdat = __raw_readl(GPCDAT);
    __raw_writel(gpdat|(1<<6),GPCDAT);  
	  
//craystal power gpp2
    gpcon = __raw_readl(GPPCON);
    gpcon  = (gpcon & ~(3<<4));
     __raw_writel(gpcon|(0x1<<4), GPPCON);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPPCON);
		printf("the GPPCON is 0x%08x ...\n",reg);
	}
 
    gpdat = __raw_readl(GPPDAT);
    __raw_writel(gpdat|(1<<2),GPPDAT);  
	if(EINK_DEBUG)
	{	
		reg = __raw_readl(GPNDAT);
		printf("the GPPDAT is 0x%08x ...\n",reg);	
	}
	
/*    //gpo9 craystal power
    gpcon = __raw_readl(GPOCON);
    gpcon  = (gpcon & ~(3<<18));
     __raw_writel(gpcon|(0x1<<18), GPOCON);
 
    gpdat = __raw_readl(GPODAT);
    __raw_writel(gpdat|(1<<9),GPODAT);    */
	
	mdelay(200);
	
 //config reset pin gpp9
    gpcon = __raw_readl(GPPCON);
    gpcon  = (gpcon & ~(3<<18));
    __raw_writel(gpcon|(0x1<<18), GPPCON);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPPCON);
		printf("the GPPCON is 0x%08x ...\n",reg);	
	}
	
	mdelay(10);
 //reset broadsheet 
    HIGH_RESET_PIN;
    mdelay(10);
    LOW_RESET_PIN;
    mdelay(30);
    HIGH_RESET_PIN;
 //reset 0k   

    
//setup HD/C signal config gpo2
	gpcon = __raw_readl(GPOCON);
	gpcon =(gpcon & ~(3<<4));
	__raw_writel(gpcon|(0x1<<4),GPOCON);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPOCON);
		printf("the GPOCON is 0x%08x ...\n",reg);	
	}
	
	gpdat = __raw_readl(GPODAT);
        gpdat =(gpdat & ~(1<<2));
        __raw_writel(gpdat|(0x1<<2), GPODAT);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPODAT);
		printf("the GPPDAT is 0x%08x ...\n",reg);	
	}
	
//now HRDY  setup to input gpo4 
	gpcon = __raw_readl(GPOCON);
        gpcon  = (gpcon & ~(3<<8));
        __raw_writel(gpcon, GPOCON);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPOCON);
		printf("the GPPCON is 0x%08x ...\n",reg);
	}
	
//now HIRQ  setup to input gpo3

	gpcon = __raw_readl(GPOCON);
        gpcon  = (gpcon & ~(3<<6));
        __raw_writel(gpcon, GPOCON);
	if(EINK_DEBUG)
	{
		reg = __raw_readl(GPOCON);
		printf("the GPOCON is 0x%08x ...\n",reg);	
	}

	mdelay(20);
		
	SROM_BW_REG &= ~(0x0000000f); 	
	SROM_BW_REG |= (1<<3) | (1<<2) | (1<<0);	

	//SROM_BC0_REG = (0x1<<28)|(0xf<<24)|(0x1c<<16)|(0x1<<12)|(0x6<<8)|(0x2<<4)|(0x0<<0);
	SROM_BC0_REG = (0x0<<28)|(0x0<<24)|(0xA<<16)|(0x1<<12)|(0x0<<8)|(0x2<<4)|(0x0<<0);
	//udelay(5); 
	udelay(12);
	
	reg = epd_read_epson_reg(0x10, 0x02);
	printf("Product ID is %x\n",reg);
	
	printf("end eink_init...\n");
	
	init_controller();
	
	bs_cmd_get_wfm_info();

			
 	epd_write_command(0x37);
	epd_write_command(0x28); 
	
	epd_show_logo(0);
	mdelay(5000);
	//epd_update_area(500,500,150,150,NULL);
	
	printf("************end epd_show_logo...\n");
	return result;
}