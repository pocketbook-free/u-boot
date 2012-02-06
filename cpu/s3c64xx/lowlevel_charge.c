#include <config.h>
#include <common.h>
#include <asm-arm/io.h>
#include <regs.h>
#include <s3c6410.h>

#define	I2C_WRITE	0
#define I2C_READ	1

#define I2C_OK		0
#define I2C_NOK		1
#define I2C_NACK	2
#define I2C_NOK_LA	3		/* Lost arbitration */
#define I2C_NOK_TOUT	4		/* time out */

#define I2CSTAT_BSY	0x20		/* Busy bit */
#define I2CSTAT_NACK	0x01		/* Nack bit */
#define I2CCON_IRPND	0x10		/* Interrupt pending bit */
#define I2C_MODE_MT	0xC0		/* Master Transmit Mode */
#define I2C_MODE_MR	0x80		/* Master Receive Mode */
#define I2C_START_STOP	0x20		/* START / STOP */
#define I2C_TXRX_ENA	0x10		/* I2C Tx/Rx enable */

#define MAX8698_BATT_SUPPLY	 1
#define MAX8698_USB_SUPPLY	 2
#define MAX8698_LINE_SUPPLY	 4

#define I2C_TIMEOUT 1			/* 1 second */

#define rIICCON		*((volatile unsigned long*) 0x7F00F000)
#define rIICSTAT 	*((volatile unsigned long*) 0x7F00F004)
#define rIICADD		*((volatile unsigned long*) 0x7F00F008)
#define rIICDS		*((volatile unsigned long*) 0x7F00F00C)
#define rIICLC		*((volatile unsigned long*) 0x7F00F010)


#define MAX17043_REG_VCELL		0x02
#define MAX17043_REG_SOC		  0x04
#define MAX17043_REG_MODE		  0x06 /* Relative State-of-Charge */
#define MAX17043_REG_VERSION	0x08
#define MAX17043_REG_CONFIG		0x0C
#define MAX17043_REG_COMMAND	0xFE


static inline void mdelay(int n)
{
	u32 ms = n;
	while(ms--)
	 udelay(1000);
}
static 
int WaitForXfer (void)
{
	int i;
	unsigned long status;
	
	i = I2C_TIMEOUT * 10000;
	status = rIICCON;
	while ((i > 0) && !(status & I2CCON_IRPND)) {
		udelay (100);
		status = rIICCON;
		i--;
	}

	return (status & I2CCON_IRPND) ? I2C_OK : I2C_NOK_TOUT;
}

static 
int IsACK (void)
{
	return (!(rIICSTAT & I2CSTAT_NACK));
}

static 
void ReadWriteByte (void)
{
	rIICCON &= ~I2CCON_IRPND;
}

static
void i2c_init (int speed, int slaveadd)
{
	int i, status;

	/* wait for some time to give previous transfer a chance to finish */

	i = I2C_TIMEOUT * 1000;
	status = rIICSTAT;
	while ((i > 0) && (status & I2CSTAT_BSY)) {
		udelay (1000);
		status = rIICSTAT;
		i--;
	}

	/* set prescaler, divisor according to freq, also set
	 * ACKGEN, IRQ */
	rIICCON = (0<<6) | (1<<5) | (7&0xf);

	/* init to SLAVE REVEIVE and set slaveaddr */
	rIICSTAT = 0;
	rIICADD = slaveadd;
	/* program Master Transmit (and implicit STOP) */
	rIICSTAT = I2C_MODE_MT | I2C_TXRX_ENA;

}

/*
 * cmd_type is 0 for write, 1 for read.
 *
 * addr_len can take any value from 0-255, it is only limited
 * by the char, we could make it larger if needed. If it is
 * 0 we skip the address write cycle.
 */
static
int i2c_transfer (unsigned char cmd_type,
		  unsigned char chip,
		  unsigned char addr[],
		  unsigned char addr_len,
		  unsigned char data[], unsigned short data_len)
{
	int i, status, result;

	if (data == 0 || data_len == 0) {
		/*Don't support data transfer of no length or to address 0 */
		return I2C_NOK;
	}

	/* Check I2C bus idle */
	i = I2C_TIMEOUT * 1000;
	status = rIICSTAT;
	while ((i > 0) && (status & I2CSTAT_BSY)) {
		udelay (1000);
		status = rIICSTAT;
		i--;
	}

	if (status & I2CSTAT_BSY)
		return I2C_NOK_TOUT;

	rIICCON |= 0x80;
	result = I2C_OK;

	switch (cmd_type) {
	case I2C_WRITE:
		if (addr && addr_len) {
			rIICDS = chip;
			/* send START */
			rIICSTAT = I2C_MODE_MT | I2C_TXRX_ENA | I2C_START_STOP;
			i = 0;
			while ((i < addr_len) && (result == I2C_OK)) {
				result = WaitForXfer ();
				rIICDS = addr[i];
				ReadWriteByte ();
				i++;
			}
			i = 0;
			while ((i < data_len) && (result == I2C_OK)) {
				result = WaitForXfer ();
				rIICDS = data[i];
				ReadWriteByte ();
				i++;
			}
		} else {
			rIICDS = chip;
			/* send START */
			rIICSTAT = I2C_MODE_MT | I2C_TXRX_ENA | I2C_START_STOP;
			i = 0;
			while ((i < data_len) && (result == I2C_OK)) {
				result = WaitForXfer ();
				rIICDS = data[i];
				ReadWriteByte ();
				i++;
			}
		}

		if (result == I2C_OK)
			result = WaitForXfer ();

		/* send STOP */
		rIICSTAT = I2C_MODE_MR | I2C_TXRX_ENA;
		ReadWriteByte ();
		break;

	case I2C_READ:
		if (addr && addr_len) {
			rIICSTAT = I2C_MODE_MT | I2C_TXRX_ENA;
			rIICDS = chip;
			/* send START */
			rIICSTAT |= I2C_START_STOP;
			result = WaitForXfer ();
			if (IsACK ()) {
				i = 0;
				while ((i < addr_len) && (result == I2C_OK)) {
					rIICDS = addr[i];
					ReadWriteByte ();
					result = WaitForXfer ();
					i++;
				}

				rIICDS = chip;
				/* resend START */
				rIICSTAT =  I2C_MODE_MR | I2C_TXRX_ENA |
						I2C_START_STOP;
				ReadWriteByte ();
				result = WaitForXfer ();
				i = 0;
				while ((i < data_len) && (result == I2C_OK)) {
					/* disable ACK for final READ */
					if (i == data_len - 1)
						rIICCON &= ~0x80;
					ReadWriteByte ();
					result = WaitForXfer ();
					data[i] = rIICDS;
					i++;
				}
			} else {
				result = I2C_NACK;
			}

		} else {
			rIICSTAT = I2C_MODE_MR | I2C_TXRX_ENA;
			rIICDS = chip;
			/* send START */
			rIICSTAT |= I2C_START_STOP;
			result = WaitForXfer ();

			if (IsACK ()) {
				i = 0;
				while ((i < data_len) && (result == I2C_OK)) {
					/* disable ACK for final READ */
					if (i == data_len - 1)
						rIICCON &= ~0x80;
					ReadWriteByte ();
					result = WaitForXfer ();
					data[i] = rIICDS;
					i++;
				}
			} else {
				result = I2C_NACK;
			}
		}

		/* send STOP */
		rIICSTAT = I2C_MODE_MR | I2C_TXRX_ENA;
		ReadWriteByte ();
		break;

	default:
		result = I2C_NOK;
		break;
	}

	return (result);
}

static
int i2c_probe (uchar chip)
{
	uchar buf[1];

	buf[0] = 0;

	/*
	 * What is needed is to send the chip address and verify that the
	 * address was <ACK>ed (i.e. there was a chip at that address which
	 * drove the data line low).
	 */
	return (i2c_transfer (I2C_READ, chip << 1, 0, 0, buf, 1) != I2C_OK);
}

int i2c_read (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	uchar xaddr[4];
	int ret;

	if (alen > 4) {
		return 1;
	}

	if (alen > 0) {
		xaddr[0] = (addr >> 24) & 0xFF;
		xaddr[1] = (addr >> 16) & 0xFF;
		xaddr[2] = (addr >> 8) & 0xFF;
		xaddr[3] = addr & 0xFF;
	}

	if ((ret =
	     i2c_transfer (I2C_READ, chip << 1, &xaddr[4 - alen], alen,
			   buffer, len)) != 0) {
		return 1;
	}
	return 0;
}

static
int i2c_write (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	uchar xaddr[4];

	if (alen > 4) {
		return 1;
	}

	if (alen > 0) {
		xaddr[0] = (addr >> 24) & 0xFF;
		xaddr[1] = (addr >> 16) & 0xFF;
		xaddr[2] = (addr >> 8) & 0xFF;
		xaddr[3] = addr & 0xFF;
	}
	return (i2c_transfer
		(I2C_WRITE, chip << 1, &xaddr[4 - alen], alen, buffer,
		 len) != 0);
}

static int max17043_read(uchar chipaddr,int reg)
{
	int ret;
	unsigned short value; 
	 ret = i2c_read(chipaddr, reg, 1, (uchar*) &value, 2);
	 if(ret)
	 {
	   printf("the max17043 read reg error\n");
	  return ret;
	 }
	 return value;
}

int max17043_write(uchar chipaddr,int reg,unsigned short value)
{
	int ret;
	 ret = i2c_write(chipaddr, reg, 1, (uchar*) &value, 2);
	 if(ret)
	 {
	   printf("the max17043 read reg error\n");
	  return ret;
	 }
	 return 0;
}

int max17043_battery_capacity(int reg)
{
	int capacity;
	int data;
	unsigned short value; 
//	data = max17043_read(0x36,reg);
	
	data = i2c_read(0x36, reg, 1, (uchar*) &value, 2);
	
	printf("reg = 0x%x,value = 0x%x,data = %d\n",reg,value,data);
	capacity = value&0xff;
	printf("the battery capacity = %d\n",capacity);
	return capacity;
}

int max17043_battery_voltage(int reg)
{
	int voltage;
	int data;
	int value_high_bit;
	int value_low_bit;
	
	data = max17043_read(0x36,reg);
	
	printf("reg = 0x%x,data = 0x%x\n",reg,data);
	
	value_high_bit = (data & 0xff)<<4;
	value_low_bit = (data & 0xff00)>>12;

	voltage = (value_high_bit + value_low_bit);
	
	voltage = voltage * 125;
	
	return voltage;
}

int detect_usb_ac_charge()
{
	unsigned long val;
	int ret = 0;
	unsigned long cfg;
	//hprt judge usb or adapter supply
	
	//s3c_usbctl_init();
	//s3c_usbc_activate();
	
	#define PORT_LINE_STATUS_MASK	(0x3 << 10)
		val = __raw_readl(S3C_OTG_HPRT);
    val  = val & PORT_LINE_STATUS_MASK;
    printf("******val = 0x%x********\n",val);
  
	if(val && (val == PORT_LINE_STATUS_MASK))
	{ 
	  ret = MAX8698_LINE_SUPPLY;
	} 
	else if(val == 0x400)
	{
		ret = MAX8698_USB_SUPPLY;
	}
	else if(val == 0x0)
	{
		ret = MAX8698_BATT_SUPPLY;
	}
	return ret;
}

static int charger_config(int supplies)
{
	int ret = 0;
	unsigned long cfg;
	printf( "****%s start ....*****\n",__FUNCTION__);

	if(supplies & MAX8698_LINE_SUPPLY)
	 {
		/********************************* 
		 charge_en1 , charge_en2 
		     1           0
		 GPP11 set 1 , GPP10 set 0
		*********************************/ 
   // GPP11 set 1
  cfg = __raw_readl(GPPCON);
  cfg  = (cfg & ~(0x3<<22));
  __raw_writel(cfg | (0x01<<22), GPPCON);
  
  cfg = __raw_readl(GPPDAT);
  cfg =(cfg & ~(0x1<<11));
  __raw_writel(cfg | (0x1<<11) ,GPPDAT);
  
 // GPF10 set 0
  cfg = __raw_readl(GPPCON);
  cfg  = (cfg & ~(0x3<<20));
  __raw_writel(cfg | (0x1<<20), GPPCON);
  
  cfg = __raw_readl(GPPDAT);
  cfg =(cfg & ~(0x1<<10));
  __raw_writel(cfg ,GPPDAT);
  
  printf("adapter charge\n");
    ret = 750;
	 }
	else
	 {
		/********************************* 
		 charge_en1 , charge_en2 
		     0           1
		 GPP11 set 1 , GPP10 set 0
		*********************************/

   // GPP11 set 0
  cfg = __raw_readl(GPPCON);
  cfg  = (cfg & ~(0x3<<22));
  __raw_writel(cfg | (0x01<<22), GPPCON);
  
  cfg = __raw_readl(GPPDAT);
  cfg =(cfg & ~(1<<11));
  __raw_writel(cfg ,GPPDAT);
  
 // GPP10 set 1
  cfg = __raw_readl(GPPCON);
  cfg  = cfg & ~(0x3<<20);
  __raw_writel(cfg | (0x01<<20), GPPCON);
  
  cfg = __raw_readl(GPPDAT);
  cfg =(cfg & ~(1<<10));
  __raw_writel(cfg | (1<<10) ,GPPDAT);
  
   printf("usb charge\n");
    ret = 500;
	 }
	
	return ret;
}

void lowlevel_init_max17043(void)
{
	unsigned short value; 
	int ret;
	int batt_capacity;
	unsigned long cfg;
	printf("enter lowlevle_init_max17043\n");
	
	volatile uint* gpio_reg1 = (uint*)0x7F008020;
	volatile uint* gpio_reg2 = (uint*)0x7F008028;
	
//	*gpio_reg1 = 0x02246600;
//	*gpio_reg2 = 0x000028a0;

 	cfg = __raw_readl(GPBCON);
  cfg  = (cfg & ~(0x7<<8) & ~(0x07<<12));
  __raw_writel(cfg |(0x06<<8)|(0x06<<12), GPBCON);
  
  cfg = __raw_readl(GPBPUD);
  cfg  = (cfg & ~(0x3<<4) & ~(0x03<<6));
  __raw_writel(cfg | (0x02<<4)|(0x02<<6), GPBPUD);
	printf("********GPBCON =0x%x,GPBCON= 0x%x,GPBPUD-data = 0x%x,GPBPUD = 0x%x*********\n",__raw_readl(GPBCON),GPBCON,__raw_readl(GPBPUD),GPBPUD);
	printf("********gpio_reg1 =0x%x,gpio_reg2 = 0x%x*********\n",*gpio_reg1,*gpio_reg2);
	
/*
 // GPF12 set 1  codec_en set 1
  cfg = __raw_readl(GPFCON);
  cfg  = (cfg & ~(0x3<<24));
  __raw_writel(cfg | (0x01<<24), GPFCON);
  
  cfg = __raw_readl(GPFDAT);
  cfg =(cfg & ~(0x1<<12));
  __raw_writel(cfg | (0x1<<12) ,GPFDAT);
*/
	printf("enter i2c_init\n");
	i2c_init(0, 0x10);
	
//	s3c_usbctl_init();
//	s3c_usbc_activate();
	i2c_read(0x36, 0x02, 1, (uchar*) &value, 2);
		printf("0x02 = 0x%x\n",value);
  printf("******val = 0x%x********\n",__raw_readl(S3C_OTG_HPRT));
	i2c_read(0x36, 0x0c, 1, (uchar*) &value, 2);
	printf("0x0c = 0x%x\n",value);
	value = 0x1697;                                               /// enable LD0O -4 , DCDC1,3,4,5,6
	i2c_write(0x36, 0x0c, 1, (uchar*) &value, 2);                 ///William 20090609
	
	ret = i2c_read(0x36, 0x0c, 1, (uchar*) &value, 2);
	printf("0x0c = 0x%x,ret = %d\n",value,ret);
	
	i2c_read(0x36, 0x04, 1, (uchar*) &value, 2);
	printf("0x04 = 0x%x\n",value);
	
	batt_capacity = max17043_battery_capacity(MAX17043_REG_SOC);
	
	i2c_read(0x36, 0x02, 1, (uchar*) &value, 2);
		printf("0x02 = 0x%x\n",value);
  printf("****batt_vol=%d******\n",max17043_battery_voltage(0x02));
	
	/* disable IIC */
//	rIICCON = (1<<6) | (1<<5) | (0xF&0xf);
//	rIICSTAT = 0;
}


void ShowVoltage(void)
{
	rIICCON = (1<<6) | (1<<5) | (0xF&0xf);
	rIICSTAT = 0;

}

void set_red_led(int value)
{
	unsigned long cfg;
	//set gpk2 
	 cfg = __raw_readl(GPKCON0);
  cfg  = (cfg & ~(0xf<<8));
  __raw_writel(cfg | (0x0001<<8), GPKCON0);
  
  cfg = __raw_readl(GPKDAT);
  cfg =(cfg & ~(0x1<<2));
  __raw_writel(cfg | (value<<2) ,GPKDAT);
}

void set_green_led(int value)
{
	unsigned long cfg;
	//set gpk1 
	 cfg = __raw_readl(GPKCON0);
  cfg  = (cfg & ~(0xf<<4));
  __raw_writel(cfg | (0x0001<<4), GPKCON0);
  
  cfg = __raw_readl(GPKDAT);
  cfg =(cfg & ~(0x1<<1));
  __raw_writel(cfg | (value<<1) ,GPKDAT);
}

void set_led_flash()
{
	mdelay(1500);
	set_red_led(1);
	mdelay(1500);
	set_red_led(0);
}

int shutdown_process()
{
	unsigned long cfg;

		//set gpn12
	 cfg = __raw_readl(GPLCON0);
   cfg  = (cfg & ~(0x0f<<8));
   __raw_writel(cfg | (0x01<<8), GPLCON0);
  
   cfg = __raw_readl(GPLDAT);
   cfg =(cfg & ~(0x1<<2));
   __raw_writel(cfg,GPLDAT);
   
 //  printf("gpncon = 0x%x,gpndat=0x%x\n",__raw_readl(GPNCON),__raw_readl(GPNDAT));
	
	return 0;
}



