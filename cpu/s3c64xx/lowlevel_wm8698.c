/*
 *			  \cpu\s3c64xx\lowlevel_wm8698.c
 *				Update Logs
 *		modify gpio from GPB5/6 to GPB2/3, use I2c channel 1 
 *						James J.Weng	2010.4.12
 *     
 */
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

#define I2C_TIMEOUT 1			/* 1 second */

#define rIICCON		*((volatile unsigned long*) 0x7F004000)
#define rIICSTAT 	*((volatile unsigned long*) 0x7F004004)
#define rIICADD		*((volatile unsigned long*) 0x7F004008)
#define rIICDS		*((volatile unsigned long*) 0x7F00400C)
#define rIICLC		*((volatile unsigned long*) 0x7F004010)

typedef unsigned long ulong;
typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned short ushort;

static
void udelay(uint n)
{
	while(n > 1) n--;

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
				result = WaitForXfer ();//bus idle
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

static
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

//ushort id_linus;

void lowlevel_init_wm8698()
{
	ushort value;
	ushort *pData = 0x0c000000;

	volatile uint* gpio_reg1 = (uint*)0x7F008020;
	volatile uint* gpio_reg2 = (uint*)0x7F008028;

	*gpio_reg1 = 0x02246600;
	*gpio_reg2 = 0x00001a65;

	i2c_init(0, 0x10);
	//With current DCDC/LDO status/outputs(CONF0=1,CONF1=0), please help to write registers by IIC.
	//0xB4h = 0x000Eh; //DCDC1 output from 1V change to 1.2V
	//0xC8h = 0x0006h; //LDO1 output from 1V change to 1.2V

/*    	i2c_read(0x66, 0x0, 1, (uchar*) &value, 2);
	value=value|0x0c;
	i2c_write(0x66, 0x0, 1, (uchar*) &value, 2);
	
	i2c_read(0x66, 0x1, 1, (uchar*) &value, 2);
	value=value|0x60;
	i2c_write(0x66, 0x1, 1, (uchar*) &value, 2); 
	
 	value = 0x99;
	i2c_write(0x66, 0x06, 1, (uchar*) &value, 1);
	
	value = 0x02;
	i2c_write(0x66, 0x07, 1, (uchar*) &value, 1);
	
	//ldo2/ldo3 1.2V
	value = 0x08;
	i2c_write(0x66, 0x08, 1, (uchar*) &value, 1);
	
	//ldo4 3.3 to 2.8V
	value = 0x11;
	//value = 0x0C;
	i2c_write(0x66, 0x09, 1, (uchar*) &value, 1);
	
	//ldo5 3.0 to 1.8V
  	value = 0x11;
  	//value = 0x02;
	i2c_write(0x66, 0x0A, 1, (uchar*) &value, 1);
	
	//ldo6 3.3V
	value = 0x11;
	i2c_write(0x66, 0x0B, 1, (uchar*) &value, 1);
	
	//ldo7 3.3 to 1.7V
	value = 0x11;
	//value = 0x01;
	i2c_write(0x66, 0x0c, 1, (uchar*) &value, 1);  
	
	//ldo9 3.3 to 2.8V
	value = 0x11;
	//value = 0x0C;
	i2c_write(0x66, 0x0e, 1, (uchar*) &value, 1);  */   
	///////////////////////////////////////////
	
	i2c_read(0x66, 0x0, 1, (uchar*) &value, 1);
	value=value|0xfe;
	i2c_write(0x66, 0x0, 1, (uchar*) &value, 1);
	
	i2c_read(0x66, 0x2, 1, (uchar*) &value, 1);
	value=value|0x08;
	i2c_write(0x66, 0x2, 1, (uchar*) &value, 1);
	
 	i2c_read(0x66, 0x1, 1, (uchar*) &value, 1);
	value=value|0x60;
	i2c_write(0x66, 0x1, 1, (uchar*) &value, 1);  
	
  	value = 0x99;
	i2c_write(0x66, 0x06, 1, (uchar*) &value, 1);
	
	value = 0x02;
	i2c_write(0x66, 0x07, 1, (uchar*) &value, 1);
	
	//ldo2/ldo3 1.2V
	value = 0x88;
	i2c_write(0x66, 0x08, 1, (uchar*) &value, 1);
	
	//ldo4 3.3
	value = 0x11;
	i2c_write(0x66, 0x09, 1, (uchar*) &value, 1);
	
	//ldo5 3.3
  	value = 0x11;
	i2c_write(0x66, 0x0A, 1, (uchar*) &value, 1);
	
	//ldo6 none
/* 	value = 0x11;
	i2c_write(0x66, 0x0B, 1, (uchar*) &value, 1); */
	
	//ldo7 3.3
	value = 0x11;
	i2c_write(0x66, 0x0c, 1, (uchar*) &value, 1);  
	
	//ldo8 3.3
	value = 0x30;
	i2c_write(0x66, 0x0d, 1, (uchar*) &value, 1);  
	
	//ldo9 3.3
	value = 0x11;
	i2c_write(0x66, 0x0e, 1, (uchar*) &value, 1);     
	
	///////////////////////////////////////////
	//i2c_read(0x66, 0x1, 1, (uchar*) pData, 2);
	//printf("\nwm8698: 0x%08x\n", *pData);
/* #ifdef ENPMIC
	pmic_write(pmicclient,0xb4,0x0e);
	pmic_write(pmicclient,0xc3,0x0e);
	pmic_write(pmicclient,0xc8,0x06); */

//	i2c_read(0x1a, 0, 1, (uchar*) &id_linus, 2);
//	printf("\nwm8698: 0x%x\n", id_linus);
//	i2c_read(0x1a, 0xB4, 1, (uchar*) &value, 2);
//	i2c_read(0x1a, 0xC8, 1, (uchar*) &value, 2);

	/* disable IIC */
//  rIICCON = (1<<6) | (1<<5) | (0xF&0xf);
//	rIICSTAT = 0;
}

void lowlevel_init_wm8698_for_poweroff()
{
 	ushort value;
	unsigned long cfg;
	ushort *pData = 0x0c000000;

	volatile uint* gpio_reg1 = (uint*)0x7F008020;
	volatile uint* gpio_reg2 = (uint*)0x7F008028;

	//*gpio_reg1 = 0x02246600;
	//*gpio_reg2 = 0x00001a65;
/*  	cfg = readl(GPBCON);
	cfg  = (cfg & ~(0x3<<20) & ~(0x3<<24));
	writel(cfg |(0x02<<20)|(0x02<<24), GPBCON);
  
	cfg = readl(GPBPUD);
	cfg  = (cfg & ~(0x3<<10) & ~(0x03<<12));
	writel(cfg | (0x02<<10)|(0x02<<12), GPBPUD); */
	*gpio_reg1 = *gpio_reg1 & ~(0xff << 20) | (0x22 << 20);
	*gpio_reg2 = *gpio_reg2 & ~(0xf << 10) | (0xa << 10);

	i2c_init(0, 0x10); 
	
 	i2c_read(0x66, 0x0, 1, (uchar*) &value, 1);
	value=0xfa;
	i2c_write(0x66, 0x0, 1, (uchar*) &value, 1);
	
	i2c_read(0x66, 0x1, 1, (uchar*) &value, 1);
	value=0x30;
	i2c_write(0x66, 0x1, 1, (uchar*) &value, 1);  
	
/* 	//ldo9 3.3
	value = 0x11;
	i2c_write(0x66, 0x0e, 1, (uchar*) &value, 1);   
	
	//ldo5 3.3
  	value = 0x11;
	i2c_write(0x66, 0x0A, 1, (uchar*) &value, 1);	 */
}
//0110 0001 0100 0011
//0011 1010 0001 1010

extern void wm8698_diag(void)
{
	ushort id;

	i2c_read(0x66, 0x0, 1, (uchar*) &id, 2);
	printf("\nwm8698: 0x%08x\n", id);
	i2c_read(0x66, 0x0a, 1, (uchar*) &id, 2);
	printf("\nwm8698: 0x%08x\n", id);
}