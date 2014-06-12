/*
 * (C) Copyright 2013
 * Kentaro Sekimoto
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/fm3.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/i2c.h>

//#define FM3_I2C_DEBUG_FUNC
//#define FM3_I2C_DEBUG_BYTE
//#define FM3_I2C_DEBUG
#define FM3_I2C_DEBUG_KERNEL_LOG
#define FM3_I2C_DEBUG_KERNEL_LOG_BYTE

#ifdef FM3_I2C_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#endif

#ifdef FM3_I2C_DEBUG
#define dprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

// IBCR
#define IBCR_MSS    (1 << 7)    // master/slave selection bit
#define IBCR_ACT    (1 << 6)    // action flag/repeated-start condition bit (ACT for writing/SCC for reading)
#define IBCR_SCC    (1 << 6)    // action flag/repeated-start condition bit (ACT for writing/SCC for reading)
#define IBCR_ACKE   (1 << 5)    // data byte acknowledge bit
#define IBCR_WSEL   (1 << 4)    // wait selection bit
#define IBCR_CNDE   (1 << 3)    // condition detection enable bit
#define IBCR_INTE   (1 << 2)    // interrupt enable bit
#define IBCR_BER    (1 << 1)    // bus error flag bit (1:error detected)
#define IBCR_INT    (1 << 0)    // interrupt flag bit
// IBSR
#define IBSR_FBT    (1 << 7)    // first byte bit
#define IBSR_RACK   (1 << 6)    // acknowledge flag bit
#define IBSR_RSA    (1 << 5)    // reservation detection bit
#define IBSR_TRX    (1 << 4)    // data transfer direction bit
#define IBSR_AL     (1 << 3)    // arbitration lost bit
#define IBSR_RSC    (1 << 2)    // repeated-start condition confirmation bit
#define IBSR_SPC    (1 << 1)    // stop condition confirmation bit
#define IBSR_BB     (1 << 0)    // bus state bit
// SMR
#define SMR_MD2     (1 << 7)    // operation mode bit2
#define SMR_MD1     (1 << 6)    // operation mode bit1
#define SMR_MD0     (1 << 5)    // operation mode bit0
#define SMR_WUCR    (1 << 4)    // wake up control bit
#define SMR_RIE     (1 << 3)    // receive interrupt enable
#define SMR_TIE     (1 << 2)    // transmit interrupt enable
// EIBCR
#define EIBCR_SDAS  (1 << 5)    // SDA status bit
#define EIBCR_SCLS  (1 << 4)    // SCL status bit
#define EIBCR_SDAC  (1 << 3)    // SDA control bit
#define EIBCR_SCLC  (1 << 2)    // SCL control bit
#define EIBCR_SOCE  (1 << 1)    // serial output control enable
#define EIBCR_BEC   (1 << 0)    // bus error control bit

struct fm3_i2c_regs
{
	volatile uint8_t SMR;
	union {
		volatile uint8_t SCR;
		volatile uint8_t IBCR;
	};
	volatile uint8_t RESERVED0[2];
	union {
		volatile uint8_t ESCR;
		volatile uint8_t IBSR;
	};
	volatile uint8_t SSR;
	volatile uint8_t RESERVED1[2];
	union {
		volatile uint16_t RDR;
		volatile uint16_t TDR;
	};
	volatile uint8_t RESERVED2[2];
	volatile uint16_t BGR;
	volatile uint8_t RESERVED3[2];
	volatile uint8_t ISBA;
	volatile uint8_t ISMK;
	volatile uint8_t RESERVED4[2];
	volatile uint8_t FCR0;
	volatile uint8_t FCR1;
	volatile uint8_t RESERVED5[2];
	volatile uint8_t FBYTE1;
	volatile uint8_t FBYTE2;
	volatile uint8_t RESERVED6[2];
	union {
		volatile uint8_t SCSTR0;
		volatile uint8_t NFCR;
	};
	union {
		volatile uint8_t SCSTR1;
		volatile uint8_t EIBCR;
	};
	volatile uint8_t RESERVED7[2];
};

static struct fm3_i2c_regs *FM3_CSIO_REGS[] = {
	(struct fm3_i2c_regs  *)FM3_MFS0_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS1_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS2_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS3_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS4_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS5_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS6_CSIO_BASE,
	(struct fm3_i2c_regs  *)FM3_MFS7_CSIO_BASE
};

struct i2c_fm3 {
	struct platform_device *dev;	/* Platform device */
	int bus;						/* Bus (ID): channel number */
	int mod;						/* Mode: channel mode */
	unsigned int regs_base;			/* Regs base (phys) */
	unsigned int regs_size;			/* Regs size */
	void * __iomem regs;			/* Regs base (virt) */
	int irq;						/* IRQ # */
	unsigned int ref_clk;			/* Ref clock */
	unsigned int i2c_clk;			/* Bus clock */
	struct i2c_msg*	msg;			/* Current message */
	int msg_n;						/* Segments in msg */
	int msg_i;						/* Idx in a segment */
	volatile int msg_status;		/* Message status */
	struct i2c_adapter adap;		/* I2C adapter data */
	wait_queue_head_t wait;			/* Wait queue */
	spinlock_t lock;
};

#define DEF_BAUDRATE_I2C	400000

#define SMR_SOE          0x01U
#define SMR_SCKE         0x02U
#define SMR_BDS          0x04U
#define SMR_SCINV        0x08U
#define SMR_WUCR         0x10U
#define SMR_MD_UART      0x00U
#define SMR_MD_UART_MP   0x20U
#define SMR_MD_SIO       0x40U
#define SMR_MD_LIN       0x60U
#define SMR_MD_I2C       0x80U

#define SCR_TXE          0x01U
#define SCR_RXE          0x02U
#define SCR_TBIE         0x04U
#define SCR_TIE          0x08U
#define SCR_RIE          0x10U
#define SCR_SPI          0x20U
#define SCR_MS           0x40U
#define SCR_UPCL         0x80U

#define SSR_TBI          0x01U
#define SSR_TDRE         0x02U
#define SSR_RDRF         0x04U
#define SSR_ORE          0x08U
#define SSR_FRE          0x10U
#define SSR_PE           0x20U
#define SSR_REC          0x80U

#define ESCR_P           0x08U
#define ESCR_PEN         0x10U
#define ESCR_INV         0x20U
#define ESCR_ESBL        0x40U
#define ESCR_FLWEN       0x80U
#define ESCR_DATABITS_8  0x00U
#define ESCR_DATABITS_5  0x01U
#define ESCR_DATABITS_6  0x02U
#define ESCR_DATABITS_7  0x03U
#define ESCR_DATABITS_9  0x04U

static int i2c_fm3_hw_initialized = 0;

static inline int i2c_fm3_hw_cs_max(struct i2c_fm3 *c)
{
	int ret = 8;
	FUNC_ENTER();
	FUNC_EXIT();
	return ret;
}

static int i2c_fm3_hw_init(struct i2c_fm3 *c)
{
	int ret = 0;
	struct fm3_i2c_regs *i2c;
	FUNC_ENTER();
	if (c->bus < i2c_fm3_hw_cs_max(c)) {
		fm3_mfs_set_mode(c->bus*3 + c->mod);
		i2c = FM3_CSIO_REGS[c->bus];
		i2c->ISMK &= 0x7f;		// Disable
		i2c->SMR = 0x80;		// I2C mode
		i2c->SSR = 0;
		if (c->i2c_clk)
			i2c->BGR = (uint16_t)((c->ref_clk / c->i2c_clk) - 1);
		else
			i2c->BGR = (uint16_t)((c->ref_clk / DEF_BAUDRATE_I2C) - 1);
		i2c->ISBA = 0;			// Doesn't compare slave address
		i2c->ISMK |= 0x80;		// Enable
	} else
		ret = -1;
	FUNC_EXIT();
	return ret;
}

static void i2c_fm3_hw_release(struct i2c_fm3 *c)
{
	struct fm3_i2c_regs *i2c;
	FUNC_ENTER();
	// ToDo:
	i2c = FM3_CSIO_REGS[c->bus];
	i2c->ISMK &= 0x7f;	// Disable
	FUNC_EXIT();
}

static inline void i2c_fm3_hw_clear(struct i2c_fm3 *c)
{
	//struct fm3_i2c_regs *i2c;
	FUNC_ENTER();
	FUNC_EXIT();
}

static irqreturn_t i2c_fm3_irq(int irq, void *d)
{
	irqreturn_t ret = IRQ_HANDLED;
	struct i2c_fm3 *c = (struct i2c_fm3 *)d;
	struct fm3_i2c_regs *i2c = FM3_CSIO_REGS[c->bus];
	uint8_t ibcr = i2c->IBCR;
	uint8_t ibsr = i2c->IBSR;
	FUNC_ENTER();
	dprintk("ch=%d ibcr=0x%02x ibsr=0x%02x n=%d i/len=(%d/%d) addr=0x%0x flags=0x%0x\n", c->bus, ibcr, ibsr, c->msg_n, c->msg_i, c->msg->len, c->msg->addr, c->msg->flags);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
	pr_debug("ch=%d ibcr=0x%02x ibsr=0x%02x n=%d i/len=(%d/%d) addr=0x%0x flags=0x%0x\n", c->bus, ibcr, ibsr, c->msg_n, c->msg_i, c->msg->len, c->msg->addr, c->msg->flags);
#endif
	if (ibcr & IBCR_BER) {
		// detect bus error
		dprintk("I2C bus err\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C bus err\n");
#endif
		i2c->IBCR &= ~(IBCR_INT);
		c->msg_status = -EIO;
		goto i2c_fm3_irq_err;
	}
	if ((ibcr & IBCR_ACT) == 0) {
		// detect arbitration lost
		dprintk("I2C arbitration err\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C arbitration err\n");
#endif
		i2c->IBCR &= ~(IBCR_INT);
		c->msg_status = -EIO;
		goto i2c_fm3_irq_err;

	}
	if ((ibcr & IBCR_MSS) == 0) {
		// detect slave configuration
		dprintk("I2C slave err\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C slave err\n");
#endif
		i2c->IBCR &= ~(IBCR_INT);
		c->msg_status = -EIO;
		goto i2c_fm3_irq_err;

	}
	if (ibsr & IBSR_RSA) {
		// detect reservation address
		dprintk("I2C rsa err\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C rsa err\n");
#endif
		i2c->IBCR &= ~(IBCR_INT);
		c->msg_status = -EIO;
		goto i2c_fm3_irq_err;

	}
	if (ibsr & IBSR_RACK) {
		// detect H
		dprintk("I2C NACK detected\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C NACK detected\n");
#endif
		c->msg_status = -EIO;
		goto i2c_fm3_irq_err;
	}

	if (ibsr & IBSR_SPC) {
		// detect stop condition
		dprintk("I2C stop condition\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C stop condition\n");
#endif
		i2c->IBSR &= ~IBSR_SPC;
		i2c->IBCR &= ~(IBCR_CNDE |	// disable repeated start/stop condition interrupt
				IBCR_SCC);
		c->msg_status = 0;
	} else
	if (ibsr & IBSR_TRX) {
		// write
		if (c->msg_i < c->msg->len) {
#if defined(FM3_I2C_DEBUG_KERNEL_LOG_BYTE)
				pr_debug("w[%d]= %02x\n",c->msg_i, c->msg->buf[c->msg_i]);
				i2c->TDR = c->msg->buf[c->msg_i];
#else
				i2c->TDR = c->msg->buf[c->msg_i];
#endif
				c->msg_i++;
				i2c->IBCR =			// 0x84
						IBCR_MSS |	// select master
						IBCR_INTE;	// enable interrupt
			}
			else if (--(c->msg_n) == 0) {
				// completed
				dprintk("I2C tx completed\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
				pr_debug("I2C tx completed\n");
#endif
				c->msg_status = 0;
				goto i2c_fm3_irq_exit;
			}
			else {
				c->msg++;
				c->msg_i = 0;
				i2c->TDR = (c->msg->addr << 1) | ((c->msg->flags & I2C_M_RD)? 1:0);
		        i2c->IBCR =   // 0xc5
		                IBCR_MSS |  // select master
		                IBCR_SCC |  // invoke repeated-start condition
		                IBCR_INTE | // enable interrupt
		                IBCR_INT;   // set interrupt flag
		        dprintk("I2C tx rstart:ibcr=0x%02x\n", i2c->IBCR);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		        pr_debug("I2C tx rstart:ibcr=0x%02x\n", i2c->IBCR);
#endif
		}
		goto i2c_fm3_irq_exit;
	} else {
		// read
		if (ibsr & IBSR_FBT) {
			// 1st byte interrupt
			dprintk("I2C rx first byte\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
			pr_debug("I2C rx first byte\n");
#endif
		} else {
#if defined(FM3_I2C_DEBUG_KERNEL_LOG_BYTE)
			uint8_t v = (uint8_t)i2c->RDR;
			pr_debug("r[%d]= %02x\n",c->msg_i, v);
			c->msg->buf[(c->msg_i)++] = v;
#else
			c->msg->buf[(c->msg_i)++] = (uint8_t)i2c->RDR;
#endif
		}
		if (c->msg_i < c->msg->len) {
			i2c->IBCR =
					IBCR_MSS |	// select master
					IBCR_ACKE |	// invoke "L" as acknowledge
					IBCR_WSEL |	// wait selection bit
					IBCR_INTE;	// enable interrupt
			dprintk("I2C rx msg_i=0x%02d ibcr=0x%02x\n", c->msg_i, ibcr);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
			pr_debug("I2C rx msg_i=0x%02d ibcr=0x%02x\n", c->msg_i, ibcr);
#endif
			goto i2c_fm3_irq_exit;
		} else {
			// last byte: NACK response
			ibcr = (ibcr & ~(IBCR_ACKE | IBCR_INTE | IBCR_INT)) | IBCR_WSEL;
			i2c->IBCR = ibcr;
			dprintk("I2C rx NACK ibcr=0x%02x\n", ibcr);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
			pr_debug("I2C rx NACK ibcr=0x%02x\n", ibcr);
#endif
			if (--(c->msg_n) == 0) {
				dprintk("I2C end\n");
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
				pr_debug("I2C end\n");
#endif
				c->msg_status = 0;
				goto i2c_fm3_irq_exit;
			}
			// repeated start condition
			c->msg++;
			c->msg_i = 0;
			i2c->TDR = (c->msg->addr << 1) | ((c->msg->flags & I2C_M_RD)? 1:0);
			i2c->IBCR =   // 0xc5
					IBCR_MSS |  // select master
					IBCR_SCC |  // invoke repeated-start condition
					IBCR_ACKE |	// invoke "L" as acknowledge
					IBCR_INTE | // enable interrupt
					IBCR_INT;   // set interrupt flag
			dprintk("I2C rstart:ibcr=0x%02x\n", i2c->IBCR);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
			pr_debug("I2C rstart:ibcr=0x%02x\n", i2c->IBCR);
#endif
		}
	}
i2c_fm3_irq_err:
i2c_fm3_irq_exit:
	if (c->msg_status != -EBUSY) {
	    ibcr = IBCR_ACKE;
	    i2c->IBCR = ibcr;
		dprintk("I2C stop condition ibcr=0x%02x\n", ibcr);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("I2C stop condition ibcr=0x%02x\n", ibcr);
#endif
		wake_up(&c->wait);
	}
	FUNC_EXIT();
	return ret;
}

static int i2c_fm3_transfer(struct i2c_adapter *a, struct i2c_msg *m, int n)
{
	int ret = 0;
	struct i2c_fm3 *c = a->algo_data;
	struct fm3_i2c_regs *i2c;
	unsigned short sla;
	unsigned char ibcr, ibsr, ssr;
	FUNC_ENTER();
	if (!i2c_fm3_hw_initialized) {
		ret = -1;
		dprintk("%s warning not initialized yet\n", __func__);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
		pr_debug("%s warning not initialized yet\n", __func__);
#endif
		goto i2c_fm3_transfer_exit;
	}
	c->msg = &m[0];
	c->msg_i = 0;
	c->msg_n = n;
	c->msg_status = -EBUSY;
	//i2c_fm3_hw_clear(c);
	i2c = FM3_CSIO_REGS[c->bus];
	sla = (c->msg->addr << 1) | ((c->msg->flags & I2C_M_RD)? 1:0);
	ibcr = i2c->IBCR;
	ibsr = i2c->IBSR;
	ssr = i2c->SSR;
	i2c->IBCR = 0;
	i2c->TDR = sla;
	i2c->IBCR =   // 0x85
			IBCR_MSS |  // select master
			IBCR_INTE | // enable interrupt
			IBCR_INT;   // set interrupt
	dprintk("I2C start: ibcr=0x%02x(<-0x%02x) ibsr=0x%02x(<-0x%02x) ssr=0x%02x\n",
			i2c->IBCR, ibcr, i2c->IBSR, ibsr, i2c->SSR);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
	pr_debug("I2C start: ibcr=0x%02x(<-0x%02x) ibsr=0x%02x(<-0x%02x) ssr=0x%02x\n",
			i2c->IBCR, ibcr, i2c->IBSR, ibsr, i2c->SSR);
#endif
    enable_irq(c->irq);
    enable_irq(c->irq+1);
	if (wait_event_timeout(c->wait, c->msg_status != -EBUSY, HZ) == 0) {
		ret = -ETIMEDOUT;
	} else {
		ret = c->msg_status;
		if (!ret) {
			ret = n;
		}
	}
	//disable_irq_nosync(c->irq);
	//disable_irq_nosync(c->irq+1);
	dprintk("I2C done: ret=%d ibcr=0x%02x ibsr=0x%02x ssr=0x%02x\n",
			ret, i2c->IBCR, i2c->IBSR, i2c->SSR);
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
	pr_debug("I2C done: ret=%d ibcr=0x%02x ibsr=0x%02x ssr=0x%02x\n",
			ret, i2c->IBCR, i2c->IBSR, i2c->SSR);
#endif
i2c_fm3_transfer_exit:
	FUNC_EXIT();
	return ret;
}

static unsigned int i2c_fm3_functionality(struct i2c_adapter *a)
{
	//FUNC_ENTER();
	//FUNC_EXIT();
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_fm3_algorithm = {
	.functionality	= i2c_fm3_functionality,
	.master_xfer	= i2c_fm3_transfer,
};

static int __devinit i2c_fm3_probe(struct platform_device *dev)
{
	struct i2c_fm3 *c = NULL;
	struct i2c_fm3_data *d;
	struct resource *regs;
	int bus;
	int irq;
	int ret = 0;
	i2c_fm3_hw_initialized = 0;
	FUNC_ENTER();
	bus = dev->id;
	if (! (0 <= bus && bus <= 8)) {
		dev_err(&dev->dev, "invalid bus number %d\n", bus);
		ret = -ENXIO;
		goto i2c_fm3_probe_err1;
	}
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "invalid IRQ number %d\n", irq);
		ret = irq;
		goto i2c_fm3_probe_err1;
	}
	regs = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&dev->dev, "no register base provided\n");
		ret = -ENXIO;
		goto i2c_fm3_probe_err1;
	}
	c = kzalloc(sizeof(struct i2c_fm3), GFP_KERNEL);
	if (!c) {
		dev_err(&dev->dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto i2c_fm3_probe_err1;
	}
	c->dev = dev;
	c->bus = bus;
	if (!request_mem_region(regs->start, resource_size(regs), regs->name)) {
		dev_err(&dev->dev, "registers already in use\n");
		ret = -ENOMEM;
		goto i2c_fm3_probe_err2;
	}
	c->regs_base = regs->start;
	c->regs_size = resource_size(regs);
	c->regs = ioremap(regs->start, resource_size(regs));
	if (!c->regs) {
		dev_err(&dev->dev, "unable to map registers\n");
		ret = -EINVAL;
		goto i2c_fm3_probe_err3;
	}
	d = (struct i2c_fm3_data *) platform_get_drvdata(dev);
	c->mod = d->ch_mod;
	c->ref_clk = d->ref_clk;
	c->i2c_clk = d->i2c_clk;
	platform_set_drvdata(dev, c);

	c->adap.owner = THIS_MODULE;
	c->adap.nr = bus;
	snprintf(c->adap.name, sizeof(c->adap.name), "i2c_fm3.%u", bus);
	c->adap.algo = &i2c_fm3_algorithm;
	c->adap.algo_data = c;
	c->adap.dev.parent = &dev->dev;
	c->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	if (i2c_add_numbered_adapter(&c->adap)) {
		dev_err(&dev->dev, "unable to add adapter\n");
		ret = -ENXIO;
		goto i2c_fm3_probe_err4;
	}
	init_waitqueue_head(&c->wait);
	ret = i2c_fm3_hw_init(c);
	if (ret) {
		goto i2c_fm3_probe_err5;
	}
	ret = request_irq(irq, i2c_fm3_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq);
		goto i2c_fm3_probe_err5;
	}
	c->irq = irq;
	ret = request_irq(irq+1, i2c_fm3_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request for IRQ %d failed\n", irq+1);
		goto i2c_fm3_probe_err6;
	}
	pr_info("I2C Controller %s ch%d_%d clk=%dKhz at 0x%p irq=%d-%d\n",
			dev_name(&c->adap.dev), c->bus, c->mod, c->i2c_clk/1000, c->regs, c->irq, c->irq+1);
	i2c_fm3_hw_initialized = 1;
	//enable_irq(c->irq);
	//enable_irq(c->irq+1);
	goto i2c_fm3_probe_exit;
	//free_irq(c->irq + 1, c);
i2c_fm3_probe_err6:
	free_irq(c->irq, c);
i2c_fm3_probe_err5:
	i2c_del_adapter(&c->adap);
i2c_fm3_probe_err4:
	iounmap(c->regs);
i2c_fm3_probe_err3:
	release_mem_region(regs->start, resource_size(regs));
i2c_fm3_probe_err2:
	kfree(c);
	platform_set_drvdata(dev, NULL);
i2c_fm3_probe_err1:
i2c_fm3_probe_exit:
#if defined(FM3_I2C_DEBUG_KERNEL_LOG)
	if (ret < 0)
		pr_debug("%s %s ret=%d\n", __func__, dev_name(&dev->dev), ret);
#endif
	FUNC_EXIT();
	return ret;
}

static int __devexit i2c_fm3_remove(struct platform_device *dev)
{
	int ret = 0;
	struct i2c_fm3 *c  = platform_get_drvdata(dev);
	FUNC_ENTER();
	i2c_fm3_hw_release(c);
	platform_set_drvdata(dev, NULL);
	i2c_del_adapter(&c->adap);
	free_irq(c->irq, c);
	free_irq(c->irq + 1, c);
	iounmap(c->regs);
	release_mem_region(c->regs_base, c->regs_size);
	kfree(c);
	dprintk("dev=%s,ret=%d\n", dev_name(&dev->dev), ret);
	FUNC_EXIT();
	return ret;
}

static struct platform_driver i2c_fm3_drv = {
	.probe	= i2c_fm3_probe,
	.remove	= __devexit_p(i2c_fm3_remove),
	.driver = {
		.name = FM3_I2C_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init i2c_fm3_module_init(void)
{
	int ret;
	FUNC_ENTER();
	ret = platform_driver_register(&i2c_fm3_drv);
	if (ret)
		printk(KERN_ERR "%s: probe failed: %d\n", FM3_I2C_DRV_NAME, ret);
	FUNC_EXIT();
	return ret;
}

static void __exit i2c_fm3_module_exit(void)
{
	FUNC_ENTER();
	platform_driver_unregister(&i2c_fm3_drv);
	FUNC_EXIT();
}

module_init(i2c_fm3_module_init);
module_exit(i2c_fm3_module_exit);
MODULE_AUTHOR("Kentaro Sekimoto");
MODULE_DESCRIPTION("Device driver for the I2C controller of FM3");
MODULE_LICENSE("GPL");

