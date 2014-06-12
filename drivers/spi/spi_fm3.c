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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_fm3.h>
#include <mach/platform.h>
#include <mach/fm3.h>
#include <mach/gpio.h>
#include <mach/spi.h>

void fm3_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int v);
void fm3_mfs_set_mode(uint32_t mode);


//#define FM3_SPI_DEBUG_FUNC
//#define FM3_SPI_DEBUG

#ifdef FM3_SPI_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#endif

#ifdef FM3_SPI_DEBUG
#define dprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

/*
 * Polled or interrupt driven PIO mode.
 */
#if 1
#define	CONFIG_SPI_FM3_POLLED
#endif

#define DEF_BAUDRATE	8000000    // 1MHz, unit=Hz

struct fm3_spi_regs
{
	volatile uint8_t SMR;
	volatile uint8_t SCR;
	volatile uint8_t RESERVED0[2];
	volatile uint8_t ESCR;
	volatile uint8_t SSR;
	volatile uint8_t RESERVED1[2];
	union {
		volatile uint16_t RDR;
		volatile uint16_t TDR;
	};
	volatile uint8_t RESERVED2[2];
	volatile uint16_t BGR;
};

static struct fm3_spi_regs *FM3_CSIO_REGS[] = {
	(struct fm3_spi_regs  *)FM3_MFS0_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS1_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS2_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS3_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS4_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS5_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS6_CSIO_BASE,
	(struct fm3_spi_regs  *)FM3_MFS7_CSIO_BASE
};

struct spi_fm3 {
	int bus;							/* Bus (ID): channel number */
	int mod;							/* Mode: channel mode */
	void * __iomem regs;				/* Registers base */
	unsigned int speed_hz;				/* Max clock rate */
	unsigned char stopping;				/* Is being stopped? */
	spinlock_t lock;					/* Exclusive access */
	struct list_head queue;				/* Message Q */
	struct work_struct work;			/* Work Q */
	struct workqueue_struct *workqueue;	/* Work Q */
#if !defined(CONFIG_SPI_FM3_POLLED)
	wait_queue_head_t wait;				/* Wait queue */
	int irq;							/* IRQ # */
#endif
	struct spi_device *slave;			/* Current SPI slave */
	struct spi_message *msg;			/* SPI message */
	volatile int xfer_status;			/* Xfer status */
	int len;							/* Xfer len */
	int wb;								/* Xfer width */
#if 0
	struct spi_transfer *tx_t;			/* Cur Tx xfer */
	struct spi_transfer *rx_t;			/* Cur Rx xfer */
	int tx_l;							/* Tx len */
	int rx_l;							/* Rx len */
	int tx_i;							/* Cur Tx index */
	int rx_i;							/* Cur Rx index */
	int ti;								/* Tx count */
	int ri;								/* Rx count */
#endif
};

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

static inline int spi_fm3_hw_cs_max(struct spi_fm3 *c)
{
	int ret = 8;
	FUNC_ENTER();
	dprintk("bus=%d,ret=%d\n", c->bus, ret);
	FUNC_EXIT();
	return ret;
}

static int spi_fm3_hw_init(struct spi_fm3 *c)
{
	int ret = 0;
	struct fm3_spi_regs *spi;
	FUNC_ENTER();
	if (c->bus < spi_fm3_hw_cs_max(c)) {
		fm3_mfs_set_mode(c->bus*3 + c->mod);
		spi = FM3_CSIO_REGS[c->bus];
		spi->SCR = SCR_UPCL;        // Initialize
		spi->SMR |= (SMR_MD_SIO     // SPI mode
					| SMR_SCINV     // Down edge(transfer)/Up edge(receive)
					| SMR_BDS       // MBS first
					| SMR_SCKE      // CLK enable
					| SMR_SOE);     // Data output enable
		if (c->speed_hz)
			spi->BGR = (uint16_t)((c->speed_hz / DEF_BAUDRATE) - 1);
		spi->ESCR = (ESCR_DATABITS_8);  // 8 bit
		spi->SCR |= (SCR_SPI |      // SPI mode
					SCR_RXE |       // Receive enable
					SCR_TXE);       // Transfer enable
	} else
		ret = -1;
	FUNC_EXIT();
	return ret;
}

static inline int spi_fm3_hw_cs_set(struct spi_fm3 *c, int n, int activate)
{
	int ret = 0;
	FUNC_ENTER();
	fm3_gpio_set_value((struct gpio_chip *)NULL, n, !activate);
	dprintk("bus=%d,cs=%d,b=%d,ret=%d\n", c->bus, n, activate, ret);
	FUNC_EXIT();
	return ret;
}

static inline int spi_fm3_hw_clk_set(struct spi_fm3 *c, unsigned int spd)
{
	int ret = 0;
	struct fm3_spi_regs *spi;
	FUNC_ENTER();
	if (c->bus <spi_fm3_hw_cs_max(c)) {
		spi = FM3_CSIO_REGS[c->bus];
		if (c->speed_hz)
			spi->BGR = (uint16_t)((c->speed_hz / spd) - 1);
	} else
		ret = -1;
	dprintk("bus=%d spd=%dHz ret=%d\n", c->bus, spd, ret);
	FUNC_EXIT();
	return ret;
}

static inline int spi_fm3_hw_bt_check(struct spi_fm3 *c, int bt)
{
	int ret = (8 == bt) ? 0 : 1;
	FUNC_ENTER();
	dprintk("bus=%d bt=%d ret=%d\n", c->bus, bt, ret);
	FUNC_EXIT();
	return ret;
}

static inline int spi_fm3_hw_bt_set(struct spi_fm3 *c, int bt)
{
	FUNC_ENTER();
	FUNC_EXIT();
	return 0;
}

static inline int spi_fm3_hw_mode_set(struct spi_fm3 *c, unsigned int mode)
{
	FUNC_ENTER();
	FUNC_EXIT();
	return 0;
}

static int spi_fm3_prepare_for_slave(struct spi_fm3 *c, struct spi_device *s)
{
	int ret = 0;
	int spd;
	FUNC_ENTER();
	if (spi_fm3_hw_bt_set(c, s->bits_per_word)) {
		dev_err(&c->slave->dev, "unsupported frame size: %d\n", s->bits_per_word);
		ret = -EINVAL;
		goto spi_fm3_prepare_for_slave_exit;
	}
	if (spi_fm3_hw_clk_set(c, spd = min(s->max_speed_hz, c->speed_hz))) {
		dev_err(&c->slave->dev, "slave rate too low: %d\n", spd);
		ret = -EINVAL;
		goto spi_fm3_prepare_for_slave_exit;
	}
	if (spi_fm3_hw_mode_set(c, s->mode)) {
		dev_err(&c->slave->dev, "unsupported mode: %x\n", s->mode);
		ret = -EINVAL;
		goto spi_fm3_prepare_for_slave_exit;
	}

spi_fm3_prepare_for_slave_exit:
	dprintk("slv=%s,ret=%d\n", dev_name(&c->slave->dev), ret);
	FUNC_EXIT();
	return ret;
}

static void spi_fm3_capture_slave(struct spi_fm3 *c, struct spi_device *s)
{
	struct spi_fm3_slave *v = s->controller_data;
	FUNC_ENTER();
	if (spi_fm3_hw_cs_set(c, v->cs_gpio, 1)) {
		dev_err(&c->slave->dev, "incorrect chip select: %d\n", s->chip_select);
	}
	FUNC_EXIT();
}

static void spi_fm3_release_slave(struct spi_fm3 *c, struct spi_device *s)
{
	struct spi_fm3_slave *v = s->controller_data;
	FUNC_ENTER();
	if (spi_fm3_hw_cs_set(c, v->cs_gpio, 0)) {
		dev_err(&c->slave->dev, "incorrect chip select: %d\n", s->chip_select);
	}
	FUNC_EXIT();
}

#if defined(CONFIG_SPI_FM3_POLLED)

static int spi_fm3_pio_polled(struct spi_fm3 *c, struct spi_device *s, int *rlen)
{
	struct spi_transfer *t;
	struct fm3_spi_regs *spi;
	int ret = 0;
	int i = 0;
	int len = 0;
	FUNC_ENTER();
	spi = FM3_CSIO_REGS[c->bus];
	list_for_each_entry(t, &c->msg->transfers, transfer_list) {
		u8 v, *tp, *rp;
		tp = (u8 *)t->tx_buf;
		rp = (u8 *)t->rx_buf;
		for (;i < t->len; i++) {
			v = (tp? *tp++ : 0xff);
			while ((spi->SSR & SSR_TDRE) == 0) ;
			spi->TDR = (u8)v;
			while ((spi->SSR & SSR_RDRF) == 0) ;
			v = (u8)spi->RDR;
			if (rp)
				*rp++ = v;
		}
		len += t->len;
	}
	*rlen = len;
spi_fm3_pio_polled_exit:
	dprintk("msg=%p,len=%d,rlen=%d,ret=%d\n", c->msg, c->len, *rlen, ret);
	FUNC_EXIT();
	return ret;
}

#else

static irqreturn_t spi_fm3_irq(int irq, void *dev_id)
{
	FUNC_ENTER();
	FUNC_EXIT();
	return IRQ_HANDLED;
}

static int spi_fm3_pio_interrupted(struct spi_fm3 *c, struct spi_device *s, int *rlen)
{
	int ret = 0;
	FUNC_ENTER();
	FUNC_EXIT();
	return ret;
}

#endif

static int spi_fm3_handle_message(struct spi_fm3 *c, struct spi_message *msg)
{
	struct spi_device *s = msg->spi;
	int rlen = 0;
	int ret = 0;
	FUNC_ENTER();
	if (c->slave != s) {
		c->slave = s;
		ret = spi_fm3_prepare_for_slave(c, s);
		if (ret) {
			c->slave = NULL;
			goto spi_fm3_handle_message_exit;
		}
	}
	spi_fm3_capture_slave(c, s);
	c->msg = msg;
#if defined(CONFIG_SPI_FM3_POLLED)
	ret = spi_fm3_pio_polled(c, s, &rlen);
#else
	ret = spi_fm3_pio_interrupted(c, s, &rlen);
#endif
	if (ret) {
		goto spi_fm3_handle_message_exit;
	}
	msg->actual_length = rlen;

spi_fm3_handle_message_exit:
	spi_fm3_release_slave(c, s);
	dprintk("ret=%d rlen=%d\n", ret, rlen);
	FUNC_EXIT();
	return ret;
}

static void spi_fm3_handle(struct work_struct *w)
{
	struct spi_message *msg;
	struct spi_fm3 *c = container_of(w, struct spi_fm3, work);
	unsigned long f = 0;

	FUNC_ENTER();
	dprintk("0 head=%p head.next=%p head.prev=%p\n", &c->queue, c->queue.next, c->queue.prev);
	spin_lock_irqsave(&c->lock, f);
	while (!c->xfer_status && !list_empty(&c->queue)) {
		msg = container_of(c->queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);
		dprintk("1 head=%p head.next=%p head.prev=%p\n", &c->queue, c->queue.next, c->queue.prev);
		spin_unlock_irqrestore(&c->lock, f);
		msg->status = spi_fm3_handle_message(c, msg);
		msg->complete(msg->context);
		spin_lock_irqsave(&c->lock, f);
		//dprintk("2 head=%p head.next=%p head.prev=%p\n", &c->queue, c->queue.next, c->queue.prev);
		//dprintk("xfer_status=%d list_empty=%d\n", c->xfer_status, list_empty(&c->queue));
	}
	spin_unlock_irqrestore(&c->lock, f);
	FUNC_EXIT();
}

static int spi_fm3_setup(struct spi_device *s)
{
	int ret = 0;
	struct spi_fm3 *c = spi_master_get_devdata(s->master);
	FUNC_ENTER();
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto spi_fm3_setup_exit;
	}
	if (spi_fm3_hw_bt_check(c, s->bits_per_word)) {
		dev_err(&s->dev, "unsupported bits per word %d\n", s->bits_per_word);
		ret = -EINVAL;
		goto spi_fm3_setup_exit;
	}
	spi_fm3_release_slave(c, s);
	dprintk("slv=%s,spd=%d,cs=%d,bt=%d,md=0x%x,ret=%d\n", dev_name(&s->dev), s->max_speed_hz,
			((struct spi_fm3_slave *)s->controller_data)->cs_gpio, s->bits_per_word, s->mode, ret);
spi_fm3_setup_exit:
	FUNC_EXIT();
	return ret;
}

static void spi_fm3_cleanup(struct spi_device *s)
{
	FUNC_ENTER();
	FUNC_EXIT();
}

static int spi_fm3_transfer(struct spi_device *s, struct spi_message *msg)
{
	int ret = 0;
	FUNC_ENTER();
	struct spi_fm3 *c = spi_master_get_devdata(s->master);
	unsigned long f;
	if (c->stopping) {
		ret = -ESHUTDOWN;
		goto spi_fm3_transfer_exit;
	}
	if (unlikely(list_empty(&msg->transfers))) {
		ret = -EINVAL;
		goto spi_fm3_transfer_exit;
	}
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;
	spin_lock_irqsave(&c->lock, f);
	dprintk("&msg->queue=%p &c->queue=%p\n", &msg->queue, &c->queue);
	list_add_tail(&msg->queue, &c->queue);
	//queue_work(c->workqueue, &c->work);
	while (!c->xfer_status && !list_empty(&c->queue)) {
		msg = container_of(c->queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);
		//dprintk("1 head=%p head.next=%p head.prev=%p\n", &c->queue, c->queue.next, c->queue.prev);
		//spin_unlock_irqrestore(&c->lock, f);
		msg->status = spi_fm3_handle_message(c, msg);
		msg->complete(msg->context);
		//spin_lock_irqsave(&c->lock, f);
		//dprintk("2 head=%p head.next=%p head.prev=%p\n", &c->queue, c->queue.next, c->queue.prev);
		//dprintk("xfer_status=%d list_empty=%d\n", c->xfer_status, list_empty(&c->queue));
	}
	spin_unlock_irqrestore(&c->lock, f);
spi_fm3_transfer_exit:
	FUNC_EXIT();
	return ret;
}

static int __devinit spi_fm3_probe(struct platform_device *dev)
{
	struct spi_master *m = NULL;
	struct spi_fm3 *c = NULL;
	struct spi_fm3_data *data = NULL;
	struct resource *regs;
	int bus;
	int irq;
	int ret = 0;
	FUNC_ENTER();
#if 1
	bus = dev->id;
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "invalid IRQ %d for SPI controller %d\n", irq, bus);
		ret = irq;
		goto spi_fm3_probe_exit;
	}
	regs = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&dev->dev, "no register base for SPI controller %d\n", bus);
		ret = -ENXIO;
		goto spi_fm3_probe_exit;
	}
	m = spi_alloc_master(&dev->dev, sizeof(struct spi_fm3));
	if (!m) {
		dev_err(&dev->dev, "unable to allocate master for SPI controller %d\n", bus);
		ret = -ENOMEM;
		goto spi_fm3_probe_exit;
	}
	c = spi_master_get_devdata(m);
	m->bus_num = bus;
	c->bus = bus;
	c->regs = ioremap(regs->start, resource_size(regs));
	if (!c->regs) {
		dev_err(&dev->dev, "unable to map registers for SPI controller %d, base=%08x\n", bus, regs->start);
		ret = -EINVAL;
		goto spi_fm3_probe_exit;
	}
#if !defined(CONFIG_SPI_FM3_POLLED)
	ret = request_irq(irq, spi_fm3_irq, 0, dev_name(&dev->dev), c);
	if (ret) {
		dev_err(&dev->dev, "request irq %d failed for SPI controller %d\n", irq, bus);
		goto spi_fm3_probe_exit;
	}
	c->irq = irq;
	init_waitqueue_head(&c->wait);
#endif
	c->workqueue = create_singlethread_workqueue(dev_name(&dev->dev));
	if (!c->workqueue) {
		dev_err(&dev->dev, "unable to create workqueue for SPI controller %d\n", bus);
		ret = -ENXIO;
		goto spi_fm3_probe_exit;
	}
	INIT_WORK(&c->work, spi_fm3_handle);
	INIT_LIST_HEAD(&c->queue);
	spin_lock_init(&c->lock);
	c->xfer_status = 0;
	data = (struct spi_fm3_data *)platform_get_drvdata(dev);
	c->speed_hz = data->ref_clk;
	c->mod = data->ch_mod;
	if (spi_fm3_hw_init(c)) {
		dev_err(&dev->dev, "unable to initialize hardware for SPI controller %d\n", bus);
		ret = -ENXIO;
		goto spi_fm3_probe_exit;
	}
	m->mode_bits = SPI_CPOL | SPI_CPHA;
	m->num_chipselect = spi_fm3_hw_cs_max(c);
	m->setup = spi_fm3_setup;
	m->cleanup = spi_fm3_cleanup;
	m->transfer = spi_fm3_transfer;
	c->slave = NULL;
	c->stopping = 0;
	ret = spi_register_master(m);
	if (ret) {
		dev_err(&dev->dev, "unable to register master for SPI controller %d\n", bus);
		goto spi_fm3_probe_exit;
	}
	platform_set_drvdata(dev, m);
#if !defined(CONFIG_SPI_FM3_POLLED)
	dev_info(&dev->dev, "SPI Controller %d-%d at 0x%p irq=%d-%d\n", m->bus_num, c->mod, c->regs, c->irq, c->irq+1);
#else
	dev_info(&dev->dev, "SPI Controller %d-%d at 0x%p no irq\n", m->bus_num, c->mod, c->regs);
#endif
#endif
spi_fm3_probe_exit:
	FUNC_EXIT();
	return ret;
}

static int __devexit spi_fm3_remove(struct platform_device *dev)
{
	int ret = 0;
	FUNC_ENTER();
	FUNC_EXIT();
	return ret;
}

static struct platform_driver spi_fm3_drv = {
	.probe	= spi_fm3_probe,
	.remove	= __devexit_p(spi_fm3_remove),
	.driver = {
		.name = FM3_CSIO_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init spi_fm3_module_init(void)
{
	int ret;
	FUNC_ENTER();
	ret = platform_driver_register(&spi_fm3_drv);
	FUNC_EXIT();
	return ret;
}

static void __exit spi_fm3_module_exit(void)
{
	FUNC_ENTER();
	platform_driver_unregister(&spi_fm3_drv);
	FUNC_EXIT();
}

module_init(spi_fm3_module_init);
module_exit(spi_fm3_module_exit);
MODULE_AUTHOR("Kentaro Sekimoto");
MODULE_DESCRIPTION("Device driver for the FM3 SPI controller");
MODULE_LICENSE("GPL");
