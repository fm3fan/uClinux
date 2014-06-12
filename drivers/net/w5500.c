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
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/spi/spi.h>
#include <asm/setup.h>
#if defined(CONFIG_MACH_FM4)
#include <mach/fm4.h>
#endif
#if defined(CONFIG_MACH_FM3)
#include <mach/fm3.h>
#endif
#include <mach/gpio.h>
#include "w5500.h"

//#define W5500_DEBUG
//#define W5500_DEBUG_FUNC
//#define W5500_DEBUG_SPI
#define W5500_DEBUG_POLL
//#define W5500_DEBUG_KERNEL_LOG

#ifdef W5500_DEBUG
#define dprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

#ifdef W5500_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#define fprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#define	fprintk(fmt, ...)
#endif

#define W5500_DRV_NAME	"w5500_eth"
#define W5500_INFO	KERN_INFO W5500_DRV_NAME
#define W5500_FRAME_MAX_SIZE	1536
#define RX_BUF_SIZE	0x4000
#define RX_OFF_MASK (RX_BUF_SIZE - 1)
#define TX_BUF_SIZE 0x4000
#define TX_OFF_MASK (TX_BUF_SIZE - 1)
#define W5500_SEND_TIMEOUT 1000000
#define INTLEVEL0_COUNT 1
#define INTLEVEL1_COUNT	1
#define DEF_IP_ADDR ((192<<24) + (162<<16) + (1<<8) + 150)
#define DEF_NETMASK ((255<<24) + (255<<16) + (255<<8) + 0)
#define DEF_GATEWAY ((192<<24) + (162<<16) + (1<<8) + 1)
#ifndef TOWORD
#define TOWORD(h,l) ((((unsigned short)h) << 8) + (((unsigned short)l) & 0xff))
#endif

struct w5500_net {
	struct net_device *netdev;
	struct spi_device *spidev;
	struct napi_struct napi;
	struct net_device_stats stat;
	spinlock_t spi_lock;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	struct mutex lock;
	spinlock_t statelock;
	//union w5500_tx_hdr	txh ____cacheline_aligned;
	u32 msg_enable ____cacheline_aligned;
	//u16 tx_space;
	//u8 fid;
	//u16 rc_ier;
	//u16 rc_rxqcr;
	struct mii_if_info	mii;
	//struct w5500_rxctrl	rxctrl;
	struct work_struct	tx_work;
	struct work_struct	irq_work;
	//struct work_struct	rxctrl_work;
	struct sk_buff_head	txq;
	struct spi_message	spi_msg1;
	struct spi_message	spi_msg2;
	struct spi_transfer	spi_xfer1;
	struct spi_transfer	spi_xfer2[2];
	u32 frame_max_size;
	u32 rx_done_idx;
	u32 tx_todo_idx;
	u32 tx_done_idx;
	u32 tx_pending;
	u32 tx_blocked;
	u8 rx_buf[W5500_FRAME_MAX_SIZE + 4];
	u8 tx_buf[W5500_FRAME_MAX_SIZE + 4];
};

#if 0
struct w5500_mac_regs {
	u32 dummy;
};
#endif

static int msg_enable;
static u8 mac_addr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
//static u8 broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#if 0
#define w5500_info(_w5500, _msg...) dev_info(&(_w5500)->spidev->dev, _msg)
#define w5500_warn(_w5500, _msg...) dev_warn(&(_w5500)->spidev->dev, _msg)
#define w5500_dbg(_w5500, _msg...) dev_dbg(&(_w5500)->spidev->dev, _msg)
#define w5500_err(_w5500, _msg...) dev_err(&(_w5500)->spidev->dev, _msg)
#endif

#define INT_LEVEL_LOW	0
#define INT_LEVEL_HIGH	1
#define INT_EDGE_HIGH	1
#define INT_EDGE_LOW	1
#if defined(CONFIG_MACH_FM3)
#if defined(CONFIG_MACH_FM3_CQFM3DUINO)
#define W5500_INTPIN	0x06
#define W5500_INTNO		1
#define	W5500_IRQ		4
#endif
#elif defined(CONFIG_MACH_FM4)
#define W5500_INTPIN	0x40
#define W5500_INTNO		12
#define	W5500_IRQ		55
#else
#endif

void cpu_gpio_int_enable(void)
{
#if defined(CONFIG_MACH_FM3)
	u8 gpio = W5500_INTPIN;
	u8 intno = W5500_INTNO;
	u8 intmd = 1;
	//u32 edge = INT_EDGE_LOW;
	u32 edge = INT_LEVEL_LOW;
	u32 port = GPIO_PORT(gpio);
	u32 mask = GPIO_MASK(gpio);
	GPIO_PCR(port) |= mask;				// pull up
	GPIO_PFR(port) |= mask;				// peripheral
	FM3_GPIO->EPFR06 |= ((intmd+1) << (intno*2));
	FM3_EXTI->ENIR &= ~(1 << intno);
	FM3_EXTI->ELVR = (FM3_EXTI->ELVR & (0xFC << (intno*2))) | (edge << (intno*2));
	FM3_EXTI->EICL &= ~(1 << intno);    // Clear External Interrupt CLear register
	FM3_EXTI->ENIR |= (1 << intno);     // Set ENable Interrupt request Register
#endif
#if defined(CONFIG_MACH_FM4)
	u8 gpio = W5500_INTPIN;
	u8 intno = W5500_INTNO;
	u8 intmd = 1;
	//u32 edge = INT_EDGE_LOW;
	u32 edge = INT_LEVEL_LOW;
	u32 port = GPIO_PORT(gpio);
	u32 mask = GPIO_MASK(gpio);
	GPIO_PCR(port) |= mask;				// pull up
	GPIO_PFR(port) |= mask;				// peripheral
	FM4_GPIO->EPFR06 |= ((intmd+1) << (intno*2));
	FM4_EXTI->ENIR &= ~(1 << intno);
	FM4_EXTI->ELVR = (FM4_EXTI->ELVR & (0xFC << (intno*2))) | (edge << (intno*2));
	FM4_EXTI->EICL &= ~(1 << intno);    // Clear External Interrupt CLear register
	FM4_EXTI->ENIR |= (1 << intno);     // Set ENable Interrupt request Register
#endif
}

void cpu_gpio_int_disable(void)
{
#if defined(CONFIG_MACH_FM3)
	u8 gpio = W5500_INTPIN;
	u8 intno = W5500_INTNO;
	u8 intmd = 1;
	u32 edge = INT_EDGE_LOW;
	u32 port = GPIO_PORT(gpio);
	u32 mask = GPIO_MASK(gpio);
	FM3_EXTI->ENIR &= ~(1 << intno);
	GPIO_PFR(port) &= ~mask;
#endif
#if defined(CONFIG_MACH_FM4)
	u8 gpio = W5500_INTPIN;
	u8 intno = W5500_INTNO;
	u8 intmd = 1;
	u32 edge = INT_EDGE_LOW;
	u32 port = GPIO_PORT(gpio);
	u32 mask = GPIO_MASK(gpio);
	FM4_EXTI->ENIR &= ~(1 << intno);
	GPIO_PFR(port) &= ~mask;
#endif
}

void cpu_gpio_int_clear(void)
{
#if defined(CONFIG_MACH_FM3)
	u8 intno = W5500_INTNO;
	FM3_EXTI->EICL &= ~(1 << intno);
#endif
#if defined(CONFIG_MACH_FM4)
	u8 intno = 12;
	FM4_EXTI->EICL &= ~(1 << intno);
#endif
}

static int w5500_spi_sync(struct w5500_net *w5500, struct spi_message *msg)
{
	int ret;
	spin_lock(&w5500->spi_lock);
	ret = spi_sync(w5500->spidev, msg);
	spin_unlock(&w5500->spi_lock);
	return ret;
}

unsigned char w5500_spi_read_creg(struct w5500_net *w5500, unsigned short addr)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	unsigned char cmd[4];
	unsigned char data[4];
	int ret = 0;
	cmd[0] = (unsigned char)(addr >> 8);
	cmd[1] = (unsigned char)addr;
	cmd[2] = (0x00 << 3) + (0x00 << 2) + 0x00;
	cmd[3] = 0xff;
	msg = &w5500->spi_msg1;
	xfer = &w5500->spi_xfer1;
	xfer->tx_buf = &cmd;
	xfer->rx_buf = &data;
	xfer->len = 4;
	ret = w5500_spi_sync(w5500, msg);
	if (ret < 0)
		pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	if (addr != PHYCFGR)
		dprintk("R creg[0x%02x]=0x%02x\n", addr, data[3]);
#endif
	return data[3];
}

unsigned char w5500_spi_read_sreg(struct w5500_net *w5500, unsigned char regno, unsigned short addr)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	unsigned char cmd[4];
	unsigned char data[4];
	int ret = 0;
	cmd[0] = (unsigned char)(addr >> 8);
	cmd[1] = (unsigned char)addr;
	cmd[2] = ((regno*4 + 0x01) << 3) + (0x00 << 2) + 0x00;
	cmd[3] = 0xff;
	msg = &w5500->spi_msg1;
	xfer = &w5500->spi_xfer1;
	xfer->tx_buf = &cmd;
	xfer->rx_buf = &data;
	xfer->len = 4;
	ret = w5500_spi_sync(w5500, msg);
	//if (ret < 0)
	//	pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	//if (addr != Sn_IR)
		dprintk("R S%d[0x%02x]=0x%02x\n", regno, addr, data[3]);
#endif
	return data[3];
}

void w5500_spi_readbuf(struct w5500_net *w5500, unsigned short addr, unsigned char *rbuf, unsigned short len)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	//u8 *buf;
	int ret = 0;
	//buf = (u8 *)kzalloc(len + 3, GFP_KERNEL);
	//if (!buf) {
	//	dprintk("kzalloc NG\n");
	//	return;
	//}
	w5500->rx_buf[0] = (unsigned char)(addr >> 8);
	w5500->rx_buf[1] = (unsigned char)addr;
	w5500->rx_buf[2] = (0x03 << 3) + (0x00 << 2) + 0x00;
	//memset((void *)w5500->rx_buf+3, 0xff, len);
	msg = &w5500->spi_msg1;
	xfer = (struct spi_transfer *)&w5500->spi_xfer1;
	xfer->tx_buf = w5500->rx_buf;
	xfer->rx_buf = w5500->rx_buf;
	xfer->len = len + 3;
	ret = w5500_spi_sync(w5500, msg);
	memcpy((void *)rbuf, (void *)w5500->rx_buf+3, len);
	//kzfree(buf);
	if (ret < 0)
		pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	dprintk("R rbuf=%p len=%d\n", rbuf, len);
#endif
	return;
}

void w5500_spi_write_creg(struct w5500_net *w5500, unsigned short addr, unsigned char data)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	unsigned char cmd[4];
	int ret = 0;
	cmd[0] = (unsigned char)(addr >> 8);
	cmd[1] = (unsigned char)addr;
	cmd[2] = (0x00 << 3) + (0x01 << 2) + 0x00;
	cmd[3] = data;
	msg = &w5500->spi_msg1;
	xfer = &w5500->spi_xfer1;
	xfer->tx_buf = &cmd;
	xfer->rx_buf = NULL;
	xfer->len = 4;
	ret = w5500_spi_sync(w5500, msg);
	if (ret < 0)
		pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	dprintk("W creg[0x%02x]=0x%02x\n", addr, data);
#endif
	return;
}

void w5500_spi_write_sreg(struct w5500_net *w5500, unsigned char regno, unsigned short addr, unsigned char data)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	unsigned char cmd[4];
	int ret = 0;
	cmd[0] = (unsigned char)(addr >> 8);
	cmd[1] = (unsigned char)addr;
	cmd[2] = ((regno*4 + 0x01) << 3) + (0x01 << 2) + 0x00;
	cmd[3] = data;
	msg = &w5500->spi_msg1;
	xfer = &w5500->spi_xfer1;
	xfer->tx_buf = &cmd;
	xfer->rx_buf = NULL;
	xfer->len = 4;
	ret = w5500_spi_sync(w5500, msg);
	if (ret < 0)
		pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	dprintk("W S%d[0x%02x]=0x%02x\n", regno, addr, data);
#endif
	return;
}

void w5500_spi_writebuf(struct w5500_net *w5500, unsigned short addr, unsigned char *wbuf, unsigned short len)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	//u8 *buf;
	int ret = 0;
	//buf = (u8 *)kzalloc(len + 3, GFP_KERNEL);
	//if (!buf) {
	//	dprintk("kzalloc NG\n");
	//	return;
	//}
	w5500->tx_buf[0] = (unsigned char)(addr >> 8);
	w5500->tx_buf[1] = (unsigned char)addr;
	w5500->tx_buf[2] = (0x02 << 3) + (0x01 << 2) + 0x00;
	memcpy((void *)&w5500->tx_buf[3], (void *)wbuf, len);
	msg = &w5500->spi_msg1;
	xfer = (struct spi_transfer *)&w5500->spi_xfer1;
	xfer->tx_buf = w5500->tx_buf;
	xfer->rx_buf = NULL;
	xfer->len = len + 3;
	ret = w5500_spi_sync(w5500, msg);
	//kzfree(buf);
	if (ret < 0)
		pr_debug("%s: spi_sync() failed\n", __func__);
#if defined(W5500_DEBUG_SPI)
	dprintk("W wbuf=%p len=%d\n", wbuf, len);
#endif
	return;
}

void set_S0TxFsr(struct w5500_net *w5500, unsigned short data)
{
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_FSR0, (unsigned char)(data >> 8));
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_FSR1, (unsigned char)(data & 0xff));
}

void set_S0TxRd(struct w5500_net *w5500, unsigned short data)
{
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_RD0, (unsigned char)(data >> 8));
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_RD1, (unsigned char)(data & 0xff));
}

void set_S0TxWr(struct w5500_net *w5500, unsigned short data)
{
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_WR0, (unsigned char)(data >> 8));
	w5500_spi_write_sreg(w5500, SR0, Sn_TX_WR1, (unsigned char)(data & 0xff));
}

void set_S0RxRd(struct w5500_net *w5500, unsigned short data)
{
	w5500_spi_write_sreg(w5500, SR0, Sn_RX_RD0, (unsigned char)(data >> 8));
	w5500_spi_write_sreg(w5500, SR0, Sn_RX_RD1, (unsigned char)(data & 0xff));
}

void set_S0RxWr(struct w5500_net *w5500, unsigned short data)
{
	w5500_spi_write_sreg(w5500, SR0, Sn_RX_WR0, (unsigned char)(data >> 8));
	w5500_spi_write_sreg(w5500, SR0, Sn_RX_WR1, (unsigned char)(data & 0xff));
}

u16 get_S0TxFsr(struct w5500_net *w5500)
{
	u16 val;
	//u16 val1;
	//do {
		val = w5500_spi_read_sreg(w5500, SR0, Sn_TX_FSR0);
		val = (val << 8) + w5500_spi_read_sreg(w5500, SR0, Sn_TX_FSR1);
	//	if (val != 0) {
	//		val1 = w5500_spi_read_sreg(w5500, SR0, Sn_TX_FSR0);
	//		val1 = val1 << 8 + w5500_spi_read_sreg(w5500, SR0, Sn_TX_FSR1);
	//	}
	//} while (val != val1);
	return val;
}

u16 get_S0TxWr(struct w5500_net *w5500)
{
	u16 val;
	//u16 val1;
	//do {
		val = w5500_spi_read_sreg(w5500, SR0, Sn_TX_WR0);
		val = (val << 8) + w5500_spi_read_sreg(w5500, SR0, Sn_TX_WR1);
	//	if (val != 0) {
	//		val1 = w5500_spi_read_sreg(w5500, SR0, Sn_TX_WR0);
	//		val1 = val1 << 8 + w5500_spi_read_sreg(w5500, SR0, Sn_TX_WR1);
	//	}
	//} while (val != val1);
	return val;
}

u16 get_S0Rsr(struct w5500_net *w5500)
{
	u16 val;
	//u16 val1;
	//do {
		val = w5500_spi_read_sreg(w5500, SR0, Sn_RX_RSR0);
		val = (val << 8) + w5500_spi_read_sreg(w5500, SR0, Sn_RX_RSR1);
	//	if (val != 0) {
	//		val1 = w5500_spi_read_sreg(w5500, SR0, Sn_RX_RSR0);
	//		val1 = val1 << 8 + w5500_spi_read_sreg(w5500, SR0, Sn_RX_RSR1);
	//	}
	//} while (val != val1);
	return val;
}

u16 get_S0Rd(struct w5500_net *w5500)
{
	u16 val;
	//u16 val1;
	//do {
		val = w5500_spi_read_sreg(w5500, SR0, Sn_RX_RD0);
		val = (val << 8) + w5500_spi_read_sreg(w5500, SR0, Sn_RX_RD1);
	//	if (val != 0) {
	//		val1 = w5500_spi_read_sreg(w5500, SR0, Sn_RX_RD0);
	//		val1 = val1 << 8 + w5500_spi_read_sreg(w5500, SR0, Sn_RX_RD1);
	//	}
	//} while (val != val1);
	return val;
}

static void inline w5500_int_init(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	w5500_spi_write_creg(w5500, SIMR, 0);
	FUNC_EXIT();
}

static void inline w5500_int_enable(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_creg(w5500, SIMR);
	w5500_spi_write_creg(w5500, SIMR, value | (1 << SR0));
	FUNC_EXIT();
}

static void inline w5500_int_disable(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_creg(w5500, SIMR);
	w5500_spi_write_creg(w5500, SIMR, value & ~(1 << SR0));
	FUNC_EXIT();
}

static void inline w5500_int_clear(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_creg(w5500, SIR);
	w5500_spi_write_creg(w5500, SIR, value | (1 << SR0));
	FUNC_EXIT();
}

static void inline w5500_rx_int_init(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	w5500_spi_write_sreg(w5500, SR0, Sn_IMR, 0);
	FUNC_EXIT();
}

static void inline w5500_rx_int_enable(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_sreg(w5500, SR0, Sn_IMR);
	w5500_spi_write_sreg(w5500, SR0, Sn_IMR, value | Sn_IR_RECV);
	FUNC_EXIT();
}

static void inline w5500_rx_int_disable(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_sreg(w5500, SR0, Sn_IMR);
	w5500_spi_write_sreg(w5500, SR0, Sn_IMR, value & ~Sn_IR_RECV);
	FUNC_EXIT();
}

static void inline w5500_rx_int_clear(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_sreg(w5500, SR0, Sn_IR);
	w5500_spi_write_sreg(w5500, SR0, Sn_IR, value | Sn_IR_RECV);
	FUNC_EXIT();
}

static int inline w5500_rx_int_check(struct w5500_net *w5500)
{
	u8 value;
	FUNC_ENTER();
	value = w5500_spi_read_sreg(w5500, SR0, Sn_IR);
	//dprintk("S0_IR=0x%02x\n", value);
	FUNC_EXIT();
	return (value & Sn_IR_RECV);
}

static void w5500_hw_stop(struct net_device *dev)
{
	FUNC_ENTER();
	FUNC_EXIT();
}

static int w5500_rx_get(struct net_device *dev, int processed, int budget)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	unsigned char *addr;
	u8 buf[2];
	u16 offset;
	u16 length;
	u32 recvd = 0;
	FUNC_ENTER();
	while (processed < budget) {
		struct sk_buff *skb;
		recvd = get_S0Rsr(w5500);
		if (recvd == 0) {
			processed = 0;
			goto w5500_rx_get_exit;
		}
		skb = netdev_alloc_skb_ip_align(dev, recvd);
		if (unlikely(!skb)) {
			w5500->stat.rx_dropped++;
			goto w5500_rx_get_exit;
		}
		addr = (unsigned char *)skb_put(skb, recvd);
		offset = get_S0Rd(w5500);
		w5500_spi_readbuf(w5500, offset, buf, 2);
		length = TOWORD(buf[0], buf[1]) - 2;
		offset += 2;
		w5500_spi_readbuf(w5500, offset, addr, length);
		printk(KERN_DEBUG "RX off:%04x len:%04x\n", offset, length);
		dprintk("RX off:%04x len:%04x\n", offset, length);
		offset += length;
		set_S0RxRd(w5500, offset);
		w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_RECV);
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
		w5500->stat.rx_packets++;
		w5500->stat.rx_bytes += recvd;
	}
w5500_rx_get_exit:
	FUNC_EXIT();
	return processed;
}

static int w5500_rx_poll(struct napi_struct *napi, int budget)
{
	struct w5500_net *w5500 = container_of(napi, struct w5500_net, napi);
	struct net_device *dev = w5500->netdev;
	unsigned long flags;
	int ret = 0;
	int more;
	FUNC_ENTER();
	do {
		more = 0;
		ret = w5500_rx_get(dev, ret, budget);
		if (!(ret < budget)) {
			// ToDo: enable RIE
			break;
		}
		spin_lock_irqsave(&w5500->rx_lock, flags);
		__napi_complete(napi);
		// ToDo: enable RIE
		more = 1;
		spin_unlock_irqrestore(&w5500->rx_lock, flags);
	} while (more && napi_reschedule(napi));
	FUNC_EXIT();
	return ret;
}

static void w5500_rx_packet(struct net_device *dev)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	struct sk_buff *skb;
	u16 recvd;
	u16 offset;
	u16 length;
	u8 *addr;
	u8 buf[2];
	FUNC_ENTER();
	//while ((recvd = get_S0Rsr(w5500)) != 0) {
	if ((recvd = get_S0Rsr(w5500)) != 0) {
		offset = get_S0Rd(w5500);
		w5500_spi_readbuf(w5500, offset, buf, 2);
		w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_RECV);
		while (w5500_spi_read_sreg(w5500, SR0, Sn_CR));
		length = TOWORD(buf[0], buf[1]);
		if (length == 0) {
			offset += 2;
			set_S0RxRd(w5500, offset);
			//w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_RECV);
			dprintk("RX recvd:%04x off:%04x len:%04x\n", recvd, offset, length);
			goto w5500_rx_packet_exit;
		}
		skb = dev_alloc_skb(length + 2);
		if (skb) {
			skb_reserve(skb, 2);
			addr = (unsigned char *)skb_put(skb, length);
			length -= 2;
			offset += 2;
			w5500_spi_readbuf(w5500, offset, addr, length);
			dprintk("RX recvd:%04x off:%04x len:%04x\n", recvd, offset, length);
			//printk(KERN_DEBUG "RX off:%04x len:%04x\n", offset, length);
			offset += length;
			set_S0RxRd(w5500, offset);
			w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_RECV);
			while (w5500_spi_read_sreg(w5500, SR0, Sn_CR));
			skb->protocol = eth_type_trans(skb, dev);
			//netif_rx(skb);
			netif_rx_ni(skb);
			w5500->stat.rx_packets++;
			w5500->stat.rx_bytes += recvd;
		} else {
			w5500->stat.rx_dropped++;
			printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.\n", dev->name);
			goto w5500_rx_packet_exit;
		}
	}
w5500_rx_packet_exit:
	FUNC_EXIT();
}

static void w5500_irq_work(struct work_struct *work)
{
	struct w5500_net *w5500 = container_of(work, struct w5500_net, irq_work);
	mutex_lock(&w5500->lock);
	//dprintk("*+\n");
	w5500_int_clear(w5500);
	w5500_int_disable(w5500);
	if (w5500_rx_int_check(w5500)) {
		w5500_rx_int_clear(w5500);
		w5500_rx_packet(w5500->netdev);
	}
	w5500_int_enable(w5500);
	mutex_unlock(&w5500->lock);
	cpu_gpio_int_enable();
	//dprintk("*-\n");
	enable_irq(w5500->netdev->irq);
}

static irqreturn_t w5500_irq(int irq, void *dev_id)
{
	struct w5500_net *w5500 = (struct w5500_net *)dev_id;
	FUNC_ENTER();
	//dprintk("+\n");
	disable_irq_nosync(irq);
	cpu_gpio_int_clear();
	cpu_gpio_int_disable();
	schedule_work(&w5500->irq_work);
	//dprintk("-\n");
	FUNC_EXIT();
	return IRQ_HANDLED;
}

static int w5500_net_open(struct net_device *dev)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	int ret = 0;
	FUNC_ENTER();
	mutex_lock(&w5500->lock);
	if (netif_msg_ifup(w5500))
		printk(KERN_DEBUG "opening %s\n", dev->name);
	//ret = w5500_buffers_alloc(dev);
	//if (ret != 0)
	//	goto w5500_net_open_exit;
	napi_enable(&w5500->napi);
	spin_lock_init(&w5500->spi_lock);
	spin_lock_init(&w5500->rx_lock);
	spin_lock_init(&w5500->tx_lock);
	w5500->rx_done_idx = 0;
	w5500->tx_todo_idx = 0;
	w5500->tx_done_idx = 0;
	w5500->tx_pending = 0;
	w5500->tx_blocked = 0;
	netif_start_queue(dev);
	if (netif_msg_ifup(w5500))
		printk(KERN_DEBUG "network device %s up\n", dev->name);
	mutex_unlock(&w5500->lock);
	w5500_rx_int_init(w5500);
	w5500_int_init(w5500);
	w5500_spi_write_creg(w5500, INTLEVEL0, INTLEVEL0_COUNT);
	w5500_spi_write_creg(w5500, INTLEVEL1, INTLEVEL1_COUNT);
	w5500_rx_int_enable(w5500);
	w5500_int_enable(w5500);
	cpu_gpio_int_enable();
w5500_net_open_exit:
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
	FUNC_EXIT();
	return ret;
}

static int w5500_net_close(struct net_device *dev)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	FUNC_ENTER();
	cpu_gpio_int_disable();
	w5500_rx_int_disable(w5500);
	w5500_int_disable(w5500);
	napi_disable(&w5500->napi);
	netif_stop_queue(dev);
	w5500_hw_stop(dev);
	FUNC_EXIT();
	return 0;
}

void w5500_tx_packet(struct w5500_net *w5500, uint8_t *p, u32 size)
{
	u16 offset;
	u16 free_tx_size;
	u16 s;
	volatile int tmo;
	FUNC_ENTER();
	free_tx_size = get_S0TxFsr(w5500);
	if (free_tx_size < size) {
		printk(KERN_DEBUG "short of free tx size\n");
		//eth_reset();
		//eth_reginit();
		return;
	}
	if ((size < 0) || (size > 8192))
		printk(KERN_DEBUG "eth_send:exceed length!\n");
	offset = get_S0TxWr(w5500);
	dprintk("TX off:%04x len:%04x\n", offset, size);
	//printk(KERN_DEBUG "TX off:%04x len:%04x\n", offset, size);
	w5500_spi_writebuf(w5500, offset, (u8 *)p, size);
	offset += (u16)size;
	set_S0TxWr(w5500, offset);
	w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_SEND);
	while (w5500_spi_read_sreg(w5500, SR0, Sn_CR));
	//w5500_spi_write_sreg(w5500, SR0, Sn_IR, Sn_IR_SEND_OK);
	while (1) {
		s = w5500_spi_read_sreg(w5500, SR0, Sn_IR);
		if (s & Sn_IR_SEND_OK) {
			w5500_spi_write_sreg(w5500, SR0, Sn_IR, Sn_IR_SEND_OK);
			break;
		} else if (s & Sn_IR_TIMEOUT) {
			w5500_spi_write_sreg(w5500, SR0, Sn_IR, Sn_IR_TIMEOUT);
			dprintk("TX timeout\n");
			break;
		}
	}
	FUNC_EXIT();
	return;
}

static void w5500_tx_work(struct work_struct *work)
{
	struct w5500_net *w5500 = container_of(work, struct w5500_net, tx_work);
	struct net_device *dev = w5500->netdev;
	struct sk_buff *txb;
	bool last = skb_queue_empty(&w5500->txq);
	FUNC_ENTER();
	mutex_lock(&w5500->lock);
	while (!last) {
		txb = skb_dequeue(&w5500->txq);
		last = skb_queue_empty(&w5500->txq);
		dev->trans_start = jiffies;
		w5500_tx_packet(w5500, txb->data, txb->len);
		w5500->stat.tx_packets++;
		w5500->stat.tx_bytes += txb->len;
		dev_kfree_skb(txb);
	}
	mutex_unlock(&w5500->lock);
	FUNC_EXIT();
}

#define SCHEDULE_WORK

static netdev_tx_t w5500_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	//unsigned needed = calc_txlen(skb->len);
#ifndef SCHEDULE_WORK
	struct sk_buff *txb;
	bool last = skb_queue_empty(&w5500->txq);
#endif
	netdev_tx_t ret = NETDEV_TX_OK;
	if (netif_msg_tx_queued(w5500))
		printk(KERN_DEBUG "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);
	spin_lock(&w5500->statelock);
	skb_queue_tail(&w5500->txq, skb);
	spin_unlock(&w5500->statelock);
#ifdef SCHEDULE_WORK
	schedule_work(&w5500->tx_work);
#else
	mutex_lock(&w5500->lock);
	while (!last) {
		txb = skb_dequeue(&w5500->txq);
		last = skb_queue_empty(&w5500->txq);
		dev->trans_start = jiffies;
		w5500_tx_packet(w5500, txb->data, txb->len);
		w5500->stat.tx_packets++;
		w5500->stat.tx_bytes += txb->len;
		dev_kfree_skb(txb);
	}
	mutex_unlock(&w5500->lock);
#endif
	return ret;
}

static struct net_device_stats *w5500_net_get_stats(struct net_device *dev)
{
	struct w5500_net *w5500 = netdev_priv(dev);
	FUNC_ENTER();
	if (netif_running(dev)) {
	}
	FUNC_EXIT();
	return &w5500->stat;
}

static int w5500_net_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	//struct w5500_net *w5500 = netdev_priv(dev);
	int ret = 0;
	FUNC_ENTER();
	if (!netif_running(dev)) {
		ret = -EINVAL;
		goto w5500_net_ioctl_exit;
	}
	//ret = phy_mii_ioctl(phydev, if_mii(ifr), cmd);
w5500_net_ioctl_exit:
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	FUNC_EXIT();
	return ret;
}

int w5500_validate_addr(struct net_device *dev)
{
	int ret;
	FUNC_ENTER();
	ret = eth_validate_addr(dev);
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	FUNC_EXIT();
	return ret;
}

int w5500_change_mtu(struct net_device *dev, int new_mtu)
{
	int ret;
	FUNC_ENTER();
	ret = eth_change_mtu(dev, new_mtu);
	FUNC_EXIT();
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	return ret;
}

int w5500_mac_addr(struct net_device *dev, void *p)
{
	int ret;
	FUNC_ENTER();
	ret = eth_mac_addr(dev, p);
	FUNC_EXIT();
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	return ret;
}

static const struct net_device_ops	w5500_netdrv_ops = {
	.ndo_open				= w5500_net_open,
	.ndo_stop				= w5500_net_close,
	.ndo_start_xmit			= w5500_start_xmit,
	.ndo_get_stats			= w5500_net_get_stats,
	.ndo_do_ioctl			= w5500_net_ioctl,
	.ndo_validate_addr		= w5500_validate_addr,
	.ndo_change_mtu			= w5500_change_mtu,
	.ndo_set_mac_address	= w5500_mac_addr,
};

static int w5500_hw_init(struct w5500_net *w5500)
{
	int i;
	unsigned char v;
	int ret = 0;
	FUNC_ENTER();
	i = 0;
CHIP_RST_LP:
	if (i++ > 10) {
		pr_debug("CHIP RESET failed...\n");
		ret = -1;
		goto w5500_phy_init_exit;
	}
	w5500_spi_write_creg(w5500, MR, MR_RST);
	mdelay(200);	/* wait for 200ms */
	if ((w5500_spi_read_creg(w5500, MR) & MR_RST) !=  0x00) {
		goto CHIP_RST_LP;
	}
	v = w5500_spi_read_creg(w5500, PHYCFGR) | PHYCFGR_OPMDC_ALLA;
	w5500_spi_write_creg(w5500, PHYCFGR, v);
	v = w5500_spi_read_creg(w5500, PHYCFGR) & PHYCFGR_RST;
	w5500_spi_write_creg(w5500, PHYCFGR, v);
	v = w5500_spi_read_creg(w5500, PHYCFGR) | ~PHYCFGR_RST;
	w5500_spi_write_creg(w5500, PHYCFGR, v);
	i = 0;
PHY_NEG_LP:
	if (i++ > 100) {
		pr_debug("PHY NEG failed...\n");
		ret = -1;
		goto w5500_phy_init_exit;
	}
	mdelay(100);	/* wait for 100ms */
	if ((w5500_spi_read_creg(w5500, PHYCFGR) & 0x07) !=  0x07) {
		goto PHY_NEG_LP;
	}
w5500_phy_init_exit:
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	dprintk("PHYCFGR=0x%01x\n", w5500_spi_read_creg(w5500, PHYCFGR));
	FUNC_EXIT();
	return ret;
}

static int w5500_eth_conf(struct w5500_net *w5500, char mac_addr[], u32 ip_addr, u32 netmask, u32 gateway)
{
	w5500_spi_write_creg(w5500, SHAR0, mac_addr[0]);
	w5500_spi_write_creg(w5500, SHAR1, mac_addr[1]);
	w5500_spi_write_creg(w5500, SHAR2, mac_addr[2]);
	w5500_spi_write_creg(w5500, SHAR3, mac_addr[3]);
	w5500_spi_write_creg(w5500, SHAR4, mac_addr[4]);
	w5500_spi_write_creg(w5500, SHAR5, mac_addr[5]);

	w5500_spi_write_creg(w5500, SIPR0, (u8)(ip_addr >> 24));
	w5500_spi_write_creg(w5500, SIPR1, (u8)(ip_addr >> 16));
	w5500_spi_write_creg(w5500, SIPR2, (u8)(ip_addr >> 8));
	w5500_spi_write_creg(w5500, SIPR3, (u8)(ip_addr & 0xff));

	w5500_spi_write_creg(w5500, SUBR0, (u8)(netmask >> 24));
	w5500_spi_write_creg(w5500, SUBR1, (u8)(netmask >> 16));
	w5500_spi_write_creg(w5500, SUBR2, (u8)(netmask >> 8));
	w5500_spi_write_creg(w5500, SUBR3, (u8)(netmask & 0xff));

	w5500_spi_write_creg(w5500, GAR0, (u8)(gateway >> 24));
	w5500_spi_write_creg(w5500, GAR1, (u8)(gateway >> 16));
	w5500_spi_write_creg(w5500, GAR2, (u8)(gateway >> 8));
	w5500_spi_write_creg(w5500, GAR3, (u8)(gateway & 0xff));

	return 0;
}

static int w5500_reg_init (struct w5500_net *w5500)
{
	int i;
	int ret = 0;

	FUNC_ENTER();
	w5500_spi_write_creg(w5500, IMR, 0x0);	// no interrupt
	w5500_spi_write_sreg(w5500, SR0, Sn_RXBUF_SIZE, RX_BUF_SIZE >> 10);
	w5500_spi_write_sreg(w5500, SR0, Sn_TXBUF_SIZE, TX_BUF_SIZE >> 10);
	for (i = 1; i < SR_NUM; i++) {
		w5500_spi_write_sreg(w5500, i, Sn_RXBUF_SIZE, 0);
		w5500_spi_write_sreg(w5500, i, Sn_TXBUF_SIZE, 0);
	}
	// channel 0 : MACRAW mode
	i = 0;
MACRAW:
	if (i++ > 10) {
		pr_debug("MACRAW open failed...\n");
		ret = -1;
		goto w5500_hw_init_exit;
	}
	w5500_spi_write_sreg(w5500, SR0, Sn_MR, 0x80 | Sn_MR_MACRAW);
	w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_OPEN);
	mdelay(10);      // 10ms
	//DPRINTF("Sn_SSR = 0x%04x\n", IINCHIP_READ(Sn_SSR(0)));
	if((w5500_spi_read_sreg(w5500, SR0, Sn_SR) & Sn_SR_MACRAW) != Sn_SR_MACRAW) {
		w5500_spi_write_sreg(w5500, SR0, Sn_CR, Sn_CR_CLOSE);
		goto MACRAW;
	}
w5500_hw_init_exit:
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	FUNC_EXIT();
	return ret;
}

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/
static int __devinit w5500_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct w5500_net *w5500;
	char *p;
	int ret = 0;
	FUNC_ENTER();
	ndev = alloc_etherdev(sizeof(struct w5500_net));
	if (!ndev) {
		dev_err(&spi->dev, "failed to alloc ethernet device\n");
		ret = -ENOMEM;
		goto w5500_probe_exit;
	}
	spi->bits_per_word = 8;
	spi->chip_select = 0x07;
	spi->max_speed_hz = 8000000;
	spi->irq = W5500_IRQ;
	w5500 = (struct w5500_net *)netdev_priv(ndev);
	w5500->netdev = ndev;
	w5500->spidev = spi;
	w5500->frame_max_size = W5500_FRAME_MAX_SIZE;
	mutex_init(&w5500->lock);
	spin_lock_init(&w5500->statelock);
	INIT_WORK(&w5500->tx_work, w5500_tx_work);
	INIT_WORK(&w5500->irq_work, w5500_irq_work);
	//INIT_WORK(&w5500->rxctrl_work, w5500_rxctrl_work);
	spi_message_init(&w5500->spi_msg1);
	spi_message_add_tail(&w5500->spi_xfer1, &w5500->spi_msg1);
	spi_message_init(&w5500->spi_msg2);
	spi_message_add_tail(&w5500->spi_xfer2[0], &w5500->spi_msg2);
	spi_message_add_tail(&w5500->spi_xfer2[1], &w5500->spi_msg2);
	/* setup mii state */
	w5500->mii.dev = ndev;
	w5500->mii.phy_id = 1,
	w5500->mii.phy_id_mask = 1;
	w5500->mii.reg_num_mask	= 0xf;
	//w5500->mii.mdio_read	= w5500_phy_read;
	//w5500->mii.mdio_write	= w5500_phy_write;
	dev_info(&spi->dev, "message enable is %d\n", msg_enable);
	w5500->msg_enable = netif_msg_init(msg_enable, (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK));
	skb_queue_head_init(&w5500->txq);
	//SET_ETHTOOL_OPS(ndev, &w5500_ethtool_ops);
	SET_NETDEV_DEV(ndev, &spi->dev);
	p = strnstr(boot_command_line, "ethaddr=", COMMAND_LINE_SIZE);
	if (p) {
		char ethaddr[18];
		int i;
		memcpy(ethaddr, &p[strlen("ethaddr=")], sizeof(ethaddr));
		p = ethaddr;
		for (i = 0; i < ETH_ALEN; i++) {
			ndev->dev_addr[i] = (simple_strtol(p, &p, 16) << 0) |
					   (simple_strtol(p, &p, 16) << 4);
			p++; /* skip ":" in  ethaddr */
		}
	} else {
		memcpy(ndev->dev_addr, &mac_addr, ETH_ALEN);
	}
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		printk(KERN_DEBUG ": ethernet address is not set or invalid, using random.\n");
		random_ether_addr(ndev->dev_addr);
	}
	dev_set_drvdata(&spi->dev, w5500);
	ndev->if_port = IF_PORT_100BASET;
	ndev->netdev_ops = &w5500_netdrv_ops;
	ndev->irq = spi->irq;
	w5500_hw_init(w5500);
	w5500_reg_init(w5500);
	w5500_eth_conf(w5500, ndev->dev_addr, DEF_IP_ADDR, DEF_NETMASK, DEF_GATEWAY);
	netif_napi_add(ndev, &w5500->napi, w5500_rx_poll, 64);
	ret = request_irq(spi->irq, w5500_irq, IRQF_TRIGGER_LOW, ndev->name, w5500);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to get irq\n");
		goto err_irq;
	}
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&spi->dev, "failed to register network device\n");
		goto err_netdev;
	}
	enable_irq(ndev->irq);
	goto w5500_probe_exit;
err_netdev:
	free_irq(ndev->irq, ndev);
//err_id:
err_irq:
	free_netdev(ndev);
w5500_probe_exit:
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	FUNC_EXIT();
	return ret;
}

static int w5500_remove(struct spi_device *spi)
{
	struct w5500_net *w5500 = dev_get_drvdata(&spi->dev);
	FUNC_ENTER();
	disable_irq(spi->irq);
	if (netif_msg_drv(w5500))
		dev_info(&spi->dev, "remove");
	unregister_netdev(w5500->netdev);
	free_irq(spi->irq, w5500);
	free_netdev(w5500->netdev);
	FUNC_EXIT();
	return 0;
}

static struct spi_driver w5500_driver = {
	.driver = {
		.name = W5500_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = w5500_probe,
	.remove = __devexit_p(w5500_remove),
};

static int __init w5500_drv_init(void)
{
	int ret;
	FUNC_ENTER();
	ret = spi_register_driver(&w5500_driver);
#if defined(W5500_DEBUG_KERNEL_LOG)
	printk(KERN_DEBUG "%s ret=%d\n", __func__, ret);
#endif
	FUNC_EXIT();
	return ret;
}

static void __exit w5500_drv_exit(void)
{
	FUNC_ENTER();
	spi_unregister_driver(&w5500_driver);
	FUNC_EXIT();
}

module_init(w5500_drv_init);
module_exit(w5500_drv_exit);

MODULE_ALIAS("spi:w5500_eth");
MODULE_DESCRIPTION("W5500 MAC driver");
MODULE_AUTHOR("Kentaro Sekimoto");
MODULE_LICENSE("GPL");
module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, 31=all)");
