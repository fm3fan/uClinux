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

#undef	VERBOSE
#undef	PACKET_TRACE
//#define FM3_USB_DEBUG
//#define FM3_USB_DEBUG_FUNC
//#define FM3_USB_DEBUG_INT
//#define FM3_USB_DEBUG_PACKET

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/fm3_usb.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>

#include <mach/fm3.h>
#include <mach/usb.h>

#include "../core/hcd.h"
#include "fm3_hcd.h"

#ifdef FM3_USB_DEBUG_API_FUNC
#define API_FUNC_ENTER()	printk("%s enter\n", __func__)
#define API_FUNC_EXIT()		printk("%s exit\n", __func__)
#else
#define API_FUNC_ENTER()
#define API_FUNC_EXIT()
#endif

#ifdef FM3_USB_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#endif

#ifdef FM3_USB_DEBUG
#define dprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

MODULE_DESCRIPTION("FM3 USB Host Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fm3-hcd");
#define DRIVER_VERSION	"29 Aprl 2014"
//#ifndef DEBUG
//#define	STUB_DEBUG_FILE
//#endif
//#define	DISABLE_ISO

#define MIN_JIFFIES	((msecs_to_jiffies(2) > 1) ? msecs_to_jiffies(2) : 2)

static const char hcd_name[] = FM3_USB_DRV_NAME;
static volatile int f_busreset_done = 0;
static volatile int f_device_connect = 0;
static volatile int transfer_speed = 1;
static volatile int transfer_busy = 0;

/*-------------------------------------------------------------------------*/

static inline void __NOP(void)
{
	__asm volatile ("nop");
}

static void fm3_usbh_clock_init(void)
{
	//bFM3_CLK_GATING_CKEN2_USBCK0 = 1;	/* only for FM3 */
    bFM3_USBETHERNETCLK_UCCR_UCEN1 = 0;               /* disable USB clock */
    while (bFM3_USBETHERNETCLK_UCCR_UCEN1 != 0) ;     /* wait for USB clock stop */
    bFM3_USBETHERNETCLK_UPCR1_UPLLEN = 0;             /* disable USB-PLL clock */
    bFM3_USBETHERNETCLK_UCCR_UCSEL1 = 1;              /* select PLL macro clock */
    bFM3_USBETHERNETCLK_UPCR1_UPINC = 0;              /* select main clock as input clock */
    /* select clock stabilization time */
    FM3_USBETHERNETCLK->UPCR2 = UPCR2_UPOWT_INIT_VALUE;
    /* USB-PLL=Fin*N/K -> 96MHz=4MHz*24/1 */
    /* USB-PLL clock configuration register(K) initialize */
    FM3_USBETHERNETCLK->UPCR3 = UPCR3_UPLLK_INIT_VALUE;        // 4MHz:0, 16MHz:0
    /* USB-PLL clock configuration register(N) initialize */
    FM3_USBETHERNETCLK->UPCR4 = UPCR4_UPLLN_INIT_VALUE;        // 4MHz:60, 16Mz:24
    /* USB-PLL clock configuration register(N) initialize */
    FM3_USBETHERNETCLK->UPCR5 = UPCR5_UPLLN_INIT_VALUE;        // 4MHz:5,  16Mz:6
    bFM3_USBETHERNETCLK_UPINT_ENR_UPCSE = 0;          /* USB-PLL clock stabilize interrupt disable  */
    bFM3_USBETHERNETCLK_UPCR1_UPLLEN = 1;             /* enable USB-PLL clock */
    while (bFM3_USBETHERNETCLK_UP_STR_UPRDY == 0) ;   /* wait for USB-PLL clock ready */
    bFM3_USBETHERNETCLK_UCCR_UCEN1 = 1;               /* enable USB clock */
    /* wait for 5 cycle */
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    bFM3_USBETHERNETCLK_USBEN1_USBEN1 = 1;            /* enable USB controller */
    mdelay(100);
    return;
}

static void fm3_usbh_out_set_fifo_size(uint16_t size)
{
    bFM3_USB1_EP2S_BFINI = 1;
    FM3_USB1->EP2C = ((FM3_USB1->EP2C & 0xFF80) | size);
    bFM3_USB1_EP2C_DIR = 1;
    bFM3_USB1_EP2S_BFINI = 0;
}

static void fm3_usbh_in_set_fifo_size(uint16_t size)
{
    bFM3_USB1_EP1S_BFINI = 1;
    FM3_USB1->EP1C = ((FM3_USB1->EP1C & 0xFE00) | size);
    bFM3_USB1_EP1C_DIR = 0;
    bFM3_USB1_EP1S_BFINI = 0;
}

static inline void fm3_usbh_clear_fifo(void)
{
    bFM3_USB1_EP2S_BFINI = 1;   // initialize fifo
    bFM3_USB1_EP1S_BFINI = 1;   // initialize fifo
}

static inline void fm3_usbh_set_fifo(void)
{
    bFM3_USB1_EP2S_BFINI = 0;   // initialize fifo
    bFM3_USB1_EP1S_BFINI = 0;   // initialize fifo
}

static void fm3_usbh_read_fifo(__u8 *buffer, int len)
{
    int i;
    if (buffer != NULL) {
        for (i = 0; i < len; i++) {
            if ((i & 0x01) == 0x00) {
                *buffer = FM3_USB1->EP1DTL;
            } else {
                *buffer = FM3_USB1->EP1DTH;
            }
            buffer++;
        }
    }
}

static void fm3_usbh_write_fifo(__u8 *buffer, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        if ((i & 0x01) == 0x00) {
            FM3_USB1->EP2DTL = *buffer;
        } else {
            FM3_USB1->EP2DTH = *buffer;
        }
        buffer++;
    }
}

static inline void fm3_usbh_clear_out_ep_drq(void)
{
    bFM3_USB1_EP2S_DRQ = 0;
}

static inline void fm3_usbh_clear_in_ep_drq(void)
{
    bFM3_USB1_EP1S_DRQ = 0;
}

static void fm3_usbh_set_token(u8 addr, u8 toggle, u8 token, u8 num)
{
    u8 reg;
    reg = (token << 4) | (num & 0x0f);
    if (toggle)
        reg |= 0x80;
    else
        reg &= ~0x80;
    FM3_USB1->HADR = addr;
    FM3_USB1->HTOKEN = reg;
}

static inline void fm3_usbh_disable_all_int(void)
{
    bFM3_USB1_HCNT_RWKIRE = 0;  /* remote wake up interrupt */
    bFM3_USB1_HCNT_URIRE = 0;   /* USB bus reset interrupt */
    bFM3_USB1_HCNT_CMPIRE = 0;  /* completion interrupt */
    bFM3_USB1_HCNT_CNNIRE = 0;  /* connection interrupt */
    bFM3_USB1_HCNT_DIRE = 0;    /* disconnection interrupt */
    bFM3_USB1_HCNT_SOFIRE = 0;  /* SOF interrupt */
}

static inline void fm3_usbh_select_clock(void)
{
    bFM3_USB1_UDCC_RST = 1;
    bFM3_USB1_HSTATE_CLKSEL = bFM3_USB1_HSTATE_TMODE;
    bFM3_USB1_UDCC_RST = 0;
}

static void fm3_usbh_finish_request(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep, struct urb *urb, int status
)
{
	unsigned i;

	FUNC_ENTER();
	if (usb_pipecontrol(urb->pipe))
		ep->nextpid = USB_PID_SETUP;
	usb_hcd_unlink_urb_from_ep(fm3_usb_to_hcd(fm3_usb), urb);
	spin_unlock(&fm3_usb->lock);
	usb_hcd_giveback_urb(fm3_usb_to_hcd(fm3_usb), urb, status);
	spin_lock(&fm3_usb->lock);
	/* leave active endpoints in the schedule */
	if (!list_empty(&ep->hep->urb_list)) {
		FUNC_EXIT();
		return;
	}
	/* async deschedule? */
	if (!list_empty(&ep->schedule)) {
		list_del_init(&ep->schedule);
		if (ep == fm3_usb->next_async) {
			fm3_usb->next_async = NULL;
			dprintk("next_async=NULL\n");
		}
		FUNC_EXIT();
		return;
	}
	/* periodic deschedule */
	DBG("deschedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
	printk("deschedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
	for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
		struct fm3_usbh_ep *temp;
		struct fm3_usbh_ep **prev = &fm3_usb->periodic[i];
		while (*prev && ((temp = *prev) != ep))
			prev = &temp->next;
		if (*prev)
			*prev = ep->next;
		fm3_usb->load[i] -= ep->load;
	}
	ep->branch = PERIODIC_SIZE;
	fm3_usb->periodic_count--;
	fm3_usb_to_hcd(fm3_usb)->self.bandwidth_allocated
		-= ep->load / ep->period;
	if (ep == fm3_usb->next_periodic)
		fm3_usb->next_periodic = ep->next;
	FUNC_EXIT();
}

static void fm3_usbh_dump_regs(void);

#define HERR_LSTSOF_M	0x80
#define	HERR_RERR_M		0x40
#define	HERR_TOUT_M		0x20
#define	HERR_CRC_M		0x10
#define	HERR_TGERR_M	0x08
#define	HERR_STUFF_M	0x04
#define	HERR_FS_M		0x03

static void fm3_usbh_done(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep)
{
	struct urb *urb;
	int urbstat = -EINPROGRESS;
	int status = 0;
	int handshake = FM3_USB1->HERR;
	FM3_USB1->HERR &= 0x03;
	//FUNC_ENTER();
	if (unlikely(!ep))
		return;
	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
#ifdef FM3_USB_DEBUG_PACKET
	if (handshake & HERR_LSTSOF_M)
		printk("fm3_usbh err: LSTSOF\n");
	else if (handshake & HERR_RERR_M)
		printk("fm3_usbh err: RERR\n");
	else if (handshake & HERR_TOUT_M)
		printk("fm3_usbh err: TOUT\n");
	else if (handshake & HERR_CRC_M)
		printk("fm3_usbh err: CRC\n");
	else if (handshake & HERR_TGERR_M)
		printk("fm3_usbh err: TGERR\n");
	else if (handshake & HERR_STUFF_M)
		printk("fm3_usbh err: STUFF\n");
#endif
	switch (handshake & 0x03) {
	case USBH_MH_HANDSHAKE_ACK:
		status |= FM3_USBH_STATMASK_ACK;
		break;
	case USBH_MH_HANDSHAKE_NAK:
		status |= FM3_USBH_STATMASK_NAK;
#ifdef FM3_USB_DEBUG_PACKET
		printk("fm3_usbh warn: NAK\n");
#endif
		break;
	case USBH_MH_HANDSHAKE_STALL:
		status |= FM3_USBH_STATMASK_STALL;
		break;
	default:
		break;
	}
	if (status & FM3_USBH_STATMASK_NAK) {
		if (!ep->period)
			ep->nak_count++;
		ep->error_count = 0;
	} else if (status & FM3_USBH_STATMASK_ACK) {
		struct usb_device *udev = urb->dev;
		int len;
		unsigned char *buf;

		ep->nak_count = ep->error_count = 0;
		switch (ep->nextpid) {
		case USB_PID_OUT:
#ifdef FM3_USB_DEBUG_PACKET
			dprintk("PID_OUT\n");
#endif
			urb->actual_length += ep->length;
			usb_dotoggle(udev, ep->epnum, 1);
			if (urb->actual_length == urb->transfer_buffer_length) {
				if (usb_pipecontrol(urb->pipe))
					ep->nextpid = USB_PID_ACK;
				else if (ep->length < ep->maxpacket || !(urb->transfer_flags & URB_ZERO_PACKET))
					urbstat = 0;
			}
			break;
		case USB_PID_IN:
			buf = urb->transfer_buffer + urb->actual_length;
			prefetchw(buf);
            len = FM3_USB1->EP1S & 0x01FF;
			if (len > ep->length) {
				len = ep->length;
				urbstat = -EOVERFLOW;
			}
            if (len) {
            	fm3_usbh_read_fifo(buf, len);
            	fm3_usbh_clear_in_ep_drq();
            }
			urb->actual_length += len;
#ifdef FM3_USB_DEBUG_PACKET
			dprintk("PID_IN: len=%d\n", len);
#endif
			usb_dotoggle(udev, ep->epnum, 0);
			if (urbstat == -EINPROGRESS && (len < ep->maxpacket ||
				urb->actual_length == urb->transfer_buffer_length)) {
				if (usb_pipecontrol(urb->pipe))
					ep->nextpid = USB_PID_ACK;
				else
					urbstat = 0;
			}
			break;
		case USB_PID_SETUP:
#ifdef FM3_USB_DEBUG_PACKET
			dprintk("PID_SETUP\n");
#endif
			if (urb->transfer_buffer_length == urb->actual_length)
				ep->nextpid = USB_PID_ACK;
			else if (usb_pipeout(urb->pipe)) {
				usb_settoggle(udev, 0, 1, 1);
				ep->nextpid = USB_PID_OUT;
			} else {
				usb_settoggle(udev, 0, 0, 1);
				ep->nextpid = USB_PID_IN;
			}
			break;
		case USB_PID_ACK:
#ifdef FM3_USB_DEBUG_PACKET
			dprintk("PID_ACK\n");
#endif
			urbstat = 0;
			break;
		}
	} else if (status & FM3_USBH_STATMASK_STALL) {
#ifdef FM3_USB_DEBUG_PACKET
		dprintk("STALL\n");
#endif
		ep->nak_count = ep->error_count = 0;
		urbstat = -EPIPE;
	} else if (++ep->error_count >= 3) {
		if (status & FM3_USBH_STATMASK_TMOUT)
			urbstat = -ETIME;
		else if (status & FM3_USBH_STATMASK_OVF)
			urbstat = -EOVERFLOW;
		else
			urbstat = -EPROTO;
		ep->error_count = 0;
	}
	//if (urbstat == -EINPROGRESS) {
	//	fm3_usbh_dump_regs();
	//}
	if (urbstat != -EINPROGRESS || urb->unlinked)
		fm3_usbh_finish_request(fm3_usb, ep, urb, urbstat);
	//dprintk("%s urbstat=%d error_count=%d\n", __func__, urbstat, ep->error_count);
	transfer_busy = 0;
	//FUNC_EXIT();
}

static void fm3_usbh_CMPIRQ(struct fm3_usb *fm3_usb)
{
	fm3_usbh_done(fm3_usb, fm3_usb->active_a);
	fm3_usb->active_a = NULL;
}

static inline void fm3_usbh_CNNIRQ(void)
{
	//fm3_usbh_select_clock();
	fm3_usbh_clear_fifo();
	fm3_usbh_set_fifo();
	bFM3_USB1_HCNT_CNNIRE = 0;	// disable CNNIRQ
	bFM3_USB1_HIRQ_DIRQ = 0;	// clear DIRQ flag
	bFM3_USB1_HCNT_DIRE = 1;	// enable DIRQ
}

static inline void fm3_usbh_DIRQ(void)
{
	bFM3_USB1_HCNT_DIRE = 0;	// disable DIRQ
	bFM3_USB1_HIRQ_CNNIRQ = 0;	// clear CNNIRQ flag
	bFM3_USB1_HCNT_CNNIRE = 1;	// enable CNNIRQ
}

static int is_set_address(unsigned char *setup_packet)
{
	if (((setup_packet[0] & USB_TYPE_MASK) == USB_TYPE_STANDARD) &&
			setup_packet[1] == USB_REQ_SET_ADDRESS)
		return 1;
	else
		return 0;
}

static void fm3_usbh_setup_packet(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep, struct urb *urb)
{
	int addr = urb->dev->devnum;
	int toggle = 0;
	int len = sizeof(struct usb_ctrlrequest);
	int epnum = ep->epnum;
#ifdef FM3_USB_DEBUG_PACKET
	printk("PSetup addr=%d tog=%d ep=%d len=%d\n", addr, toggle, epnum, len);
#endif
	if (is_set_address(urb->setup_packet)) {
		addr = 0;
	}
	if (len) {
			fm3_usbh_out_set_fifo_size(ep->maxpacket);
			fm3_usbh_in_set_fifo_size(DEFAULT_MAX_PKS);
			fm3_usbh_write_fifo(urb->setup_packet, len);
	}
	fm3_usbh_clear_out_ep_drq();
	fm3_usbh_set_token(addr, toggle, USBH_MH_TOKEN_SETUP, epnum);
	ep->length = 0;
	transfer_busy = 1;

}

static void fm3_usbh_status_packet(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep, struct urb *urb)
{
	int addr = urb->dev->devnum;
	int toggle = 1;
	int epnum = ep->epnum;
	int do_out;
#ifdef FM3_USB_DEBUG_PACKET
	printk("PStatus addr=%d tog=%d ep=%d len=%d\n", addr, toggle, epnum, 0);
#endif
	do_out = urb->transfer_buffer_length && usb_pipein(urb->pipe);
	if (do_out) {
		fm3_usbh_clear_out_ep_drq();
		fm3_usbh_set_token(addr, toggle, USBH_MH_TOKEN_OUT, epnum);
	} else {
		fm3_usbh_in_set_fifo_size(DEFAULT_MAX_PKS);
		fm3_usbh_out_set_fifo_size(DEFAULT_MAX_PKS);
		fm3_usbh_set_token(addr, toggle, USBH_MH_TOKEN_IN, epnum);
	}
	ep->length = 0;
	transfer_busy = 1;
}

static void fm3_usbh_in_packet(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep, struct urb *urb)
{
	int addr = urb->dev->devnum;
	int toggle;
	int len;
	int epnum = ep->epnum;
	len = ep->maxpacket;
	toggle = usb_gettoggle(urb->dev, ep->epnum, 0);
	fm3_usbh_in_set_fifo_size(ep->maxpacket);
	fm3_usbh_out_set_fifo_size(DEFAULT_MAX_PKS);
	fm3_usbh_set_token(addr, toggle, USBH_MH_TOKEN_IN, epnum);
	ep->length = min_t(u32, len, urb->transfer_buffer_length - urb->actual_length);
#ifdef FM3_USB_DEBUG_PACKET
	printk("PIn    addr=%d tog=%d ep=%d (maxpacket=%d)\n", addr, toggle, epnum, len);
#endif
	transfer_busy = 1;
}

static void fm3_usbh_out_packet(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep, struct urb *urb)
{
	int addr = urb->dev->devnum;
	int toggle = 0;
	int len;
	int epnum = ep->epnum;
	void *buf;
	buf = urb->transfer_buffer + urb->actual_length;
	prefetch(buf);
	toggle = usb_gettoggle(urb->dev, ep->epnum, 1);
	len = min_t(u32, ep->maxpacket, urb->transfer_buffer_length - urb->actual_length);
	if (len) {
		fm3_usbh_out_set_fifo_size(len);
		fm3_usbh_in_set_fifo_size(DEFAULT_MAX_PKS);
		fm3_usbh_write_fifo(buf, len);
	}
	fm3_usbh_clear_out_ep_drq();
	fm3_usbh_set_token(addr, toggle, USBH_MH_TOKEN_OUT, epnum);
	ep->length = len;
#ifdef FM3_USB_DEBUG_PACKET
	printk("POut   addr=%d tog=%d ep=%d len=%d\n", addr, toggle, epnum, len);
#endif
	transfer_busy = 1;
}

static struct fm3_usbh_ep *fm3_usbh_get_scheduled_ep(struct fm3_usb *fm3_usb)
{
	struct fm3_usbh_ep *ep = (struct fm3_usbh_ep *)NULL;

	if (fm3_usb->next_periodic) {	/* use endpoint at schedule head */
		//printk("p.");
		ep = fm3_usb->next_periodic;
		fm3_usb->next_periodic = ep->next;
	} else {
		if (fm3_usb->next_async) {
			ep = fm3_usb->next_async;
			//printk("p1");
		} else if (!list_empty(&fm3_usb->async)) {
			ep = container_of(fm3_usb->async.next, struct fm3_usbh_ep, schedule);
			//printk("p2");
	} else {
			/* could set up the first fullspeed periodic
			 * transfer for the next frame ...
			 */
			//printk("p3");
			return (struct fm3_usbh_ep *)NULL;
		}
		if (ep->schedule.next == &fm3_usb->async) {
			fm3_usb->next_async = NULL;
			//printk("p4");
		} else {
			fm3_usb->next_async = container_of(ep->schedule.next, struct fm3_usbh_ep, schedule);
			//printk("p5");
		}
	}
	if (unlikely(list_empty(&ep->hep->urb_list))) {
		DBG("empty %p queue?\n", ep);
		return (struct fm3_usbh_ep *)NULL;
	}
	return ep;
}

static struct fm3_usbh_ep *fm3_usbh_send_packet(struct fm3_usb *fm3_usb, struct fm3_usbh_ep *ep)
{
	struct urb *urb;

	//FUNC_ENTER();
	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
	switch (ep->nextpid) {
	case USB_PID_IN:
		fm3_usbh_in_packet(fm3_usb, ep, urb);
		break;
	case USB_PID_OUT:
		fm3_usbh_out_packet(fm3_usb, ep, urb);
		break;
	case USB_PID_SETUP:
		fm3_usbh_setup_packet(fm3_usb, ep, urb);
		break;
	case USB_PID_ACK:		/* for control status */
		fm3_usbh_status_packet(fm3_usb, ep, urb);
		break;
	default:
		DBG("bad ep%p pid %02x\n", ep, ep->nextpid);
		ep = (struct fm3_usbh_ep *)NULL;
	}
	//FUNC_EXIT();
	return ep;
}

static void fm3_usbh_start_transfer(struct fm3_usb *fm3_usb)
{
	struct fm3_usbh_ep *ep;
	if (fm3_usb->hubport & (1 << USB_PORT_FEAT_SUSPEND)) {
		return;
	}
	bFM3_USB1_HCNT_CMPIRE = 0;
	bFM3_USB1_HCNT_SOFIRE = 0;
	if (!transfer_busy) {
		if (fm3_usb->active_a == NULL) {
			spin_lock(&fm3_usb->lock);
			ep = fm3_usbh_get_scheduled_ep(fm3_usb);
			if (ep) {
				fm3_usb->active_a = fm3_usbh_send_packet(fm3_usb, ep);
				if (fm3_usb->active_a != NULL)
					fm3_usb->jiffies_a = jiffies + MIN_JIFFIES;
			}
			spin_unlock(&fm3_usb->lock);
		}
	}
	bFM3_USB1_HCNT_CMPIRE = 1;
	bFM3_USB1_HCNT_SOFIRE = 1;
}

static inline void fm3_usbh_SOFIRQ(struct fm3_usb *fm3_usb)
{
	fm3_usbh_start_transfer(fm3_usb);
}

static void fm3_usbh_dump_regs(void)
{
	u8 hcnt0;
    u8 hcnt1;
    u8 hirq;
    u8 herr;
    u8 hstate;
    u8 hfcomp;
    u8 hrtimer;
    u8 hadr;
    u8 heof;
    u8 hframe;
    u8 htoken;
    hcnt0 = FM3_USB1->HCNT0;
    hcnt1 = FM3_USB1->HCNT1;
    hirq = FM3_USB1->HIRQ;
    herr = FM3_USB1->HERR;
    hstate = FM3_USB1->HSTATE;
    hfcomp = FM3_USB1->HFCOMP;
    hrtimer = FM3_USB1->HRTIMER;
    hadr = FM3_USB1->HADR;
    heof = FM3_USB1->HEOF;
    hframe = FM3_USB1->HFRAME;
    htoken = FM3_USB1->HTOKEN;
    dprintk("HCNT0=%02x HNCT1=%02x HIRQ=%02x HERR=%02x HSTATE=%02x\n", hcnt0, hcnt1, hirq, herr, hstate);
}

static irqreturn_t fm3_usbh_irq(struct usb_hcd *hcd)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	irqreturn_t	ret = IRQ_HANDLED;

	//dprintk("U.");
	spin_lock(&fm3_usb->lock);
    if (bFM3_USB1_HIRQ_CMPIRQ == 1) {
        bFM3_USB1_HIRQ_CMPIRQ = 0;
#ifdef FM3_USB_DEBUG_INT
        printk("INT CMPIRQ\n");
#endif
        fm3_usbh_CMPIRQ(fm3_usb);
    }
    if (bFM3_USB1_HIRQ_URIRQ == 1) {	// busreset
        bFM3_USB1_HIRQ_URIRQ = 0;
        bFM3_USB1_HCNT_URIRE = 0;		// disable URIRQ
        f_busreset_done = 1;
#ifdef FM3_USB_DEBUG_INT
        printk("INT URIRQ\n");
#endif
    }
    if (bFM3_USB1_HIRQ_CNNIRQ == 1) {	// device connect
        bFM3_USB1_HIRQ_CNNIRQ = 0;
        f_device_connect = 1;
        fm3_usb->hubport |= (1 << USB_PORT_FEAT_CONNECTION);
        fm3_usb->hubport |= (1 << USB_PORT_FEAT_C_CONNECTION);
        fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_RESET);
        fm3_usbh_CNNIRQ();
#ifdef fm3_USB_DEBUG_INT
        printk("INT CNNIRQ\n");
#endif
    }
    if (bFM3_USB1_HIRQ_DIRQ == 1) {		// device disconnect
    	bFM3_USB1_HIRQ_DIRQ = 0;
        f_device_connect = 0;
        fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_CONNECTION);
        fm3_usb->hubport |= (1 << USB_PORT_FEAT_C_CONNECTION);
    	fm3_usbh_DIRQ();
#ifdef fm3_USB_DEBUG_INT
    	printk("INT DIRQ\n");
#endif
    }
    if (bFM3_USB1_HIRQ_SOFIRQ == 1) {	// SOF interrupt
        bFM3_USB1_HIRQ_SOFIRQ = 0;
        fm3_usbh_SOFIRQ(fm3_usb);
#ifdef FM3_USB_DEBUG_INT
    	//printk("s.");
#endif
    } else if (FM3_USB1->HIRQ != 0) {
    	fm3_usbh_dump_regs();
    }
	spin_unlock(&fm3_usb->lock);
	return ret;
}

/* The function works only if interrupt is disabled. */
static int fm3_usbh_is_device_connected(void)
{
    int timeout = FM3_USBH_TIMEOUT;
    while (bFM3_USB1_HIRQ_CNNIRQ) {
        if (timeout-- <= 0) {
            return 0;
        }
    }
    return 1;
}

static void fm3_usbh_set_tmode(void)
{
    bFM3_USB1_UDCC_RST = 1;
    if (bFM3_USB1_HSTATE_TMODE == 1) {
        transfer_speed = 1;
        bFM3_USB1_HSTATE_ALIVE = 0;
        printk("FULL SPEED detected\n");
    } else {
        transfer_speed = 0;
        bFM3_USB1_HSTATE_ALIVE = 1;
        printk("LOW SPEED detected\n");
    }
    bFM3_USB1_HSTATE_CLKSEL = transfer_speed;
    while (bFM3_USB1_HSTATE_CLKSEL != transfer_speed) ;
    bFM3_USB1_UDCC_RST = 0;
}

static void fm3_usbh_force_tmode(int mode, int fmessage)
{
    bFM3_USB0_UDCC_RST = 1;
    if (mode == 1) {
        transfer_speed = 1;
        bFM3_USB0_HSTATE_ALIVE = 0;
        if (fmessage)
        	printk("Changed to FULL SPEED\n");
    } else {
        transfer_speed = 0;
        bFM3_USB0_HSTATE_ALIVE = 1;
        if (fmessage)
        	printk("Changed to LOW SPEED\n");
    }
    bFM3_USB0_HSTATE_CLKSEL = transfer_speed;
    while (bFM3_USB0_HSTATE_CLKSEL != transfer_speed) ;
    bFM3_USB0_UDCC_RST = 0;
    //mdelay(10);
    //fm3_usbh_set_token(0, 0, USBH_MH_TOKEN_SOF, 0);
}

static void fm3_usbh_busreset(void)
{
    int timeout;
    FUNC_ENTER();
    if (bFM3_USB1_HSTATE_CSTAT == 1) {
        f_busreset_done = 0;
        f_device_connect = 1;
        bFM3_USB1_HCNT_URIRE = 1;
        bFM3_USB1_HCNT_URST = 1;
        timeout = FM3_USBH_TIMEOUT * 10;
        while (f_busreset_done == 0) {
            if (timeout-- <= 0) {
                printk("%s err: busreset timeout\n", hcd_name);
                break;
            }
        }
        timeout = FM3_USBH_TIMEOUT;
        while (f_device_connect == 0) {
        	if (timeout-- <= 0) {
        		break;
        	}
        }
        fm3_usbh_set_tmode();
    } else {
    	printk("%s warn: device not connected\n", hcd_name);
    }
    FUNC_EXIT();
}

static int fm3_usbh_initialize(void)
{
	FUNC_ENTER();
	//__disable_irq();
    transfer_busy = 0;
    fm3_usbh_disable_all_int();
    fm3_usbh_clock_init();
    //FM3_GPIO->PFR8 |= 0x00000003;
    FM3_GPIO->PFR8 |= 0x0000000C;
    bFM3_GPIO_SPSR_USB1C = 1;
#if 0
    bFM3_GPIO_PDOR6_P62 = ~(USBH_VBUS_IO_VALID_LEVEL);
    bFM3_GPIO_DDR6_P62 = 1;
#endif

    bFM3_USB1_HCNT_HOST=0;
    bFM3_USB1_UDCC_HCONX=1;     /* set host mode */
    bFM3_USB1_UDCC_RST = 1;     /* reset USB function */

    bFM3_USB1_EP1C_EPEN = 0;
    bFM3_USB1_EP2C_EPEN = 0;
    fm3_usbh_clear_fifo();
    FM3_USB1->EP1C = 0x4000  | (uint16_t)USBH_MH_PKS_INIT;
    FM3_USB1->EP2C = 0x4000  | (uint16_t)USBH_MH_PKS_INIT;
    bFM3_USB1_EP1C_DIR = 0;  /* endpoint1 for IN-direction transfer */
    bFM3_USB1_EP2C_DIR = 1;  /* endpoint2 for OUT-direction transfer */
    bFM3_USB1_EP1C_EPEN = 1;
    bFM3_USB1_EP2C_EPEN = 1;

    FM3_USB1->HFCOMP = 0;
    FM3_USB1->HRTIMER0 = 0;
    FM3_USB1->HRTIMER1 = 0;
    FM3_USB1->HRTIMER2 = 0;
    bFM3_USB1_HSTATE_CSTAT = 0;
    bFM3_USB1_HSTATE_ALIVE = 0;

    FM3_USB1->HADR = 0;         /* set host address to 0 */
    bFM3_USB1_HCNT_HOST=1;      /* set host mode */
    while (bFM3_USB1_HCNT_HOST != 1) ;
    if (fm3_usbh_is_device_connected()) {
    	fm3_usbh_set_tmode();
    }
	FM3_USB1->HEOF = USBH_MH_EOF_TIME;
    bFM3_USB1_UDCC_RST = 0;     /* cancel reset condition of USB function */
    //fm3_usbh_clear_fifo();
    //fm3_usbh_set_fifo();
    FM3_USB1->HIRQ = 0;
    bFM3_USB1_HCNT_SOFSTEP = 1; /* generate an interrupt request during SOF processing */
    bFM3_USB1_HCNT_CANCEL = 0;  /* continue with the token in the EOF area             */
    bFM3_USB1_HCNT_RETRY = 0;   /* do not retry the operation when an error occurs     */

	FUNC_EXIT();
    return 0;
}

static void port_power(struct fm3_usb *fm3_usb, int is_on)
{
	struct usb_hcd	*hcd = fm3_usb_to_hcd(fm3_usb);
	/* hub is inactive unless the port is powered */
	dprintk("port_power %d\n", is_on);
	if (is_on) {
		if (fm3_usb->hubport & (1 << USB_PORT_FEAT_POWER))
			return;
		fm3_usb->hubport = (1 << USB_PORT_FEAT_POWER);
		f_device_connect = 0;
#if 0
		if (bFM3_GPIO_PDOR6_P62 != USBH_VBUS_IO_VALID_LEVEL) {
			bFM3_GPIO_PDOR6_P62 = USBH_VBUS_IO_VALID_LEVEL;
			fm3_usbh_wait(1000);
		}
#endif
		fm3_usbh_busreset();
		if (transfer_speed == 1) {
			// Full speed
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_LOWSPEED);
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_HIGHSPEED);
		} else {
			// Low speed
			fm3_usb->hubport |= (1 << USB_PORT_FEAT_LOWSPEED);
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_HIGHSPEED);
		}
		fm3_usbh_set_token(0, 0, USBH_MH_TOKEN_SOF, 0);
	} else {
		fm3_usb->hubport = 0;
		fm3_usbh_disable_all_int();
#if 0
	    bFM3_GPIO_PDOR6_P62 = ~USBH_VBUS_IO_VALID_LEVEL;
#endif
	    hcd->state = HC_STATE_HALT;
	}
}

#define	MAX_PERIODIC_LOAD	500	/* out of 1000 usec */

static int balance(struct fm3_usb *fm3_usb, u16 period, u16 load)
{
	int	i, branch = -ENOSPC;

	/* search for the least loaded schedule branch of that period
	 * which has enough bandwidth left unreserved.
	 */
	for (i = 0; i < period ; i++) {
		if (branch < 0 || fm3_usb->load[branch] > fm3_usb->load[i]) {
			int	j;

			for (j = i; j < PERIODIC_SIZE; j += period) {
				if ((fm3_usb->load[j] + load)
						> MAX_PERIODIC_LOAD)
					break;
			}
			if (j < PERIODIC_SIZE)
				continue;
			branch = i;
	}
}
	//dprintk("balance %d\n", branch);
	dprintk("balance p=%d b=%d\n", period, branch);
	return branch;
}

static int fm3_usbh_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	struct usb_device *udev = urb->dev;
	unsigned int pipe = urb->pipe;
	int is_out = !usb_pipein(pipe);
	int type = usb_pipetype(pipe);
	int epnum = usb_pipeendpoint(pipe);
	struct fm3_usbh_ep *ep = NULL;
	unsigned long flags;
	int i;
	int retval;
	struct usb_host_endpoint *hep = urb->ep;

	API_FUNC_ENTER();
#ifdef	DISABLE_ISO
	if (type == PIPE_ISOCHRONOUS)
		return -ENOSPC;
#endif
	if (!hep->hcpriv)
		ep = kzalloc(sizeof *ep, mem_flags);
	/* don't submit to a dead or disabled port */
	if (!(fm3_usb->hubport & (1 << USB_PORT_FEAT_ENABLE)) || !HC_IS_RUNNING(hcd->state)) {
		retval = -ENODEV;
		kfree(ep);
		goto fail_not_linked;
	}
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if (retval) {
		kfree(ep);
		goto fail_not_linked;
	}
	if (hep->hcpriv) {
		kfree(ep);
		ep = hep->hcpriv;
	} else if (!ep) {
		retval = -ENOMEM;
		goto fail;
	} else {
		INIT_LIST_HEAD(&ep->schedule);
		ep->udev = udev;
		ep->epnum = epnum;
		ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
		ep->defctrl = FM3_USBH_HCTLMASK_ARM | FM3_USBH_HCTLMASK_ENABLE;
		usb_settoggle(udev, epnum, is_out, 0);

		if (type == PIPE_CONTROL)
			ep->nextpid = USB_PID_SETUP;
		else if (is_out)
			ep->nextpid = USB_PID_OUT;
		else
			ep->nextpid = USB_PID_IN;
		if (udev->speed == USB_SPEED_LOW) {
			/* ToDo: Low speed */
			/* send preamble for external hub? */
		}
		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			if (urb->interval > PERIODIC_SIZE)
				urb->interval = PERIODIC_SIZE;
			ep->period = urb->interval;
			ep->branch = PERIODIC_SIZE;
			if (type == PIPE_ISOCHRONOUS)
				ep->defctrl |= FM3_USBH_HCTLMASK_ISOCH;
			ep->load = usb_calc_bus_time(udev->speed, !is_out, (type == PIPE_ISOCHRONOUS), usb_maxpacket(udev, pipe, is_out)) / 1000;
			printk("p=%d b=%d l=%d\n", ep->period, ep->branch, ep->load);
			break;
		}
		ep->hep = hep;
		hep->hcpriv = ep;
	}
	switch (type) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		if (list_empty(&ep->schedule)) {
			list_add_tail(&ep->schedule, &fm3_usb->async);
		}
		break;
	case PIPE_ISOCHRONOUS:
	case PIPE_INTERRUPT:
		printk("i=%d b=%d\n", urb->interval, ep->branch);
		urb->interval = ep->period;
		if (ep->branch < PERIODIC_SIZE) {
			urb->start_frame = (fm3_usb->frame & (PERIODIC_SIZE - 1)) + ep->branch;
			break;
		}
		retval = balance(fm3_usb, ep->period, ep->load);
		if (retval < 0)
			goto fail;
		ep->branch = retval;
		retval = 0;
		urb->start_frame = (fm3_usb->frame & (PERIODIC_SIZE - 1)) + ep->branch;
		DBG("schedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
		printk("schedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
		for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
			struct fm3_usbh_ep **prev = &fm3_usb->periodic[i];
			struct fm3_usbh_ep *here = *prev;

			while (here && ep != here) {
				if (ep->period > here->period)
					break;
				prev = &here->next;
				here = *prev;
			}
			if (ep != here) {
				ep->next = here;
				*prev = ep;
			}
			fm3_usb->load[i] += ep->load;
		}
		fm3_usb->periodic_count++;
		hcd->self.bandwidth_allocated += ep->load / ep->period;
		//sofirq_on(fm3_usb);
	}
	urb->hcpriv = hep;
	//fm3_usbh_start_transfer(fm3_usb);
fail:
	if (retval)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
fail_not_linked:
	spin_unlock_irqrestore(&fm3_usb->lock, flags);
	spin_lock_irqsave(&fm3_usb->lock, flags);
	//dprintk("type=%d ret=%d\n", type, retval);
	API_FUNC_EXIT();
	return retval;
}

/* manage i/o requests, device state */
static int fm3_usbh_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	struct usb_host_endpoint *hep;
	unsigned long flags;
	struct fm3_usbh_ep *ep;
	int retval;

	API_FUNC_ENTER();
	spin_lock_irqsave(&fm3_usb->lock, flags);
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (retval)
		goto fail;
	hep = urb->hcpriv;
	ep = hep->hcpriv;
	if (ep) {
		if (ep->hep->urb_list.next != &urb->urb_list) {
			/* not front of queue?  never active */
		/* for active transfers, we expect an IRQ */
		} else if (fm3_usb->active_a == ep) {
			if (time_before_eq(fm3_usb->jiffies_a, jiffies)) {
				/* happens a lot with lowspeed?? */
				/* ToDo: */
				fm3_usb->active_a = NULL;
			} else
				urb = NULL;
		} else {
			/* front of queue for inactive endpoint */
		}
		if (urb)
			fm3_usbh_finish_request(fm3_usb, ep, urb, 0);
		else
			VDBG("dequeue, urb %p active %s; wait4irq\n", urb, (fm3_usb->active_a == ep) ? "A" : "B");
	} else
		retval = -EINVAL;
 fail:
	spin_unlock_irqrestore(&fm3_usb->lock, flags);
	API_FUNC_EXIT();
	return retval;
}

static void fm3_usbh_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct fm3_usbh_ep *ep = hep->hcpriv;
	API_FUNC_ENTER();
	if (ep) {
		if (!list_empty(&hep->urb_list))
			msleep(3);
		if (!list_empty(&hep->urb_list))
			WARNING("ep %p not empty?\n", ep);
		kfree(ep);
		hep->hcpriv = NULL;
	}
	//fm3_usbh_disable_all_int();
	API_FUNC_EXIT();
}

/* return current frame number */
static int fm3_usbh_get_frame(struct usb_hcd *hcd)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	API_FUNC_ENTER();
	dprintk("frame=%d\n", fm3_usb->frame);
	API_FUNC_EXIT();
	return fm3_usb->frame;
}

static int fm3_usbh_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	int retval = 0;
	API_FUNC_ENTER();
	if (fm3_usb->hubport & (0xffff << 16)) {
		*buf = (1 << 1);
		retval = 1;
		dprintk("hubport=%0x ret=%d\n", fm3_usb->hubport, retval);
	}
	//dprintk("hubport=%0x ret=%d\n", fm3_usb->hubport, retval);
	API_FUNC_EXIT();
	return retval;
}

static void fm3_usbh_hub_descriptor (struct fm3_usb *fm3_usb, struct usb_hub_descriptor *desc)
{
	u16		temp = 0;

	FUNC_ENTER();
	desc->bDescriptorType = 0x29;
	desc->bHubContrCurrent = 0;
	desc->bNbrPorts = 1;
	desc->bDescLength = 9;
	/* per-port power switching (gang of one!), or none */
	desc->bPwrOn2PwrGood = 0;
	if (fm3_usb->board && fm3_usb->board->port_power) {
		desc->bPwrOn2PwrGood = fm3_usb->board->potpg;
		if (!desc->bPwrOn2PwrGood)
			desc->bPwrOn2PwrGood = 10;
		temp = 0x0001;
	} else
		temp = 0x0002;
	/* no overcurrent errors detection/handling */
	temp |= 0x0010;

	desc->wHubCharacteristics = cpu_to_le16(temp);
	/* two bitmaps:  ports removable, and legacy PortPwrCtrlMask */
	desc->bitmap[0] = 0 << 1;
	desc->bitmap[1] = ~0;
	FUNC_EXIT();
}

static void fm3_usbh_timer(unsigned long _fm3_usb)
{
	struct fm3_usb *fm3_usb = (void *) _fm3_usb;
	unsigned long flags;
	FUNC_ENTER();
	//fm3_usbh_force_tmode(0, 0);
	//if (fm3_usbh_is_device_connected()) {
	//    fm3_usbh_set_tmode(0);
	//}
	spin_lock_irqsave(&fm3_usb->lock, flags);
	fm3_usb->hubport |= ((1 << USB_PORT_FEAT_CONNECTION) | (1 << USB_PORT_FEAT_ENABLE));
	// USB_PORT_FEAT_RESET
	if (fm3_usb->hubport & (1 << USB_PORT_FEAT_RESET)) {
		if (transfer_speed == 1) {
			// Full Speed
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_LOWSPEED);
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_HIGHSPEED);
		} else {
			// Low Speed
			fm3_usb->hubport |= (1 << USB_PORT_FEAT_LOWSPEED);
			fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_HIGHSPEED);
		}
		fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_RESET);
		fm3_usb->hubport |= 1 << USB_PORT_FEAT_ENABLE;;
	}
	spin_unlock_irqrestore(&fm3_usb->lock, flags);
	FUNC_EXIT();
}

static int fm3_usbh_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	int retval = 0;
	unsigned long flags;

	API_FUNC_ENTER();
	//dprintk("req=%0x val=%0x\n", typeReq, wValue);
	spin_lock_irqsave(&fm3_usb->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		dprintk("CHubF\n");
		break;
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
			dprintk("SHubF:OVER_CURRENT\n");
			break;
		case C_HUB_LOCAL_POWER:
			dprintk("SHubF:LOCAL_POWER\n");
			break;
		default:
			dprintk("SHubF:Error\n");
			goto error;
		}
		break;
	case ClearPortFeature:
		if (wIndex != 1 || wLength != 0) {
			dprintk("CPortF:Error\n");
			goto error;
		}
		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			dprintk("CPortF:ENABLE\n");
			//fm3_usb->hubport &= ~(1 << USB_PORT_FEAT_POWER);
			break;
		case USB_PORT_FEAT_SUSPEND:
			dprintk("CportF:SUSPEND\n");
			mod_timer(&fm3_usb->timer, jiffies + msecs_to_jiffies(20));
			break;
		case USB_PORT_FEAT_POWER:
			dprintk("CPortF:POWER\n");
			port_power(fm3_usb, 0);
			break;
		case USB_PORT_FEAT_C_ENABLE:
			dprintk("CPortF:C_ENABLE\n");
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			dprintk("CPortF:C_SUSPEND\n");
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			dprintk("CPortF:C_CONNECTION\n");
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			dprintk("CPortF:OVER_CURRENT\n");
			break;
		case USB_PORT_FEAT_C_RESET:
			dprintk("CPortF:C_RESET\n");
			break;
		default:
			dprintk("CPortF:Error\n");
			goto error;
		}
		fm3_usb->hubport &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		dprintk("GetHubDescriptor\n");
		fm3_usbh_hub_descriptor(fm3_usb, (struct usb_hub_descriptor *) buf);
		break;
	case GetHubStatus:
		dprintk("GetHubStatus\n");
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		dprintk("GetPortStatus %08x\n", fm3_usb->hubport);
		//if (wIndex != 1)
		//	goto error;
		*(__le32 *) buf = cpu_to_le32(fm3_usb->hubport);
		break;
	case SetPortFeature:
		if (wIndex != 1 || wLength != 0) {
			dprintk("SPortF:Error\n");
			goto error;
		}
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			dprintk("SPortF:SUSPEND\n");
			if (fm3_usb->hubport & (1 << USB_PORT_FEAT_RESET))
				goto error;
			if (!(fm3_usb->hubport & (1 << USB_PORT_FEAT_ENABLE)))
				goto error;
			dprintk("suspend...\n");
			// ToDo: SOF disabled?
			break;
		case USB_PORT_FEAT_POWER:
			dprintk("SPortF:POWER\n");
			port_power(fm3_usb, 1);
			fm3_usb->hubport |= (1 << USB_PORT_FEAT_POWER);
			break;
		case USB_PORT_FEAT_RESET:
			dprintk("SportF:RESET\n");
			if (fm3_usb->hubport & (1 << USB_PORT_FEAT_SUSPEND))
				goto error;
			//if (!(fm3_usb->hubport & (1 << USB_PORT_FEAT_POWER)))
			//	break;
			fm3_usb->hubport |= (1 << USB_PORT_FEAT_RESET);
			mod_timer(&fm3_usb->timer, jiffies + msecs_to_jiffies(50));
			break;
		default:
			dprintk("SPortF:Error\n");
			goto error;
		}
		fm3_usb->hubport |= 1 << wValue;
		break;
	default:
error:
		dprintk("default:Error\n");
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&fm3_usb->lock, flags);
	if (retval)
		dprintk("%s Err retval=%d\n", __func__, retval);
	API_FUNC_EXIT();
	return retval;
}

#ifdef	CONFIG_PM

static int fm3_usbh_bus_suspend(struct usb_hcd *hcd)
{
	// SOFs off
	DBG("%s\n", __func__);
	return 0;
}

static int fm3_usbh_bus_resume(struct usb_hcd *hcd)
{
	// SOFs on
	DBG("%s\n", __func__);
	return 0;
}

#else

#define	fm3_usbh_bus_suspend	NULL
#define	fm3_usbh_bus_resume	NULL

#endif

#ifdef STUB_DEBUG_FILE

static inline void create_debug_file(struct fm3_usb *fm3_usb) { }
static inline void remove_debug_file(struct fm3_usb *fm3_usb) { }

#else

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void dump_irq(struct seq_file *s, char *label, u8 mask)
{
}

static int proc_fm3_usbh_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int proc_fm3_usbh_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_fm3_usbh_show, PDE(inode)->data);
}

static const struct file_operations proc_ops = {
	.open		= proc_fm3_usbh_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* expect just one fm3_usb per system */
static const char proc_filename[] = "driver/fm3_usbh";

static void create_debug_file(struct fm3_usb *fm3_usb)
{
	fm3_usb->pde = proc_create_data(proc_filename, 0, NULL, &proc_ops, fm3_usb);
}

static void remove_debug_file(struct fm3_usb *fm3_usb)
{
	if (fm3_usb->pde)
		remove_proc_entry(proc_filename, NULL);
}

#endif

/*-------------------------------------------------------------------------*/

static void fm3_usbh_stop(struct usb_hcd *hcd)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	unsigned long flags;

	API_FUNC_ENTER();
	del_timer_sync(&hcd->rh_timer);
	spin_lock_irqsave(&fm3_usb->lock, flags);
	port_power(fm3_usb, 0);
    bFM3_USB1_UDCC_RST = 1;     /* reset USB function */
	spin_unlock_irqrestore(&fm3_usb->lock, flags);
	API_FUNC_EXIT();
}

static int fm3_usbh_start(struct usb_hcd *hcd)
{
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	/* chip has been reset, VBUS power is off */
	/* enable power and interrupts */
	API_FUNC_ENTER();
	hcd->state = HC_STATE_RUNNING;
	if (fm3_usb->board) {
		if (!device_can_wakeup(hcd->self.controller))
			device_init_wakeup(hcd->self.controller,
				fm3_usb->board->can_wakeup);
		hcd->power_budget = 500;
	}
	port_power(fm3_usb, 1);
	API_FUNC_EXIT();
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct hc_driver fm3_usbh_hc_driver = {
	.description =		hcd_name,
	.hcd_priv_size =	sizeof(struct fm3_usb),
	/*
	 * generic hardware linkage
	 */
	.irq =			fm3_usbh_irq,
	.flags =		HCD_USB2 | HCD_MEMORY,
	/* Basic lifecycle operations */
	.start =		fm3_usbh_start,
	.stop =			fm3_usbh_stop,
	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		fm3_usbh_urb_enqueue,
	.urb_dequeue =		fm3_usbh_urb_dequeue,
	.endpoint_disable =	fm3_usbh_endpoint_disable,
	/*
	 * periodic schedule support
	 */
	.get_frame_number =	fm3_usbh_get_frame,
	/*
	 * root hub support
	 */
	.hub_status_data =	fm3_usbh_hub_status_data,
	.hub_control =		fm3_usbh_hub_control,
	.bus_suspend =		fm3_usbh_bus_suspend,
	.bus_resume =		fm3_usbh_bus_resume,
};

/*-------------------------------------------------------------------------*/

static int __devexit fm3_usbh_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	API_FUNC_ENTER();
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	API_FUNC_EXIT();
	return 0;
}

static int __devinit fm3_usbh_probe(struct platform_device *dev)
{
	struct usb_hcd *hcd;
	struct fm3_usb *fm3_usb;
	unsigned long irqflags = 0;
	int retval = 0;
	int irq = USB1F_USB1H_IRQn;

	API_FUNC_ENTER();
	//ires = platform_get_resource(dev, IORESOURCE_IRQ, 0);
	hcd = usb_create_hcd(&fm3_usbh_hc_driver, &dev->dev, dev_name(&dev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fm3_usbh_probe_exit;
	}
	fm3_usb = hcd_to_fm3_usb(hcd);
	spin_lock_init(&fm3_usb->lock);
	INIT_LIST_HEAD(&fm3_usb->async);
	fm3_usb->board = dev->dev.platform_data;
	init_timer(&fm3_usb->timer);
	fm3_usb->timer.function = fm3_usbh_timer;
	fm3_usb->timer.data = (unsigned long) fm3_usb;

	spin_lock_irq(&fm3_usb->lock);
	fm3_usbh_initialize();
	//port_power(fm3_usb, 0);
	spin_unlock_irq(&fm3_usb->lock);
	msleep(200);

    bFM3_USB1_HCNT_RWKIRE = 1;  /* remote wake up interrupt */
    bFM3_USB1_HCNT_CMPIRE = 1;  /* token completion interrupt */
    bFM3_USB1_HCNT_CNNIRE = 1;  /* connection interrupt of USB device */
    bFM3_USB1_HCNT_DIRE = 0;    /* disconnection interrupt of USB device */
    bFM3_USB1_HCNT_SOFIRE = 1;  /* SOF token interrupt */
	irqflags |= IRQF_SHARED;
	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | irqflags);
	if (retval != 0) {
		usb_put_hcd(hcd);
		goto fm3_usbh_probe_exit;
	}
fm3_usbh_probe_exit:
	dprintk("%s retval=%d\n", __func__, retval);
	API_FUNC_EXIT();
	return retval;
}

#ifdef	CONFIG_PM
static int
fm3_usbh_suspend(struct platform_device *dev, pm_message_t state)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct fm3_usb	*fm3_usb = hcd_to_fm3_usb(hcd);
	int		retval = 0;
	API_FUNC_ENTER();
	switch (state.event) {
	case PM_EVENT_FREEZE:
		retval = fm3_usbh_bus_suspend(hcd);
		break;
	case PM_EVENT_SUSPEND:
	case PM_EVENT_HIBERNATE:
	case PM_EVENT_PRETHAW:		/* explicitly discard hw state */
		port_power(fm3_usb, 0);
		break;
	}
	API_FUNC_EXIT();
	return retval;
}

static int
fm3_usbh_resume(struct platform_device *dev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct fm3_usb *fm3_usb = hcd_to_fm3_usb(hcd);
	API_FUNC_ENTER();
	if (!fm3_usb->hubport || !device_can_wakeup(&hcd->self.root_hub->dev)) {
		fm3_usb->hubport = 0;
		port_power(fm3_usb, 1);
		usb_root_hub_lost_power(hcd->self.root_hub);
		return 0;
	}
	API_FUNC_EXIT();
	return fm3_usbh_bus_resume(hcd);
}

#else

#if 0
#define	fm3_usbh_suspend	NULL
#define	fm3_usbh_resume	NULL
#else
static int fm3_usbh_suspend(struct platform_device *dev, pm_message_t state)
{
	API_FUNC_ENTER();
	API_FUNC_EXIT();
	return 0;
}

static int fm3_usbh_resume(struct platform_device *dev)
{
	API_FUNC_ENTER();
	API_FUNC_EXIT();
	return 0;
}
#endif

#endif

struct platform_driver fm3_usbh_driver = {
	.probe =	fm3_usbh_probe,
	.remove =	__devexit_p(fm3_usbh_remove),

	.suspend =	fm3_usbh_suspend,
	.resume =	fm3_usbh_resume,
	.driver = {
		.name =	(char *) hcd_name,
		.owner = THIS_MODULE,
	},
};
EXPORT_SYMBOL(fm3_usbh_driver);

static int __init fm3_usbh_init(void)
{
	int retval;
	API_FUNC_ENTER();
	if (usb_disabled())
		retval = -ENODEV;
	else {
		INFO("driver %s, %s\n", hcd_name, DRIVER_VERSION);
		retval = platform_driver_register(&fm3_usbh_driver);
	}
	API_FUNC_EXIT();
	return retval;
}
module_init(fm3_usbh_init);

static void __exit fm3_usbh_cleanup(void)
{
	API_FUNC_ENTER();
	platform_driver_unregister(&fm3_usbh_driver);
	API_FUNC_EXIT();
}
module_exit(fm3_usbh_cleanup);
