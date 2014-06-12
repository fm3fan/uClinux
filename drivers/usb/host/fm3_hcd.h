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

#ifndef FM3_HCD_H_
#define FM3_HCD_H_

#define FM3_USBH_TIMEOUT    100000
#define FM3_USBH_ERR_MAX    2
#define DEFAULT_MAX_PKS	64

#define USBH_VBUS_IO_VALID_LEVEL    ((uint8_t)0U)
#define USBH_MH_EP_INIT         ((uint16_t)0x4000)
#define USBH_MH_EOF_TIME        ((uint16_t)0x02C9)
#define USBH_MH_PKS_INIT        ((uint8_t)0x40)
#define USBH_MH_HRTIMER10_INIT  ((uint16_t)0xFFFF)
#define USBH_MH_HRTIMER2_INIT   ((uint8_t)0x03)
#define USBH_MH_EOF_TIME        ((uint16_t)0x02C9)

#define USBH_EP_DISABLE 0
#define USBH_EP_ENABLE  1
#define USBH_EP_TYPE_BULK   2
#define USBH_EP_TYPE_INT    3
#define USBH_EP_DIR_IN      0
#define USBH_EP_DIR_OUT     1

#define UPCR2_UPOWT_INIT_VALUE  (0x07)  /* initial value of UPCR2 register's UPOWT bits    */
#define UPCR3_UPLLK_INIT_VALUE  (0x00)  /* initial value of UPCR3 register's UPLLK bits    */
#define UPCR4_UPLLN_INIT_VALUE  (0x3B)  /* initial value of UPCR4 register's UPLLN bits    */
#define UPCR5_UPLLN_INIT_VALUE  (0x04)  /* initial value of UPCR4 register's UPLLM bits    */

#define USBH_MH_HANDSHAKE_ACK         (0x00)  /* ACK   */
#define USBH_MH_HANDSHAKE_NAK         (0x01)  /* NAK   */
#define USBH_MH_HANDSHAKE_STALL       (0x02)  /* STALL */
#define USBH_MH_HANDSHAKE_NULL        (0x03)  /* NULL  */
/* token type */
#define USBH_MH_TOKEN_SETUP           (1)     /* SETUP token */
#define USBH_MH_TOKEN_IN              (2)     /* IN token    */
#define USBH_MH_TOKEN_OUT             (3)     /* OUT token   */
#define USBH_MH_TOKEN_SOF             (4)     /* SOF token   */

/* TRANSFER REGISTERS:  host and peripheral sides are similar
 * except for the control models (master vs slave).
 */
//#define FM3_USBH_HOSTCTLREG    0

#   define FM3_USBH_HCTLMASK_ARM   0x01
#   define FM3_USBH_HCTLMASK_ENABLE    0x02
#   define FM3_USBH_HCTLMASK_IN    0x00
#   define FM3_USBH_HCTLMASK_OUT   0x04
#   define FM3_USBH_HCTLMASK_ISOCH 0x10
#   define FM3_USBH_HCTLMASK_AFTERSOF  0x20
#   define FM3_USBH_HCTLMASK_TOGGLE    0x40
#   define FM3_USBH_HCTLMASK_PREAMBLE  0x80

#   define FM3_USBH_STATMASK_ACK   0x01
#   define FM3_USBH_STATMASK_ERROR 0x02
#   define FM3_USBH_STATMASK_TMOUT 0x04
#   define FM3_USBH_STATMASK_SEQ   0x08
#   define FM3_USBH_STATMASK_SETUP 0x10
#   define FM3_USBH_STATMASK_OVF   0x20
#   define FM3_USBH_STATMASK_NAK   0x40
#   define FM3_USBH_STATMASK_STALL 0x80

#define LOG2_PERIODIC_SIZE  5   /* arbitrary; this matches OHCI */
#define PERIODIC_SIZE       (1 << LOG2_PERIODIC_SIZE)

struct fm3_usb {
	spinlock_t lock;
	//void __iomem	*addr_reg;
	//void __iomem	*data_reg;
	struct fm3_usb_platform_data  *board;
	struct proc_dir_entry *pde;
	/* sw model */
	struct timer_list timer;
	struct fm3_usbh_ep *next_periodic;
	struct fm3_usbh_ep *next_async;

	struct fm3_usbh_ep	*active_a;
	unsigned long	jiffies_a;
	struct fm3_usbh_ep	*active_b;
	unsigned long	jiffies_b;

	u32 hubport;
	u16 frame;

	/* async schedule: control, bulk */
	struct list_head async;

	/* periodic schedule: interrupt, iso */
	u16 load[PERIODIC_SIZE];
	struct fm3_usbh_ep *periodic[PERIODIC_SIZE];
	unsigned periodic_count;
};

static inline struct fm3_usb *hcd_to_fm3_usb(struct usb_hcd *hcd)
{
    return (struct fm3_usb *) (hcd->hcd_priv);
}

static inline struct usb_hcd *fm3_usb_to_hcd(struct fm3_usb *fm3_usb)
{
    return container_of((void *) fm3_usb, struct usb_hcd, hcd_priv);
}

struct fm3_usbh_ep {
	struct usb_host_endpoint *hep;
	struct usb_device *udev;

	u8 defctrl;
	u8 maxpacket;
	u8 epnum;
	u8 nextpid;

	u16 error_count;
	u16 nak_count;
	u16 length;     /* of current packet */

	/* periodic schedule */
	u16 period;
	u16 branch;
	u16 load;
	struct fm3_usbh_ep *next;

	/* async schedule */
	struct list_head schedule;
};

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#define DBG(stuff...)       printk(KERN_DEBUG "fm3_usb: " stuff)
#else
#define DBG(stuff...)       do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG        DBG
#else
#    define VDBG(stuff...)  do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET      VDBG
#else
#    define PACKET(stuff...)    do{}while(0)
#endif

#define ERR(stuff...)       printk(KERN_ERR "fm3_usb: " stuff)
#define WARNING(stuff...)   printk(KERN_WARNING "fm3_usb: " stuff)
#define INFO(stuff...)      printk(KERN_INFO "fm3_usb: " stuff)

#endif /* FM3_HCD_H_ */
