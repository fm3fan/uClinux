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

//#define FM3_ETH_DEBUG
//#define FM3_ETH_DEBUG_FUNC
//#define FM3_ETH_DEBUG_PHY
#define CONFIG_FM3_ETH_CH0
#define ONLY_DIRECT_AND_BROADCAST

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/fm3_eth.h>

#include <asm/setup.h>
#include <mach/fm3.h>
#include <mach/eth.h>

#ifdef FM3_ETH_DEBUG
#define dprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

#ifdef FM3_ETH_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#define fprintk(fmt, ...)	printk("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#define	fprintk(fmt, ...)
#endif

#define FM3_INFO		KERN_INFO FM3_ETH_DRV_NAME
#define FM3_DMA_MAX		0x3FFF

/* LAN8187C PHY Registers */
#define PHY_REG_BMCR    0x00    // Basic Mode Control Register
#define PHY_REG_BMSR    0x01    // Basic Mode Status Register
#define PHY_REG_IDR1    0x02    // PHY Identifier 1
#define PHY_REG_IDR2    0x03    // PHY Identifier 2
#define PHY_REG_STS     0x10    // Status Register
#define PHY_REG_19      19      // Extended Control Register 19
#define PHY_REG_LEDC    20
#define PHY_REG_INTC    22

#define BMCR_RESET      0x8000
#define BMSR_AUTO_DONE  0x0020

//#define PHY_AUTO_NEG  0x1000    // Select Auto Negotiation
#define PHY_AUTO_NEG    0x1200      // Select Auto Negotiation

#define STS_LINK_ON     0x1
#define STS_10_MBIT     0x2
#define STS_FULL_DUP    0x4

#define MII_WR_TOUT     0x010000    // MII Write timeout count
#define MII_RD_TOUT     0x010000    // MII Read timeout count

#define ICS1894_ADDR    0x10        // PHY device address for Wakamatsu FM3 LAN board
#define LAN8187_ADDR    0x06        // PHY device address for Wakamatsu ARM LAN board
#define DP83848_ADDR    0x01        // PHY device address for DP83848
#define LAN8700_ADDR    0x1F        // PHY device address for Will Electronics WX-PHY
#define ICS1894_ID      0x0015f450  // PHY Identifier of ICS1894
#define LAN8187_ID      0x0006C0C4  // PHY Identifier of LAN8187
#define DP83848_ID      0x20005C90  // PHY Identifier of DP83848
#define LAN8700_ID      0x0007C0C0  // PHY Identifier of LAN8700i
#define PHY_ADDR        ICS1894_ADDR
#define DEF_PHY_RESET_RETRY     3
#define DEF_PHY_AUTONEG_RETRY   1
#define DEF_PHY_RESET_STATUS_RETRY      10000
#define DEF_PHY_AUTONEG_STATUS_RETRY    100000

#define ETH_BUF_MIN
// DMA Register 0 (BMR)
#define DEF_DMA_PBL     16      // Programmable Burst Length
#define DEF_DMA_DSL     0       // Ring Mode
#define DEF_DMA_PR      1       // 1 -> Rx:Tx=2:1
// TX and RX descriptors info
#ifdef ETH_BUF_MIN
#define DEF_TXDESC_NUM  3       // Num of TX Descripter
#define DEF_RXDESC_NUM  4       // Num of RX Descripter
#define DEF_TX_BUF_NUM  DEF_TXDESC_NUM      // Num of TX buffer
#define DEF_TX_BUF_SIZE 1536                // Size of TX buffer
#define DEF_RX_BUF_NUM  DEF_RXDESC_NUM      // Num of RX buffer
#define DEF_RX_BUF_SIZE 1536                // Size of RX buffer
#else
#define DEF_TXDESC_NUM  4       // Num of TX Descripter
#define DEF_RXDESC_NUM  4       // Num of RX Descripter
#define DEF_TX_BUF_NUM  (DEF_TXDESC_NUM*2)  // Num of TX buffer
#define DEF_TX_BUF_SIZE 1536                // Size of TX buffer
#define DEF_RX_BUF_NUM  (DEF_RXDESC_NUM*2)  // Num of RX buffer
#define DEF_RX_BUF_SIZE 1536                // Size of RX buffer
#endif
#define MFFR_PROMISCUOUS                    // PROMISCUOUS mode

struct fm3_mac_regs {
	u32	maccr;		/* MAC configuration			      */
	u32	macffr;		/* MAC frame filter			      */
	u32	machthr;	/* MAC hash table high			      */
	u32	machtlr;	/* MAC hash table low			      */
	u32	macmiiar;	/* MAC MII address			      */
	u32	macmiidr;	/* MAC MII data				      */
	u32	macfcr;		/* MAC flow control			      */
	u32	macvlantr;	/* MAC VLAN tag				      */
	u32	rsv0[2];
	u32	macrwuffr;	/* MAC remote wakeup frame filter	      */
	u32	macpmtcsr;	/* MAC PMT control and status		      */
	u32	rsv1;
	u32	macdbgr;	/* MAC debug				      */
	u32	macsr;		/* MAC interrupt status			      */
	u32	macimr;		/* MAC interrupt mask			      */
	u32	maca0hr;	/* MAC address 0 high			      */
	u32	maca0lr;	/* MAC address 0 low			      */
	u32	maca1hr;	/* MAC address 1 high			      */
	u32	maca1lr;	/* MAC address 1 low			      */
	u32	maca2hr;	/* MAC address 2 high			      */
	u32	maca2lr;	/* MAC address 2 low			      */
	u32	maca3hr;	/* MAC address 3 high			      */
	u32	maca3lr;	/* MAC address 3 low			      */
	u32	rsv2[40];
	u32	mmccr;		/* MMC control				      */
	u32	mmcrir;		/* MMC receive interrupt		      */
	u32	mmctir;		/* MMC transmit interrupt		      */
	u32	mmcrimr;	/* MMC receive interrupt mask		      */
	u32	mmctimr;	/* MMC transmit interrupt mask		      */
	u32	rsv3[14];
	u32	mmctgfsccr;	/* MMC transmitted good frms after single col */
	u32	mmctgfmsccr;	/* MMC transmitted good frms after more col   */
	u32	rsv4[5];
	u32	mmctgfcr;	/* MMC transmitted good frames counter	      */
	u32	rsv5[10];
	u32	mmcrfcecr;	/* MMC received frames with CRC error counter */
	u32	mmcrfaecr;	/* MMC received frames with alignment error   */
	u32	rsv6[10];
	u32	mmcrgufcr;	/* MMC received good unicast frames counter   */
	u32	rsv7[334];
	u32	ptptscr;	/* PTP time stamp control		      */
	u32	ptpssir;	/* PTP subsecond increment		      */
	u32	ptptshr;	/* PTP time stamp high			      */
	u32	ptptslr;	/* PTP time stamp low			      */
	u32	ptptshur;	/* PTP time stamp high update		      */
	u32	ptptslur;	/* PTP time stamp low update		      */
	u32	ptptsar;	/* PTP time stamp addend		      */
	u32	ptptthr;	/* PTP target time high			      */
	u32	ptpttlr;	/* PTP target time low			      */
	u32	rsv8;
	u32	ptptssr;	/* PTP time stamp status		      */
	u32	ptpppscr;	/* PTP PPS control			      */
	u32	rsv9[564];
	u32	dmabmr;		/* DMA bus mode				      */
	u32	dmatpdr;	/* DMA transmit poll demand		      */
	u32	dmarpdr;	/* DMA receive poll demand		      */
	u32	dmardlar;	/* DMA receive descriptor list address	      */
	u32	dmatdlar;	/* DMA transmit descriptor list address	      */
	u32	dmasr;		/* DMA status				      */
	u32	dmaomr;		/* DMA operation mode			      */
	u32	dmaier;		/* DMA interrupt enable			      */
	u32	dmamfbocr;	/* DMA missed frame and buffer overflow	      */
	u32	dmarswtr;	/* DMA receive status watchdog timer	      */
	u32	rsv10[8];
	u32	dmachtdr;	/* DMA current host transmit descriptor	      */
	u32	dmachrdr;	/* DMA current host receive descriptor	      */
	u32	dmachtbar;	/* DMA current host transmit buffer address   */
	u32	dmachrbar;	/* DMA current host receive buffer address    */
};

FM3_USBETHERNETCLK_TypeDef *ethclk = FM3_USBETHERNETCLK;
FM3_ETHERNET_MAC_TypeDef *ethmac = FM3_ETHERNET_MAC0;
FM3_ETHERNET_CONTROL_TypeDef *ethctrl = FM3_ETHERNET_CONTROL;

char TxBuf[DEF_TX_BUF_NUM][DEF_TX_BUF_SIZE] __attribute__((aligned(4)));
char RxBuf[DEF_RX_BUF_NUM][DEF_RX_BUF_SIZE] __attribute__((aligned(4)));
EMAC_DMA_TXDESC txdesc[DEF_TXDESC_NUM] __attribute__((aligned(4)));
EMAC_DMA_RXDESC rxdesc[DEF_RXDESC_NUM] __attribute__((aligned(4)));
 //char tmpbuf[DEF_TX_BUF_SIZE] __attribute__((aligned(4)));
u32 txdesc_id = 0;    // Current index of TX Descriptor

 // PHY Address
 char PhyAddr[] = {
     ICS1894_ADDR,
     LAN8187_ADDR,
     DP83848_ADDR,
     LAN8700_ADDR
 };
 u32 PhyAddrIdx = -1;
 u32 phy_addr = PHY_ADDR;
 #define PHY_MAX (sizeof(PhyAddr)/sizeof(char))

 // MAC Address
 char MACAddr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
 char Broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

 /*
 * Private ethernet device data
 */
struct fm3_eth_priv {
	volatile struct fm3_mac_regs *regs;
	int irq;
	struct platform_device *pdev;
	struct net_device *dev;
	struct napi_struct napi;
	struct net_device_stats stat;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	u32 phy_id;
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	int link;
	int speed;
	int duplex;
	spinlock_t lock;
#if 0
	struct fm3_eth_dma_bd *rx_bd;
	dma_addr_t rx_bd_dma_addr;
	struct fm3_eth_dma_bd *tx_bd;
	dma_addr_t tx_bd_dma_addr;
#endif
#if 0
	struct sk_buff **rx_skb;
	struct sk_buff **tx_skb;
#endif
	u32 rx_done_idx;
	u32 tx_todo_idx;
	u32 tx_done_idx;
	u32 tx_pending;
	u32 tx_blocked;
	u32 frame_max_size;
	u32 rx_buf_num;
	u32 tx_buf_num;
};

static void fm3_eth_hw_stop(struct net_device *dev);
static void fm3_eth_buffers_free(struct net_device *dev);
static int  fm3_plat_remove(struct platform_device *pdev);

static void wait(volatile u32 count)
{
    while (count-- > 0) ;
}

// Initialize TX Descriptor
void TXDESC_Init(EMAC_DMA_TXDESC *txdesc, u32 b1ap, u32 b2ap)
{
    txdesc->TDES0 = 0;
    txdesc->TDES1_f.TBS1 = 0;
    txdesc->TDES1_f.TBS2 = 0;
    txdesc->TDES2 = b1ap;
    txdesc->TDES3 = b2ap;
    txdesc_id = 0;
}

// Initialize RX Desctiptor
void RXDESC_Init(EMAC_DMA_RXDESC *rxdesc, u32 b1ap, u32 b2ap)
{
    rxdesc->RDES0 = 0;
    rxdesc->RDES1_f.RBS1 = DEF_RX_BUF_SIZE;
#ifdef ETH_BUF_MIN
    rxdesc->RDES1_f.RBS2 = 0;
#else
    rxdesc->RDES1_f.RBS2 = DEF_RX_BUF_SIZE;
#endif
    rxdesc->RDES2 = b1ap;
    rxdesc->RDES3 = b2ap;
}

// Initialize TX and RX Descriptors
void DESC_Init(void)
{
    u32 i;
#ifdef ETH_BUF_MIN
    for (i=0; i<DEF_TXDESC_NUM; i++) {
        TXDESC_Init(&txdesc[i], (u32)&TxBuf[i], (u32)0);
        if (i == (DEF_TXDESC_NUM-1))
            txdesc[i].TDES0_f.TER = 1;
    }
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        RXDESC_Init(&rxdesc[i], (u32)&RxBuf[i], (u32)0);
        rxdesc[i].RDES0_f.OWN = 1;
        if (i == (DEF_RXDESC_NUM-1))
            rxdesc[i].RDES1_f.RER = 1;
    }
#else
    for (i=0; i<DEF_TXDESC_NUM; i++) {
        TXDESC_Init(&txdesc[i], (u32)&TxBuf[i*2], (u32)&TxBuf[i*2+1]);
        if (i == (DEF_TXDESC_NUM-1))
            txdesc[i].TDES0_f.TER = 1;
    }
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        RXDESC_Init(&rxdesc[i], (u32)&RxBuf[i*2], (u32)&RxBuf[i*2+1]);
        rxdesc[i].RDES0_f.OWN = 1;
        if (i == (DEF_RXDESC_NUM-1))
            rxdesc[i].RDES1_f.RER = 1;
    }
#endif
    ethmac->RDLAR = (u32)&rxdesc[0];
    ethmac->TDLAR = (u32)&txdesc[0];
}

#define DMA_TIMEOUT 1000

// Initialize Ethernet DMA
// Refer to Programming Guide P.170
int32_t DMA_Init(void)
{
    u32 dma_timeout = DMA_TIMEOUT;
    ethmac->BMR = DMA_BMR_SWR;                      // soft reset
    while ((ethmac->BMR & DMA_BMR_SWR) == 1) {      // wait for reset completion (== 0)
        if (dma_timeout-- == 0)
            return FALSE;
    }
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    ethmac->MCR |= GMAC_MCR_PS;                     // select MII/RMII interface (MUST)
    // DMA Bus Mode Register
    // Recommend: Mixed Burst=1 , AAL=0, FB = 0, Fixed burst or undefined burst.
#if 0
    ethmac->BMR &= ~(DMA_BMR_FB | DMA_BMR_AAL);
    ethmac->BMR |= (DMA_BMR_MB |                    // Mixed Burst = 1
            (DEF_DMA_PBL << DMA_BMR_B_PBL) |        // Programmable Burst Length
            (DEF_DMA_DSL << DMA_BMR_B_DSL) |        // Descriptor Skip Length
            DMA_BMR_DA |                            // DMA arbitration scheme
            (DEF_DMA_PR << DMA_BMR_B_PR));          // Priority Ratio
#else
    ethmac->BMR = 0x04025002;
#endif
    DESC_Init();
    // DMA Operation Mode Register
    // RSF (Receive Store and Forward)
    // TSF (Transmit Store Forward)
    // FUF (Forward Undersized Good Frames) Transfer less than 64 bytes
    ethmac->OMR = (DMA_OMR_RSF | DMA_OMR_TSF | DMA_OMR_FUF);
    ethmac->SR |= DMA_SR_NIS;                       // clear normal interrupt status
    // NIE (Normal Interrupt Summary Enable)
    // ERE (Early Receive Interrupt Enable)
    // RIE (Receive Interrupt Enable)
    // TIE (Transmit Interrupt)
#ifdef FM3_ETH_INT
    ethmac->IER = (DMA_IER_NIE |DMA_IER_TIE | DMA_IER_RIE | DMA_IER_ERE);
#endif
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    // SR DMA RX Running
    // ST DMA TX Running
    ethmac->OMR |= (DMA_OMR_SR | DMA_OMR_ST);
    //ethmac->OMR |= DMA_OMR_SR;
    return TRUE;
}

// Initialize GMAC
// GMAC : Ethernet Media Access Controller
int32_t GMAC_Init(uint8_t *pmac)
{
    ethmac->GAR |= GMAC_GAR_CR;             // CR:0001 -> SYS_CLK 100-150MHz SYS_CLK/62
    //if (PHY_Init() == FALSE)                // PHY Initialize
    //    return FALSE;
    //PHY_LinkSpeed();                        // Set LinkSpeed
    memcpy((void *)MACAddr, (void *)pmac, 6);
    ethmac->MAR0H = (u32)(*((uint16_t *)pmac));
    ethmac->MAR0L = (u32)(*((u32 *)(pmac+2)));
#ifdef MFFR_PROMISCUOUS
    ethmac->MFFR = (GMAC_MFFR_PR | GMAC_MFFR_RA);
#else
    ethmac->MAR1H = (u32)(*((uint16_t *)pmac)) | 0x80000000;
    ethmac->MAR1L = (u32)(*((u32 *)(pmac+2)));
    ethmac->MFFR &= ~(GMAC_MFFR_DB);
    ethmac->MFFR |= (GMAC_MFFR_HUC | GMAC_MFFR_RA);
#endif
    ethmac->MCR |= GMAC_MCR_TE | GMAC_MCR_RE;   // enable transmit and receive
    return TRUE;
}

// Initialize Ethernet Control registers
void ETHCTRL_Init(void)
{
	//FUNC_ENTER();
    ethctrl->ETH_CLKG = (3 << ETH_CLKG_MACEN);      // start EMAC clock
#ifdef CONFIG_FM3_ETH_CH0
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and reset EMAC
            (1 << ETH_MODE_RST0);
    wait(500000);
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and start EMAC
            (0 << ETH_MODE_RST0);
#endif
#ifdef CONFIG_FM3_ETH_CH1
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and reset EMAC
            (1 << ETH_MODE_RST1);
    wait(500000);
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and start EMAC
            (0 << ETH_MODE_RST1);
#endif
    //FUNC_EXIT();
}

// Get an available TX descriptor
EMAC_DMA_TXDESC *TXDESC_Available(void)
{
    int32_t i = DEF_TXDESC_NUM;
    EMAC_DMA_TXDESC *ptxdesc;
    while (i-- > 0) {
        ptxdesc = (EMAC_DMA_TXDESC *)&txdesc[txdesc_id];
        txdesc_id++;
        if (txdesc_id == DEF_TXDESC_NUM)
            txdesc_id = 0;
        if (ptxdesc->TDES0_f.OWN == 0)
            return ptxdesc;
    }
    return (EMAC_DMA_TXDESC *)NULL;
}

// Get a received RX descriptor
EMAC_DMA_RXDESC *RXDESC_Received(void)
{
    int32_t i;
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        if (rxdesc[i].RDES0_f.OWN == 0) {
            return (EMAC_DMA_RXDESC *)&rxdesc[i];
        }
    }
    return (EMAC_DMA_RXDESC *)NULL;
}

void fm3_eth_send_packet(uint8_t *p, u32 size)
{
	//FUNC_ENTER();
    EMAC_DMA_TXDESC *ptxdesc;
    while ((ptxdesc = TXDESC_Available()) == NULL);
    uint8_t *dst = (uint8_t *)ptxdesc->TDES2;
    memcpy((void *)dst, (void *)p, size);
    //ptxdesc->TDES0_f.CIC = 3;           // Automatically adding checksum
    ptxdesc->TDES0_f.CIC = 0;           // Not automatically adding checksum
    //ptxdesc->TDES0_f.IC = 1;          //@TX interrupt invoked after sending current frame
    ptxdesc->TDES0_f.FS = 1;            // Buffer includes the first frame
    ptxdesc->TDES0_f.LS = 1;            // Buffer includes the last frame
    ptxdesc->TDES1_f.TBS1 = size;       // Size of TX
    ptxdesc->TDES0_f.OWN = 1;           // Set descriptor to DMA own to start TX operation
    ethmac->OMR_f.ST = 1;               // Set DMA to run state
    ethmac->TPDR = (u32)0;         // Request polling descriptors to restart TX operation
    dprintk("tlen=%d\n", size);
    //FUNC_EXIT();
}

#ifdef DEBUG_EMAC_REG_DUMP
u32 phy_register_mask = 0b00000011111111110000000111111111;

// Dump PHY registers
void PHY_Register_Dump(void)
{
    u32 value;
    u32 bit_mask = phy_register_mask;
    int32_t i;
    for (i = 0; i < 31; i++) {
        PHY_Read(phy_addr, (u32)i, &value, MII_RD_TOUT);
        if (bit_mask & 0x1)
            printk("Reg %02d: %04x\r\n", i, value);
        bit_mask >>= 1;
    }
}
#endif

static int fm3_eth_hw_init(struct net_device *dev)
{
	int ret = 0;
	//FUNC_ENTER();
    DESC_Init();
#if 0
    ethmac->BMR &= ~(DMA_BMR_FB | DMA_BMR_AAL);
    ethmac->BMR |= (DMA_BMR_MB |                    // Mixed Burst = 1
            (DEF_DMA_PBL << DMA_BMR_B_PBL) |        // Programmable Burst Length
            (DEF_DMA_DSL << DMA_BMR_B_DSL) |        // Descriptor Skip Length
            DMA_BMR_DA |                            // DMA arbitration scheme
            (DEF_DMA_PR << DMA_BMR_B_PR));          // Priority Ratio
#else
    ethmac->BMR = 0x04025002;
#endif
    // DMA Operation Mode Register
    // RSF (Receive Store and Forward)
    // TSF (Transmit Store Forward)
    // FUF (Forward Undersized Good Frames) Transfer less than 64 bytes
    ethmac->OMR = (DMA_OMR_RSF | DMA_OMR_TSF | DMA_OMR_FUF);
    ethmac->SR |= DMA_SR_NIS;                       // clear normal interrupt status
    // NIE (Normal Interrupt Summary Enable)
    // ERE (Early Receive Interrupt Enable)
    // RIE (Receive Interrupt Enable)
    // TIE (Transmit Interrupt)
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    // SR DMA RX Running
    // ST DMA TX Running
    ethmac->OMR |= (DMA_OMR_SR | DMA_OMR_ST);
    //ethmac->OMR |= DMA_OMR_SR;
    ethmac->GAR |= GMAC_GAR_CR;             // CR:0001 -> SYS_CLK 100-150MHz SYS_CLK/62
    //if (PHY_Init() == FALSE)                // PHY Initialize
    //    return FALSE;
    //PHY_LinkSpeed();                        // Set LinkSpeed
    memcpy((void *)MACAddr, (void *)dev->dev_addr, 6);
	printk("ethaddr=%02X:%02X:%02X:%02X:%02X:%02X\n",
			dev->dev_addr[0],dev->dev_addr[1],dev->dev_addr[2],dev->dev_addr[3],dev->dev_addr[4],dev->dev_addr[5]);    ethmac->MAR0H = (u32)(*((uint16_t *)&MACAddr[0]));
    ethmac->MAR0L = (u32)(*((u32 *)(&MACAddr[2])));
#ifdef MFFR_PROMISCUOUS
    ethmac->MFFR = (GMAC_MFFR_PR | GMAC_MFFR_RA);
#else
    ethmac->MAR1H = (u32)(*((uint16_t *)pmac)) | 0x80000000;
    ethmac->MAR1L = (u32)(*((u32 *)(pmac+2)));
    ethmac->MFFR &= ~(GMAC_MFFR_DB);
    ethmac->MFFR |= (GMAC_MFFR_HUC | GMAC_MFFR_RA);
#endif
fm3_eth_hw_init_exit:
	//dprintk("ret=%d\n", ret);
    //FUNC_EXIT();
    return 0;
}

static void fm3_eth_hw_stop(struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
	FUNC_ENTER();
	FUNC_EXIT();
}

/*
 * Allocate buffer and descriptors necessary for net device
 */
static int fm3_eth_buffers_alloc(struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
	int ret = 0;
	int i;
	FUNC_ENTER();
	FUNC_EXIT();
	return ret;
}

static void fm3_eth_buffers_free(struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
	int i;
	FUNC_ENTER();
	FUNC_EXIT();
	return;
}

static int fm3_eth_rx_get(struct net_device *dev, int processed, int budget)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
    EMAC_DMA_RXDESC *prxdesc;
    int32_t recvd = 0;
	FUNC_ENTER();
	while (processed < budget) {
		struct sk_buff *skb;
		u32 stat, len, idx;
	    while ((prxdesc = RXDESC_Received()) != NULL) {
	        recvd = prxdesc->RDES0_f.FL;
	        skb = netdev_alloc_skb_ip_align(dev, len);
			if (unlikely(!skb)) {
				fm3->stat.rx_dropped++;
				goto next;
			}
			memcpy(skb_put(skb, len), (void *)prxdesc->RDES2, len);
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			fm3->stat.rx_packets++;
			fm3->stat.rx_bytes += len;
	    }
next:
		prxdesc->RDES0_f.OWN = 1;
	}
	FUNC_EXIT();
	return processed;
}

static int fm3_eth_rx_poll(struct napi_struct *napi, int budget)
{
	struct fm3_eth_priv *fm3 = container_of(napi, struct fm3_eth_priv, napi);
	struct net_device *dev = fm3->dev;
	unsigned long flags;
	int ret = 0;
	int more;
	FUNC_ENTER();
	do {
		more = 0;
		ret = fm3_eth_rx_get(dev, ret, budget);
		if (!(ret < budget)) {
			// ToDo: enable RIE
			break;
		}
		spin_lock_irqsave(&fm3->rx_lock, flags);
		__napi_complete(napi);
		// ToDo: enable RIE
			more = 1;
		spin_unlock_irqrestore(&fm3->rx_lock, flags);
	} while (more && napi_reschedule(napi));
	FUNC_EXIT();
	return ret;
}

static void fm3_eth_tx_complete(struct net_device *dev)
{
	struct fm3_eth_priv	*fm3 = netdev_priv(dev);
	FUNC_ENTER();
	FUNC_EXIT();
}

static void fm3_eth_recv_packet(struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
    FM3_ETHERNET_MAC_TypeDef *pethmac = FM3_ETHERNET_MAC0;
    EMAC_DMA_RXDESC *prxdesc = 0;
    int len;
    struct sk_buff *skb;
    //FUNC_ENTER();
    while ((prxdesc = RXDESC_Received()) != NULL) {
    	len = prxdesc->RDES0_f.FL;
    	dprintk("rlen=%d\n", len);
    	skb = dev_alloc_skb(len + 2);
    	if (skb) {
    		skb_reserve(skb, 2);
    		//skb_copy_to_linear_data(skb, (void *)prxdesc->RDES2, len);
    		//skb_put(skb, len);
    		memcpy(skb_put(skb, len), (void *)prxdesc->RDES2, len);
    		skb->protocol = eth_type_trans(skb, dev);
    		netif_rx(skb);
    		fm3->stat.rx_packets++;
    		fm3->stat.rx_bytes += len;
        	prxdesc->RDES0_f.OWN = 1;
    	} else {
    		fm3->stat.rx_dropped++;
    		printk(KERN_NOTICE "%s: Memory squeeze, dropping packet.\n", dev->name);
    		break;
    	}
    }
    //FUNC_EXIT();
}

static irqreturn_t fm3_eth_irq(int irq, void *dev_id)
{
	irqreturn_t ret;
	u32 status;
    int32_t i;
    struct net_device *dev = (struct net_device *) dev_id;
    FM3_ETHERNET_MAC_TypeDef *pethmac = FM3_ETHERNET_MAC0;
    //EMAC_DMA_TXDESC *ptxdesc = 0;
    EMAC_DMA_RXDESC *prxdesc = 0;
    u32 gmac_isr = pethmac->ISR;
    u32 dma_sr = pethmac->SR;
    if ((dma_sr & DMA_SR_RI) != 0) {
        if ((prxdesc = RXDESC_Received()) != NULL) {
            pethmac->SR |= DMA_SR_RI;
#ifdef ONLY_DIRECT_AND_BROADCAST
            struct eth_packet *ethp = (struct eth_packet *)prxdesc->RDES2;
            if ((strncmp((const char*)ethp, (const char*)&MACAddr[0], 6) == 0) ||
                (strncmp((const char*)ethp, (const char*)&Broadcast[0], 6) == 0))
            {
                fm3_eth_recv_packet(dev);
            } else {
                prxdesc->RDES0_f.OWN = 1;
            }
#else
            fm3_eth_recv_packet(dev);
#endif
        }
    } else {
    	fprintk("t.");
        if ((dma_sr & DMA_SR_TU) != 0) {
            // TU (Transmit Buffer Unavailable)
            // For continuously sending
            // TODO
            pethmac->SR |= (DMA_SR_TU);
            printk("TU\n");
        } else if ((dma_sr & DMA_SR_TI) != 0) {
            // TI TI (Transmit Interrupt)
            // For continuously sending
            // TODO
            //ethmac->SR |= (DMA_SR_TI);
            printk("TI\n");
        } else if ((dma_sr & DMA_SR_ERI) != 0) {
            // ERI (Early Receive Interrupt))
            //  For DMA, indicate DMA receives the first packet
            // ToDo
            //printk("ISR:%08x DSR:%08x\r\n", gmac_isr, dma_sr);
        } else {
            // For debugging
            printk("ISR:%08x DSR:%08x\r\n", gmac_isr, dma_sr);
        }
	}
	ret = IRQ_HANDLED;
	return ret;
}

static void fm3_phy_start(void)
{
	FUNC_ENTER();
	FUNC_EXIT();
}

static int fm3_netdev_open(struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
	int ret = 0;
	//FUNC_ENTER();
	ret = fm3_eth_hw_init(dev);
	if (ret != 0)
		goto fm3_netdev_open_exit;
	ret = fm3_eth_buffers_alloc(dev);
	if (ret != 0)
		goto fm3_netdev_open_exit;
	napi_enable(&fm3->napi);
	spin_lock_init(&fm3->rx_lock);
	spin_lock_init(&fm3->tx_lock);
	fm3->rx_done_idx = 0;
	fm3->tx_todo_idx = 0;
	fm3->tx_done_idx = 0;
	fm3->tx_pending = 0;
	fm3->tx_blocked = 0;
	ret = request_irq(fm3->irq, fm3_eth_irq, IRQF_SHARED, dev->name, dev);
	if (ret) {
		napi_disable(&fm3->napi);
		fm3_eth_hw_stop(dev);
		fm3_eth_buffers_free(dev);
		goto fm3_netdev_open_exit;
	}
	netif_start_queue(dev);
	// enable interrupt
//#ifdef FM3_ETH_INT
    ethmac->MCR |= GMAC_MCR_TE | GMAC_MCR_RE;   // enable transmit and receive
    ethmac->IER = (DMA_IER_NIE |DMA_IER_TIE | DMA_IER_RIE | DMA_IER_ERE);
//#endif
fm3_netdev_open_exit:
	//dprintk("ret=%d\n", ret);
	//FUNC_EXIT();
	return ret;
}

static int fm3_netdev_close(struct net_device *dev)
{
	struct fm3_eth_priv	*fm3 = netdev_priv(dev);
	FUNC_ENTER();
	napi_disable(&fm3->napi);
	netif_stop_queue(dev);
	// ToDo DMA stop
	free_irq(fm3->irq, dev);
	fm3_eth_hw_stop(dev);
	fm3_eth_buffers_free(dev);
	FUNC_EXIT();
	return 0;
}

static int fm3_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(dev);
	unsigned long flags;
	int idx;
	int ret = 0;
	//FUNC_ENTER();
	if (unlikely(skb->len > fm3->frame_max_size)) {
		fm3->stat.tx_dropped++;
		dev_kfree_skb(skb);
		printk("tdrop\n");
		ret = NETDEV_TX_OK;
		goto fm3_netdev_xmit_exit;
	}
	dev->trans_start = jiffies;
	fm3_eth_send_packet(skb->data, skb->len);
	fm3->stat.tx_packets++;
	fm3->stat.tx_bytes += skb->len;
	dev_kfree_skb(skb);
	ret = NETDEV_TX_OK;
fm3_netdev_xmit_exit:
	//dprintk("ret=%d\n", ret);
	//FUNC_EXIT();
	return ret;
}

static struct net_device_stats *fm3_netdev_get_stats(struct net_device *dev)
{
	struct fm3_eth_priv	*fm3 = netdev_priv(dev);
	//FUNC_ENTER();
	if (netif_running(dev)) {
	}
	//FUNC_EXIT();
	return &fm3->stat;
}

static int fm3_netdev_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct fm3_eth_priv	*fm3 = netdev_priv(dev);
	struct phy_device *phydev = fm3->phy_dev;
	int ret = 0;

	FUNC_ENTER();
	if (!netif_running(dev)) {
		ret = -EINVAL;
		goto fm3_netdev_ioctl_exit;
	}
	if (!phydev) {
		ret = -ENODEV;
		goto fm3_netdev_ioctl_exit;
	}
	ret = phy_mii_ioctl(phydev, if_mii(ifr), cmd);
fm3_netdev_ioctl_exit:
	FUNC_EXIT();
	return ret;
}

int fm3_validate_addr(struct net_device *dev)
{
	int ret;
	//FUNC_ENTER();
	ret = eth_validate_addr(dev);
	//FUNC_EXIT();
	fprintk("%s ret=%d\n", __func__, ret);
	return ret;
}

int fm3_change_mtu(struct net_device *dev, int new_mtu)
{
	int ret;
	//FUNC_ENTER();
	ret = eth_change_mtu(dev, new_mtu);
	//FUNC_EXIT();
	fprintk("%s ret=%d\n", __func__, ret);
	return ret;
}

int fm3_mac_addr(struct net_device *dev, void *p)
{
	int ret;
	//FUNC_ENTER();
	ret = eth_mac_addr(dev, p);
	//FUNC_EXIT();
	fprintk("%s ret=%d\n", __func__, ret);
	return ret;
}

static const struct net_device_ops	fm3_netdev_ops = {
	.ndo_open		= fm3_netdev_open,
	.ndo_stop		= fm3_netdev_close,
	.ndo_start_xmit		= fm3_netdev_xmit,
	.ndo_get_stats		= fm3_netdev_get_stats,
	.ndo_do_ioctl		= fm3_netdev_ioctl,

#ifdef FM3_ETH_DEBUG_FUNC
	.ndo_validate_addr	= fm3_validate_addr,
	.ndo_change_mtu		= fm3_change_mtu,
	.ndo_set_mac_address	= fm3_mac_addr,
#else
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
#endif
};

static int fm3_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 data)
{
	struct fm3_eth_priv *fm3 = bus->priv;
	int ret = 0;
	int timeout = MII_RD_TOUT;
#ifdef FM3_ETH_DEBUG_PHY
	FUNC_ENTER();
#endif
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (timeout-- == 0) {
            ret = 1;
            goto fm3_mdio_write_exit;
        }
    }
    ethmac->GDR = (uint16_t)data;
    ethmac->GAR &= ~((0x1f << GMAC_GAR_B_PA) | (0x1f << GMAC_GAR_B_GR));
    ethmac->GAR |= ((phy_id << GMAC_GAR_B_PA) | (reg << GMAC_GAR_B_GR));
    ethmac->GAR |=  ((1 << GMAC_GAR_B_GW) |(1 << GMAC_GAR_B_GB));
fm3_mdio_write_exit:
#ifdef FM3_ETH_DEBUG_PHY
	fprintk("%s ret=%d id=%02x reg=%02x data=%04x\n", __func__, ret, phy_id, reg, data);
    FUNC_EXIT();
#endif
    return ret;
}

static int fm3_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct fm3_eth_priv *fm3 = bus->priv;
	int value = 0;
#ifdef FM3_ETH_DEBUG_PHY
	FUNC_ENTER();
#endif
    u32 count = MII_RD_TOUT;
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (count-- == 0) {
        	value = -1;
            goto fm3_mdio_read_exit;
        }
    }
    ethmac->GAR &= ~((0x1f << GMAC_GAR_B_PA) | (0x1f << GMAC_GAR_B_GR));
    ethmac->GAR |= ((phy_id << GMAC_GAR_B_PA) | (reg << GMAC_GAR_B_GR));
    ethmac->GAR &=  ~(1 << GMAC_GAR_B_GW);
    ethmac->GAR |= (1 << GMAC_GAR_B_GB);
    count = MII_RD_TOUT;
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (count-- == 0) {
        	value = -1;
            goto fm3_mdio_read_exit;
        }
    }
    value = (u32)ethmac->GDR;
fm3_mdio_read_exit:
#ifdef FM3_ETH_DEBUG_PHY
	fprintk("%s ret=%d id=%02x reg=%02x data=%04x\n", __func__, ret, phy_id, reg, value);
    FUNC_EXIT();
#endif
    return value;
}

static int fm3_phy_auto_negotiate(struct mii_bus *bus, int phy_id)
{
    u32 value;
    int32_t flag = 1;
    u32 reset_retry = DEF_PHY_RESET_RETRY;
    while (reset_retry-- > 0) {
        if (fm3_mdio_write(bus, phy_id, PHY_REG_BMCR, PHY_AUTO_NEG) == 0) {
            u32 status_retry = DEF_PHY_AUTONEG_STATUS_RETRY;
            while (status_retry-- > 0) {
                if ((value = fm3_mdio_read(bus, phy_addr, PHY_REG_BMSR)) != -1) {
                    if (value & BMSR_AUTO_DONE) {
                        flag = 0;
                        break;
                    }
                }
            }
            if (flag == 0)
                break;
        }
    }
    if (flag == 0)
        printk("PHY: AutoNegotiate OK\r\n");
    else
        printk("PHY: AutoNegotiate NG\r\n");
    return flag;
}

static int fm3_phy_link_speed(struct mii_bus *bus, int phy_id)
{
    int value;
    int32_t full_duplex, mbit_100;

    printk("Waiting for auto negotiation...\r\n");
    if (fm3_phy_auto_negotiate(bus, phy_id) != 0) {
        mbit_100 = TRUE;
        full_duplex = TRUE;
        fm3_mdio_write(bus, phy_id, PHY_REG_BMCR, 0x3100);
        ethmac->MCR |= (GMAC_MCR_FES | GMAC_MCR_DM);
        printk("LinkSpeed: Full Duplex\r\n");
        printk("LinkSpeed: 100Mbs\r\n");
    } else {
        if ((value = fm3_mdio_read(bus, phy_id, PHY_REG_BMSR)) == -1){
            return -1;
        }
        if (value & STS_FULL_DUP) {
            printk("LinkSpeed: Full Duplex\r\n");
            full_duplex = TRUE;
        } else {
            printk("LinkSpeed: Half Duplex\r\n");
            full_duplex = FALSE;
        }
        if (value & STS_10_MBIT) {
            printk("LinkSpeed: 10Mbs\r\n");
            mbit_100 = FALSE;
        } else {
            printk("LinkSpeed: 100Mbs\r\n");
            mbit_100 = TRUE;
            fm3_mdio_write(bus, phy_id, PHY_REG_BMCR, 0x2100);
            ethmac->MCR |= (GMAC_MCR_FES | GMAC_MCR_DM);
        }
    }
    return 0;
}

static void fm3_params_setup(struct fm3_eth_priv *fm3)
{
	FUNC_ENTER();
	FUNC_EXIT();
}

static void fm3_handle_link_change(struct net_device *ndev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(ndev);
	struct phy_device *phydev = fm3->phy_dev;
	unsigned long flags;
	FUNC_ENTER();
	s32 status_change = 0;
	spin_lock_irqsave(&fm3->lock, flags);
	if (phydev->link) {
		if ((fm3->speed != phydev->speed) ||
		    (fm3->duplex != phydev->duplex)) {
			fm3->speed = phydev->speed;
			fm3->duplex = phydev->duplex;
			status_change = 1;
		}
	}
	if (phydev->link != fm3->link) {
		if (!phydev->link) {
			fm3->speed = 0;
			fm3->duplex = -1;
		}
		fm3->link = phydev->link;
		status_change = 1;
	}
	spin_unlock_irqrestore(&fm3->lock, flags);
	if (status_change) {
		phy_print_status(phydev);
		fm3_params_setup(fm3);
	}
	FUNC_EXIT();
}

static int fm3_mii_probe(struct net_device *ndev)
{
	struct fm3_eth_priv *fm3 = netdev_priv(ndev);
	struct phy_device *phydev = NULL;
	int phy_addr;

	FUNC_ENTER();
	for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++) {
		if (fm3->mii_bus->phy_map[phy_addr]) {
			phydev = fm3->mii_bus->phy_map[phy_addr];
			printk(KERN_INFO "found PHY id 0x%x addr %d\n", phydev->phy_id, phydev->addr);
			break;
		}
	}
	if (!phydev) {
		printk(KERN_ERR "%s: no PHY found\n", ndev->name);
		return -ENODEV;
	}
	printk(KERN_INFO "%s: using MII interface\n", ndev->name);
	phydev = phy_connect(ndev, dev_name(&phydev->dev), &fm3_handle_link_change, 0, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", ndev->name);
		return PTR_ERR(phydev);
	}
	fm3_phy_link_speed(fm3->mii_bus, phydev->phy_id);
	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = phydev->supported;
	fm3->link = 0;
	fm3->speed = 0;
	fm3->duplex = -1;
	fm3->phy_dev = phydev;
	FUNC_EXIT();
	return 0;
}

static int fm3_mii_init(struct fm3_eth_priv	*fm3)
{
	int err = -ENOMEM;
	int i;

#ifdef FM3_ETH_DEBUG_PHY
	FUNC_ENTER();
#endif
	fm3->mii_bus = mdiobus_alloc();
	if (!fm3->mii_bus) {
		goto err_out;
	}
	fm3->mii_bus->name = "fm3_mii_bus";
	fm3->mii_bus->read = &fm3_mdio_read;
	fm3->mii_bus->write = &fm3_mdio_write;
	snprintf(fm3->mii_bus->id, MII_BUS_ID_SIZE, "%02x", fm3->pdev->id);
	fm3->mii_bus->priv = fm3;
	fm3->mii_bus->parent = &fm3->pdev->dev;
	fm3->mii_bus->phy_mask = 0;
	fm3->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!fm3->mii_bus->irq) {
		goto err_out_1;
	}
	for (i = 0; i < PHY_MAX_ADDR; i++)
		fm3->mii_bus->irq[i] = PHY_POLL;
	err = mdiobus_register(fm3->mii_bus);
	if (err) {
		goto err_out_free_mdio_irq;
	}
	err = fm3_mii_probe(fm3->dev);
	if (err) {
		goto err_out_unregister_bus;
	}
	err = 0;
	goto err_out;
err_out_unregister_bus:
	mdiobus_unregister(fm3->mii_bus);
err_out_free_mdio_irq:
	kfree(fm3->mii_bus->irq);
err_out_1:
	mdiobus_free(fm3->mii_bus);
err_out:
#ifdef FM3_ETH_DEBUG_PHY
	fprintk("%s ret=%d\n", __func__, err);
	FUNC_EXIT();
#endif
	return err;
}

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/
static void fm3_eth_dma_reset(void)
{
    u32 dma_timeout = DMA_TIMEOUT;
    ethmac->BMR = DMA_BMR_SWR;                      // soft reset
    while ((ethmac->BMR & DMA_BMR_SWR) == 1) {      // wait for reset completion (== 0)
        if (dma_timeout-- == 0) {
        	printk("%s failed\n", __func__);
            return;
        }
    }
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    ethmac->MCR |= GMAC_MCR_PS;                     // select MII/RMII interface (MUST)
}

static int __init fm3_plat_probe(struct platform_device *pdev)
{
	struct fm3_eth_data *data;
	struct fm3_eth_priv *fm3;
	struct net_device *dev;
	struct resource	 *rs;
	struct phy_device *phydev;
	char *p;
	int ret = 0;

	//FUNC_ENTER();
	ETHCTRL_Init();
	//fm3_eth_dma_reset();
	DMA_Init();
	if (!pdev) {
		printk(FM3_INFO ": no device specified\n");
		ret = -EINVAL;
		goto out;
	}
	data = pdev->dev.platform_data;
	dev = alloc_etherdev(sizeof(struct fm3_eth_priv));
	if (!dev) {
		printk(FM3_INFO ": etherdev allocation failed\n");
		ret = -ENOMEM;
		goto out;
	}
	SET_NETDEV_DEV(dev, &pdev->dev);
	p = strnstr(boot_command_line, "ethaddr=", COMMAND_LINE_SIZE);
	if (p) {
		char ethaddr[18];
		int i;
		memcpy(ethaddr, &p[strlen("ethaddr=")], sizeof(ethaddr));
		p = ethaddr;
		for (i = 0; i < ETH_ALEN; i++) {
			dev->dev_addr[i] = (simple_strtol(p, &p, 16) << 0) |
					   (simple_strtol(p, &p, 16) << 4);
			p++; /* skip ":" in  ethaddr */
		}
	} else {
		memcpy(dev->dev_addr, data->mac_addr, ETH_ALEN);
	}
	if (!is_valid_ether_addr(dev->dev_addr)) {
		printk(FM3_INFO ": ethernet address is not set or invalid, using random.\n");
		random_ether_addr(dev->dev_addr);
	}
	//dprintk("ethaddr=%02X:%02X:%02X:%02X:%02X:%02X\n",
	//		dev->dev_addr[0],dev->dev_addr[1],dev->dev_addr[2],dev->dev_addr[3],dev->dev_addr[4],dev->dev_addr[5]);
	dev->netdev_ops = &fm3_netdev_ops;
	fm3 = netdev_priv(dev);
	fm3->dev = dev;
	fm3->pdev = pdev;
	netif_napi_add(dev, &fm3->napi, fm3_eth_rx_poll, 64);
	platform_set_drvdata(pdev, dev);
	rs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rs) {
		printk(FM3_INFO ": MAC reg base isn't specified\n");
		ret = -EINVAL;
		goto out;
	}
	fm3->regs = (struct fm3_mac_regs *)rs->start;
	rs = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!rs) {
		printk(FM3_INFO ": MAC IRQ isn't specified\n");
		ret= -EINVAL;
		goto out;
	}
	fm3->irq = rs->start;
	printk(FM3_INFO ": found MAC at 0x%x, irq %d\n", (s32)fm3->regs, fm3->irq);
	//fm3->frame_max_size = data->frame_max_size;
	fm3->frame_max_size = DEF_RX_BUF_SIZE;
	if (fm3->frame_max_size < 64 ||
	    fm3->frame_max_size > FM3_DMA_MAX) {
		printk(FM3_INFO ": incorrect frame_max_size param value\n");
		ret = -EINVAL;
		goto out;
	}
	fm3->rx_buf_num = data->rx_buf_num;
	fm3->tx_buf_num = data->tx_buf_num;
	if (!fm3->tx_buf_num || !fm3->rx_buf_num) {
		printk(FM3_INFO ": incorrect xx_buf_num param value\n");
		ret = -EINVAL;
		goto out;
	}
#if 0
	fm3->rx_skb = kmalloc(fm3->rx_buf_num * sizeof(void *), GFP_KERNEL);
	fm3->tx_skb = kmalloc(fm3->tx_buf_num * sizeof(void *), GFP_KERNEL);
	if (!fm3->rx_skb || !fm3->tx_skb) {
		printk(FM3_INFO ": rx/tx (%d/%d) bufs allocation failed\n",
			fm3->rx_buf_num, fm3->tx_buf_num);
		ret = -ENOMEM;
		goto out;
	}
#endif
	ret = register_netdev(dev);
	if (ret) {
		printk(FM3_INFO ": netdev registration failed\n");
		ret = -ENODEV;
		goto out;
	}
	if ((ret = fm3_mii_init(fm3)) != 0) {
		goto out;
	}
	phydev = fm3->phy_dev;
	printk(KERN_INFO "%s: attached PHY driver [%s] "
		"(mii_bus:phy_addr=%s, irq=%d)\n",
		dev->name, phydev->drv->name, dev_name(&phydev->dev), phydev->irq);
	GMAC_Init((uint8_t *)&MACAddr[0]);
	ret = 0;
out:
	if (ret != 0)
		fm3_plat_remove(pdev);
	fprintk("%s ret=%d\n", __func__, ret);
	//FUNC_EXIT();
	return ret;
}

static int fm3_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct fm3_eth_priv *fm3;

	FUNC_ENTER();
	if (!pdev)
		goto out;
	dev = platform_get_drvdata(pdev);
	if (!dev)
		goto out;
	platform_set_drvdata(pdev, NULL);
	fm3 = netdev_priv(dev);
	unregister_netdev(dev);
	fm3_eth_buffers_free(dev);
#if 0
	if (fm3->tx_skb) {
		kfree(fm3->tx_skb);
		fm3->tx_skb = NULL;
	}
	if (fm3->rx_skb) {
		kfree(fm3->rx_skb);
		fm3->rx_skb = NULL;
	}
#endif
	free_netdev(dev);
out:
	FUNC_EXIT();
	return 0;
}

static struct platform_driver fm3_eth_driver = {
	.probe		= fm3_plat_probe,
	.remove		= fm3_plat_remove,
	.driver		= {
		.name	= FM3_ETH_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init fm3_eth_drv_init(void)
{
	return platform_driver_register(&fm3_eth_driver);
}

static void __exit fm3_eth_drv_exit(void)
{
	printk(FM3_INFO ": cleanup\n");
	platform_driver_unregister(&fm3_eth_driver);
}

module_init(fm3_eth_drv_init);
module_exit(fm3_eth_drv_exit);

MODULE_ALIAS("platform:" FM3_ETH_DRV_NAME);
MODULE_DESCRIPTION("FM3 MAC driver");
MODULE_AUTHOR("Kentaro Sekimoto");
MODULE_LICENSE("GPL");
