/*
 * (C) Copyright 2013
 * Kentaro Sekimoto
 *
 * Based on emcraft implementation.
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

#ifndef FM3_ETH_H_
#define FM3_ETH_H_

// Pin  CH  Pin name
// PC0: 1   E_RXDV1
// PC1: 1   E_RX11
// PC2: 1   E_RX10
// PC3: 0   E_RX01
// PC4: 0   E_RX00
// PC5: 0   E_RXDV0
// PC6: 0   E_MDIO0
// PC7: 0   E_MDC0
// PC8: 01  E_RXCK0_REFCK
// PC9:
// PCA:
// PCB:
// PCC: 1   E_MDIO1
// PCD: 1   E_MDC1
// PCE: 1   E_TXEN1
// PCF: 1   E_TX11
// PD0: 1   E_TX10
// PD1: 0   E_TX01
// PD2: 0   E_TX00
// PD3: 0   E_TXEN0

//#define PFRC_ETH0 0x01F8
//#define PFRD_ETH0 0x000E
#define PFRC_ETH    0xF1FF
#define PFRD_ETH    0x000F

#define B_E_TD0E    18
#define B_E_TD1E    19
#define B_E_TE0E    20
#define B_E_TE1E    21
#define B_E_MC0E    22
#define B_E_MC1B    23
#define B_E_MD0B    24
#define B_E_MD1B    25
#define B_E_CKE     26
#define B_E_PSE     27
#define B_E_SPLC    28

#define ETH_CLKG_MACEN  0   // 2bits
#define ETH_MODE_IFMODE 0
#define ETH_MODE_RST0   8
#define ETH_MODE_RST1   9
#define ETH_MODE_PPSSWL 28

// GMAC Register 0 (MCR) - MCR (MAC Configuration Register) Address 0000h
#define GMAC_MCR_B_RE   2       // E (Receiver Enable)
#define GMAC_MCR_B_TE   3       // TE (Transmitter Enable)
#define GMAC_MCR_B_DC   4       // DC (Deferral Check)
#define GMAC_MCR_B_BL   5       // BL (Back-off Limit)
#define GMAC_MCR_B_ACS  7       // ACS (Automatic Pad/CRC Stripping)
#define GMAC_MCR_B_LUD  8       // LUD(Link Up/Down in RGMII)
#define GMAC_MCR_B_DR   9       // DR (Disable Retry)
#define GMAC_MCR_B_IPC  10      // IPC (Checksum Offload)
#define GMAC_MCR_B_DM   11      // DM (Duplex mode)
#define GMAC_MCR_B_LM   12      // LM (Loop-back Mode)
#define GMAC_MCR_B_DO   13      // DO (Disable Receive OWN)
#define GMAC_MCR_B_FES  14      // FES(Speed)
#define GMAC_MCR_B_PS   15      // PS (Port Select)
#define GMAC_MCR_B_DCRS 16      // DCRS(Disable Carrier Sense During Transaction)
#define GMAC_MCR_B_IFG  17      // IFG (Inter-Frame GAP)
#define GMAC_MCR_B_JE   20      // JE (Jumbo Frame Enable)
#define GMAC_MCR_B_BE   21      // BE (Frame Burst Enable)
#define GMAC_MCR_B_JD   22      // JD (Jabber Disable)
#define GMAC_MCR_B_WD   23      // WD (Watchdog Disable)
#define GMAC_MCR_B_TC   24      // TC (Transmit Configuration in RGMII)
#define GMAC_MCR_B_CST  25      // CST (CRC stripping for Type frames)
#define GMAC_MCR_RE     (1 << GMAC_MCR_B_RE)
#define GMAC_MCR_TE     (1 << GMAC_MCR_B_TE)
#define GMAC_MCR_DC     (1 << GMAC_MCR_B_DC)
#define GMAC_MCR_BL     (1 << GMAC_MCR_B_BL)
#define GMAC_MCR_ACS    (1 << GMAC_MCR_B_ACS)
#define GMAC_MCR_LUD    (1 << GMAC_MCR_B_LUD)
#define GMAC_MCR_DR     (1 << GMAC_MCR_B_DR)
#define GMAC_MCR_IPC    (1 << GMAC_MCR_B_IPS)
#define GMAC_MCR_DM     (1 << GMAC_MCR_B_DM)
#define GMAC_MCR_LM     (1 << GMAC_MCR_B_LM)
#define GMAC_MCR_DO     (1 << GMAC_MCR_B_DO)
#define GMAC_MCR_FES    (1 << GMAC_MCR_B_FES)
#define GMAC_MCR_PS     (1 << GMAC_MCR_B_PS)
#define GMAC_MCR_DCRS   (1 << GMAC_MCR_B_DCRS)
#define GMAC_MCR_IFG    (1 << GMAC_MCR_B_IFG)
#define GMAC_MCR_JE     (1 << GMAC_MCR_B_JE)
#define GMAC_MCR_BE     (1 << GMAC_MCR_B_BE)
#define GMAC_MCR_JD     (1 << GMAC_MCR_B_JD)
#define GMAC_MCR_WD     (1 << GMAC_MCR_B_WD)
#define GMAC_MCR_TC     (1 << GMAC_MCR_B_TC)
#define GMAC_MCR_CST    (1 << GMAC_MCR_B_CST)

// GMAC Register 1 (MFFR) - MFFR (MAC Frame Filter Register) Address 00004h
#define GMAC_MFFR_B_PR      0   // PR (Promiscuous Mode)
#define GMAC_MFFR_B_HUC     1   // HUC (Hash Unicast)
#define GMAC_MFFR_B_HMC     2   // HMC (Hash Multicast)
#define GMAC_MFFR_B_DAIF    3   // DAIF (DA Inverse Filtering)
#define GMAC_MFFR_B_PM      4   // PM (Pass All Multicast)
#define GMAC_MFFR_B_DB      5   // DB (Disable Broadcast Frames)
#define GMAC_MFFR_B_PCF     6   // PCF (Pass Control Frames)
#define GMAC_MFFR_B_SAIF    8   // SAIF (Source Address Inverse Filter)
#define GMAC_MFFR_B_SAF     9   // SAF (Source Address Filter)
#define GMAC_MFFR_B_HPF     10  // HPF (Hash or Perfect Filter)
#define GMAC_MFFR_B_RA      31  // RA (Receive All)
#define GMAC_MFFR_PR    (1 << GMAC_MFFR_B_PR)   // PR (Promiscuous Mode)
#define GMAC_MFFR_HUC   (1 << GMAC_MFFR_B_HUC)  // HUC (Hash Unicast)
#define GMAC_MFFR_HMC   (1 << GMAC_MFFR_B_HMC)  // HMC (Hash Multicast)
#define GMAC_MFFR_DAIF  (1 << GMAC_MFFR_B_DAIF) // DAIF (DA Inverse Filtering)
#define GMAC_MFFR_PM    (1 << GMAC_MFFR_B_PM)   // PM (Pass All Multicast)
#define GMAC_MFFR_DB    (1 << GMAC_MFFR_B_DB)   // DB (Disable Broadcast Frames)
#define GMAC_MFFR_PCF   (1 << GMAC_MFFR_B_PCF)  // PCF (Pass Control Frames)
#define GMAC_MFFR_SAIF  (1 << GMAC_MFFR_B_SAIF) // SAIF (Source Address Inverse Filter)
#define GMAC_MFFR_SAF   (1 << GMAC_MFFR_B_SAF)  // SAF (Source Address Filter)
#define GMAC_MFFR_HPF   (1 << GMAC_MFFR_B_HPF)  // HPF (Hash or Perfect Filter)
#define GMAC_MFFR_RA    (1 << GMAC_MFFR_B_RA)   // RA (Receive All)


// GMAC Register 2, 3 (MHTRH, MHTRL)
// MHTRH (MAC Hash Table Register (High)) Address 0008h
// MHTRL (MAC Hash Table Register (Low))

// GMAC Register 4 (GAR) - GAR (GMII/MII Address Register) Address 0010h
#define GMAC_GAR_B_GB   0
#define GMAC_GAR_B_GW   1
#define GMAC_GAR_B_CR   2
#define GMAC_GAR_B_GR   6
#define GMAC_GAR_B_PA   11
#define GMAC_GAR_GB (1 << GMAC_GAR_B_GB)
#define GMAC_GAR_GW (1 << GMAC_GAR_B_GW)
#define GMAC_GAR_CR (1 << GMAC_GAR_B_CR)
#define GMAC_GAR_GR (1 << GMAC_GAR_B_GR)
#define GMAC_GAR_PA (1 << GMAC_GAR_B_PA)

// GMAC Register 5 (GDR) - GDR (GMII/MII Data Register) Address 0014h

// GMAC Register 6 (FCR) - FCR (Flow Control Register) Address 0018h
#define GMAC_FCR_B_FCB  0   // FCB/BPA (Flow Control Busy/Backpressure Activate)
#define GMAC_FCR_B_TFE  1   // TFE (Transmit Flow Control Enable)
#define GMAC_FCR_B_RFE  2   // RFE (Receive Flow Control Enable)
#define GMAC_FCR_B_UP   3   // UP (Unicast Pause Frame detect)
#define GMAC_FCR_B_PLT  4   // PLT (Pause Low Threshold)
#define GMAC_FCR_B_DZPQ 7   // DZPQ (Disable Zero-Quanta Pause) - Reserved
#define GMAC_FCR_B_PT   16  // PT (Pause Time)

// GMAC Register 7 (VTR)

// GMAC Register 10 (RWFFR)

// GMAC Register 11 (PMTR)

// GMAC Register 12 (LPICSR)

// GMAC Register 13 (LPITCR) - LPITCR (LPI Timers Control Register) Address 0034h

// GMAC Register 14 (ISR) - ISR (Interrupt Status Register) Address 0038h

// GMAC Register 15 (IMR)

// GMAC Register 16 (MAR0H)

// GMAC Register 17 (MAR0L)

// GMAC Register 18, 20, 22, …, 542 (MAR1H, 2H, 3H, …,31H)

// GMAC Register 19, 21, 23, …, 543 (MAR1L, 2L, 3L, …,31L)

// GMAC Register 54 (RGSR) - RGSR (RGMII Status Register) Address 00D8h

// GMAC Register 448 (TSCR) - TSCR (Time Stamp Control Register) Address 0700h

// GMAC Register 449 (SSIR) - SSIR (Sub-Second Increment Register) Address 0704h

// GMAC Register 450 (STSR) - STSR (System Time - Seconds Register) Address 0708h

// GMAC Register 451 (STNR) - STNR (System Time - Nanoseconds Register) Address 070Ch

// GMAC Register 452 (STSUR) - STSUR (System Time - Seconds Update Register) Address 0710h

// GMAC Register 453 (STNUR) - STSNUR (System Time - Nanoseconds Update Register) Address 0714h

// GMAC Register 454 (TSAR)- TSAR (Time Stamp Addend Register) Address 0718h

// GMAC Register 455 (TTSR) - TTSR (Target Time Seconds Register) Address 071Ch

// GMAC Register 456 (TTNR) - TTNR (Target Time Nanoseconds Register) Address 0720h

// GMAC Register 457 (STHWSR) - STHWSR (System Time - Higher Word Seconds Register) Address 0724h

// GMAC Register 458 (TSR) - TSR (Time Stamp Status Register) Address 0728h

// GMAC Register 459 (PPSCR) - PPSCR (PPS Control Register) Address 072Ch

// GMAC Register 460 (ATNR) - ATNR (Auxiliary Time Stamp - Nanoseconds Register) Address 0730h

// GMAC Register 461 (ATSR) - ATSR (Auxiliary Time Stamp - Seconds Register) Address 0734h

// DMA Register 0 (BMR)

#define DMA_BMR_B_SWR   0
#define DMA_BMR_B_DA    1
#define DMA_BMR_B_DSL   2
#define DMA_BMR_B_ATDS  7
#define DMA_BMR_B_PBL   8
#define DMA_BMR_B_PR    14
#define DMA_BMR_B_FB    16
#define DMA_BMR_B_RPBL  17
#define DMA_BMR_B_USP   23
#define DMA_BMR_B_8xPBL 24
#define DMA_BMR_B_AAL   25
#define DMA_BMR_B_MB    26
#define DMA_BMR_B_TXPR  27
#define DMA_BMR_SWR (1 << DMA_BMR_B_SWR)
#define DMA_BMR_DA  (1 << DMA_BMR_B_DA)
#define DMA_BMR_FB  (1 << DMA_BMR_B_FB)
#define DMA_BMR_AAL (1 << DMA_BMR_B_AAL)
#define DMA_BMR_MB  (1 << DMA_BMR_B_MB)

// DMA Register 1 (TPDR)

// DMA Register 2 (RPDR)

// DMA Register 3 (RDLAR)

// DMA Register 4 (TDLAR)

// DMA Register 5 (SR)
#define DMA_SR_B_TI     0       // Transmit Interrupt
#define DMA_SR_B_TPS    1       // Transmit Process Stopped
#define DMA_SR_B_TU     2       // Transmit Buffer Unavailable
#define DMA_SR_B_TJT    3       // Transmit Jabber Timeout
#define DMA_SR_B_OVF    4       // Receive overflow
#define DMA_SR_B_UNF    5       // Transmit underflow
#define DMA_SR_B_RI     6       // Receive Interrupt
#define DMA_SR_B_RU     7       // Receive Buffer Unavailable
#define DMA_SR_B_RPS    8       // Receive Process Stopped
#define DMA_SR_B_RWT    9       // RWT Receive Watchdog Timeout
#define DMA_SR_B_ETI    10      // Early Transmit Interrupt
#define DMA_SR_B_FBI    13      // Fatal Bus Error Interrupt
#define DMA_SR_B_ERI    14      // Early Receive Interrupt
#define DMA_SR_B_AIS    15      // Abnormal Interrupt Summary
#define DMA_SR_B_NIS    16      // Normal Interrupt Summary
#define DMA_SR_B_RS     17      // RS Receive Process State
#define DMA_SR_B_TS     20      // Transmit Process State
#define DMA_SR_B_EB     23      // Error Bits
#define DMA_SR_B_GLI    26      // GMAC Line interface Interrupt (Reserved)
#define DMA_SR_B_GMI    27      // GMAC MMC Interrupt
#define DMA_SR_B_GPI    28      // GMAC PMT Interrupt
#define DMA_SR_B_TTI    29      // Time-Stamp Trigger Interrupt
#define DMA_SR_B_GLPII  30      // GMAC LPI Interrupt
#define DMA_SR_TPS      (1 << DMA_SR_B_PS)
#define DMA_SR_TI       (1 << DMA_SR_B_TI)
#define DMA_SR_TU       (1 << DMA_SR_B_TU)
#define DMA_SR_TJT      (1 << DMA_SR_B_TJT)
#define DMA_SR_OVF      (1 << DMA_SR_B_OVF)
#define DMA_SR_UNF      (1 << DMA_SR_B_UNF)
#define DMA_SR_RI       (1 << DMA_SR_B_RI)
#define DMA_SR_RU       (1 << DMA_SR_B_RU)
#define DMA_SR_RPS      (1 << DMA_SR_B_RPS)
#define DMA_SR_RWT      (1 << DMA_SR_B_RWT)
#define DMA_SR_ETI      (1 << DMA_SR_B_ETI)
#define DMA_SR_FBI      (1 << DMA_SR_B_FBI)
#define DMA_SR_ERI      (1 << DMA_SR_B_ERI)
#define DMA_SR_AIS      (1 << DMA_SR_B_AIS)
#define DMA_SR_NIS      (1 << DMA_SR_B_NIS)
#define DMA_SR_RS       (1 << DMA_SR_B_RS)
#define DMA_SR_TS       (1 << DMA_SR_B_TS)
#define DMA_SR_EB       (1 << DMA_SR_B_EB)
#define DMA_SR_GLI      (1 << DMA_SR_B_GLI)
#define DMA_SR_GMI      (1 << DMA_SR_B_GMI)
#define DMA_SR_GPI      (1 << DMA_SR_B_CPI)
#define DMA_SR_TTI      (1 << DMA_SR_B_TTI)
#define DMA_SR_GLPII    (1 << DMA_SR_B_GLPII)

// DMA Register 6 (OMR) Operation Mode Register
#define DMA_OMR_B_SR    1       // Start/Stop Recieve
#define DMA_OMR_B_OSF   2       // Operate on Second Frame
#define DMA_OMR_B_RTC   4       // Receive Threshold Control
#define DMA_OMR_B_FUF   6       // Foward Undersized Good Frames
#define DMA_OMR_B_FEF   7       // Forward Error Frames
#define DMA_OMR_B_ST    13      // Start/Stop Transmission Command
#define DMA_OMR_B_TTC   14      // Transmit Threshold Control
#define DMA_OMR_B_FTF   20      // Flush Tramsmit FIFO
#define DMA_OMR_B_TSF   21      // Transmit Store Forward
#define DMA_OMR_B_DEF   24      // Disable Flushing of Received Frames
#define DMA_OMR_B_RSF   25      // Receive Store and Forward
#define DMA_OMR_B_DT    26      // Disable Dropping of TCP/IP Chechsum Error Frames
#define DMA_OMR_SR      (1 << DMA_OMR_B_SR)     // Start/Stop Recieve
#define DMA_OMR_OSF     (1 << DMA_OMR_B_OSF)    // Operate on Second Frame
#define DMA_OMR_RTC     (1 << DMA_OMR_B_RTC)    // Receive Threshold Control
#define DMA_OMR_FUF     (1 << DMA_OMR_B_FUF)    // Foward Undersized Good Frames
#define DMA_OMR_FEF     (1 << DMA_OMR_B_FEF)    // Forward Error Frames
#define DMA_OMR_ST      (1 << DMA_OMR_B_ST)     // Start/Stop Transmission Command
#define DMA_OMR_TTC     (1 << DMA_OMR_B_TTC)    // Transmit Threshold Control
#define DMA_OMR_FTF     (1 << DMA_OMR_B_FTF)    // Flush Tramsmit FIFO
#define DMA_OMR_TSF     (1 << DMA_OMR_B_TSF)    // Transmit Store Forward
#define DMA_OMR_DEF     (1 << DMA_OMR_B_DEF)    // Disable Flushing of Received Frames
#define DMA_OMR_RSF     (1 << DMA_OMR_B_RSF)    // Receive Store and Forward
#define DMA_OMR_DT      (1 << DMA_OMR_B_DT)     // Disable Dropping of TCP/IP Chechsum Error Frames


// DMA Register 7 (IER) Interrupt Enable Register
#define DMA_IER_B_TIE   0       // Transmit Process Stopped
#define DMA_IER_B_TSE   1       // Transmit Process Stopped
#define DMA_IER_B_TUE   2       // Transmit Buffer Unavailable
#define DMA_IER_B_TJE   3       // Transmit Jabber Timeout
#define DMA_IER_B_OVE   4       // Receive Overflow Enable
#define DMA_IER_B_UNE   5       // Transmit Underflow Enable
#define DMA_IER_B_RIE   6       // Receive Interrupt Enable
#define DMA_IER_B_RUE   7       // Receive Buffer Unavailable Enable
#define DMA_IER_B_RSE   8       // Receive Process Stopped Enable
#define DMA_IER_B_RWE   9       // Receive Watchdog Timeout Enable
#define DMA_IER_B_ETE   10      // Early Transmit Interrupt Enable
#define DMA_IER_B_FBE   13      // Fatal Bus Error Enable
#define DMA_IER_B_ERE   14      // Early Receive Interrupt Enable
#define DMA_IER_B_AIE   15      // Abnormal Interrupt Summary Enable
#define DMA_IER_B_NIE   16      // Normal Interrupt Summary Enable
#define DMA_IER_TIE     (1 << DMA_IER_B_TIE)
#define DMA_IER_TUE     (1 << DMA_IER_B_TUE)
#define DMA_IER_RIE     (1 << DMA_IER_B_RIE)
#define DMA_IER_RUE     (1 << DMA_IER_B_RUE)
#define DMA_IER_ERE     (1 << DMA_IER_B_ERE)
#define DMA_IER_NIE     (1 << DMA_IER_B_NIE)

// DMA Register 8 (MFBOCR)

// DMA Register 9 (RIWTR)

// DMA Register 11 (AHBSR)
#define DMA_AHBSR_B_AHBS    0
#define DMA_AHBSR_AHBS  (1 << DMA_AHBSR_B_AHBS)

// DMA Register 18 (CHTDR)

// DMA Register 19 (CHRDR)

// DMA Register 20 (CHTBAR)

// DMA Register 21 (CHRBAR)

// Transmit Enhanced Descriptor 0 (TDES0)
#define TDES0_B_DB      0       // DB (Deferred Bit)
#define TDES0_B_UF      1       // CC (Collision Count)
#define TDES0_B_ED      2       // ED (Excessive Deferral)
#define TDES0_B_CC      3       // ED (Excessive Deferral)
#define TDES0_B_VF      7       // VF (VLAN Frame)
#define TDES0_B_EC      8       // EC (Excessive Collision)
#define TDES0_B_LCO     9       // LCO (Late Collision)
#define TDES0_B_NC      10      // NC (No Carrier)
#define TDES0_B_LC      11      // LC (Loss of Carrier)
#define TDES0_B_IPE     12      // IPE (IP Payload Error)
#define TDES0_B_FF      13      // FF(Frame Flushed)
#define TDES0_B_JT      14      // JT (Jabber Timeout)
#define TDES0_B_ES      15      // ES (Error Summary)
#define TDES0_B_IHE     16      // IHE (IP Header Error)
#define TDES0_B_TTSS    17      // TTSS (Transmit Time Stamp Status)
#define TDES0_B_TCH     20      // TCH (Second Address Chained)
#define TDES0_B_TER     21      // TER (Transmit End of Ring)
#define TDES0_B_CIC     22      // CIC (Checksum lnsertion Control)
#define TDES0_B_TTSE    25      // TTSE (Transmit Time Stamp Enable)
#define TDES0_B_DP      26      // DP (Disable Pad)
#define TDES0_B_DC      27      // DC (Disable CRC)
#define TDES0_B_FS      28      // FS (First Segment)
#define TDES0_B_LS      29      // LS (Last Segment)
#define TDES0_B_IC      30      // IC (Interrupt on Completion)
#define TDES0_B_OWN     31      // OWN (OWN bit)

// Transmit Enhanced Descriptor 1 (TDES1)

// Transmit Enhanced Descriptor 2 (TDES2)

// Transmit Enhanced Descriptor 3 (TDES3)

// Transmit Enhanced Descriptor 6 (TDES6)

// Transmit Enhanced Descriptor 7 (TDES7)

// Receive Enhanced Descriptor 0 (RDES0)
#define RDES0_B_ESA     0       // ESA (Extended Status Available)
#define RDES0_B_CE      1       // CE (CRC Error)
#define RDES0_B_DBE     2       // DBE (Dribble Bit Error)
#define RDES0_B_RE      3       // RE (Receive Error)
#define RDES0_B_RWT     4       // RWT (Receive Watchdog Timeout)
#define RDES0_B_FT      5       // FT (Frame Type)
#define RDES0_B_LC      6       // LC (Late Collision)
#define RDES0_B_TS      7       // TS (Time Stamp)
#define RDES0_B_LS      8       // LS (Last Descriptor)
#define RDES0_B_FS      9       // FS (First Descriptor)
#define RDES0_B_VLAN    10      // VLAN (VLAN tag)
#define RDES0_B_OE      11      // OE (Overflow Error)
#define RDES0_B_LE      12      // LE (Length Error)
#define RDES0_B_SAF     13      // SAF(Source Address Filter Fail)
#define RDES0_B_DE      14      // DE (Descriptor Error)
#define RDES0_B_ES      15      // ES (Error Summary)
#define RDES0_B_FL      16      // FL[13:0] (Frame Length)
#define RDES0_B_AFM     30      // AFM(Destination Address Filter Fail)
#define RDES0_B_OWN     31      // OWN (OWN bit)

// Receive Enhanced Descriptor 1 (RDES1)

// Receive Enhanced Descriptor 2 (RDES2)

// Receive Enhanced Descriptor 3 (RDES3)

// Receive Enhanced Descriptor 4 (RDES4)

// Receive Enhanced Descriptor 6 (RDES6)

// Receive Enhanced Descriptor 7 (RDES7)

#if 0

typedef struct stc_emac_dma_tdes0_field
{
    uint8_t DB      : 1;
    uint8_t UF      : 1;
    uint8_t ED      : 1;
    uint8_t CC      : 4;
    uint8_t VF      : 1;
    uint8_t EC      : 1;
    uint8_t LCO     : 1;
    uint8_t NC      : 1;
    uint8_t LC      : 1;
    uint8_t IPE     : 1;
    uint8_t FF      : 1;
    uint8_t JT      : 1;
    uint8_t ES      : 1;
    uint8_t IHE     : 1;
    uint8_t TTSS    : 1;
    uint8_t Reserved1: 2;
    uint8_t TCH     : 1;
    uint8_t TER     : 1;
    uint8_t CIC     : 2;
    uint8_t Reserved2: 1;
    uint8_t TTSE    : 1;
    uint8_t DP      : 1;
    uint8_t DC      : 1;
    uint8_t FS      : 1;
    uint8_t LS      : 1;
    uint8_t IC      : 1;
    uint8_t OWN     : 1;
} stc_emac_dma_tdes0_field_t;

typedef struct stc_emac_dma_tdes1_field
{
#if 0
    uint16_t TBS1       : 13;
    uint16_t Reserved1  : 3;
    uint16_t TBS2       : 13;
    uint16_t Reserved2  : 3;
#else
    uint16_t TBS1       : 16;
    uint16_t TBS2       : 16;
#endif
} stc_emac_dma_tdes1_field_t;

typedef struct stc_emac_dma_tdes2_field
{
    uint32_t B1AP       : 32;
} stc_emac_dma_tdes2_field_t;

typedef struct stc_emac_dma_tdes3_field
{
    uint32_t B2AP       : 32;
} stc_emac_dma_tdes3_field_t;

typedef struct _EMAC_DMA_TXDESC {
    union {
        uint32_t TDES0;
        stc_emac_dma_tdes0_field_t TDES0_f;
      };
    union {
        uint32_t TDES1;
        stc_emac_dma_tdes1_field_t TDES1_f;
      };
    union {
        uint32_t TDES2;
        stc_emac_dma_tdes2_field_t TDES2_f;
      };
    union {
        uint32_t TDES3;
        stc_emac_dma_tdes3_field_t TDES3_f;
      };
} EMAC_DMA_TXDESC;

typedef struct stc_emac_dma_rdes0_field
{
    uint8_t ESA     : 1;
    uint8_t CE      : 1;
    uint8_t DE      : 1;
    uint8_t RE      : 1;
    uint8_t RWT     : 1;
    uint8_t FT      : 1;
    uint8_t LC      : 1;
    uint8_t TS      : 1;
    uint8_t LS      : 1;
    uint8_t FS      : 1;
    uint8_t VLAN    : 1;
    uint8_t OE      : 1;
    uint8_t LE      : 1;
    uint8_t SAF     : 1;
    uint8_t DBE     : 1;
    uint8_t ES      : 1;
    uint16_t FL     : 14;
    uint16_t AFM    : 1;
    uint16_t OWN    : 1;
} stc_emac_dma_rdes0_field_t;

typedef struct stc_emac_dma_rdes1_field
{
    uint16_t RBS1       : 13;
    uint16_t Reserved1  : 1;
    uint16_t RCH        : 1;
    uint16_t RER        : 1;
    uint16_t RBS2       : 13;
    uint16_t Reserved2  : 2;
    uint16_t DIC        : 1;
} stc_emac_dma_rdes1_field_t;

typedef struct stc_emac_dma_rdes2_field
{
    uint32_t B1AP       : 32;
} stc_emac_dma_rdes2_field_t;

typedef struct stc_emac_dma_rdes3_field
{
    uint32_t B2AP       : 32;
} stc_emac_dma_rdes3_field_t;

typedef struct _EMAC_DMA_RXDESC {
    union {
        uint32_t RDES0;
        stc_emac_dma_rdes0_field_t RDES0_f;
      };
    union {
        uint32_t RDES1;
        stc_emac_dma_rdes1_field_t RDES1_f;
      };
    union {
        uint32_t RDES2;
        stc_emac_dma_rdes2_field_t RDES2_f;
      };
    union {
        uint32_t RDES3;
        stc_emac_dma_rdes3_field_t RDES3_f;
      };
} EMAC_DMA_RXDESC;

#endif

#endif /* FM3_ETH_H_ */
