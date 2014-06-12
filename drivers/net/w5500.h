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

#ifndef W5500_H_
#define W5500_H_

#define SR_NUM		8
#define SR0			0
#define	SR1			1
#define	SR2			2
#define	SR3			3
#define	SR4			4
#define	SR5			5
#define	SR6			6
#define	SR7			7

#define MR          0x0000
#define GAR0        0x0001
#define GAR1        0x0002
#define GAR2        0x0003
#define GAR3        0x0004
#define SUBR0       0x0005
#define SUBR1       0x0006
#define SUBR2       0x0007
#define SUBR3       0x0008
#define SHAR0       0x0009
#define SHAR1       0x000A
#define SHAR2       0x000B
#define SHAR3       0x000C
#define SHAR4       0x000D
#define SHAR5       0x000E
#define SIPR0       0x000F
#define SIPR1       0x0010
#define SIPR2       0x0011
#define SIPR3       0x0012
#define INTLEVEL0   0x0013
#define INTLEVEL1   0x0014
#define IR          0x0015
#define IMR         0x0016
#define SIR         0x0017
#define SIMR        0x0018
#define RTR0        0x0019
#define RTR1        0x001A
#define RCR         0x001B
#define PTIMER      0x001C
#define PMAGIC      0x001D
#define PHAR0       0x001E
#define PHAR1       0x001F
#define PHAR2       0x0020
#define PHAR3       0x0021
#define PHAR4       0x0022
#define PHAR5       0x0023
#define PSID0       0x0024
#define PSID1       0x0025
#define PMRU0       0x0026
#define PMEU1       0x0027
#define UIPR0       0x0028
#define UIPR1       0x0029
#define UIPR2       0x002A
#define UIPR3       0x002B
#define UPORT0      0x002C
#define UPORT1      0x002D
#define PHYCFGR     0x002E
#define VERSIONR    0x0039

#define S0_BASE     0x0000
#define S1_BASE     0x0000
#define S2_BASE     0x0000
#define S3_BASE     0x0000

#define Sn_MR       0x0000  /* [R/W] モードレジスタ */
#define Sn_CR       0x0001  /* [R/W] コマンドレジスタ (Open,Close,Connect,Listen,Send,Recv)コマンドを設定する */
#define Sn_IR       0x0002  /* [R]   割り込みレジスタ */
#define Sn_SR       0x0003  /* [R]   ステータスレジスタ */
#define Sn_PORT0    0x0004  /* [R/W] ポート */
#define Sn_PORT1    0x0005
#define Sn_DHAR0    0x0006  /* [R/W] 通信相手のMACアドレス */
#define Sn_DHAR1    0x0007
#define Sn_DHAR2    0x0008
#define Sn_DHAR3    0x0009
#define Sn_DHAR4    0x000A
#define Sn_DHAR5    0x000B
#define Sn_DIPR0    0x000C  /* [R/W] 通信相手のＩＰアドレス */
#define Sn_DIPR1    0x000D
#define Sn_DIPR2    0x000E
#define Sn_DIPR3    0x000F
#define Sn_DPORT0   0x0010  /* [R/W] 通信相手のポート */
#define Sn_DPORT1   0x0011
#define Sn_MSSR0    0x0012  /* [R/W] Maximum Segment Size(MSS) */
#define Sn_MSSR1    0x0013
#define Sn_PROTO    0x0014  /* [R/W] IP RAWモードのプロトコル番号 */
#define Sn_TOS      0x0015  /* [R/W] IPヘッダのTOS値(Type Of servise) */
#define Sn_TTL      0x0016  /* [R/W] IPヘッダのTTL値(Time to live) */
#define Sn_RXBUF_SIZE   0x001E
#define Sn_TXBUF_SIZE   0x001F
#define Sn_TX_FSR0  0x0020  /* [R]   送信メモリの空きサイズ */
#define Sn_TX_FSR1  0x0021
#define Sn_TX_RD0   0x0022  /* [R]   送信データ読み込み位置 */
#define Sn_TX_RD1   0x0023
#define Sn_TX_WR0   0x0024  /* [R/W] 送信データ書き込み位置 */
#define Sn_TX_WR1   0x0025
#define Sn_RX_RSR0  0x0026  /* [R]   受信メモリ中の受信データサイズ */
#define Sn_RX_RSR1  0x0027
#define Sn_RX_RD0   0x0028  /* [R/W] 受信データ読み込み位置 */
#define Sn_RX_RD1   0x0029
#define Sn_RX_WR0   0x002A
#define Sn_RX_WR1   0x002B
#define Sn_IMR      0x002C
#define Sn_FRAG0    0x002D
#define Sn_FRAG1    0x002E
#define Sn_KPALVTR  0x002F

#define MR_RST              (1<<7)  /* リセット'1'の時リセットします。このbitはリセット後に自動でクリアされます。 */
#define MR_PB               (1<<4)  /* pingブロックモード (0:禁止 1:許可) このbitが'1'に設定される場合はping echoは行わない */
#define MR_PPPOE            (1<<3)  /* PPPoE許可          (0:禁止 1:許可) */
#define MR_AI               (1<<1)  /* オートインクリメント(indirect mode) */
#define MR_IND              (1<<0)  /* インダイレクトモード(0:禁止 1:許可) */

#define IR_CONFLICT         (1<<7)  /* IPアドレス衝突 */
#define IR_UNREACH          (1<<6)  /* UDP送信における不到達 */
#define IR_PPPoE            (1<<5)  /* PPPoEクローズ */
#define IR_SOCK3            (1<<3)  /* ソケット割り込み */
#define IR_SOCK2            (1<<2)
#define IR_SOCK1            (1<<1)
#define IR_SOCK0            (1<<0)

#define Sn_MR_CLOSE         0x00    /* ソケットクローズ */
#define Sn_MR_TCP           0x01    /* TCPを使用 */
#define Sn_MR_UDP           0x02    /* UDPを使用 */
#define Sn_MR_IPRAW         0x03    /* ネットワーク層レベルの通信 */
#define Sn_MR_MACRAW        0x04    /* データリンク層レベル */
#define Sn_MR_PPPoE         0x05    /* PPPoE */

#define Sn_MR_ND            (1<<5)  /* No Delayed Ack(0:禁止 1:許可) ※TCPの場合に有効 データパケットを受け取った場合すぐにACKパケット送信、性能向上のため'1' */
#define Sn_MR_MC            (1<<5)  /* using IGMP version (0:Ver.2 / 1:Ver.1) ※マルチキャスト許可 ＆ UDPの場合に有効    */
#define Sn_MR_MULTI         (1<<7)  /* マルチキャスト(0:禁止 1:許可) ※UDPの場合に有効 */

#define Sn_CR_OPEN          0x01    /* ソケットのオープン／初期化 */
#define Sn_CR_LISTEN        0x02    /* クライアントからの接続待ち(TCP サーバモード) */
#define Sn_CR_CONNECT       0x04    /* サーバへの接続(TCP クライアントモード) */
#define Sn_CR_DISCON        0x08    /* サーバからの切断(TCP) */
#define Sn_CR_CLOSE         0x10    /* ソケットのクローズ */
#define Sn_CR_SEND          0x20    /* データ送信(送信ポインタ更新) */
#define Sn_CR_SEND_MAC      0x21    /* SENDと同じ、ARPで解決せずにMACアドレス付きでデータ送信(UDP,IPRAW) */
#define Sn_CR_SEND_KEEP     0x22    /* キープアライブメッセージ送信(TCP) */
#define Sn_CR_RECV          0x40    /* データ受信(受信ポインタ更新) */

#define Sn_IR_SEND_OK       (1<<4)  /* 送信完了 */
#define Sn_IR_TIMEOUT       (1<<3)  /* タイムアウト発生 */
#define Sn_IR_RECV          (1<<2)  /* データ受信 */
#define Sn_IR_DISCON        (1<<1)  /* コネクションクローズ */
#define Sn_IR_CON           (1<<0)  /* コネクション確立 */

#define Sn_SR_CLOSED        0x00    /* クローズ */
#define Sn_SR_INIT          0x13    /* TCP 初期状態 */
#define Sn_SR_LISTEN        0x14    /* TCP LISTEN状態 */
#define Sn_SR_ESTABLISHED   0x17    /* TCP コネクション成立 */
#define Sn_SR_CLOSE_WAIT    0x1C    /* クローズ中(自動でSn_SR_CLOSED) */
#define Sn_SR_UDP           0x22    /* UDP使用 */
//#define Sn_SR_IPRAW         0x32    /* IPrawモード使用 */
#define Sn_SR_MACRAW        0x02    /* MACrawモード使用 */
//#define Sn_SR_PPPoE         0x5F    /* PPPoEモード使用 */

#define Sn_SR_SYNSENT       0x15    /* TCP SYN送信 */
#define Sn_SR_SYNRECV       0x16    /* TCP SYN受信 */
#define Sn_SR_FIN_WAIT      0x18    /* クローズ中(自動でSn_SR_CLOSED) */
#define Sn_SR_CLOSING       0x1A    /* クローズ中(自動でSn_SR_CLOSED) */
#define Sn_SR_TIME_WAIT     0x1B    /* クローズ中(自動でSn_SR_CLOSED) */
#define Sn_SR_LAST_ACK      0x1D    /* クローズ中(自動でSn_SR_CLOSED) */
//#define Sn_SR_ARP           0x11    /* ARP-requestを伝送する状態 */
//#define Sn_SR_ARP2          0x21
//#define Sn_SR_ARP3          0x31

#define Sn_TOS_DEFAULT              0x00    /* デフォルト    */
#define Sn_TOS_MIN_MONETARY_COST    0x01    /* コスト最小    */
#define Sn_TOS_MAX_RELIABILITY      0x02    /* 信頼性最大    */
#define Sn_TOS_MAX_THROUGHPUT       0x04    /* スループット最大 */
#define Sn_TOS_MIN_DELAY            0x08    /* 遅延最小         */
#define Sn_TOS_MAX_SECURITY         0x0f    /* 最大セキュリティ */

/* PHYCFGR register value */
#define PHYCFGR_RST                  ~(1<<7)  //< For PHY reset, must operate AND mask.
#define PHYCFGR_OPMD                 (1<<6)   // Configre PHY with OPMDC value
#define PHYCFGR_OPMDC_ALLA           (7<<3)
#define PHYCFGR_OPMDC_PDOWN          (6<<3)
#define PHYCFGR_OPMDC_NA             (5<<3)
#define PHYCFGR_OPMDC_100FA          (4<<3)
#define PHYCFGR_OPMDC_100F           (3<<3)
#define PHYCFGR_OPMDC_100H           (2<<3)
#define PHYCFGR_OPMDC_10F            (1<<3)
#define PHYCFGR_OPMDC_10H            (0<<3)
#define PHYCFGR_DPX_FULL             (1<<2)
#define PHYCFGR_DPX_HALF             (0<<2)
#define PHYCFGR_SPD_100              (1<<1)
#define PHYCFGR_SPD_10               (0<<1)
#define PHYCFGR_LNK_ON               (1<<0)
#define PHYCFGR_LNK_OFF              (0<<0)

#endif /* W5500_H_ */
