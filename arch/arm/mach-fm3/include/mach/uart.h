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

#ifndef _MACH_FM3_UART_H_
#define _MACH_FM3_UART_H_

#include <mach/fm3.h>

#define FM3_UART_BASE       0x40038000
#define FM3_UART_TDR        0x8
#define FM3_UART_RDR        0x8
#define FM3_UART_ESCR       0x4
#define FM3_UART_SSR        0x5
#define FM3_UART_SSR_TDRE   0x02
#define FM3_UART_SSR_RDRF   0x04
#define FM3_UART_ESCR_FLWEN 0x80

#ifndef __ASSEMBLY__
#include <linux/init.h>

#define FM3_UART_DRV_NAME	"fm3_uart"

struct fm3_uart_regs {
    volatile  unsigned char SMR;
    volatile  unsigned char SCR;
    unsigned char RESERVED0[2];
    volatile  unsigned char ESCR;
    volatile  unsigned char SSR;
    unsigned char RESERVED1[2];
    union {
        volatile uint16_t RDR;
        volatile uint16_t TDR;
    };
    unsigned char RESERVED2[2];
    volatile uint16_t BGR;
};

struct fm3_uart_drvdata {
	unsigned int	ch_mod;
};

void __init fm3_uart_init(void);

#endif /* __ASSEMBLY__ */

#endif /*_MACH_FM3_UART_H_ */
