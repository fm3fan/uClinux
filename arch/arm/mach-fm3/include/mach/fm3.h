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

#ifndef FM3_H_
#define FM3_H_

#define MB9BF618T

#ifndef __ASSEMBLY__

#include <asm/types.h>
#include "fm3_eth.h"

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

void LED_init(void);
void LED_on(void);
void LED_off(void);
void LED_toggle(void);

#ifdef MB9BF506N
#include "system_mb9bf50x.h"

#define SYSTEM_CYCLE_CLOCK_HZ   80000000
#define SysFreHCLK      (SYSTEM_CYCLE_CLOCK_HZ)
#define SysFrePCLK1     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFrePCLK2     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFreSYSTICK   (SYSTEM_CYCLE_CLOCK_HZ)
#endif

#ifdef MB9BF618T
#define __CM3_REV               0x0200
//#include "system_mb9bf61x.h"
#define PSW_TMR_val	0	// dummy
#include "mb9bf618t.h"

#define SYSTEM_CYCLE_CLOCK_HZ   144000000
#define SysFreHCLK      (SYSTEM_CYCLE_CLOCK_HZ)
#define SysFrePCLK1     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFrePCLK2     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFreSYSTICK   (SYSTEM_CYCLE_CLOCK_HZ)
#endif

// MFS defines
#define SMR_SOE			0x01U
#define SMR_BDS			0x04U
#define SMR_SBL			0x08U
#define SMR_WUCR		0x10U
#define SMR_MD_UART		0x00U
#define SMR_MD_UART_MP	0x20U
#define SMR_MD_SIO		0x40U
#define SMR_MD_LIN		0x60U
#define SMR_MD_I2C		0x80U

#define SCR_TXE			0x01U
#define SCR_RXE			0x02U
#define SCR_TBIE		0x04U
#define SCR_TIE			0x08U
#define SCR_RIE			0x10U
#define SCR_UPGL		0x80U

#define SSR_TBI			0x01U
#define SSR_TDRE		0x02U
#define SSR_RDRF		0x04U
#define SSR_ORE			0x08U
#define SSR_FRE			0x10U
#define SSR_PE			0x20U
#define SSR_REC			0x80U

#define ESCR_P			0x08U
#define ESCR_PEN		0x10U
#define ESCR_INV		0x20U
#define ESCR_ESBL		0x40U
#define ESCR_FLWEN		0x80U
#define ESCR_DATABITS_8	0x00U
#define ESCR_DATABITS_5	0x01U
#define ESCR_DATABITS_6	0x02U
#define ESCR_DATABITS_7	0x03U
#define ESCR_DATABITS_9	0x04U

#define BGR_EXT			0x8000U

#define FCR1_FSEL		0x01U
#define FCR1_FTIE		0x02U
#define FCR1_FDRQ		0x04U
#define FCR1_FRIIE		0x08U
#define FCR1_FLSTE		0x10U

#define FCR0_FE1		0x01U
#define FCR0_FE2		0x02U
#define FCR0_FCL1		0x04U
#define FCR0_FCL2		0x08U
#define FCR0_FSET		0x10U
#define FCR0_FLD		0x20U
#define FCR0_FLST		0x40U

// Dual Timer
#define DtimDev_FREERUN_MODE    0U
#define DtimDev_CYCLE_MODE      1U
#define DtimDev_ONESHOT_MODE    2U
#define DtimDev_INT_ENABLE      1U
#define DtimDev_TIMER_PRE_1     0U
#define DtimDev_TIMER_PRE_16    1U
#define DtimDev_TIMER_PRE_256   2U
#define DtimDev_TIMER_SIZE_16   0U
#define DtimDev_TIMER_SIZE_32   1U
#define DtimDev_LOAD            0U
#define DtimDev_BG_LOAD         1U
#define DtimDev_IRQSTATUS_UNDERFLOW 0x01U

#endif /* __ASSEMBLY__ */

#endif /* FM3_H_ */
