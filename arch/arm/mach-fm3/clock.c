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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <asm/clkdev.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/fm3.h>

static u32 clock_val[CLOCK_END];

// based on Fujitsu clock initialization sample
void __init fm3_clock_init(void)
{
    static uint32_t u32IoRegisterRead;  // Workaround variable for MISRA C rule conformance

    FM3_HWWDT->WDG_LCK = 0x1ACCE551;                 /* HW Watchdog Unlock */
    FM3_HWWDT->WDG_LCK = 0xE5331AAE;
    FM3_HWWDT->WDG_CTL = 0;                          /* HW Watchdog stop */

    FM3_CRG->BSC_PSR   = BSC_PSR_Val;                /* set System Clock presacaler */
    FM3_CRG->APBC0_PSR = APBC0_PSR_Val;              /* set APB0 presacaler */
    FM3_CRG->APBC1_PSR = APBC1_PSR_Val;              /* set APB1 presacaler */
    FM3_CRG->APBC2_PSR = APBC2_PSR_Val;              /* set APB2 presacaler */
    FM3_CRG->SWC_PSR   = SWC_PSR_Val | (1UL << 7);   /* set SW Watchdog presacaler */
    FM3_CRG->TTC_PSR   = TTC_PSR_Val;                /* set Trace Clock presacaler */

    FM3_CRG->CSW_TMR   = CSW_TMR_Val;                /* set oscillation stabilization wait time */

    if (SCM_CTL_Val & (1UL << 1)) {                  /* Main clock oscillator enabled ? */
        FM3_CRG->SCM_CTL |= (1UL << 1);              /* enable main oscillator */
        while (!(FM3_CRG->SCM_STR & (1UL << 1)));    /* wait for Main clock oscillation stable */
    }

    if (SCM_CTL_Val & (1UL << 3)) {                  /* Sub clock oscillator enabled ? */
        FM3_CRG->SCM_CTL |= (1UL << 3);              /* enable sub oscillator */
        while (!(FM3_CRG->SCM_STR & (1UL << 3)));    /* wait for Sub clock oscillation stable */
    }

    FM3_CRG->PSW_TMR   = PSW_TMR_Val;                /* set PLL stabilization wait time */
    FM3_CRG->PLL_CTL1  = PLL_CTL1_Val;               /* set PLLM and PLLK */
    FM3_CRG->PLL_CTL2  = PLL_CTL2_Val;               /* set PLLN          */

    if (SCM_CTL_Val & (1UL << 4)) {                  /* PLL enabled ? */
        FM3_CRG->SCM_CTL  |= (1UL << 4);             /* enable PLL */
        while (!(FM3_CRG->SCM_STR & (1UL << 4)));    /* wait for PLL stable */
    }

    FM3_CRG->SCM_CTL  |= (SCM_CTL_Val & 0xE0);       /* Set Master Clock switch */

    // Workaround for preventing MISRA C:1998 Rule 46 (MISRA C:2004 Rule 12.2)
    // violations:
    //   "Unordered reads and writes to or from same location" and
    //   "Unordered accesses to a volatile location"
    do {
        u32IoRegisterRead = (FM3_CRG->SCM_CTL & 0xE0);
    } while ((FM3_CRG->SCM_STR & 0xE0) != u32IoRegisterRead);

    /* CR Trimming Data  */
    if( 0x000003FF != (FM3_FLASH_IF->CRTRMM & 0x000003FF) )
    {
      /* UnLock (MCR_FTRM) */
      FM3_CRTRIM->MCR_RLR = 0x1ACCE554;
      /* Set MCR_FTRM */
      FM3_CRTRIM->MCR_FTRM = FM3_FLASH_IF->CRTRMM;
      /* Lock (MCR_FTRM) */
      FM3_CRTRIM->MCR_RLR = 0x00000000;
    }

    clock_val[CLOCK_SYSCLK] = SYSTEM_CYCLE_CLOCK_HZ;
    clock_val[CLOCK_HCLK] = SysFreHCLK;
    clock_val[CLOCK_PCLK1] = SysFrePCLK1;
    clock_val[CLOCK_PCLK2] = SysFrePCLK2;
    clock_val[CLOCK_SYSTICK] = SysFreSYSTICK;
}

unsigned int fm3_clock_get(enum fm3_clock clk)
{
	return clock_val[clk];
}
EXPORT_SYMBOL(fm3_clock_get);
