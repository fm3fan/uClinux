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

#ifndef _MACH_FM3_CLOCK_H_
#define _MACH_FM3_CLOCK_H_

/*
 * Clocks enumeration
 */
enum fm3_clock {
    CLOCK_SYSCLK,   /* SYSCLK clock frequency expressed in Hz     */
    CLOCK_HCLK,     /* HCLK clock frequency expressed in Hz       */
    CLOCK_PCLK1,    /* PCLK1 clock frequency expressed in Hz      */
    CLOCK_PCLK2,    /* PCLK2 clock frequency expressed in Hz      */
    CLOCK_SYSTICK,  /* Systimer clock frequency expressed in Hz   */
    CLOCK_END       /* for internal usage                 */
};

/*
 * Initialize the clock section of the FM3
 */
void __init fm3_clock_init(void);

/*
 * Return a clock value for the specified clock
 */
unsigned int fm3_clock_get(enum fm3_clock clk);

#endif	/*_MACH_FM3_CLOCK_H_ */
