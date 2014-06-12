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

#ifndef _MACH_FM3_ETH_H_
#define _MACH_FM3_ETH_H_

#include <mach/fm3.h>

#define FM3_MAC_BASE	(0x40064000)

#define ICS1894_ADDR    0x10        // PHY device address for Wakamatsu FM3 LAN board
#define LAN8187_ADDR    0x06        // PHY device address for Wakamatsu ARM LAN board
#define DP83848_ADDR    0x01        // PHY device address for DP83848
#define LAN8700_ADDR    0x1F        // PHY device address for Will Electronics WX-PHY
#define ICS1894_ID      0x0015f450  // PHY Identifier of ICS1894
#define LAN8187_ID      0x0006C0C4  // PHY Identifier of LAN8187
#define DP83848_ID      0x20005C90  // PHY Identifier of DP83848
#define LAN8700_ID      0x0007C0C0  // PHY Identifier of LAN8700i

void __init fm3_eth_init(void);

#endif
/* _MACH_FM3_ETH_H_ */
