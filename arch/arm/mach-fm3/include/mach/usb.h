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

#ifndef _MACH_FM3_USB_H_
#define _MACH_FM3_USB_H_

#include <mach/fm3.h>

#ifndef __ASSEMBLY__

#include <linux/init.h>

#define FM3_USB_DRV_NAME	"fm3_usb"

void __init fm3_usb_init(void);

#endif
/* __ASSEMBLY__ */

#endif
/*_MACH_FM3_USB_H_ */
