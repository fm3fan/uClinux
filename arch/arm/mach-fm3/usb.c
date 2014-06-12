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

#include <linux/platform_device.h>
#include <mach/fm3.h>
#include <mach/usb.h>

// Note: support one channel as USB HOST (default is CH1)
// if you use USB CH0, you need to change as FM3_USB0_BASE and USB0F_USB0H_IRQn
#define USB_PLAT_RESOURCES(uid)    \
static struct resource  fm3_usb_## uid ##_resources[] = {  \
    {                                                       \
        .start  = FM3_USB1_BASE,                    \
        .end    = FM3_USB1_BASE + 1,                \
        .flags  = IORESOURCE_MEM,                           \
    },                                                      \
    {                                                       \
        .start  = USB1F_USB1H_IRQn,         \
        .flags  = IORESOURCE_IRQ,                           \
    }                                                       \
}

#define USB_PLAT_DEVICE(uid)                                   \
static struct platform_device   fm3_usb_## uid ##_device = {   \
    .name           = FM3_USB_DRV_NAME,                        \
    .id             = uid,                                     \
    .resource       = fm3_usb_## uid ##_resources,             \
    .num_resources  = 2,                                       \
}

#define usb_init_register(uid) do {                            \
    platform_device_register(&fm3_usb_## uid ##_device);       \
} while (0)

USB_PLAT_RESOURCES(0);
USB_PLAT_DEVICE(0);

void __init fm3_usb_init(void)
{
    usb_init_register(0);
}
