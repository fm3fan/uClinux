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

#ifndef _MACH_FM3_GPIO_H_
#define _MACH_FM3_GPIO_H_

#define FM3_GPIO_PORTS		16
#define FM3_GPIO_PORT_PINS	16
#define FM3_GPIO_OFF		0
#define FM3_GPIO_LEN		(FM3_GPIO_PORT_PINS * FM3_GPIO_PORTS)
#define FM3_GPIO_PORTPIN2NUM(port, pin)	(FM3_GPIO_OFF + ((port) * FM3_GPIO_PORT_PINS + (pin)))
#define ARCH_NR_GPIOS		(FM3_GPIO_LEN - FM3_GPIO_OFF)

#if defined(CONFIG_GPIOLIB)

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_to_irq		__gpio_to_irq
#define gpio_cansleep	__gpio_cansleep

#include <asm-generic/gpio.h>

#endif /* CONFIG_GPIOLIB */

#define GPIO_PORT(pin)  (pin >> 4)
#define GPIO_MASK(pin)  (1 << (pin & 0xF))

#define GPIO_PFR(port)  *((volatile uint32_t*)(FM3_GPIO_BASE + 0x000UL + ((port) << 2)))
#define GPIO_PCR(port)  *((volatile uint32_t*)(FM3_GPIO_BASE + 0x100UL + ((port) << 2)))
#define GPIO_DDR(port)  *((volatile uint32_t*)(FM3_GPIO_BASE + 0x200UL + ((port) << 2)))
#define GPIO_PDIR(port) *((volatile uint32_t*)(FM3_GPIO_BASE + 0x300UL + ((port) << 2)))
#define GPIO_PDOR(port) *((volatile uint32_t*)(FM3_GPIO_BASE + 0x400UL + ((port) << 2)))
#define GPIO_EPFR(port) *((volatile uint32_t*)(FM3_GPIO_BASE + 0x600UL + ((port) << 2)))
#define MFS_MODE_MAX (8*3)

void __init fm3_gpio_init(void);

#endif /* _MACH_FM3_GPIO_H_ */
