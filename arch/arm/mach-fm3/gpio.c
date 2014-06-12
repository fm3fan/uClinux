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

#include <linux/init.h>
#include <asm/mach-types.h>
#include <asm/hardware/nvic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/clock.h>
#include <mach/fm3.h>
#include <mach/gpio.h>

//#define FM3_GPIO_DEBUG_FUNC
//#define FM3_GPIO_DEBUG_KERNEL_LOG

#ifdef FM3_GPIO_DEBUG_FUNC
#define FUNC_ENTER()	printk("%s enter\n", __func__)
#define FUNC_EXIT()		printk("%s exit\n", __func__)
#else
#define FUNC_ENTER()
#define FUNC_EXIT()
#endif

void fm3_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int v)
{
	int port = GPIO_PORT(gpio);
	int mask = GPIO_MASK(gpio);

	FUNC_ENTER();
    GPIO_PFR(port) &= ~mask;		// GPIO mode
    GPIO_DDR(port) |= mask;			// output
    if (v)
        GPIO_PDOR(port) |= mask;
    else
        GPIO_PDOR(port) &= ~mask;
    FUNC_EXIT();
}

static int fm3_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	int port = GPIO_PORT(gpio);
	int mask = GPIO_MASK(gpio);

	FUNC_ENTER();
	GPIO_PFR(port) &= ~mask;		// GPIO mode
	GPIO_DDR(port) &= ~mask;		// input
	FUNC_EXIT();
	return (GPIO_PDIR(port) & mask) ? 1 : 0;
}

static int fm3_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	int port = GPIO_PORT(gpio);
	int mask = GPIO_MASK(gpio);

	FUNC_ENTER();
	GPIO_PFR(port) &= ~mask;		// GPIO mode
	GPIO_DDR(port) &= ~mask;		// input
	FUNC_EXIT();
	return 0;
}

static int fm3_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int level)
{
	int port = GPIO_PORT(gpio);
	int mask = GPIO_MASK(gpio);

	FUNC_ENTER();
	GPIO_PFR(port) &= ~mask;		// GPIO mode
    GPIO_DDR(port) |= mask;			// output
    FUNC_EXIT();
    return 0;
}

static struct gpio_chip fm3_gpio_chip = {
	.label			= "fm3",
	.direction_input	= fm3_gpio_direction_input,
	.get			= fm3_gpio_get_value,
	.direction_output	= fm3_gpio_direction_output,
	.set			= fm3_gpio_set_value,
	.base			= FM3_GPIO_OFF,
	.ngpio			= FM3_GPIO_LEN,
	.can_sleep		= 1,
};

void __init fm3_gpio_init(void)
{
	int ret;
	ret = gpiochip_add(&fm3_gpio_chip);
	if (ret < 0)
		pr_err("%s: gpiochip_add failed.(%d)\n", __func__, ret);
#if defined(FM3_GPIO_DEBUG_KERNEL_LOG)
	else
		printk(KERN_DEBUG "fm3_gpio_init OK\n");
#endif
}
