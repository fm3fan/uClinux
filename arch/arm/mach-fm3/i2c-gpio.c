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
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/at24.h>

#include <mach/i2c-gpio.h>
#include <mach/platform.h>
#include <mach/gpio.h>

#if defined(CONFIG_I2C_GPIO)
static struct i2c_gpio_platform_data i2c_gpio_data_lfcq1_0 = {
	.sda_pin	= 0x57,		/* P57: FRAM SDA */
	.scl_pin	= 0x58,		/* P58: FRAM SCL */
};

static struct platform_device fm3_i2c_gpio_device_lfcq1_0 = {
	.name	= "i2c-gpio",
	.id		= 0,
};

static struct i2c_gpio_platform_data i2c_gpio_data_lfcq1_1 = {
	.sda_pin	= 0x34,		/* P34: FRAM SDA */
	.scl_pin	= 0x35,		/* P35: FRAM SCL */
};

static struct platform_device fm3_i2c_gpio_device_lfcq1_1 = {
	.name	= "i2c-gpio",
	.id		= 1,
};

#if defined(CONFIG_EEPROM_AT24)
static struct at24_platform_data at24c1024 = {
	.byte_len	= 1024 / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};
#endif

static struct i2c_board_info i2c_board_info_ks_mb9bf568[] = {
#if defined(CONFIG_EEPROM_AT24)
	{
		I2C_BOARD_INFO("24c1024", 0x50),
		.platform_data = &at24c1024,
	},
#endif
#if defined(CONFIG_EEPROM_93CX6)
	{
		I2C_BOARD_INFO("93c56", 0x50)
	},
#endif
};

void __init fm3_i2c_gpio_init(void)
{
	int id;
	int ret;
	id = fm3_platform_get();
	switch (id) {
	default:
		fm3_i2c_gpio_device_lfcq1_0.dev.platform_data = &i2c_gpio_data_lfcq1_0;
		ret = platform_device_register(&fm3_i2c_gpio_device_lfcq1_0);
		if (ret < 0)
			pr_err("%s: fm3_i2c_gpio_device_lfcq1_0 not registered(%d)\n", __func__, ret);
		fm3_i2c_gpio_device_lfcq1_1.dev.platform_data = &i2c_gpio_data_lfcq1_1;
		ret = platform_device_register(&fm3_i2c_gpio_device_lfcq1_1);
		if (ret < 0)
			pr_err("%s: fm3_i2c_gpio_device_lfcq1_1 not registered(%d)\n", __func__, ret);
#if 0
#if defined(CONFIG_EEPROM_93CX6) || defined(CONFIG_EEPROM_AT24)
		ret = i2c_register_board_info(5, &i2c_board_info_ks_mb9bf568, ARRAY_SIZE(i2c_board_info_ks_mb9bf568));
    	if (ret < 0)
    		pr_err("%s: i2c_register_board_info failed.(%d)\n", __func__, ret);
    	else
    		printk("%s: i2c_register_board_info registered.\n", __func__);
#endif
#endif
		break;
	}
}
#endif
