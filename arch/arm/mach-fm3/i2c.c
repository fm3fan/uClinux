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
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#if defined(CONFIG_EEPROM_AT24)
#include <linux/i2c/at24.h>
#endif
#include <mach/fm3.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/i2c.h>

#define CSIO_PLAT_RESOURCES(uid) \
	static struct resource  fm3_i2c_## uid ##_resources[] = {	\
	{														\
		.start	= FM3_MFS## uid ##_CSIO_BASE,				\
		.end	= FM3_MFS## uid ##_CSIO_BASE + 1,			\
		.flags	= IORESOURCE_MEM,							\
	},														\
	{														\
		.start  = MFS## uid ##RX_IRQn,						\
		.flags  = IORESOURCE_IRQ,							\
	}														\
}

#define CSIO_PLAT_DEVICE(uid) \
static struct platform_device   fm3_i2c_## uid ##_device = {   \
	.name           = FM3_I2C_DRV_NAME,                        \
	.id             = uid,                                     \
	.resource       = fm3_i2c_## uid ##_resources,             \
	.num_resources  = 2,                                       \
}

#define I2C_FM3_DATA(uid)									\
static struct i2c_fm3_data i2c_fm3_## uid ##_data = {		\
	.i2c_clk		= 100000,								\
}

 #define i2c_init_register(uid) do {                            \
     platform_device_register(&fm3_i2c_## uid ##_device);       \
     platform_set_drvdata(&fm3_i2c_## uid ##_device, (void *)(&i2c_fm3_## uid ##_data)); \
 } while (0)

#if defined(CONFIG_FM3_I2C0)
CSIO_PLAT_RESOURCES(0);
CSIO_PLAT_DEVICE(0);
I2C_FM3_DATA(0);
#endif
#if defined(CONFIG_FM3_I2C1)
CSIO_PLAT_RESOURCES(1);
CSIO_PLAT_DEVICE(1);
I2C_FM3_DATA(1);
#endif
#if defined(CONFIG_FM3_I2C2)
CSIO_PLAT_RESOURCES(2);
CSIO_PLAT_DEVICE(2);
I2C_FM3_DATA(2);
#endif
#if defined(CONFIG_FM3_I2C3)
CSIO_PLAT_RESOURCES(3);
CSIO_PLAT_DEVICE(3);
I2C_FM3_DATA(3);
#endif
#if defined(CONFIG_FM3_I2C4)
CSIO_PLAT_RESOURCES(4);
CSIO_PLAT_DEVICE(4);
I2C_FM3_DATA(4);
#endif
#if defined(CONFIG_FM3_I2C5)
CSIO_PLAT_RESOURCES(5);
CSIO_PLAT_DEVICE(5);
I2C_FM3_DATA(5);
#endif
#if defined(CONFIG_FM3_I2C6)
CSIO_PLAT_RESOURCES(6);
CSIO_PLAT_DEVICE(6);
I2C_FM3_DATA(6);
#endif
#if defined(CONFIG_FM3_I2C7)
CSIO_PLAT_RESOURCES(7);
CSIO_PLAT_DEVICE(7);
I2C_FM3_DATA(7);
#endif
#if defined(CONFIG_EEPROM_AT24)
static struct at24_platform_data at24c1024 = {
	.byte_len	= 1024 / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};
#endif

#define i2c_board_info_cq_frk_fm3_BUS 5

#if defined(CONFIG_EEPROM_AT24)
static struct i2c_board_info const i2c_board_info_at24 = {
	I2C_BOARD_INFO("24c1024", 0x50),
	.platform_data = &at24c1024,
};
#endif
#if defined(CONFIG_EEPROM_93CX6)
static struct i2c_board_info const i2c_board_info_93c56 = {
{
	I2C_BOARD_INFO("93c56", 0x50)
};
#endif

void __init fm3_i2c_init(void)
{
	int ret;
	int	id = fm3_platform_get();

#if defined(CONFIG_FM3_I2C0)
	i2c_fm3_0_data.ch_mod = 0;
	i2c_fm3_0_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(0);
#endif
#if defined(CONFIG_FM3_I2C1)
#if defined(CONFIG_MACH_FM3_CQFM3DUINO) || \
	defined(CONFIG_MACH_FM3_LFCQ1) || \
	defined(CONFIG_MACH_FM3_WXMP3)
	i2c_fm3_1_data.ch_mod = 0;
#else
	i2c_fm3_1_data.ch_mod = 0;
#endif
	i2c_fm3_1_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(1);
#endif
#if defined(CONFIG_FM3_I2C2)
	i2c_fm3_2_data.ch_mod = 0;
	i2c_fm3_2_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(2);
#endif
#if defined(CONFIG_FM3_I2C3)
	i2c_fm3_3_data.ch_mod = 0;
	i2c_fm3_3_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(3);
#endif
#if defined(CONFIG_FM3_I2C4)
	i2c_fm3_4_data.ch_mod = 0;
	i2c_fm3_4_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(4);
#endif
#if defined(CONFIG_FM3_I2C5)
#if defined(CONFIG_MACH_FM3_LFCQ1) || \
	defined(CONFIG_MACH_FM3_WXMP3)
	i2c_fm3_5_data.ch_mod = 2;
#else
	i2c_fm3_5_data.ch_mod = 2;
#endif
	i2c_fm3_5_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(5);
#endif
#if defined(CONFIG_FM3_I2C6)
	i2c_fm3_6_data.ch_mod = 0;
	i2c_fm3_6_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(6);
#endif
#if defined(CONFIG_FM3_I2C7)
#if defined(CONFIG_MACH_FM3_CQFM3DUINO)
	i2c_fm3_7_data.ch_mod = 2;
#else
	i2c_fm3_7_data.ch_mod = 2;
#endif
	i2c_fm3_7_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
	i2c_init_register(7);
#endif

	switch (id) {
	case PLATFORM_FM3_LFCQ1:
	case PLATFORM_FM3_WXMP3PLCD:
	default:
#if defined(CONFIG_EEPROM_AT24)
		ret = i2c_register_board_info(i2c_board_info_cq_frk_fm3_BUS, &i2c_board_info_at24, 1);
		if (ret < 0)
			pr_err("%s: i2c_register_board_info failed.(%d)\n", __func__, ret);
		else
			pr_debug("%s: i2c_register_board_info registered.\n", __func__);
#endif
#if defined(CONFIG_EEPROM_93CX6)
		ret = i2c_register_board_info(i2c_board_info_cq_frk_fm3_BUS, &i2c_board_info_93cx6, 1);
		if (ret < 0)
			pr_err("%s: i2c_register_board_info failed.(%d)\n", __func__, ret);
		else
			pr_debug("%s: i2c_register_board_info registered.\n", __func__);
#endif
    	break;
	}
}
