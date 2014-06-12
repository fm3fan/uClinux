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
#include <mach/fm3.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/uart.h>

static struct fm3_uart_regs *FM3_UART_REGS[] = {
	(struct fm3_uart_regs  *)FM3_MFS0_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS1_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS2_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS3_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS4_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS5_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS6_UART_BASE,
	(struct fm3_uart_regs  *)FM3_MFS7_UART_BASE
};

#define UART_PLAT_RESOURCES(uid)	\
static struct resource  fm3_uart_## uid ##_resources[] = {	\
	{														\
		.start	= FM3_MFS## uid ##_UART_BASE,				\
		.end	= FM3_MFS## uid ##_UART_BASE + 1,			\
		.flags	= IORESOURCE_MEM,							\
	},														\
	{														\
		.start	= MFS## uid ##RX_IRQn,						\
		.flags	= IORESOURCE_IRQ,							\
	}														\
}

#define UART_PLAT_DEVICE(uid)	\
static struct platform_device   fm3_uart_## uid ##_device = {	\
	.name			= FM3_UART_DRV_NAME,						\
	.id				= uid,										\
	.resource		= fm3_uart_## uid ##_resources,				\
	.num_resources	= 2,										\
}

#define FM3_UART_DRVDATA(uid)									\
static struct fm3_uart_drvdata fm3_uart_## uid ##_drvdata = {	\
	.ch_mod		= 0,											\
}

#define uart_init_register(uid) do {                            \
    platform_device_register(&fm3_uart_## uid ##_device);       \
    platform_set_drvdata(&fm3_uart_## uid ##_device, (void *)(&fm3_uart_## uid ##_drvdata)); \
} while (0)

#define BAUDRATE    115200
// Note: The default serial channel is CH0_0.
// If you use another channel, you need to manually change the followings.
#define FM3_UART_DEF_CH_NUM	0
#define	FM3_UART_DEF_CH_MOD	0

#if defined(CONFIG_FM3_USART0)
UART_PLAT_RESOURCES(0);
UART_PLAT_DEVICE(0);
FM3_UART_DRVDATA(0);
#endif
#if defined(CONFIG_FM3_USART1)
UART_PLAT_RESOURCES(1);
UART_PLAT_DEVICE(1);
FM3_UART_DRVDATA(1);
#endif
#if defined(CONFIG_FM3_USART2)
UART_PLAT_RESOURCES(2);
UART_PLAT_DEVICE(2);
FM3_UART_DRVDATA(2);
#endif
#if defined(CONFIG_FM3_USART3)
UART_PLAT_RESOURCES(3);
UART_PLAT_DEVICE(3);
FM3_UART_DRVDATA(3);
#endif
#if defined(CONFIG_FM3_USART4)
UART_PLAT_RESOURCES(4);
UART_PLAT_DEVICE(4);
FM3_UART_DRVDATA(3);
#endif
#if defined(CONFIG_FM3_USART5)
UART_PLAT_RESOURCES(5);
UART_PLAT_DEVICE(5);
FM3_UART_DRVDATA(5);
#endif
#if defined(CONFIG_FM3_USART6)
UART_PLAT_RESOURCES(6);
UART_PLAT_DEVICE(6);
FM3_UART_DRVDATA(6);
#endif
#if defined(CONFIG_FM3_USART7)
UART_PLAT_RESOURCES(7);
UART_PLAT_DEVICE(7);
FM3_UART_DRVDATA(7);
#endif

void __init fm3_uart_init(void)
{
	struct fm3_uart_regs *uart_regs = FM3_UART_REGS[FM3_UART_DEF_CH_NUM];
	fm3_mfs_set_mode(FM3_UART_DEF_CH_NUM*3 + FM3_UART_DEF_CH_MOD);
	uart_regs->SMR = (SMR_MD_UART | SMR_SOE);
	uart_regs->SCR = 0x00;
	uart_regs->BGR = (uint16_t)((SysFrePCLK2 / (unsigned int)BAUDRATE) - 1);
	uart_regs->SCR &= ~SCR_UPGL;
	uart_regs->SSR = 0x00;
	uart_regs->ESCR = 0;
	uart_regs->SCR  |= (SCR_TXE);

#if defined(CONFIG_FM3_USART0)
	uart_init_register(0);
	fm3_uart_0_drvdata.ch_mod = 0;	// need to set channel mode x (ch0_x)
#endif
#if defined(CONFIG_FM3_USART1)
	uart_init_register(1);
	fm3_uart_1_drvdata.ch_mod = 0;	// need to set channel mode x (ch1_x)
#endif
#if defined(CONFIG_FM3_USART2)
	uart_init_register(2);
	fm3_uart_2_drvdata.ch_mod = 0;	// need to set channel mode x (ch2_x)
#endif
#if defined(CONFIG_FM3_USART3)
	uart_init_register(3);
	fm3_uart_3_drvdata.ch_mod = 0;	// need to set channel mode x (ch3_x)
#endif
#if defined(CONFIG_FM3_USART4)
	uart_init_register(4);
	fm3_uart_4_drvdata.ch_mod = 0;	// need to set channel mode x (ch4_x)
#endif
#if defined(CONFIG_FM3_USART5)
	uart_init_register(5);
	fm3_uart_5_drvdata.ch_mod = 0;	// need to set channel mode x (ch5_x)
#endif
#if defined(CONFIG_FM3_USART6)
	uart_init_register(6);
	fm3_uart_6_drvdata.ch_mod = 0;	// need to set channel mode x (ch6_x)
#endif
#if defined(CONFIG_FM3_USART7)
	uart_init_register(7);
	fm3_uart_7_drvdata.ch_mod = 0;	// need to set channel mode x (ch7_x)
#endif
}
