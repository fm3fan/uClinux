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
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/platform.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/eth.h>
#include <mach/spi.h>
#include <mach/i2c.h>
#include <mach/flash.h>
#include <mach/sdcard.h>
#include <mach/usb.h>
#if defined(CONFIG_GPIOLIB)
#include <mach/i2c-gpio.h>
#endif

static struct sys_timer fm3_timer = {
    .init = fm3_timer_init,
};

#if defined(CONFIG_MACH_FM3_LFCQ1)
static int fm3_platform = PLATFORM_FM3_LFCQ1;
#elif defined(CONFIG_MACH_FM3_WXMP3PLCD)
static int fm3_platform = PLATFORM_FM3_WXMP3PLCD;
#elif defined(CONFIG_MACH_FM3_CQFM3DUINO)
static int fm3_platform = PLATFORM_FM3_CQFM3DUINO;
#else
#endif

EXPORT_SYMBOL(fm3_platform_get);
int fm3_platform_get(void)
{
	return fm3_platform;
}

void LED_init(void)
{
#ifdef MB9BF618T
    // LED PF3
    FM3_GPIO->PFRF &= 0xFFF7;   // PF3 GPIO
    FM3_GPIO->DDRF  |= 0x0008;  // PF3 output
#endif
#ifdef MB9BF506N
    FM3_GPIO->PFR4 &= 0xFFFE;   // P40 GPIO
    FM3_GPIO->DDR4  |= 0x0001;  // P40 output
#endif
}

#define PIN_MCLKOUT	0x38

// FM3
static uint8_t mfs_sin[] = {
	0x21, 0x14, 0xb4,
	0x56, 0x11, 0xf0,
	0x72, 0x24, 0x17,
	0x75, 0x50, 0x48,
	0xd2, 0x1A, 0x05,
	0x60, 0x92, 0x36,
	0x53, 0x33, 0xf3,
	0x59, 0x4e, 0xb0
};

static uint8_t mfs_sot[] = {
    0x22, 0x15, 0xb5,
    0x57, 0x12, 0xf1,
    0x73, 0x25, 0x18,
    0x76, 0x51, 0x49,
    0xd1, 0x1b, 0x06,
    0x61, 0x93, 0x37,
    0x54, 0x32, 0xf4,
    0x5a, 0x4d, 0xb1
};

static uint8_t mfs_sck[] = {
    0x23, 0x16, 0xb6,
    0x58, 0x13, 0xf2,
    0x74, 0x26, 0x19,
    0x77, 0x52, 0x4a,
    0xd0, 0x1c, 0x07,
    0x62, 0x94, 0x38,
    0x55, 0x31, 0xf5,
    0x5b, 0x4c, 0xb2
};

static uint8_t mfs_epfr_shift [] = {
    4, 10, 16, 22, 36, 42, 48, 54
};

static uint32_t mfs_mode [] = {
    0x15, 0x2a, 0x3f
};

void __init fm3_mfs_set_mode(uint32_t mode)
{
	uint32_t port;
	uint32_t mask;
	uint32_t ch_num;
	uint32_t ch_mod;
	uint32_t m;

	if (mode > MFS_MODE_MAX)
		goto fm3_mfs_set_mode_exit;
	if (mfs_sot[mode] == 0xff)
		goto fm3_mfs_set_mode_exit;
	port = GPIO_PORT(mfs_sot[mode]);
	mask = GPIO_MASK(mfs_sin[mode]) + GPIO_MASK(mfs_sot[mode]) + GPIO_MASK(mfs_sck[mode]);
	GPIO_PFR(port) |= mask;
	ch_mod = mode % 3;
	ch_num = mode / 3;
#if defined(CONFIG_MACH_FM3_LFCQ1)
	if ((ch_num == 5) && (ch_mod == 2)) {
		GPIO_EPFR(10) &= 0xfffffffb;
		pr_debug("warning MCLKOUT being off\n");
	}
#endif
	if (ch_num < 4) {
		m = mfs_mode[ch_mod] << mfs_epfr_shift[ch_num];
		GPIO_EPFR(7) |= m;
#if defined(FM3_GPIO_DEBUG)
		dprintk("CH%d_%d: EPFR[7] |= 0x%08x PFR[%x] |= 0x%04x\n", ch_num, ch_mod, m, port, mask);
#endif
#if defined(FM3_GPIO_DEBUG_KERNEL_LOG)
		printk(KERN_DEBUG "CH%d_%d: EPFR[7] |= 0x%08x PFR[%x] |= 0x%04x\n", ch_num, ch_mod, m, port, mask);
#endif
	} else {
		m = mfs_mode[ch_mod] << (mfs_epfr_shift[ch_num] - 32);
		GPIO_EPFR(8) |= m;
#if defined(FM3_GPIO_DEBUG)
		dprintk("CH%d_%d: EPFR[8] |= 0x%08x PFR[%x] |= 0x%04x\n", ch_num, ch_mod, m, port, mask);
#endif
#if defined(FM3_GPIO_DEBUG_KERNEL_LOG)
		printk(KERN_DEBUG "CH%d_%d: EPFR[8] |= 0x%08x PFR[%x] |= 0x%04x\n", ch_num, ch_mod, m, port, mask);
#endif
	}
fm3_mfs_set_mode_exit:
	return;
}

void LED_off(void)
{
#ifdef MB9BF618T
    FM3_GPIO->PDORF |= 0x0008;
#endif
#ifdef MB9BF506N
    FM3_GPIO->PDOR4 |= 0x0001;
#endif
}

void LED_on(void)
{
#ifdef MB9BF618T
    FM3_GPIO->PDORF &= ~0x0008;
#endif
#ifdef MB9BF506N
    FM3_GPIO->PDOR4 &= ~0x0001;
#endif
}

void LED_toggle(void)
{
#ifdef MB9BF618T
    FM3_GPIO->PDORF ^= 0x0008;
#endif
#ifdef MB9BF506N
    FM3_GPIO->PDOR4 ^= 0x0001;
#endif
}

static void __init fm3_map_io(void)
{

}

static void __init fm3_init_irq(void)
{
	nvic_init();
}

/*
 * FM3 plaform initialization.
 */
static void __init fm3_init(void)
{
    FM3_GPIO->ADE = 0;
#if defined(CONFIG_GPIOLIB)
	/*
	 * Register the MCU GPIO chip
	 */
	fm3_gpio_init();
#endif

#if defined(CONFIG_SERIAL_FM3)
	/*
	 * Configure the USART devices
	 */
	fm3_uart_init();
#endif

#if defined(CONFIG_FM3_MAC)
	/*
	 * Configure the FM3 MAC
	 */
	fm3_eth_init();
#endif

#if defined(CONFIG_SPI_FM3)
	/*
	 * Configure the FM3 SPI devices
	 */
	fm3_spi_init();
#endif

#if defined(CONFIG_I2C_FM3)
	/*
	 * Configure the FM3 I2C devices
	 */
	fm3_i2c_init();
#endif

#if defined(CONFIG_I2C_GPIO)
	/*
	 * Configure the FM3 I2C_GPIO devices
	 */
	fm3_i2c_gpio_init();
#endif

#if defined(CONFIG_MTD_PHYSMAP)
	/*
	 * Configure external Flash
	 */
	fm3_flash_init();
#endif

#if defined(CONFIG_USB_FM3_HCD)
	/*
	 * Configure the USB devices
	 */
	fm3_usb_init();
#endif

#if defined(CONFIG_FM3_SPI6) && defined(CONFIG_MMC_SPI)
	/*
	 * Configure the FM3 SD devices
	 */
	fm3_sdcard_init();
#endif
}

MACHINE_START(FM3, "Fujitsu FM3")
    .phys_io        = FM3_UART_BASE,
    .io_pg_offst    = (IO_ADDRESS(FM3_UART_BASE) >> 18) & 0xfffc,
    .map_io         = fm3_map_io,
    .init_irq       = fm3_init_irq,
    .timer          = &fm3_timer,
    .init_machine   = fm3_init,
MACHINE_END
