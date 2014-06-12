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
#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>

#include <mach/fm3.h>
#include <mach/sdcard.h>
#include <mach/platform.h>

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#define MMC_SPI_CARD_DETECT_INT 0x00
static int fm3_mmc_spi_init(struct device *dev, irqreturn_t (*detect_int)(int, void *), void *data)
{
#ifdef MF3_MMC_SPI_INT
	return request_irq(MMC_SPI_CARD_DETECT_INT, detect_int,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"mmc-spi-detect", data);
#else
	return 0;
#endif
}

static void fm3_mmc_spi_exit(struct device *dev, void *data)
{
#ifdef MF3_MMC_SPI_INT
	free_irq(MMC_SPI_CARD_DETECT_INT, data);
#endif
}

struct mmc_spi_platform_data fm3_mmc_spi_pdata = {
	.init = fm3_mmc_spi_init,
	.exit = fm3_mmc_spi_exit,
	.detect_delay = 100, /* msecs */
};
#endif

void __init fm3_sdcard_init(void)
{
}
