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
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_fm3.h>
#include <linux/spi/flash.h>
#include <mach/fm3.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/spi.h>

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
extern struct mmc_spi_platform_data fm3_mmc_spi_pdata;
#endif

/*
 * WKFM3
 *
 * SD-CARD
 *  SPI 6_1
 *  CS  P30
 *  CD  PA3
 *  WP  PA1
 */

/*
 * WXMP3PLCD_FL
 *
 * SD-CARD
 *  SPI 6_1
 *  CS  P30
 *  CD  PA3
 *  WP  PA1
 *
 * M25PXX
 *  SPI 6_1
 *  CS  P34 (CH3-43)
 *
 *  SPI 1_2
 *  CS  P3D (CH2-26)
 */

 static struct fm3_spi_regs *FM3_CSIO_REGS[] = {
     (struct fm3_spi_regs  *)FM3_MFS0_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS1_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS2_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS3_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS4_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS5_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS6_CSIO_BASE,
     (struct fm3_spi_regs  *)FM3_MFS7_CSIO_BASE
 };

 #define SPI_PLAT_RESOURCES(uid)    \
 static struct resource  fm3_spi_## uid ##_resources[] = {  \
     {                                                       \
         .start  = FM3_MFS## uid ##_CSIO_BASE,               \
         .end    = FM3_MFS## uid ##_CSIO_BASE + 1,           \
         .flags  = IORESOURCE_MEM,                           \
     },                                                      \
     {                                                       \
         .start  = MFS## uid ##RX_IRQn,                      \
         .flags  = IORESOURCE_IRQ,                           \
     }                                                       \
 }

 #define SPI_PLAT_DEVICE(uid)                                   \
 static struct platform_device   fm3_spi_## uid ##_device = {   \
     .name           = FM3_CSIO_DRV_NAME,                        \
     .id             = uid,                                      \
     .resource       = fm3_spi_## uid ##_resources,             \
     .num_resources  = 2,                                        \
 }

 #define spi_init_register(uid) do {                            \
     platform_device_register(&fm3_spi_## uid ##_device);       \
     platform_set_drvdata(&fm3_spi_## uid ##_device, &spi_fm3_## uid ##_data ); \
 } while (0)

#if defined(CONFIG_FM3_SPI0)
static struct spi_fm3_data spi_fm3_0_data = {
	.ch_mod = 0,
};
SPI_PLAT_RESOURCES(0);
SPI_PLAT_DEVICE(0);
#endif
#if defined(CONFIG_FM3_SPI1)
static struct spi_fm3_data spi_fm3_1_data = {
	.ch_mod = 2,
};
SPI_PLAT_RESOURCES(1);
SPI_PLAT_DEVICE(1);
#endif
#if defined(CONFIG_FM3_SPI2)
static struct spi_fm3_data spi_fm3_2_data = {
	.ch_mod = 0,
};
SPI_PLAT_RESOURCES(2);
SPI_PLAT_DEVICE(2);
#endif
#if defined(CONFIG_FM3_SPI3)
static struct spi_fm3_data spi_fm3_3_data = {
	.ch_mod = 0,
};
SPI_PLAT_RESOURCES(3);
SPI_PLAT_DEVICE(3);
#endif
#if defined(CONFIG_FM3_SPI4)
static struct spi_fm3_data spi_fm3_4_data = {
	.ch_mod = 0,
};
SPI_PLAT_RESOURCES(4);
SPI_PLAT_DEVICE(4);
#endif
#if defined(CONFIG_FM3_SPI5)
static struct spi_fm3_data spi_fm3_5_data = {
#if defined(CONFIG_MACH_FM3_LFCQ1)
	.ch_mod = 2,
#else
	.ch_mod = 2,
#endif
};
SPI_PLAT_RESOURCES(5);
SPI_PLAT_DEVICE(5);
#endif
#if defined(CONFIG_FM3_SPI6)
static struct spi_fm3_data spi_fm3_6_data = {
#if defined(CONFIG_MACH_FM3_LFCQ1) || \
	defined(CONFIG_MACH_FM3_WXMP3PLCD)
	.ch_mod = 1,
#else
	.ch_mod = 1,
#endif
};
SPI_PLAT_RESOURCES(6);
SPI_PLAT_DEVICE(6);
#endif
#if defined(CONFIG_FM3_SPI7)
static struct spi_fm3_data spi_fm3_7_data = {
	.ch_mod = 0,
};
SPI_PLAT_RESOURCES(7);
SPI_PLAT_DEVICE(7);
#endif

#define SPI_FLASH_SIZE      (1024*1024*4)
#define SPI_FLASH_OFFSET0   (1024*1024*0)
#define SPI_FLASH_OFFSET1   (1024*1024*1)

/* ========== LFCQ1 ========== */
#if defined(CONFIG_MACH_FM3_LFCQ1)

#define SPI_CH_WKFM3		6
#define SPI_CS_SDCARD_WKFM3	(0x30)
#define SPI_CS_DEV_WKFM3	(0x35)

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
static struct spi_fm3_slave spi_fm3_sdcard_lfcq1 = {
	.cs_gpio = SPI_CS_SDCARD_WKFM3,
	.timeout = 3,
};
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static struct spi_fm3_slave spi_fm3_dev_lfcq1 = {
	.cs_gpio = SPI_CS_DEV_WKFM3,
	.timeout = 3,
};
#endif

static struct spi_board_info spi_board_info_lfcq1[] = {
/* SDCARD */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 8000000,	/* max spi clock (SCK) speed in HZ */
		.bus_num = SPI_CH_WKFM3,
		.chip_select = 0,
		.platform_data = &fm3_mmc_spi_pdata,
		.controller_data = &spi_fm3_sdcard_lfcq1,
		.mode = SPI_MODE_3,
	},
#endif

/* External SPI device */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.bus_num = SPI_CH_WKFM3,
		.chip_select = 1,
		.controller_data = &spi_fm3_dev_lfcq1,
		.mode = SPI_MODE_3,
	},
#endif
};

#endif

/* ========== WXMP3PLCD ==========*/
#if defined(CONFIG_MACH_FM3_WXMP3PLCD)

#define SPI_CH_WXMP3PLCD		6
#define SPI_CS_SDCARD_WXMP3PLCD	(0x30)
#define SPI_CS_DEV_WXMP3PLCD	(0x35)
#define SPI_CS_FLASH_WXMP3PLCD	(0x34)

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition spi_fm3_flash_partitions_wxmp3plcd[] = {
    {
        .name = "spi_flash_part0",
        .size = SPI_FLASH_OFFSET1 - SPI_FLASH_OFFSET0,
        .offset = SPI_FLASH_OFFSET0,
    },
    {
        .name = "spi_flash_part1",
        .size = SPI_FLASH_SIZE - SPI_FLASH_OFFSET1,
        .offset = SPI_FLASH_OFFSET1,
    },
};

static struct flash_platform_data spi_fm3_flash_data_wxmp3plcd = {
    .name = "m25p80",
    .parts =  spi_fm3_flash_partitions_wxmp3plcd,
    .nr_parts = ARRAY_SIZE(spi_fm3_flash_partitions_wxmp3plcd),
    .type = "m25p32",
};
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
static struct spi_fm3_slave spi_fm3_sdcard_wxmp3plcd = {
	.cs_gpio = SPI_CS_SDCARD_WXMP3PLCD,
	.timeout = 3,
};
#endif

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static struct spi_fm3_slave spi_fm3_dev_wxmp3plcd = {
	.cs_gpio = SPI_CS_DEV_WXMP3PLCD,
	.timeout = 3,
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct spi_fm3_slave spi_fm3_flash_wxmp3plcd = {
	.cs_gpio = SPI_CS_FLASH_WXMP3PLCD,
	.timeout = 3,
};
#endif

/* SDCARD */
static struct spi_board_info spi_board_info_wxmp3plcd[] = {
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 8000000,	/* max spi clock (SCK) speed in HZ */
		.bus_num = SPI_CH_WXMP3PLCD,
		.chip_select = 0,
		.platform_data = &fm3_mmc_spi_pdata,
		.controller_data = &spi_fm3_sdcard_wxmp3plcd,
		.mode = SPI_MODE_3,
	},
#endif

/* External SPI device */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.bus_num = SPI_CH_WXMP3PLCD,
		.chip_select = 1,
		.controller_data = &spi_fm3_dev_wxmp3plcd,
		.mode = SPI_MODE_3,
	},
#endif

/* SPI Serial Flash */
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		.modalias = "m25p80",
		.platform_data = &spi_fm3_flash_data_wxmp3plcd,
		.max_speed_hz = 8000000,
		.bus_num = SPI_CH_WXMP3PLCD,
		.chip_select = 2,
		.controller_data = &spi_fm3_flash_wxmp3plcd,
		.mode = SPI_MODE_3,
	},
#endif
};

#endif /* ========== End of WXMP3PLCD ==========*/

/* ========== CQFM3DUINO ==========*/
#if defined(CONFIG_MACH_FM3_CQFM3DUINO)

#define SPI_CH_CQFM3DUINO			5

#define SPI_CS_SDCARD_CQFM3DUINO	(0x3b)
#define SPI_CS_DEV_CQFM3DUINO		(0x39)
#define SPI_CS_FLASH_CQFM3DUINO		(0x08)
#define SPI_CS_W5500_CQFM3DUINO		(0x3d)		/* Arduino Ethernet Sheild */

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition spi_fm3_flash_partitions_cqfm3duino[] = {
    {
        .name = "spi_flash_part0",
        .size = SPI_FLASH_OFFSET1 - SPI_FLASH_OFFSET0,
        .offset = SPI_FLASH_OFFSET0,
    },
    {
        .name = "spi_flash_part1",
        .size = SPI_FLASH_SIZE - SPI_FLASH_OFFSET1,
        .offset = SPI_FLASH_OFFSET1,
    },
};

static struct flash_platform_data spi_fm3_flash_data_cqfm3duino = {
    .name = "m25p80",
    .parts =  spi_fm3_flash_partitions_cqfm3duino,
    .nr_parts = ARRAY_SIZE(spi_fm3_flash_partitions_cqfm3duino),
    .type = "m25p32",
};
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
static struct spi_fm3_slave spi_fm3_sdcard_cqfm3duino = {
	.cs_gpio = SPI_CS_SDCARD_CQFM3DUINO,
	.timeout = 3,
};
#endif

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static struct spi_fm3_slave spi_fm3_dev_cqfm3duino = {
	.cs_gpio = SPI_CS_DEV_CQFM3DUINO,
	.timeout = 3,
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct spi_fm3_slave spi_fm3_flash_cqfm3duino = {
	.cs_gpio = SPI_CS_FLASH_CQFM3DUINO,
	.timeout = 3,
};
#endif

#if defined(CONFIG_W5500)
static struct spi_fm3_slave spi_fm3_w5500_cqfm3duino = {
	.cs_gpio = SPI_CS_W5500_CQFM3DUINO,
	.timeout = 3,
};
#endif

static struct spi_board_info spi_board_info_cqfm3duino[] = {
/* Ethernet Shield */
#if defined(CONFIG_W5500)
	{
		.modalias = "w5500_eth",
		.max_speed_hz = 16000000,
		.bus_num = SPI_CH_CQFM3DUINO,
		.chip_select = 3,
		.controller_data = &spi_fm3_w5500_cqfm3duino,
		.mode = SPI_MODE_3,
	},
#endif

/* SDCARD */
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias = "mmc_spi",
		.max_speed_hz = 8000000,	/* max spi clock (SCK) speed in HZ */
		.bus_num = SPI_CH_CQFM3DUINO,
		.chip_select = 0,
		.platform_data = &fm3_mmc_spi_pdata,
		.controller_data = &spi_fm3_sdcard_cqfm3duino,
		.mode = SPI_MODE_3,
	},
#endif

/* External SPI device */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.bus_num = SPI_CH_CQFM3DUINO,
		.chip_select = 1,
		.controller_data = &spi_fm3_dev_cqfm3duino,
		.mode = SPI_MODE_3,
	},
#endif

/* SPI Serial Flash */
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		.modalias = "m25p80",
		.platform_data = &spi_fm3_flash_data_cqfm3duino,
		.max_speed_hz = 8000000,
		.bus_num = SPI_CH_CQFM3DUINO,
		.chip_select = 2,
		.controller_data = &spi_fm3_flash_cqfm3duino,
		.mode = SPI_MODE_3,
	},
#endif

};

#endif /* ========== End of CQFM3DUINO ==========*/

void __init fm3_spi_init(void)
{
#if defined(CONFIG_FM3_SPI0)
	spi_init_register(0);
	spi_fm3_0_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI1)
	spi_init_register(1);
	spi_fm3_1_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI2)
	spi_init_register(2);
	spi_fm3_2_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI3)
	spi_init_register(3);
	spi_fm3_3_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI4)
	spi_init_register(4);
	spi_fm3_4_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI5)
	spi_init_register(5);
	spi_fm3_5_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI6)
	spi_init_register(6);
	spi_fm3_6_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif
#if defined(CONFIG_FM3_SPI7)
	spi_init_register(7);
	spi_fm3_7_data.ref_clk = fm3_clock_get(CLOCK_PCLK1);
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE) || \
	defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE) || \
	defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE) || \
	defined(CONFIG_W5500)

#if defined(CONFIG_MACH_FM3_LFCQ1)
	spi_register_board_info((struct spi_board_info const *)&spi_board_info_lfcq1,
			ARRAY_SIZE(spi_board_info_lfcq1));
#endif
#if defined(CONFIG_MACH_FM3_WXMP3PLCD)
	spi_register_board_info((struct spi_board_info const *)&spi_board_info_wxmp3plcd,
			ARRAY_SIZE(spi_board_info_wxmp3plcd));
#endif
#if defined(CONFIG_MACH_FM3_CQFM3DUINO)
	spi_register_board_info((struct spi_board_info const *)&spi_board_info_cqfm3duino,
			ARRAY_SIZE(spi_board_info_cqfm3duino));
#endif

#endif
}
