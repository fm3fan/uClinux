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
#include <linux/fm3_eth.h>

#include <mach/eth.h>
#include <mach/platform.h>

#if defined(CONFIG_FM3_MAC)
#define FM3_MAC_IRQ	32		// MAC0:32, MAC1:33
static struct resource eth_resources[] = {
	{
		.start	= FM3_MAC_BASE,
		.end	= FM3_MAC_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= FM3_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct fm3_eth_data wkfm3_eth_data = {
		.frame_max_size	= 1536,
		.tx_buf_num	= 24,
		.rx_buf_num	= 32,
		.mac_addr	= {0x00, 0x11, 0x22, 0x33, 0x44, 0x55},
		.phy_id		= ICS1894_ADDR,
};

static struct fm3_eth_data wxphy_eth_data = {
		.frame_max_size	= 1536,
		.tx_buf_num	= 24,
		.rx_buf_num	= 32,
		.mac_addr	= {0x00, 0x11, 0x22, 0x33, 0x44, 0x55},
		.phy_id		= LAN8700_ADDR,
};

static struct platform_device eth_device = {
	.name		= FM3_ETH_DRV_NAME,
	.resource	= eth_resources,
	.num_resources	= 2,
	.id		= 0,
};
#endif /* CONFIG_FM3_MAC */

void __init fm3_eth_init(void)
{
#if defined(CONFIG_FM3_MAC)
	int	platform;

	platform = fm3_platform_get();
	switch (platform) {
	case PLATFORM_FM3_WXMP3PLCD:
		eth_device.dev.platform_data = &wxphy_eth_data;
		break;
	default:
        eth_device.dev.platform_data = &wkfm3_eth_data;
		break;
	}

    FM3_GPIO->PFR6 &= ~0x0004;
    FM3_GPIO->DDR6 &= ~0x0004;
    FM3_GPIO->PFRC = PFRC_ETH;      // GPIO configuration for CH0 and CH1
    FM3_GPIO->PFRD = PFRD_ETH;      // GPIO configuration for CH0 and CH1
    FM3_GPIO->EPFR14 = (3 << B_E_SPLC) |
                    // (1 << B_E_PSE) |
            (1 << B_E_CKE) |
            (1 << B_E_MD1B) |
            (1 << B_E_MD0B) |
            (1 << B_E_MC1B) |
            (1 << B_E_MC0E) |
            (1 << B_E_TE1E) |
            (1 << B_E_TE0E) |
            (1 << B_E_TD1E) |
            (1 << B_E_TD0E);

	platform_device_register(&eth_device);
#endif
}
