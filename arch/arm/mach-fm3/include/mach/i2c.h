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

#ifndef _MACH_FM3_I2C_H_
#define _MACH_FM3_I2C_H_

#define FM3_I2C_DRV_NAME	"fm3_i2c"

struct i2c_fm3_data {
	unsigned int	ch_mod;
	unsigned int	ref_clk;	/* Reference clock */
	unsigned int	i2c_clk;	/* Bus clock */
};

extern void __init fm3_i2c_init(void);

#endif	/*_MACH_FM3_I2C_H_ */
