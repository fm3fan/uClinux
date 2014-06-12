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

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

/*
 * PLATFORM_FM3_LFCQ1: CQ_FRK_FM3 board + LFCQ1 board
 *   CQ_FRK_FM3: http://www.kumikomi.net/interface/contents/fm3.php (in Japanese)
 *   LFCQ1: http://www.l-and-f.co.jp/seihin/LF/LFCQ1%282%29.htm (in Japanese)
 * PLATFORM_FM3_WXMP3PLCD: CQ_FRK_FM3 board + WXMP3PLCD FM3 board
 *   WXMP3PLCD: http://homepage3.nifty.com/fpga/wx/sram-phy/index.htm (in Japanese)
 * PLARFORM_FM3_CQFM3DUINO
 *   TBD
 * PLATFORM_FM3_KS_MB9BF506
 *   TBD
 */
#define PLATFORM_FM3_LFCQ1	0
#define	PLATFORM_FM3_WXMP3PLCD	1
#define	PLATFORM_FM3_CQFM3DUINO	2
#define PLATFORM_FM3_KS_MB9BF506 3

int fm3_device_get(void);
int fm3_platform_get(void);
void fm3_mfs_set_mode(uint32_t mode);

#endif
/* __ASM_ARCH_PLATFORM_H */
