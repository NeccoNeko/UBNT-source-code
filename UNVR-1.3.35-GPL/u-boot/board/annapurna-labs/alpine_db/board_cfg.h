/**
 * board/annapurna-labs/alpine_db/board_cfg.h
 *
 * Annapurna Labs Alpine development board static configuration
 *
 * Copyright (C) 2013 Annapurna Labs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <pll_init.h>
#include <gpio_board_init.h>
#include <al_hal_muio_mux.h>

/******************************************************************************
 * PLL
 ******************************************************************************/
#define PLL_SB_CHAN_IDX_ETH0_REF_CLK_OUT	9
#define PLL_SB_CHAN_IDX_ETH1_REF_CLK_OUT	10
#define PLL_SB_CHAN_IDX_SERDES_R2L_CLK		14
#define PLL_SB_CHAN_IDX_SERDES_L2R_CLK		15

/**
 * - Set Ethernet 0/1 RGMII reference clock outputs to 25 MHz
 * - Set Serdes R2L and L2R clocks to 100MHz (currently disabled because using
 *   PLL bypass and assuming 100MHz reference clock)
 */
static const struct pll_cfg_ent pll_sb_cfg[] = {
	{ PLL_SB_CHAN_IDX_ETH0_REF_CLK_OUT, 25000 },
	{ PLL_SB_CHAN_IDX_ETH0_REF_CLK_OUT, 25000 },
/*	{ PLL_SB_CHAN_IDX_SERDES_R2L_CLK, 100000 },*/	/* R2L */
/*	{ PLL_SB_CHAN_IDX_SERDES_L2R_CLK, 100000 },*/	/* L2R */
};

