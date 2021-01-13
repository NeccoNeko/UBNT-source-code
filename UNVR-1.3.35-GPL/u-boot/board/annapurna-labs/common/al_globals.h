/**
 * board/annapurna-labs/common/al_globals.h
 *
 * Thie file contains global data specific to Annapurna Labs based projects
 *
 * Copyright (C) 2012 Annapurna Labs Ltd.
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
#ifndef __AL_GLOBALS_H__
#define __AL_GLOBALS_H__

#include <common.h>
#include <pci.h>

#include <al_hal_iomap.h>

#include "al_hal_bootstrap.h"
#include "al_hal_muio_mux.h"
#include "al_serdes.h"
#include "al_hal_eth.h"
#include "al_hal_pcie.h"
#include "al_hal_thermal_sensor.h"

#if (AL_DEV_ID >= AL_DEV_ID_ALPINE_V3)
#include "al_hal_serdes_avg.h"
#endif

/* PCIe Configuration */
struct al_pcie_cfg {
	/* Card Connected */
	int present;

	/* Max speed */
	enum al_pcie_link_speed max_speed;

	/* Number of lanes */
	int num_lanes;

	/* Is endpoint */
	int ep;
};

/* Internal PCIe Configuration */
struct al_pcie_int_cfg_entry {
	int		is_enabled;
	int		is_vf_enabled;
	pci_dev_t	devno;
	int		(*pd)(struct pci_controller *hose, pci_dev_t dev);
	int		(*pre_flr)(struct pci_controller *hose, pci_dev_t dev);
	int		(*post_flr)(struct pci_controller *hose, pci_dev_t dev);
};

struct al_pcie_int_cfg {
	struct al_pcie_int_cfg_entry	*dev_arr;
	unsigned int			dev_arr_size;
};

/* Globals Structure - Can be used only after relocation */
struct al_globals {
	/* Chip device and device revision IDs */
	unsigned int			dev_id;
	unsigned int			rev_id;

	struct al_bootstrap		bootstraps;
	struct al_muio_mux_obj		muio_mux;
	struct al_serdes_grp_obj	serdes[AL_SRDS_NUM_GROUPS];

	struct al_pcie_cfg		*pcie_cfg;
	al_bool				pcie_any_link_up;
	al_bool				pcie_rc_manual_init;
	unsigned int			pcie_rc_link_pre_delay;
	unsigned int			pcie_rc_link_up_timeout;
	unsigned int			pcie_rc_link_post_delay;
	al_bool				pcie_rc_link_retry;
	struct al_pcie_int_cfg		pcie_int_cfg;

	struct al_eth_board_params	eth_board_params[AL_ETH_NUM];

	unsigned int			toc_offset;

	unsigned int			env_offset;
	unsigned int			env_offset_valid;

	unsigned int			env_redund_offset;
	unsigned int			env_redund_offset_valid;

	struct al_thermal_sensor_handle thermal_sensor_handle;

#if (AL_DEV_ID == AL_DEV_ID_ALPINE_V3) && (AL_DEV_REV_ID == 0)
	struct al_sbus_obj		thermal_sensor_sbus_handle;
#endif

#if (AL_DEV_ID >= AL_DEV_ID_ALPINE_V3)
	struct al_serdes_complex_obj	srds_cmplx_tc_main;
	struct al_serdes_grp_obj	srds_cmplx_tc_main_lanes[
		AL_SRDS_LANES_PER_COMPLEX_TR_TC_MAIN];
#endif
};

extern struct al_globals al_globals;

#endif

