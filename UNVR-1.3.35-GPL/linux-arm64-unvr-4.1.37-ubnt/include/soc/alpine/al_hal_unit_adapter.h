/*******************************************************************************
Copyright (C) 2015 Annapurna Labs Ltd.

This file may be licensed under the terms of the Annapurna Labs Commercial
License Agreement.

Alternatively, this file can be distributed under the terms of the GNU General
Public License V2 as published by the Free Software Foundation and can be
found at http://www.gnu.org/licenses/gpl-2.0.html

Alternatively, redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:

*     Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

*     Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

/**
 * @defgroup group_unit_adapter	Unit Adapter
 *  @{
 *
 * Common operation example:
 * @code
 * int main()
 * {
 *	int err;
 *	struct al_unit_adapter sata_adapter;
 *
 *	err = al_unit_adapter_handle_init(&sata_adapter, AL_UNIT_ADAPTER_TYPE_SATA,
 *		(void __iomem *)0xfc8f8000, NULL, NULL, NULL, NULL);
 *	if (err)
 *		return err;
 *
 *	al_unit_adapter_mem_shutdown_ctrl(&sata__adapter, AL_TRUE);
 *	al_unit_adapter_clk_gating_ctrl(&sata__adapter, AL_TRUE);
 *	al_unit_adapter_pd_ctrl(&sata__adapter, AL_TRUE);
 *
 *	return 0;
 * }
 * @endcode
 *
 * @file   al_hal_unit_adapter.h
 * @brief Header file for the unit adapter HAL
 * The adapter is responsible for exposing an internal bus-master unit inside
 * the I/O Fabric as an Integrated PCIe Endpoint.
 * Most configuration and registers of this adapter is used for trouble shooting
 * and handling bus-level errors. It is not used for mainstream operation.
 * Throughout this file, AXI refers to the internal name of the bus used inside
 * the I/O Fabric.
 *
 */

#ifndef __AL_HAL_UNIT_ADAPTER_H__
#define __AL_HAL_UNIT_ADAPTER_H__

#include "al_hal_common.h"

#define AL_UNIT_ADAPTER_SATA_DEV_ID	0x0031
#define AL_UNIT_ADAPTER_SATA_REV_ID_1	1
#define AL_UNIT_ADAPTER_SATA_REV_ID_2	2

/* Enum definitions */

/** Unit Adapter Type */
enum al_unit_adapter_type {
	AL_UNIT_ADAPTER_TYPE_ETH,	/* Ethernet unit adapter */
	AL_UNIT_ADAPTER_TYPE_SSM,	/* Security, storage, and memory unit adapter */
	AL_UNIT_ADAPTER_TYPE_SATA,	/* SATA unit adapter */
};

/* Struct definitions */
struct al_unit_adapter {
	enum al_unit_adapter_type type;
	void __iomem *regs;
	int (* pcie_read_config_u32)(void *handle, int where, uint32_t *val);
	int (* pcie_write_config_u32)(void *handle, int where, uint32_t val);
	int (* pcie_flr)(void *handle);
	void *handle;
};

struct al_unit_adapter_int_fields {
	/*
	 * AXI master write error interrupt enabled
	 */
	al_bool wr_err;
	/*
	 * AXI master read error interrupt enabled
	 */
	al_bool rd_err;
	/*
	 * Application parity error interrupt enabled
	 */
	al_bool app_parity_error;
};

/**
 * Unit adapter AXI master error attributes
 * This structure groups the set of errors that a bus transaction can see.
 * Errors for a read transaction and Errors for write transactions share many
 * common attributes, hence this structure is used for both.
 */
struct al_unit_adapter_err_attr {
	/**
	 * For units that do not check parity themselves, the parity is checked
	 * inside the adaptor (only applicable for read transactions)
	 */
	al_bool read_parity_error;

	/**
	 * True when the AXI sub-master tries to initiate a transaction
	 * when bit BME (Bus Master Enable) is cleared to 0
	 */
	al_bool error_blocked;

	/**
	 * Ture when the AXI sub-master does not get an acknowledge completion
	 */
	al_bool completion_timeout;

	/**
	 * True when the bus response of the transaction returns with an error
	 */
	al_bool completion_error;

	/**
	 * True when the AXI sub master does not get a transaction address
	 * acknowledge
	 */
	al_bool address_timeout;

	/** ID of the sub-master that initiated the erroneous transaction */
	unsigned int error_master_id;

	/**
	 * When a transaction is completed with an error, this field captures
	 * the error completion status.
	 */
	unsigned int error_completion_status;

	/** Captured address related to the error - higher 32 bits */
	uint32_t error_address_latch_hi;

	/** Captured address related to the error - lower 32 bits */
	uint32_t error_address_latch_lo;
};

/**
 * Unit adapter Application Parity error attributes
 * This structure groups the set of SRAM's that can have parity errors
 */
struct al_unit_adapter_app_parity_err_status {
	/**
	 *Total parity errors
	 */
	int total_errors;
	/**
	 * SATA port 0 TX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_0_tx_ram;
	/**
	 * SATA port 1 TX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_1_tx_ram;
	/**
	 * SATA port 2 TX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_2_tx_ram;
	/**
	 * SATA port 3 TX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_3_tx_ram;
	/**
	 * SATA port 0 RX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_0_rx_ram;
	/**
	 * SATA port 1 RX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_1_rx_ram;
	/**
	 * SATA port 2 RX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_2_rx_ram;
	/**
	 * SATA port 3 RX RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_3_rx_ram;
	/**
	 * SATA port 0 FB low RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_0_fb_low_ram;
	/**
	 * SATA port 1 FB low RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_1_fb_low_ram;
	/**
	 * SATA port 2 FB low RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_2_fb_low_ram;
	/**
	 * SATA port 3 FB low RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_3_fb_low_ram;
	/**
	 * SATA port 0 FB high RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_0_fb_high_ram;
	/**
	 * SATA port 1 FB high RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_1_fb_high_ram;
	/**
	 * SATA port 2 FB high RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_2_fb_high_ram;
	/**
	 * SATA port 3 FB high RAM parity error
	 * Relevant only in SATA unit adapter
	 */
	al_bool sata_port_3_fb_high_ram;
	/**
	 * ROB out SRAM parity error
	 */
	al_bool dbg_axi_urob_sram_out;
	/**
	 *ROB in SRAM parity error
	 */
	al_bool dbg_axi_urob_sram_in;
};

/**
 *  SATA Target-ID MSIX control configuration
 */
struct al_sata_tgtid_msix_conf {
	/* If acces_en == true, enable write to all tgtid_n registers in the
	 * MSI-X Controller.
	 */
	al_bool access_en;

	/* If sel == true ,use tgtid_n [15:0] from MSI-X Controller as AxUSER[19:4] field for
	 * MSI-X message. Else use AxUser[19:4] as configured in the PCI adapter.
	 */
	al_bool sel;
};

/**
 * SATA adapter Target-ID configuration
 */
struct al_sata_tgtid_advanced_conf {
	/*
	 * For Tx/Rx , replacement bits for the original address.
	 * The number of bits replaced is determined according to
	 * 'addr_hi_sel'
	 */
	unsigned int addr_hi;

	/*
	 * For Tx/Rx , 6 bits serving the number of bits taken from the
	 * extra register on account of bits coming from the original address
	 * field.
	 * When 'addr_hi_sel'=32 all of 'tx_addr_hi' will be taken.
	 * When 'addr_hi_sel'=0 none of it will be taken, and when any
	 * value in between, it will start from the MSB bit and sweep down as
	 * many bits as needed. For example if 'addr_hi_sel'=8, the final
	 * address [63:56] will carry 'tx_addr_hi'[31:24] while [55:32] will
	 * carry the original buffer address[55:32].
	 */
	unsigned int addr_hi_sel;

	/*
	 * Tx/Rx Target-ID
	 * Masked per bit with 'tgtid_mask'
	 */
	unsigned int tgtid;

	/*
	 * Tx/Rx Target-ID mask
	 * Each '1' selects from the buffer address, each '0' selects from
	 * 'tgtid'
	 */
	unsigned int tgtid_mask;
};


/** Enum definitions **/

/* API definitions */

/**
 * Handle initialization
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	type
 *		The unit adapter type
 * @param	unit_adapter_regs_base
 *		The base address of the unit adapter registers
 * @param	pcie_read_config_u32
 *		pointer to function that reads register from pcie config space
 *		if NULL is passed, al_reg_read32 from unit_adapter_regs_base
 *		is used
 * @param	pcie_write_config_u32
 *		register to pcie config space
 *		if NULL is passed, al_reg_write32 from unit_adapter_regs_base
 *		is used
 * @param	pcie_flr
 *		pointer to function that makes the actual reset.
 *		That function is responsible for performing the post reset
 *		delay.
 *		if NULL is passed, FLR is performed through the device
 *		capability register.
 * @param	handle
 *		pointer passes to the above functions as first parameter
 *
 * @return	0 if finished successfully
 *		<0 if an error occurred
 */
int al_unit_adapter_handle_init(
	struct al_unit_adapter *unit_adapter,
	enum al_unit_adapter_type type,
	void __iomem *unit_adapter_regs_base,
	int (* pcie_read_config_u32)(void *handle, int where, uint32_t *val),
	int (* pcie_write_config_u32)(void *handle, int where, uint32_t val),
	int (* pcie_flr)(void *handle),
	void *handle);

/**
 * Set AXI master timeout configuration (disabled by default)
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	wr_timeout
 *		AXI Master write timeout threshold (in I/O fabric clock cycles).
 *		A value of 0 disables the timer.
 *		This value is multiplied by 4.
 * @param	rd_timeout
 *		AXI Master read timeout threshold (in I/O fabric clock cycles).
 *		A value of 0 disables the timer.
 *		This value is multiplied by 4.
 */
void al_unit_adapter_axi_master_timeout_config(
	struct al_unit_adapter	*unit_adapter,
	unsigned int		wr_timeout,
	unsigned int		rd_timeout);

/**
 * Snooping enable/disable
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	func_num
 *		Function number
 * @param	snoop_en
 *		Snooping enabled
 */
void al_unit_adapter_snoop_enable(
	struct al_unit_adapter	*unit_adapter,
	unsigned int	func_num,
	al_bool	snoop_en);

/**
 * Adapter errors tracking enable/disable
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	func_num
 *		Function number
 * @param	error_track_en
 *		Errors tracking enabled
 */
void al_unit_adapter_error_track_enable(
	struct al_unit_adapter	*unit_adapter,
	unsigned int	func_num,
	al_bool	error_track_en);

/**
 * Enable interrupt from adapter
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	intr_en
 *		Interrupt from adapter enabled
 */
void al_unit_adapter_int_enable(
	struct al_unit_adapter	*unit_adapter,
	al_bool			intr_en);

/**
 * Set interrupt cause mask
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	wr_err_en
 *		AXI master write error interrupt enabled
 * @param	rd_err_en
 *		AXI master read error interrupt enabled
 */
void al_unit_adapter_int_cause_mask_set(
	struct al_unit_adapter	*unit_adapter,
	al_bool			wr_err_en,
	al_bool			rd_err_en);

/**
 * Set interrupt cause mask extended
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	params
 *		unit_adapter Interrupt fields instance
 */
void al_unit_adapter_int_cause_mask_set_ex(
	struct al_unit_adapter	*unit_adapter,
	struct al_unit_adapter_int_fields *params);

/**
 * Get & clear interrupt cause
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	wr_err
 *		AXI master write error
 * @param	rd_err
 *		AXI master read error
 */
void al_unit_adapter_int_cause_get_and_clear(
	struct al_unit_adapter	*unit_adapter,
	al_bool			*wr_err,
	al_bool			*rd_err);

/**
 * Get & clear interrupt cause extended
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	params
 *		unit_adapter Interrupt fields instance
 */
void al_unit_adapter_int_cause_get_and_clear_ex(
	struct al_unit_adapter	*unit_adapter,
	struct al_unit_adapter_int_fields *params);

/**
 * Get & clear last captured AXI master write error attributes
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	wr_err_attr
 *		AXI master write error attributes
 */
void al_unit_adapter_axi_master_wr_err_attr_get_and_clear(
	struct al_unit_adapter		*unit_adapter,
	struct al_unit_adapter_err_attr	*wr_err_attr);

/**
 * Get & clear last captured AXI master read error attributes
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	rd_err_attr
 *		AXI master read error attributes
 */
void al_unit_adapter_axi_master_rd_err_attr_get_and_clear(
	struct al_unit_adapter		*unit_adapter,
	struct al_unit_adapter_err_attr	*rd_err_attr);

/**
 * Get & clear last captured Application Parity error status
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	err_attr
 *		Application parity error attributes
 */
void al_unit_adapter_app_parity_status_get_and_clear(
	struct al_unit_adapter *unit_adapter,
	struct al_unit_adapter_app_parity_err_status *err_attr);

/**
 * Perform function level reset and takes care for all needed PCIe config space
 * register save and restore.
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 */
void al_pcie_perform_flr(
	struct al_unit_adapter	*unit_adapter);

/**
 * Reorder buffer (ROB) control
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	rd_rob_en
 *		Read ROB Enable, when disabled the read ROB is bypassed
 * @param	rd_rob_force_in_order
 *		Read force in-order of every read transaction
 * @param	wr_rob_en
 *		Write ROB Enable, when disabled the Write ROB is bypassed
 * @param	wr_rob_force_in_order
 *		Write force in-order of every write transaction
 */
void al_unit_adapter_rob_cfg(
	struct al_unit_adapter	*unit_adapter,
	al_bool			rd_rob_en,
	al_bool			rd_rob_force_in_order,
	al_bool			wr_rob_en,
	al_bool			wr_rob_force_in_order);

/**
 * Internal memories shutdown control
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	en
 *		Internal memories shutdown enable/disable
 */
void al_unit_adapter_mem_shutdown_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en);

/**
 * Clock gating control
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	en
 *		Clock gating enable/disable
 */
void al_unit_adapter_clk_gating_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en);

/**
 * Powerdown control
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	en
 *		Powerdown enable/disable (D3)
 */
void al_unit_adapter_pd_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en);

/** SATA Target-ID MSI-X control configuration
 *  *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	conf
 *		SATA Target-ID MSI-X configuration
 */
void al_sata_tgtid_msix_conf_set(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_msix_conf	*conf);

/**
 * SATA Target-ID control advanced Tx configuration
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	conf
 *		SATA Target-ID Tx configuration
 */
void al_unit_sata_adapter_advanced_tgtid_tx_conf(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_advanced_conf	*conf);

/**
 * SATA Target-ID control advanced Rx configuration
 *
 * @param	unit_adapter
 *		The unit_adapter instance
 * @param	conf
 *		SATA Target-ID Rx configuration
 */
void al_unit_sata_adapter_advanced_tgtid_rx_conf(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_advanced_conf	*conf);

#endif

/** @} end of UNIT_ADAPTER group */
