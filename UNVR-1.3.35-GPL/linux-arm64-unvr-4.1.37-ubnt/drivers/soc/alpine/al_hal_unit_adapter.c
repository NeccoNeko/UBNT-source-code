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
 * @defgroup group_unit_adapter Unit Adapter
 *  @{
 * @file   al_hal_unit_adapter.c
 *
 * @brief  Unit Adapter HAL
 *
 */

#include "al_hal_unit_adapter.h"
#include "al_hal_unit_adapter_regs.h"

#define ADAPTER_READ32(where, val)	\
	do {				\
		unit_adapter->pcie_read_config_u32(unit_adapter->handle, (where), (val));	\
	} while (0)

#define ADAPTER_WRITE32(where, val)	\
	do {				\
		unit_adapter->pcie_write_config_u32(unit_adapter->handle, (where), (val));	\
	} while (0)

#define ADAPTER_WRITE32_MASKED(where, mask, val)			\
	do {								\
		uint32_t temp;						\
		ADAPTER_READ32(where, &temp);				\
		ADAPTER_WRITE32(where, AL_MASK_VAL(mask, val, temp));	\
	} while (0)

/******************************************************************************
 ******************************************************************************/
static int al_unit_adapter_read_config_u32(
	void		*handle,
	int		where,
	uint32_t	*val)
{
	*val = al_reg_read32((uint32_t *)(((uint8_t *)handle) + where));

	return 0;
}

/******************************************************************************
 ******************************************************************************/
static int al_unit_adapter_write_config_u32(
	void		*handle,
	int		where,
	uint32_t	val)
{
	al_reg_write32((uint32_t *)(((uint8_t *)handle) + where), val);

	return 0;
}

/******************************************************************************
 ******************************************************************************/
static void al_unit_adapter_axi_master_err_attr_get_and_clear(
	struct al_unit_adapter		*unit_adapter,
	struct al_unit_adapter_err_attr	*err_attr,
	int				is_read)
{
	uint32_t val;

	if (is_read)
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_RD_ERR_ATTR, &val);
	else
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_WR_ERR_ATTR, &val);

	err_attr->read_parity_error = (val & AL_ADAPTER_AXI_MSTR_RD_ERR_ATTR_RD_PARITY_ERR) ? AL_TRUE : AL_FALSE;
	err_attr->error_blocked = (val & AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_ERR_BLK) ? AL_TRUE : AL_FALSE;
	err_attr->completion_timeout = (val & AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_TO) ? AL_TRUE : AL_FALSE;
	err_attr->completion_error = (val & AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_ERR) ? AL_TRUE : AL_FALSE;
	err_attr->address_timeout = (val & AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_ADDR_TO) ? AL_TRUE : AL_FALSE;
	err_attr->error_master_id = AL_REG_FIELD_GET(val, AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_MASK, AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_SHIFT);
	err_attr->error_completion_status = AL_REG_FIELD_GET(val, AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_MASK, AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_SHIFT);

	if (is_read) {
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_RD_ERR_HI_LATCH, &err_attr->error_address_latch_hi);
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_RD_ERR_LO_LATCH, &err_attr->error_address_latch_lo);
	} else {
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_WR_ERR_HI_LATCH, &err_attr->error_address_latch_hi);
		ADAPTER_READ32(AL_ADAPTER_AXI_MSTR_WR_ERR_LO_LATCH, &err_attr->error_address_latch_lo);
	}
}

/******************************************************************************
 ******************************************************************************/
int al_unit_adapter_handle_init(
	struct al_unit_adapter *unit_adapter,
	enum al_unit_adapter_type type,
	void __iomem *unit_adapter_regs_base,
	int (* pcie_read_config_u32)(void *handle, int where, uint32_t *val),
	int (* pcie_write_config_u32)(void *handle, int where, uint32_t val),
	int (* pcie_flr)(void *handle),
	void *handle)
{
	al_assert(unit_adapter);

	unit_adapter->type = type;
	unit_adapter->regs = unit_adapter_regs_base;

	unit_adapter->pcie_read_config_u32 =
		pcie_read_config_u32 ?
		pcie_read_config_u32 :
		al_unit_adapter_read_config_u32;

	unit_adapter->pcie_write_config_u32 =
		pcie_write_config_u32 ?
		pcie_write_config_u32 :
		al_unit_adapter_write_config_u32;

	unit_adapter->pcie_flr = pcie_flr;

	unit_adapter->handle =
		handle ?
		handle :
		unit_adapter_regs_base;

	return 0;
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_axi_master_timeout_config(
	struct al_unit_adapter	*unit_adapter,
	unsigned int		wr_timeout,
	unsigned int		rd_timeout)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_AXI_MSTR_TO,
		AL_ADAPTER_AXI_MSTR_TO_WR_MASK |
		AL_ADAPTER_AXI_MSTR_TO_RD_MASK,
		(wr_timeout << AL_ADAPTER_AXI_MSTR_TO_WR_SHIFT) |
		(rd_timeout << AL_ADAPTER_AXI_MSTR_TO_RD_SHIFT));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_snoop_enable(
	struct al_unit_adapter	*unit_adapter,
	unsigned int	func_num,
	al_bool	snoop_en)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_SMCC + (AL_ADAPTER_SMCC_BUNDLE_SIZE * func_num),
		AL_ADAPTER_SMCC_CONF_SNOOP_OVR |
		AL_ADAPTER_SMCC_CONF_SNOOP_ENABLE,
		(snoop_en ?
			(AL_ADAPTER_SMCC_CONF_SNOOP_OVR |
			AL_ADAPTER_SMCC_CONF_SNOOP_ENABLE) : 0));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_error_track_enable(
	struct al_unit_adapter	*unit_adapter,
	unsigned int	func_num,
	al_bool	error_track_en)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_SMCC_CONF_2 +
			(AL_ADAPTER_SMCC_BUNDLE_SIZE * func_num),
		AL_ADAPTER_SMCC_CONF_2_DIS_ERROR_TRACK,
		(error_track_en ? 0 :
			AL_ADAPTER_SMCC_CONF_2_DIS_ERROR_TRACK));

}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_int_enable(
	struct al_unit_adapter	*unit_adapter,
	al_bool			intr_en)
{
	ADAPTER_WRITE32_MASKED(
		AL_PCI_COMMAND,
		AL_PCI_COMMAND_INTR_DISABLE,
		(intr_en ? 0 : AL_PCI_COMMAND_INTR_DISABLE));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_int_cause_mask_set(
	struct al_unit_adapter	*unit_adapter,
	al_bool			wr_err_en,
	al_bool			rd_err_en)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_INT_CAUSE_MASK,
		AL_ADAPTER_INT_CAUSE_MASK_WR_ERR |
		AL_ADAPTER_INT_CAUSE_MASK_RD_ERR,
		(wr_err_en ? 0 : AL_ADAPTER_INT_CAUSE_MASK_WR_ERR) |
		(rd_err_en ? 0 : AL_ADAPTER_INT_CAUSE_MASK_RD_ERR));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_int_cause_mask_set_ex(
	struct al_unit_adapter	*unit_adapter,
	struct al_unit_adapter_int_fields *params)
{
	al_assert(unit_adapter);
	al_assert(params);

	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_INT_CAUSE_MASK,
		AL_ADAPTER_INT_CAUSE_MASK_WR_ERR |
		AL_ADAPTER_INT_CAUSE_MASK_RD_ERR |
		AL_ADAPTER_INT_CAUSE_MASK_APP_PARITY_ERR,
		(params->wr_err ? 0 : AL_ADAPTER_INT_CAUSE_MASK_WR_ERR) |
		(params->rd_err ? 0 : AL_ADAPTER_INT_CAUSE_MASK_RD_ERR) |
		(params->app_parity_error ? 0 : AL_ADAPTER_INT_CAUSE_MASK_APP_PARITY_ERR));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_int_cause_get_and_clear(
	struct al_unit_adapter	*unit_adapter,
	al_bool			*wr_err,
	al_bool			*rd_err)
{
	uint32_t reg_val;

	ADAPTER_READ32(AL_ADAPTER_INT_CAUSE, &reg_val);

	*wr_err = (reg_val & AL_ADAPTER_INT_CAUSE_WR_ERR) ? AL_TRUE : AL_FALSE;
	*rd_err = (reg_val & AL_ADAPTER_INT_CAUSE_RD_ERR) ? AL_TRUE : AL_FALSE;
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_int_cause_get_and_clear_ex(
	struct al_unit_adapter	*unit_adapter,
	struct al_unit_adapter_int_fields *params)
{
	uint32_t reg_val;

	al_assert(unit_adapter);
	al_assert(params);

	/* read clears this register */
	ADAPTER_READ32(AL_ADAPTER_INT_CAUSE, &reg_val);

	params->wr_err = (reg_val & AL_ADAPTER_INT_CAUSE_WR_ERR) ? AL_TRUE : AL_FALSE;
	params->rd_err = (reg_val & AL_ADAPTER_INT_CAUSE_RD_ERR) ? AL_TRUE : AL_FALSE;
	params->app_parity_error = (reg_val & AL_ADAPTER_INT_CAUSE_MASK_APP_PARITY_ERR) ?
					AL_TRUE : AL_FALSE;
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_axi_master_wr_err_attr_get_and_clear(
	struct al_unit_adapter		*unit_adapter,
	struct al_unit_adapter_err_attr	*wr_err_attr)
{
	al_unit_adapter_axi_master_err_attr_get_and_clear(
		unit_adapter,
		wr_err_attr,
		0);
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_axi_master_rd_err_attr_get_and_clear(
	struct al_unit_adapter		*unit_adapter,
	struct al_unit_adapter_err_attr	*rd_err_attr)
{
	al_unit_adapter_axi_master_err_attr_get_and_clear(
		unit_adapter,
		rd_err_attr,
		1);
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_app_parity_status_get_and_clear(
	struct al_unit_adapter *unit_adapter,
	struct al_unit_adapter_app_parity_err_status *status)
{
	uint32_t val;

	al_assert(unit_adapter);
	al_assert(status);

	/* read clears this register */
	ADAPTER_READ32(AL_ADAPTER_APP_PARITY_STATUS, &val);

	status->total_errors = al_popcount(val);

	status->sata_port_0_tx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_TX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_1_tx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_TX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_2_tx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_TX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_3_tx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_TX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_0_rx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_RX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_1_rx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_RX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_2_rx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_RX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_3_rx_ram = (val & AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_RX_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_0_fb_low_ram = (val &
					  AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_FB_LOW_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_1_fb_low_ram = (val &
					  AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_FB_LOW_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_2_fb_low_ram = (val &
					  AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_FB_LOW_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_3_fb_low_ram = (val &
					  AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_FB_LOW_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_0_fb_high_ram = (val &
					   AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_FB_HIGH_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_1_fb_high_ram = (val &
					   AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_FB_HIGH_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_2_fb_high_ram = (val &
					   AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_FB_HIGH_RAM) ?
						AL_TRUE : AL_FALSE;
	status->sata_port_3_fb_high_ram = (val &
					   AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_FB_HIGH_RAM) ?
						AL_TRUE : AL_FALSE;
	status->dbg_axi_urob_sram_out = (val & AL_ADAPTER_APP_PARITY_STATUS_DBG_AXI_UROB_SRAM_OUT) ?
						AL_TRUE : AL_FALSE;
	status->dbg_axi_urob_sram_in = (val & AL_ADAPTER_APP_PARITY_STATUS_DBG_AXI_UROB_SRAM_IN) ?
						AL_TRUE : AL_FALSE;
}

/******************************************************************************
 ******************************************************************************/
void al_pcie_perform_flr(
	struct al_unit_adapter	*unit_adapter)
{
	int i;
	uint32_t cfg_reg_store[15];

	i = 0;
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_COMMAND, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_DEVICE_ID, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_0, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_0_HI, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_2, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_2_HI, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_4, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_4_HI, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_0, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_1, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_2, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_3, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_APP_CONTROL, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_PCI_VF_BASE_ADDRESS_0, &cfg_reg_store[i++]);
	unit_adapter->pcie_read_config_u32(
		unit_adapter->handle, AL_ADAPTER_SMCC_CONF_2, &cfg_reg_store[i++]);


	unit_adapter->pcie_flr(unit_adapter->handle);

	i = 0;
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_COMMAND, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_DEVICE_ID, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_0, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_0_HI, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_2, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_2_HI, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_4, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_BASE_ADDRESS_4_HI, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_0, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_1, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_2, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_AXI_CFG_AND_CTR_3, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_APP_CONTROL, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_PCI_VF_BASE_ADDRESS_0, cfg_reg_store[i++]);
	unit_adapter->pcie_write_config_u32(
		unit_adapter->handle, AL_ADAPTER_SMCC_CONF_2, cfg_reg_store[i++]);
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_rob_cfg(
	struct al_unit_adapter	*unit_adapter,
	al_bool			rd_rob_en,
	al_bool			rd_rob_force_in_order,
	al_bool			wr_rob_en,
	al_bool			wr_rob_force_in_order)
{
	/**
	 * Addressing RMN: TBD
	 *
	 * RMN description:
	 * TBD
	 * Software flow:
	 * Reset ROB prior to it's configuration
	 */
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_GENERIC_CONTROL_19,
		AL_ADPTR_GEN_CTL_19_READ_ROB_SW_RESET | AL_ADPTR_GEN_CTL_19_WRITE_ROB_SW_RESET,
		AL_ADPTR_GEN_CTL_19_READ_ROB_SW_RESET | AL_ADPTR_GEN_CTL_19_WRITE_ROB_SW_RESET);
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_GENERIC_CONTROL_19,
		AL_ADPTR_GEN_CTL_19_READ_ROB_SW_RESET | AL_ADPTR_GEN_CTL_19_WRITE_ROB_SW_RESET, 0);

	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_GENERIC_CONTROL_19,
		AL_ADPTR_GEN_CTL_19_READ_ROB_EN |
		AL_ADPTR_GEN_CTL_19_READ_ROB_FORCE_INORDER |
		AL_ADPTR_GEN_CTL_19_WRITE_ROB_EN |
		AL_ADPTR_GEN_CTL_19_WRITE_ROB_FORCE_INORDER,
		(rd_rob_en ? AL_ADPTR_GEN_CTL_19_READ_ROB_EN : 0) |
		(rd_rob_force_in_order ? AL_ADPTR_GEN_CTL_19_READ_ROB_FORCE_INORDER : 0) |
		(wr_rob_en ? AL_ADPTR_GEN_CTL_19_WRITE_ROB_EN : 0) |
		(wr_rob_force_in_order ? AL_ADPTR_GEN_CTL_19_WRITE_ROB_FORCE_INORDER : 0));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_mem_shutdown_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en)
{
	if (unit_adapter->type == AL_UNIT_ADAPTER_TYPE_SATA) {
		ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_3,
			0x10000000,
			en ? 0x10000000 : 0);
	} else {
		ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_3,
			0x8f000000,
			en ? 0x8f000000 : 0);
	}
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_clk_gating_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_GENERIC_CONTROL_0,
		AL_ADAPTER_GENERIC_CONTROL_0_CLK_GATE_EN,
		en ? AL_ADAPTER_GENERIC_CONTROL_0_CLK_GATE_EN : 0);
}

/******************************************************************************
 ******************************************************************************/
void al_unit_adapter_pd_ctrl(
	struct al_unit_adapter	*unit_adapter,
	al_bool			en)
{
	ADAPTER_WRITE32_MASKED(
		AL_ADAPTER_PM_1,
		AL_ADAPTER_PM_1_PME_EN | AL_ADAPTER_PM_1_PWR_STATE_D3,
		en ? (AL_ADAPTER_PM_1_PME_EN | AL_ADAPTER_PM_1_PWR_STATE_D3) : 0);
}

/******************************************************************************
 ******************************************************************************/
void al_sata_tgtid_msix_conf_set(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_msix_conf	*conf)
{
	al_assert(unit_adapter->type == AL_UNIT_ADAPTER_TYPE_SATA);

	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_14,
			AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_SEL |
			AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_ACCESS_EN,
			(conf->access_en ? AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_ACCESS_EN : 0) |
			(conf->sel ? AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_SEL : 0));
}

/******************************************************************************
 ******************************************************************************/
void al_unit_sata_adapter_advanced_tgtid_tx_conf(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_advanced_conf	*conf)
{
	al_assert(unit_adapter->type == AL_UNIT_ADAPTER_TYPE_SATA);

	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_13,
			AL_ADPTR_GEN_CTL_13_SATA_ARUSER_SEL_MASK |
			AL_ADPTR_GEN_CTL_13_SATA_ARUSER_VAL_MASK,
			(conf->tgtid_mask << AL_ADPTR_GEN_CTL_13_SATA_ARUSER_SEL_SHIFT) |
			(conf->tgtid << AL_ADPTR_GEN_CTL_13_SATA_ARUSER_VAL_SHIFT)
	);
	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_14,
			AL_ADPTR_GEN_CTL_14_SATA_VM_ARADDR_SEL_MASK,
			conf->addr_hi_sel << AL_ADPTR_GEN_CTL_14_SATA_VM_ARADDR_SEL_SHIFT
	);
	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_16,
			AL_ADPTR_GEN_CTL_16_SATA_VM_ARDDR_HI,
			conf->addr_hi
	);
}

/******************************************************************************
 ******************************************************************************/
void al_unit_sata_adapter_advanced_tgtid_rx_conf(
		struct al_unit_adapter	*unit_adapter,
		struct al_sata_tgtid_advanced_conf	*conf)
{
	al_assert(unit_adapter->type == AL_UNIT_ADAPTER_TYPE_SATA);

	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_12,
			AL_ADPTR_GEN_CTL_12_SATA_AWUSER_SEL_MASK |
			AL_ADPTR_GEN_CTL_12_SATA_AWUSER_VAL_MASK,
			(conf->tgtid_mask << AL_ADPTR_GEN_CTL_12_SATA_AWUSER_SEL_SHIFT) |
			(conf->tgtid << AL_ADPTR_GEN_CTL_12_SATA_AWUSER_VAL_SHIFT)
	);
	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_14,
			AL_ADPTR_GEN_CTL_14_SATA_VM_AWADDR_SEL_MASK,
			conf->addr_hi_sel << AL_ADPTR_GEN_CTL_14_SATA_VM_AWADDR_SEL_SHIFT
	);
	ADAPTER_WRITE32_MASKED(
			AL_ADAPTER_GENERIC_CONTROL_15,
			AL_ADPTR_GEN_CTL_15_SATA_VM_AWDDR_HI,
			conf->addr_hi
	);
}
/** @} */
