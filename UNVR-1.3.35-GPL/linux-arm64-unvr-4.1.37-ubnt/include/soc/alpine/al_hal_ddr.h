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
 * @defgroup groupddr DDR controller & PHY hardrware abstraction layer
 *  @{
 * @file   al_hal_ddr.h
 *
 * @brief Header file for the DDR HAL driver
 */

#ifndef __AL_HAL_DDR_H__
#define __AL_HAL_DDR_H__

#include "al_hal_common.h"
#include "al_hal_ddr_cfg.h"

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

/**
 * DDR address mapping - not connected bit
 * See explanation about al_ddr_addrmap below.
 */
#define AL_DDR_ADDRMAP_NC	0xff

#define AL_DDR_BIST_DQ_BITS	8

enum al_ddr_revision {
	AL_DDR_REV_ID_ALPINE_V1 = 1,
	AL_DDR_REV_ID_ALPINE_V2 = 2,
	AL_DDR_REV_ID_ALPINE_V3 = 3,
};

/*
 * Data Width
 * For Alpine V1 : AL_DDR_DATA_WIDTH_16_BITS is not supported
 */
enum al_ddr_data_width {
	AL_DDR_DATA_WIDTH_32_BITS,
	AL_DDR_DATA_WIDTH_64_BITS,
	AL_DDR_DATA_WIDTH_16_BITS,
};

/*
 * PHY ROUT: Selects DDR PHY Output impedance
 * Applicable to both DQ and A/C
 */
enum al_ddr_phy_rout {
	AL_DDR_PHY_ROUT_80OHM,
	AL_DDR_PHY_ROUT_68OHM,
	AL_DDR_PHY_ROUT_60OHM,
	AL_DDR_PHY_ROUT_53OHM,
	AL_DDR_PHY_ROUT_48OHM,
	AL_DDR_PHY_ROUT_44OHM,
	AL_DDR_PHY_ROUT_40OHM,
	AL_DDR_PHY_ROUT_37OHM,
	AL_DDR_PHY_ROUT_34OHM,
	AL_DDR_PHY_ROUT_32OHM,
	AL_DDR_PHY_ROUT_30OHM,
};

/* PHY ODT: Selects DDR PHY On-die termination */
enum al_ddr_phy_odt {
	AL_DDR_PHY_ODT_200OHM,
	AL_DDR_PHY_ODT_133OHM,
	AL_DDR_PHY_ODT_100OHM,
	AL_DDR_PHY_ODT_77OHM,
	AL_DDR_PHY_ODT_66OHM,
	AL_DDR_PHY_ODT_56OHM,
	AL_DDR_PHY_ODT_50OHM,
	AL_DDR_PHY_ODT_44OHM,
	AL_DDR_PHY_ODT_40OHM,
	AL_DDR_PHY_ODT_36OHM,
	AL_DDR_PHY_ODT_33OHM,
	AL_DDR_PHY_ODT_30OHM,
	AL_DDR_PHY_ODT_28OHM,
	AL_DDR_PHY_ODT_26OHM,
	AL_DDR_PHY_ODT_25OHM,
};

/**
 * Address mapping:
 * Read and write requests are provided to the DDR controller with a system
 * address.  The system address is the command address of a transaction as
 * presented on one of the data ports inside the system fabric. The DDR controller
 * is responsible for mapping this system address to rank, bank, row, and column
 * addresses to the SDRAM. It converts the system address to a physical address.
 * For each CS/bank/column/row bit assign a system memory address bit index.
 * Set to AL_DDR_ADDRMAP_NC if not connected.
 * CS minimal supported memory address bit index is 10.
 * Bank minimal supported memory address bit index is 6.
 * Column minimal supported memory address bit index is 4.
 * Row minimal supported memory address bit index is 10.
 *
 * Note that as DDR3/4 supports 8 data beats, only column bit 3 and above are
 * controlled by the user
 *
 * swap_14_16_with_17_19 configuration is used to swap system address bits [16:14]
 * with system address bits [19:17]. This allows some more flexability when choosing
 * the address mapping.
 *
 * Address mapping might affect the system performance and should be optimized
 * according to the specific application nature. The basic guideline is keeping
 * as much open pages as possible and avoiding frequent closing of pages and
 * opening new ones.
 *
 * Example:
 * Mapping of 16GB memory device with 64 bits data width, 1KB page
 *
 * System address bit index |	SDRAM required mapping
 * ----------------------------------------------------
 * 33:32			cs[1:0]
 * 31:16			row[15:0]
 * 15:13			bank[2:0]
 * 12:3				col[9:0]
 * 2:0				N/A since 8 bytes are accessed at a time
 *
 * In this case the following setting is required:
 * col_b3_9_b11_13 = { 6, 7, 8, 9, 10, 11, 12, AL_DDR_ADDRMAP_NC, ... }
 * bank_b0_2 = { 13, 14, 15 }
 * row_b0_17 = { 16, 17, 18 , 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
 *		AL_DDR_ADDRMAP_NC, AL_DDR_ADDRMAP_NC}
 * cs_b0_1 = { 32, 33 }
 * swap_14_16_with_17_19 = 0
 *
 * Example 2:
 * Mapping of 16GB memory device with 64 bits data width, 1KB page
 * for reducing L2 evict collisions on same DDR page
 * (L2 tag is mapped to sysaddr[17] and up)
 *
 * System address bit index |	SDRAM required mapping
 * ----------------------------------------------------
 * 33:20			row[15:2]
 * 19:17			bank[2:0]
 * 16:15			row[1:0]
 * 14:13			cs[1:0]
 * 12:3				col[9:0]
 * 2:0				N/A since 8 bytes are accessed at a time
 *
 * In this case the following setting is required:
 * col_b3_9_b11_13 = { 6, 7, 8, 9, 10, 11, 12, AL_DDR_ADDRMAP_NC, ... }
 * bank_b0_2 = { 14, 15, 16 }	// will be replaced with sysaddr[19:17], see swap command below
 * sysaddr[15:16] will be swapped with sysaddr[18:19],
 * hence we'll assign row[0:1] to sysaddr[18:19]
 * row_b0_17 = { 18, 19, 20 , 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
 *		AL_DDR_ADDRMAP_NC, AL_DDR_ADDRMAP_NC}
 * cs_b0_1 = { 13, 17 }		// 17 will be replaced with sysaddr[14]
 * swap_14_16_with_17_19 = 1
 */
struct al_ddr_addrmap {
	/**
	 * Column bits 3 - 9, 11 - 13
	 * For 64 bits data bus, Bit 13 can be ignored
	 * For 32 bits data bus, Bit 3 can be ignored
	 * For 16 bits data bus, Bits 3, 4 can be ignored
	 */
	uint8_t col_b3_9_b11_13[10];

	/**
	 * Bank bits 0 - 2
	 * For DDR4 : bit 2 should be set to AL_DDR_ADDRMAP_NC
	 */
	uint8_t bank_b0_2[3];

	/**
	 * Bank Group bits 0 - 1
	 * Applicable to DDR4 only
	 * For DDR3 : should be set to AL_DDR_ADDRMAP_NC
	 */
	uint8_t bg_b0_1[2];

	/**
	 * Row bits 0 - 17
	 * For Alpine V1 : Bits 3 - 10 are not used and must follow bit 2
	 *	(if bit 2 is set with 10 for example, bit 3 must be 11, bit 4 must be 12, etc.)
	 * For Alpine V2 : Bits 6 - 10 are not used and must follow bit 6
	 *	(if bit 6 is set with 14 for example, bit 7 must be 15, bit 8 must be 16, etc.)
	 */
	uint8_t row_b0_17[18];

	/**
	 * CS bits 0 - 1
	 * This is to support up to 4 ranks
	 * If using less ranks, must set proper bits to AL_DDR_ADDRMAP_NC
	 */
	uint8_t	cs_b0_1[2];

	/**
	 * Swap addr[16:14] with addr[19:17]
	 * As not all row bits can be freely chosen (as described above), this option
	 * gives some more flexability for address mapping.
	 * See example above for related scenario.
	 */
	al_bool	swap_14_16_with_17_19;
};


/**
 * DDR BIST mode
 * Applicable only to data BIST
 */
enum al_ddr_bist_mode {
	/**
	 * Loopback mode:
	 * Address, commands and data loop back at the PHY I/Os
	 */
	AL_DDR_BIST_MODE_LOOPBACK,

	/**
	 * DRAM mode:
	 * Address, commands and data go to DRAM for normal memory accesses.
	 */
	AL_DDR_BIST_MODE_DRAM,
};

/* Data/AC BIST pattern */
enum al_ddr_bist_pat {
	/* Walking '0' */
	AL_DDR_BIST_PATTERN_WALK_0,

	/* Walking '1' */
	AL_DDR_BIST_PATTERN_WALK_1,

	/* LFSR-based pseudo-random */
	AL_DDR_BIST_PATTERN_LFSR,

	/* User programmable (Not valid for AC loopback) */
	AL_DDR_BIST_PATTERN_USER,
};

/* Data BIST parameters */
struct al_ddr_bist_params {
	/**
	 * Mode
	 * Applicable only to data BIST.
	 * A/C BIST always runs in Loopback
	 */
	enum al_ddr_bist_mode	mode;

	/* Pattern */
	enum al_ddr_bist_pat	pat;

	/**
	 * User Data Pattern 0:
	 * Data to be applied on even DQ pins during BIST.
	 * Valid values: 0x0000 - 0xffff
	 */
	unsigned int		user_pat_even;

	/**
	 * User Data Pattern 1:
	 * Data to be applied on odd DQ pins during BIST.
	 * Valid values: 0x0000 - 0xffff
	 */
	unsigned int		user_pat_odd;

	/** Word count
	 * Indicates the number of words to generate during BIST.
	 * Valid values are 4, 8, 12, 16, and so on.
	 * Maximal value: 0xfffc
	 */
	unsigned int		wc;

	/** Address increment
	 * Selects the value by which the SDRAM address is incremented for each
	 * write/read access. This value must be at the beginning of a burst
	 * boundary, i.e. the lower bits must be "000".
	 * Maximal value: 0xff8
	 */
	unsigned int		inc;

	/**
	 * BIST Column Address:
	 * Selects the SDRAM column address to be used during BIST. The lower
	 * bits of this address must be "000".
	 */
	unsigned int		col_min;

	/**
	 * BIST Maximum Column Address:
	 * Specifies the maximum SDRAM column address to be used during BIST
	 * before the address increments to the next row.
	 */
	unsigned int		col_max;

	/**
	 * BIST Row Address:
	 * Selects the SDRAM row address to be used during BIST.
	 */
	unsigned int		row_min;

	/**
	 * BIST Maximum Row Address:
	 * Specifies the maximum SDRAM row address to be used during BIST
	 * before the address increments to the next bank.
	 */
	unsigned int		row_max;

	/**
	 * BIST Bank Address:
	 * Selects the SDRAM bank address to be used during BIST.
	 * For DDR3 : bank_min[2:0] are bank address,
	 * For DDR4 : bank_min[1:0] are bank address,
	 *            bank_min[3:2] are bank group
	 */
	unsigned int		bank_min;

	/**
	 * BIST Maximum Bank Address:
	 * Specifies the maximum SDRAM bank address to be used during BIST
	 * before the address increments to the next rank.
	 * For DDR3 : bank_max[2:0] are bank address,
	 * For DDR4 : bank_max[1:0] are bank address,
	 *            bank_max[3:2] are bank group
	 */
	unsigned int		bank_max;

	/**
	 * BIST Rank:
	 * Selects the SDRAM rank to be used during BIST.
	 */
	unsigned int		rank_min;

	/**
	 * BIST Maximum Rank:
	 * Specifies the maximum SDRAM rank to be used during BIST.
	 */
	unsigned int		rank_max;

	/**
	 * Active byte lanes to have the BIST applied upon.
	 * User may use al_ddr_active_byte_lanes_get to get the currently active lanes
	 */
	int			active_byte_lanes[AL_DDR_PHY_NUM_BYTE_LANES];

	/**
	 * Run BIST on all active lanes simultaneously
	 * Not applicable for Alpine V1
	 */
	al_bool			all_lanes_active;
};

/* ECC status parameters */
struct al_ddr_ecc_status {
	/* Number of ECC errors detected */
	unsigned int err_cnt;

	/* Rank number of a read resulting in an ECC error */
	unsigned int rank;

	/* Bank number of a read resulting in an ECC error */
	unsigned int bank;

	/* Bank Group number of a read resulting in an ECC error */
	unsigned int bg;

	/* Row number of a read resulting in an ECC error */
	unsigned int row;

	/* Column number of a read resulting in an ECC error */
	unsigned int col;

	/* Data pattern that resulted in a corrected error */
	uint32_t syndromes_31_0;
	uint32_t syndromes_63_32;	/* For 32-bit ECC - not used. */
	uint32_t syndromes_ecc;		/* ECC lane */

	/**
	 * Mask for the corrected data portion
	 * 1 on any bit indicates that the bit has been corrected by the ECC
	 * logic
	 * 0 on any bit indicates that the bit has not been corrected by the
	 * ECC logic
	 * This register accumulates data over multiple ECC errors, to give an
	 * overall indication of which bits are being fixed. It is cleared by
	 * calling al_ddr_ecc_corr_int_clear.
	 */
	uint32_t corr_bit_mask_31_0;
	uint32_t corr_bit_mask_63_32;	/* For 32-bit ECC - not used. */
	uint32_t corr_bit_mask_ecc;	/* ECC lane */

	/* Bit number corrected by single-bit ECC error */
	unsigned int ecc_corrected_bit_num;
};


/* On-chip parity status parameters */
struct al_ddr_onchip_par_status {

	/* Read address On-Chip parity error */
	al_bool raddr_err;

	/* Write address On-Chip parity error */
	al_bool waddr_err;

	/* Read data On-Chip parity error */
	al_bool rdata_err;

	/* Write data On-Chip parity error on controller input port (from system fabric) */
	al_bool wdata_in_err;

	/* Write data On-Chip parity error on controller output port (DFI interface to PHY) */
	al_bool wdata_out_err;

	/**
	 * Rank number of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_rank;

	/**
	 * Bank number of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_bank;

	/**
	 * Bank Group number of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_bg;

	/**
	 * Row number of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_row;

	/**
	 * Column number of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_col;

	/**
	 * Failing byte location of a write resulting in an On-Chip parity error on output port
	 * (DFI interface to PHY)
	 */
	unsigned int wdata_out_byte_loc;

	/**
	 * System address of a write resulting in an On-Chip parity error on input port
	 * (from system fabric)
	 */
	al_phys_addr_t waddr_address;

	/**
	 * Failing byte location of a read resulting in an On-Chip parity error on input port
	 * (to system fabric)
	 */
	unsigned int rdata_byte_loc;

	/**
	 * System address of a read resulting in an On-Chip parity error on input port
	 * (from system fabric)
	 */
	al_phys_addr_t raddr_address;
};

struct al_ddr_cfg {
	/* North bridge registers base address */
	void __iomem			*nb_regs_base;

	/* DDR controller registers base address */
	void __iomem			*ddr_ctrl_regs_base;

	/* DDR PHY registers base address */
	void __iomem			*ddr_phy_regs_base;

	/* DDR revision */
	enum al_ddr_revision		rev;
};

struct al_ddr_ecc_cfg {
	/* ECC mode indicator */
	al_bool ecc_enabled;

	/* Enable ECC scrubs - applicable only when ecc is enabled */
	al_bool scrub_enabled;
};

struct al_ddr_bist_err_status {
	/* the lane of which the err occurred */
	int lane_err;
	/*
	 * Byte Word Error: Indicates the number of word errors on the byte lane
	 * An error on any bit of the data bus including
	 * the data mask bit increments the error count.
	 */
	uint16_t word_err;
	/*
	 * Data Bit Error: The error count for even DQS cycles.
	 * error count on the rising edge of DQS.
	 */
	uint8_t even_risg_err[AL_DDR_BIST_DQ_BITS];
	/*
	 * Data Bit Error: The error count for even DQS cycles.
	 * error count on the falling edge of DQS.
	 */
	uint8_t even_fall_err[AL_DDR_BIST_DQ_BITS];
	/*
	 * Data Bit Error: The error count for odd DQS cycles.
	 * error count on the rising edge of DQS.
	 */
	uint8_t odd_risg_err[AL_DDR_BIST_DQ_BITS];
	/*
	 * Data Bit Error: The error count for odd DQS cycles.
	 * error count on the falling edge of DQS.
	 */
	uint8_t odd_fall_err[AL_DDR_BIST_DQ_BITS];
	/*
	 * Byte Word Count: Indicates the number of words received from
	 * the byte lane.
	 */
	uint16_t word_count;
	/* Bit status during a word error for each of the 8 data (DQ) bits */
	uint8_t  beat_1st_err[AL_DDR_BIST_DQ_BITS];
	uint8_t  beat_2nd_err[AL_DDR_BIST_DQ_BITS];
	uint8_t  beat_3rd_err[AL_DDR_BIST_DQ_BITS];
	uint8_t  beat_4th_err[AL_DDR_BIST_DQ_BITS];
	/* failed on timeout */
	al_bool timeout;
};

/* DDR controller power modes */
enum al_ddr_power_mode {
	/* No power mode enabled */
	AL_DDR_POWERMODE_OFF,

	/**
	 * Self refresh:
	 * Puts the SDRAM into self refresh when no active transactions
	 */
	AL_DDR_POWERMODE_SELF_REFRESH,

	/**
	 * Power down:
	 * The DDR controller goes into power-down after a
	 * programmable number of idle cycles (Multiples of 32 clocks)
	 */
	AL_DDR_POWERMODE_POWER_DOWN,

	/**
	 * Maximum power saving:
	 * The DDR controller goes into maximum power saving mode.
	 * Exit should be requested explicitly by the user
	 * Only applicable for DDR4
	 */
	AL_DDR_POWERMODE_MPSM,

	/**
	 * Self refresh:
	 * Puts the SDRAM into self refresh immediately.
	 * Exit should be requested explicitly by the user
	 * Not applicable for Alpine V1
	 */
	AL_DDR_POWERMODE_SELF_REFRESH_SW,
};

/* DDR operating modes */
enum al_ddr_operating_mode {
	/* Initialiazation */
	AL_DDR_OPERATING_MODE_INIT,

	/* Normal operation */
	AL_DDR_OPERATING_MODE_NORMAL,

	/* Power down */
	AL_DDR_OPERATING_MODE_POWER_DOWN,

	/* Self refresh */
	AL_DDR_OPERATING_MODE_SELF_REFRESH,

};

/*
 *  must execute prior to 'al_ddr_phy_datx_bist'
 *  the function disables VT calc
 */
void al_ddr_phy_datx_bist_pre(
	struct al_ddr_cfg	*ddr_cfg);

/*
 *  must execute prior to 'al_ddr_phy_datx_bist'
 *  the function disables VT calc if requested
 */
void al_ddr_phy_datx_bist_pre_adv(
	struct al_ddr_cfg	*ddr_cfg,
	al_bool			vt_calc_disable);

/*
 *  must execute after call 'al_ddr_phy_datx_bist'
 *  the function enables VT calc
 */
void al_ddr_phy_datx_bist_post(
	struct al_ddr_cfg	*ddr_cfg);

/*
 *  must execute after call 'al_ddr_phy_datx_bist'
 *  the function enables VT calc if requested
 */
void al_ddr_phy_datx_bist_post_adv(
	struct al_ddr_cfg	*ddr_cfg,
	al_bool			vt_calc_enable);

/*
 * performs bist,
 * 'al_ddr_phy_datx_bist_pre' must be call prior to bist,
 * 'al_ddr_phy_datx_bist_post' must be call after bist done
 */
int al_ddr_phy_datx_bist(
	struct al_ddr_cfg		*ddr_cfg,
	struct al_ddr_bist_params	*params,
	struct al_ddr_bist_err_status	*bist_err_status);

int al_ddr_phy_ac_bist(
	struct al_ddr_cfg		*ddr_cfg,
	struct al_ddr_bist_err_status	*bist_err_status,
	enum al_ddr_bist_pat		pat);


/**
 * @brief Get DDR address mapping
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	addrmap
 *		DDR Address mapping
 */
void al_ddr_address_map_get(
	struct al_ddr_cfg	*ddr_cfg,
	struct al_ddr_addrmap	*addrmap);

/**
 * @brief Get current data bus width
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The data bus width
 */
enum al_ddr_data_width al_ddr_data_width_get(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Get the current number of available ranks
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The number of available ranks
 */
unsigned int al_ddr_active_ranks_get(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Get the logical rank number of a physical rank
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	rank
 *		Physical rank number
 *
 * @returns	Logical rank number
 */
unsigned int al_ddr_logical_rank_from_phys(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		rank);

/**
 * @brief Get the current number of available banks
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The number of available banks
 */
unsigned int al_ddr_active_banks_get(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Get the current number of available bank groups
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The number of available bank groups
 */
unsigned int al_ddr_active_bg_get(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Get the current number of available columns
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The number of available columns
 */
unsigned int al_ddr_active_columns_get(
	struct al_ddr_cfg	*ddr_cfg);


/**
 * @brief Get the current number of available rows
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	The number of available rows
 */
unsigned int al_ddr_active_rows_get(
	struct al_ddr_cfg	*ddr_cfg);


/**
 * @brief Get the current corrected/uncorrected error status
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	corr_status
 *		The corrected error status (use NULL if no status is required)
 * @param	uncorr_status
 *		The uncorrected error status (use NULL if no status is required)
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_ecc_status_get(
	struct al_ddr_cfg		*ddr_cfg,
	struct al_ddr_ecc_status	*corr_status,
	struct al_ddr_ecc_status	*uncorr_status);

/**
 * @brief Get the current ECC configuration
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	ecc_cfg
 *		The ECC configuration
 */
void al_ddr_ecc_cfg_get(
	struct al_ddr_cfg		*ddr_cfg,
	struct al_ddr_ecc_cfg		*ecc_cfg);

int al_ddr_ecc_corr_count_clear(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Clear the correctable error interrupt
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_ecc_corr_int_clear(
	struct al_ddr_cfg	*ddr_cfg);


int al_ddr_ecc_uncorr_count_clear(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Clear the uncorrectable error interrupt
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_ecc_uncorr_int_clear(
	struct al_ddr_cfg	*ddr_cfg);


/**
 * Data poisoning enable
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	rank
 *		Rank to poison
 * @param	bank
 *		Bank to poison
 * @param	bg
 *		Bank group to poison (DDR4 only)
 * @param	col
 *		Column to poison
 * @param	row
 *		Row to poison
 * @param	correctable
 *		Correctable or uncorrectable error poisoning (In Alpine V1 only uncorrectable
 *		poisoning is supported)
 *
 * @returns	0 upon success
 */
int al_ddr_ecc_data_poison_enable(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		rank,
	unsigned int		bank,
	unsigned int		bg,
	unsigned int		col,
	unsigned int		row,
	al_bool			correctable);

int al_ddr_ecc_data_poison_disable(
	struct al_ddr_cfg	*ddr_cfg);

unsigned int al_ddr_parity_count_get(
	struct al_ddr_cfg	*ddr_cfg);

void al_ddr_parity_count_clear(
	struct al_ddr_cfg	*ddr_cfg);

void al_ddr_parity_int_clear(
	struct al_ddr_cfg	*ddr_cfg);


/**
 * @brief Get the current on chip parity error status
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	status
 *		The on chip parity error status
 *
 * Not applicable to Alpine V1
 */
void al_ddr_onchip_parity_status_get(
	struct al_ddr_cfg		*ddr_cfg,
	struct al_ddr_onchip_par_status	*status);

/**
 * @brief Clear the on chip parity error interrupts
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * Not applicable to Alpine V1
 */
void al_ddr_onchip_parity_int_clear(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Unmask the on chip parity error interrupts
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	local_int_idx
 *		local interrupt index to unmask
 *
 * Not applicable to Alpine V1
 */
void al_ddr_onchip_parity_int_unmask(
	struct al_ddr_cfg		*ddr_cfg,
	unsigned int			local_int_idx);

/**
 * On-Chip parity poisoning enable
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	wr_inject
 *		Enables parity poisoning on write data at the system fabric interface
 *		before the input parity check logic
 * @param	rd_sf_inject
 *		Enables parity poisoning on read data at the system fabric interface
 *		after the parity check logic. An error injected here is not logged
 *		and does not trigger SLVERR or interrupt by the controller.
 *		Currently not supported
 * @param	rd_dfi_inject
 *		Enables parity poisoning on read data at the DFI interface
 *		after the parity generation logic.
 * @param	continuous
 *		Continuous mode: If set to 1, parity error is injected continuously
 *		on every read or write data beat passing through the interfaces determined
 *		by *inject parameters as long as poison is enabled.
 *		Trigger mode: If set to 0, parity error is injected for one data beat when
 *		a certain group of data patterns are detected on the selected interfaces
 *		determined by *inject parameters.
 * @param	byte_num
 *		Byte number (binary encoded) where the parity error is to be injected.
 * @param	pattern0
 *		Data patterns to be detected on the LSB of data interfaces for parity error
 *		injection. The error is injected at pattern1 when it comes right after pattern0.
 * @param	pattern1
 *		Data patterns to be detected on the LSB of data interfaces for parity error
 *		injection. The error is injected at pattern1 when it comes right after pattern0.
 *
 * Not applicable to Alpine V1
 */
void al_ddr_onchip_parity_poison_enable(
	struct al_ddr_cfg	*ddr_cfg,
	al_bool			wr_inject,
	al_bool			rd_sf_inject,
	al_bool			rd_dfi_inject,
	al_bool			continuous,
	unsigned int		byte_num,
	uint32_t		pattern0,
	uint32_t		pattern1);

/**
 * On-Chip parity poisoning disable
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * Not applicable to Alpine V1
 */
void al_ddr_onchip_parity_poison_disable(
	struct al_ddr_cfg	*ddr_cfg);

/**
 * @brief Set DDR power saving mode
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	power_mode
 *		Power mode to set
 * @param	clk_disable
 *		Whether to disable DDR clk when moving to power mode
 *		For DDR3 : only with Self refresh
 *		For DDR4 : only with Self refresh and MPSM
 *		For RDIMM this parameter must be AL_FALSE
 * @param	timer_x32
 *		number of cycles to wait after last transaction before
 *		changing power mode.
 *		For self refresh and power down only.
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_power_mode_set(
	struct al_ddr_cfg	*ddr_cfg,
	enum al_ddr_power_mode	power_mode,
	al_bool			clk_disable,
	unsigned int		timer_x32);

enum al_ddr_operating_mode al_ddr_operating_mode_get(
	struct al_ddr_cfg	*ddr_cfg);

int al_ddr_address_translate_sys2dram(
	struct al_ddr_cfg		*ddr_cfg,
	al_phys_addr_t			sys_address,
	unsigned int			*rank,
	unsigned int			*bank,
	unsigned int			*bg,
	unsigned int			*col,
	unsigned int			*row);

int al_ddr_address_translate_dram2sys(
	struct al_ddr_cfg		*ddr_cfg,
	al_phys_addr_t			*sys_address,
	unsigned int			rank,
	unsigned int			bank,
	unsigned int			bg,
	unsigned int			col,
	unsigned int			row);

/**
 * @brief Get the amount of connected address bits
 *
 * User can use these bits i.o. to calculate the memory device's rank size
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	Num of connected address bits (rank size == 1 << active_bits)
 */
unsigned int al_ddr_bits_per_rank_get(
	struct al_ddr_cfg	*ddr_cfg);


/**
 * Get which byte lanes are active
 * Depends on whether the bus width is 16, 32 or 64 bits and whether ECC is enabled
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	active_byte_lanes
 *		An array of active byte lanes
 *		'active_byte_lanes[i] == 1' means that byte lane 'i' is active
 */
void al_ddr_active_byte_lanes_get(
	struct al_ddr_cfg	*ddr_cfg,
	int		active_byte_lanes[AL_DDR_PHY_NUM_BYTE_LANES]);

/**
 * Get DRAM Mode Register
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	mr_index
 *		Mode register index
 *		DDR3 : value should be 0-3
 *		DDR4 : value should be 0-7
 *
 * @returns	MR value
 */
unsigned int al_ddr_mode_register_get(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		mr_index);

/**
 * Set DRAM Mode Register
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	ranks
 *		Determines which ranks will be set (1 bit per rank)
 *		For example : 0x5 - select ranks 0 and 2
 * @param	mr_index
 *		Mode register index
 *		DDR3 : value should be 0-3
 *		DDR4 : value should be 0-7
 * @param	mr_value
 *		Mode register value
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_mode_register_set(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		ranks,
	unsigned int		mr_index,
	unsigned int		mr_value);

/**
 * Get DDR PHY Rout and ODT
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	zq_segment
 *		ZQ segment to read from
 *		Valid values : 0-2
 * @param	phy_rout
 *		DDR PHY Output impedance
 * @param	phy_odt
 *		DDR PHY On-die termination
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_phy_zq_get(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		zq_segment,
	enum al_ddr_phy_rout	*phy_rout,
	enum al_ddr_phy_odt	*phy_odt);

/** DDR PHY Training Results Per Octet */
struct al_ddr_phy_training_results_per_octet {
	/**
	 * Initial Period: Initial period measured by the master delay line
	 * calibration for VT drift compensation. This value is used as the
	 * denominator when calculating the ratios of updates during VT
	 * compensation.
	 */
	uint8_t dll_num_taps_init;

	/**
	 * Target Period: Target period measured by the master delay line
	 * calibration for VT drift compensation. This is the current measured
	 * value of the period and is continuously updated if the MDL is
	 * enabled to do so.
	 */
	uint8_t dll_num_taps_curr;

	/** Write Leveling Delay (per rank) */
	uint8_t wld[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Write Leveling Delay for upper Nibble (per rank) */
	uint8_t x4wld[AL_DDR_NUM_RANKS];
	/** Write Leveling System Latency (per rank) */
	uint8_t wld_extra[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Write Leveling System Latency for upper Nibble (per rank) */
	uint8_t x4wld_extra[AL_DDR_NUM_RANKS];
	/** Read DQS Gating Delay (per rank) */
	uint8_t rdqsgd[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Read DQS Gating Delay for upper Nibble (per rank) */
	uint8_t x4rdqsgd[AL_DDR_NUM_RANKS];
	/** DQS Gating System Latency (per rank) */
	uint8_t rdqsgd_extra[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : DQS Gating System Latency for upper Nibble (per rank) */
	uint8_t x4rdqsgd_extra[AL_DDR_NUM_RANKS];

	/**
	 * Write Data Delay
	 * ALPINE V2 : per rank
	 * ALPINE V1 : only entry 0 is valid
	 */
	uint8_t wdqd[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Write Data Delay for upper Nibble (per rank) */
	uint8_t x4wdqd[AL_DDR_NUM_RANKS];
	/**
	 * Read DQS Delay
	 * ALPINE V2 : per rank
	 * ALPINE V1 : only entry 0 is valid
	 */
	uint8_t rdqsd[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Read DQS Delay for upper Nibble (per rank) */
	uint8_t x4rdqsd[AL_DDR_NUM_RANKS];
	/**
	 * Read DQSN Delay
	 * ALPINE V2 : per rank
	 * ALPINE V1 : only entry 0 is valid
	 */
	uint8_t rdqsnd[AL_DDR_NUM_RANKS];
	/** ALPINE V2 Only : Read DQSN Delay for upper Nibble (per rank) */
	uint8_t x4rdqsnd[AL_DDR_NUM_RANKS];

	/** ALPINE V2 Only : DQS Gate Status Delay */
	uint8_t dqsgsd;
	/** ALPINE V2 Only : DQS Gate Status Delay for upper Nibble */
	uint8_t x4dqsgsd;

	uint8_t dtwlmn; /** Data Training WDQ LCDL Minimum */
	uint8_t dtwlmx; /** Data Training WDQ LCDL Maximum */
	uint8_t dtwbmn; /** Data Training Write BDL Shift Minimum */
	uint8_t dtwbmx; /** Data Training Write BDL Shift Maximum */
	uint8_t dtrlmn; /** Data Training RDQS LCDL Minimum */
	uint8_t dtrlmx; /** Data Training RDQS LCDL Minimum */
	uint8_t dtrbmn; /** Data Training Read BDL Shift Minimum */
	uint8_t dtrbmx; /** Data Training Read BDL Shift Maximum */

	/** ALPINE V2 Only : Data Training WDQ LCDL Minimum for upper Nibble */
	uint8_t x4dtwlmn;
	/** ALPINE V2 Only : Data Training WDQ LCDL Maximum for upper Nibble */
	uint8_t x4dtwlmx;
	/** ALPINE V2 Only : Data Training Write BDL Shift Minimum for upper Nibble */
	uint8_t x4dtwbmn;
	/** ALPINE V2 Only : Data Training Write BDL Shift Maximum for upper Nibble */
	uint8_t x4dtwbmx;
	/** ALPINE V2 Only : Data Training RDQS LCDL Minimum for upper Nibble */
	uint8_t x4dtrlmn;
	/** ALPINE V2 Only : Data Training RDQS LCDL Minimum for upper Nibble */
	uint8_t x4dtrlmx;
	/** ALPINE V2 Only : Data Training Read BDL Shift Minimum for upper Nibble */
	uint8_t x4dtrbmn;
	/** ALPINE V2 Only : Data Training Read BDL Shift Maximum for upper Nibble */
	uint8_t x4dtrbmx;

	/** DDR4 Only : Vref Training DRAM DQ Minimum (per rank) */
	uint8_t dvrefmn[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training DRAM DQ Minimum for upper Nibble (per rank) */
	uint8_t x4dvrefmn[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training DRAM DQ Maximum (per rank) */
	uint8_t dvrefmx[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training DRAM DQ Maximum for upper Nibble (per rank) */
	uint8_t x4dvrefmx[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ Minimum (per rank) */
	uint8_t hvrefmn[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ Minimum for upper Nibble (per rank) */
	uint8_t x4hvrefmn[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ Maximum (per rank) */
	uint8_t hvrefmx[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ Maximum for upper Nibble (per rank) */
	uint8_t x4hvrefmx[AL_DDR_NUM_RANKS];

	/** DDR4 Only : Vref Training DRAM DQ (per rank) */
	uint8_t dxrefisel[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training DRAM DQ (per rank) */
	uint8_t x4dxrefisel[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ (per rank) */
	uint8_t dxdqvrefr[AL_DDR_NUM_RANKS];
	/** DDR4 Only : Vref Training HOST DQ (per rank) */
	uint8_t x4dxdqvrefr[AL_DDR_NUM_RANKS];

	/** ALPINE V2 Only : Read FIFO read delay */
	uint8_t rddly;
	/** ALPINE V2 Only : Read FIFO read delay for upper Nibble */
	uint8_t x4rddly;

	uint8_t dqwbd[8]; /** ALPINE V2 Only : DQ Write BDL */
	uint8_t dmwbd; /** ALPINE V2 Only : DM Write BDL */
	uint8_t dqrbd[8]; /** ALPINE V2 Only : DQ Read BDL */
	uint8_t dmrbd; /** ALPINE V2 Only : DM Read BDL */
};

/** DDR PHY Training Results */
struct al_ddr_phy_training_results {
	struct al_ddr_phy_training_results_per_octet
		octets[AL_DDR_PHY_NUM_BYTE_LANES];
};

/**
 * Get the DDR PHY training results
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	results
 *		DDR PHY training results
 */
void al_ddr_phy_training_results_get(
	struct al_ddr_cfg			*ddr_cfg,
	struct al_ddr_phy_training_results	*results);

/**
 * Print the DDR PHY training results
 *
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @param	ddr_cfg
 */
void al_ddr_phy_training_results_print(
	struct al_ddr_cfg *ddr_cfg);

/**
 * Initialize DDR cfg object
 *
 * @param	nb_regs_base
 *		Address of the NB register base
 * @param	ddr_ctrl_regs_base
 *		Address of the DDR controller register base
 * @param	ddr_phy_regs_base
 *		Address of the DDR PHY register base
 * @param	ddr_cfg
 *		DDR cfg object
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_cfg_init(
	void __iomem		*nb_regs_base,
	void __iomem		*ddr_ctrl_regs_base,
	void __iomem		*ddr_phy_regs_base,
	struct al_ddr_cfg	*ddr_cfg);


/**
 * Reads MPR page
 * The DDR4 SDRAMs contain four 8-bit programmable MPR which can be used for
 * DQ training, CA parity log, MRS readout, or for vendor specific purposes
 * Applicable to DDR4 only
 *
 * @param	ddr_cfg
 *		DDR cfg object
 * @param	rank_num
 *		Rank to read MPR from
 * @param	device_num
 *		Device (DRAM component) to read MPR from
 * @param	page_num
 *		MPR page to read
 * @param	page_data
 *		MPR page data
 *
 * @returns	0 if successful
 *		<0 otherwise
 */
int al_ddr_mpr_get(
	struct al_ddr_cfg	*ddr_cfg,
	unsigned int		rank_num,
	unsigned int		device_num,
	unsigned int		page_num,
	unsigned int		*page_data
);

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */
/** @} end of DDR group */
#endif

