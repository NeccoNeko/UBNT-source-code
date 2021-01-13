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
 * @defgroup group_sgpo SGPO
 *  SGPO
 *  @{
 * @file   al_hal_sgpo.c
 *
 * @brief  SGPO HAL driver
 *
 */

#include "al_hal_sgpo.h"
#include "al_hal_sgpo_regs.h"


#define GROUP_MASK			0x000000FF
#define ID_TO_GROUP_NUM(id)		((id) / AL_SGPO_NUM_OF_PINS_IN_GROUP)
#define ID_TO_GROUP_OFFSET(id)		((id) % AL_SGPO_NUM_OF_PINS_IN_GROUP)
#define DATA_ARRAY_ID_TO_IDX(id)	(1 << (id))

#define GROUP_ACTIVE(groups, group_idx)	((groups) & (1 << (group_idx)))
#define PIN_MODE_MASK(id)		(0x03 << (2*(id)))
#define PIN_MODE_SHIFT(id)		(2*(id))

#define MAX_SDI_SETUP			8

#define ROUND_LOG2_ERR_THRESHOLD 3 /* for 33.33% accuracy, THRESHOLD = 100/33.33 */

/*
 *  calculate the log2 of a number,
 *  return integer so the result is round to the closest integer
 */
static int round_log2(uint32_t n, unsigned int *res)
{
	int i;
	uint32_t m = n;
	int error;

	*res = 0;

	if(n == 0)
		return 0;

	for (i = 4; i >= 0; --i) {
		uint32_t factor = 1 << i; /* factor = 16,8,4,2,1 */
		if (m >= (1ul << factor)) {
			*res += factor;
			m >>= factor;
		}
	}

	/* check for round up */
	if (n & (1 << (*res-1)))
		++(*res);

	error = (1 << *res) - n;
	if (error < 0)
		error *= -1;
	if ((unsigned int)(error) * ROUND_LOG2_ERR_THRESHOLD > n)
		return -EINVAL;

	return 0;
}

/**
 * Convert time in units of nSec into the correct factor (time_factor)
 * used in the function al_sgpo_clk_rise_time_set,
 * depend on ref_clk_freq which supply in the al_sgpo_init function,
 * this is only an approximation to the closest time possible
 */
static inline unsigned int clk_rise_pre_warp(
		unsigned int ref_clk_freq,
		unsigned int time)
{
	unsigned int time_factor;

	 /* clk_freq in MHz, time in ns */
	uint32_t num_of_clks = ref_clk_freq * time / 1000;
	/* time_factor = log2(num_of_clk) */
	if (round_log2(num_of_clks, &time_factor) == -EINVAL)
		al_warn("%s: inaccuracy, desired time - %u, calculation time - %u\n",
			__func__, time,
			1000*(1 << time_factor)/ref_clk_freq);

	return time_factor;
}

/**
 * Convert time in units of milliseconds into the correct factor (time_factor)
 * used in the functions: al_sgpo_stretch_len_set, al_sgpo_blink_low_set,
 * 			  al_sgpo_blink_high_set
 * depend on ref_clk_freq which supply in the al_sgpo_init function,
 * this is only an approximation to the closest time possible
 */
static inline unsigned int ms_pre_warp(
	struct al_sgpo_regs __iomem	*regs,
	unsigned int			ref_clk_freq,
	unsigned int			time)
{
	unsigned int time_factor;
	unsigned int scale =
		2 * ((al_reg_read32(&regs->common.conf.control) &
		SGPO_COMMON_CONF_CONTROL_CNTR_SCALE_MASK) >>
		SGPO_COMMON_CONF_CONTROL_CNTR_SCALE_SHIFT);

	 /* clk_freq in MHz, time in ms */
	uint32_t num_of_clks = ref_clk_freq * time * 1000;
	/* time_factor + scale = log2(num_of_clk) */
	if (round_log2(num_of_clks, &time_factor) == -EINVAL)
		al_warn("%s: inaccuracy, desired time - %u, calculation time - %u\n",
			__func__, time,
			(1 << time_factor)/(ref_clk_freq * 1000));

	al_assert(time_factor >= scale);

	return time_factor - scale;
}

/**
 * Convert frequency in units of kHz into the correct factor (freq)
 * used in the functions: al_sgpo_clk_freq_set & al_sgpo_upd_freq_set,
 * depend on ref_clk_freq which supply in the al_sgpo_init function,
 * this is only an approximation to the closest frequency possible
 */
static inline unsigned int freq_pre_warp(
		unsigned int ref_clk_freq,
		unsigned int freq)
{
	unsigned int freq_factor;

	/* clk_freq in MHz, freq in kHz */
	uint32_t freq_ratio = (ref_clk_freq * 1000) / freq;
	/* freq_factor = log2(freq_ratio) */
	if (round_log2(freq_ratio, &freq_factor) == -EINVAL)
		al_warn("%s: inaccuracy, desired freq - %u, calculation freq - %u\n",
			__func__, freq,
			1000 * ref_clk_freq / (1 << freq_factor));

	return freq_factor;
}


int al_sgpo_hw_init(
	struct al_sgpo_if		*sgpo_if,
	enum al_sgpo_blink_base_rate	blink_base_rate)
{
	struct al_sgpo_regs __iomem *regs;
	uint32_t reg_val;
	uint32_t reg_mask;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	/** enable serialization **/
	/* set the blink base rate */
	al_reg_write32_masked(
		&regs->common.conf.control,
		SGPO_COMMON_CONF_CONTROL_CNTR_SCALE_MASK,
		(blink_base_rate == AL_SGPO_BLINK_BASE_RATE_FAST) ?
		SGPO_COMMON_CONF_CONTROL_CNTR_SCALE_VAL_FAST_BLINK :
		SGPO_COMMON_CONF_CONTROL_CNTR_SCALE_VAL_NORMAL_BLINK);

	/* set all out_en_sgpo bits */
	reg_val = SGPO_COMMON_CONF_OUT_EN_SGPO_SHCP |
		  SGPO_COMMON_CONF_OUT_EN_SGPO_STCP |
		  SGPO_COMMON_CONF_OUT_EN_SGPO_DS_3_0_MASK;
	reg_mask = SGPO_COMMON_CONF_OUT_EN_SGPO_SHCP |
		   SGPO_COMMON_CONF_OUT_EN_SGPO_STCP |
		   SGPO_COMMON_CONF_OUT_EN_SGPO_DS_3_0_MASK;
	al_reg_write32_masked(
			&regs->common.conf.out_en_sgpo, reg_mask, reg_val);

	/* set all out_en_cpu_vdd_ctrl bits */
	reg_val = SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_SHCP |
		  SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_STCP |
		  SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_DS;
	reg_mask = SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_SHCP |
		   SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_STCP |
		   SGPO_COMMON_CONF_OUT_EN_CPU_VDD_CTRL_DS;
	al_reg_write32_masked(
			&regs->common.conf.out_en_cpu_vdd_ctrl,
			reg_mask,
			reg_val);

	/* enable the serial cpu_vdd_ctrl block */
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_CPU_VDD_EN,
			SGPO_COMMON_CONF_CONTROL_CPU_VDD_EN);

	/* enable the sgi_block */
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_SGI_ENABLE,
			SGPO_COMMON_CONF_CONTROL_SGI_ENABLE);

	return 0;
}

int al_sgpo_handle_init(
		struct al_sgpo_if *sgpo_if,
		void __iomem *sgpo_regs_base,
		unsigned int ref_clk_freq)
{
	al_assert(sgpo_if);
	if (ref_clk_freq == 0)
		return -EIO;

	sgpo_if->regs = sgpo_regs_base;
	sgpo_if->ref_clk_freq = ref_clk_freq;

	return 0;
}

void al_sgpo_clk_rise_time_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int time_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_NS));

	if (units == AL_SGPO_NS)
		time_factor =
				clk_rise_pre_warp(sgpo_if->ref_clk_freq, time_factor);

	al_assert(time_factor < MAX_SDI_SETUP);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_SDI_SETUP_MASK,
			(uint32_t)time_factor <<
			SGPO_COMMON_CONF_CONTROL_SDI_SETUP_SHIFT);
}

void al_sgpo_upd_freq_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int freq_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_KHZ));

	if (units == AL_SGPO_KHZ)
		freq_factor =
				freq_pre_warp(sgpo_if->ref_clk_freq, freq_factor);

	al_assert(freq_factor < 0x20);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_UPDATE_FREQ_MASK,
			(uint32_t)freq_factor <<
			SGPO_COMMON_CONF_CONTROL_UPDATE_FREQ_SHIFT);
}

void al_sgpo_clk_freq_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int freq_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_KHZ));

	if (units == AL_SGPO_KHZ)
		freq_factor =
				freq_pre_warp(sgpo_if->ref_clk_freq, freq_factor);

	al_assert(freq_factor <= 0x10);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_CLK_FREQ_MASK,
			(uint32_t)freq_factor <<
			SGPO_COMMON_CONF_CONTROL_CLK_FREQ_SHIFT);
}

void al_sgpo_sata_mode_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_sata_mode mode)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;
	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_SATA_LED_SEL_MASK,
			(uint32_t)mode <<
			SGPO_COMMON_CONF_CONTROL_SATA_LED_SEL_SHIFT);
}

void al_sgpo_cpu_vdd_serial_mode_en(
		struct al_sgpo_if *sgpo_if,
		al_bool en)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_CPU_VDD_SEL,
			(uint32_t)((en == AL_TRUE) ?
					SGPO_COMMON_CONF_CONTROL_CPU_VDD_SEL :
					0));
}

void al_sgpo_group_mode_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group_mode mode)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	al_reg_write32_masked(
			&regs->common.conf.control,
			SGPO_COMMON_CONF_CONTROL_GROUPS_MASK,
			(uint32_t)mode <<
			SGPO_COMMON_CONF_CONTROL_GROUPS_SHIFT);
}

void al_sgpo_perform_serial_cpu_vdd_upd(
		struct al_sgpo_if *sgpo_if)
{
	struct al_sgpo_regs __iomem *regs;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	al_reg_write32(&regs->common.conf.cpu_vdd,
			SGPO_COMMON_CONF_CPU_VDD_UPDATE);
}

void al_sgpo_stretch_len_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;
	int group;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_MS));

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	if (units == AL_SGPO_MS)
		time_factor = ms_pre_warp(regs, sgpo_if->ref_clk_freq, time_factor);

	for (group = 0; group < AL_SGPO_NUM_OF_GROUPS; ++group)	{
		if (GROUP_ACTIVE(groups,group))
			al_reg_write32_masked(
				&regs->group[group].conf.rates,
				SGPO_PERGROUP_CONF_RATES_STRETCH_MASK,
				(uint32_t)time_factor <<
				SGPO_PERGROUP_CONF_RATES_STRETCH_SHIFT);
	}
}

void al_sgpo_blink_high_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;
	int group;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_MS));

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	if (units == AL_SGPO_MS)
		time_factor = ms_pre_warp(regs, sgpo_if->ref_clk_freq, time_factor);

	for (group = 0; group < AL_SGPO_NUM_OF_GROUPS; ++group)	{
		if (GROUP_ACTIVE(groups,group))
			al_reg_write32_masked(
				&regs->group[group].conf.rates,
				SGPO_PERGROUP_CONF_RATES_BLINK_HIGH_MASK,
				(uint32_t)time_factor <<
				SGPO_PERGROUP_CONF_RATES_BLINK_HIGH_SHIFT);
	}
}

void al_sgpo_blink_low_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units)
{
	struct al_sgpo_regs __iomem *regs;
	int group;

	al_assert(sgpo_if);
	al_assert((units == AL_SGPO_RAW) || (units == AL_SGPO_MS));

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	if (units == AL_SGPO_MS)
		time_factor = ms_pre_warp(regs, sgpo_if->ref_clk_freq, time_factor);

	for (group = 0; group < AL_SGPO_NUM_OF_GROUPS; ++group)	{
		if (GROUP_ACTIVE(groups,group))
			al_reg_write32_masked(
				&regs->group[group].conf.rates,
				SGPO_PERGROUP_CONF_RATES_BLINK_LOW_MASK,
				(uint32_t)time_factor <<
				SGPO_PERGROUP_CONF_RATES_BLINK_LOW_SHIFT);
	}
}

void al_sgpo_pin_mode_group_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		enum al_sgpo_mode mode)
{
	struct al_sgpo_regs __iomem *regs;
	int group;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	for (group = 0; group < AL_SGPO_NUM_OF_GROUPS; ++group)	{
		if (GROUP_ACTIVE(groups,group))
			/* writing 0xFFFF set all pins to user mode */
			al_reg_write32_masked(
					&regs->group[group].conf.mode,
					0xFFFF,
					(mode == AL_SGPO_HW) ? 0 : 0xFFFF);
	}
}

int al_sgpo_user_val_get(
		struct al_sgpo_if *sgpo_if,
		unsigned int id)
{
	struct al_sgpo_regs __iomem *regs;
	unsigned int reg_val;
	unsigned int idx;
	unsigned int group;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	group = ID_TO_GROUP_NUM(id);
	id = ID_TO_GROUP_OFFSET(id);
	idx = DATA_ARRAY_ID_TO_IDX(id);

	reg_val = al_reg_read32(&regs->group[group].conf_vec[idx].val);
	return reg_val >> id;
}


void al_sgpo_user_val_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		unsigned int val)
{
	struct al_sgpo_regs __iomem *regs;
	unsigned int idx;
	unsigned int group;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);
	al_assert(val <= 1); /* val should be '0' or '1' */

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	group = ID_TO_GROUP_NUM(id);
	id = ID_TO_GROUP_OFFSET(id);
	idx = DATA_ARRAY_ID_TO_IDX(id);

	al_reg_write32(&regs->group[group].conf_vec[idx].val, val << id);
}

void al_sgpo_user_val_array_set(
		struct al_sgpo_if *sgpo_if,
		uint32_t mask,
		uint32_t vals)
{
	struct al_sgpo_regs __iomem *regs;
	int group;
	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	for (group = 0; group < AL_SGPO_NUM_OF_GROUPS; ++group) {
		int group_mask = mask & GROUP_MASK;
		if (group_mask > 0)
			al_reg_write32(
				&regs->group[group].conf_vec[group_mask].val,
				vals & GROUP_MASK);
		vals >>= AL_SGPO_NUM_OF_PINS_IN_GROUP;
		mask >>= AL_SGPO_NUM_OF_PINS_IN_GROUP;
	}
}

uint32_t al_sgpo_user_val_array_get(
		struct al_sgpo_if *sgpo_if)
{
	struct al_sgpo_regs __iomem *regs;
	uint32_t sgpo_bits_array = 0; /* hold the values of the 32 sgpo pins */
	int group;

	al_assert(sgpo_if);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	for (group = AL_SGPO_NUM_OF_GROUPS - 1; group >= 0  ; --group) {
		sgpo_bits_array <<= AL_SGPO_NUM_OF_PINS_IN_GROUP;
		sgpo_bits_array += al_reg_read32(
				&regs->group[group].conf_vec[GROUP_MASK].val);
	}

	return sgpo_bits_array;
}

void al_sgpo_pin_mode_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		enum al_sgpo_mode mode)
{
	struct al_sgpo_regs __iomem *regs;
	int idx;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	idx = ID_TO_GROUP_OFFSET(id);

	al_reg_write32_masked(
			&regs->group[ID_TO_GROUP_NUM(id)].conf.mode,
			PIN_MODE_MASK(idx),
			(uint32_t)(mode) << PIN_MODE_SHIFT(idx));
}

void al_sgpo_invert_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en)
{
	struct al_sgpo_regs __iomem *regs;
	int idx;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	idx = ID_TO_GROUP_OFFSET(id);

	al_reg_write32_masked(
			&regs->group[ID_TO_GROUP_NUM(id)].conf.invert,
			1 << idx,
			(en == AL_TRUE) ? 1 << idx : 0);
}

void al_sgpo_blink_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en)
{
	struct al_sgpo_regs __iomem *regs;
	int idx;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	idx = ID_TO_GROUP_OFFSET(id);

	al_reg_write32_masked(
			&regs->group[ID_TO_GROUP_NUM(id)].conf.blink,
			1 << idx,
			(en == AL_TRUE) ? 1 << idx : 0);
}

void al_sgpo_stretch_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en)
{
	struct al_sgpo_regs __iomem *regs;
	int idx;

	al_assert(sgpo_if);
	al_assert(id < AL_SGPO_PINS_NUM);

	regs = (struct al_sgpo_regs __iomem *)sgpo_if->regs;

	idx = ID_TO_GROUP_OFFSET(id);

	al_reg_write32_masked(
			&regs->group[ID_TO_GROUP_NUM(id)].conf.stretch,
			1 << idx,
			(en == AL_TRUE) ? 1 << idx : 0);
}


/** @} end of GPIO group */
