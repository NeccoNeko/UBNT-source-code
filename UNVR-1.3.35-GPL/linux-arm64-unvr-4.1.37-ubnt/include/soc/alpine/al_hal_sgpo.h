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
 * @defgroup group_sgpo	SGPO
 * @ingroup group_pbs
 *  @{
 *
 * Common operation example:
 * @code
 * int main()
 * {
 * 	struct al_sgpo_if sgpo_if;
 * 	int ret = 0;
 * 	int ref_clk_freq = 375; // 375 MHz
 *
 * 	// initialize the sgpo interface
 * 	ret = al_sgpo_handle_init(
 * 		&sgpo_if, SGPO_REGS_BASE_ADDRESS, ref_clk_freq);
 * 	if (ret != 0) {
 * 		printf("handle Initialization failed\n");
 * 		return ret;
 * 	}
 *
 * 	ret = al_sgpo_hw_init(&sgpo_if);
 * 	if (ret != 0) {
 * 		printf("HW Initialization failed\n");
 * 		return ret;
 * 	}
 *
 * 	// config pin#0 as user output
 * 	al_sgpo_pin_mode_set(&sgpo_if, 0, AL_SGPO_USER);
 *
 * 	// get ETH0 last written value
 * 	al_sgpo_user_val_get(&sgpo_if, AL_SGPO_ETH0);
 *
 * 	// set pin#0 to 1
 * 	al_sgpo_user_val_set(&sgpo_if, 0, 1);
 *
 * 	// set stretch time to 0.5 sec for group 0 (assuming ref_clk = 375)
 * 	al_sgpo_stretch_len_set(&sgpo_if, AL_SGPO_GROUP0, 500, AL_SGPO_MS);
 *
 * 	// set strech time to 1431 ms for group 0 (assuming ref_clk = 375)
 * 	al_sgpo_stretch_len_set(&sgpo_if, AL_SGPO_GROUP0, 9, AL_SGPO_RAW);
 *
 * 	// set blink high time to 0.5 sec for group 1 (assuming ref_clk = 375)
 * 	al_sgpo_blink_high_set(&sgpo_if, AL_SGPO_GROUP1, 500, AL_SGPO_MS);
 *
 * 	// set blink low time to 0.1 sec for groups 0-1 (assuming ref_clk = 375)
 * 	// this cause blink period time of 0.6s(duty cycle of 83.33%)
 * 	al_sgpo_blink_low_set(&sgpo_if, AL_SGPO_GROUP0_1,100, AL_SGPO_MS);
 *
 * 	// set update time to 1500 kHz
 * 	al_sgpo_upd_freq_set(&sgpo_if, 1500, AL_SGPO_KHZ);
 *
 * 	// set clk frequency to 1000 kHz
 * 	al_sgpo_clk_freq_set(&sgpo_if, 1000, AL_SGPO_KHZ);
 *
 * 	// enable invert mode with SATA1
 * 	al_sgpo_invert_mode_en(&sgpo_if, AL_SGPO_SATA1, AL_TRUE);
 *
 * 	// disable blink mode with pin#7
 * 	al_sgpo_blink_mode_en(&sgpo_if, 7, AL_FALSE);
 *
 * 	return 0;
 * }
 * @endcode
 *
 * @file   al_hal_gpio.h
 * @brief Header file for the GPIO HAL driver
 *
 */

#ifndef __AL_HAL_SGPO_H__
#define __AL_HAL_SGPO_H__

#include "al_hal_common.h"

#define AL_SGPO_NUM_OF_GROUPS		4
#define AL_SGPO_NUM_OF_PINS_IN_GROUP 	8

/* Struct definitions */
struct al_sgpo_if {
	void __iomem *regs;
	unsigned int ref_clk_freq;
};

/** Enum definitions **/

/*
 * control the output mode of each pin,
 * can be either from a HW output (ETH or SATA) or from a user setting.
 */
enum al_sgpo_mode {
	AL_SGPO_HW = 0,
	AL_SGPO_USER = 3
};

/* when use HW mode, this is the pin id for each ETH, SATA, MISC port */
enum al_sgpo_id {
	/* Group 0 */
	AL_SGPO_ETH0   = 0,
	AL_SGPO_ETH1   = 1,
	AL_SGPO_ETH2   = 2,
	AL_SGPO_ETH3   = 3,
	AL_SGPO_ETH4   = 4,
	AL_SGPO_ETH5   = 5,
	AL_SGPO_ETH6   = 6,
	AL_SGPO_ETH7   = 7,
	/* Group 1 */
	AL_SGPO_SATA0  = 8,
	AL_SGPO_SATA1  = 9,
	AL_SGPO_SATA2  = 10,
	AL_SGPO_SATA3  = 11,
	AL_SGPO_SATA4  = 12,
	AL_SGPO_SATA5  = 13,
	AL_SGPO_SATA6  = 14,
	AL_SGPO_SATA7  = 15,
	/* Group 2 */
	AL_SGPO_SATA8  = 16,
	AL_SGPO_SATA9  = 17,
	AL_SGPO_SATA10 = 18,
	AL_SGPO_SATA11 = 19,
	AL_SGPO_SATA12 = 20,
	AL_SGPO_SATA13 = 21,
	AL_SGPO_SATA14 = 22,
	AL_SGPO_SATA15 = 23,
	/* Group 3 */
	AL_SGPO_MISC0  = 24,
	AL_SGPO_MISC1  = 25,
	AL_SGPO_MISC2  = 26,
	AL_SGPO_MISC3  = 27,
	AL_SGPO_MISC4  = 28,
	AL_SGPO_MISC5  = 29,
	AL_SGPO_MISC6  = 30,
	AL_SGPO_MISC7  = 31,
	AL_SGPO_PINS_NUM
};

/*
 * on active mode the sata led is turned on after link-up and stays on
 * on presence mode the sata led is blinking upon traffic in the line.
 * on active-presence mode the sata led in turned on after link-up
 * 					  and blinks upon traffic.
 */
enum al_sgpo_sata_mode {
	AL_SGPO_SATA_ACTIVE = 1,
	AL_SGPO_SATA_PRESENCE = 2,
	AL_SGPO_SATA_ACTTIVE_PRESENCE = 3
};

/*
 * group mode controls the number of serials
 * lines of which the pins's data is transmitting,
 * one group sets all the 32 pins to one serial line,
 * two groups sets two serial lines of 16 pins each,
 * four groups sets four serial lines of 8 pins each
 */
enum al_sgpo_group_mode {
	AL_SGPO_ONE_GROUP = 0,
	AL_SGPO_TWO_GROUPS = 1,
	AL_SGPO_FOUR_GROUPS = 2
};

/*
 * follow to the enum above (al_sgpo_group_mode),
 * this are the optional groups to split the 32 pins into.
 */
enum al_sgpo_group {
	AL_SGPO_GROUP0 = 1 << 0,
	AL_SGPO_GROUP1 = 1 << 1,
	AL_SGPO_GROUP2 = 1 << 2,
	AL_SGPO_GROUP3 = 1 << 3,
	AL_SGPO_GROUP0_1 = AL_SGPO_GROUP0 | AL_SGPO_GROUP1,
	AL_SGPO_GROUP2_3 = AL_SGPO_GROUP2 | AL_SGPO_GROUP3,
	AL_SGPO_GROUP0_1_2_3 = AL_SGPO_GROUP0_1 | AL_SGPO_GROUP2_3,
};

enum al_sgpo_units {
	AL_SGPO_RAW,
	AL_SGPO_NS,
	AL_SGPO_MS,
	AL_SGPO_KHZ,
};

/* Blink base rate */
enum al_sgpo_blink_base_rate {
	AL_SGPO_BLINK_BASE_RATE_NORMAL,
	AL_SGPO_BLINK_BASE_RATE_FAST,
};

/* API definitions */

/**
 * Handle initialization to the SGPO controller
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	sgpo_regs_base
 *		The base address of the SGPO configuration registers
 * @param	ref_clk_freq
 *		The reference clk frequency in MHz
 *
 * @return	0 if finished successfully
 *		<0 if an error occurred
 */
int al_sgpo_handle_init(
		struct al_sgpo_if *sgpo_if,
		void __iomem *sgpo_regs_base,
		unsigned int ref_clk_freq);

/**
 * Hardware initialization to the SGPO controller,
 * after performing the initialization one can set the
 * configuration of sgpo pins as follow:
 * 		Global setting: (default value assume ref_clk_freq = 375)
 * 		* clk rise time (default: 42ns)
 * 		* update freq	(default: 1500Hz)
 * 		* clk freq	(default: 1500kHz)
 * 		* sata mode	(default: active-presence)
 * 		* cpu vdd serial mode	(default: enable)
 * 		Group setting:
 * 		* stretch length	(default: 360ms)
 * 		* blik high and low	(defualt: 360ms)
 * 		Pin setting:
 * 		* pin mode (user or HW)	(default: HW)
 * 		* pin value (when set to user mode)
 * 		* invert enable		(default: disable)
 * 		* blink enable		(default: disable)
 * 		* stretch enable	(default: disable)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	blink_base_rate
 *		The required blink base rate
 * @return	0 if finished successfully
 *		<0 if an error occurred
 */
int al_sgpo_hw_init(
	struct al_sgpo_if		*sgpo_if,
	enum al_sgpo_blink_base_rate	blink_base_rate);

/**
 * Set data to clk rise setup time, in powers of 2 (up to N=8 --> 2^8)
 * 	of I/O fabric clock cycle,
 *	the actual time calculate as follow:
 *	setup_time = 2^(time_factor)/ref_clk_freq
 *	so for 40ns (and ref_clk_freq of 375 MHz)
 *	one should set time_factor to 4
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	time_factor
 *		The update freq_factor value to set
 *		range: 0 - 8
 *		(default value: 0x4)
 * @param	units
 *		Value units
 */
void al_sgpo_clk_rise_time_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int time_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO update frequency
 *	the actual frequency calculate as follow:
 *		f = ref_clk_freq / ( 2^(freq_factor + 10) )
 *	so for 1500 Hz (and ref_clk_freq of 375 MHz)
 *	one should set freq to 18.
 *	(use al_sgpo_freq_pre_warp function to get freq_factor value)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	freq_factor
 *		The update freq_factor value to set
 *		range: 0 - 255
 *		(default value: 0x18)
 * @param	units
 *		Value units
 */
void al_sgpo_upd_freq_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int freq_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO clk frequency
 *	the actual frequency calculate as follow:
 *		f = ref_clk_freq / ( 2^freq )
 *	so for 1500 kHz (and ref_clk_freq of 375 MHz)
 *	one should set freq to 8.
 *	(use al_sgpo_freq_pre_warp function to get freq_factor value)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	freq_factor
 *		The clk freq_factor value to set
 *		range: 0 - 255
 *		(default value: 8)
 * @param	units
 *		Value units
 */
void al_sgpo_clk_freq_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int freq_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO SATA mode,
 * active, presence or active and presence
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	mode
 * 		the SATA mode to set
 */
void al_sgpo_sata_mode_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_sata_mode mode);

/**
 * set the SGPO cpu vdd control out mode,
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	en
 * 		AL_TRUE to enable, AL_FALSE to disable
 * 		defualt: enable
 */
void al_sgpo_cpu_vdd_serial_mode_en(
		struct al_sgpo_if *sgpo_if,
		al_bool en);

/**
 * perfomrs serial cpu vdd update
 *
 * @param	sgpo_if
 *		The sgpo instance
 */
void al_sgpo_perform_serial_cpu_vdd_upd(
		struct al_sgpo_if *sgpo_if);

/**
 * Set the SGPO group mode,
 * one group with 32 pins,
 * two groups with 16 pins,
 * four groups with 8 pins.
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	mode
 * 		the group mode to set
 */
void al_sgpo_group_mode_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group_mode mode);

/**
 * Set the SGPO stretch length
 *	the actual length calculate as follow:
 *		t = 2^(20+time_factor) / ref_clk_freq
 *	so for 500 ms (and ref_clk_freq of 375 MHz)
 *	one should set time_factor to 7.
 *	(use al_sgpo_ms_pre_warp function to get time_factor value)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	groups
 * 		the group registers to set
 * @param	time_factor
 *		The stretch length value to set
 *		range: 0 - 255
 *		(default value: 0x7)
 * @param	units
 *		Value units
 */
void al_sgpo_stretch_len_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO blink high length
 *	the actual length calculate as follow:
 *		t = 2^(20+time_factor) / ref_clk_freq
 *	so for 500 ms (and ref_clk_freq of 375 MHz)
 *	one should set time_factor to 7.
 *	(use al_sgpo_ms_pre_warp function to get time_factor value)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	groups
 * 		the group registers to set
 * @param	time_factor
 *		The blink high length value to set
 *		range: 0 - 15
 *		(default value: 0x7)
 * @param	units
 *		Value units
 */
void al_sgpo_blink_high_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO blink low length
 *	the actual length calculate as follow:
 *		t = 2^(20+time_factor) / ref_clk_freq
 *	so for 500 ms (and ref_clk_freq of 375 MHz)
 *	one should set time_factor to 7.
 *	(use al_sgpo_ms_pre_warp function to get time_factor value)
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	groups
 * 		the group registers to set
 * @param	time_factor
 *		The blink low length value to set
 *		range: 0 - 15
 *		(default value: 0x7)
 * @param	units
 *		Value units
 */
void al_sgpo_blink_low_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		unsigned int time_factor,
		enum al_sgpo_units units);

/**
 * Set the SGPO pins mode for entire group
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	groups
 * 		the group registers to set
 * @param	mode
 *		The pin mode to set (hardware or user)
 */
void al_sgpo_pin_mode_group_set(
		struct al_sgpo_if *sgpo_if,
		enum al_sgpo_group groups,
		enum al_sgpo_mode mode);

/**
 * Get the last written value to the SGPO pin
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 *
 * @return	The data bit value
 */
int al_sgpo_user_val_get(
		struct al_sgpo_if *sgpo_if,
		unsigned int id);

/**
 * Set the SGPO pin output value
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 * @param	val
 *		The value to set
 */
void al_sgpo_user_val_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		unsigned int val);

/**
 * Set the SGPO pins output value,
 * use this function to set multiple pins on same group simultaneously.
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	mask
 *		the array of bits to write (e.g. 0x20004101 - bits 0,8,14,29)
 * @param	vals
 *		The values of the bits (e.g. 0x20004001 - set 0,14,29. clear 8)
 */
void al_sgpo_user_val_array_set(
		struct al_sgpo_if *sgpo_if,
		uint32_t mask,
		uint32_t vals);

/**
 * Get the last SGPO value output of all pins,
 *
 * @param	sgpo_if
 *		The sgpo instance
 *
 * @return	a uint32_t which hold the 32 pins value as bits array.
 */
uint32_t al_sgpo_user_val_array_get(
		struct al_sgpo_if *sgpo_if);

/**
 * Set the SGPO pin mode
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 * @param	mode
 *		The pin mode to set (hardware or user)
 */
void al_sgpo_pin_mode_set(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		enum al_sgpo_mode mode);


/**
 * Set the SGPO pin invert mode
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 * @param	en
 *		AL_TRUE to enable, AL_FALSE to disable
 */
void al_sgpo_invert_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en);

/**
 * Set the SGPO pin blink mode
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 * @param	en
 *		AL_TRUE to enable, AL_FALSE to disable
 */
void al_sgpo_blink_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en);

/**
 * Set the SGPO pin stretch mode
 *
 * @param	sgpo_if
 *		The sgpo instance
 * @param	id
 *		The id of the SGPO pin
 * @param	en
 *		AL_TRUE to enable, AL_FALSE to disable
 */
void al_sgpo_stretch_mode_en(
		struct al_sgpo_if *sgpo_if,
		unsigned int id,
		al_bool en);

#endif

/** @} end of SGPO group */
