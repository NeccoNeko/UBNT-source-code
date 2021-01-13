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
 *  @{
 * @file   al_init_eth_lm.c
 *
 * @brief ethernet link management common utilities
 *
 */
#include <linux/sfp.h>
#include "al_init_eth_lm.h"
#include "al_serdes.h"
#include "al_hal_eth.h"
#include "al_init_eth_kr.h"

/* delay before checking link status with new serdes parameters (uSec) */
#define AL_ETH_LM_LINK_STATUS_DELAY	1000
/* delay before checking link status after reconfiguring the retimer (mSec) */
#define AL_ETH_LM_RETIMER_LINK_STATUS_DELAY 50

#define AL_ETH_LM_RETIMER_CONFIG_DELAY	1000

#define AL_ETH_LM_EQ_ITERATIONS		15
#define AL_ETH_LM_MAX_DCGAIN		8

/* num of link training failures till serdes reset */
#define AL_ETH_LT_FAILURES_TO_RESET	(10)

#define MODULE_IDENTIFIER_IDX		0
#define MODULE_IDENTIFIER_SFP		0x3
#define MODULE_IDENTIFIER_QSFP		0xd

#define SFP_PRESENT			0
#define SFP_NOT_PRESENT			1

/* SFP+ module */
#define SFP_I2C_HEADER_10G_IDX		3
#define SFP_I2C_HEADER_10G_DA_IDX	8
#define SFP_I2C_HEADER_10G_DA_LEN_IDX	18
#define SFP_I2C_HEADER_1G_IDX		6
#define SFP_I2C_HEADER_SIGNAL_RATE	12 /* Nominal signaling rate, units of 100MBd. */

#define SFP_MIN_SIGNAL_RATE_25G		250
#define SFP_MIN_SIGNAL_RATE_10G		100

/* QSFP+ module */
#define QSFP_COMPLIANCE_CODE_IDX	131
/* 40GBASE-LR4 and 40GBASE-SR4 are optic modules */
#define QSFP_COMPLIANCE_CODE_OPTIC	(AL_BIT(1) | AL_BIT(2))
#define QSFP_COMPLIANCE_CODE_DAC	(AL_BIT(3))
#define QSFP_CABLE_LEN_IDX		146

/* TODO: need to check the necessary delay */
#define AL_ETH_LM_RETIMER_WAIT_FOR_LOCK	500 /* delay after retimer reset to lock (mSec) */
#define AL_ETH_LM_SERDES_WAIT_FOR_LOCK	50 /* delay after signal detect to lock (mSec) */

#define AL_ETH_LM_GEARBOX_RESET_DELAY	1000 /* (uSec) */

/* Auto-FEC Macros - units are 1msec */
#define AUTO_FEC_INITIAL_TIMEOUT_DEFAULT	1000
#define AUTO_FEC_TOGGLE_TIMEOUT_DEFAULT		500

/* LM step */
#define LM_STEP_LOOP_INTERVAL  10 /* usec */
#define LM_STEP_RETIMER_WAIT_FOR_LOCK_CHECK_INTERVAL 10 /* msec */

#define SFP_PRESENT_GPIO_DEBOUNCE_ITERS	9

enum al_eth_lm_step_detection_state {
	LM_STEP_DETECTION_INIT,
	LM_STEP_DETECTION_RETIMER_RX_ADAPT_WAIT,
	LM_STEP_DETECTION_RETIMER_RX_ADAPT_POST,
	LM_STEP_DETECTION_FINAL,
};

enum al_eth_lm_step_establish_state {
	LM_STEP_ESTABLISH_INIT,
	LM_STEP_ESTABLISH_RETIMER_SIGNAL_LOCK_CHECK,
	LM_STEP_ESTABLISH_RETIMER_RX_ADAPT_WAIT,
	LM_STEP_ESTABLISH_MIDDLE,
	LM_STEP_ESTABLISH_RETIMER_EXISTS_LINK_STATUS_DELAY,
	LM_STEP_ESTABLISH_FINAL,
};
/* al_eth_lm_retimer_25g_rx_adaptation_step STATES */
enum al_eth_lm_step_retimer_rx_adaptation_state {
	LM_STEP_RETIMER_RX_ADAPTATION_RETIMER_LOCK_TO,
	LM_STEP_RETIMER_RX_ADAPTATION_SERDES_LOCK_DELAY,
};

/* al_eth_lm_retimer_signal_lock_check_step STATES */
enum al_eth_lm_step_retimer_slc_state {
	LM_STEP_RETIMER_SIGNAL_LOCK_INIT,
	LM_STEP_RETIMER_SIGNAL_LOCK_IN_PROGRESS,
};

/* al_eth_lm_wait_for_lock_step STATES */
enum al_eth_lm_step_retimer_wfl_state {
	LM_STEP_RETIMER_WAIT_FOR_LOCK_INIT,
	LM_STEP_RETIMER_WAIT_FOR_LOCK_IN_PROGRESS,
};

static const uint32_t
	al_eth_retimer_boost_addr[AL_ETH_RETIMER_CHANNEL_MAX][AL_ETH_RETIMER_TYPE_MAX] = {
					/* BR_210  |  BR_410 */
	/* AL_ETH_RETIMER_CHANNEL_A */	{0xf,		0x1a	},
	/* AL_ETH_RETIMER_CHANNEL_B */	{0x16,		0x18	},
	/* AL_ETH_RETIMER_CHANNEL_C */	{0x0,		0x16	},
	/* AL_ETH_RETIMER_CHANNEL_D */	{0x0,		0x14	},
};

#define RETIMER_LENS_MAX		5
static const uint32_t al_eth_retimer_boost_lens[RETIMER_LENS_MAX] = {0, 1, 2, 3, 5};

struct al_eth_retimer_params {
	uint32_t eq_boost;
	uint32_t de_emphasis;
	uint32_t vod;
};
/* The br410 retimer params for 1m and 3m are calibrated for EVP board */
static const struct al_eth_retimer_params
al_eth_retimer_params[RETIMER_LENS_MAX + 1][AL_ETH_RETIMER_TYPE_MAX] = {
		/* BR_210
		 * BR_410 */
	/* 0m */{{.eq_boost = 0x0,},
		 {.eq_boost = 0x0,} },
	/* 1m */{{.eq_boost = 0x1,},
		 {.eq_boost = 0x1, .de_emphasis = 0, .vod = 0x1} }, /*7.3 dB, 0dB, 0.8V*/
	/* 2m */{{.eq_boost = 0x2,},
		 {.eq_boost = 0x1,} },
	/* 3m */{{.eq_boost = 0x3,},
		 {.eq_boost = 0x3, .de_emphasis = 0, .vod = 0x1} }, /*12.2 dB, 0dB, 0.8V*/
	/* 5m */{{.eq_boost = 0x7,},
		 {.eq_boost = 0x3,} },
	/*+5m */{{.eq_boost = 0xb,},
		 {.eq_boost = 0x7,} },
};

struct retimer_config_reg {
	uint8_t addr;
	uint8_t value;
	uint8_t mask;
};

static struct retimer_config_reg retimer_ds25_25g_mode_tx_ch[] = {
	{.addr = 0x0A, .value = 0x0C, .mask = 0xff },
	{.addr = 0x2F, .value = 0x54, .mask = 0xff },
	{.addr = 0x31, .value = 0x20, .mask = 0xff },
	{.addr = 0x1E, .value = 0xE9, .mask = 0xff },
	{.addr = 0x1F, .value = 0x0B, .mask = 0xff },
	{.addr = 0xA6, .value = 0x43, .mask = 0xff },
	{.addr = 0x2A, .value = 0x5A, .mask = 0xff },
	{.addr = 0x2B, .value = 0x0A, .mask = 0xff },
	{.addr = 0x2C, .value = 0xF6, .mask = 0xff },
	{.addr = 0x70, .value = 0x05, .mask = 0xff },
	{.addr = 0x6A, .value = 0x21, .mask = 0xff },
	{.addr = 0x35, .value = 0x0F, .mask = 0xff },
	{.addr = 0x12, .value = 0x83, .mask = 0xff },
	{.addr = 0x9C, .value = 0x24, .mask = 0xff },
	{.addr = 0x98, .value = 0x00, .mask = 0xff },
	{.addr = 0x42, .value = 0x50, .mask = 0xff },
	{.addr = 0x44, .value = 0x90, .mask = 0xff },
	{.addr = 0x45, .value = 0xC0, .mask = 0xff },
	{.addr = 0x46, .value = 0xD0, .mask = 0xff },
	{.addr = 0x47, .value = 0xD1, .mask = 0xff },
	{.addr = 0x48, .value = 0xD5, .mask = 0xff },
	{.addr = 0x49, .value = 0xD8, .mask = 0xff },
	{.addr = 0x4A, .value = 0xEA, .mask = 0xff },
	{.addr = 0x4B, .value = 0xF7, .mask = 0xff },
	{.addr = 0x4C, .value = 0xFD, .mask = 0xff },
	{.addr = 0x8E, .value = 0x00, .mask = 0xff },
	{.addr = 0x3D, .value = 0x94, .mask = 0xff },
	{.addr = 0x3F, .value = 0x40, .mask = 0xff },
	{.addr = 0x3E, .value = 0x43, .mask = 0xff },
	{.addr = 0x0A, .value = 0x00, .mask = 0xff },
};

static struct retimer_config_reg retimer_ds25_25g_mode_rx_ch[] = {
	{.addr = 0x0A, .value = 0x0C, .mask = 0xff},
	{.addr = 0x2F, .value = 0x54, .mask = 0xff},
	{.addr = 0x31, .value = 0x40, .mask = 0xff},
	{.addr = 0x1E, .value = 0xE3, .mask = 0xff},
	{.addr = 0x1F, .value = 0x0B, .mask = 0xff},
	{.addr = 0xA6, .value = 0x43, .mask = 0xff},
	{.addr = 0x2A, .value = 0x5A, .mask = 0xff},
	{.addr = 0x2B, .value = 0x0A, .mask = 0xff},
	{.addr = 0x2C, .value = 0xF6, .mask = 0xff},
	{.addr = 0x70, .value = 0x05, .mask = 0xff},
	{.addr = 0x6A, .value = 0x21, .mask = 0xff},
	{.addr = 0x35, .value = 0x0F, .mask = 0xff},
	{.addr = 0x12, .value = 0x83, .mask = 0xff},
	{.addr = 0x9C, .value = 0x24, .mask = 0xff},
	{.addr = 0x98, .value = 0x00, .mask = 0xff},
	{.addr = 0x42, .value = 0x50, .mask = 0xff},
	{.addr = 0x44, .value = 0x90, .mask = 0xff},
	{.addr = 0x45, .value = 0xC0, .mask = 0xff},
	{.addr = 0x46, .value = 0xD0, .mask = 0xff},
	{.addr = 0x47, .value = 0xD1, .mask = 0xff},
	{.addr = 0x48, .value = 0xD5, .mask = 0xff},
	{.addr = 0x49, .value = 0xD8, .mask = 0xff},
	{.addr = 0x4A, .value = 0xEA, .mask = 0xff},
	{.addr = 0x4B, .value = 0xF7, .mask = 0xff},
	{.addr = 0x4C, .value = 0xFD, .mask = 0xff},
	{.addr = 0x8E, .value = 0x00, .mask = 0xff},
	{.addr = 0x3D, .value = 0x94, .mask = 0xff},
	{.addr = 0x3F, .value = 0x40, .mask = 0xff},
	{.addr = 0x3E, .value = 0x43, .mask = 0xff},
	{.addr = 0x0A, .value = 0x00, .mask = 0xff},
};

static struct retimer_config_reg retimer_ds25_auto_mode_tx_ch[] = {
	{.addr = 0x0A, .value = 0x0C, .mask = 0xff },
	{.addr = 0x2F, .value = 0x64, .mask = 0xff },
	{.addr = 0x31, .value = 0x20, .mask = 0xff },
	{.addr = 0x1E, .value = 0xE9, .mask = 0xff },
	{.addr = 0x1F, .value = 0x0B, .mask = 0xff },
	{.addr = 0xA6, .value = 0x43, .mask = 0xff },
	{.addr = 0x2A, .value = 0x5A, .mask = 0xff },
	{.addr = 0x2B, .value = 0x0A, .mask = 0xff },
	{.addr = 0x2C, .value = 0xF6, .mask = 0xff },
	{.addr = 0x70, .value = 0x05, .mask = 0xff },
	{.addr = 0x6A, .value = 0x23, .mask = 0xff },
	{.addr = 0x35, .value = 0x0F, .mask = 0xff },
	{.addr = 0x12, .value = 0x83, .mask = 0xff },
	{.addr = 0x9C, .value = 0x24, .mask = 0xff },
	{.addr = 0x98, .value = 0x00, .mask = 0xff },
	{.addr = 0x42, .value = 0x50, .mask = 0xff },
	{.addr = 0x44, .value = 0x90, .mask = 0xff },
	{.addr = 0x45, .value = 0xC0, .mask = 0xff },
	{.addr = 0x46, .value = 0xD0, .mask = 0xff },
	{.addr = 0x47, .value = 0xD1, .mask = 0xff },
	{.addr = 0x48, .value = 0xD5, .mask = 0xff },
	{.addr = 0x49, .value = 0xD8, .mask = 0xff },
	{.addr = 0x4A, .value = 0xEA, .mask = 0xff },
	{.addr = 0x4B, .value = 0xF7, .mask = 0xff },
	{.addr = 0x4C, .value = 0xFD, .mask = 0xff },
	{.addr = 0x8E, .value = 0x00, .mask = 0xff },
	{.addr = 0x3D, .value = 0x94, .mask = 0xff },
	{.addr = 0x3F, .value = 0x40, .mask = 0xff },
	{.addr = 0x3E, .value = 0x43, .mask = 0xff },
	{.addr = 0x0A, .value = 0x00, .mask = 0xff },
};

static struct retimer_config_reg retimer_ds25_auto_mode_rx_ch[] = {
	{.addr = 0x0A, .value = 0x0C, .mask = 0xff},
	{.addr = 0x2F, .value = 0x64, .mask = 0xff},
	{.addr = 0x31, .value = 0x40, .mask = 0xff},
	{.addr = 0x1E, .value = 0xE3, .mask = 0xff},
	{.addr = 0x1F, .value = 0x0B, .mask = 0xff},
	{.addr = 0xA6, .value = 0x43, .mask = 0xff},
	{.addr = 0x2A, .value = 0x5A, .mask = 0xff},
	{.addr = 0x2B, .value = 0x0A, .mask = 0xff},
	{.addr = 0x2C, .value = 0xF6, .mask = 0xff},
	{.addr = 0x70, .value = 0x05, .mask = 0xff},
	{.addr = 0x6A, .value = 0x23, .mask = 0xff},
	{.addr = 0x35, .value = 0x0F, .mask = 0xff},
	{.addr = 0x12, .value = 0x83, .mask = 0xff},
	{.addr = 0x9C, .value = 0x24, .mask = 0xff},
	{.addr = 0x98, .value = 0x00, .mask = 0xff},
	{.addr = 0x42, .value = 0x50, .mask = 0xff},
	{.addr = 0x44, .value = 0x90, .mask = 0xff},
	{.addr = 0x45, .value = 0xC0, .mask = 0xff},
	{.addr = 0x46, .value = 0xD0, .mask = 0xff},
	{.addr = 0x47, .value = 0xD1, .mask = 0xff},
	{.addr = 0x48, .value = 0xD5, .mask = 0xff},
	{.addr = 0x49, .value = 0xD8, .mask = 0xff},
	{.addr = 0x4A, .value = 0xEA, .mask = 0xff},
	{.addr = 0x4B, .value = 0xF7, .mask = 0xff},
	{.addr = 0x4C, .value = 0xFD, .mask = 0xff},
	{.addr = 0x8E, .value = 0x00, .mask = 0xff},
	{.addr = 0x3D, .value = 0x94, .mask = 0xff},
	{.addr = 0x3F, .value = 0x40, .mask = 0xff},
	{.addr = 0x3E, .value = 0x43, .mask = 0xff},
	{.addr = 0x0A, .value = 0x00, .mask = 0xff},
};

static struct retimer_config_reg retimer_ds25_10g_mode[] = {
	/* Assert CDR reset (6.3) */
	{.addr = 0x0A, .value = 0x0C, .mask = 0x0C},
	/* Select 10.3125Gbps standard rate mode (6.6) */
	{.addr = 0x2F, .value = 0x00, .mask = 0xF0},
	/* Enable loop filter auto-adjust */
	{.addr = 0x1F, .value = 0x08, .mask = 0x08},
	/* Set Adapt Mode 1 (6.13) */
	{.addr = 0x31, .value = 0x20, .mask = 0x60},
	/* Disable the DFE since most applications do not need it (6.18) */
	{.addr = 0x1E, .value = 0x08, .mask = 0x08},
	/* Release CDR reset (6.4) */
	{.addr = 0x0A, .value = 0x00, .mask = 0x0C},
	/* Enable FIR (6.12) */
	{.addr = 0x3D, .value = 0x80, .mask = 0x80},
	/* Set Main-cursor tap sign to positive (6.12) */
	{.addr = 0x3D, .value = 0x00, .mask = 0x40},
	/* Set Post-cursor tap sign to negative (6.12) */
	{.addr = 0x3F, .value = 0x40, .mask = 0x40},
	/* Set Pre-cursor tap sign to negative (6.12) */
	{.addr = 0x3E, .value = 0x40, .mask = 0x40},
	/* Set Main-cursor tap magnitude to 13 (6.12) */
	{.addr = 0x3D, .value = 0x0D, .mask = 0x1F},
};

static struct retimer_config_reg retimer_ds125_10g_mode[] = {
       /* Enable Broadcast. All writes target all channel register sets */
       {.addr = 0xFF, .value = 0x0C, .mask = 0xFF},
       /* Reset Channel Registers */
       {.addr = 0x00, .value = 0x04, .mask = 0xFF},
       /* Enable override divider select and Enable Override Output Mux */
       {.addr = 0x09, .value = 0x24, .mask = 0xFF},
       /* Channel B idle control */
       {.addr = 0x15, .value = 0x12, .mask = 0xFF},
       /* Select VCO Divider to full rate (000) */
       {.addr = 0x18, .value = 0x00, .mask = 0xFF},
       /* Selects active PFD MUX Input as Re-timed Data (001) */
       {.addr = 0x1E, .value = 0x21, .mask = 0xFF},
       /* VOD control for channel B */
       {.addr = 0x2D, .value = 0x81, .mask = 0xFF},
       /* Set data rate as 10.3125 Gbps */
       {.addr = 0x60, .value = 0x00, .mask = 0xFF},
       {.addr = 0x61, .value = 0xB2, .mask = 0xFF},
       {.addr = 0x62, .value = 0x90, .mask = 0xFF},
       {.addr = 0x63, .value = 0xB3, .mask = 0xFF},
       {.addr = 0x64, .value = 0xCD, .mask = 0xFF},
};

enum speed_detection {
	SPEED_DETECTION_25G,
	SPEED_DETECTION_10G
};

static int al_eth_lm_retimer_br210_config(struct al_eth_lm_context *lm_context);
static int al_eth_lm_retimer_br410_config(struct al_eth_lm_context *lm_context);
static int al_eth_lm_retimer_ds25_full_config(struct al_eth_lm_context *lm_context);
static int al_eth_lm_retimer_ds125_config(struct al_eth_lm_context *lm_context);
static al_bool al_eth_lm_retimer_ds25_signal_detect(
		struct al_eth_lm_context *lm_context, uint32_t channel);
static int al_eth_lm_retimer_ds25_cdr_reset(struct al_eth_lm_context *lm_context, uint32_t channel);
static al_bool al_eth_lm_retimer_ds25_cdr_lock(
		struct al_eth_lm_context *lm_context, uint32_t channel);
static int al_eth_lm_retimer_25g_rx_adaptation_step(struct al_eth_lm_context *lm_context);
static enum speed_detection al_eth_lm_retimer_ds25_speed_get(
						struct al_eth_lm_context *lm_context,
						uint32_t channel);

struct al_eth_lm_retimer {
	int (*config)(struct al_eth_lm_context *lm_context);
	int (*reset)(struct al_eth_lm_context *lm_context, uint32_t channel);
	int (*signal_detect)(struct al_eth_lm_context *lm_context, uint32_t channel);
	int (*cdr_lock)(struct al_eth_lm_context *lm_context, uint32_t channel);
	int (*rx_adaptation)(struct al_eth_lm_context *lm_context);
	enum speed_detection (*speed_get)(struct al_eth_lm_context *lm_context, uint32_t channel);
};

static struct al_eth_lm_retimer retimer[] = {
	{.config = al_eth_lm_retimer_br210_config, .signal_detect = NULL,
		.reset = NULL, .cdr_lock = NULL, .rx_adaptation = NULL, .speed_get = NULL},
	{.config = al_eth_lm_retimer_br410_config, .signal_detect = NULL,
		.reset = NULL, .cdr_lock = NULL, .rx_adaptation = NULL, .speed_get = NULL},
	{.config = al_eth_lm_retimer_ds25_full_config,
		.signal_detect = al_eth_lm_retimer_ds25_signal_detect,
		.reset = al_eth_lm_retimer_ds25_cdr_reset,
		.cdr_lock = al_eth_lm_retimer_ds25_cdr_lock,
		.rx_adaptation = al_eth_lm_retimer_25g_rx_adaptation_step,
		.speed_get = al_eth_lm_retimer_ds25_speed_get},
	{.config = al_eth_lm_retimer_ds125_config, .signal_detect = NULL,
              .reset = NULL, .cdr_lock = NULL, .rx_adaptation = NULL, .speed_get = NULL},
};

#define SFP_10G_DA_ACTIVE		0x8
#define SFP_10G_DA_PASSIVE		0x4

#define lm_debug(...)				\
	do {					\
		if (lm_context->debug)		\
			al_warn(__VA_ARGS__);	\
		else				\
			al_dbg(__VA_ARGS__);	\
	} while (0)

#define TIMED_OUT(current_time, start_time, timeout_val)	\
	(((current_time) - (start_time) >= (timeout_val)) ? AL_TRUE : AL_FALSE)

static inline al_bool elapsed_time_msec(unsigned int current_time,
					unsigned int *start_time,
					unsigned int time_interval)
{
	if ((current_time - *start_time) >= time_interval) {
		*start_time = current_time;
		return AL_TRUE;
	}

	return AL_FALSE;
}

static unsigned int al_eth_sfp_crc(void *buf, size_t len)
{
	uint8_t *p, check;

	for (p = buf, check = 0; len; p++, len--)
		check += *p;

	return check;
}

static bool al_eth_sfp_is_10g(struct sfp_eeprom_id *id)
{
	return (id->base.e10g_base_er || id->base.e10g_base_lrm || id->base.e10g_base_lr ||
		id->base.e10g_base_sr || id->base.if_1x_sx || id->base.if_1x_lx ||
		id->base.if_1x_copper_active || id->base.if_1x_copper_passive) ?
		       true:
		       false;
}

static bool al_eth_sfp_is_1g(struct sfp_eeprom_id *id)
{
	return (id->base.e_base_px || id->base.e_base_bx10 || id->base.e100_base_fx ||
		id->base.e100_base_lx || id->base.e1000_base_t || id->base.e1000_base_cx ||
		id->base.e1000_base_lx || id->base.e1000_base_sx) ?
		       true :
		       false;
}

static int al_eth_sfp_mii_read(struct al_eth_lm_context *lm_context, int addr, uint16_t *val) {

	uint8_t data[2] = {0};
	int rc = 0;

	al_assert(val);
	al_assert(lm_context);
	al_assert(lm_context->i2c_read_data);

	rc = lm_context->i2c_read_data(lm_context->i2c_context, lm_context->sfp_bus_id,
				       SFP_I2C_ADDR_PHY, addr, data, sizeof(data));
	if (rc)
		return rc;

	*val = data[0] << 8 | data[1];

	return 0;
}

static int al_eth_sfp_phy_id(struct al_eth_lm_context *lm_context, int *phy_id) {
	uint16_t phy_reg;
	int rc = 0;

	al_assert(phy_id);

	rc = al_eth_sfp_mii_read(lm_context, MII_PHYSID1, &phy_reg);
	if(rc) {
		*phy_id = 0xffffffff;
		return rc;
	}

	*phy_id = phy_reg << 16;
	rc = al_eth_sfp_mii_read(lm_context, MII_PHYSID2, &phy_reg);
	if(rc) {
		*phy_id = 0xffffffff;
		return rc;
	}
	*phy_id |= phy_reg;
	return 0;
}

static int al_eth_sfp_detect(struct al_eth_lm_context *lm_context,
				 enum al_eth_lm_link_mode *new_mode)
{
	/* SFP module inserted - read I2C data */
	struct sfp_eeprom_id id;
	bool cotsworks;
	uint8_t cc = 0;
	int rc = 0;
	int phy_id;

	al_assert(lm_context->i2c_read_data);

	lm_context->sfp_has_phyreg = AL_FALSE;

	do {
		rc = lm_context->i2c_read_data(lm_context->i2c_context, lm_context->sfp_bus_id,
					       lm_context->sfp_i2c_addr, 0, (uint8_t *)&id,
					       sizeof(id));
		if (rc)
			break;

		/* Cotsworks do not seem to update the checksums when they
		* do the final programming with the final module part number,
		* serial number and date code.
		*/
		cotsworks = !memcmp(id.base.vendor_name, "COTSWORKS       ", 16);
		cc = al_eth_sfp_crc(&id.base, sizeof(id.base) - 1);
		if (cc != id.base.cc_base && !cotsworks) {
			al_warn("%s: EEPROM base structure checksum failure (0x%02x != 0x%02x)",
				    __func__, cc, id.base.cc_base);
			break;
		}

		cc = al_eth_sfp_crc(&id.ext, sizeof(id.ext) - 1);
		if (cc != id.ext.cc_ext && !cotsworks) {
			al_warn("%s: EEPROM extended structure checksum failure: 0x%02x != 0x%02x",
				 __func__, cc, id.ext.cc_ext);
			memset(&id.ext, 0, sizeof(id.ext));
		}

		if (!al_eth_sfp_phy_id(lm_context, &phy_id)) {
			lm_debug("%s: SFP module has PHY with ID 0x%08x",
			__func__, phy_id);
			lm_context->sfp_has_phyreg = AL_TRUE;
		}
	} while (0);

	if (rc) {
		if (rc == -ETIMEDOUT) {
			/* ETIMEDOUT is returned when no SFP is connected */
			if (lm_context->mode != AL_ETH_LM_MODE_DISCONNECTED)
				lm_debug("%s: SFP Disconnected", __func__);
			*new_mode = AL_ETH_LM_MODE_DISCONNECTED;
		} else {
			return rc;
		}
	} else if (lm_context->sfp_probe_10g && (id.base.sfp_ct_passive || id.base.sfp_ct_active)) {
		if ((id.base.br_nominal >= SFP_MIN_SIGNAL_RATE_25G) &&
		    ((lm_context->max_speed == AL_ETH_LM_MAX_SPEED_25G) ||
		     (lm_context->max_speed == AL_ETH_LM_MAX_SPEED_MAX)))
			*new_mode = AL_ETH_LM_MODE_25G;
		else if ((id.base.br_nominal >= SFP_MIN_SIGNAL_RATE_10G) &&
			 ((lm_context->max_speed == AL_ETH_LM_MAX_SPEED_10G) ||
			  (lm_context->max_speed == AL_ETH_LM_MAX_SPEED_MAX)))
			*new_mode = AL_ETH_LM_MODE_10G_DA;
		else
			*new_mode = AL_ETH_LM_MODE_1G;

		if ((lm_context->mode != *new_mode) &&
		    (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED)) {
			lm_debug("%s: %s DAC (%d M) detected (max signal rate %d)", __func__,
				 (id.base.sfp_ct_passive) ? "Passive" : "Active",
				 id.base.link_len[5], id.base.br_nominal);
		}
		/* for active direct attached need to use len 0 in the retimer configuration */
		lm_context->da_len = (id.base.sfp_ct_passive) ? id.base.link_len[5] : 0;
	} else if (lm_context->sfp_probe_10g && al_eth_sfp_is_10g(&id)) {
		*new_mode = AL_ETH_LM_MODE_10G_OPTIC;
		if ((lm_context->mode != *new_mode) &&
		    (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED))
			lm_debug("%s: 10 SFP detected\n", __func__);
	} else if (lm_context->sfp_probe_1g && al_eth_sfp_is_1g(&id)) {
		*new_mode = AL_ETH_LM_MODE_1G;
		if ((lm_context->mode != *new_mode) &&
		    (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED))
			lm_debug("%s: 1G SFP detected\n", __func__);
	} else {
		*new_mode = lm_context->default_mode;
		lm_context->da_len = lm_context->default_dac_len;
		if ((lm_context->mode != *new_mode) &&
		    (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED)) {
			al_warn("%s: unknown SFP inserted. default to %s", __func__,
				    al_eth_lm_mode_convert_to_str(lm_context->default_mode));
			print_hex_dump(KERN_ERR, "SFP EE: ", DUMP_PREFIX_OFFSET, 16, 1, &id,
				       sizeof(id), true);
		}
	}

	if ((lm_context->sfp_detect_force_mode) && (*new_mode != AL_ETH_LM_MODE_DISCONNECTED) &&
	    (*new_mode != lm_context->default_mode)) {
		if (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED)
			al_warn(
				"%s: Force mode to default (%s). mode based of the SFP EEPROM %s",
				__func__,
				al_eth_lm_mode_convert_to_str(lm_context->default_mode),
				al_eth_lm_mode_convert_to_str(*new_mode));

		*new_mode = lm_context->default_mode;
	}

	mutex_lock(&lm_context->lock);
	lm_context->sfp_id = id;
	if (*new_mode != AL_ETH_LM_MODE_DISCONNECTED) {
		if (lm_context->link_conf.speed > SPEED_10 &&
		    lm_context->link_conf.speed <= SPEED_1000) {
			/* 1G DA needs special serdes rx/tx params */
			*new_mode = (*new_mode == AL_ETH_LM_MODE_10G_DA) ? AL_ETH_LM_MODE_1G_DA :
									   AL_ETH_LM_MODE_1G;
		} else if (lm_context->link_conf.speed > SPEED_1000) {
			*new_mode = AL_ETH_LM_MODE_10G_OPTIC;
		}
	}
	mutex_unlock(&lm_context->lock);
	lm_context->mode = *new_mode;

	lm_debug(
		"%s: (mode %s) SFP inserted -> Vendor:%.*s,PN:%.*s, REV:%.*s, SERIAL:%.*s PHY : %s",
		__func__, al_eth_lm_mode_convert_to_str(lm_context->mode),
		(int)sizeof(id.base.vendor_name), id.base.vendor_name,
		(int)sizeof(id.base.vendor_pn), id.base.vendor_pn, (int)sizeof(id.base.vendor_rev),
		id.base.vendor_rev, (int)sizeof(id.ext.vendor_sn), id.ext.vendor_sn,
		lm_context->sfp_has_phyreg ? "yes" : "no");

	return 0;
}

static int al_eth_qsfp_detect(struct al_eth_lm_context	*lm_context,
			      enum al_eth_lm_link_mode	*new_mode)
{
	int rc = 0;
	uint8_t qsfp_comp_code;
	uint8_t qsfp_da_len;

	do {
		rc = lm_context->i2c_read(lm_context->i2c_context,
					  lm_context->sfp_bus_id,
					  lm_context->sfp_i2c_addr,
					  QSFP_COMPLIANCE_CODE_IDX, &qsfp_comp_code);
		if (rc)
			break;

		rc = lm_context->i2c_read(lm_context->i2c_context,
					  lm_context->sfp_bus_id,
					  lm_context->sfp_i2c_addr,
					  QSFP_CABLE_LEN_IDX, &qsfp_da_len);
		if (rc)
			break;
	} while (0);

	if (rc) {
		if (rc == -ETIMEDOUT) {
			/* ETIMEDOUT is returned when no SFP is connected */
			lm_debug("%s: SFP Disconnected\n", __func__);
			*new_mode = AL_ETH_LM_MODE_DISCONNECTED;
		} else {
			return rc;
		}
	} else if (qsfp_comp_code & QSFP_COMPLIANCE_CODE_DAC) {
		*new_mode = AL_ETH_LM_MODE_10G_DA;
		lm_context->da_len = qsfp_da_len;
		if ((lm_context->mode != *new_mode) &&
			(lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED))
			lm_debug("%s: 10G passive DAC (%d M) detected\n", __func__, qsfp_da_len);
	} else if (qsfp_comp_code & QSFP_COMPLIANCE_CODE_OPTIC) {
		*new_mode = AL_ETH_LM_MODE_10G_OPTIC;
		if ((lm_context->mode != *new_mode) &&
			(lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED))
			lm_debug("%s: 10G optic module detected\n", __func__);
	} else {
		*new_mode = lm_context->default_mode;
		lm_context->da_len = lm_context->default_dac_len;
		if ((lm_context->mode != *new_mode) &&
			(lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED))
			al_warn("%s: unknown QSFP inserted. eeprom content: 10G compliance 0x%x."
			" default to %s\n",
			__func__, qsfp_comp_code,
			al_eth_lm_mode_convert_to_str(lm_context->default_mode));
	}

	lm_context->mode = *new_mode;

	return 0;
}

static int al_eth_module_detect(struct al_eth_lm_context	*lm_context,
				enum al_eth_lm_link_mode	*new_mode)
{
	int i;
	int rc = 0;
	uint8_t module_idx;
	int sfp_present = SFP_PRESENT;
	al_bool use_gpio_present = AL_FALSE;
	al_bool bounce_detected = AL_FALSE;

	if ((lm_context->gpio_get) && (lm_context->gpio_present != 0)) {
		int sfp_present_debounce;

		use_gpio_present = AL_TRUE;
		sfp_present = lm_context->gpio_get(lm_context->gpio_present);
		for (i = 0; i < SFP_PRESENT_GPIO_DEBOUNCE_ITERS; i++) {
			al_udelay(1);
			sfp_present_debounce = lm_context->gpio_get(lm_context->gpio_present);
			if (sfp_present_debounce != sfp_present) {
				bounce_detected = AL_TRUE;
				break;
			}
		}
	}

	if (bounce_detected) {
		*new_mode = lm_context->mode;

		return 0;
	}

	if (sfp_present == SFP_NOT_PRESENT) {
		if (lm_context->mode != AL_ETH_LM_MODE_DISCONNECTED)
			lm_debug("%s: SFP not present\n", __func__);
		*new_mode = AL_ETH_LM_MODE_DISCONNECTED;

		return 0;
	} else if (use_gpio_present && lm_context->sfp_detect_force_mode) {
		if (lm_context->mode == AL_ETH_LM_MODE_DISCONNECTED)
			lm_debug("%s: Force mode to default (%s)\n",
			__func__, al_eth_lm_mode_convert_to_str(lm_context->default_mode));
		*new_mode = lm_context->default_mode;

		return 0;
	}

	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->sfp_bus_id,
				  lm_context->sfp_i2c_addr,
				  MODULE_IDENTIFIER_IDX, &module_idx);
	if (rc) {
		if (rc == -ETIMEDOUT) {
			if (use_gpio_present) {
				sfp_present = lm_context->gpio_get(lm_context->gpio_present);
				if (sfp_present == SFP_PRESENT) {
					al_warn("%s: SFP present, but i2c read from SFP failed. Force mode to default (%s).\n",
						__func__, al_eth_lm_mode_convert_to_str(
						lm_context->default_mode));
					*new_mode = lm_context->default_mode;
				} else {
					lm_debug("%s: SFP disconnected during i2c read\n",
						__func__);
					*new_mode = AL_ETH_LM_MODE_DISCONNECTED;

					return 0;
				}
			} else {
				/* ETIMEDOUT is returned when no SFP is connected */
				if (lm_context->mode != AL_ETH_LM_MODE_DISCONNECTED)
					lm_debug("%s: SFP Disconnected\n", __func__);
				*new_mode = AL_ETH_LM_MODE_DISCONNECTED;
			}
			return 0;
		} else {
			return rc;
		}
	}

	if (module_idx == MODULE_IDENTIFIER_QSFP)
		return al_eth_qsfp_detect(lm_context, new_mode);
	else
		return al_eth_sfp_detect(lm_context, new_mode);

	return 0;
}

static struct al_serdes_adv_tx_params da_tx_params = {
	.override		= AL_TRUE,
	.amp			= 0x1,
	.total_driver_units	= 0x13,
	.c_plus_1		= 0x2,
	.c_plus_2		= 0,
	.c_minus_1		= 0x2,
	.slew_rate		= 0,
};

static struct al_serdes_adv_rx_params da_rx_params = {
	.override		= AL_TRUE,
	.dcgain			= 0x4,
	.dfe_3db_freq		= 0x4,
	.dfe_gain		= 0x3,
	.dfe_first_tap_ctrl	= 0x5,
	.dfe_secound_tap_ctrl	= 0x1,
	.dfe_third_tap_ctrl	= 0x8,
	.dfe_fourth_tap_ctrl	= 0x1,
	.low_freq_agc_gain	= 0x7,
	.precal_code_sel	= 0,
	.high_freq_agc_boost	= 0x1d,
};

static struct al_serdes_adv_tx_params da_1g_tx_params = {
	.override		= AL_TRUE,
	.amp			= 0x1,
	.total_driver_units	= 0x13,
	.c_plus_1		= 0x2,
	.c_plus_2		= 0,
	.c_minus_1		= 0,
	.slew_rate		= 0,
};

static struct al_serdes_adv_rx_params da_1g_rx_params = {
	.override		= AL_TRUE,
	.dcgain			= 0x0,
	.dfe_3db_freq		= 0x7,
	.dfe_gain		= 0x0,
	.dfe_first_tap_ctrl	= 0x0,
	.dfe_secound_tap_ctrl	= 0x8,
	.dfe_third_tap_ctrl	= 0x0,
	.dfe_fourth_tap_ctrl	= 0x8,
	.low_freq_agc_gain	= 0x7,
	.precal_code_sel	= 0,
	.high_freq_agc_boost	= 0x4,
};

/* These params have been calibrated for br410 retimer on EVP only! */
static struct al_serdes_adv_tx_params tx_params_br410 = {
	.override		= AL_TRUE,
	.amp			= 0x1,
	.total_driver_units	= 0x1b,
	.c_plus_1		= 0x6,
	.c_plus_2		= 0,
	.c_minus_1		= 0,
	.slew_rate		= 0,
};

/* These params have been calibrated for EVP only! */
static struct al_serdes_adv_rx_params rx_params_br410 = {
	.override		= AL_TRUE,
	.dcgain			= 0x0,
	.dfe_3db_freq		= 0x7,
	.dfe_gain		= 0x0,
	.dfe_first_tap_ctrl	= 0x0,
	.dfe_secound_tap_ctrl	= 0x0,
	.dfe_third_tap_ctrl	= 0x0,
	.dfe_fourth_tap_ctrl	= 0x0,
	.low_freq_agc_gain	= 0x7,
	.precal_code_sel	= 0,
	.high_freq_agc_boost	= 0x4,
};

static struct al_serdes_adv_tx_params optic_tx_params = {
	.override		= AL_TRUE,
	.amp			= 0x1,
	.total_driver_units	= 0x13,
	.c_plus_1		= 0x2,
	.c_plus_2		= 0,
	.c_minus_1		= 0,
	.slew_rate		= 0,
};

static struct al_serdes_adv_rx_params optic_rx_params = {
	.override		= AL_TRUE,
	.dcgain			= 0x0,
	.dfe_3db_freq		= 0x7,
	.dfe_gain		= 0x0,
	.dfe_first_tap_ctrl	= 0x0,
	.dfe_secound_tap_ctrl	= 0x8,
	.dfe_third_tap_ctrl	= 0x0,
	.dfe_fourth_tap_ctrl	= 0x8,
	.low_freq_agc_gain	= 0x7,
	.precal_code_sel	= 0,
	.high_freq_agc_boost	= 0x4,
};


static void al_eth_serdes_static_tx_params_set(struct al_eth_lm_context *lm_context)
{
	if (lm_context->tx_param_dirty == 0)
		return;

	if (lm_context->serdes_tx_params_valid) {
		lm_context->tx_param_dirty = 0;

		lm_context->tx_params_override.override = AL_TRUE;

		if (!(lm_context->serdes_obj->tx_advanced_params_set)) {
			al_err("tx_advanced_params_set is not supported for this serdes group\n");
			return;
		}

		lm_context->serdes_obj->tx_advanced_params_set(
					lm_context->serdes_obj,
					lm_context->lane,
					&lm_context->tx_params_override);

	} else if (lm_context->static_values) {
		lm_context->tx_param_dirty = 0;

		if (!(lm_context->serdes_obj->tx_advanced_params_set)) {
			al_err("tx_advanced_params_set is not supported for this serdes group\n");
			return;
		}

		if ((!lm_context->retimer_exist) && (lm_context->mode == AL_ETH_LM_MODE_10G_DA))
			lm_context->serdes_obj->tx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&da_tx_params);
		else if (lm_context->retimer_exist &&
			(lm_context->retimer_type == AL_ETH_RETIMER_BR_410))
			lm_context->serdes_obj->tx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&tx_params_br410);
		else if (lm_context->mode == AL_ETH_LM_MODE_1G_DA)
			lm_context->serdes_obj->tx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&da_1g_tx_params);
		else
			lm_context->serdes_obj->tx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&optic_tx_params);
	}
}

static void al_eth_serdes_static_rx_params_set(struct al_eth_lm_context *lm_context)
{
	if (lm_context->rx_param_dirty == 0)
		return;

	if (lm_context->serdes_rx_params_valid) {
		lm_context->rx_param_dirty = 0;

		lm_context->rx_params_override.override = AL_TRUE;

		if (!(lm_context->serdes_obj->rx_advanced_params_set)) {
			al_err("rx_advanced_params_set is not supported for this serdes group\n");
			return;
		}

		lm_context->serdes_obj->rx_advanced_params_set(
					lm_context->serdes_obj,
					lm_context->lane,
					&lm_context->rx_params_override);

	} else if (lm_context->static_values) {
		lm_context->rx_param_dirty = 0;

		if (!(lm_context->serdes_obj->rx_advanced_params_set)) {
			al_err("rx_advanced_params_set is not supported for this serdes group\n");
			return;
		}

		if ((!lm_context->retimer_exist) && (lm_context->mode == AL_ETH_LM_MODE_10G_DA))
			lm_context->serdes_obj->rx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&da_rx_params);
		else if (lm_context->retimer_exist &&
			(lm_context->retimer_type == AL_ETH_RETIMER_BR_410))
			lm_context->serdes_obj->rx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&rx_params_br410);
		else if (lm_context->mode == AL_ETH_LM_MODE_1G_DA)
			lm_context->serdes_obj->rx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&da_1g_rx_params);
		else
			lm_context->serdes_obj->rx_advanced_params_set(
						lm_context->serdes_obj,
						lm_context->lane,
						&optic_rx_params);
	}
}

static int al_eth_rx_equal_run(struct al_eth_lm_context	*lm_context)
{
	struct al_serdes_adv_rx_params rx_params;
	int dcgain;
	int best_dcgain = -1;
	int i;
	int best_score  = -1;
	int test_score = -1;

	rx_params.override = AL_FALSE;
	lm_context->serdes_obj->rx_advanced_params_set(
					lm_context->serdes_obj,
					lm_context->lane,
					&rx_params);

	lm_debug("score | dcgain | dfe3db | dfegain | tap1 | tap2 | tap3 | tap4 | low freq | high freq\n");

	for (dcgain = 0; dcgain < AL_ETH_LM_MAX_DCGAIN; dcgain++) {
		lm_context->serdes_obj->dcgain_set(
					lm_context->serdes_obj,
					dcgain);

		test_score = lm_context->serdes_obj->rx_equalization(
					lm_context->serdes_obj,
					lm_context->lane);

		if (test_score < 0) {
			al_dbg("serdes rx equalization failed on error\n");
			return test_score;
		}

		if (test_score > best_score) {
			best_score = test_score;
			best_dcgain = dcgain;
		}

		lm_context->serdes_obj->rx_advanced_params_get(
					lm_context->serdes_obj,
					lm_context->lane,
					&rx_params);

		lm_debug("%6d|%8x|%8x|%9x|%6x|%6x|%6x|%6x|%10x|%10x|\n",
			test_score, rx_params.dcgain, rx_params.dfe_3db_freq,
			rx_params.dfe_gain, rx_params.dfe_first_tap_ctrl,
			rx_params.dfe_secound_tap_ctrl, rx_params.dfe_third_tap_ctrl,
			rx_params.dfe_fourth_tap_ctrl, rx_params.low_freq_agc_gain,
			rx_params.high_freq_agc_boost);

	}

	lm_context->serdes_obj->dcgain_set(
					lm_context->serdes_obj,
					best_dcgain);

	best_score = -1;
	for(i = 0; i < AL_ETH_LM_EQ_ITERATIONS; i++) {
		test_score = lm_context->serdes_obj->rx_equalization(
						lm_context->serdes_obj,
						lm_context->lane);

		if (test_score < 0) {
			al_dbg("serdes rx equalization failed on error\n");
			return test_score;
		}

		if (test_score > best_score) {
			best_score = test_score;
			lm_context->serdes_obj->rx_advanced_params_get(
						lm_context->serdes_obj,
						lm_context->lane,
						&rx_params);
		}
	}

	rx_params.precal_code_sel = 0;
	rx_params.override = AL_TRUE;
	lm_context->serdes_obj->rx_advanced_params_set(
					lm_context->serdes_obj,
					lm_context->lane,
					&rx_params);

	lm_debug("-------------------- best dcgain %d ------------------------------------\n", best_dcgain);
	lm_debug("%6d|%8x|%8x|%9x|%6x|%6x|%6x|%6x|%10x|%10x|\n",
		best_score, rx_params.dcgain, rx_params.dfe_3db_freq,
		rx_params.dfe_gain, rx_params.dfe_first_tap_ctrl,
		rx_params.dfe_secound_tap_ctrl, rx_params.dfe_third_tap_ctrl,
		rx_params.dfe_fourth_tap_ctrl, rx_params.low_freq_agc_gain,
		rx_params.high_freq_agc_boost);

	return 0;

}

static int al_eth_lm_retimer_br210_config(struct al_eth_lm_context	*lm_context)
{
	int i;
	int rc = 0;
	uint8_t boost = 0;
	uint32_t boost_addr =
		al_eth_retimer_boost_addr[lm_context->retimer_channel][lm_context->retimer_type];

	if (lm_context->mode != AL_ETH_LM_MODE_10G_DA) {
		boost = al_eth_retimer_params[0][lm_context->retimer_type].eq_boost;
	} else {
		for (i = 0; i < RETIMER_LENS_MAX; i++) {
			if (lm_context->da_len <= al_eth_retimer_boost_lens[i]) {
				boost = al_eth_retimer_params[i][lm_context->retimer_type].eq_boost;
				break;
			}

		}

		if (i == RETIMER_LENS_MAX)
			boost = al_eth_retimer_params[RETIMER_LENS_MAX][lm_context->retimer_type].
				eq_boost;
	}

	lm_debug("config retimer boost in channel %d (addr %x) to 0x%x\n",
		 lm_context->retimer_channel, boost_addr, boost);

	rc = lm_context->i2c_write(lm_context->i2c_context,
				   lm_context->retimer_bus_id,
				   lm_context->retimer_i2c_addr,
				   boost_addr,
				   boost);

	if (rc) {
		al_err("%s: Error occurred (%d) while writing retimer configuration (bus-id %x i2c-addr %x)\n",
		       __func__, rc, lm_context->retimer_bus_id, lm_context->retimer_i2c_addr);
		return rc;
	}


	return 0;
}

#define RETIMER_BR410_DE_EMPHASIS_CTRL_REG 0x11
#define RETIMER_BR410_DE_EMPHASIS_CTRL_MASK(channel) (0x2 << 2*(channel))
#define RETIMER_BR410_DE_EMPHASIS_CTRL_SHIFT(channel) (2*(channel))

#define RETIMER_BR410_DRIVER_VOD_CTRL_REG 0x08
#define RETIMER_BR410_DRIVER_VOD_CTRL_MASK 0xC
#define RETIMER_BR410_DRIVER_VOD_CTRL_SHIFT 2

static int al_eth_lm_retimer_br410_config(struct al_eth_lm_context	*lm_context)
{
	int rc = 0;
	struct al_eth_retimer_params retimer_params;
	uint32_t boost_addr;
	uint8_t reg;
	uint8_t vod_val;

	al_assert((((int)lm_context->retimer_channel) >= 0) && (lm_context->retimer_channel < 4));

	al_memset(&retimer_params, 0, sizeof(retimer_params));

	boost_addr =
		al_eth_retimer_boost_addr[lm_context->retimer_channel][lm_context->retimer_type];

	/* config eq_boost value */
	if ((lm_context->da_len == 1) || (lm_context->da_len == 3))
		retimer_params =
			al_eth_retimer_params[lm_context->da_len][lm_context->retimer_type];
	else {
		al_err("%s: Unsupported cable length (%dm) for retimer_type %d on this board!\n",
			__func__, lm_context->da_len, lm_context->retimer_type);
		al_err("Currently only 1m and 3m cables are supported\n");
		return -EIO;
	}

	if (lm_context->da_len <= 1)
		retimer_params =
			al_eth_retimer_params[1][lm_context->retimer_type];
	else
		retimer_params =
			al_eth_retimer_params[3][lm_context->retimer_type];

	if ((lm_context->da_len != 1) || (lm_context->da_len != 3)) {
		al_warn("%s: Uncalibrated cable length (%dm) for retimer_type %d on this board is being used! Link might not be optimal.\n",
			__func__, lm_context->da_len, lm_context->retimer_type);
		al_warn("Retimer params have been calibrated for 1m and 3m cables only\n");
	}

	lm_debug("%s: Configuring eq_boost for retimer_type %d in channel %d (addr 0x%x) to 0x%x\n",
		__func__, lm_context->retimer_type, lm_context->retimer_channel, boost_addr,
		retimer_params.eq_boost);

	rc = lm_context->i2c_write(lm_context->i2c_context,
				   lm_context->retimer_bus_id,
				   lm_context->retimer_i2c_addr,
				   boost_addr,
				   retimer_params.eq_boost);

	if (rc) {
		al_err("%s: Error occurred (%d) while writing to eq_boost to retimer (bus-id %x i2c-addr %x)\n",
		       __func__, rc, lm_context->retimer_bus_id, lm_context->retimer_i2c_addr);
		return rc;
	}

	/* config de_emphasis value */
	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  RETIMER_BR410_DE_EMPHASIS_CTRL_REG,
				  &reg);

	if (rc) {
		al_err("%s: Error occurred (%d) while reading de_emphasis control reg from retimer (bus-id 0x%x i2c-addr 0x%x internal_reg 0x%x)\n",
			__func__, rc, lm_context->retimer_bus_id, lm_context->retimer_i2c_addr,
			RETIMER_BR410_DE_EMPHASIS_CTRL_REG);
		return rc;
	}

	AL_REG_FIELD_SET(reg,
			 RETIMER_BR410_DE_EMPHASIS_CTRL_MASK(lm_context->retimer_channel),
			 RETIMER_BR410_DE_EMPHASIS_CTRL_SHIFT(lm_context->retimer_channel),
			 retimer_params.de_emphasis);

	lm_debug("%s: Configuring de_emphasis for retimer_type %d in channel %d to 0x%x\n",
		__func__, lm_context->retimer_type, lm_context->retimer_channel,
		retimer_params.de_emphasis);

	rc = lm_context->i2c_write(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  RETIMER_BR410_DE_EMPHASIS_CTRL_REG,
				  reg);

	if (rc) {
		al_err("%s: Error occurred (%d) while writing to de_emphasis control reg of retimer (bus-id 0x%x i2c-addr 0x%x internal_reg 0x%x)\n",
			__func__, rc, lm_context->retimer_bus_id, lm_context->retimer_i2c_addr,
			RETIMER_BR410_DE_EMPHASIS_CTRL_REG);
		return rc;
	}

	/* config Vod value */
	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  RETIMER_BR410_DRIVER_VOD_CTRL_REG,
				  &reg);

	if (rc) {
		al_err("%s: Error occurred (%d) while reading Vod control reg from retimer (bus-id 0x%x i2c-addr 0x%x internal_reg 0x%x)\n",
			__func__, rc, lm_context->retimer_bus_id, lm_context->retimer_i2c_addr,
			RETIMER_BR410_DRIVER_VOD_CTRL_REG);
		return rc;
	}

	vod_val = AL_REG_FIELD_GET(reg, RETIMER_BR410_DRIVER_VOD_CTRL_MASK,
				RETIMER_BR410_DRIVER_VOD_CTRL_SHIFT);

	if (vod_val != retimer_params.vod) {
		AL_REG_FIELD_SET(reg,
				RETIMER_BR410_DRIVER_VOD_CTRL_MASK,
				RETIMER_BR410_DRIVER_VOD_CTRL_SHIFT,
				retimer_params.vod);

		lm_debug("%s: Configuring Vod for retimer_type %d in channel %d to 0x%x\n",
		__func__, lm_context->retimer_type, lm_context->retimer_channel,
		retimer_params.vod);

		rc = lm_context->i2c_write(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  RETIMER_BR410_DRIVER_VOD_CTRL_REG,
				  reg);

		if (rc) {
			al_err("%s: Error occurred (%d) while writing to Vod control reg in retimer (bus-id 0x%x i2c-addr 0x%x internal_reg 0x%x)\n",
				__func__, rc, lm_context->retimer_bus_id,
				lm_context->retimer_i2c_addr,
				RETIMER_BR410_DRIVER_VOD_CTRL_REG);
		return rc;
		}
	}

	return 0;
}

/*******************************************************************************
 ************************** retimer DS25 ***************************************
 ******************************************************************************/
#define LM_DS25_CHANNEL_EN_REG		0xff
#define LM_DS25_CHANNEL_EN_MASK		0x03
#define LM_DS25_CHANNEL_EN_VAL		0x01

#define LM_DS25_CHANNEL_SEL_REG		0xfc
#define LM_DS25_CHANNEL_SEL_MASK	0xff

#define LM_DS25_CDR_RESET_REG		0x0a
#define LM_DS25_CDR_RESET_MASK		0x0c
#define LM_DS25_CDR_RESET_ASSERT	0x0c
#define LM_DS25_CDR_RESET_RELEASE	0x00

#define LM_DS25_SIGNAL_DETECT_REG	0x78
#define LM_DS25_SIGNAL_DETECT_MASK	0x20

#define LM_DS25_CDR_LOCK_REG		0x78
#define LM_DS25_CDR_LOCK_MASK		0x10

#define LM_DS25_DRV_PD_REG		0x15
#define LM_DS25_DRV_PD_MASK		0x08

#define LM_DS25_CDR_STATUS_CTRL_REG	0xc
#define LM_DS25_CDR_STATUS_CTRL_DIV_MASK 0xff
#define LM_DS25_CDR_STATUS_CTRL_DIV_VAL	0x30
#define LM_DS25_CDR_STATUS_REG		0x2
#define LM_DS25_CDR_STATUS_DIV_10G	0x9

static int al_eth_lm_retimer_ds25_write_reg(struct al_eth_lm_context	*lm_context,
					    uint8_t			reg_addr,
					    uint8_t			reg_mask,
					    uint8_t			reg_value)
{
	uint8_t reg = 0;
	int rc;

	al_assert((reg_mask & reg_value) == reg_value);

	if (reg_mask != 0xff) {
		rc = lm_context->i2c_read(lm_context->i2c_context,
					lm_context->retimer_bus_id,
					lm_context->retimer_i2c_addr,
					reg_addr,
					&reg);

		if (rc)
			return -EIO;

		reg &= ~(reg_mask);
	}

	reg |= reg_value;

	rc = lm_context->i2c_write(lm_context->i2c_context,
				   lm_context->retimer_bus_id,
				   lm_context->retimer_i2c_addr,
				   reg_addr,
				   reg);

	if (rc)
		return -EIO;

	return 0;
}

static int al_eth_lm_retimer_ds25_channel_select(struct al_eth_lm_context	*lm_context,
						 uint8_t			channel)
{
	int rc = 0;

	/* Write to specific channel */
	rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
					      LM_DS25_CHANNEL_EN_REG,
					      LM_DS25_CHANNEL_EN_MASK,
					      LM_DS25_CHANNEL_EN_VAL);

	if (rc)
		return rc;

	rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
					      LM_DS25_CHANNEL_SEL_REG,
					      LM_DS25_CHANNEL_SEL_MASK,
					      (1 << channel));

	return rc;
}

static int al_eth_lm_retimer_ds25_channel_config(struct al_eth_lm_context	*lm_context,
						 uint8_t			channel,
						 struct retimer_config_reg	*config,
						 uint8_t			config_size)
{
	uint8_t i;
	int rc;

	rc = al_eth_lm_retimer_ds25_channel_select(lm_context, channel);
	if (rc)
		goto config_error;

	for (i = 0; i < config_size; i++) {
		rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
						      config[i].addr,
						      config[i].mask,
						      config[i].value);

		if (rc)
			goto config_error;
	}

	lm_debug("%s: retimer channel config done for channel %d\n", __func__, channel);

	return 0;

config_error:
	al_err("%s: failed to access to the retimer\n", __func__);

	return rc;
}

static int al_eth_lm_retimer_ds25_cdr_reset(struct al_eth_lm_context *lm_context, uint32_t channel)
{
	int rc;

	lm_debug("Perform CDR reset to channel %d\n", channel);

	rc = al_eth_lm_retimer_ds25_channel_select(lm_context, channel);
	if (rc)
		goto config_error;

	rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
					      LM_DS25_CDR_RESET_REG,
					      LM_DS25_CDR_RESET_MASK,
					      LM_DS25_CDR_RESET_ASSERT);

	if (rc)
		goto config_error;

	rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
					      LM_DS25_CDR_RESET_REG,
					      LM_DS25_CDR_RESET_MASK,
					      LM_DS25_CDR_RESET_RELEASE);

	if (rc)
		goto config_error;

	return 0;

config_error:
	al_err("%s: failed to access to the retimer\n", __func__);

	return rc;
}

static al_bool al_eth_lm_retimer_ds25_signal_detect(struct al_eth_lm_context *lm_context,
						    uint32_t channel)
{
	int rc = 0;
	uint8_t reg;

	rc = al_eth_lm_retimer_ds25_channel_select(lm_context, channel);
	if (rc)
		goto config_error;

	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  LM_DS25_SIGNAL_DETECT_REG,
				  &reg);

	if (rc)
		goto config_error;

	if (reg & LM_DS25_SIGNAL_DETECT_MASK)
		return AL_TRUE;

	return AL_FALSE;

config_error:
	al_err("%s: failed to access to the retimer\n", __func__);

	return AL_FALSE;
}

static al_bool al_eth_lm_retimer_ds25_cdr_lock(struct al_eth_lm_context *lm_context,
					       uint32_t channel)
{
	int rc = 0;
	uint8_t reg;

	rc = al_eth_lm_retimer_ds25_channel_select(lm_context, channel);
	if (rc)
		goto config_error;

	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  LM_DS25_CDR_LOCK_REG,
				  &reg);

	if (rc)
		goto config_error;

	if (reg & LM_DS25_CDR_LOCK_MASK)
		return AL_TRUE;

	return AL_FALSE;

config_error:
	al_err("%s: failed to access to the retimer\n", __func__);

	return AL_FALSE;
}

static enum speed_detection al_eth_lm_retimer_ds25_speed_get(
					struct al_eth_lm_context *lm_context,
					uint32_t channel)
{
	int rc = 0;
	uint8_t reg;

	rc = al_eth_lm_retimer_ds25_channel_select(lm_context, channel);
	if (rc)
		goto i2c_error;

	/* the following write will set the div value register LM_DS25_DIV_REG */
	rc = al_eth_lm_retimer_ds25_write_reg(
					lm_context,
					LM_DS25_CDR_STATUS_CTRL_REG,
					LM_DS25_CDR_STATUS_CTRL_DIV_MASK,
					LM_DS25_CDR_STATUS_CTRL_DIV_VAL);

	if (rc)
		goto i2c_error;

	rc = lm_context->i2c_read(lm_context->i2c_context,
				  lm_context->retimer_bus_id,
				  lm_context->retimer_i2c_addr,
				  LM_DS25_CDR_STATUS_REG,
				  &reg);

	if (rc)
		goto i2c_error;

	if (reg & LM_DS25_CDR_STATUS_DIV_10G)
		return SPEED_DETECTION_10G;

	return SPEED_DETECTION_25G;

i2c_error:
	al_err("%s: failed to access to the retimer\n", __func__);

	return SPEED_DETECTION_25G;
}

/* return value is 0 if done, or -EINPROGRESS if still waiting
 * lock parameter is true if lock succeded (or exited on error), and false if not
 */
static int al_eth_lm_wait_for_lock_step(struct al_eth_lm_context	*lm_context,
					uint32_t			channel,
					al_bool				*lock)
{
	unsigned int current_time;
	struct al_eth_lm_step_retimer_data *retimer_data = &lm_context->step_data.retimer_data;

	current_time = lm_context->get_msec();
	*lock = AL_FALSE;

	switch (retimer_data->wfl_state) {
	case LM_STEP_RETIMER_WAIT_FOR_LOCK_INIT:
		lm_context->step_data.start_time = current_time;
		retimer_data->wfl_last_time_checked =
			lm_context->step_data.start_time;
		retimer_data->wfl_state =
			LM_STEP_RETIMER_WAIT_FOR_LOCK_IN_PROGRESS;
		/* fall through*/
	case LM_STEP_RETIMER_WAIT_FOR_LOCK_IN_PROGRESS:

		/* check lock in 10ms intervals */
		if (elapsed_time_msec(current_time, &retimer_data->wfl_last_time_checked,
				      LM_STEP_RETIMER_WAIT_FOR_LOCK_CHECK_INTERVAL))
			*lock = retimer[lm_context->retimer_type].cdr_lock(lm_context, channel);

		if (*lock == AL_TRUE ||
		    TIMED_OUT(current_time, lm_context->step_data.start_time,
			      AL_ETH_LM_RETIMER_WAIT_FOR_LOCK)) {
			retimer_data->wfl_state =
				LM_STEP_RETIMER_WAIT_FOR_LOCK_INIT;
			return 0;
		}

		break;
	default:
		al_err("%s: Reached undefined state: %d\n", __func__,
		       retimer_data->wfl_state);
		retimer_data->wfl_state =
			LM_STEP_RETIMER_WAIT_FOR_LOCK_INIT;
		return 0;
	}

	al_dbg("%s: wfl_state:%d:, lock:%d, current_time:%d, start_time:%d\n",
		 __func__, retimer_data->wfl_state, *lock,
		 current_time, lm_context->step_data.start_time);

	return -EINPROGRESS;
}

/* return value is 0 if done, or -EINPROGRESS if still waiting
 * ready parameter is true if lock succeded (or exited on error), and false if not
 */
static al_bool al_eth_lm_retimer_signal_lock_check_step(struct al_eth_lm_context	*lm_context,
						uint32_t			channel,
						struct al_eth_lm_retimer_channel_status
							*last_channel_status,
						al_bool				force_reset,
						al_bool				*lock)
{
	al_bool signal_detect = AL_TRUE;
	al_bool cdr_lock = AL_TRUE;
	int rc = 0;

	switch (lm_context->step_data.retimer_data.slc_state) {
	case LM_STEP_RETIMER_SIGNAL_LOCK_INIT:
		if (!retimer[lm_context->retimer_type].signal_detect)
			break;

		signal_detect = retimer[lm_context->retimer_type].signal_detect(
			lm_context, channel);

		if (!signal_detect) {
			if (signal_detect != last_channel_status->signal_detect)
				lm_debug("%s: no signal detected on retimer channel %d\n", __func__,
					channel);
			break;
		}

		if (!retimer[lm_context->retimer_type].cdr_lock)
			break;
		cdr_lock = retimer[lm_context->retimer_type].cdr_lock(lm_context, channel);

		if (!force_reset && cdr_lock)
			break;

		if (!retimer[lm_context->retimer_type].reset)
			break;
		retimer[lm_context->retimer_type].reset(lm_context, channel);

		lm_context->step_data.retimer_data.slc_state =
			LM_STEP_RETIMER_SIGNAL_LOCK_IN_PROGRESS;
		/* fall through */
	case LM_STEP_RETIMER_SIGNAL_LOCK_IN_PROGRESS:
		rc = al_eth_lm_wait_for_lock_step(lm_context, channel, &cdr_lock);
		if (rc == -EINPROGRESS)
			goto exit_in_progress;

		break;
	default:
		al_err("%s: Reached undefined state: %d\n", __func__,
		       lm_context->step_data.retimer_data.slc_state);
	}

	*lock = (cdr_lock == AL_TRUE) && (signal_detect == AL_TRUE);

	lm_context->step_data.retimer_data.slc_state =
		LM_STEP_RETIMER_SIGNAL_LOCK_INIT;

	if (((signal_detect != last_channel_status->signal_detect) ||
			(cdr_lock != last_channel_status->cdr_lock)) ||
		last_channel_status->first_check)
		lm_debug("%s: (channel %d) signal %d cdr lock %d rc:%d\n",
			__func__, channel, signal_detect, (signal_detect) ? cdr_lock : 0, rc);


exit_in_progress:

	last_channel_status->signal_detect = signal_detect;
	last_channel_status->cdr_lock = cdr_lock;
	last_channel_status->first_check = AL_FALSE;

	return rc;
}

static int al_eth_lm_retimer_ds25_full_config(struct al_eth_lm_context *lm_context)
{
	int rc = 0;
	struct retimer_config_reg *config_tx;
	uint32_t config_tx_size;
	struct retimer_config_reg *config_rx;
	uint32_t config_rx_size;

	if (lm_context->speed_detection == AL_TRUE) {
		config_tx = retimer_ds25_auto_mode_tx_ch;
		config_tx_size = AL_ARR_SIZE(retimer_ds25_auto_mode_tx_ch);

		config_rx = retimer_ds25_auto_mode_rx_ch;
		config_rx_size = AL_ARR_SIZE(retimer_ds25_auto_mode_rx_ch);
	} else if (lm_context->mode == AL_ETH_LM_MODE_25G) {
		config_tx = retimer_ds25_25g_mode_tx_ch;
		config_tx_size = AL_ARR_SIZE(retimer_ds25_25g_mode_tx_ch);

		config_rx = retimer_ds25_25g_mode_rx_ch;
		config_rx_size = AL_ARR_SIZE(retimer_ds25_25g_mode_rx_ch);
	} else {
		config_tx = retimer_ds25_10g_mode;
		config_tx_size = AL_ARR_SIZE(retimer_ds25_10g_mode);

		config_rx = retimer_ds25_10g_mode;
		config_rx_size = AL_ARR_SIZE(retimer_ds25_10g_mode);
	}

	rc = al_eth_lm_retimer_ds25_channel_config(lm_context,
					lm_context->retimer_channel,
					config_rx,
					config_rx_size);

	if (rc)
		return rc;

	rc = al_eth_lm_retimer_ds25_channel_config(lm_context,
					lm_context->retimer_tx_channel,
					config_tx,
					config_tx_size);

	if (rc)
		return rc;

	if (lm_context->serdes_obj->type_get() == AL_SRDS_TYPE_25G) {
		lm_debug("%s: serdes 25G - perform tx and rx gearbox reset\n", __func__);
		al_eth_gearbox_reset(lm_context->adapter, AL_TRUE, AL_TRUE);
		al_udelay(AL_ETH_LM_GEARBOX_RESET_DELAY);
	}

	lm_debug("%s: retimer full configuration done\n", __func__);

	return rc;
}

static int al_eth_lm_retimer_ds125_config(struct al_eth_lm_context *lm_context)
{
 int rc = 0, i;
 struct retimer_config_reg *config;
 uint32_t config_size;

 config = retimer_ds125_10g_mode;
 config_size = AL_ARR_SIZE(retimer_ds125_10g_mode);

 for (i = 0; i < config_size; i++) {
       rc = al_eth_lm_retimer_ds25_write_reg(lm_context,
                                                               config[i].addr,
                                                               config[i].mask,
                                                               config[i].value);
       if (rc)
               return rc;
       }

 lm_debug("%s: retimer full configuration done\n", __func__);

 return rc;
}

static int al_eth_lm_retimer_25g_rx_adaptation_step(struct al_eth_lm_context *lm_context)
{
	al_bool lock;
	unsigned int current_time;
	int rc;

	switch (lm_context->step_data.retimer_data.rx_adap_state) {
	case LM_STEP_RETIMER_RX_ADAPTATION_RETIMER_LOCK_TO:
		rc = al_eth_lm_retimer_signal_lock_check_step(lm_context,
							lm_context->retimer_channel,
							&lm_context->retimer_rx_channel_last_status,
							AL_FALSE,
							&lock);

		if (rc == -EINPROGRESS)
			break;

		if (!lock) {
			al_dbg("%s: no signal detected on retimer Rx channel (%d)\n",
				 __func__,  lm_context->retimer_channel);

			return -EIO;
		}

		lm_context->step_data.start_time =
			lm_context->get_msec();

		lm_context->step_data.retimer_data.rx_adap_state =
				LM_STEP_RETIMER_RX_ADAPTATION_SERDES_LOCK_DELAY;
		/* fall through */
	case LM_STEP_RETIMER_RX_ADAPTATION_SERDES_LOCK_DELAY:
		current_time = lm_context->get_msec();

		al_dbg("%s: In SERDES_LOCK_DELAY. current_time:%d, start_time:%d, waiting delay:%d\n",
			 __func__, current_time, lm_context->step_data.start_time,
			 AL_ETH_LM_SERDES_WAIT_FOR_LOCK);

		if (TIMED_OUT(current_time, lm_context->step_data.start_time,
			      AL_ETH_LM_SERDES_WAIT_FOR_LOCK)) {
			lm_context->step_data.retimer_data.rx_adap_state =
				LM_STEP_RETIMER_RX_ADAPTATION_RETIMER_LOCK_TO;

			return 0;
		}
		break;
	default:
		al_err("%s: Reached undefined state: %d\n", __func__,
		       lm_context->step_data.retimer_data.rx_adap_state);
		return -EINVAL;
	}

	return -EINPROGRESS;
}

static int al_eth_lm_check_for_link(struct al_eth_lm_context *lm_context, al_bool *link_up)
{
	struct al_eth_link_status status;
	int ret = 0;
	uint16_t mii_bmsr = 0;

	al_eth_link_status_clear(lm_context->adapter);
	al_eth_link_status_get(lm_context->adapter, &status);

	if(lm_context->sfp_has_phyreg) {
		/* Read link and autonegotiation status */
		ret = al_eth_sfp_mii_read(lm_context, MII_BMSR, &mii_bmsr);
		if(ret) {
			lm_debug("%s: unable to read MII_BMSR\n", __func__);
			return ret;
		}
		status.link_up = status.link_up && (mii_bmsr & BMSR_LSTATUS);
		lm_debug("%s: >>>> PHY link state %s autoneg %s\n", __func__,
			 (mii_bmsr & BMSR_LSTATUS) ? "UP" : "DOWN",
			 (mii_bmsr & BMSR_ANEGCOMPLETE) ? "DONE" : "IN PROGRESS");
	}

	if (status.link_up == AL_TRUE) {
		lm_debug("%s: >>>> Link state DOWN ==> UP\n", __func__);
		al_eth_led_set(lm_context->adapter, AL_TRUE);
		lm_context->link_state = AL_ETH_LM_LINK_UP;
		*link_up = AL_TRUE;

		return 0;
	} else if (status.local_fault) {
		lm_context->link_state = AL_ETH_LM_LINK_DOWN;
		al_eth_led_set(lm_context->adapter, AL_FALSE);

		/* TODO -> EOSN-385
		 * if ((lm_context->mode == AL_ETH_LM_MODE_25G) && lm_context->auto_fec_enable)
		 *	lm_debug("%s: Failed to establish link\n", __func__);
		 * else
		 *	al_err("%s: Failed to establish link\n", __func__);
		 */
		ret = -1;
	} else {
		lm_debug("%s: >>>> Link state DOWN ==> DOWN_RF\n", __func__);
		lm_context->link_state = AL_ETH_LM_LINK_DOWN_RF;
		al_eth_led_set(lm_context->adapter, AL_FALSE);

		ret = 0;
	}

	*link_up = AL_FALSE;
	return ret;
}

static int al_eth_lm_link_state_check_for_detection(struct al_eth_lm_context *lm_context,
						al_bool *link_fault,
						struct al_eth_link_status *status)
{
	switch (lm_context->link_state) {
	case AL_ETH_LM_LINK_UP:
		al_eth_link_status_get(lm_context->adapter, status);

		if (status->link_up) {
			if (link_fault)
				*link_fault = AL_FALSE;

			al_eth_led_set(lm_context->adapter, AL_TRUE);

			return 0;
		} else if (status->local_fault) {
			lm_debug("%s: >>>> Link state UP ==> DOWN\n", __func__);
			lm_context->link_state = AL_ETH_LM_LINK_DOWN;
		} else {
			lm_debug("%s: >>>> Link state UP ==> DOWN_RF\n", __func__);
			lm_context->link_state = AL_ETH_LM_LINK_DOWN_RF;
		}

		break;
	case AL_ETH_LM_LINK_DOWN_RF:
		al_eth_link_status_get(lm_context->adapter, status);

		if (status->local_fault) {
			lm_debug("%s: >>>> Link state DOWN_RF ==> DOWN\n", __func__);
			lm_context->link_state = AL_ETH_LM_LINK_DOWN;

			break;
		} else if (status->remote_fault == AL_FALSE) {
			lm_debug("%s: >>>> Link state DOWN_RF ==> UP\n", __func__);
			lm_context->link_state = AL_ETH_LM_LINK_UP;
		}
		/* in case of remote fault only no need to check SFP again */
		return 0;
	case AL_ETH_LM_LINK_DOWN:
		break;
	};

	return -ENETDOWN;
}

static void al_eth_lm_auto_fec_init(struct al_eth_lm_context	*lm_context)
{
	lm_context->auto_fec_state = AL_ETH_LM_AUTO_FEC_INIT_ENABLED;
	lm_context->auto_fec_wait_to_toggle = lm_context->auto_fec_initial_timeout;
	al_eth_fec_enable(lm_context->adapter, AL_TRUE);

	al_info("%s: Auto-FEC mode enabled. FEC state initialized to be Enabled\n",
		__func__);
	lm_context->fec_state_last_time_toggled = lm_context->get_msec();
	lm_debug("Initial sys_time (msec): %u, Initial timeout: %u, Toggle timeout: %u\n",
		 lm_context->fec_state_last_time_toggled,
		 lm_context->auto_fec_initial_timeout,
		 lm_context->auto_fec_toggle_timeout);
}

static void al_eth_lm_fec_config(struct al_eth_lm_context	*lm_context)
{
	if (lm_context->mode == AL_ETH_LM_MODE_25G) {
		if (lm_context->auto_fec_enable) {
			/* Auto-FEC mode for 25G */
			if (lm_context->auto_fec_state == AL_ETH_LM_AUTO_FEC_INIT)
				al_eth_lm_auto_fec_init(lm_context);
		} else {
			/* manual control of fec for 25G */
			lm_debug("%s: manual fec enabled %d\n", __func__,
				 lm_context->local_adv.fec_capability);
			al_eth_fec_enable(lm_context->adapter,
					  lm_context->local_adv.fec_capability);
		}
	} else {
		al_eth_fec_enable(lm_context->adapter, AL_FALSE);
	}
}

#define LM_STEP_CHANGE_STATE(state, new_state)							\
	do {											\
		al_dbg("%s: CHANGING STATE from %d to %s(%d)\n", __func__, (state), (#new_state),\
			(new_state));								\
		(state) = (new_state);								\
	} while (0)										\

/*****************************************************************************/
/***************************** API functions *********************************/
/*****************************************************************************/
int al_eth_lm_get_module_info(struct al_eth_lm_context *lm_context,
		   struct ethtool_modinfo *modinfo)
{
	al_assert(lm_context);
	al_assert(modinfo);

	mutex_lock(&lm_context->lock);
	if (lm_context->sfp_id.ext.sff8472_compliance &&
	    !(lm_context->sfp_id.ext.diagmon & SFP_DIAGMON_ADDRMODE)) {
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}
	mutex_unlock(&lm_context->lock);

	return 0;
}

int al_eth_lm_get_module_eeprom(struct al_eth_lm_context *lm_context,
		struct ethtool_eeprom *eeprom, uint8_t *data) {

	unsigned int first, last, len;
	int rc;

	al_assert(lm_context);
	al_assert(eeprom);
	al_assert(data);
	al_assert(lm_context->i2c_read_data);

	if (eeprom->len == 0)
		return -EINVAL;

	memset(data, 0, eeprom->len);

	first = eeprom->offset;
	last = eeprom->offset + eeprom->len;
	if (first < ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8079_LEN);
		len -= first;
		rc = lm_context->i2c_read_data(lm_context->i2c_context, lm_context->sfp_bus_id,
					       SFP_I2C_ADDR, first, data, len);
		if (rc)
			return rc;

		first += len;
		data += len;
	}
	if (first < ETH_MODULE_SFF_8472_LEN && last > ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8472_LEN);
		len -= first;
		first -= ETH_MODULE_SFF_8079_LEN;
		rc = lm_context->i2c_read_data(lm_context->i2c_context, lm_context->sfp_bus_id,
					       SFP_I2C_ADDR_A2, first, data, len);
		if (rc)
			return rc;
	}

	return 0;
}

int al_eth_lm_init(struct al_eth_lm_context	*lm_context,
		   struct al_eth_lm_init_params	*params)
{
	lm_context->adapter = params->adapter;
	lm_context->serdes_obj = params->serdes_obj;
	lm_context->lane = params->lane;
	lm_context->sfp_detection = params->sfp_detection;
	lm_context->sfp_bus_id = params->sfp_bus_id;
	lm_context->sfp_i2c_addr = params->sfp_i2c_addr;

	lm_context->retimer_exist = params->retimer_exist;
	lm_context->retimer_type = params->retimer_type;
	lm_context->retimer_bus_id = params->retimer_bus_id;
	lm_context->retimer_i2c_addr = params->retimer_i2c_addr;
	lm_context->retimer_channel = params->retimer_channel;
	lm_context->retimer_tx_channel = params->retimer_tx_channel;

	lm_context->default_mode = params->default_mode;
	lm_context->default_dac_len = params->default_dac_len;

	/* enable 10/1g by default */
	params->sfp_probe_1g = true;
	params->sfp_probe_10g = true;

	lm_context->sfp_probe_1g = params->sfp_probe_1g;
	lm_context->sfp_probe_10g = params->sfp_probe_10g;
	lm_context->link_training = params->link_training;
	lm_context->rx_equal = params->rx_equal;
	lm_context->static_values = params->static_values;
	lm_context->i2c_read_data = params->i2c_read_data;
	lm_context->i2c_read = params->i2c_read;
	lm_context->i2c_write_data = params->i2c_write_data;
	lm_context->i2c_write = params->i2c_write;
	lm_context->i2c_context = params->i2c_context;
	lm_context->get_random_byte = params->get_random_byte;

	/* eeprom_read must be provided if sfp_detection is true */
	al_assert((lm_context->sfp_detection == AL_FALSE) ||
		  (lm_context->i2c_read != NULL));

	al_assert((lm_context->retimer_exist == AL_FALSE) ||
		  (lm_context->i2c_write != NULL));

	lm_context->local_adv.selector_field = 1;
	lm_context->local_adv.capability = 0;
	lm_context->local_adv.remote_fault = 0;
	lm_context->local_adv.acknowledge = 0;
	lm_context->local_adv.next_page = 0;
	lm_context->local_adv.technology = AL_ETH_AN_TECH_10GBASE_KR;
	lm_context->local_adv.fec_capability = params->kr_fec_enable;

	lm_context->mode = AL_ETH_LM_MODE_DISCONNECTED;
	lm_context->serdes_tx_params_valid = AL_FALSE;
	lm_context->serdes_rx_params_valid = AL_FALSE;

	lm_context->rx_param_dirty = 1;
	lm_context->tx_param_dirty = 1;

	lm_context->gpio_get = params->gpio_get;
	lm_context->gpio_present = params->gpio_present;

	lm_context->max_speed = params->max_speed;
	lm_context->sfp_detect_force_mode = params->sfp_detect_force_mode;
	lm_context->speed_detection = params->speed_detection;

	lm_context->lm_pause = params->lm_pause;

	lm_context->led_config = params->led_config;

	lm_context->retimer_configured = AL_FALSE;

	lm_context->retimer_rx_channel_last_status.signal_detect = AL_FALSE;
	lm_context->retimer_rx_channel_last_status.cdr_lock = AL_FALSE;
	lm_context->retimer_rx_channel_last_status.first_check = AL_TRUE;

	lm_context->retimer_tx_channel_last_status.signal_detect = AL_FALSE;
	lm_context->retimer_tx_channel_last_status.cdr_lock = AL_FALSE;
	lm_context->retimer_tx_channel_last_status.first_check = AL_TRUE;

	lm_context->link_state = AL_ETH_LM_LINK_DOWN;

	al_assert(params->get_msec);
	lm_context->get_msec = params->get_msec;

	lm_context->auto_fec_enable = params->auto_fec_enable;
	lm_context->auto_fec_initial_timeout =
		params->auto_fec_initial_timeout ?
		params->auto_fec_initial_timeout : AUTO_FEC_INITIAL_TIMEOUT_DEFAULT;
	lm_context->auto_fec_toggle_timeout =
		params->auto_fec_toggle_timeout ?
		params->auto_fec_toggle_timeout : AUTO_FEC_TOGGLE_TIMEOUT_DEFAULT;
	lm_context->auto_fec_state = AL_ETH_LM_AUTO_FEC_INIT;
	lm_context->link_prev_up = AL_FALSE;

	lm_context->speed_change = AL_FALSE;
	lm_context->last_detected_mode = AL_ETH_LM_MODE_DISCONNECTED;

	lm_context->step_data.detection_state = LM_STEP_DETECTION_INIT;
	lm_context->step_data.establish_state = LM_STEP_ESTABLISH_INIT;
	lm_context->step_data.retimer_data.rx_adap_state =
		LM_STEP_RETIMER_RX_ADAPTATION_RETIMER_LOCK_TO;
	lm_context->step_data.retimer_data.slc_state = LM_STEP_RETIMER_SIGNAL_LOCK_INIT;
	lm_context->step_data.retimer_data.wfl_state = LM_STEP_RETIMER_WAIT_FOR_LOCK_INIT;

	memset(&lm_context->link_conf, 0, sizeof(lm_context->link_conf));
	mutex_init(&lm_context->lock);
	lm_context->sfp_has_phyreg = AL_FALSE;

	return 0;
}

/*NOTICE: need to preserve old_mode and new_mode until return value is no longer -EINPROGRESS*/
int al_eth_lm_link_detection_step(struct al_eth_lm_context	*lm_context,
			     al_bool			*link_fault,
			     enum al_eth_lm_link_mode	*old_mode,
			     enum al_eth_lm_link_mode	*new_mode)
{
	int rc;
	int ret = -EIO;
	struct al_eth_link_status status;

	al_assert(lm_context != NULL);
	al_assert(old_mode != NULL);
	al_assert(new_mode != NULL);

	if (link_fault)
		*link_fault = AL_TRUE;

	switch (lm_context->step_data.detection_state) {
	case LM_STEP_DETECTION_INIT:

		/**
		 * if Link management is disabled, report no link fault in case the link was up
		 * before and set new mode to disconnected to avoid calling to link establish
		 * if the link wasn't up.
		 */
		if (lm_context->lm_pause) {
			al_bool lm_pause = lm_context->lm_pause(lm_context->i2c_context);
			if (lm_pause == AL_TRUE) {
				*new_mode = AL_ETH_LM_MODE_DISCONNECTED;
				if (link_fault) {
					if (lm_context->link_state == AL_ETH_LM_LINK_UP)
						*link_fault = AL_FALSE;
					else
						*link_fault = AL_TRUE;
				}
				return 0;
			}
		}

		*old_mode = lm_context->mode;
		*new_mode = lm_context->mode;

		rc = al_eth_lm_link_state_check_for_detection(lm_context, link_fault, &status);
		if (!rc)
			return 0;


		al_eth_led_set(lm_context->adapter, AL_FALSE);

		if (lm_context->link_state == AL_ETH_LM_LINK_DOWN) {
			if (lm_context->sfp_detection) {
				rc = al_eth_module_detect(lm_context, new_mode);
				if (rc) {
					al_err("module_detection failed!\n");
					goto exit_error;
				}

				lm_context->mode = *new_mode;

				if (*new_mode == AL_ETH_LM_MODE_DISCONNECTED &&
				    lm_context->auto_fec_enable) {
					lm_context->auto_fec_state = AL_ETH_LM_AUTO_FEC_INIT;
					lm_context->link_prev_up = AL_FALSE;
				}
			} else {
				lm_context->mode = lm_context->default_mode;
				*new_mode = lm_context->mode;
			}
		}

		if ((lm_context->link_state == AL_ETH_LM_LINK_DOWN) &&
		    (lm_context->retimer_exist) &&
		    (*new_mode != AL_ETH_LM_MODE_DISCONNECTED)) {
			if (*old_mode != *new_mode) {
				lm_context->rx_param_dirty = 1;
				lm_context->tx_param_dirty = 1;

				al_eth_serdes_static_rx_params_set(lm_context);
				al_eth_serdes_static_tx_params_set(lm_context);

				if (retimer[lm_context->retimer_type].config(lm_context)) {
					al_info("%s: failed to configure the retimer\n", __func__);
					rc = -EIO;
					goto exit_error;
				}

				al_udelay(AL_ETH_LM_RETIMER_CONFIG_DELAY);
			}

			if (lm_context->speed_detection) {
				if (retimer[lm_context->retimer_type].speed_get == NULL) {
					al_info("Speed detection not supported in the retimer\n");
					rc = -EOPNOTSUPP;
					goto exit_error;
				}

				if (retimer[lm_context->retimer_type].rx_adaptation)
					LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state,
							     LM_STEP_DETECTION_RETIMER_RX_ADAPT_WAIT
						);
				else
					LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state,
							     LM_STEP_DETECTION_RETIMER_RX_ADAPT_POST
						);
				break;
			}
		}

		LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state, LM_STEP_DETECTION_FINAL)
			;
		break;
	case LM_STEP_DETECTION_RETIMER_RX_ADAPT_WAIT:
		ret = retimer[lm_context->retimer_type].rx_adaptation(lm_context);

		al_dbg("%s: post rx_adaptation. ret:%d\n", __func__, ret);

		if (ret == -EINPROGRESS)
			break;

		LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state,
				     LM_STEP_DETECTION_RETIMER_RX_ADAPT_POST);
		/* fall through */
	case LM_STEP_DETECTION_RETIMER_RX_ADAPT_POST:
		if (ret) {
			al_dbg("%s: Rx channel is not locked\n", __func__);
		} else {
			int speed = 0;

			speed = retimer[lm_context->retimer_type].speed_get(
				lm_context,
				lm_context->retimer_channel);

			if (speed == SPEED_DETECTION_10G) {
				if (lm_context->last_detected_mode ==
					AL_ETH_LM_MODE_25G)
					lm_context->speed_change = AL_TRUE;

				*new_mode = AL_ETH_LM_MODE_10G_DA;
				lm_context->mode = AL_ETH_LM_MODE_10G_DA;
			} else {
				if (lm_context->last_detected_mode !=
					AL_ETH_LM_MODE_25G)
					lm_context->speed_change = AL_TRUE;

				*new_mode = AL_ETH_LM_MODE_25G;
				lm_context->mode = AL_ETH_LM_MODE_25G;
			}
		}

		if (lm_context->mode != AL_ETH_LM_MODE_DISCONNECTED)
			lm_context->last_detected_mode = lm_context->mode;

		LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state,
				     LM_STEP_DETECTION_FINAL);
		/* fall through */
	case LM_STEP_DETECTION_FINAL:
		if (*old_mode != *new_mode) {
			al_info("%s: New SFP mode detected %s -> %s\n",
				__func__, al_eth_lm_mode_convert_to_str(*old_mode),
				al_eth_lm_mode_convert_to_str(*new_mode));

			if ((*new_mode != AL_ETH_LM_MODE_DISCONNECTED) &&
			    (lm_context->led_config)) {
				struct al_eth_lm_led_config_data data = {0};

				switch (*new_mode) {
				case AL_ETH_LM_MODE_10G_OPTIC:
				case AL_ETH_LM_MODE_10G_DA:
					data.speed = AL_ETH_LM_LED_CONFIG_10G;
					break;
				case AL_ETH_LM_MODE_1G:
				case AL_ETH_LM_MODE_1G_DA:
					data.speed = AL_ETH_LM_LED_CONFIG_1G;
					break;
				case AL_ETH_LM_MODE_25G:
					data.speed = AL_ETH_LM_LED_CONFIG_25G;
					break;
				default:
					al_err("%s: unknown LM mode!\n", __func__);
				};

				lm_context->led_config(lm_context->i2c_context, &data);
			}
		}

		LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state, LM_STEP_DETECTION_INIT);
		return 0;
	default:
		al_err("%s: Undefined lm_step_destection_state: %d\n", __func__,
		       lm_context->step_data.detection_state);
		rc = -EINVAL;
		goto exit_error;
	}

	return -EINPROGRESS;

exit_error:
	LM_STEP_CHANGE_STATE(lm_context->step_data.detection_state, LM_STEP_DETECTION_INIT);
	lm_debug("%s: Exiting with error %d\n", __func__, rc);
	return rc;
}

int al_eth_lm_link_detection(struct al_eth_lm_context	*lm_context,
			     al_bool			*link_fault,
			     enum al_eth_lm_link_mode	*old_mode,
			     enum al_eth_lm_link_mode	*new_mode)
{
	int rc;

	while (1) {
		rc = al_eth_lm_link_detection_step(lm_context, link_fault, old_mode, new_mode);
		if (rc != -EINPROGRESS)
			break;
		al_udelay(LM_STEP_LOOP_INTERVAL);
	}

	return rc;
}

int al_eth_lm_link_establish_step(struct al_eth_lm_context	*lm_context,
				  al_bool			*link_up)
{
	al_bool signal_detected;
	int ret = 0;
	int rc;
	al_bool lock;
	unsigned int current_time = 0;

	switch (lm_context->step_data.establish_state) {
	case LM_STEP_ESTABLISH_INIT:
		switch (lm_context->link_state) {
		case AL_ETH_LM_LINK_UP:
			*link_up = AL_TRUE;
			lm_debug("%s: return link up\n", __func__);

			return 0;
		case AL_ETH_LM_LINK_DOWN_RF:
			*link_up = AL_FALSE;
			lm_debug("%s: return link down (DOWN_RF)\n", __func__);

			return 0;
		case AL_ETH_LM_LINK_DOWN:
			break;
		};

		/**
		 * At this point we will get LM disable only if changed to disable after link
		 * detection finished. in this case link will not be established until LM will
		 * be enable again.
		 */
		if (lm_context->lm_pause) {
			al_bool lm_pause = lm_context->lm_pause(lm_context->i2c_context);
			if (lm_pause == AL_TRUE) {
				*link_up = AL_FALSE;

				return 0;
			}
		}

		if (lm_context->retimer_exist)
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_RETIMER_SIGNAL_LOCK_CHECK);
		else
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_FINAL);

		break;
	case LM_STEP_ESTABLISH_RETIMER_SIGNAL_LOCK_CHECK:
		rc = al_eth_lm_retimer_signal_lock_check_step(lm_context,
							lm_context->retimer_tx_channel,
							&lm_context->retimer_tx_channel_last_status,
							AL_FALSE,
							&lock);

		al_dbg("%s: post signal_lock_check_step. rc:%d, lock:%d\n",
		       __func__, rc, lock);

		if (rc == -EINPROGRESS)
			break;

		if (!lock) {
			al_info("%s: Failed to lock tx channel\n", __func__);
			rc = -EIO;
			goto exit_error;
		}

		if (lm_context->speed_change) {
			lm_debug("%s: serdes 25G - perform tx gearbox reset\n", __func__);
			al_eth_gearbox_reset(lm_context->adapter, AL_TRUE, AL_FALSE);
			lm_context->speed_change = AL_FALSE;
		}

		if (retimer[lm_context->retimer_type].rx_adaptation)
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_RETIMER_RX_ADAPT_WAIT);
		else
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_MIDDLE);

		break;
	case LM_STEP_ESTABLISH_RETIMER_RX_ADAPT_WAIT:
		ret = retimer[lm_context->retimer_type].rx_adaptation(lm_context);

		al_dbg("%s: post rx_adaptation. ret:%d\n", __func__, ret);

		if (ret == -EINPROGRESS) {
			*link_up = AL_FALSE;
			break;
		}

		if (ret) {
			al_dbg("%s: retimer rx is not ready\n", __func__);
			*link_up = AL_FALSE;

			rc = ret;
			goto exit_error;
		}

		LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
				     LM_STEP_ESTABLISH_MIDDLE);
		/* fall through */
	case LM_STEP_ESTABLISH_MIDDLE:
		signal_detected = lm_context->serdes_obj->signal_is_detected(
			lm_context->serdes_obj,
			lm_context->lane);

		if (signal_detected == AL_FALSE) {
			/* if no signal detected there is nothing to do */
			lm_debug("serdes signal is down\n");
			*link_up = AL_FALSE;

			rc = -EIO;
			goto exit_error;
		}

		if (lm_context->serdes_obj->type_get() == AL_SRDS_TYPE_25G) {
			if (lm_context->link_training == AL_FALSE)
				al_eth_lm_fec_config(lm_context);

			lm_debug("%s: serdes 25G - perform rx gearbox reset\n", __func__);
			al_eth_gearbox_reset(lm_context->adapter, AL_FALSE, AL_TRUE);
			al_udelay(AL_ETH_LM_GEARBOX_RESET_DELAY);
		}

		if (lm_context->retimer_exist) {
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_RETIMER_EXISTS_LINK_STATUS_DELAY);
			lm_context->step_data.start_time = lm_context->get_msec();
		} else {
			LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state,
					     LM_STEP_ESTABLISH_FINAL);
		}
		break;
	case LM_STEP_ESTABLISH_RETIMER_EXISTS_LINK_STATUS_DELAY:
		current_time = lm_context->get_msec();

		al_dbg("%s: step_state:%d, current_time:%d, start_time:%d, waiting delay: %d\n",
		       __func__, lm_context->step_data.establish_state, current_time,
		       lm_context->step_data.start_time,
		       AL_ETH_LM_RETIMER_LINK_STATUS_DELAY);

		if (!TIMED_OUT(current_time, lm_context->step_data.start_time,
			       AL_ETH_LM_RETIMER_LINK_STATUS_DELAY))
			break;

		ret = al_eth_lm_check_for_link(lm_context, link_up);

		if (ret == 0) {
			lm_debug("%s: link is %s with retimer\n", __func__,
				(link_up ? "UP" : "DOWN_RF"));
			if ((lm_context->mode == AL_ETH_LM_MODE_25G) &&
				lm_context->auto_fec_enable) {
				if (lm_context->auto_fec_state == AL_ETH_LM_AUTO_FEC_INIT_ENABLED) {
					/* Case when link went up during initial toggling cycle */
					lm_context->auto_fec_state = AL_ETH_LM_AUTO_FEC_ENABLED;
					lm_context->auto_fec_wait_to_toggle =
						lm_context->auto_fec_toggle_timeout;
				}
				lm_context->link_prev_up = AL_TRUE;
				lm_debug("%s: Auto FEC state is %s\n",
					__func__, lm_context->auto_fec_state ?
					"Enabled" : "Disabled");
			}

			goto exit_done;
		}

		if ((lm_context->mode == AL_ETH_LM_MODE_25G) && lm_context->auto_fec_enable) {
			unsigned int new_time = lm_context->get_msec();
			if (lm_context->link_prev_up == AL_TRUE) {
				/* Case when link just went down - want to preserve the FEC
				 * state */
				lm_context->link_prev_up = AL_FALSE;
				lm_context->fec_state_last_time_toggled = new_time;
				lm_debug("%s: Link went DOWN. Starting Auto FEC toggling in %s state\n",
					__func__, (lm_context->auto_fec_state ==
						AL_ETH_LM_AUTO_FEC_ENABLED) ?
					"Enabled" : "Disabled");
			} else {
				if ((new_time - lm_context->fec_state_last_time_toggled) >=
					lm_context->auto_fec_wait_to_toggle) {
					/* If new_time wraps around,the calculation will
					 * still be correct, since LHS will be modulo
					 * reduced and get_msec() returns time which wraps
					 * around full range of uint
					 */
					al_bool fec_enabled_bool;
					lm_context->fec_state_last_time_toggled = new_time;
					switch (lm_context->auto_fec_state) {
					case AL_ETH_LM_AUTO_FEC_INIT_ENABLED:
						lm_context->auto_fec_wait_to_toggle =
							lm_context->auto_fec_toggle_timeout;
					case AL_ETH_LM_AUTO_FEC_ENABLED:
						lm_context->auto_fec_state =
							AL_ETH_LM_AUTO_FEC_DISABLED;
						fec_enabled_bool = AL_FALSE;
						break;
					case AL_ETH_LM_AUTO_FEC_DISABLED:
						lm_context->auto_fec_state =
							AL_ETH_LM_AUTO_FEC_ENABLED;
						fec_enabled_bool = AL_TRUE;
						break;
					default:
						al_err("%s: Invalid Auto FEC state value: %d\n",
						       __func__, lm_context->auto_fec_state);
						rc = -EINVAL;
						goto exit_error;
					}
					al_eth_fec_enable(lm_context->adapter,
							  fec_enabled_bool);
					al_info("%s: Auto FEC state is %s\n", __func__,
						fec_enabled_bool ? "Enabled" : "Disabled");
				}
			}
		}
		rc = ret;
		goto exit_error;

	case LM_STEP_ESTABLISH_FINAL:

		if ((lm_context->mode == AL_ETH_LM_MODE_10G_DA) && (lm_context->link_training)) {
			lm_context->local_adv.transmitted_nonce = lm_context->get_random_byte();
			lm_context->local_adv.transmitted_nonce &= 0x1f;

			ret = al_eth_an_lt_execute(lm_context->adapter,
						lm_context->serdes_obj,
						lm_context->lane,
						&lm_context->local_adv,
						&lm_context->partner_adv);

			lm_context->rx_param_dirty = 1;
			lm_context->tx_param_dirty = 1;

			if (ret == 0) {
				al_info("%s: link training finished successfully\n", __func__);
				lm_context->link_training_failures = 0;
				ret = al_eth_lm_check_for_link(lm_context, link_up);

				if (ret == 0) {
					lm_debug("%s: link is up with LT\n", __func__);

					goto exit_done;
				}
			}

			lm_context->link_training_failures++;
			if (lm_context->link_training_failures > AL_ETH_LT_FAILURES_TO_RESET) {
				lm_debug("%s: failed to establish LT %d times. reset serdes\n",
					 __func__, AL_ETH_LT_FAILURES_TO_RESET);

				lm_context->serdes_obj->pma_hard_reset_lane(
					lm_context->serdes_obj,
					lm_context->lane,
					AL_TRUE);
				lm_context->serdes_obj->pma_hard_reset_lane(
					lm_context->serdes_obj,
					lm_context->lane,
					AL_FALSE);
				lm_context->link_training_failures = 0;
			}
		}

		if(lm_context->mode == AL_ETH_LM_MODE_1G_DA) {
			lm_context->rx_param_dirty = 1;
			lm_context->tx_param_dirty = 1;
		}

		al_eth_serdes_static_tx_params_set(lm_context);

		if ((lm_context->mode == AL_ETH_LM_MODE_10G_DA) && (lm_context->rx_equal)) {
			ret = al_eth_rx_equal_run(lm_context);

			if (ret == 0) {
				al_udelay(AL_ETH_LM_LINK_STATUS_DELAY);
				ret = al_eth_lm_check_for_link(lm_context, link_up);

				if (ret == 0) {
					lm_debug("%s: link is up with Rx Equalization\n", __func__);
					goto exit_done;
				}
			}
		}

		al_eth_serdes_static_rx_params_set(lm_context);

		al_udelay(AL_ETH_LM_LINK_STATUS_DELAY);

		ret = al_eth_lm_check_for_link(lm_context, link_up);

		if (ret == 0) {
			lm_debug("%s: link is up with static parameters\n", __func__);

			goto exit_done;
		}

		*link_up = AL_FALSE;
		rc = -1;
		goto exit_error;
	default:
		al_err("%s: Undefined lm_step_establish_state: %d\n", __func__,
		       lm_context->step_data.establish_state);
		*link_up = AL_FALSE;
		rc = -EINVAL;
		goto exit_error;
	};

	return -EINPROGRESS;

exit_done:
	LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state, LM_STEP_ESTABLISH_INIT);
	return 0;

exit_error:
	LM_STEP_CHANGE_STATE(lm_context->step_data.establish_state, LM_STEP_ESTABLISH_INIT);
	al_dbg("%s: Exiting with error %d\n", __func__, rc);
	return rc;
}

int al_eth_lm_link_establish(struct al_eth_lm_context	*lm_context,
			     al_bool			*link_up)
{
	int rc;

	while (1) {
		rc = al_eth_lm_link_establish_step(lm_context, link_up);
		if (rc != -EINPROGRESS)
			break;
		al_udelay(LM_STEP_LOOP_INTERVAL);
	}

	return rc;
}

int al_eth_lm_static_parameters_override(struct al_eth_lm_context	*lm_context,
					 struct al_serdes_adv_tx_params *tx_params,
					 struct al_serdes_adv_rx_params *rx_params)
{
	if (tx_params) {
		lm_context->tx_params_override = *tx_params;
		lm_context->tx_param_dirty = 1;
		lm_context->serdes_tx_params_valid = AL_TRUE;
	}

	if (rx_params) {
		lm_context->rx_params_override = *rx_params;
		lm_context->rx_param_dirty = 1;
		lm_context->serdes_rx_params_valid = AL_TRUE;
	}

	return 0;
}

int al_eth_lm_static_parameters_override_disable(
					struct al_eth_lm_context *lm_context,
					al_bool			 tx_params,
					al_bool			 rx_params)
{
	if (tx_params)
		lm_context->serdes_tx_params_valid = AL_FALSE;
	if (rx_params)
		lm_context->serdes_tx_params_valid = AL_FALSE;

	return 0;
}

int al_eth_lm_static_parameters_get(struct al_eth_lm_context	*lm_context,
				    struct al_serdes_adv_tx_params *tx_params,
				    struct al_serdes_adv_rx_params *rx_params)
{
	if (tx_params) {
		if (lm_context->serdes_tx_params_valid)
			*tx_params = lm_context->tx_params_override;
		else
			lm_context->serdes_obj->tx_advanced_params_get(
							lm_context->serdes_obj,
							lm_context->lane,
							tx_params);
	}

	if (rx_params) {
		if (lm_context->serdes_rx_params_valid)
			*rx_params = lm_context->rx_params_override;
		else
			lm_context->serdes_obj->rx_advanced_params_get(
							lm_context->serdes_obj,
							lm_context->lane,
							rx_params);
	}

	return 0;
}

const char *al_eth_lm_mode_convert_to_str(enum al_eth_lm_link_mode val)
{
	switch (val) {
	case AL_ETH_LM_MODE_DISCONNECTED:
		return "AL_ETH_LM_MODE_DISCONNECTED";
	case AL_ETH_LM_MODE_10G_OPTIC:
		return "AL_ETH_LM_MODE_10G_OPTIC";
	case AL_ETH_LM_MODE_10G_DA:
		return "AL_ETH_LM_MODE_10G_DA";
	case AL_ETH_LM_MODE_1G:
		return "AL_ETH_LM_MODE_1G";
	case AL_ETH_LM_MODE_1G_DA:
		return "AL_ETH_LM_MODE_1G_DA";
	case AL_ETH_LM_MODE_25G:
		return "AL_ETH_LM_MODE_25G";
	}

	return "N/A";
}

void al_eth_lm_debug_mode_set(struct al_eth_lm_context	*lm_context,
			      al_bool			enable)
{
	lm_context->debug = enable;
}


al_bool al_eth_lm_retimer_tx_cdr_lock_get(struct al_eth_lm_context *lm_context)
{
	al_bool lock = AL_FALSE;

	if (lm_context->retimer_exist && retimer[lm_context->retimer_type].cdr_lock) {
		lock = retimer[lm_context->retimer_type].cdr_lock(lm_context,
				lm_context->retimer_tx_channel);
	}

	return lock;
}

enum al_eth_lm_link_state al_eth_lm_link_state_get(struct al_eth_lm_context *lm_context)
{
	return lm_context->link_state;
}

int al_eth_lm_link_check(struct al_eth_lm_context *lm_context,
			enum al_eth_lm_link_state *link_state)
{
	struct al_eth_link_status status;
	int rc;

	rc = al_eth_link_status_get(lm_context->adapter, &status);
	if (rc) {
		al_err("%s: Error getting link status from MAC\n", __func__);
		return rc;
	}

	if (status.link_up)
		*link_state = AL_ETH_LM_LINK_UP;
	else if (status.local_fault)
		*link_state = AL_ETH_LM_LINK_DOWN;
	else if (status.remote_fault)
		*link_state = AL_ETH_LM_LINK_DOWN_RF;
	else {
		al_err("%s: Invalid link state!\n", __func__);
		return -EIO;
	}

	al_dbg("%s: link_state is %s\n", __func__,
		((*link_state == AL_ETH_LM_LINK_DOWN) ? "DOWN" :
		((*link_state == AL_ETH_LM_LINK_DOWN_RF) ? "DOWN_RF" :
		((*link_state == AL_ETH_LM_LINK_UP) ? "UP" : "UNKNOWN"))));

	return 0;
}

al_bool al_eth_lm_pause_check(struct al_eth_lm_context *lm_context)
{
	if (lm_context->lm_pause)
		return lm_context->lm_pause(lm_context->i2c_context);

	return AL_FALSE;
}
