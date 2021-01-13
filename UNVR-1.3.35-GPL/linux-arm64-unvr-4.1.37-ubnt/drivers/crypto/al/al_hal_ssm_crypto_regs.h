/*******************************************************************************
Copyright (C) 2016 Annapurna Labs Ltd.

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
 * @file   al_hal_ssm_crypto_regs.h
 *
 * @brief CRYPTO_Accelerator registers
 *
 */

#ifndef __AL_HAL_CRYPTO_REGS_H__
#define __AL_HAL_CRYPTO_REGS_H__

#include "al_hal_plat_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unit Registers
 */

struct al_crypto_accelerator_configuration {
	/* [0x0] Unit Configuration register */
	uint32_t unit_conf;
	/*
	 * [0x4] Unit Debug Configuration register
	 * Number of cycles to wait before declaring timeout
	 */
	uint32_t m2s_timeout;
	/*
	 * [0x8] Unit Debug Configuration register
	 * Number of cycles to wait before declaring a timeout
	 */
	uint32_t s2m_timeout;
	/* [0xc] enable memories for APB access (production testing / accelerated boot) */
	uint32_t mem_test;
};

struct al_crypto_accelerator_debug {
	/* [0x0] Unit Debug Configuration register */
	uint32_t unit_debug_conf;
	uint32_t rsrvd[3];
};

struct al_crypto_accelerator_log {
	/*
	 * [0x0] Descriptor word 0
	 * When a fatal error occurs, this register reflects the descriptor of the command that
	 * caused the fatal error.
	 * Descriptor Word 0
	 * When a fatal error occurs, this register reflects the descriptor of the command that
	 * caused the fatal error
	 */
	uint32_t desc_word0;
	/*
	 * [0x4] Descriptor word 1
	 * When a fatal error occurs, this register reflects the descriptor of the command that
	 * caused the fatal error.
	 * Descriptor Word 1
	 * When a fatal error occurs, this register reflects the descriptor of the command that
	 * caused the fatal error.
	 */
	uint32_t desc_word1;
	/*
	 * [0x8] Transaction information of the command that triggered the error. When a fatal error
	 * occurs, this register reflects the transaction info of the command that caused the fatal
	 * error.
	 */
	uint32_t trans_info_1;
	/*
	 * [0xc] Transaction information of the command that triggered the error. When a fatal error
	 * occurs, this register reflects the transaction info of the command that caused the fatal
	 * error.
	 */
	uint32_t trans_info_2;
	uint32_t rsrvd[4];
};

struct al_crypto_accelerator_crypto_perf_counter {
	/*
	 * [0x0] The execution cycle counter measures the number of cycles that the CRYPTO
	 * accelerator is active (i.e., there is at least one valid command from the M2S or the pipe
	 * is not empty).
	 * CRYPTO_exec_cnt
	 */
	uint32_t exec_cnt;
	/*
	 * [0x4] M2S active cycles counter
	 * Measures the number of cycles M2S sends command(s) to CRYPTO engine.
	 * M2S_active_cnt
	 */
	uint32_t m2s_active_cnt;
	/*
	 * [0x8] M2S idle cycles counter
	 * Measures the number of idle cycles on M2S while the CRYPTO accelerator is waiting for
	 * data (the M2S_active_cnt is counting).
	 * M2S_Idle_cnt
	 */
	uint32_t m2s_idle_cnt;
	/*
	 * [0xc] M2S backpressure cycles counter
	 * Measures the number of cycles the Crypto engine cannot accept data from the M2S while the
	 * M2S is ready to transfer data.
	 * M2S_bp_cnt
	 */
	uint32_t m2s_backp_cnt;
	/*
	 * [0x10] S2M active cycles counter
	 * Measures the number of cycles the Crypto engine sends command(s) to S2M. In cycles where
	 * more than one GDMA S2M is active, the counter is incremented by the number of the S2M
	 * interfaces that are active.
	 * S2M_active_cnt
	 */
	uint32_t s2m_active_cnt;
	/*
	 * [0x14] S2M idle cycles counter
	 * Measures the number of idle cycles on S2M while S2M is waiting for data (the
	 * S2M_active_cnt is counting). In cycles where more than one GDMA is waiting for data from
	 * the CRYPTO, the counter is incremented by the number of S2M interfaces that are idle.
	 * S2M_Idle_cnt
	 */
	uint32_t s2m_idle_cnt;
	/*
	 * [0x18] S2M backpressure cycles counter: Measures the number of cycles where the Crypto
	 * had data to send to the S2M, but it was not sent due to backpressure. In cycles where
	 * more than one S2M performs backpressure to the CRYPTO, the counter is incremented by the
	 * number of S2M interfaces that perform backpressure.
	 * S2M_backp_cnt
	 */
	uint32_t s2m_backp_cnt;
	/*
	 * [0x1c] Crypto Command Done Counter
	 * Total number of CRYPTO commands executed
	 * CRYPTO_Command_dn_cnt
	 */
	uint32_t cmd_dn_cnt;
	/*
	 * [0x20] CRYPTO Source Blocks Counter
	 * Total number of Source Blocks read in CRYPTO commands
	 * CRYPTO_src_blocks_cnt
	 */
	uint32_t src_blocks_cnt;
	/*
	 * [0x24] CRYPTO Destination Blocks Counter
	 * Total number of Destination Blocks written in CRYPTO commands
	 * CRYPTO_dst_blocks_cnt
	 */
	uint32_t dst_blocks_cnt;
	/*
	 * [0x28] Recoverable Errors counter
	 * Total number of recoverable errors
	 * Recover_Errors_cnt
	 */
	uint32_t recover_err_cnt;
	/*
	 * [0x2c] Counts the number of data beats entering CRYPTO.
	 * CRYPTO_src_data_beats
	 */
	uint32_t src_data_beats;
	/*
	 * [0x30] Counts the number of the data beats exiting CRYPTO.
	 * CRYPTO_dst_data_beats
	 */
	uint32_t dst_data_beats;
	uint32_t rsrvd[7];
};

struct al_crypto_accelerator_perfm_cnt_cntl {
	/* [0x0] Performance counter control */
	uint32_t conf;
};

struct al_crypto_accelerator_crypto_status {
	/* [0x0] Crypto pipe status */
	uint32_t status;
};

struct al_crypto_accelerator_crypto_version {
	/* [0x0] Crypto Version */
	uint32_t revision;
	/* [0x4] Crypto Version */
	uint32_t date;
};

struct al_crypto_accelerator_crypto_algorithms {
	/* [0x0] Crypto Algorithms */
	uint32_t algorithms;
};

struct al_crypto_accelerator_reserved {
	/* [0x0] Reserved */
	uint32_t reserved_0;
	/* [0x4] Reserved */
	uint32_t reserved_1;
	/* [0x8] Reserved */
	uint32_t reserved_2;
	/* [0xc] Reserved */
	uint32_t reserved_3;
};

struct al_crypto_accelerator_sha3_round_consts {
	/* [0x0] round consts msbs */
	uint32_t hi;
	/* [0x4] round consts lsbs */
	uint32_t lo;
};

struct al_crypto_accelerator_sha3_params {
	/* [0x0] */
	uint32_t num_rounds;
	/*
	 * [0x4] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_sha3_224;
	/*
	 * [0x8] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_sha3_256;
	/*
	 * [0xc] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_sha3_384;
	/*
	 * [0x10] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_sha3_512;
	/*
	 * [0x14] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_shake_128;
	/*
	 * [0x18] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_shake_256;
	/*
	 * [0x1c] sponge function bitrate. This value determines the security level and performance
	 * of the hash function (defined by standard).
	 */
	uint32_t bitrate_default;
	/* [0x20] */
	uint32_t padding_sha3;
	/* [0x24] */
	uint32_t padding_shake;
	/* [0x28] */
	uint32_t endianity;
	uint32_t rsrvd;
};

struct al_crypto_accelerator_sha3_debug {
	/* [0x0] */
	uint32_t fsm_state;
	uint32_t rsrvd[3];
};

struct al_crypto_accelerator_xts_conf {
	/* [0x0] lba increment factor, for multiple concatenated blocks in a single pkt (lsbs) */
	uint32_t lba_inc_0;
	/* [0x4] lba increment factor, for multiple concatenated blocks in a single pkt */
	uint32_t lba_inc_1;
	/* [0x8] lba increment factor, for multiple concatenated blocks in a single pkt */
	uint32_t lba_inc_2;
	/* [0xc] lba increment factor, for multiple concatenated blocks in a single pkt (msbs) */
	uint32_t lba_inc_3;
	/* [0x10] lba alpha constant, for multiple concatenated blocks in a single pkt (lsbs) */
	uint32_t alpha_0;
	/* [0x14] lba alpha constant, for multiple concatenated blocks in a single pkt */
	uint32_t alpha_1;
	/* [0x18] lba alpha constant, for multiple concatenated blocks in a single pkt */
	uint32_t alpha_2;
	/* [0x1c] lba alpha constant, for multiple concatenated blocks in a single pkt (msbs) */
	uint32_t alpha_3;
	/* [0x20] endianity flexibility support */
	uint32_t tweak_calc_swap;
	uint32_t rsrvd[7];
};

struct crypto_regs {
	uint32_t rsrvd_0[192];
	struct al_crypto_accelerator_configuration configuration; /* [0x300] */
	struct al_crypto_accelerator_debug debug;               /* [0x310] */
	struct al_crypto_accelerator_log log;                   /* [0x320] */
	struct al_crypto_accelerator_crypto_perf_counter crypto_perf_counter; /* [0x340] */
	struct al_crypto_accelerator_perfm_cnt_cntl perfm_cnt_cntl; /* [0x390] */
	struct al_crypto_accelerator_crypto_status crypto_status; /* [0x394] */
	struct al_crypto_accelerator_crypto_version crypto_version; /* [0x398] */
	struct al_crypto_accelerator_crypto_algorithms crypto_algorithms; /* [0x3a0] */
	uint32_t rsrvd_1[19];
	struct al_crypto_accelerator_reserved reserved;         /* [0x3f0] */
	struct al_crypto_accelerator_sha3_round_consts sha3_round_consts[64]; /* [0x400] */
	struct al_crypto_accelerator_sha3_params sha3_params;   /* [0x600] */
	struct al_crypto_accelerator_sha3_debug sha3_debug;     /* [0x630] */
	struct al_crypto_accelerator_xts_conf xts_conf;         /* [0x640] */
};


/*
 * Registers Fields
 */

/**** unit_conf register ****/
/*
 * When this bit is set to 1, the Crypto engine accepts new commands, if possible before previous
 * command completion.
 */
#define CRYPTO_CONFIGURATION_UNIT_CONF_MUL_CMD_EN (1 << 0)
/* When this bit is set to 1, when an error occurs, the pipe will hold. */
#define CRYPTO_CONFIGURATION_UNIT_CONF_HOLD_PIPE_WHEN_ERROR (1 << 1)
/*
 * Enable smart clk-gating of unused Crypto engines.
 * Bit 14 enables smart clock gating for the entire Crypto unit.
 * Bits 13:11 enabling/disabling of smart clk gating in AES/SHA/DES respectively, e.g., for smart
 * clk gating in AES & DES, but no clk gating in SHA, set this field to 0b1101.
 */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CLK_GATE_UNUSED_ENGINES_MASK 0x0000003C
#define CRYPTO_CONFIGURATION_UNIT_CONF_CLK_GATE_UNUSED_ENGINES_SHIFT 2
/*
 * [18] Disable clk to Crypto unit.
 * [17:15] Disable clk to AES/SHA/DES core (respectively).
 */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CLK_SHUTDOWN_MASK 0x000003C0
#define CRYPTO_CONFIGURATION_UNIT_CONF_CLK_SHUTDOWN_SHIFT 6
/*
 * 0x0 - RR from VF #0 to VF #(N-1).
 * 0x1 - RR from VF #(N-1) to VF #0.
 * 0x2 - Strict Priority according to vf_arb_prio
 */
#define CRYPTO_CONFIGURATION_UNIT_CONF_VF_ARB_MODE_MASK 0x00000C00
#define CRYPTO_CONFIGURATION_UNIT_CONF_VF_ARB_MODE_SHIFT 10
/* Index of prioritized VF (RR will start from this VF). */
#define CRYPTO_CONFIGURATION_UNIT_CONF_VF_ARB_PRIO_MASK 0x00007000
#define CRYPTO_CONFIGURATION_UNIT_CONF_VF_ARB_PRIO_SHIFT 12
/* Enable key parity check for DES */
#define CRYPTO_CONFIGURATION_UNIT_CONF_DES_KEY_PARITY_CHECK_EN (1 << 15)
/* Enable DES EEE mode */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CHICKEN_DES_EEE_EN (1 << 16)
/* Enable Auth to start together with Enc */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CHICKEN_AUTH_START_WITH_ENC_EN (1 << 17)
/* Enable SHA-3 engine */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CHICKEN_SHA3_EN (1 << 18)
/* Enable long SAs (useful for modes requiring HMAC/broken-pkt while using SHA-3 algorithm) */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CHICKEN_LONG_SA_EN (1 << 19)
/* Disable HW padding of incoming pkts */
#define CRYPTO_CONFIGURATION_UNIT_CONF_CHICKEN_SHA_PADDING_DISABLE (1 << 20)
/* give each GDMA ability to access all SAD lines */
#define CRYPTO_CONFIGURATION_UNIT_CONF_GDMA_FULL_SAD_ACCESS_EN (1 << 21)

/**** mem_test register ****/
/* 0xA enables test mode */
#define CRYPTO_CONFIGURATION_MEM_TEST_VAL_MASK 0x0000000F
#define CRYPTO_CONFIGURATION_MEM_TEST_VAL_SHIFT 0

/**** unit_debug_conf register ****/
/* When these bits are set to 0, reset the ack FIFOs. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_ACK_MASK 0x0000000F
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_ACK_SHIFT 0
/* When this bit is set to 0, reset the payload FIFO between the engines and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_E2M_PYLD (1 << 4)
/* When this bit is set to 0, reset the sign FIFO between the engines and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_E2M_SIGN (1 << 5)
/* When this bit is set to 0, reset the intr FIFO between the engines and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_E2M_INTR (1 << 6)
/* When this bit is set to 0, reset the data FIFO between the decoder and the engines. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2E_DATA (1 << 7)
/* When this bit is set to 0, reset the control FIFO between the decoder and the engines. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2E_CTRL (1 << 8)
/* When this bit is set to 0, reset the context FIFO between the decoder and the engines. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2E_CONTEXT (1 << 9)
/* When this bit is set to 0, reset the SA bypass FIFO between the decoder and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2M_SA_BYPASS (1 << 10)
/* When this bit is set to 0, reset the IV bypass FIFO between the decoder and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2M_IV_BYPASS (1 << 11)
/* When this bit is set to 0, reset the ICV bypass FIFO between the decoder and the dispatcher. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_D2M_ICV_BYPASS (1 << 12)
/* When this bit is set to 0, reset the aligner FIFO. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_ALIGNER_AXIS (1 << 13)
/* When this bit is set to 0, reset the ingress FIFO. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_FIFO_ENABLE_INGRESS (1 << 14)
/* Clear the data_aligner (decode stage). */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_CLR_ALIGNER (1 << 15)
/* Clear the data_splitter (dispatch stage). */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_CLR_SPLITTER (1 << 16)
/* Don't perform memory init. */
#define CRYPTO_DEBUG_UNIT_DEBUG_CONF_MEM_INIT_SKIP (1 << 17)

/**** trans_info_1 register ****/
/* Transaction length in bytes */
#define CRYPTO_LOG_TRANS_INFO_1_TRANS_LEN_MASK 0x000FFFFF
#define CRYPTO_LOG_TRANS_INFO_1_TRANS_LEN_SHIFT 0
/* Number of descriptors in the transaction */
#define CRYPTO_LOG_TRANS_INFO_1_NUM_OF_DESC_MASK 0x00F00000
#define CRYPTO_LOG_TRANS_INFO_1_NUM_OF_DESC_SHIFT 20
/* Reserved */
#define CRYPTO_LOG_TRANS_INFO_1_RESERVED_MASK 0xFF000000
#define CRYPTO_LOG_TRANS_INFO_1_RESERVED_SHIFT 24

/**** trans_info_2 register ****/
/* Queue Number of the transaction */
#define CRYPTO_LOG_TRANS_INFO_2_Q_NUM_MASK 0x00000FFF
#define CRYPTO_LOG_TRANS_INFO_2_Q_NUM_SHIFT 0
/* GDMA ID of the transaction */
#define CRYPTO_LOG_TRANS_INFO_2_DMA_ID_MASK 0x0000F000
#define CRYPTO_LOG_TRANS_INFO_2_DMA_ID_SHIFT 12
/* Internal Serial Number of the transaction */
#define CRYPTO_LOG_TRANS_INFO_2_SERIAL_NUM_MASK 0x03FF0000
#define CRYPTO_LOG_TRANS_INFO_2_SERIAL_NUM_SHIFT 16
/* Reserved */
#define CRYPTO_LOG_TRANS_INFO_2_RESERVED_MASK 0xFC000000
#define CRYPTO_LOG_TRANS_INFO_2_RESERVED_SHIFT 26

/**** conf register ****/
/*
 * Does not affect the recover_err_cnt
 * 0 - Clear performance counter
 * 1 - Stop performance counter
 * 2 - Active performance counter
 */
#define CRYPTO_PERFM_CNT_CNTL_CONF_CONT_PERFORM_MASK 0x00000003
#define CRYPTO_PERFM_CNT_CNTL_CONF_CONT_PERFORM_SHIFT 0

/**** status register ****/
/* Indicates when CRYPTO is empty */
#define CRYPTO_CRYPTO_STATUS_STATUS_CRYPTO_EMPTY (1 << 0)
/* Internal state of decoder */
#define CRYPTO_CRYPTO_STATUS_STATUS_CRYPTO_DEC_STATE_MASK 0x0000001E
#define CRYPTO_CRYPTO_STATUS_STATUS_CRYPTO_DEC_STATE_SHIFT 1
/* Internal state of dispatcher */
#define CRYPTO_CRYPTO_STATUS_STATUS_CRYPTO_DISPATCHER_STATE_MASK 0x000001E0
#define CRYPTO_CRYPTO_STATUS_STATUS_CRYPTO_DISPATCHER_STATE_SHIFT 5
/* Reserved */
#define CRYPTO_CRYPTO_STATUS_STATUS_RESERVED_MASK 0xFFFFFE00
#define CRYPTO_CRYPTO_STATUS_STATUS_RESERVED_SHIFT 9

/**** revision register ****/
/* 0x0 for M0, 0x1 for A0 */
#define CRYPTO_CRYPTO_VERSION_REVISION_REV_ID_MASK 0x0000FFFF
#define CRYPTO_CRYPTO_VERSION_REVISION_REV_ID_SHIFT 0
/* Annapurna Labs PCI vendor ID */
#define CRYPTO_CRYPTO_VERSION_REVISION_VENDOR_ID_MASK 0xFFFF0000
#define CRYPTO_CRYPTO_VERSION_REVISION_VENDOR_ID_SHIFT 16

/**** date register ****/
/* Date of release */
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_DAY_MASK 0x0000001F
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_DAY_SHIFT 0
/* Date of release */
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_7_5_MASK 0x000000E0
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_7_5_SHIFT 5
/* Month of release */
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_MONTH_MASK 0x00000F00
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_MONTH_SHIFT 8
/* Date of release */
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_15_12_MASK 0x0000F000
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_15_12_SHIFT 12
/* Month of release */
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_YEAR_MASK 0x00FF0000
#define CRYPTO_CRYPTO_VERSION_DATE_DATE_YEAR_SHIFT 16
/* Date of release */
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_31_24_MASK 0xFF000000
#define CRYPTO_CRYPTO_VERSION_DATE_RESERVED_31_24_SHIFT 24

/**** algorithms register ****/
/* AES ECB */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_ECB (1 << 0)
/* AES CBC */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_CBC (1 << 1)
/* AES_CTR */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_CTR (1 << 2)
/* AES CFB */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_CFB (1 << 3)
/* AES OFB */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_OFB (1 << 4)
/* AES GCM */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_GCM (1 << 5)
/* AES CCM */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_CCM (1 << 6)
/* AES XTS */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_AES_XTS (1 << 7)
/* SHA3_DEFAULT_PARAMS */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_3_DEFAULT (1 << 8)
/* SHA3_224 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_3_224 (1 << 9)
/* SHA3_256 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_3_256 (1 << 10)
/* SHA3_384 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_3_384 (1 << 11)
/* SHA3_512 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_3_512 (1 << 12)
/* SHA-1 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_1 (1 << 13)
/* SHA2_256 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_2_256 (1 << 14)
/* SHA2_384 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_2_384 (1 << 15)
/* SHA2_512 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHA_2_512 (1 << 16)
/* MD-5 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_MD_5 (1 << 17)
/* SHAKE_128 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHAKE_128 (1 << 18)
/* SHAKE_256 */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_SHAKE_256 (1 << 19)
/* Reserved */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_RESERVED_23_20_MASK 0x00F00000
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_RESERVED_23_20_SHIFT 20
/* DES ECB */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_DES_ECB (1 << 24)
/* DES CBC */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_DES_CBC (1 << 25)
/* DES3 ECB */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_DES3_ECB (1 << 26)
/* DES3 CBC */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_DES3CBC (1 << 27)
/* Reserved */
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_RESERVED_31_28_MASK 0xF0000000
#define CRYPTO_CRYPTO_ALGORITHMS_ALGORITHMS_RESERVED_31_28_SHIFT 28

/**** num_rounds register ****/
/* number of iterations in SHA3 calculation */
#define CRYPTO_SHA3_PARAMS_NUM_ROUNDS_VAL_MASK 0x0000003F
#define CRYPTO_SHA3_PARAMS_NUM_ROUNDS_VAL_SHIFT 0

/**** bitrate_sha3_224 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_224_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_224_VAL_SHIFT 0

/**** bitrate_sha3_256 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_256_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_256_VAL_SHIFT 0

/**** bitrate_sha3_384 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_384_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_384_VAL_SHIFT 0

/**** bitrate_sha3_512 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_512_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHA3_512_VAL_SHIFT 0

/**** bitrate_shake_128 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHAKE_128_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHAKE_128_VAL_SHIFT 0

/**** bitrate_shake_256 register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_SHAKE_256_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_SHAKE_256_VAL_SHIFT 0

/**** bitrate_default register ****/
/*
 * bitrate value in bytes. value must be 0-200.
 * change from default value with care (value is defined by standard)
 */
#define CRYPTO_SHA3_PARAMS_BITRATE_DEFAULT_VAL_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_BITRATE_DEFAULT_VAL_SHIFT 0

/**** padding_sha3 register ****/
/* padding on first byte after payload */
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_START_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_START_SHIFT 0
/* padding on bytes which are not first after payload, or last in word */
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_MID_MASK 0x0000FF00
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_MID_SHIFT 8
/* padding on last byte in word */
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_END_MASK 0x00FF0000
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_END_SHIFT 16
/* single byte padding value */
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_START_END_MASK 0xFF000000
#define CRYPTO_SHA3_PARAMS_PADDING_SHA3_START_END_SHIFT 24

/**** padding_shake register ****/
/* padding on first byte after payload */
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_START_MASK 0x000000FF
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_START_SHIFT 0
/* padding on bytes which are not first after payload, or last in word */
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_MID_MASK 0x0000FF00
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_MID_SHIFT 8
/* padding on last byte in word */
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_END_MASK 0x00FF0000
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_END_SHIFT 16
/* single byte padding value */
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_START_END_MASK 0xFF000000
#define CRYPTO_SHA3_PARAMS_PADDING_SHAKE_START_END_SHIFT 24

/**** endianity register ****/
/* when asserted, sha3 performs bit-swap for every byte on the input data */
#define CRYPTO_SHA3_PARAMS_ENDIANITY_BIT_SWAP_IN (1 << 0)
/* when asserted, sha3 performs bit-swap for every byte on the output data */
#define CRYPTO_SHA3_PARAMS_ENDIANITY_BIT_SWAP_OUT (1 << 1)
/* take digests from state's msbs */
#define CRYPTO_SHA3_PARAMS_ENDIANITY_DIGEST_FROM_MSBS (1 << 2)

/**** fsm_state register ****/
/* current state of preprocessor block (in charge of padding and serial->parallel abosrption */
#define CRYPTO_SHA3_DEBUG_FSM_STATE_PREPROC_MASK 0x0000000F
#define CRYPTO_SHA3_DEBUG_FSM_STATE_PREPROC_SHIFT 0
/* current state of sha3 engine */
#define CRYPTO_SHA3_DEBUG_FSM_STATE_ENGINE_MASK 0x000000F0
#define CRYPTO_SHA3_DEBUG_FSM_STATE_ENGINE_SHIFT 4
/* current state of sha3 engine */
#define CRYPTO_SHA3_DEBUG_FSM_STATE_PERM_MASK 0x00000F00
#define CRYPTO_SHA3_DEBUG_FSM_STATE_PERM_SHIFT 8
/* current round counter value */
#define CRYPTO_SHA3_DEBUG_FSM_STATE_ROUND_CNTR_MASK 0x0003F000
#define CRYPTO_SHA3_DEBUG_FSM_STATE_ROUND_CNTR_SHIFT 12

/**** tweak_calc_swap register ****/

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_BITS (1 << 0)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_BYTES (1 << 1)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_WORDS (1 << 2)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_BITS (1 << 3)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_BYTES (1 << 4)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_WORDS (1 << 5)

#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_RESERVED_31_6_MASK 0xFFFFFFC0
#define CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_RESERVED_31_6_SHIFT 6

#ifdef __cplusplus
}
#endif

#endif

/** @} */


