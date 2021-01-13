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
 * @defgroup group_ssm_api API
 * Cryptographic / RAID Acceleration Engine common HAL API
 * @ingroup group_ssm
 * @{
 * @file   al_hal_ssm.h
 */

#ifndef __AL_HAL_SSM_H__
#define __AL_HAL_SSM_H__

#include "al_hal_common.h"
#include "al_hal_udma.h"
#include "al_hal_m2m_udma.h"

#define AL_SSM_V1_V2_CRC_NUM	2
#define AL_SSM_MAX_CRC		2

#define AL_SSM_MAX_SRC_DESCS	31
#define AL_SSM_MAX_DST_DESCS	31

enum al_ssm_op_flags {
	AL_SSM_INTERRUPT = AL_BIT(0), /* enable interrupt when the xaction
					  completes */
	AL_SSM_BARRIER = AL_BIT(1), /* data memory barrier, subsequent xactions
					will be served only when the current one
					completes */
	AL_SSM_SRC_NO_SNOOP = AL_BIT(2), /* set no snoop on source buffers */
	AL_SSM_DEST_NO_SNOOP = AL_BIT(3), /* set no snoop on destination buffers */
};

/**
 * SSM queue types.
 * Must be statically allocated and queue type can not be changed in run time
 *
 * Each DMA Queue can have one of the three types:
 * 1. AL_CRYPT_AUTH_Q:
 *    used for Crypto calculations (e.g. encryption, authentication and
 *    cryptographic hash functions). Crypto engine is shared between all SSM
 *    DMAs, transactions from two AL_CRYPTO_AUTH_Q queues are serialized before
 *    being processed by the hardware engine.
 *
 * 2. AL_RAID_Q:
 *    used for RAID and Erasure Code calculations (e.g. RAID5, RAID6) RAID
 *    engine is shared between all SSM DMAs, transactions from two AL_RAID_Q
 *    queues are serialized before being processed by the hardware engine.
 *
 * 3. AL_MEM_CRC_MEMCPY_Q:
 *    used for memory operations and CRC calculations (e.g. memory copy/compare,
 *    Checksum, CRC). MEM_CRC engine is duplicated per DMA, transactions from
 *    two AL_MEM_CRC_MEMCPY_Q queues and different DMAs are handled in parallel.
 *
 * Note: transactions from different DMAs and queue types, are handled in
 *       parallel by the hardware engines.
 */
enum al_ssm_q_type {
	AL_CRYPT_AUTH_Q,
	AL_MEM_CRC_MEMCPY_Q,
	AL_RAID_Q
};

/** SSM (security, storage, memory) DMA private data structure
 * The driver maintains M2M UDMA structure as the HW consists of two UDMAS.
 * both of the UDMAs initializes and managed using the m2m udma module.
 * the driver uses RX completion descriptors as the sole indication for
 * completing transactions, and disregards any TX completion descriptors.
 * Every queue can be marked as crypt/auth queue to be used for crypt/auth
 * transactions or crc/csum/memcpy queue to be used for crc/csum/memcpy
 * transactions or raid queue to be used for raid transactions
 */
struct al_ssm_dma {
	uint8_t rev_id; /**<PCI adapter revision ID */
	struct al_m2m_udma m2m_udma;
	enum al_ssm_q_type q_types[DMA_MAX_Q];
};

/* SSM DMA parameters from upper layer */
struct al_ssm_dma_params {
	uint8_t rev_id; /**<PCI adapter revision ID */
	void __iomem *udma_regs_base;
	char *name; /* the upper layer must keep the string area */
	uint8_t num_of_queues;/* number of queues */
};

/* SSM Unit info of register base address */
struct al_ssm_unit_regs_info {
	/* UDMA register base address */
	void __iomem *udma_regs_base;
	/* CRC register base addresses */
	void __iomem *crc_regs_base[AL_SSM_MAX_CRC];
	/* crypto register base address */
	void __iomem *crypto_regs_base;
	/* RAID register base address */
	void __iomem *raid_regs_base;
};

/* Init DMA */
int al_ssm_dma_init(struct al_ssm_dma *ssm_dma, struct al_ssm_dma_params *params);

/* Initialize the m2s(tx) and s2m(rx) components of the queue */
int al_ssm_dma_q_init(struct al_ssm_dma		*ssm_dma,
		      uint32_t			qid,
		      struct al_udma_q_params	*tx_params,
		      struct al_udma_q_params	*rx_params,
		      enum al_ssm_q_type	q_type);

/* Set ssm DMA state */
int al_ssm_dma_state_set(struct al_ssm_dma *dma, enum al_udma_state dma_state);

/* Get udma dma handle, this handle can be used to call misc configuration */
/* functions defined at al_udma_config.h. TODO: add more explanation */
int al_ssm_dma_handle_get(struct al_ssm_dma *ssm_dma,
			  enum al_udma_type type,
			  struct al_udma **udma);

/**
 * Get TX udma handle
 *
 * This handle can be used to call misc configuration functions defined at
 * al_udma_config.h
 *
 * @param	ssm_dma
 *
 * @returns	TX UDMA handle upon success, or NULL upon failure
 */
struct al_udma *al_ssm_dma_tx_udma_handle_get(struct al_ssm_dma *ssm_dma);

/**
 * Get TX queue handle
 *
 * @param	ssm_dma - ssm DMA context
 * @param	qid - Queue ID
 *
 * @returns	TX queue handle upon success, or NULL upon failure
 */
struct al_udma_q *al_ssm_dma_tx_queue_handle_get(struct al_ssm_dma *ssm_dma,
						 unsigned int	   qid);

/**
 * Get RX udma handle
 *
 * This handle can be used to call misc configuration functions defined at
 * al_udma_config.h
 *
 * @param	ssm_dma
 *
 * @returns	RX UDMA handle upon success, or NULL upon failure
 */
struct al_udma *al_ssm_dma_rx_udma_handle_get(struct al_ssm_dma *ssm_dma);

/**
 * Get RX queue handle
 *
 * @param	ssm_dma - ssm DMA context
 * @param	qid - Queue ID
 *
 * @returns	RX queue handle upon success, or NULL upon failure
 */
struct al_udma_q *al_ssm_dma_rx_queue_handle_get(struct al_ssm_dma *ssm_dma,
						 unsigned int	   qid);

/**
 * Start asynchronous execution of SSM transaction
 *
 * Update the tail pointer of the submission ring of the TX UDMA about
 * previously prepared descriptors.
 * This function could return before the hardware start the work as its an
 * asynchronous non-blocking call to the hardware.
 *
 * @param ssm_dma - ssm DMA context
 * @param qid - Queue ID
 * @param tx_descs number of tx descriptors to be processed by the engine
 *
 * @return 0 if no error found.
 *	   -EINVAL if quid is out of range
 */
int al_ssm_dma_action(struct al_ssm_dma *ssm_dma, uint32_t qid, int tx_descs);

/**
 * Get SSM unit registers addresses
 *
 * Update the SSM unit info structure according to PCI device ID
 * and bars addresses
 *
 * @param bars - PCI bars addresses
 * @param dev_id - PCI device ID
 * @param rev_id - PCI revision ID
 * @param unit_info - SSM unit register info context
 */
void al_ssm_unit_regs_info_get(void *bars[6],
			       unsigned int dev_id,
			       unsigned int rev_id,
			       struct al_ssm_unit_regs_info *unit_info);

/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */
/** @} end of SSM group */
#endif		/* __AL_HAL_SSM_H__ */
