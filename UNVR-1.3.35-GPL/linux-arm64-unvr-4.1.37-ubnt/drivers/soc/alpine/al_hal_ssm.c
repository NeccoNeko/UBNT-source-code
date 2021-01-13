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
 * @file   al_hal_ssm.c
 *
 */

#include "al_hal_ssm.h"
#include "al_hal_ssm_crypto.h"
#include "al_hal_ssm_raid.h"

/* Crypto register base offset */
#define AL_CRYPTO_V1_REGS_BASE_OFFSET			0x800
#define AL_CRYPTO_V1_CRC_REGS_BASE_OFFSET(crc_idx)	(0x200 * (crc_idx))

/* Raid registers base address offsets */
#define AL_RAID_V1_REGS_BASE_OFFSET			0x0
#define AL_RAID_V1_CRC_REGS_BASE_OFFSET(crc_idx)	(0x400 + (0x400 * (crc_idx)))

/* SSM registers base address offsets */
#define AL_SSM_V2_CRYPTO_REGS_BASE_OFFSET		0x4000
#define AL_SSM_V2_RAID_REGS_BASE_OFFSET			0x5000
#define AL_SSM_V2_CRC_REGS_BASE_OFFSET(crc_idx)		(0x1000 * (crc_idx))

/**
 * Initialize DMA for SSM operations
 *
 * @param ssm_dma ssm dma handle
 * @param params parameters from upper layer
 *
 * @return 0 on success.
 */
int al_ssm_dma_init(
	struct al_ssm_dma		*ssm_dma,
	struct al_ssm_dma_params	*params)
{
	struct al_m2m_udma_params m2m_params;
	struct unit_regs __iomem *unit_regs;
	int rc;

	al_dbg("ssm [%s]: initialize DMA\n", params->name);

	ssm_dma->rev_id = params->rev_id;

	m2m_params.name = params->name;
	unit_regs = (struct unit_regs __iomem *)params->udma_regs_base;
	m2m_params.udma_regs_base = unit_regs;
	m2m_params.num_of_queues = params->num_of_queues;
	m2m_params.max_m2s_descs_per_pkt = AL_SSM_MAX_SRC_DESCS;
	m2m_params.max_s2m_descs_per_pkt = AL_SSM_MAX_DST_DESCS;

	/* initialize the udma  */
	rc =  al_m2m_udma_init(&ssm_dma->m2m_udma, &m2m_params);
	if (rc != 0)
		al_err("failed to initialize udma, error %d\n", rc);
	return rc;
}

/**
 * Initialize the m2s(tx) and s2m(rx) components of the queue
 *
 * @param ssm_dma ssm dma handle
 * @param qid queue index
 * @param tx_params TX UDMA params
 * @param rx_params RX UDMA params
 * @param q_type indicate q type (crc/csum/memcpy, crypto, raid)
 *
 * @return 0 if no error found.
 *	   -EINVAL if the qid is out of range
 *	   -EIO if queue was already initialized
 */
int al_ssm_dma_q_init(struct al_ssm_dma		*ssm_dma,
		      uint32_t			qid,
		      struct al_udma_q_params	*tx_params,
		      struct al_udma_q_params	*rx_params,
		      enum al_ssm_q_type	q_type)
{
	int rc;

	al_dbg("ssm [%s]: Initialize queue %d\n",
		 ssm_dma->m2m_udma.name, qid);

	tx_params->adapter_rev_id = ssm_dma->rev_id;
	rx_params->adapter_rev_id = ssm_dma->rev_id;

	rc = al_m2m_udma_q_init(&ssm_dma->m2m_udma, qid, tx_params, rx_params);
	if (rc != 0)
		al_err("ssm [%s]: failed to initialize q %d, error %d\n",
			 ssm_dma->m2m_udma.name, qid, rc);
	else
		ssm_dma->q_types[qid] = q_type;

	return rc;
}

/**
 * Change the DMA state
 *
 * @param ssm_dma ssm DMA handle
 * @param dma_state the new state
 *
 * @return 0
 */
int al_ssm_dma_state_set(
	struct al_ssm_dma	*ssm_dma,
	enum al_udma_state	dma_state)
{
	int rc;

	rc =  al_m2m_udma_state_set(&ssm_dma->m2m_udma, dma_state);
	if (rc != 0)
		al_err("ssm [%s]: failed to change state, error %d\n",
			 ssm_dma->m2m_udma.name, rc);
	return rc;
}

/**
 * Get udma handle of the tx or rx udma, this handle can be used to call misc
 * configuration functions defined at al_udma_config.h
 *
 * @param ssm_dma ssm DMA handle
 * @param type tx or rx udma
 * @param udma the requested udma handle written to this pointer
 *
 * @return 0
 */
int al_ssm_dma_handle_get(
	struct al_ssm_dma	*ssm_dma,
	enum al_udma_type	type,
	struct al_udma		**udma)
{
	return al_m2m_udma_handle_get(&ssm_dma->m2m_udma, type, udma);
}

/******************************************************************************
 ******************************************************************************/
struct al_udma *al_ssm_dma_tx_udma_handle_get(
	struct al_ssm_dma *ssm_dma)
{
	struct al_udma *udma;
	int err;

	err = al_m2m_udma_handle_get(&ssm_dma->m2m_udma, UDMA_TX, &udma);
	if (err)
		return NULL;

	return udma;
}

/******************************************************************************
 ******************************************************************************/
struct al_udma_q *al_ssm_dma_tx_queue_handle_get(
	struct al_ssm_dma	*ssm_dma,
	unsigned int		qid)
{
	struct al_udma *udma;
	int err;

	err = al_m2m_udma_handle_get(&ssm_dma->m2m_udma, UDMA_TX, &udma);
	if (err)
		return NULL;

	return &udma->udma_q[qid];
}

/******************************************************************************
 ******************************************************************************/
struct al_udma *al_ssm_dma_rx_udma_handle_get(
	struct al_ssm_dma *ssm_dma)
{
	struct al_udma *udma;
	int err;

	err = al_m2m_udma_handle_get(&ssm_dma->m2m_udma, UDMA_RX, &udma);
	if (err)
		return NULL;

	return udma;
}

/******************************************************************************
 ******************************************************************************/
struct al_udma_q *al_ssm_dma_rx_queue_handle_get(
	struct al_ssm_dma	*ssm_dma,
	unsigned int		qid)
{
	struct al_udma *udma;
	int err;

	err = al_m2m_udma_handle_get(&ssm_dma->m2m_udma, UDMA_RX, &udma);
	if (err)
		return NULL;

	return &udma->udma_q[qid];
}

/******************************************************************************
 ******************************************************************************/
int al_ssm_dma_action(struct al_ssm_dma *dma, uint32_t qid, int tx_descs)
{
	struct al_udma_q *tx_udma_q;
	int rc;

	rc = al_udma_q_handle_get(&dma->m2m_udma.tx_udma, qid, &tx_udma_q);
	al_assert(!rc); /* assert valid tx q handle */

	/* add tx descriptors */
	al_udma_desc_action_add(tx_udma_q, tx_descs);
	return 0;
}

/******************************************************************************
 ******************************************************************************/
void al_ssm_unit_regs_info_get(void *bars[6],
			       unsigned int dev_id,
			       unsigned int rev_id __attribute__((__unused__)),
			       struct al_ssm_unit_regs_info *unit_info)
{
	al_assert(unit_info);
	al_assert(bars);
	al_assert(bars[0]);

	al_memset(unit_info, 0, sizeof(*unit_info));

	/* UDMA regs base is always BAR 0 address */
	unit_info->udma_regs_base = bars[0];

	/* Application regs base is always BAR 4 address */
	if (bars[4]) {
		int crc_idx;
		char *app_bar = bars[4];

		switch (dev_id) {
		case AL_CRYPTO_ALPINE_V2_DEV_ID:
			unit_info->crypto_regs_base = app_bar + AL_SSM_V2_CRYPTO_REGS_BASE_OFFSET;
			unit_info->raid_regs_base = app_bar + AL_SSM_V2_RAID_REGS_BASE_OFFSET;
			for (crc_idx = 0; crc_idx < AL_SSM_V1_V2_CRC_NUM; crc_idx++)
				unit_info->crc_regs_base[crc_idx] = app_bar +
						AL_SSM_V2_CRC_REGS_BASE_OFFSET(crc_idx);
			break;
		case AL_CRYPTO_ALPINE_V1_DEV_ID:
			unit_info->crypto_regs_base = app_bar + AL_CRYPTO_V1_REGS_BASE_OFFSET;
			for (crc_idx = 0; crc_idx < AL_SSM_V1_V2_CRC_NUM; crc_idx++)
				unit_info->crc_regs_base[crc_idx] = app_bar +
						AL_CRYPTO_V1_CRC_REGS_BASE_OFFSET(crc_idx);
			break;
		case AL_RAID_DEV_ID:
			unit_info->raid_regs_base = app_bar + AL_RAID_V1_REGS_BASE_OFFSET;
			for (crc_idx = 0; crc_idx < AL_SSM_V1_V2_CRC_NUM; crc_idx++)
				unit_info->crc_regs_base[crc_idx] = app_bar +
						AL_RAID_V1_CRC_REGS_BASE_OFFSET(crc_idx);
			break;
		default:
			al_err("%s: unknown device ID %d\n", __func__, dev_id);
			al_assert(0);
		}
	}
}
