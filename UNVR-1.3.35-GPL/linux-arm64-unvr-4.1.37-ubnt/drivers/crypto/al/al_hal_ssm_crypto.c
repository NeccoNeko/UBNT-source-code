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
#include "al_hal_ssm.h"
#include "al_hal_ssm_crypto.h"
#include "al_hal_ssm_crypto_regs.h"

/*
 * Rx (S2M) Descriptors
 */
#define RX_DESC_META			(1<<30)	/* Meta data */

/* Tx (M2S) word1 common Descriptors */
#define TX_DESC_META_OP_MASK		(0x3<<23)
#define TX_DESC_META_OP_SHIFT		(23)

/*
 * Crypto
 */
#define TX_DESC_META_CRYPT_DIR_SHIFT	(22)	/* Direction: enc/dec and/or sign/auth */
#define TX_DESC_META_CRYPT_S_GET_SA	(1<<21)	/* Evict SA */
#define TX_DESC_META_CRYPT_S_GET_ENCIV	(1<<20)	/* Get IV */
#define TX_DESC_META_CRYPT_GET_ORIG	(1<<19) /* Get Output original packet, after calc is done */
#define TX_DESC_META_CRYPT_GET_AUTHIV	(1<<18) /* Get Authentication IV */
#define TX_DESC_META_CRYPT_S_GET_SIGN	(1<<17)	/* Get Sign */

#define TX_DESC_META_AUTH_FIRST		(1<<16)	/* Auth only first bit */
#define TX_DESC_META_AUTH_LAST		(1<<15)	/* Auth only last bit */

#define TX_DESC_META_AUTH_VALID		(1<<14)	/* Validate Signature */

#define TX_DESC_META_SA_IDX_MASK	(0xff<<5) /* SA index mask */
#define TX_DESC_META_SA_IDX_SHIFT	(5)

#define TX_DESC_META_BUF_TYPE_MASK	(0x7)/* Buffer type mask */
#define TX_DESC_META_BUF_TYPE_SHIFT	(0)

/* Tx (M2S) word2 Descriptors */
#define TX_DESC_META_ENC_OFF_MASK	(0xffff<<16)
#define TX_DESC_META_ENC_OFF_SHIFT	(16)
#define TX_DESC_META_ENC_OFF_EOP_MASK	(0xffff)
#define TX_DESC_META_ENC_OFF_EOP_SHIFT	(0)

/* Tx (M2S) word3 Descriptors */
#define TX_DESC_META_AUTH_OFF_MASK	(0xffff<<16)
#define TX_DESC_META_AUTH_OFF_SHIFT	(16)
#define TX_DESC_META_AUTH_OFF_EOP_MASK	(0xffff)
#define TX_DESC_META_AUTH_OFF_EOP_SHIFT	(0)

#define RX_COMP_STATUS_MASK	(AL_CRYPT_AUTH_ERROR			| \
				 AL_CRYPT_SA_IV_EVICT_FIFO_ERROR	| \
				 AL_CRYPT_DES_ILLEGAL_KEY_ERROR		| \
				 AL_CRYPT_M2S_ERROR			| \
				 AL_CRYPT_SRAM_PARITY_ERROR		| \
				 AL_CRYPT_INTERNAL_FLOW_VIOLATION_ERROR)

/*
 * Crypto DMA operation (Enc, Auth or Enc + Auth)
 */
#define AL_CRYPT_OP 3

/** Crypto DMA buffer types */
enum al_crypto_buf_type {
	AL_CRYPT_BUF_SA_UPDATE = 0,
	AL_CRYPT_BUF_ENC_IV = 1,
	AL_CRYPT_BUF_AUTH_IV = 2,
	AL_CRYPT_BUF_SRC = 3,
	AL_CRYPT_BUF_AUTH_SIGN = 4
};


/*
 * SA
 */

/* Crypto operation check macros */
#define AL_CRYPT_OP_IS(op, is)		((op) == (is))
#define AL_CRYPT_OP_HAS(op, has)	((op) & (has))

/* Word 0 */
#define CRYPT_SAD_OP_MASK		(0x3<<30)/* Crypto Operation */
#define CRYPT_SAD_OP_SHIFT		(30)
#define CRYPT_SAD_ENC_TYPE_MASK		(0xf<<25)/* Crypto Type */
#define CRYPT_SAD_ENC_TYPE_SHIFT	(25)
#define CRYPT_SAD_SA_LENGTH_MASK	(0x3<<23)/* SA Length */
#define CRYPT_SAD_SA_LENGTH_SHIFT	(23)
#define CRYPT_SAD_TRIPDES_MODE_MASK	(0x1<<22)/* 3DES mode */
#define CRYPT_SAD_TRIPDES_MODE_SHIFT	(22)
#define CRYPT_SAD_AES_KEY_SIZE_MASK	(0x3<<20)/* AES key size */
#define CRYPT_SAD_AES_KEY_SIZE_SHIFT	(20)
#define CRYPT_SAD_AUTH_TYPE_MASK	(0xf<<12)/* Auth type */
#define CRYPT_SAD_AUTH_TYPE_SHIFT	(12)
#define CRYPT_SAD_SIGN_SIZE_MASK	(0xf<<8) /* Signature size */
#define CRYPT_SAD_SIGN_SIZE_SHIFT	(8)
#define CRYPT_SAD_SHA2_MODE_MASK	(0x3<<6) /* Sha2 mode */
#define CRYPT_SAD_SHA2_MODE_SHIFT	(6)
#define CRYPT_SAD_SHA3_MODE_MASK	(0x3<<6) /* Sha3 mode */
#define CRYPT_SAD_SHA3_MODE_SHIFT	(6)
#define CRYPT_SAD_SHA3_MODE_EXT_MASK	(0x3<<6) /* Sha3 mode extension */
#define CRYPT_SAD_SHA3_MODE_EXT_SHIFT	(6)
#define CRYPT_SAD_HMAC_EN		(1<<5)   /* Hmac enable */
#define CRYPT_SAD_SIGN_AFTER_ENC	(1<<4)   /* Sign after encryption */
#define CRYPT_SAD_AUTH_AFTER_DEC	(1<<3)   /* Auth after decryption */
#define CRYPT_SAD_AUTH_MSB_BITS         (1<<2)   /* Auth use the MSB of the signature */
#define CRYPT_SAD_CNTR_SIZE_MASK	(0x3)    /* Counter size */
#define CRYPT_SAD_CNTR_SIZE_SHIFT	(0)
/**
* Note:
*   CRYPT_SAD_CTS_MODE_MASK is allowed to overlap with CRYPT_SAD_CNTR_SIZE_MASK,
*   since counter-based cipher modes never need to support cipher-text stealing
*/
#define CRYPT_SAD_CTS_MODE_MASK		(0x3)    /* CTS Mode */
#define CRYPT_SAD_CTS_MODE_SHIFT	(0)
/* Word 1 - Encryption offsets */
/* Word 2 */
#define CRYPT_SAD_ENC_OFF_MASK		(0xffff<<16)/*Enc off- start of pkt*/
#define CRYPT_SAD_ENC_OFF_SHIFT		(16)
#define CRYPT_SAD_ENC_OFF_EOP_MASK	(0xffff)/*Enc off- end of pkt*/
#define CRYPT_SAD_ENC_OFF_EOP_SHIFT	(0)

/* Authentication offsets */
#define CRYPT_SAD_AUTH_OFF_MASK		(0xffff<<16) /*Auth off- start of pkt*/
#define CRYPT_SAD_AUTH_OFF_SHIFT	(16)
#define CRYPT_SAD_AUTH_OFF_EOP_MASK	(0xffff)    /*Auth off- end of pkt*/
#define CRYPT_SAD_AUTH_OFF_EOP_SHIFT	(0)

/* Other words */
#define CRYPT_SAD_ENC_KEY_SWORD		(4)  /* Encryption Key */
#define CRYPT_SAD_ENC_KEY_SIZE		(8)
#define CRYPT_SAD_ENC_IV_SWORD		(12) /* Encryption IV */
#define CRYPT_SAD_ENC_IV_SIZE		(4)  /* Engine update this field */
#define CRYPT_SAD_GCM_AUTH_IV_SWORD	(16) /* GCM Auth IV */
#define CRYPT_SAD_GCM_AUTH_IV_SIZE	(4)
#define CRYPT_SAD_AUTH_IV_SWORD		(12) /* Auth Only IV */
#define CRYPT_SAD_AUTH_IV_SIZE		(16) /* Engine update this field */
#define CRYPT_SAD_HMAC_IV_IN_SWORD	(28) /* HMAC_IV_in H(k xor ipad) */
#define CRYPT_SAD_HMAC_IV_IN_SIZE	(16)
#define CRYPT_SAD_HMAC_IV_OUT_SWORD	(44) /* HMAC_IV_out H(k xor opad) */
#define CRYPT_SAD_HMAC_IV_OUT_SIZE	(16)
/* XTS support*/
#define CRYPT_SAD_XTS_SEC_SIZE_WORD	(16)   /* XTS sector size */
#define CRYPT_SAD_XTS_SEC_SIZE_MASK	(0xff) /*Auth off- start of pkt*/
#define CRYPT_SAD_XTS_SEC_SIZE_SHIFT	(0)
#define CRYPT_SAD_XTS_PROTECTION_WORD	(16)   /* XTS protection info (PI) size */
#define CRYPT_SAD_XTS_PROTECTION_MASK	(0xff<<8)
#define CRYPT_SAD_XTS_PROTECTION_SHIFT	(8)
#define CRYPT_SAD_XTS_TWEAK_KEY_SWORD	(20)   /* XTS Enc Tweak Key */
#define CRYPT_SAD_XTS_TWEAK_KEY_SIZE	(8)


#define sa_init_field(dest, val, mask, shift, str)\
	do {\
		al_assert(!((val << shift) & ~(mask)));\
		al_debug(" SA %s - %x\n", str, val); \
		dest |= (val << shift) & mask;\
	} while (0);
/**
 * DEBUG
 */
#ifdef CRYPTO_DEBUG
static void al_print_crypto_desc(union al_udma_desc *desc)
{
	al_dbg(" Crypto: Desc: %08x %08x %08x %08x\n",
			desc->tx_meta.len_ctrl, desc->tx_meta.meta_ctrl,
			desc->tx_meta.meta1, desc->tx_meta.meta2);
}

static
void al_print_crypto_xaction(struct al_crypto_transaction *xaction)
{
	unsigned int i;

	al_dbg("Crypto: Transaction debug\n");
	al_dbg(" Direction %s\n",
		(xaction->dir == AL_CRYPT_ENCRYPT) ? "Encrypt" : "Decrypt");
	al_dbg(" Flags %d\n", xaction->flags);

	al_dbg("-SRC buf size %d num of buffers  %d\n",
		xaction->src_size, xaction->src.num);
	for (i = 0 ; i < xaction->src.num; i++)
		al_dbg(" addr 0x%016" PRIx64 " len %d\n",
			xaction->src.bufs[i].addr,
			xaction->src.bufs[i].len);

	al_dbg("-DST num of buffers  %d\n",
			xaction->dst.num);
	for (i = 0 ; i < xaction->dst.num; i++)
		al_dbg(" addr 0x%016" PRIx64 " len %d\n",
			xaction->dst.bufs[i].addr,
			xaction->dst.bufs[i].len);

	al_dbg("-SA index %d address 0x%016" PRIx64 " len %d\n",
		xaction->sa_indx, xaction->sa_in.addr,
		xaction->sa_in.len);
	al_dbg(" SA OUT size: %d , addr 0x%016" PRIx64 "\n",
		xaction->sa_out.len,
		xaction->sa_out.addr);

	al_dbg("-Enc IV IN size: %d, addr 0x%016" PRIx64 "\n",
		xaction->enc_iv_in.len,
		xaction->enc_iv_in.addr);
	al_dbg(" Enc IV OUT size: %d, addr 0x%016" PRIx64 "\n",
		xaction->enc_iv_out.len,
		xaction->enc_iv_out.addr);
	al_dbg(" Enc Next IV OUT size: %d, addr 0x%016" PRIx64 "\n",
		xaction->enc_next_iv_out.len,
		xaction->enc_next_iv_out.addr);
	al_dbg(" Enc Offset %d Len %d\n",
		xaction->enc_in_off, xaction->enc_in_len);

	al_dbg("-Auth fl_valid %d, first %d last %d\n",
		xaction->auth_fl_valid, xaction->auth_first,
		xaction->auth_last);
	al_dbg(" Auth IV IN size: %d, addr 0x%016" PRIx64 "\n",
		xaction->auth_iv_in.len,
		xaction->auth_iv_in.addr);
	al_dbg(" Auth IV OUT size: %d, addr 0x%016" PRIx64 "\n",
		xaction->auth_iv_out.len,
		xaction->auth_iv_out.addr);
	al_dbg(" Auth SIGN IN size: %d, addr 0x%016" PRIx64 "\n",
		xaction->auth_sign_in.len,
		xaction->auth_sign_in.addr);
	al_dbg(" Auth SIGN OUT size: %d, addr 0x%016" PRIx64 "\n",
		xaction->auth_sign_out.len,
		xaction->auth_sign_out.addr);
	al_dbg(" Auth Offset %d Len %d\n",
		xaction->auth_in_off, xaction->auth_in_len);
	al_dbg(" Auth Byte Count %d\n",
		xaction->auth_bcnt);

}

#else
#define al_print_crypto_desc(x)
#define al_print_crypto_xaction(x)
#endif

/**
 * Memcpy to HW SA
 *
 * @param dst destination buffer
 * @param src source buffer
 * @param size size in words
 */
static
void al_crypto_sa_copy(uint32_t *dst, uint8_t *src, uint32_t size)
{
	uint32_t i;
	uint8_t *cdst = (uint8_t *)dst;
	for (i = 0; i < size*4; i++)
		cdst[i] = src[i];
}

/**
 * Get number of rx submission descriptors needed for crypto transaction
 *
 * we need rx descriptor for each destination buffer.
 * if the transaction doesn't have destination buffers, then one
 * descriptor is needed
 *
 * @param xaction transaction context
 *
 * @return number of rx submission descriptors
 */
static INLINE
uint32_t al_crypto_xaction_rx_descs_count(struct al_crypto_transaction *xaction)
{
	uint32_t count = xaction->dst.num + (xaction->sa_out.len ? 1 : 0) +
	       (xaction->enc_iv_out.len ? 1 : 0) +
	       ((xaction->enc_next_iv_out.len ||
			xaction->auth_iv_out.len) ? 1 : 0) +
	       (xaction->auth_sign_out.len ? 1 : 0);

	/* valid rx descs count */
	al_assert(count <= AL_SSM_MAX_DST_DESCS);

	return count;
}

/**
 * Get number of tx submission descriptors needed for crypto transaction
 *
 * we need tx descriptor for each source buffer.
 *
 * @param xaction transaction context
 *
 * @return number of tx submission descriptors
 */
static INLINE
uint32_t al_crypto_xaction_tx_descs_count(struct al_crypto_transaction *xaction)
{
	uint32_t count = xaction->src.num + (xaction->sa_in.len ? 1 : 0) +
		(xaction->enc_iv_in.len ? 1 : 0) +
		(xaction->auth_iv_in.len ? 1 : 0) +
		(xaction->auth_sign_in.len ? 1 : 0);

	/* valid tx descs count */
	al_assert(count);
	/* Need one for metadata if offsets are valid */
	count += (xaction->enc_in_len || xaction->auth_in_len) ? 1 : 0;
	/*valid tx descs count*/
	al_assert(count <= AL_SSM_MAX_SRC_DESCS);

	return count;
}

/**
 * Fill one rx submission descriptor
 *
 * @param rx_udma_q rx udma handle
 * @param flags flags for the descriptor
 * @param buf destination buffer
 * @param tgtid virtual machine ID
 */
static INLINE
void al_crypto_prep_one_rx_desc(struct al_udma_q *rx_udma_q,
		uint32_t flags, struct al_buf *buf, uint16_t tgtid)
{
	uint64_t tgtid_shifted = ((uint64_t)tgtid) << AL_UDMA_DESC_TGTID_SHIFT;
	uint32_t flags_len = flags;
	union al_udma_desc *rx_desc;
	uint32_t ring_id;

	rx_desc = al_udma_desc_get(rx_udma_q);
	/* get ring id */
	ring_id = al_udma_ring_id_get(rx_udma_q)
		<< AL_S2M_DESC_RING_ID_SHIFT;

	flags_len |= ring_id;

	flags_len |= buf->len & AL_S2M_DESC_LEN_MASK;
	rx_desc->rx.len_ctrl = swap32_to_le(flags_len);
	rx_desc->rx.buf1_ptr = swap64_to_le(buf->addr | tgtid_shifted);
	al_print_crypto_desc(rx_desc);
}

/**
 * Fill the crypto rx submission descriptors
 *
 * this function writes the contents of the rx submission descriptors
 *
 * @param rx_udma_q rx udma handle
 * @param xaction transaction context
 * @param rx_desc_cnt number of total rx descriptors
 */
static
void al_crypto_set_rx_descs(struct al_udma_q *rx_udma_q,
		struct al_crypto_transaction *xaction, uint32_t rx_desc_cnt)
{
	uint32_t flags;
	union al_udma_desc *rx_desc;
	uint32_t buf_idx;

	/* Set descriptor flags */
	flags = (xaction->flags & AL_SSM_INTERRUPT) ?
		AL_S2M_DESC_INT_EN : 0;
	flags |= (xaction->flags & AL_SSM_DEST_NO_SNOOP) ?
		AL_S2M_DESC_NO_SNOOP_H : 0;

	/* if the xaction doesn't have destination buffers,
	 * allocate single Meta descriptor,
	 */
	if (unlikely(!rx_desc_cnt)) {
		al_debug("Crypto: Preparing Meta Rx dec\n");
		rx_desc = al_udma_desc_get(rx_udma_q);
		flags |= al_udma_ring_id_get(rx_udma_q)
			<< AL_S2M_DESC_RING_ID_SHIFT;
		flags |= RX_DESC_META;
		/* write back flags */
		rx_desc->rx.len_ctrl = swap32_to_le(flags);
		al_print_crypto_desc(rx_desc);
		return;
	}

	/* prepare descriptors for the required feilds */
	if (unlikely(xaction->sa_out.len)) {
		al_debug("Crypto: Preparing SA out Rx desc\n");
		al_crypto_prep_one_rx_desc(
			rx_udma_q, flags, &xaction->sa_out, xaction->dst.tgtid);
	}

	if (unlikely(xaction->enc_iv_out.len)) {
		al_debug("Crypto: Preparing ENC IV out Rx desc\n");
		al_crypto_prep_one_rx_desc(rx_udma_q, flags,
				&xaction->enc_iv_out, xaction->dst.tgtid);
	}

	if (xaction->dst.num) {
		struct al_buf *buf = xaction->dst.bufs;
		al_debug("Crypto: Preparing %d Crypto DST Rx desc\n",
				xaction->dst.num);
		for (buf_idx = 0; buf_idx < xaction->dst.num; buf_idx++) {
			al_crypto_prep_one_rx_desc(
				rx_udma_q, flags, buf, xaction->dst.tgtid);
			buf++;
		}
	}

	/*
	 * IV output:Encryption IV next to use or In case of auth only SA and
	 * auth_last isnt set, this is the intermidiate auto output.
	 */
	if (xaction->enc_next_iv_out.len) {
		al_debug("Crypto: Preparing ENC Next IV OUT Rx desc\n");
		al_crypto_prep_one_rx_desc(rx_udma_q, flags,
				&xaction->enc_next_iv_out, xaction->dst.tgtid);
	} else {
		if (xaction->auth_iv_out.len) {
			al_debug("Crypto: Preparing AUTH IV OUT Rx desc\n");
			al_crypto_prep_one_rx_desc(rx_udma_q, flags,
				&xaction->auth_iv_out, xaction->dst.tgtid);
		}
	}

	if (xaction->auth_sign_out.len) {
		al_debug("Crypto: Preparing SIGN out Rx desc\n");
		al_crypto_prep_one_rx_desc(rx_udma_q, flags,
				&xaction->auth_sign_out, xaction->dst.tgtid);
	}

}


/**
 * Fill one tx submission descriptor
 *
 * @param tx_udma_q tx udma handle
 * @param flags flags for the descriptor
 * @param meta metadata word1
 * @param buf source buffer
 * @param tgtid virtual machine ID
 */
static INLINE void al_crypto_prep_one_tx_desc(struct al_udma_q *tx_udma_q,
		uint32_t flags, uint32_t meta, struct al_buf *buf,
		uint16_t tgtid)
{
	uint64_t tgtid_shifted = ((uint64_t)tgtid) << AL_UDMA_DESC_TGTID_SHIFT;
	uint32_t flags_len = flags;
	union al_udma_desc *tx_desc;
	uint32_t ring_id;

	tx_desc = al_udma_desc_get(tx_udma_q);
	/* get ring id */
	ring_id = al_udma_ring_id_get(tx_udma_q)
		<< AL_M2S_DESC_RING_ID_SHIFT;

	flags_len |= ring_id;

	flags_len |= buf->len & AL_M2S_DESC_LEN_MASK;
	tx_desc->tx.len_ctrl = swap32_to_le(flags_len);
	tx_desc->tx.meta_ctrl = swap32_to_le(meta);
	tx_desc->tx.buf_ptr = swap64_to_le(buf->addr | tgtid_shifted);
	al_print_crypto_desc(tx_desc);
}

/**
 * Fill the crypto tx submission descriptors
 *
 * this function writes the contents of the tx submission descriptors
 *
 * @param tx_udma_q tx udma handle
 * @param xaction transaction context
 * @param tx_desc_cnt number of total tx descriptors
 */
static
void al_crypto_set_tx_descs(struct al_udma_q *tx_udma_q,
		struct al_crypto_transaction *xaction, uint32_t tx_desc_cnt)
{
	uint32_t flags;
	uint32_t buf_idx;
	uint32_t word1_meta;
	uint32_t desc_cnt = tx_desc_cnt;

	/* Set flags */
	flags = AL_M2S_DESC_FIRST;
	flags |= unlikely(xaction->flags & AL_SSM_SRC_NO_SNOOP) ?
		AL_M2S_DESC_NO_SNOOP_H : 0;

	/* Set first desc word1 metadata */
	word1_meta = AL_CRYPT_OP << TX_DESC_META_OP_SHIFT;
	word1_meta |= xaction->dir << TX_DESC_META_CRYPT_DIR_SHIFT;
	word1_meta |= unlikely(xaction->sa_out.len) ?
			TX_DESC_META_CRYPT_S_GET_SA : 0;
	word1_meta |= unlikely(xaction->enc_iv_out.len) ?
			TX_DESC_META_CRYPT_S_GET_ENCIV : 0;

	word1_meta |= unlikely(xaction->dst.num) ?
		TX_DESC_META_CRYPT_GET_ORIG : 0;

	word1_meta |=
		unlikely(xaction->enc_next_iv_out.len ||
			 xaction->auth_iv_out.len) ?
			TX_DESC_META_CRYPT_GET_AUTHIV : 0;

	word1_meta |= likely(xaction->auth_sign_out.len) ?
		TX_DESC_META_CRYPT_S_GET_SIGN : 0;

	if (unlikely(xaction->auth_fl_valid)) {
		word1_meta |= xaction->auth_first ? TX_DESC_META_AUTH_FIRST : 0;
		word1_meta |= xaction->auth_last ? TX_DESC_META_AUTH_LAST : 0;
	} else {
		word1_meta |= TX_DESC_META_AUTH_FIRST | TX_DESC_META_AUTH_LAST;
	}

	word1_meta |= unlikely(xaction->auth_sign_in.len) ?
		TX_DESC_META_AUTH_VALID : 0;

	word1_meta |= (xaction->sa_indx << TX_DESC_META_SA_IDX_SHIFT)
		& TX_DESC_META_SA_IDX_MASK;

	/* First Meta data desc */
	if ((xaction->enc_in_len) || (xaction->auth_in_len)) {
		uint32_t flags_len = flags;
		union al_udma_desc *tx_desc;
		uint32_t ring_id;
		uint32_t enc_meta;
		uint32_t auth_meta;

		al_debug("Crypto: preparing metadata desc: enc_in_len %d "
				"auth_in_len %d\n",
				xaction->enc_in_len, xaction->auth_in_len);
		al_debug("                  metadata desc: enc_in_off %d "
				"auth_in_off %d\n",
				xaction->enc_in_off, xaction->auth_in_off);
		/* having only metdata desc isnt valid */
		desc_cnt--;
		/* Valid desc count */
		al_assert(desc_cnt);

		tx_desc = al_udma_desc_get(tx_udma_q);
		/* UDMA feilds */
		ring_id = al_udma_ring_id_get(tx_udma_q)
			<< AL_M2S_DESC_RING_ID_SHIFT;
		flags_len |= ring_id;
		flags_len |= AL_M2S_DESC_META_DATA;
		tx_desc->tx_meta.len_ctrl = swap32_to_le(flags_len);
		/* Word1 metadata */
		tx_desc->tx_meta.meta_ctrl = 0;
		if (xaction->auth_bcnt) {
			/* Auth only, prev auth byte count */
			tx_desc->tx_meta.meta1 =
				swap32_to_le(xaction->auth_bcnt);
		} else {
			/* Encryption offsets */
			enc_meta = (xaction->src_size -
				(xaction->enc_in_len + xaction->enc_in_off))
				& TX_DESC_META_ENC_OFF_EOP_MASK;
			enc_meta |= (xaction->enc_in_off
					<< TX_DESC_META_ENC_OFF_SHIFT)
					& TX_DESC_META_ENC_OFF_MASK;

			tx_desc->tx_meta.meta1 = swap32_to_le(enc_meta);
		}
		/* Authentication offsets */
		auth_meta = (xaction->src_size -
			(xaction->auth_in_len + xaction->auth_in_off))
			& TX_DESC_META_AUTH_OFF_EOP_MASK;
		auth_meta |= (xaction->auth_in_off
				<< TX_DESC_META_AUTH_OFF_SHIFT)
				& TX_DESC_META_AUTH_OFF_MASK;
		tx_desc->tx_meta.meta2 = swap32_to_le(auth_meta);
		al_print_crypto_desc(tx_desc);
		/* clear first flag, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	flags |= unlikely(xaction->flags & AL_SSM_BARRIER) ?
		AL_M2S_DESC_DMB : 0;

	/* prepare descriptors for the SA_in if found */
	if (xaction->sa_in.len) {
		al_debug("Crypto: Preparing SA Tx desc sa_index %d\n",
				xaction->sa_indx);
		/* check for last */
		flags |= (desc_cnt == 1) ? AL_M2S_DESC_LAST : 0;
		desc_cnt--;
		/* update buffer type in metadata */
		word1_meta |=  AL_CRYPT_BUF_SA_UPDATE
			<< TX_DESC_META_BUF_TYPE_SHIFT;

		al_crypto_prep_one_tx_desc(tx_udma_q, flags, word1_meta,
				&xaction->sa_in, xaction->src.tgtid);
		word1_meta = 0;
		/* clear first and DMB flags, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	/* prepare descriptors for the enc_IV_in if found */
	if (likely(xaction->enc_iv_in.len)) {
		al_debug("Crypto: Preparing IV in Tx desc\n");
		/* check for last */
		flags |= (desc_cnt == 1) ? AL_M2S_DESC_LAST : 0;
		desc_cnt--;
		/* update buffer type in metadata */
		word1_meta |=  AL_CRYPT_BUF_ENC_IV
			<< TX_DESC_META_BUF_TYPE_SHIFT;

		al_crypto_prep_one_tx_desc(tx_udma_q, flags, word1_meta,
				&xaction->enc_iv_in, xaction->src.tgtid);
		word1_meta = 0;
		/* clear first and DMB flags, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	/* prepare descriptors for the auth_IV_in if found */
	if (unlikely(xaction->auth_iv_in.len)) {
		al_debug("Crypto: Preparing Auth IV in Tx desc\n");
		/* check for last */
		flags |= (desc_cnt == 1) ? AL_M2S_DESC_LAST : 0;
		desc_cnt--;
		/* update buffer type in metadata */
		word1_meta |=  AL_CRYPT_BUF_AUTH_IV
			<< TX_DESC_META_BUF_TYPE_SHIFT;

		al_crypto_prep_one_tx_desc(tx_udma_q, flags, word1_meta,
				&xaction->auth_iv_in, xaction->src.tgtid);
		word1_meta = 0;
		/* clear first and DMB flags, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	/* prepare descriptors for the source buffer if found */
	if (likely(xaction->src.num)) {
		struct al_buf *buf = xaction->src.bufs;
		al_debug("Crypto: Preparing SRC %d Tx desc\n",
				xaction->src.num);
		/* update buffer type in metadata */
		word1_meta |=  AL_CRYPT_BUF_SRC << TX_DESC_META_BUF_TYPE_SHIFT;

		for (buf_idx = 0; buf_idx < xaction->src.num; buf_idx++) {
			/* check for last */
			flags |= (desc_cnt == 1) ? AL_M2S_DESC_LAST : 0;
			desc_cnt--;

			al_crypto_prep_one_tx_desc(tx_udma_q, flags,
					word1_meta, buf, xaction->src.tgtid);
			word1_meta = 0;
			/* clear first and DMB flags, keep no snoop hint flag */
			flags &= AL_M2S_DESC_NO_SNOOP_H;
			flags |= AL_M2S_DESC_CONCAT;
			buf++;
		}

		/* clear first, concat and DMB flags, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	/* prepare descriptors for the auth signature if found */
	if (unlikely(xaction->auth_sign_in.len)) {
		al_debug("Crypto: Preparing Signature in Tx desc\n");
		/* check for last */
		flags |= (desc_cnt == 1) ? AL_M2S_DESC_LAST : 0;
		desc_cnt--;
		/* update buffer type in metadata */
		word1_meta |=  AL_CRYPT_BUF_AUTH_SIGN
			<< TX_DESC_META_BUF_TYPE_SHIFT;

		al_crypto_prep_one_tx_desc(tx_udma_q, flags, word1_meta,
				&xaction->auth_sign_in, xaction->src.tgtid);
		word1_meta = 0;
		/* clear first and DMB flags, keep no snoop hint flag */
		flags &= AL_M2S_DESC_NO_SNOOP_H;
	}

	al_assert(!desc_cnt);
}

/**
 * Initialize a single hw_sa
 *
 * @param sa crypto SW SA containing the desired SA parameters
 * @param hw_sa crypto HW SA filled with zeros
 *           to be initialized according to the sa
 * @param copy_only_ivs true -> copy only iv data, false -> copy everything
 */
static INLINE
void al_crypto_hw_sa_init_entry(struct al_crypto_sa *sa,
	struct al_crypto_hw_sa *hw_sa, al_bool copy_only_ivs)
{
	uint32_t tword;
	al_bool is_auth_type_sha3 = AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH) &&
				((sa->auth_type == AL_CRYPT_AUTH_SHA3) ||
				(sa->auth_type == AL_CRYPT_AUTH_SHA3_CONT));
	al_bool is_quad_sa_hmac = is_auth_type_sha3 && sa->auth_hmac_en;

	/* Words [4-63] */
	/* AES XTS */
	if (sa->enc_type == AL_CRYPT_AES_XTS) {
		al_crypto_sa_copy(
			&hw_sa->sa_word[CRYPT_SAD_XTS_TWEAK_KEY_SWORD],
			sa->enc_xts_tweak_key, CRYPT_SAD_XTS_TWEAK_KEY_SIZE);
		tword = 0;
		sa_init_field(tword,
			sa->enc_xts_sector_size,
			CRYPT_SAD_XTS_SEC_SIZE_MASK,
			CRYPT_SAD_XTS_SEC_SIZE_SHIFT,
			"valid xts sector size");
		sa_init_field(tword,
			sa->enc_xts_protection,
			CRYPT_SAD_XTS_PROTECTION_MASK,
			CRYPT_SAD_XTS_PROTECTION_SHIFT,
			"valid xts protection");
		hw_sa->sa_word[CRYPT_SAD_XTS_SEC_SIZE_WORD] = tword;
	}

	/* Encryption Key and IV, also relevant for GCM Auth */
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_ENC) || (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH)
			&& (sa->auth_type == AL_CRYPT_AUTH_AES_GCM))) {
		al_crypto_sa_copy(&hw_sa->sa_word[CRYPT_SAD_ENC_KEY_SWORD],
			sa->enc_key, CRYPT_SAD_ENC_KEY_SIZE);
		al_crypto_sa_copy(&hw_sa->sa_word[CRYPT_SAD_ENC_IV_SWORD],
			sa->enc_iv, CRYPT_SAD_ENC_IV_SIZE);
	}

	/* AES GCM IV */
	if (sa->enc_type == AL_CRYPT_AES_GCM) {
		al_crypto_sa_copy(&hw_sa->sa_word[CRYPT_SAD_GCM_AUTH_IV_SWORD],
			sa->aes_gcm_auth_iv, CRYPT_SAD_GCM_AUTH_IV_SIZE);
	}

	/* Authentication */
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH)) {
		if (sa->auth_hmac_en) {
			al_crypto_sa_copy(
				&hw_sa->sa_word[CRYPT_SAD_HMAC_IV_IN_SWORD],
				sa->hmac_iv_in, CRYPT_SAD_HMAC_IV_IN_SIZE);
			al_crypto_sa_copy(
				&hw_sa->sa_word[CRYPT_SAD_HMAC_IV_OUT_SWORD],
				sa->hmac_iv_out, CRYPT_SAD_HMAC_IV_OUT_SIZE);
		}

	}
	/* IV for broken Auth, overlap GCM feilds
	   which dont support broken Auth */
	if (AL_CRYPT_OP_IS(sa->sa_op, AL_CRYPT_AUTH) &&
				(sa->auth_type != AL_CRYPT_AUTH_AES_GCM)) {
		al_crypto_sa_copy(&hw_sa->sa_word[CRYPT_SAD_AUTH_IV_SWORD],
				sa->auth_iv_in, CRYPT_SAD_AUTH_IV_SIZE);
	}

	if(copy_only_ivs)
		return;

	/* Word 0 */
	tword = 0;
	/* Valid SA operation */
	al_assert(sa->sa_op);
	sa_init_field(tword, sa->sa_op, CRYPT_SAD_OP_MASK,
			CRYPT_SAD_OP_SHIFT, "valid sa_op");
	sa_init_field(tword, sa->sa_length, CRYPT_SAD_SA_LENGTH_MASK,
			CRYPT_SAD_SA_LENGTH_SHIFT, "valid sa_length");

	/* Make sure SA length is not single line only in SHA3 mode */
	al_assert((sa->sa_length == AL_CRYPT_SINGLE_LINE) || is_auth_type_sha3);
	/* Make sure SA length is not single line in quad SA HMAC mode */
	al_assert((!is_quad_sa_hmac) || (sa->sa_length != AL_CRYPT_SINGLE_LINE));

	/* Encryption */
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_ENC) ||
			(AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH) &&
			((sa->auth_type == AL_CRYPT_AUTH_AES_GCM) ||
			(sa->auth_type == AL_CRYPT_AUTH_AES_CCM)))) {
		sa_init_field(tword, sa->enc_type, CRYPT_SAD_ENC_TYPE_MASK,
			CRYPT_SAD_ENC_TYPE_SHIFT, "valid enc type");
		if ((sa->enc_type == AL_CRYPT_TRIPDES_ECB) ||
				(sa->enc_type == AL_CRYPT_TRIPDES_CBC)) {
			sa_init_field(tword,
				sa->tripdes_m,
				CRYPT_SAD_TRIPDES_MODE_MASK,
				CRYPT_SAD_TRIPDES_MODE_SHIFT,
				"valid 3des mode");
		}
		if (sa->enc_type > AL_CRYPT_TRIPDES_CBC) {
			sa_init_field(tword,
				sa->aes_ksize,
				CRYPT_SAD_AES_KEY_SIZE_MASK,
				CRYPT_SAD_AES_KEY_SIZE_SHIFT,
				"valid aes key size");
		}
		if ((sa->enc_type == AL_CRYPT_AES_CBC) ||
		    (sa->enc_type == AL_CRYPT_AES_ECB) ||
		    (sa->enc_type == AL_CRYPT_AES_XTS)) {
			sa_init_field(tword,
				sa->cts_mode,
				CRYPT_SAD_CTS_MODE_MASK,
				CRYPT_SAD_CTS_MODE_SHIFT,
				"valid ciphertext stealing mode");
		} else {
			sa_init_field(tword,
				sa->cntr_size,
				CRYPT_SAD_CNTR_SIZE_MASK,
				CRYPT_SAD_CNTR_SIZE_SHIFT,
				"valid counter loop");
		}
	}

	/* Authentication */
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH)) {
		sa_init_field(tword,
			sa->auth_type,
			CRYPT_SAD_AUTH_TYPE_MASK,
			CRYPT_SAD_AUTH_TYPE_SHIFT,
			"valid auth type");
		sa_init_field(tword,
			sa->signature_size,
			CRYPT_SAD_SIGN_SIZE_MASK,
			CRYPT_SAD_SIGN_SIZE_SHIFT,
			"valid sign size");
		if (sa->auth_type == AL_CRYPT_AUTH_SHA2)
			sa_init_field(tword,
				sa->sha2_mode,
				CRYPT_SAD_SHA2_MODE_MASK,
				CRYPT_SAD_SHA2_MODE_SHIFT,
				"valid sha2 mode");
		if (sa->auth_type == AL_CRYPT_AUTH_SHA3)
			sa_init_field(tword,
				sa->sha3_mode,
				CRYPT_SAD_SHA3_MODE_MASK,
				CRYPT_SAD_SHA3_MODE_SHIFT,
				"valid sha3 mode");
		if (sa->auth_type == AL_CRYPT_AUTH_SHA3_CONT)
			sa_init_field(tword,
				sa->sha3_mode_ext,
				CRYPT_SAD_SHA3_MODE_EXT_MASK,
				CRYPT_SAD_SHA3_MODE_EXT_SHIFT,
				"valid sha3 mode");
		tword |= sa->auth_signature_msb ? CRYPT_SAD_AUTH_MSB_BITS : 0;
		tword |= sa->auth_hmac_en ? CRYPT_SAD_HMAC_EN : 0;
	}

	/* Encryption + Authentication */
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_ENC) && AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH)) {
		tword |= sa->sign_after_enc ? CRYPT_SAD_SIGN_AFTER_ENC : 0;
		tword |= sa->auth_after_dec ? CRYPT_SAD_AUTH_AFTER_DEC : 0;
	}

	hw_sa->sa_word[0] = swap32_to_le(tword);

	/* Word 2 - Encryption offsets */
	tword = 0;
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_ENC)) {
		sa_init_field(tword,
			sa->enc_offset,
			CRYPT_SAD_ENC_OFF_MASK,
			CRYPT_SAD_ENC_OFF_SHIFT,
			"valid enc off");
		sa_init_field(tword,
			sa->enc_offset_eop,
			CRYPT_SAD_ENC_OFF_EOP_MASK,
			CRYPT_SAD_ENC_OFF_EOP_SHIFT,
			"valid enc off eop");
		hw_sa->sa_word[2] = swap32_to_le(tword);
	}

	/* Word 3 - Authentication offsets */
	tword = 0;
	if (AL_CRYPT_OP_HAS(sa->sa_op, AL_CRYPT_AUTH)) {
		sa_init_field(tword,
			sa->auth_offset,
			CRYPT_SAD_AUTH_OFF_MASK,
			CRYPT_SAD_AUTH_OFF_SHIFT,
			"valid auth off");
		sa_init_field(tword,
			sa->auth_offset_eop,
			CRYPT_SAD_AUTH_OFF_EOP_MASK,
			CRYPT_SAD_AUTH_OFF_EOP_SHIFT,
			"valid auth off eop");
		hw_sa->sa_word[3] = swap32_to_le(tword);
	}
}

/****************************** API functions *********************************/
void al_crypto_hw_sa_init(struct al_crypto_sa *sa,
			 struct al_crypto_hw_sa *hw_sa)
{
	uint32_t sa_idx;
	uint32_t sa_num_entries = (1 << sa->sa_length);

	/*
	 * Init the first entry with all data and the rest entries only with ivs
	 */
	for (sa_idx = 0; sa_idx < sa_num_entries; sa_idx++)
    		al_crypto_hw_sa_init_entry(&sa[sa_idx], &hw_sa[sa_idx],
    			sa_idx ? AL_TRUE : AL_FALSE);
}

/******************************************************************************
 *****************************************************************************/
int al_crypto_dma_prepare(struct al_ssm_dma *dma, uint32_t qid,
			   struct al_crypto_transaction *xaction)
{
	uint32_t rx_descs;
	uint32_t tx_descs;
	struct al_udma_q *rx_udma_q;
	struct al_udma_q *tx_udma_q;
	int rc;

	al_debug("al_crypto_dma_prepare\n");
	al_print_crypto_xaction(xaction);

	/* Check some parameters */
	/* SA out -> SA in */
	al_assert(!xaction->sa_out.len ||
		(xaction->sa_out.len && xaction->sa_in.len));
	/* Valid SA index */
	al_assert(!(xaction->sa_indx >> TX_DESC_META_SA_IDX_SHIFT
			& ~TX_DESC_META_SA_IDX_MASK));
	/* Auth first has no iv_in */
	al_assert(!(xaction->auth_fl_valid &&
		 xaction->auth_first && xaction->auth_iv_in.len));
	/* No last -> No sign_in */
	al_assert(!(xaction->auth_fl_valid &&
		!xaction->auth_last && xaction->auth_sign_in.len));
	/* Queue is for crypt/auth transactions */
	al_assert(dma->q_types[qid] == AL_CRYPT_AUTH_Q);

	/* calc tx (M2S) descriptors */
	tx_descs = al_crypto_xaction_tx_descs_count(xaction);
	rc = al_udma_q_handle_get(&dma->m2m_udma.tx_udma, qid, &tx_udma_q);
	/* valid crypto tx q handle */
	al_assert(!rc);

	if (unlikely(al_udma_available_get(tx_udma_q) < tx_descs
				+ AL_CRYPT_DESC_RES)) {
		uint32_t cdesc_count;

		/* cleanup tx completion queue */
		cdesc_count = al_udma_cdesc_get_all(tx_udma_q, NULL);
		if (likely(cdesc_count != 0))
			al_udma_cdesc_ack(tx_udma_q, cdesc_count);

		/* check again */
		if (unlikely(al_udma_available_get(tx_udma_q) < tx_descs
				+ AL_CRYPT_DESC_RES)) {
			al_dbg("crypt[%s]:tx q has no enough free desc\n",
					dma->m2m_udma.name);
			return -ENOSPC;
		}
	}

	/* calc rx (S2M) descriptors, at least one desc is required */
	rx_descs = al_crypto_xaction_rx_descs_count(xaction);
	rc = al_udma_q_handle_get(&dma->m2m_udma.rx_udma, qid, &rx_udma_q);
	/* valid crypto rx q handle */
	al_assert(!rc);
	if (unlikely(al_udma_available_get(rx_udma_q)
		     < (rx_descs ? rx_descs : 1))) {
		al_dbg("crypto [%s]: rx q has no enough free desc\n",
			 dma->m2m_udma.name);
		return -ENOSPC;
	}

	/* prepare rx descs */
	al_crypto_set_rx_descs(rx_udma_q, xaction, rx_descs);
	/* add rx descriptors */
	al_udma_desc_action_add(rx_udma_q, rx_descs ? rx_descs : 1);

	/* prepare tx descriptors */
	al_crypto_set_tx_descs(tx_udma_q, xaction, tx_descs);

	/* set number of tx descriptors */
	xaction->tx_descs_count = tx_descs;

	return 0;
}

/******************************************************************************
 *****************************************************************************/
int al_crypto_dma_action(struct al_ssm_dma *dma, uint32_t qid,
		int tx_descs)
{
	return al_ssm_dma_action(dma, qid, tx_descs);
}

/******************************************************************************
 *****************************************************************************/
int al_crypto_dma_completion(struct al_ssm_dma *dma, uint32_t qid,
			      uint32_t *comp_status)
{
	struct al_udma_q *rx_udma_q;
	volatile union al_udma_cdesc *cdesc;
	int rc;
	uint32_t cdesc_count;

	rc = al_udma_q_handle_get(&dma->m2m_udma.rx_udma, qid, &rx_udma_q);
	/* valid comp rx q handle */
	al_assert(!rc);

	cdesc_count = al_udma_cdesc_packet_get(rx_udma_q, &cdesc);
	if (!cdesc_count)
		return 0;

	/* if we have multiple completion descriptors,
	   then last one will have the valid status */
	if (unlikely(cdesc_count > 1))
		cdesc = al_cdesc_next(rx_udma_q, cdesc, cdesc_count - 1);

	*comp_status = swap32_from_le(cdesc->al_desc_comp_rx.ctrl_meta) &
		RX_COMP_STATUS_MASK;

	al_udma_cdesc_ack(rx_udma_q, cdesc_count);

	al_debug("crypto packet completed. count %d status desc %p meta %x\n",
			cdesc_count, cdesc, cdesc->al_desc_comp_rx.ctrl_meta);

	return 1;
}

void al_crypto_xts_cfg_set(
	void __iomem			*app_regs,
	const struct al_crypto_xts_cfg	*cfg)
{
	static const struct al_crypto_xts_cfg cfg_default = { .swap_pre_inc_bytes = AL_TRUE };
	struct crypto_regs __iomem *regs = (struct crypto_regs __iomem *)app_regs;

	al_assert(app_regs);

	if (!cfg)
		cfg = &cfg_default;

	al_reg_write32(&regs->xts_conf.tweak_calc_swap,
		(cfg->swap_pre_inc_bits ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_BITS : 0) |
		(cfg->swap_pre_inc_bytes ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_BYTES : 0) |
		(cfg->swap_pre_inc_words ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_PRE_INC_WORDS : 0) |
		(cfg->swap_post_inc_bits ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_BITS : 0) |
		(cfg->swap_post_inc_bytes ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_BYTES : 0) |
		(cfg->swap_post_inc_words ? CRYPTO_XTS_CONF_TWEAK_CALC_SWAP_POST_INC_WORDS : 0));
}

