/* al_eth.h: AnnapurnaLabs Unified 1GbE and 10GbE ethernet driver header.
 *
 * Copyright (c) 2013 AnnapurnaLabs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#ifndef AL_ETH_H
#define AL_ETH_H

#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/inetdevice.h>

#include "kcompat.h"

#include "al_hal_eth.h"
#ifdef CONFIG_ARCH_ALPINE
#include <soc/alpine/al_hal_udma_iofic.h>
#include <soc/alpine/al_hal_udma_debug.h>
#include <soc/alpine/al_serdes.h>
#else
#include "al_hal_udma_iofic.h"
#include "al_hal_udma_debug.h"
#include "al_serdes.h"
#endif
#include "al_init_eth_lm.h"

#ifndef BIT
#define BIT(nr)                 (1UL << (nr))
#endif

enum board_t {
	ALPINE_INTEGRATED = 0,
	ALPINE_NIC = 1,
	ALPINE_NIC_V2_10 = 2,
	ALPINE_NIC_V2_25 = 3,
	ALPINE_NIC_V2_25_DUAL = 4,
};

enum al_eth_direction {
	AL_ETH_TX,
	AL_ETH_RX
};

#define AL_ETH_MAX_HW_QUEUES	4
#define AL_ETH_NUM_QUEUES	4
#define AL_ETH_MAX_MSIX_VEC	(1 + 2 * AL_ETH_MAX_HW_QUEUES)

#define AL_ETH_DEFAULT_TX_SW_DESCS	(2*512)
#define AL_ETH_DEFAULT_TX_HW_DESCS	(2*512)
#define AL_ETH_DEFAULT_RX_DESCS		(2*512)

#if ((AL_ETH_DEFAULT_TX_SW_DESCS / 4) < (MAX_SKB_FRAGS + 2))
#define AL_ETH_TX_WAKEUP_THRESH		(AL_ETH_DEFAULT_TX_SW_DESCS / 4)
#else
#define AL_ETH_TX_WAKEUP_THRESH		(MAX_SKB_FRAGS + 2)
#endif
#define AL_ETH_DEFAULT_SMALL_PACKET_LEN		(128 - NET_IP_ALIGN)
#define AL_ETH_HEADER_COPY_SIZE			(128 - NET_IP_ALIGN)

#define AL_ETH_DEFAULT_MAX_RX_BUFF_ALLOC_SIZE 1536
/*
 * minimum the buffer size to 600 to avoid situation the mtu will be changed
 * from too little buffer to very big one and then the number of buffer per
 * packet could reach the maximum AL_ETH_PKT_MAX_BUFS
 */
#define AL_ETH_DEFAULT_MIN_RX_BUFF_ALLOC_SIZE 600
#define AL_ETH_DEFAULT_FORCE_1000_BASEX AL_FALSE

#define AL_ETH_DEFAULT_LINK_POLL_INTERVAL 100
#define AL_ETH_FIRST_LINK_POLL_INTERVAL 1

#define AL_ETH_NAME_MAX_LEN	20
#define AL_ETH_IRQNAME_SIZE	40

#define AL_ETH_DEFAULT_MDIO_FREQ_KHZ	2500

#define AL_ETH_MIN_MTU			68
#define AL_ETH_MAX_MTU			9216

#define AL_ETH_INTR_MODERATION_RESOLUTION   1 /* equivalent to ~1us in Alpine v2 */

#define AL_ETH_INTR_NO_INTERVAL            0
#define AL_ETH_INTR_NO_INTERVAL_USECS      0
#define AL_ETH_INTR_NO_INTERVAL_PKTS       4
#define AL_ETH_INTR_NO_INTERVAL_BYTES      4*1024

#define AL_ETH_INTR_LOWEST_USECS           0

#define AL_ETH_INTR_LOWEST_PKTS            3
#define AL_ETH_INTR_LOWEST_BYTES           2*1524

#define AL_ETH_INTR_LOW_USECS              32

#define AL_ETH_INTR_LOW_PKTS               12
#define AL_ETH_INTR_LOW_BYTES              16*1024

#define AL_ETH_INTR_MID_USECS              80

#define AL_ETH_INTR_MID_PKTS               48
#define AL_ETH_INTR_MID_BYTES              64*1024

#define AL_ETH_INTR_HIGH_USECS             128

#define AL_ETH_INTR_HIGH_PKTS              96
#define AL_ETH_INTR_HIGH_BYTES             128*1024

#define AL_ETH_INTR_HIGHEST_USECS          192

#define AL_ETH_INTR_HIGHEST_PKTS           128
#define AL_ETH_INTR_HIGHEST_BYTES          192*1024

#define AL_ETH_INTR_INITIAL_TX_INTERVAL_USECS		196
#define AL_ETH_INTR_INITIAL_RX_INTERVAL_USECS		0

struct al_eth_irq {
	irq_handler_t	handler;
	void		*data;
	unsigned int	vector;
	u8		requested;
	cpumask_t	affinity_hint_mask;
	char		name[AL_ETH_IRQNAME_SIZE];
};

struct al_eth_napi {
	struct napi_struct	napi		____cacheline_aligned;
	struct al_eth_adapter	*adapter;
#ifndef HAVE_NETDEV_NAPI_LIST
        struct net_device poll_dev;
#endif
	unsigned int qid;
};

enum al_eth_intr_moderation_level {
	AL_ETH_INTR_MODERATION_LOWEST = 0,
	AL_ETH_INTR_MODERATION_LOW,
	AL_ETH_INTR_MODERATION_MID,
	AL_ETH_INTR_MODERATION_HIGH,
	AL_ETH_INTR_MODERATION_HIGHEST,
	AL_ETH_INTR_MAX_NUM_OF_LEVELS,
};

struct al_eth_intr_moderation_entry {
	unsigned int intr_moderation_interval;
	unsigned int packets_per_interval;
	unsigned int bytes_per_interval;
};

struct al_eth_tx_buffer {
	struct sk_buff *skb;
	struct al_eth_pkt hal_pkt;
	unsigned int	tx_descs;
};

struct al_eth_rx_buffer {
	struct sk_buff *skb;
	struct page *page;
	unsigned int page_offset;
#if (defined(CONFIG_AL_ETH_ALLOC_FRAG) || defined(CONFIG_AL_ETH_ALLOC_SKB))
	u8 *data;
	unsigned int data_size;
	unsigned int frag_size; /* used in rx skb allocation */
#endif
	DEFINE_DMA_UNMAP_ADDR(dma);
	struct al_buf	al_buf;
};

#define AL_ETH_RX_OFFSET	NET_SKB_PAD + NET_IP_ALIGN

struct al_eth_stats_tx {
	u64 packets;
	u64 bytes;
};

struct al_eth_stats_rx {
	u64 packets;
	u64 bytes;
	u64 skb_alloc_fail;
	u64 dma_mapping_err;
	u64 small_copy_len_pkt;
};

struct al_eth_ring {
	struct device *dev;
	struct napi_struct	*napi;
	struct al_eth_pkt hal_pkt; /* used to get rx packets from hal */
	struct al_udma_q *dma_q; /* udma queue handler */
	u16 next_to_use;
	u16 next_to_clean;
	u32 __iomem *unmask_reg_offset; /* the offset of the interrupt unmask register */
	u32	unmask_val; /* the value to write to the above register to
			     * unmask the interrupt of this ring
			     */
	/* need to use union here */
	struct al_eth_meta_data hal_meta;
	struct al_eth_tx_buffer *tx_buffer_info; /* contex of tx packet */
	struct al_eth_rx_buffer *rx_buffer_info; /* contex of rx packet */
	int	sw_count; /* number of tx/rx_buffer_info's entries */
	int	hw_count; /* number of hw descriptors */
	size_t	descs_size; /* size (in bytes) of hw descriptors */
	size_t	cdescs_size; /* size (in bytes) of hw completion descriptors, used
			 for rx */
	enum al_eth_intr_moderation_level moderation_table_indx; /* interrupt coalescing */
	unsigned int smoothed_interval;
	/** Adaptive packets & bytes thresholds for interrupt moderation */
	unsigned int packets;
	unsigned int bytes;

	struct net_device *netdev;
	struct al_udma_q_params	q_params;

	struct u64_stats_sync syncp;
	union {
		struct al_eth_stats_tx tx_stats;
		struct al_eth_stats_rx rx_stats;
	};
};


#define AL_ETH_TX_RING_IDX_NEXT(tx_ring, idx) (((idx) + 1) & (AL_ETH_DEFAULT_TX_SW_DESCS - 1))

#define AL_ETH_RX_RING_IDX_NEXT(rx_ring, idx) (((idx) + 1) & (AL_ETH_DEFAULT_RX_DESCS - 1))
#define AL_ETH_RX_RING_IDX_ADD(rx_ring, idx, n) (((idx) + (n)) & (AL_ETH_DEFAULT_RX_DESCS - 1))

/* flow control configuration */
#define AL_ETH_FLOW_CTRL_RX_FIFO_TH_HIGH	0x160
#define AL_ETH_FLOW_CTRL_RX_FIFO_TH_LOW		0x90
#define AL_ETH_FLOW_CTRL_QUANTA			0xffff
#define AL_ETH_FLOW_CTRL_QUANTA_TH		0x8000

#define AL_ETH_FLOW_CTRL_AUTONEG  BIT(0)
#define AL_ETH_FLOW_CTRL_RX_PAUSE BIT(1)
#define AL_ETH_FLOW_CTRL_TX_PAUSE BIT(2)

/* link configuration for 1G port */
struct al_eth_link_config {
	int old_link;
	/* Describes what we actually have. */
	int	active_duplex;
	int	active_speed;

	/* current flow control status */
	uint8_t flow_ctrl_active;
	/* supported configuration (can be changed from ethtool) */
	uint8_t flow_ctrl_supported;

	/* the following are not relevant to RGMII */
	bool	force_1000_base_x;
	bool	autoneg;
};

/* SFP detection event */
enum al_eth_sfp_detect_evt {
	/* No change (no connect, disconnect, or new SFP module */
	AL_ETH_SFP_DETECT_EVT_NO_CHANGE,
	/* SFP module connected */
	AL_ETH_SFP_DETECT_EVT_CONNECTED,
	/* SFP module disconnected */
	AL_ETH_SFP_DETECT_EVT_DISCONNECTED,
	/* SFP module replaced */
	AL_ETH_SFP_DETECT_EVT_CHANGED,
};

/* SFP detection status */
struct al_eth_sfp_detect_stat {
	/* Status is valid (i.e. rest of fields are valid) */
	bool			valid;
	bool			connected;
	uint8_t			sfp_10g;
	uint8_t			sfp_1g;
	uint8_t			sfp_cable_tech;

	bool			lt_en;
	bool			an_en;
	enum al_eth_mac_mode	mac_mode;
};

/* Retimer parameters */
struct al_eth_retimer_params {
	al_bool				exist;
	enum al_eth_retimer_type	type;
	uint8_t				bus_id;
	uint8_t				i2c_addr;
	enum al_eth_retimer_channel	channel;
	enum al_eth_retimer_channel	tx_channel;
};

struct al_eth_stats_dev {
	u64 tx_timeout;
	u64 interface_up;
	u64 interface_down;
};

/* board specific private data structure */
struct al_eth_adapter {
	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;
	enum board_t	board_type;
	u16 dev_id;
	u8 rev_id;

	/* Some features need tri-state capability,
	 * thus the additional *_CAPABLE flags.
	 */
	u32 flags;
#define AL_ETH_FLAG_MSIX_CAPABLE		(u32)(1 << 1)
#define AL_ETH_FLAG_MSIX_ENABLED		(u32)(1 << 2)
#define AL_ETH_FLAG_IN_NETPOLL			(u32)(1 << 3)
#define AL_ETH_FLAG_MQ_CAPABLE			(u32)(1 << 4)
#define AL_ETH_FLAG_SRIOV_CAPABLE		(u32)(1 << 5)
#define AL_ETH_FLAG_SRIOV_ENABLED		(u32)(1 << 6)
#define AL_ETH_FLAG_RESET_REQUESTED		(u32)(1 << 7)

	struct al_hal_eth_adapter hal_adapter;

	/*
	 * rx packets that shorter that this len will be copied to the skb
	 * header
	 */
	unsigned int small_copy_len;

	/* Maximum size for rx buffer */
	unsigned int max_rx_buff_alloc_size;

	/* Tx fast path data */
	int num_tx_queues;

	/* Rx fast path data */
	int num_rx_queues;

	/* TX */
	struct al_eth_ring tx_ring[AL_ETH_NUM_QUEUES] ____cacheline_aligned_in_smp;

	/* RX */
	struct al_eth_ring rx_ring[AL_ETH_NUM_QUEUES];

#define AL_ETH_RXQ_NAPI_IDX(adapter, q)	(q)
#define AL_ETH_TXQ_NAPI_IDX(adapter, q)	((adapter)->num_rx_queues + (q))
	struct al_eth_napi al_napi[2 * AL_ETH_NUM_QUEUES];

	enum al_iofic_mode int_mode;
	unsigned int adaptive_intr_rate;

#define AL_ETH_MGMT_IRQ_IDX		0
#define AL_ETH_RXQ_IRQ_IDX(adapter, q)	(1 + (q))
#define AL_ETH_TXQ_IRQ_IDX(adapter, q)	(1 + (adapter)->num_rx_queues + (q))
	struct al_eth_irq irq_tbl[AL_ETH_MAX_MSIX_VEC];
	struct msix_entry *msix_entries;
	int	msix_vecs;
	int	irq_vecs;
	struct al_eth_intr_moderation_entry intr_moderation_table[AL_ETH_INTR_MAX_NUM_OF_LEVELS];
	unsigned int tx_usecs, rx_usecs; /* interrupt coalescing */

	unsigned int tx_ring_count;
	unsigned int tx_descs_count;
	unsigned int rx_ring_count;
	unsigned int rx_descs_count;

	/* RSS*/
	uint32_t toeplitz_hash_key[AL_ETH_RX_HASH_KEY_NUM];
#define AL_ETH_RX_RSS_TABLE_SIZE	AL_ETH_RX_THASH_TABLE_SIZE
	uint8_t	 rss_ind_tbl[AL_ETH_RX_RSS_TABLE_SIZE];

	uint32_t msg_enable;
	struct al_eth_mac_stats mac_stats;

	enum al_eth_mac_mode	mac_mode;
	bool			mac_mode_set; /* Relevant only when 'auto_speed' is set */
	u8 mac_addr[ETH_ALEN];
	/* mdio and phy*/
	bool			phy_exist;
	struct mii_bus		*mdio_bus;
	struct phy_device	*phydev;
	uint8_t			phy_addr;
	struct al_eth_link_config	link_config;

	/* HAL layer data */
	int	id_number;
	char	name[AL_ETH_NAME_MAX_LEN];
	void __iomem	*internal_pcie_base; /* use for ALPINE_NIC devices */
	void __iomem	*udma_base;
	void __iomem	*ec_base;
	void __iomem	*mac_base;
	void __iomem	*serdes_base;

	unsigned int	udma_num;

	struct al_eth_flow_control_params flow_ctrl_params;

	struct al_eth_adapter_params eth_hal_params;

	struct delayed_work	link_status_task;
	uint32_t		link_poll_interval; /* task interval in mSec */

	bool			serdes_init;
	struct al_serdes_grp_obj	*serdes_obj;
	uint8_t			serdes_grp;
	uint8_t			serdes_lane;

	bool			an_en;	/* run kr auto-negotiation */
	bool			lt_en;	/* run kr link-training */

	al_bool		sfp_detection_needed; /**< true if need to run sfp detection */
	bool		auto_speed; /**< true if allowed to change SerDes speed configuration */
	uint8_t		i2c_adapter_id; /**< identifier for the i2c adapter to use to access SFP+ module */
	enum al_eth_ref_clk_freq	ref_clk_freq; /**< reference clock frequency */
	unsigned int	mdio_freq; /**< MDIO frequency [Khz] */
	enum al_eth_board_ext_phy_if	phy_if;

	bool up;

	bool				last_link;
	bool				last_establish_failed;
	struct al_eth_lm_context	lm_context;
	bool				use_lm;

	struct al_eth_group_lm_context	*group_lm_context;

	bool				dont_override_serdes; /**< avoid overriding serdes parameters
								   to preset static values */
	spinlock_t			serdes_config_lock;

	uint32_t wol;

	struct al_eth_retimer_params	retimer;

	struct u64_stats_sync syncp;
	struct al_eth_stats_dev dev_stats;

	bool				phy_fixup_needed;

	enum al_eth_lm_max_speed	max_speed;
	bool				speed_detection;
	al_bool				auto_fec_enable;
	unsigned int			gpio_spd_1g;
	unsigned int			gpio_spd_10g;
	unsigned int			gpio_spd_25g;
	unsigned int			gpio_sfp_present;
	al_bool				kr_fec_enable;

	struct 	work_struct 		reset_task;
};

#endif /* !(AL_ETH_H) */
