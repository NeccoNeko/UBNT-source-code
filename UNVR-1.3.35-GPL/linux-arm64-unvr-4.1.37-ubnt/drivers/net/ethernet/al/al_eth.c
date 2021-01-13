/* al_eth.c: AnnapurnaLabs Unified 1GbE and 10GbE ethernet driver.
 *
 * Copyright (c) 2012 AnnapurnaLabs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/stringify.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/ethtool.h>
#include <linux/rtl83xx.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#if defined(CONFIG_RFS_ACCEL) || defined(CONFIG_ARCH_ALPINE)
#include <linux/cpu_rmap.h>
#endif
#include <net/ip.h>
#include <net/tcp.h>
#include <net/checksum.h>
#include <linux/prefetch.h>
#include <linux/cache.h>
#include <linux/i2c.h>
#include <linux/u64_stats_sync.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "al_hal_eth.h"
#include "al_hal_serdes_25g.h"
#include "al_init_eth_lm.h"
#include "al_eth_group_lm.h"
#ifdef CONFIG_ARCH_ALPINE
#include "soc/alpine/alpine_serdes.h"
#include "soc/alpine/alpine_group_lm.h"
#else
#include "al_hal_serdes_hssp.h"
#endif

#include "al_eth.h"
#ifdef CONFIG_ARCH_ALPINE
#include <soc/alpine/al_hal_udma_iofic.h>
#include <soc/alpine/al_hal_udma_debug.h>
#include <soc/alpine/al_hal_udma_config.h>
#else
#include "al_hal_udma_iofic.h"
#include "al_hal_udma_debug.h"
#include "al_hal_udma_config.h"
#endif /* CONFIG_ARCH_ALPINE */
#include "al_eth_sysfs.h"
#include "al_hal_unit_adapter_regs.h"
#include "al_hal_eth_ec_regs.h"

#define DRV_MODULE_NAME	 "al_eth"
#ifndef DRV_MODULE_VERSION
#define DRV_MODULE_VERSION      "0.2"
#endif
#define DRV_MODULE_RELDATE      "Mar 29, 2016"

static char version[] =
	"AnnapurnaLabs unified 1GbE and 10GbE Ethernet Driver " DRV_MODULE_NAME " v" DRV_MODULE_VERSION " (" DRV_MODULE_RELDATE ")\n";

MODULE_AUTHOR("Saeed Bishara <saeed@annapurnaLabs.com>");
MODULE_DESCRIPTION("AnnapurnaLabs unified 1GbE and 10GbE Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_MODULE_VERSION);

/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT  (5*HZ)

/* Time in mSec to keep trying to read / write from MDIO in case of error */
#define MDIO_TIMEOUT_MSEC	100

/* Time in mSec to switch auto-FEC on init */
#define AUTO_FEC_INITIAL_TIMEOUT	1000

/* Time in mSec to switch auto-FEC on toggle */
#define AUTO_FEC_TOGGLE_TIMEOUT		500

static int disable_msi;

module_param(disable_msi, int, 0);
MODULE_PARM_DESC(disable_msi, "Disable Message Signaled Interrupt (MSI)");

/** Default NIC mode port & udma numbers */
/** Linux kernel driver always uses port 0. On host driver it can be changes with loading of module in load.sh */
#define AL_ETH_DEFAULT_PORT  0

#if defined(CONFIG_ARCH_ALPINE) /** Linux kernerl driver uses udma 0 always */
#define AL_ETH_DEFAULT_UDMA  0
#else
#define AL_ETH_DEFAULT_UDMA  1  /** host driver uses udma 1 by default, can be changed in load.sh */
#endif

static int port_num = AL_ETH_DEFAULT_PORT;
static int udma_num = AL_ETH_DEFAULT_UDMA;
module_param(port_num, int, 0);
MODULE_PARM_DESC(port_num, "eth port to use in NIC mode");

module_param(udma_num, int, 0);
MODULE_PARM_DESC(udma_num, "udma number to use in NIC mode");


#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

#define IS_NIC(board_type) (((board_type) == ALPINE_NIC)  || \
		((board_type) == ALPINE_NIC_V2_10) || ((board_type) == ALPINE_NIC_V2_25) || \
		((board_type) == ALPINE_NIC_V2_25_DUAL))

/* indexed by board_t */
static struct {
	char *name;
	unsigned int bar; /* needed for NIC modes */
} board_info[] = {
	{
		.name = "AnnapurnaLabs unified 1Gbe/10Gbe" },
	{
		.name = "AnnapurnaLabs unified 1Gbe/10Gbe pcie NIC",
		.bar = 5,
	},
	{
		.name = "AnnapurnaLabs unified 1Gbe/10Gbe pcie NIC",
		.bar = 5,
	},
	{
		.name = "AnnapurnaLabs unified 10Gbe/25Gbe pcie NIC",
		.bar = 5,
	},
	{
		.name = "AnnapurnaLabs unified dual port 10Gbe/25Gbe pcie NIC",
		.bar = 5,
	},
};

static DEFINE_PCI_DEVICE_TABLE(al_eth_pci_tbl) = {
#ifdef PCI_DEVICE_ID_AL_ETH
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_INTEGRATED},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_ADVANCED
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_ADVANCED,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_INTEGRATED},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_NIC
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_NIC,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_NIC},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_NIC_V2_10
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_NIC_V2_10,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_NIC_V2_10},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_NIC_V2_25_R0A
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_NIC_V2_25_R0A,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_NIC_V2_25},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_NIC_V2_25_R0A1
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_NIC_V2_25_R0A1,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_NIC_V2_25},
#endif
#ifdef PCI_DEVICE_ID_AL_ETH_NIC_V2_K2C
	{ PCI_VENDOR_ID_ANNAPURNA_LABS, PCI_DEVICE_ID_AL_ETH_NIC_V2_K2C,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALPINE_NIC_V2_25_DUAL},
#endif
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, al_eth_pci_tbl);

#ifdef CONFIG_AL_ETH_ALLOC_SKB
static DEFINE_PER_CPU(struct sk_buff_head, rx_recycle_cache);
#endif

/* the following defines should be moved to hal */
#define AL_ETH_CTRL_TABLE_PRIO_SEL_SHIFT	0
#define AL_ETH_CTRL_TABLE_PRIO_SEL_MASK		(0xF << AL_ETH_CTRL_TABLE_PRIO_SEL_SHIFT)
#define AL_ETH_CTRL_TABLE_PRIO_SEL_0		(12 << AL_ETH_CTRL_TABLE_PRIO_SEL_SHIFT)

#define AL_ETH_CTRL_TABLE_Q_SEL_SHIFT	4
#define AL_ETH_CTRL_TABLE_Q_SEL_MASK	(0xF << AL_ETH_CTRL_TABLE_Q_SEL_SHIFT)
#define AL_ETH_CTRL_TABLE_Q_SEL_THASH	(1 << AL_ETH_CTRL_TABLE_Q_SEL_SHIFT)

#define AL_ETH_CTRL_TABLE_Q_PRIO_SEL_SHIFT	8
#define AL_ETH_CTRL_TABLE_Q_PRIO_SEL_MASK	(0x3 << AL_ETH_CTRL_TABLE_Q_PRIO_SEL_SHIFT)
/* selected queue is hash output table */
#define AL_ETH_CTRL_TABLE_Q_PRIO_SEL_Q		(3 << AL_ETH_CTRL_TABLE_Q_PRIO_SEL_SHIFT)

#define AL_ETH_CTRL_TABLE_UDMA_SEL_SHIFT	10
#define AL_ETH_CTRL_TABLE_UDMA_SEL_MASK	(0xF << AL_ETH_CTRL_TABLE_UDMA_SEL_SHIFT)
/* select UDMA from rfw_default opt1 register */
#define AL_ETH_CTRL_TABLE_UDMA_SEL_DEF_1	(7 << AL_ETH_CTRL_TABLE_UDMA_SEL_SHIFT)
#define AL_ETH_CTRL_TABLE_UDMA_SEL_0	(15 << AL_ETH_CTRL_TABLE_UDMA_SEL_SHIFT)

#define AL_ETH_CTRL_TABLE_UDMA_SEL_MASK_INPUT	(1 << 14)

#define AL_ETH_CTRL_TABLE_USE_TABLE	(1 << 20)

#define AL_ETH_MAC_TABLE_UNICAST_IDX_BASE	0
#define AL_ETH_MAC_TABLE_UNICAST_MAX_COUNT	4
#define AL_ETH_MAC_TABLE_ALL_MULTICAST_IDX	(AL_ETH_MAC_TABLE_UNICAST_IDX_BASE + \
						AL_ETH_MAC_TABLE_UNICAST_MAX_COUNT)

#define AL_ETH_MAC_TABLE_DROP_IDX		(AL_ETH_FWD_MAC_NUM - 1)
#define AL_ETH_MAC_TABLE_BROADCAST_IDX		(AL_ETH_MAC_TABLE_DROP_IDX - 1)

#define MAC_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDR(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

#define AL_ETH_SERDES_25G_OFFSET 0x2000 /** Offset into 25g serdes from serdes base */

/** MDIO clause 45 helpers */
#define AL_ETH_MDIO_C45_DEV_MASK	0x1f0000
#define AL_ETH_MDIO_C45_DEV_SHIFT	16
#define AL_ETH_MDIO_C45_REG_MASK	0xffff

#define  AL_ETH_REF_CLK_FREQ_TO_HZ(ref_clk_freq)   \
	( ((ref_clk_freq) == AL_ETH_REF_FREQ_375_MHZ) ? 375000000 : (   \
	((ref_clk_freq) == AL_ETH_REF_FREQ_500_MHZ) ? 500000000 : (   \
	((ref_clk_freq) == AL_ETH_REF_FREQ_187_5_MHZ) ? 187500000 : ( \
	((ref_clk_freq) == AL_ETH_REF_FREQ_250_MHZ) ? 250000000 : (   \
	((ref_clk_freq) == AL_ETH_REF_FREQ_428_MHZ) ? 428000000 : 500000000)))))

/**
 * Interrupt moderation
 */

/* The interrupt coalescing resolution is (N+1) x 256 clock cycles of the IO Fabric (SB),
 * where N is the value of the mod_res field in the iofic register. sb_clk_freq is in KHz.
 * NOTICE: We use nsecs (instead of usecs) to avoid floating point arithmetic in the kernel.
 */
#define AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)	\
	((uint) (((AL_ETH_INTR_MODERATION_RESOLUTION + 1) * 256 * 1000000)	\
		/ (sb_clk_freq)))

#define AL_ETH_INTR_LOWEST_VALUE(sb_clk_freq)		((uint) (1000 * AL_ETH_INTR_LOWEST_USECS / \
	AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)))

#define AL_ETH_INTR_LOW_VALUE(sb_clk_freq)		((uint)(1000 * AL_ETH_INTR_LOW_USECS /	   \
	AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)))

#define AL_ETH_INTR_MID_VALUE(sb_clk_freq)		((uint)(1000 * AL_ETH_INTR_MID_USECS /	   \
	AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)))

#define AL_ETH_INTR_HIGH_VALUE(sb_clk_freq)		((uint)(1000 * AL_ETH_INTR_HIGH_USECS /	   \
	AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)))

#define AL_ETH_INTR_HIGHEST_VALUE(sb_clk_freq)		((uint)(1000 * AL_ETH_INTR_HIGHEST_USECS / \
	AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq)))

/** Enable (AL_TRUE) / disable (AL_FALSE) interrupt moderation */
#ifdef CONFIG_ARCH_ALPINE
#define AL_ETH_DEFAULT_ADAPTIVE_INT_MODERATION	AL_FALSE
#else
#define AL_ETH_DEFAULT_ADAPTIVE_INT_MODERATION	AL_TRUE
#endif

static int adaptive_int_moderation = AL_ETH_DEFAULT_ADAPTIVE_INT_MODERATION;
module_param(adaptive_int_moderation, int, 0);
MODULE_PARM_DESC(adaptive_int_moderation,
	"Adaptive interrupt moderation Enable/Disable. 0 = Disable, 1 = Enable");

static struct al_eth_intr_moderation_entry
default_moderation_table[AL_ETH_INTR_MAX_NUM_OF_LEVELS] = {
	{
		.packets_per_interval = AL_ETH_INTR_LOWEST_PKTS,
		.bytes_per_interval = AL_ETH_INTR_LOWEST_BYTES,
	},
	{
		.packets_per_interval =  AL_ETH_INTR_LOW_PKTS,
		.bytes_per_interval = AL_ETH_INTR_LOW_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_MID_PKTS,
		.bytes_per_interval = AL_ETH_INTR_MID_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_HIGH_PKTS,
		.bytes_per_interval = AL_ETH_INTR_HIGH_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_HIGHEST_PKTS,
		.bytes_per_interval = AL_ETH_INTR_HIGHEST_BYTES,
	},
};

static struct al_eth_intr_moderation_entry
initial_moderation_table[AL_ETH_INTR_MAX_NUM_OF_LEVELS] = {
	{
		.packets_per_interval = AL_ETH_INTR_LOWEST_PKTS,
		.bytes_per_interval = AL_ETH_INTR_LOWEST_BYTES,
	},
	{
		.packets_per_interval =  AL_ETH_INTR_LOW_PKTS,
		.bytes_per_interval = AL_ETH_INTR_LOW_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_MID_PKTS,
		.bytes_per_interval = AL_ETH_INTR_MID_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_HIGH_PKTS,
		.bytes_per_interval = AL_ETH_INTR_HIGH_BYTES,
	},
	{
		.packets_per_interval = AL_ETH_INTR_HIGHEST_PKTS,
		.bytes_per_interval = AL_ETH_INTR_HIGHEST_BYTES,
	},
};

/** Forward declarations */
#ifdef CONFIG_ARCH_ALPINE
static void al_eth_serdes_mode_set(struct al_eth_adapter *adapter);
#endif
static void al_eth_down(struct al_eth_adapter *adapter);
static int al_eth_up(struct al_eth_adapter *adapter);

struct al_udma *al_eth_udma_get(struct al_eth_adapter *adapter, int tx)
{
	if (tx)
		return &adapter->hal_adapter.tx_udma;
	return &adapter->hal_adapter.rx_udma;
}

static int
al_eth_udma_queue_enable(struct al_eth_adapter *adapter, enum al_udma_type type,
	int qid)
{
	int rc = 0;
	char *name = (type == UDMA_TX) ? "Tx" : "Rx";
	struct al_udma_q_params *q_params;

	if (type == UDMA_TX)
		q_params = &adapter->tx_ring[qid].q_params;
	else
		q_params = &adapter->rx_ring[qid].q_params;

	rc = al_eth_queue_config(&adapter->hal_adapter, type, qid, q_params);
	if (rc < 0) {
		netdev_err(adapter->netdev, "config %s queue %u failed\n", name, qid);
		return rc;
	}
#if 0 /* queue enable not implemented yet */
	rc = al_eth_queue_enable(&adapter->hal_adapter, type, qid);

	if (rc < 0)
		netdev_err(adapter->netdev, "enable %s queue %u failed\n", name,
			   qid);
#endif
	return rc;
}

static int al_eth_udma_queues_enable_all(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		al_eth_udma_queue_enable(adapter, UDMA_TX, i);

	for (i = 0; i < adapter->num_rx_queues; i++)
		al_eth_udma_queue_enable(adapter, UDMA_RX, i);
	return 0;
}

static int al_eth_udma_queues_reset_all(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		if (adapter->tx_ring[i].dma_q && adapter->tx_ring[i].dma_q->q_regs)
			al_udma_q_reset(adapter->tx_ring[i].dma_q);

	for (i = 0; i < adapter->num_rx_queues; i++)
		if (adapter->rx_ring[i].dma_q && adapter->rx_ring[i].dma_q->q_regs)
			al_udma_q_reset(adapter->rx_ring[i].dma_q);
	return 0;
}

/* init FSM, no tunneling supported yet, if packet is tcp/udp over ipv4/ipv6, use 4 tuple hash */
static void
al_eth_fsm_table_init(struct al_eth_adapter *adapter)
{
	uint32_t val;
	int i;

	for (i = 0; i < AL_ETH_RX_FSM_TABLE_SIZE; i++) {
		uint8_t outer_type = AL_ETH_FSM_ENTRY_OUTER(i);
		switch (outer_type) {
		case AL_ETH_FSM_ENTRY_IPV4_TCP:
		case AL_ETH_FSM_ENTRY_IPV4_UDP:
		case AL_ETH_FSM_ENTRY_IPV6_TCP:
		case AL_ETH_FSM_ENTRY_IPV6_UDP:
			val = AL_ETH_FSM_DATA_OUTER_4_TUPLE | AL_ETH_FSM_DATA_HASH_SEL;
			break;
		case AL_ETH_FSM_ENTRY_IPV6_NO_UDP_TCP:
		case AL_ETH_FSM_ENTRY_IPV4_NO_UDP_TCP:
			val = AL_ETH_FSM_DATA_OUTER_2_TUPLE | AL_ETH_FSM_DATA_HASH_SEL;
			break;
		default:
			val = (0 << AL_ETH_FSM_DATA_DEFAULT_Q_SHIFT |
				((1 << adapter->udma_num) << AL_ETH_FSM_DATA_DEFAULT_UDMA_SHIFT));
		}
		al_eth_fsm_table_set(&adapter->hal_adapter, i, val);
	}
}

/**
 * MAC tables
 */

static void al_eth_mac_table_unicast_add(
	struct al_eth_adapter *adapter,
	uint8_t idx,
	uint8_t *addr,
	uint8_t udma_mask)
{
	struct al_eth_fwd_mac_table_entry entry = { { 0 } };

	memcpy(entry.addr, adapter->mac_addr, sizeof(adapter->mac_addr));

	memset(entry.mask, 0xff, sizeof(entry.mask));
	entry.rx_valid = true;
	entry.tx_valid = false;
	entry.udma_mask = udma_mask;
	entry.filter = false;

	netdev_dbg(adapter->netdev, "%s: [%d]: addr "MAC_ADDR_STR" mask "MAC_ADDR_STR"\n",
		__func__, idx, MAC_ADDR(entry.addr), MAC_ADDR(entry.mask));

	al_eth_fwd_mac_table_set(&adapter->hal_adapter, idx, &entry);
}

static void al_eth_mac_table_broadcast_add(
	struct al_eth_adapter	*adapter,
	uint8_t idx,
	uint8_t udma_mask)
{
	struct al_eth_fwd_mac_table_entry entry = { { 0 } };

	memset(entry.addr, 0xff, sizeof(entry.addr));
	memset(entry.mask, 0xff, sizeof(entry.mask));

	entry.rx_valid = true;
	entry.tx_valid = false;
	entry.udma_mask = udma_mask;
	entry.filter = false;

	netdev_dbg(adapter->netdev, "%s: [%d]: addr "MAC_ADDR_STR" mask "MAC_ADDR_STR"\n",
		__func__, idx, MAC_ADDR(entry.addr), MAC_ADDR(entry.mask));

	al_eth_fwd_mac_table_set(&adapter->hal_adapter, idx, &entry);
}

#ifdef HAVE_SET_RX_MODE
static void al_eth_mac_table_all_multicast_add(
	struct al_eth_adapter *adapter,
	uint8_t idx,
	uint8_t udma_mask)
{
	struct al_eth_fwd_mac_table_entry entry = { { 0 } };

	memset(entry.addr, 0x00, sizeof(entry.addr));
	memset(entry.mask, 0x00, sizeof(entry.mask));
	entry.mask[0] |= BIT(0);
	entry.addr[0] |= BIT(0);

	entry.rx_valid = true;
	entry.tx_valid = false;
	entry.udma_mask = udma_mask;
	entry.filter = false;

	netdev_dbg(adapter->netdev, "%s: [%d]: addr "MAC_ADDR_STR" mask "MAC_ADDR_STR"\n",
		__func__, idx, MAC_ADDR(entry.addr), MAC_ADDR(entry.mask));

	al_eth_fwd_mac_table_set(&adapter->hal_adapter, idx, &entry);
}
#endif

static void al_eth_mac_table_promiscuous_set(
	struct al_eth_adapter *adapter,
	al_bool promiscuous,
	uint8_t udma_mask)
{
	struct al_eth_fwd_mac_table_entry entry = { { 0 } };

	memset(entry.addr, 0x00, sizeof(entry.addr));
	memset(entry.mask, 0x00, sizeof(entry.mask));

	entry.rx_valid = true;
	entry.tx_valid = false;
	entry.udma_mask = (promiscuous) ? udma_mask : 0;
	entry.filter = (promiscuous) ? false : true;

	netdev_dbg(adapter->netdev, "%s: %s promiscuous mode\n",
		__func__, (promiscuous) ? "enter" : "exit");

	al_eth_fwd_mac_table_set(&adapter->hal_adapter,
		 AL_ETH_MAC_TABLE_DROP_IDX,
		 &entry);
}

static void al_eth_mac_table_entry_clear(
	struct al_eth_adapter *adapter,
	uint8_t idx)
{
	struct al_eth_fwd_mac_table_entry entry = { { 0 } };

	netdev_dbg(adapter->netdev, "%s: clear entry %d\n", __func__, idx);

	al_eth_fwd_mac_table_set(&adapter->hal_adapter, idx, &entry);
}

static int al_eth_get_serdes_25g_speed(struct al_eth_adapter *adapter, uint *speed)
{
	struct al_serdes_grp_obj	*serdes_obj;
	enum al_serdes_group_mode serdes_group_mode;
	uint32_t serdes_25g_gen_ctrl;

	if (unlikely(!adapter->serdes_base)) {
		netdev_err(adapter->netdev, "%s: serdes base isn't initialized\n", __func__);
		return -EINVAL;
	}

	if (unlikely(!adapter->serdes_obj)) {
		netdev_err(adapter->netdev, "%s: Not a valid serdes obj\n", __func__);
		return -EINVAL;
	}

	serdes_obj =  adapter->serdes_obj;

	serdes_25g_gen_ctrl = serdes_obj->serdes_mode_get(adapter->serdes_obj, &serdes_group_mode);

	if (serdes_group_mode == AL_SRDS_CFG_ETH_25G)
		*speed = 25000;
	else if (serdes_group_mode == AL_SRDS_CFG_KR)
		*speed = 10000;
	else
		return -EIO;

	return 0;
}

static int al_eth_board_params_init(struct al_eth_adapter *adapter)
{
	if (adapter->board_type == ALPINE_NIC) {
		adapter->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		adapter->sfp_detection_needed = false;
		adapter->phy_exist = false;
		adapter->an_en = false;
		adapter->lt_en = false;
		adapter->ref_clk_freq = AL_ETH_REF_FREQ_375_MHZ;
		adapter->mdio_freq = AL_ETH_DEFAULT_MDIO_FREQ_KHZ;
	} else if (adapter->board_type == ALPINE_NIC_V2_10) {
		adapter->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		adapter->sfp_detection_needed = false;
		adapter->phy_exist = false;
		adapter->an_en = false;
		adapter->lt_en = false;
		adapter->ref_clk_freq = AL_ETH_REF_FREQ_500_MHZ;
		adapter->mdio_freq = AL_ETH_DEFAULT_MDIO_FREQ_KHZ;
		adapter->link_config.active_duplex = DUPLEX_FULL;
		adapter->link_config.autoneg = AUTONEG_DISABLE;
		adapter->link_config.active_speed = 10000;
	} else if ((adapter->board_type == ALPINE_NIC_V2_25) ||
			(adapter->board_type == ALPINE_NIC_V2_25_DUAL)) {
		struct al_eth_board_params params;
		int rc;

		adapter->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
		adapter->sfp_detection_needed = false;
		adapter->phy_exist = false;
		adapter->an_en = false;
		adapter->lt_en = false;
		adapter->ref_clk_freq = AL_ETH_REF_FREQ_500_MHZ;
		adapter->mdio_freq = AL_ETH_DEFAULT_MDIO_FREQ_KHZ;
		adapter->link_config.active_duplex = DUPLEX_FULL;
		adapter->link_config.autoneg = AUTONEG_DISABLE;

		rc = al_eth_board_params_get(adapter->mac_base, &params);
		if (rc) {
			dev_err(&adapter->pdev->dev, "board info not available\n");
			return -1;
		}

		adapter->phy_if = params.phy_if;

		switch (params.media_type) {
		case AL_ETH_BOARD_MEDIA_TYPE_25G_10G_AUTO:
			al_eth_get_serdes_25g_speed(adapter, &adapter->link_config.active_speed);
			if (rc) {
				netdev_err(adapter->netdev, "Invalid speed read from 25G serdes\n");
				return rc;
			}
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_25G:
			adapter->link_config.active_speed = 25000;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_10GBASE_SR:
		default:
			adapter->link_config.active_speed = 10000;
			break;
		}
	} else if (adapter->board_type == ALPINE_INTEGRATED) {
		struct al_eth_board_params params;
		int rc;
		struct device_node *np;
		int8_t port_idx = -1;
		char path[128];

		adapter->auto_speed = false;

		rc = al_eth_board_params_get(adapter->mac_base, &params);
		if (rc) {
			dev_err(&adapter->pdev->dev, "board info not available\n");
			return -1;
		}

		adapter->phy_exist = params.phy_exist == AL_TRUE;
		adapter->phy_addr = params.phy_mdio_addr;
		adapter->an_en = params.autoneg_enable;
		adapter->lt_en = params.kr_lt_enable;
		adapter->serdes_grp = params.serdes_grp;
		adapter->serdes_lane = params.serdes_lane;
		adapter->sfp_detection_needed = params.sfp_plus_module_exist;
		adapter->i2c_adapter_id = params.i2c_adapter_id;
		adapter->ref_clk_freq = params.ref_clk_freq;
		adapter->dont_override_serdes = params.dont_override_serdes;
		adapter->link_config.active_duplex = !params.half_duplex;
		adapter->link_config.autoneg = (adapter->phy_exist) ?
			(params.an_mode == AL_ETH_BOARD_AUTONEG_IN_BAND) :
			(!params.an_disable);
		adapter->link_config.force_1000_base_x = params.force_1000_base_x;
		adapter->retimer.exist = params.retimer_exist;
		adapter->retimer.type = params.retimer_type;
		adapter->retimer.bus_id = params.retimer_bus_id;
		adapter->retimer.i2c_addr = params.retimer_i2c_addr;
		adapter->retimer.channel = params.retimer_channel;
		adapter->retimer.tx_channel = params.retimer_tx_channel;
		adapter->phy_if = params.phy_if;
		adapter->kr_fec_enable = params.kr_fec_enable;
		adapter->auto_fec_enable = params.auto_fec_enable;
		adapter->gpio_sfp_present = params.gpio_sfp_present;

		if (adapter->serdes_grp == 3) {
			if (adapter->serdes_lane == 2)
				port_idx = 0;
			else
				port_idx = 2;
		} else if (adapter->serdes_grp == 2) {
			if (adapter->serdes_lane == 2)
				port_idx = 2;
			else
				port_idx = 0;
		}

		sprintf(path, "/soc/board-cfg/ethernet/port%d/leds/sfp_1g", port_idx);
		np = of_find_node_by_path(path);
		adapter->gpio_spd_1g = 0;
		if (np) {
			adapter->gpio_spd_1g = of_get_named_gpio(np, "gpios", 0);
			if (adapter->gpio_spd_1g < 0)
				adapter->gpio_spd_1g = 0;
		} else {
			adapter->gpio_spd_1g = params.gpio_spd_1g;
		}

		sprintf(path, "/soc/board-cfg/ethernet/port%d/leds/sfp_10g", port_idx);
		np = of_find_node_by_path(path);
		adapter->gpio_spd_10g = 0;
		if (np) {
			adapter->gpio_spd_10g = of_get_named_gpio(np, "gpios", 0);
			if (adapter->gpio_spd_10g < 0)
				adapter->gpio_spd_10g = 0;
		} else {
			adapter->gpio_spd_10g = params.gpio_spd_10g;
		}

		sprintf(path, "/soc/board-cfg/ethernet/port%d/leds/sfp_25g", port_idx);
		np = of_find_node_by_path(path);
		adapter->gpio_spd_25g = 0;
		if (np) {
			adapter->gpio_spd_25g = of_get_named_gpio(np, "gpios", 0);
			if (adapter->gpio_spd_25g < 0)
				adapter->gpio_spd_25g = 0;
		} else {
			adapter->gpio_spd_25g = params.gpio_spd_25g;
		}

		switch (params.speed) {
		default:
			dev_warn(&adapter->pdev->dev,
				"%s: invalid speed (%d)\n", __func__,
				params.speed);
		case AL_ETH_BOARD_1G_SPEED_1000M:
			adapter->link_config.active_speed = 1000;
			break;
		case AL_ETH_BOARD_1G_SPEED_100M:
			adapter->link_config.active_speed = 100;
			break;
		case AL_ETH_BOARD_1G_SPEED_10M:
			adapter->link_config.active_speed = 10;
			break;
		}

		switch (params.mdio_freq) {
		default:
			dev_warn(&adapter->pdev->dev,
			"%s: invalid mdio freq (%d)\n", __func__,
			params.mdio_freq);
		case AL_ETH_BOARD_MDIO_FREQ_2_5_MHZ:
			adapter->mdio_freq = 2500;
			break;
		case AL_ETH_BOARD_MDIO_FREQ_1_MHZ:
			adapter->mdio_freq = 1000;
			break;
		}

		switch (params.media_type) {
		case AL_ETH_BOARD_MEDIA_TYPE_RGMII:
			if (params.sfp_plus_module_exist == AL_TRUE)
				/* Backward compatibility */
				adapter->mac_mode = AL_ETH_MAC_MODE_SGMII;
			else
				adapter->mac_mode = AL_ETH_MAC_MODE_RGMII;

			adapter->use_lm = false;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_SGMII:
			adapter->mac_mode = AL_ETH_MAC_MODE_SGMII;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_1G;
			if (adapter->link_config.force_1000_base_x)
				adapter->use_lm = false;
			else
				adapter->use_lm = true;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_SGMII_2_5G:
			adapter->mac_mode = AL_ETH_MAC_MODE_SGMII_2_5G;
			adapter->use_lm = false;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_10GBASE_SR:
			adapter->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_10G;
			adapter->use_lm = true;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT:
			adapter->sfp_detection_needed = AL_TRUE;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_10G;
			adapter->auto_speed = false;
			adapter->use_lm = true;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT_AUTO_SPEED:
			adapter->sfp_detection_needed = AL_TRUE;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_10G;
			adapter->auto_speed = true;
			adapter->mac_mode_set = false;
			adapter->use_lm = true;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_NBASE_T:
			adapter->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_10G;
			adapter->phy_fixup_needed = true;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_25G:
			adapter->sfp_detection_needed = AL_TRUE;
			adapter->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
			adapter->use_lm = true;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_25G;
			break;
		case AL_ETH_BOARD_MEDIA_TYPE_25G_10G_AUTO:
			adapter->sfp_detection_needed = AL_TRUE;
			adapter->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
			adapter->use_lm = true;
			adapter->max_speed = AL_ETH_LM_MAX_SPEED_25G;
			adapter->speed_detection = true;
			adapter->auto_speed = true;
			break;
		default:
			dev_err(&adapter->pdev->dev,
				"%s: unsupported media type %d\n",
				__func__, params.media_type);
			return -1;
		}
		dev_info(&adapter->pdev->dev,
			"Board info: phy exist %s. phy addr %d. mdio freq %u Khz. SFP connected %s. media %d\n",
			params.phy_exist == AL_TRUE ? "Yes" : "No",
			params.phy_mdio_addr,
			adapter->mdio_freq,
			params.sfp_plus_module_exist == AL_TRUE ? "Yes" : "No",
			params.media_type);
	} else {
			dev_err(&adapter->pdev->dev,
				"%s: unsupported board type %d\n",
				__func__, adapter->board_type);
			return -EPERM;
	}

	al_eth_mac_addr_read(adapter->ec_base, 0, adapter->mac_addr);

	return 0;
}

static inline void
al_eth_flow_ctrl_init(struct al_eth_adapter *adapter)
{
	uint8_t default_flow_ctrl;

	default_flow_ctrl = AL_ETH_FLOW_CTRL_TX_PAUSE;
	default_flow_ctrl |= AL_ETH_FLOW_CTRL_RX_PAUSE;

	adapter->link_config.flow_ctrl_supported = default_flow_ctrl;
}

static int
al_eth_flow_ctrl_config(struct al_eth_adapter *adapter)
{
	struct al_eth_flow_control_params *flow_ctrl_params;
	uint8_t active = adapter->link_config.flow_ctrl_active;
	int i;

	flow_ctrl_params = &adapter->flow_ctrl_params;
	memset(flow_ctrl_params, 0, sizeof(*flow_ctrl_params));

	flow_ctrl_params->type = AL_ETH_FLOW_CONTROL_TYPE_LINK_PAUSE;
	flow_ctrl_params->obay_enable =
		((active & AL_ETH_FLOW_CTRL_RX_PAUSE) != 0);
	flow_ctrl_params->gen_enable =
		((active & AL_ETH_FLOW_CTRL_TX_PAUSE) != 0);

	flow_ctrl_params->rx_fifo_th_high = AL_ETH_FLOW_CTRL_RX_FIFO_TH_HIGH;
	flow_ctrl_params->rx_fifo_th_low = AL_ETH_FLOW_CTRL_RX_FIFO_TH_LOW;
	flow_ctrl_params->quanta = AL_ETH_FLOW_CTRL_QUANTA;
	flow_ctrl_params->quanta_th = AL_ETH_FLOW_CTRL_QUANTA_TH;

	/* map priority to queue index, queue id = priority/2 */
	for (i = 0; i < AL_ETH_FWD_PRIO_TABLE_NUM; i++)
		flow_ctrl_params->prio_q_map[adapter->udma_num][i] =  1 << (i >> 1);

	al_eth_flow_control_config(&adapter->hal_adapter, flow_ctrl_params);

	return 0;
}

static void
al_eth_flow_ctrl_enable(struct al_eth_adapter *adapter)
{
	/* change the active configuration to the default / force by ethtool
	 * and call to configure */
	adapter->link_config.flow_ctrl_active =
		adapter->link_config.flow_ctrl_supported;

	al_eth_flow_ctrl_config(adapter);
}

static void
al_eth_flow_ctrl_disable(struct al_eth_adapter *adapter)
{
	adapter->link_config.flow_ctrl_active = 0;
	al_eth_flow_ctrl_config(adapter);
}

#ifdef CONFIG_PHYLIB
static uint8_t al_eth_flow_ctrl_mutual_cap_get(struct al_eth_adapter *adapter)
{
	struct phy_device *phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
	struct al_eth_link_config *link_config = &adapter->link_config;
	uint8_t peer_flow_ctrl = AL_ETH_FLOW_CTRL_AUTONEG;
	uint8_t new_flow_ctrl = AL_ETH_FLOW_CTRL_AUTONEG;

	if (phydev->pause)
		peer_flow_ctrl |= (AL_ETH_FLOW_CTRL_TX_PAUSE | AL_ETH_FLOW_CTRL_RX_PAUSE);
	if (phydev->asym_pause)
		peer_flow_ctrl ^= (AL_ETH_FLOW_CTRL_RX_PAUSE);

	/*
	 * in autoneg mode, supported flow ctrl is also
	 * the current advertising
	 */
	if ((peer_flow_ctrl & AL_ETH_FLOW_CTRL_TX_PAUSE) ==
	    (link_config->flow_ctrl_supported & AL_ETH_FLOW_CTRL_TX_PAUSE))
		new_flow_ctrl |= AL_ETH_FLOW_CTRL_TX_PAUSE;
	if ((peer_flow_ctrl & AL_ETH_FLOW_CTRL_RX_PAUSE) ==
		(link_config->flow_ctrl_supported & AL_ETH_FLOW_CTRL_RX_PAUSE))
		new_flow_ctrl |= AL_ETH_FLOW_CTRL_RX_PAUSE;

	return new_flow_ctrl;
}
#endif /* CONFIG_PHYLIB */


static int al_eth_hw_init_adapter(struct al_eth_adapter *adapter)
{
	struct al_eth_adapter_params *params = &adapter->eth_hal_params;
	struct al_udma_gen_tgtid_conf	conf;
	struct al_udma_gen_tgtid_msix_conf msix_conf;
	struct unit_regs __iomem	*unit_regs;

	int rc;
	int i;
	int tgtid;

	params->rev_id = adapter->rev_id;
	params->udma_id = adapter->udma_num;
	params->enable_rx_parser = 1; /* enable rx epe parser*/
	params->udma_regs_base = adapter->udma_base; /* UDMA register base address */
	params->ec_regs_base = adapter->ec_base; /* Ethernet controller registers base address */
	params->mac_regs_base = adapter->mac_base; /* Ethernet MAC registers base address */
	params->name = adapter->name;
	params->serdes_lane = adapter->serdes_lane;

	rc = al_eth_adapter_init(&adapter->hal_adapter, params);
	if (rc)
		netdev_err(adapter->netdev, "%s failed at hal init!\n", __func__);

	if (IS_NIC(adapter->board_type)) {
		/* in pcie NIC mode, force eth UDMA to access PCIE0 using the tgtid */
		if (adapter->rev_id >= AL_ETH_REV_ID_1) {

			if (adapter->rev_id < AL_ETH_REV_ID_3)
				tgtid = 0x100;
			else {
				if ((adapter->board_type == ALPINE_NIC_V2_25_DUAL) &&
						(PCI_FUNC(adapter->pdev->devfn) == 1))
					tgtid = 0x8001;
				else
					tgtid = 0x8000;
			}

#ifdef CONFIG_AL_ETH_SRIOV
			if (!(adapter->pdev->is_physfn)) {
				netdev_err(adapter->netdev, "setting tgtid to 0x1\n");
				if (adapter->rev_id < AL_ETH_REV_ID_3)
					tgtid = 0x101;
				else
					tgtid = 0x8010;
			}
#endif
			for (i = 0; i < DMA_MAX_Q; i++) {
				conf.tx_q_conf[i].queue_en = AL_TRUE;
				conf.tx_q_conf[i].desc_en = AL_FALSE;
				conf.tx_q_conf[i].tgtid = tgtid; /* for access from PCIE0 */
				conf.rx_q_conf[i].queue_en = AL_TRUE;
				conf.rx_q_conf[i].desc_en = AL_FALSE;
				conf.rx_q_conf[i].tgtid = tgtid; /* for access from PCIE0 */
			}
			al_udma_gen_tgtid_conf_set(adapter->udma_base, &conf);

			/* force MSIX to access PCIE 0 using the tgtid */
			msix_conf.access_en = AL_TRUE;
			msix_conf.sel = AL_TRUE;
			al_udma_gen_tgtid_msix_conf_set(adapter->udma_base, &msix_conf);

			unit_regs = (struct unit_regs __iomem *)params->udma_regs_base;
			al_iofic_msix_tgtid_attributes_config(
				&unit_regs->gen.interrupt_regs.main_iofic,
				AL_INT_GROUP_A, 0, tgtid, AL_TRUE);
			for (i = 0; i < DMA_MAX_Q; i++) {
				al_iofic_msix_tgtid_attributes_config(
					&unit_regs->gen.interrupt_regs.main_iofic,
					AL_INT_GROUP_B, i, tgtid, AL_TRUE);
				al_iofic_msix_tgtid_attributes_config(
					&unit_regs->gen.interrupt_regs.main_iofic,
					AL_INT_GROUP_C, i, tgtid, AL_TRUE);
			}
		}
	}
	return rc;
}

static int al_eth_hw_init(struct al_eth_adapter *adapter)
{
	int rc;
	enum al_eth_mdio_type mdio_type;

	netdev_dbg(adapter->netdev, "Init adapter\n");
	rc = al_eth_hw_init_adapter(adapter);
	if (rc)
		return rc;

	/**
	 * In NIC mode, dont do mac_config to prevent reset of FEC settings
	 * (the device should do mac_config once on init)
	 **/
	if (!IS_NIC(adapter->board_type)) {
		netdev_dbg(adapter->netdev, "mac config\n");

		rc = al_eth_mac_config(&adapter->hal_adapter, adapter->mac_mode);
		if (rc < 0) {
			netdev_err(adapter->netdev, "%s failed to configure mac!\n", __func__);
			return rc;
		}
	} else
		adapter->hal_adapter.mac_mode = adapter->mac_mode;

       if (((adapter->mac_mode == AL_ETH_MAC_MODE_SGMII) && (adapter->phy_exist == AL_FALSE)) ||
		(adapter->mac_mode == AL_ETH_MAC_MODE_RGMII && adapter->phy_exist == AL_FALSE)) {
		netdev_err(adapter->netdev, "link config (0x%x)\n", adapter->mac_mode);

		rc = al_eth_mac_link_config(&adapter->hal_adapter,
			adapter->link_config.force_1000_base_x,
			adapter->link_config.autoneg,
			adapter->link_config.active_speed,
			adapter->link_config.active_duplex);
		if (rc) {
			netdev_err(adapter->netdev,
				"%s failed to configure link parameters!\n", __func__);
			return rc;
		}
	}

	netdev_dbg(adapter->netdev, "mdio config\n");
	if (adapter->phy_if == AL_ETH_BOARD_PHY_IF_XMDIO) {
#ifndef CONFIG_ARCH_ALPINE
		/** Host driver doesnt support xmdio */
		return -EOPNOTSUPP;
#else
		mdio_type = AL_ETH_MDIO_TYPE_CLAUSE_45;
#endif
	} else
		mdio_type = AL_ETH_MDIO_TYPE_CLAUSE_22;

	rc = al_eth_mdio_config(&adapter->hal_adapter, mdio_type,
		AL_TRUE/*shared_mdio_if*/,
		adapter->ref_clk_freq, adapter->mdio_freq);
	if (rc) {
		netdev_err(adapter->netdev, "%s failed at mdio config!\n", __func__);
		return rc;
	}

	netdev_dbg(adapter->netdev, "flow ctrl config\n");
	al_eth_flow_ctrl_init(adapter);

	return rc;
}

static int al_eth_hw_stop(struct al_eth_adapter *adapter)
{
	al_eth_mac_stop(&adapter->hal_adapter);

	if (adapter->udma_num != 0) {
		/* if NIC mode, steer all packets to udma 0*/
		al_eth_mac_table_unicast_add(adapter, AL_ETH_MAC_TABLE_UNICAST_IDX_BASE,
			adapter->mac_addr, 1);
		al_eth_mac_table_broadcast_add(adapter, AL_ETH_MAC_TABLE_BROADCAST_IDX, 1);
		al_eth_mac_table_promiscuous_set(adapter, true, 1);
	}

	/* wait till pending rx packets written and UDMA becomes idle,
	 * the MAC has ~10KB fifo, 10us should be enought time for the
	 * UDMA to write to the memory
	 */
	udelay(100);

	al_eth_udma_queues_reset_all(adapter);
	al_eth_adapter_stop(&adapter->hal_adapter);

	adapter->flags |= AL_ETH_FLAG_RESET_REQUESTED;

	/* disable flow ctrl to avoid pause packets*/
	al_eth_flow_ctrl_disable(adapter);

	return 0;
}

static int al_eth_change_mtu(struct net_device *dev, int new_mtu)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;

	if ((new_mtu < AL_ETH_MIN_MTU) || (new_mtu > AL_ETH_MAX_MTU)) {
		netdev_err(dev, "Invalid MTU setting\n");
		return -EINVAL;
	}

	netdev_dbg(adapter->netdev, "changing MTU from %d to %d\n", dev->mtu, new_mtu);
	al_eth_rx_pkt_limit_config(&adapter->hal_adapter,
		AL_ETH_MIN_FRAME_LEN, max_frame);

	dev->mtu = new_mtu;

	return 0;
}

/**
 * al_eth_intr_msix_all - MSIX Interrupt Handler for all interrupts
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t al_eth_intr_msix_all(int irq, void *data)
{
	return IRQ_HANDLED;
}

/**
 * al_eth_intr_msix_mgmt - MSIX Interrupt Handler for Management interrupts
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t al_eth_intr_msix_mgmt(int irq, void *data)
{
	/* Currently this interrupt is triggered due to the following events:
	 * 1. UDMA M2S error/hint aggregate
	 * 2. UDMA S2M error/hint aggregate
	 * so we crash in order to prevent potential user data corruption.
	 * Specific handling of each error/hint will be added in the future.
	 */
	struct al_eth_adapter *adapter = data;
	struct unit_regs __iomem *regs_base = (struct unit_regs __iomem *)adapter->udma_base;
	uint32_t cause;

	cause = al_udma_iofic_read_cause(regs_base,
					   AL_UDMA_IOFIC_LEVEL_PRIMARY,
					   AL_INT_GROUP_D);
	netdev_err(adapter->netdev, "got interrupt from primary iofic, group D. cause 0x%x\n",
			   cause);

	cause = al_udma_iofic_read_cause(regs_base,
					   AL_UDMA_IOFIC_LEVEL_SECONDARY,
					   AL_INT_GROUP_A);
	netdev_err(adapter->netdev, "secondary iofic, group A (UDMA M2S errors) cause 0x%x\n",
		cause);

	cause = al_udma_iofic_read_cause(regs_base,
					   AL_UDMA_IOFIC_LEVEL_SECONDARY,
					   AL_INT_GROUP_B);
	netdev_err(adapter->netdev, "secondary iofic, group B (UDMA S2M errors) cause 0x%x\n",
			   cause);

	panic("Ethernet fatal error! (due to a UDMA error/hint)\n");

	return IRQ_HANDLED;
}

/**
 * al_eth_intr_msix_tx - MSIX Interrupt Handler for Tx
 * @irq: interrupt number
 * @data: pointer to a network interface private napi device structure
 **/
static irqreturn_t al_eth_intr_msix_tx(int irq, void *data)
{
	struct al_eth_napi *al_napi = data;

	pr_debug("%s\n", __func__);
	napi_schedule(&al_napi->napi);

	return IRQ_HANDLED;
}

/**
 * al_eth_intr_msix_rx - MSIX Interrupt Handler for Rx
 * @irq: interrupt number
 * @data: pointer to a network interface private napi device structure
 **/
static irqreturn_t al_eth_intr_msix_rx(int irq, void *data)
{
	struct al_eth_napi *al_napi = data;

	pr_debug("%s\n", __func__);
	napi_schedule(&al_napi->napi);
	return IRQ_HANDLED;
}

static int al_init_rx_cpu_rmap(struct al_eth_adapter *adapter)
{
#ifdef CONFIG_RFS_ACCEL
	unsigned int i;
	int rc;

	adapter->netdev->rx_cpu_rmap = alloc_irq_cpu_rmap(adapter->num_rx_queues);
	if (!adapter->netdev->rx_cpu_rmap)
		return -ENOMEM;
	for (i = 0; i < adapter->num_rx_queues; i++) {
		int	irq_idx = AL_ETH_RXQ_IRQ_IDX(adapter, i);

		rc = irq_cpu_rmap_add(adapter->netdev->rx_cpu_rmap,
				      adapter->msix_entries[irq_idx].vector);
		if (rc) {
			free_irq_cpu_rmap(adapter->netdev->rx_cpu_rmap);
			adapter->netdev->rx_cpu_rmap = NULL;
			return rc;
		}
	}
#endif
	return 0;
}

static void al_eth_enable_msix(struct al_eth_adapter *adapter)
{
	int i, msix_vecs, rc;

	msix_vecs  = 1 + adapter->num_rx_queues + adapter->num_tx_queues;

	dev_dbg(&adapter->pdev->dev, "Try to enable MSIX, vectors %d\n",
			msix_vecs);

	adapter->msix_entries = kcalloc(msix_vecs,
					sizeof(struct msix_entry), GFP_KERNEL);

	if (!adapter->msix_entries) {
		dev_err(&adapter->pdev->dev, "failed to allocate msix_entries, vectors %d\n",
			msix_vecs);

		return;
	}

	/* management vector (GROUP_A) @2*/
	adapter->msix_entries[AL_ETH_MGMT_IRQ_IDX].entry = 2;
	adapter->msix_entries[AL_ETH_MGMT_IRQ_IDX].vector = 0;

	/* rx queues start @3 */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		int	irq_idx = AL_ETH_RXQ_IRQ_IDX(adapter, i);

		adapter->msix_entries[irq_idx].entry = 3 + i;
		adapter->msix_entries[irq_idx].vector = 0;
	}
	/* tx queues start @7 */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		int	irq_idx = AL_ETH_TXQ_IRQ_IDX(adapter, i);

		adapter->msix_entries[irq_idx].entry = 3 + AL_ETH_MAX_HW_QUEUES + i;
		adapter->msix_entries[irq_idx].vector = 0;
	}

	rc = -ENOSPC;
	while (msix_vecs >= 1) {
		rc = pci_enable_msix(adapter->pdev, adapter->msix_entries, msix_vecs);
		if (rc <= 0)
			break;
		if (rc > 0)
			msix_vecs = 1; /* if we can't allocate all, then try only 1; */
	}

	if (rc != 0) {
		dev_dbg(&adapter->pdev->dev, "failed to enable MSIX, vectors %d\n",
			   msix_vecs);
		adapter->msix_vecs = 0;
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
		dev_dbg(&adapter->pdev->dev, "%s %d\n", __func__, __LINE__);

		return;
	}
	dev_dbg(&adapter->pdev->dev, "enable MSIX, vectors %d\n", msix_vecs);

	/* enable MSIX in the msix capability of the eth controller
	 * as the pci_enable_msix enables it in the pcie unit capability
	 */
	if (IS_NIC(adapter->board_type))
		writew(PCI_MSIX_FLAGS_ENABLE,
			adapter->internal_pcie_base + 0x1000 * adapter->udma_num + 0x92);

	if (msix_vecs >= 1) {
		if (al_init_rx_cpu_rmap(adapter))
			dev_warn(&adapter->pdev->dev, "failed to map irqs to cpus\n");
	}

	adapter->msix_vecs = msix_vecs;
	adapter->flags |= AL_ETH_FLAG_MSIX_ENABLED;
}

/* This function initializes the initial interrupt moderation table (which can be changed through a
 * sysfs exposed interace (as opposed to the default interrtupt moderation table which cannot).
 */
static void al_eth_configure_intr_moderation_table(struct al_eth_intr_moderation_entry
						*intr_moderation_table)
{
	unsigned int i;

	for (i = 0; i < AL_ETH_INTR_MAX_NUM_OF_LEVELS; i++) {
		intr_moderation_table[i].intr_moderation_interval =
			initial_moderation_table[i].intr_moderation_interval;
		intr_moderation_table[i].packets_per_interval =
			initial_moderation_table[i].packets_per_interval;
		intr_moderation_table[i].bytes_per_interval =
			initial_moderation_table[i].bytes_per_interval;
	}
}


/* This function initializes the static default_moderation_table interval values according to the
 * actual SB clock of the chip we are running on
 */
static void al_eth_init_intr_default_moderation_table_intervals(struct al_eth_adapter *adapter)
{
	uint sb_clk_freq = AL_ETH_REF_CLK_FREQ_TO_HZ(adapter->ref_clk_freq) / 1000;

	default_moderation_table[AL_ETH_INTR_MODERATION_LOWEST].intr_moderation_interval =
		AL_ETH_INTR_LOWEST_VALUE(sb_clk_freq);
	default_moderation_table[AL_ETH_INTR_MODERATION_LOW].intr_moderation_interval =
		AL_ETH_INTR_LOW_VALUE(sb_clk_freq);
	default_moderation_table[AL_ETH_INTR_MODERATION_MID].intr_moderation_interval =
		AL_ETH_INTR_MID_VALUE(sb_clk_freq);
	default_moderation_table[AL_ETH_INTR_MODERATION_HIGHEST].intr_moderation_interval =
		AL_ETH_INTR_HIGH_VALUE(sb_clk_freq);
	default_moderation_table[AL_ETH_INTR_MODERATION_HIGHEST].intr_moderation_interval =
		AL_ETH_INTR_HIGHEST_VALUE(sb_clk_freq);
}


void al_eth_initial_moderation_table_restore_default(void)
{
	int i;

	for (i = 0; i < AL_ETH_INTR_MAX_NUM_OF_LEVELS; i++)
		initial_moderation_table[i] = default_moderation_table[i];
}


void al_eth_init_intr_moderation_entry(enum al_eth_intr_moderation_level level,
		struct al_eth_intr_moderation_entry *entry)
{
	if (level >= AL_ETH_INTR_MAX_NUM_OF_LEVELS)
		return;

	initial_moderation_table[level].intr_moderation_interval =
			entry->intr_moderation_interval;
	initial_moderation_table[level].packets_per_interval =
			entry->packets_per_interval;
	initial_moderation_table[level].bytes_per_interval =
			entry->bytes_per_interval;
}

void al_eth_get_intr_moderation_entry(enum al_eth_intr_moderation_level level,
	struct al_eth_intr_moderation_entry *entry)
{
	if (level >= AL_ETH_INTR_MAX_NUM_OF_LEVELS)
		return;

	entry->intr_moderation_interval =
			initial_moderation_table[level].intr_moderation_interval;
	entry->packets_per_interval =
			initial_moderation_table[level].packets_per_interval;
	entry->bytes_per_interval =
			initial_moderation_table[level].bytes_per_interval;
}

void al_eth_update_intr_moderation(struct al_eth_adapter *adapter, unsigned int qid,
		enum al_eth_direction direction)
{
	struct al_eth_ring *ring;
	unsigned int packets, bytes;
	unsigned int smoothed_interval;
	enum al_eth_intr_moderation_level current_moderation_indx, new_moderation_indx;
	struct al_eth_intr_moderation_entry *intr_moderation_table = adapter->intr_moderation_table;
	struct al_eth_intr_moderation_entry current_moderation_entry;
	struct al_eth_intr_moderation_entry preceding_moderation_entry;
	struct al_eth_intr_moderation_entry new_moderation_entry;
	struct unit_regs *udma_base = (struct unit_regs *)(adapter->udma_base);

	if (direction == AL_ETH_TX)
		ring = &adapter->tx_ring[qid];
	else
		ring = &adapter->rx_ring[qid];
	pr_debug("qid=%d\n", qid);

	packets = ring->packets;
	bytes = ring->bytes;

	if (!packets || !bytes)
		return;

	current_moderation_indx = ring->moderation_table_indx;
	if (current_moderation_indx >=  AL_ETH_INTR_MAX_NUM_OF_LEVELS) {
		netdev_dbg(adapter->netdev, "Wrong moderation index\n");
		return;
	}
	current_moderation_entry = intr_moderation_table[current_moderation_indx];

	new_moderation_indx = current_moderation_indx;

	if (current_moderation_indx == AL_ETH_INTR_MODERATION_LOWEST) {
		if ((packets > current_moderation_entry.packets_per_interval) ||
				(bytes > current_moderation_entry.bytes_per_interval))
			new_moderation_indx = current_moderation_indx + 1;
	} else {
		preceding_moderation_entry = intr_moderation_table[current_moderation_indx - 1];

		if ((packets <= preceding_moderation_entry.packets_per_interval) ||
				(bytes <= preceding_moderation_entry.bytes_per_interval))
			new_moderation_indx = current_moderation_indx - 1;

		else if ((packets > current_moderation_entry.packets_per_interval) ||
				(bytes > current_moderation_entry.bytes_per_interval)) {
			if (current_moderation_indx != AL_ETH_INTR_MODERATION_HIGHEST)
				new_moderation_indx = current_moderation_indx + 1;
		}
	}
	new_moderation_entry = intr_moderation_table[new_moderation_indx];

	ring->packets = 0;
	ring->bytes = 0;

	smoothed_interval = ((new_moderation_entry.intr_moderation_interval * 4 +
			6 * ring->smoothed_interval) + 5) / 10;
	ring->smoothed_interval = smoothed_interval;
	ring->moderation_table_indx = new_moderation_indx;

	if (direction == AL_ETH_TX) {
		pr_debug("new tx interval:%d\n", smoothed_interval);
		al_iofic_msix_moder_interval_config(&udma_base->gen.interrupt_regs.main_iofic,
				AL_INT_GROUP_C, qid, smoothed_interval);
	} else { /* RX */
		pr_debug("interval:%d\n", smoothed_interval);
		al_iofic_msix_moder_interval_config(&udma_base->gen.interrupt_regs.main_iofic,
				AL_INT_GROUP_B, qid, smoothed_interval);
	}
	return;
}

static int al_eth_get_coalesce(struct net_device *net_dev,
				    struct ethtool_coalesce *coalesce)
{
	struct al_eth_adapter *adapter = netdev_priv(net_dev);

	coalesce->tx_coalesce_usecs = adapter->tx_usecs;
	coalesce->tx_coalesce_usecs_irq = adapter->tx_usecs;
	coalesce->rx_coalesce_usecs = adapter->rx_usecs;
	coalesce->rx_coalesce_usecs_irq = adapter->rx_usecs;
	coalesce->use_adaptive_rx_coalesce = adapter->adaptive_intr_rate;

	return 0;
}

static void al_eth_set_coalesce(
	struct al_eth_adapter *adapter,
	unsigned int tx_usecs,
	unsigned int rx_usecs)
{
	uint sb_clk_freq;
	uint interval;
	struct unit_regs *udma_base;
	sb_clk_freq = AL_ETH_REF_CLK_FREQ_TO_HZ(adapter->ref_clk_freq) / 1000;

	udma_base = (struct unit_regs *)(adapter->udma_base);

	if (adapter->tx_usecs != tx_usecs) {
		int qid;
		interval =
			(uint) (1000 * tx_usecs / AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(
				sb_clk_freq));
		BUG_ON(interval > 255);
		adapter->tx_usecs =
			interval * AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(
				sb_clk_freq) / 1000;

		for (qid = 0; qid < adapter->num_tx_queues; qid++)
			al_iofic_msix_moder_interval_config(
				&udma_base->gen.interrupt_regs.main_iofic,
				AL_INT_GROUP_C, qid, interval);
	}
	if (adapter->rx_usecs != rx_usecs) {
		int qid;
		interval =
			(uint) (1000 * rx_usecs / AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(
				sb_clk_freq));
		BUG_ON(interval > 255);
		adapter->rx_usecs =
			interval * AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(
				sb_clk_freq) / 1000;

		for (qid = 0; qid < adapter->num_rx_queues; qid++)
			al_iofic_msix_moder_interval_config(
				&udma_base->gen.interrupt_regs.main_iofic,
				AL_INT_GROUP_B, qid, interval);
	}
}

static int al_eth_configure_int_mode(struct al_eth_adapter *adapter)
{
	enum al_iofic_mode int_mode;
	uint32_t	m2s_errors_disable = 0x480;
	uint32_t	m2s_aborts_disable = 0x480;
	uint32_t	s2m_errors_disable = 0x1E0;
	uint32_t	s2m_aborts_disable = 0x1E0;
	uint32_t	intr_mod_res;

	/* single INTX mode */
	if (adapter->msix_vecs == 0)
		int_mode = AL_IOFIC_MODE_LEGACY;
	else if (adapter->msix_vecs > 1)
		int_mode = AL_IOFIC_MODE_MSIX_PER_Q;
	else {
		netdev_err(adapter->netdev, "udma doesn't support single MSI-X mode yet.\n");
		return -EIO;
	}

	if (adapter->board_type != ALPINE_INTEGRATED) {
		m2s_errors_disable |= 0x3f << 25;
		m2s_errors_disable |= 0x3f << 25;
		s2m_aborts_disable |= 0x3f << 25;
		s2m_aborts_disable |= 0x3f << 25;
	}

	if (al_udma_iofic_config((struct unit_regs __iomem *)adapter->udma_base,
		int_mode, m2s_errors_disable, m2s_aborts_disable,
		s2m_errors_disable, s2m_aborts_disable)) {
			netdev_err(adapter->netdev, "al_udma_unit_int_config failed!.\n");
			return -EIO;
	}
	adapter->int_mode = int_mode;
	netdev_info(adapter->netdev, "using %s interrupt mode",
		int_mode == AL_IOFIC_MODE_LEGACY ? "INTx" :
		int_mode == AL_IOFIC_MODE_MSIX_PER_Q ? "MSI-X per Queue" :
		"Unknown");
	/* set interrupt moderation resolution */
	intr_mod_res = AL_ETH_INTR_MODERATION_RESOLUTION;

	al_iofic_moder_res_config(
		&((struct unit_regs *)(adapter->udma_base))->gen.interrupt_regs.main_iofic,
			AL_INT_GROUP_B, intr_mod_res);
	al_iofic_moder_res_config(
		&((struct unit_regs *)(adapter->udma_base))->gen.interrupt_regs.main_iofic,
		AL_INT_GROUP_C, intr_mod_res);

	if (adapter->adaptive_intr_rate) {
		/* set initial moderation settings */
		al_eth_set_coalesce(adapter, AL_ETH_INTR_INITIAL_TX_INTERVAL_USECS,
				AL_ETH_INTR_INITIAL_RX_INTERVAL_USECS);
		al_eth_configure_intr_moderation_table(adapter->intr_moderation_table);
	}

	return 0;
}

static void al_eth_interrupts_unmask(struct al_eth_adapter *adapter)
{
	u32 group_a_mask = AL_INT_GROUP_A_GROUP_D_SUM; /* enable group D summery */
	u32 group_b_mask = (1 << adapter->num_rx_queues) - 1;/* bit per Rx q*/
	u32 group_c_mask = (1 << adapter->num_tx_queues) - 1;/* bit per Tx q*/
	u32 group_d_mask = 3 << 8;
	struct unit_regs __iomem *regs_base = (struct unit_regs __iomem *)adapter->udma_base;

	if (adapter->int_mode == AL_IOFIC_MODE_LEGACY)
		group_a_mask |= AL_INT_GROUP_A_GROUP_B_SUM |
				AL_INT_GROUP_A_GROUP_C_SUM |
				AL_INT_GROUP_A_GROUP_D_SUM;

	al_udma_iofic_unmask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_A, group_a_mask);
	al_udma_iofic_unmask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_B, group_b_mask);
	al_udma_iofic_unmask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_C, group_c_mask);
	al_udma_iofic_unmask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_D, group_d_mask);
}

static void al_eth_interrupts_mask(struct al_eth_adapter *adapter)
{
	struct unit_regs __iomem *regs_base = (struct unit_regs __iomem *)adapter->udma_base;

	/* mask all interrupts */
	al_udma_iofic_mask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_A, 0x7);
	al_udma_iofic_mask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_B, 0xF);
	al_udma_iofic_mask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_C, 0xF);
	al_udma_iofic_mask(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_D, 0xFFFFFFFF);
}

static void al_eth_disable_int_sync(struct al_eth_adapter *adapter)
{
	int i;

	if (!netif_running(adapter->netdev))
		return;

/** TODO - is just checking SRIOV enough? perhaps if we wish to use al_eth on EVP + SRIOV this */
/** will ruin it */
#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
	if (adapter->pdev->is_physfn) {
		/* disable forwarding interrupts from eth through pci end point*/
		if (IS_NIC(adapter->board_type)) {
			netdev_dbg(adapter->netdev, "disable int forwarding\n");
			writel(0, adapter->internal_pcie_base + 0x1800000 + 0x1210);
		}

#else
	/* Disable forwarding interrupts from eth through pci end point*/
	if (IS_NIC(adapter->board_type)) {
		netdev_dbg(adapter->netdev, "disable int forwarding\n");
		writel(0, adapter->internal_pcie_base + 0x1800000 + 0x1210);
	}
#endif

	/* Mask hw interrupts */
	al_eth_interrupts_mask(adapter);

	for (i = 0; i < adapter->irq_vecs; i++)
		synchronize_irq(adapter->irq_tbl[i].vector);
}

/**
 * al_eth_intr_intx_all - Legacy Interrupt Handler for all interrupts
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t al_eth_intr_intx_all(int irq, void *data)
{
	struct al_eth_adapter *adapter = data;
	struct unit_regs __iomem *regs_base = (struct unit_regs __iomem *)adapter->udma_base;
	uint32_t reg;

	reg = al_udma_iofic_read_cause(regs_base, AL_UDMA_IOFIC_LEVEL_PRIMARY, AL_INT_GROUP_A);
	if (likely(reg))
		pr_debug("%s group A cause %x\n", __func__, reg);

	if (unlikely(reg & AL_INT_GROUP_A_GROUP_D_SUM)) {
		struct al_iofic_grp_ctrl __iomem *sec_ints_base;
		uint32_t cause_d =  al_udma_iofic_read_cause(regs_base,
							     AL_UDMA_IOFIC_LEVEL_PRIMARY,
							     AL_INT_GROUP_D);

		sec_ints_base = &regs_base->gen.interrupt_regs.secondary_iofic_ctrl[0];
		if (cause_d) {
			pr_debug("got interrupt from group D. cause %x\n", cause_d);

			cause_d = al_iofic_read_cause(sec_ints_base, AL_INT_GROUP_A);
			pr_debug("secondary A cause %x\n", cause_d);

			cause_d = al_iofic_read_cause(sec_ints_base, AL_INT_GROUP_B);

			pr_debug("secondary B cause %x\n", cause_d);
		}
	}
	if (reg & AL_INT_GROUP_A_GROUP_B_SUM) {
		uint32_t cause_b = al_udma_iofic_read_cause(regs_base,
							    AL_UDMA_IOFIC_LEVEL_PRIMARY,
							    AL_INT_GROUP_B);
		int qid;
		for (qid = 0; qid < adapter->num_rx_queues; qid++) {
			if (cause_b & (1 << qid)) {
				/* mask */
				al_udma_iofic_mask(
					(struct unit_regs __iomem *)adapter->udma_base,
					AL_UDMA_IOFIC_LEVEL_PRIMARY,
					AL_INT_GROUP_B, 1 << qid);

				napi_schedule(
					&adapter->al_napi[AL_ETH_RXQ_NAPI_IDX(adapter, qid)].napi);
			}
		}
	}
	if (reg & AL_INT_GROUP_A_GROUP_C_SUM) {
		uint32_t cause_c = al_udma_iofic_read_cause(regs_base,
							    AL_UDMA_IOFIC_LEVEL_PRIMARY,
							    AL_INT_GROUP_C);
		int qid;
		for (qid = 0; qid < adapter->num_tx_queues; qid++) {
			if (cause_c & (1 << qid)) {
				/* mask */
				al_udma_iofic_mask(
					(struct unit_regs __iomem *)adapter->udma_base,
					AL_UDMA_IOFIC_LEVEL_PRIMARY,
					AL_INT_GROUP_C, 1 << qid);

				napi_schedule(
					&adapter->al_napi[AL_ETH_TXQ_NAPI_IDX(adapter, qid)].napi);
			}
		}
	}

	return IRQ_HANDLED;
}

static int al_eth_setup_int_mode(struct al_eth_adapter *adapter, int dis_msi)
{
	int i;
	unsigned int cpu;

	netdev_dbg(adapter->netdev, " setup int %d %d %d\n",
		dis_msi, adapter->msix_vecs, adapter->pdev->irq);

	if (!dis_msi)
		al_eth_enable_msix(adapter);

	adapter->irq_vecs = max(1, adapter->msix_vecs);

	/* single INTX mode */
	if (adapter->msix_vecs == 0) {
		snprintf(adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].name, AL_ETH_IRQNAME_SIZE,
			 "al-eth-intx-all@pci:%s", pci_name(adapter->pdev));
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].handler = al_eth_intr_intx_all;
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].vector = adapter->pdev->irq;
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].data = adapter;

		cpu = cpumask_first(cpu_online_mask);
		cpumask_set_cpu(cpu, &adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].affinity_hint_mask);

		return 0;
	}

	/* single MSI-X mode */
	if (adapter->msix_vecs == 1) {
		snprintf(adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].name, AL_ETH_IRQNAME_SIZE,
			 "al-eth-msix-all@pci:%s", pci_name(adapter->pdev));
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].handler = al_eth_intr_msix_all;
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].vector =
			adapter->msix_entries[AL_ETH_MGMT_IRQ_IDX].vector;
		adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].data = adapter;

		cpu = cpumask_first(cpu_online_mask);
		cpumask_set_cpu(cpu, &adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].affinity_hint_mask);

		return 0;
	}
	/* MSI-X per queue*/
	snprintf(adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].name, AL_ETH_IRQNAME_SIZE,
		"al-eth-msix-mgmt@pci:%s", pci_name(adapter->pdev));
	adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].handler = al_eth_intr_msix_mgmt;

	adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].data = adapter;
	adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].vector =
		adapter->msix_entries[AL_ETH_MGMT_IRQ_IDX].vector;
	cpu = cpumask_first(cpu_online_mask);
	cpumask_set_cpu(cpu, &adapter->irq_tbl[AL_ETH_MGMT_IRQ_IDX].affinity_hint_mask);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		int	irq_idx = AL_ETH_RXQ_IRQ_IDX(adapter, i);
		int	napi_idx = AL_ETH_RXQ_NAPI_IDX(adapter, i);

		snprintf(adapter->irq_tbl[irq_idx].name, AL_ETH_IRQNAME_SIZE,
			 "al-eth-rx-comp-%d@pci:%s", i,
			 pci_name(adapter->pdev));
		adapter->irq_tbl[irq_idx].handler = al_eth_intr_msix_rx;
		adapter->irq_tbl[irq_idx].data = &adapter->al_napi[napi_idx];
		adapter->irq_tbl[irq_idx].vector = adapter->msix_entries[irq_idx].vector;

		cpu = cpumask_next((i % num_online_cpus() - 1), cpu_online_mask);
		cpumask_set_cpu(cpu, &adapter->irq_tbl[irq_idx].affinity_hint_mask);
	}

	for (i = 0; i < adapter->num_tx_queues; i++) {
		int	irq_idx = AL_ETH_TXQ_IRQ_IDX(adapter, i);
		int	napi_idx = AL_ETH_TXQ_NAPI_IDX(adapter, i);

		snprintf(adapter->irq_tbl[irq_idx].name,
			 AL_ETH_IRQNAME_SIZE, "al-eth-tx-comp-%d@pci:%s", i,
			 pci_name(adapter->pdev));
		adapter->irq_tbl[irq_idx].handler = al_eth_intr_msix_tx;
		adapter->irq_tbl[irq_idx].data = &adapter->al_napi[napi_idx];
		adapter->irq_tbl[irq_idx].vector = adapter->msix_entries[irq_idx].vector;

		cpu = cpumask_next((i % num_online_cpus() - 1), cpu_online_mask);
		cpumask_set_cpu(cpu, &adapter->irq_tbl[irq_idx].affinity_hint_mask);
	}

	return 0;
}

static inline void
al_eth_vlan_rx_frag(struct al_eth_adapter *adapter, struct sk_buff *skb, u8 *va, unsigned int *len)
{
	struct skb_frag_struct *frag;
	struct vlan_ethhdr *veh = (struct vlan_ethhdr *)va;

	if (!al_eth_vlan_hwaccel_check_and_put(&adapter->netdev->features, skb, veh)) {
		/* remove vlan header and modify the length bookkeeping fields accordignly */
		memmove(va + VLAN_HLEN, va, 2 * ETH_ALEN);
		skb->len -= VLAN_HLEN;
		skb->data_len -= VLAN_HLEN;
		frag = skb_shinfo(skb)->frags;
		frag->page_offset += VLAN_HLEN;
		skb_frag_size_sub(frag, VLAN_HLEN);
		if (len)
			*len -= VLAN_HLEN;
	}
}

static inline void
al_eth_vlan_rx_linear(struct al_eth_adapter *adapter, struct sk_buff *skb, u8 **va,
		unsigned int *len)
{
	struct vlan_ethhdr *veh = (struct vlan_ethhdr *)*va;

	if (!al_eth_vlan_hwaccel_check_and_put(&adapter->netdev->features, skb, veh)) {
		/* remove vlan header (length bookkeeping will be done by calling function) */
		memmove(*va + VLAN_HLEN, *va, 2 * ETH_ALEN);
		*va += VLAN_HLEN;
		if (len)
			*len -= VLAN_HLEN;
	}
}

#ifdef CONFIG_AL_ETH_ALLOC_PAGE
static	struct sk_buff *al_eth_rx_skb(struct al_eth_adapter *adapter,
				      struct al_eth_ring *rx_ring,
				      struct al_eth_pkt *hal_pkt,
				      unsigned int descs,
				      u16 *next_to_clean)
{
	struct sk_buff *skb;
	struct al_eth_rx_buffer *rx_info =
		&rx_ring->rx_buffer_info[*next_to_clean];
	struct page *page = rx_info->page;
	unsigned int len, orig_len;
	unsigned int buf = 0;
	u8 *va;

	len = hal_pkt->bufs[0].len;
	dev_dbg(&adapter->pdev->dev, "rx_info %p page %p\n",
		rx_info, rx_info->page);

	page = rx_info->page;
	/* save virt address of first buffer */
	va = page_address(rx_info->page) + rx_info->page_offset;
	prefetch(va + AL_ETH_RX_OFFSET);

	if (len <= adapter->small_copy_len) {
		/** Smaller that this len will be copied to the skb header & dont use NAPI skb */
		skb = netdev_alloc_skb_ip_align(adapter->netdev, adapter->small_copy_len);
		if (unlikely(!skb)) {
			/*rx_ring->rx_stats.alloc_rx_buff_failed++;*/
			u64_stats_update_begin(&rx_ring->syncp);
			rx_ring->rx_stats.skb_alloc_fail++;
			u64_stats_update_end(&rx_ring->syncp);
			netdev_dbg(adapter->netdev, "Failed allocating skb\n");
			return NULL;
		}

		netdev_dbg(adapter->netdev, "rx skb allocated. len %d. data_len %d\n",
						skb->len, skb->data_len);

		netdev_dbg(adapter->netdev, "rx small packet. len %d\n", len);
		/* Need to use the same length when using the dma_* APIs */
		orig_len = len;
		/* sync this buffer for CPU use */
		dma_sync_single_for_cpu(rx_ring->dev, rx_info->dma, orig_len,
					DMA_FROM_DEVICE);
		if (hal_pkt->source_vlan_count > 0)
			/* This function alters len when the packet has a vlan tag */
			al_eth_vlan_rx_linear(adapter, skb, &va, &len);

		skb_copy_to_linear_data(skb, va, len);

		dma_sync_single_for_device(rx_ring->dev, rx_info->dma, orig_len,
					DMA_FROM_DEVICE);

		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, adapter->netdev);
		*next_to_clean = AL_ETH_RX_RING_IDX_ADD(rx_ring, *next_to_clean, descs);
		/** Increase counters (relevent for adaptive interrupt modertaion only) */
		rx_ring->bytes += orig_len;
		return skb;
	}
	/**
	 * Use napi_get_frags, it will call __napi_alloc_skb that will allocate the head from
	 * a special region used only for NAPI Rx allocation.
	 */
	skb = napi_get_frags(rx_ring->napi);
	if (unlikely(!skb)) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.skb_alloc_fail++;
		u64_stats_update_end(&rx_ring->syncp);
		netdev_dbg(adapter->netdev, "Failed allocating skb\n");
		return NULL;
	}

	do {
		dma_unmap_page(rx_ring->dev, dma_unmap_addr(rx_info, dma),
				PAGE_SIZE, DMA_FROM_DEVICE);

		skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, rx_info->page,
				rx_info->page_offset, len, PAGE_SIZE);

		netdev_dbg(adapter->netdev, "rx skb updated. len %d. data_len %d\n",
					skb->len, skb->data_len);

		rx_info->page = NULL;
		*next_to_clean = AL_ETH_RX_RING_IDX_NEXT(rx_ring, *next_to_clean);
		if (likely(--descs == 0))
			break;
		rx_info = &rx_ring->rx_buffer_info[*next_to_clean];
		len += hal_pkt->bufs[++buf].len;
	} while (1);

	al_eth_vlan_rx_frag(adapter, skb, va, &len);

	/** Increase counters (relevent for adaptive interrupt modertaion only) */
	rx_ring->bytes += len;

	return skb;
}
#elif defined(CONFIG_AL_ETH_ALLOC_FRAG)
static	struct sk_buff *al_eth_rx_skb(struct al_eth_adapter *adapter,
				      struct al_eth_ring *rx_ring,
				      struct al_eth_pkt *hal_pkt,
				      unsigned int descs,
				      u16 *next_to_clean)
{
	struct sk_buff *skb;
	struct al_eth_rx_buffer *rx_info =
		&rx_ring->rx_buffer_info[*next_to_clean];
	u8 *va = rx_info->data + AL_ETH_RX_OFFSET;
	unsigned int len, orig_len;
	unsigned int buf = 0;

	len = hal_pkt->bufs[0].len;
	netdev_dbg(adapter->netdev, "rx_info %p data %p\n", rx_info,
		   rx_info->data);

	prefetch(va);

	if (len <= adapter->small_copy_len) {
		netdev_dbg(adapter->netdev, "rx small packet. len %d\n", len);

		skb = netdev_alloc_skb_ip_align(adapter->netdev,
				adapter->small_copy_len);
		if (unlikely(!skb)) {
			u64_stats_update_begin(&rx_ring->syncp);
			rx_ring->rx_stats.skb_alloc_fail++;
			u64_stats_update_end(&rx_ring->syncp);
			return NULL;
		}

		/* Need to use the same length when using the dma_* APIs */
		orig_len = len;

		pci_dma_sync_single_for_cpu(adapter->pdev, rx_info->dma,
					    orig_len, DMA_FROM_DEVICE);
		if (hal_pkt->source_vlan_count > 0)
			/* This function alters len when the packet has a vlan tag */
			al_eth_vlan_rx_linear(adapter, skb, &va, &len);

		skb_copy_to_linear_data(skb, va, len);

		pci_dma_sync_single_for_device(adapter->pdev, rx_info->dma,
				orig_len, DMA_FROM_DEVICE);

		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, adapter->netdev);
		*next_to_clean = AL_ETH_RX_RING_IDX_NEXT(rx_ring, *next_to_clean);
		rx_ring->bytes += orig_len;
		return skb;
	}

	dma_unmap_single(rx_ring->dev, dma_unmap_addr(rx_info, dma),
			 rx_info->data_size, DMA_FROM_DEVICE);
#if 0
	skb = build_skb(rx_info->data, rx_ring->frag_size);
	if (unlikely(!skb))
		return NULL;
#else
	skb = napi_get_frags(rx_ring->napi);
	if (unlikely(!skb)) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.skb_alloc_fail++;
		u64_stats_update_end(&rx_ring->syncp);
		return NULL;
	}

	skb_fill_page_desc(skb, skb_shinfo(skb)->nr_frags,
				rx_info->page,
				rx_info->page_offset + AL_ETH_RX_OFFSET, len);

	skb->len += len;
	skb->data_len += len;
	skb->truesize += len;
	/** Increase adaptive threshold (relevent only when adapter->adaptive_intr_rate is set */
	rx_ring->bytes += len;
#endif
#if 0
	skb_reserve(skb, AL_ETH_RX_OFFSET);
	skb_put(skb, len);
#endif
	al_eth_vlan_rx_frag(adapter, skb, va, NULL);

	netdev_dbg(adapter->netdev, "rx skb updated. len %d. data_len %d\n",
				skb->len, skb->data_len);

	rx_info->data = NULL;
	*next_to_clean = AL_ETH_RX_RING_IDX_NEXT(rx_ring, *next_to_clean);

	while (--descs) {
		rx_info = &rx_ring->rx_buffer_info[*next_to_clean];
		len = hal_pkt->bufs[++buf].len;
		rx_ring->bytes += len;

		dma_unmap_single(rx_ring->dev, dma_unmap_addr(rx_info, dma),
				 rx_info->data_size, DMA_FROM_DEVICE);

		skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags,
				rx_info->page,
				rx_info->page_offset + AL_ETH_RX_OFFSET,
				len, rx_info->data_size);

		netdev_dbg(adapter->netdev, "rx skb updated. len %d. data_len %d\n",
			skb->len, skb->data_len);

		rx_info->data = NULL;

		*next_to_clean = AL_ETH_RX_RING_IDX_NEXT(rx_ring, *next_to_clean);
	}

	return skb;
}
#elif defined(CONFIG_AL_ETH_ALLOC_SKB)
static	struct sk_buff *al_eth_rx_skb(struct al_eth_adapter *adapter,
				      struct al_eth_ring *rx_ring,
				      struct al_eth_pkt *hal_pkt,
				      unsigned int descs,
				      u16 *next_to_clean)
{
	struct sk_buff *skb;
	struct al_eth_rx_buffer *rx_info =
		&rx_ring->rx_buffer_info[*next_to_clean];
	unsigned int len;

	prefetch(rx_info->data + AL_ETH_RX_OFFSET);
	skb = rx_info->skb;
	prefetch(skb);
	prefetch(&skb->end);
	prefetch(&skb->dev);

	len = hal_pkt->bufs[0].len;

	dma_unmap_single(rx_ring->dev, dma_unmap_addr(rx_info, dma),
			rx_info->data_size, DMA_FROM_DEVICE);

	skb_reserve(skb, AL_ETH_RX_OFFSET);
	skb_put(skb, len);

	skb->protocol = eth_type_trans(skb, adapter->netdev);
	rx_info->skb = NULL;
	*next_to_clean = AL_ETH_RX_RING_IDX_NEXT(rx_ring, *next_to_clean);
	/* prefetch next packet */
	prefetch((rx_info + 1)->data + AL_ETH_RX_OFFSET);
	prefetch((rx_info + 1)->skb);

	return skb;
}
#endif

/* configure the RX forwarding (UDMA/QUEUE.. selection)
 * currently we don't use the full control table, we use only the default configuration
 */

static void
al_eth_config_rx_fwd(struct al_eth_adapter *adapter)
{
	struct al_eth_fwd_ctrl_table_entry entry;
	int i;

	/* let priority be equal to pbits */
	for (i = 0; i < AL_ETH_FWD_PBITS_TABLE_NUM; i++)
		al_eth_fwd_pbits_table_set(&adapter->hal_adapter, i, i);

	/* map priority to queue index, queue id = priority/2 */
	for (i = 0; i < AL_ETH_FWD_PRIO_TABLE_NUM; i++)
		al_eth_fwd_priority_table_set(&adapter->hal_adapter, i, i >> 1);

	entry.prio_sel = AL_ETH_CTRL_TABLE_PRIO_SEL_VAL_0;
	entry.queue_sel_1 = AL_ETH_CTRL_TABLE_QUEUE_SEL_1_THASH_TABLE;
	entry.queue_sel_2 = AL_ETH_CTRL_TABLE_QUEUE_SEL_2_NO_PRIO;
	entry.udma_sel = AL_ETH_CTRL_TABLE_UDMA_SEL_MAC_TABLE;
	entry.filter = AL_FALSE;

	al_eth_ctrl_table_def_set(&adapter->hal_adapter, AL_FALSE, &entry);

	/*
	 * By default set the mac table to forward all unicast packets to our
	 * MAC address and all broadcast. all the rest will be dropped.
	 */
	al_eth_mac_table_unicast_add(adapter, AL_ETH_MAC_TABLE_UNICAST_IDX_BASE,
				     adapter->mac_addr, 1 << adapter->udma_num);
	al_eth_mac_table_broadcast_add(adapter, AL_ETH_MAC_TABLE_BROADCAST_IDX, 1 << adapter->udma_num);
	al_eth_mac_table_promiscuous_set(adapter, false, 1 << adapter->udma_num);

	/* set toeplitz hash keys */
	get_random_bytes(adapter->toeplitz_hash_key,
			 sizeof(adapter->toeplitz_hash_key));

	for (i = 0; i < AL_ETH_RX_HASH_KEY_NUM; i++)
		al_eth_hash_key_set(&adapter->hal_adapter, i,
				    htonl(adapter->toeplitz_hash_key[i]));

	for (i = 0; i < AL_ETH_RX_RSS_TABLE_SIZE; i++)
		al_eth_thash_table_set(&adapter->hal_adapter, i, adapter->udma_num, adapter->rss_ind_tbl[i]);

	al_eth_fsm_table_init(adapter);
}

#ifdef CONFIG_PHYLIB
/* MDIO */
static int al_mdio_read(struct mii_bus *bp, int mii_id, int reg)
{
	struct al_eth_adapter *adapter = bp->priv;
	u16 value = 0;
	int rc;
	int timeout = MDIO_TIMEOUT_MSEC;

	while (timeout > 0) {
		if (reg & MII_ADDR_C45) {
			/** Clause 45 */
			al_dbg("%s [c45]: dev %x reg %x val %x\n",
				__func__,
				((reg & AL_ETH_MDIO_C45_DEV_MASK) >> AL_ETH_MDIO_C45_DEV_SHIFT),
				(reg & AL_ETH_MDIO_C45_REG_MASK), value);
			rc = al_eth_mdio_read(&adapter->hal_adapter, adapter->phy_addr,
				((reg & AL_ETH_MDIO_C45_DEV_MASK) >> AL_ETH_MDIO_C45_DEV_SHIFT),
				(reg & AL_ETH_MDIO_C45_REG_MASK), &value);
		} else {
			/** Clause 22 */
			rc = al_eth_mdio_read(&adapter->hal_adapter, adapter->phy_addr,
					      MDIO_DEVAD_NONE, reg, &value);
		}

		if (rc == 0)
			return value;

		netdev_dbg(adapter->netdev,
			   "mdio read failed. try again in 10 msec\n");

		timeout -= 10;
		msleep(10);
	}

	if (rc)
		netdev_err(adapter->netdev, "MDIO read failed on timeout\n");

	return value;
}

static int
al_mdio_write(struct mii_bus *bp, int mii_id, int reg, u16 val)
{
	struct al_eth_adapter *adapter = bp->priv;
	int rc;
	int timeout = MDIO_TIMEOUT_MSEC;

	while (timeout > 0) {
		if (reg & MII_ADDR_C45) {
			al_dbg("%s [c45]: device %x reg %x val %x\n",
				__func__,
				((reg & AL_ETH_MDIO_C45_DEV_MASK) >> AL_ETH_MDIO_C45_DEV_SHIFT),
				(reg & AL_ETH_MDIO_C45_REG_MASK), val);
			rc = al_eth_mdio_write(&adapter->hal_adapter, adapter->phy_addr,
				((reg & AL_ETH_MDIO_C45_DEV_MASK) >> AL_ETH_MDIO_C45_DEV_SHIFT),
				(reg & AL_ETH_MDIO_C45_REG_MASK), val);
		} else {
			rc = al_eth_mdio_write(&adapter->hal_adapter, adapter->phy_addr,
				       MDIO_DEVAD_NONE, reg, val);
		}

		if (rc == 0)
			return 0;

		netdev_err(adapter->netdev,
			   "mdio write failed. try again in 10 msec\n");

		timeout -= 10;
		msleep(10);
	}


	if (rc)
		netdev_err(adapter->netdev, "MDIO write failed on timeout\n");

	return rc;
}

/**
 * al_eth_mdiobus_teardown - mdiobus unregister
 *
 *
 **/
static void al_eth_mdiobus_teardown(struct al_eth_adapter *adapter)
{
	if (!adapter->mdio_bus)
		return;

	mdiobus_unregister(adapter->mdio_bus);
	kfree(adapter->mdio_bus->irq);
	mdiobus_free(adapter->mdio_bus);
	phy_device_free(adapter->phydev);
}

/**
 * al_eth_mdiobus_setup - initialize mdiobus and register to kernel
 *
 *
 **/
static int al_eth_mdiobus_setup(struct al_eth_adapter *adapter)
{
	struct phy_device *phydev;
	int i;
	int ret;

	adapter->mdio_bus = mdiobus_alloc();
	if (adapter->mdio_bus == NULL)
		return -ENOMEM;

	adapter->mdio_bus->name     = "al mdio bus";
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	adapter->mdio_bus->id = (adapter->pdev->bus->number << 8) | adapter->pdev->devfn;
#else
	snprintf(adapter->mdio_bus->id, MII_BUS_ID_SIZE, "%x",
		 (adapter->pdev->bus->number << 8) | adapter->pdev->devfn);
#endif
	adapter->mdio_bus->priv     = adapter;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
	adapter->mdio_bus->dev = &adapter->pdev->dev;
#else
	adapter->mdio_bus->parent   = &adapter->pdev->dev;
#endif
	adapter->mdio_bus->read     = &al_mdio_read;
	adapter->mdio_bus->write    = &al_mdio_write;
	adapter->mdio_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);

	if (!adapter->mdio_bus->irq) {
		mdiobus_free(adapter->mdio_bus);
		return -ENOMEM;
	}

	/* Initialise the interrupts to polling */
	for (i = 0; i < PHY_MAX_ADDR; i++)
		adapter->mdio_bus->irq[i] = PHY_POLL;

	if (adapter->phy_if == AL_ETH_BOARD_PHY_IF_XMDIO) {
#if defined(CONFIG_ARCH_ALPINE)
		/** XMDIO is not supported on host driver due to kernel 3.2 compatability issue */

		/**
		 * Mask out all the devices from auto probing
		 * (we get device address from board params)
		 */
		adapter->mdio_bus->phy_mask = 0xffffffff;
		i = mdiobus_register(adapter->mdio_bus);
		if (i) {
			netdev_warn(adapter->netdev, "mdiobus_reg failed (0x%x)\n", i);
			mdiobus_free(adapter->mdio_bus);
			return i;
		}

		phydev = get_phy_device(adapter->mdio_bus, adapter->phy_addr, true);
		if (!phydev) {
			netdev_err(adapter->netdev, "%s: phy device get failed\n", __func__);
			goto error;
		}

		ret = phy_device_register(phydev);
		if (ret) {
			netdev_err(adapter->netdev, "%s: phy device register failed\n", __func__);
			goto error_xmdio_phy;
		}
#else
		ret = -EOPNOTSUPP;
		return ret;
#endif
	} else {
		adapter->mdio_bus->phy_mask = ~(1 << adapter->phy_addr);
		i = mdiobus_register(adapter->mdio_bus);
		if (i) {
			netdev_warn(adapter->netdev, "mdiobus_reg failed (0x%x)\n", i);
			mdiobus_free(adapter->mdio_bus);
			return i;
		}
		phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
	}

	if (!phydev || !phydev->drv) {
		netdev_err(adapter->netdev, "%s: phy device get failed\n", __func__);
		goto error;
	}

	return 0;

#if defined(CONFIG_ARCH_ALPINE)
error_xmdio_phy:
	phy_device_free(phydev);
#endif
error:
	netdev_warn(adapter->netdev, "No PHY devices\n");
	mdiobus_unregister(adapter->mdio_bus);
	kfree(adapter->mdio_bus->irq);
	mdiobus_free(adapter->mdio_bus);
	return -ENODEV;
}

static void al_eth_adjust_link(struct net_device *dev)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);
	struct al_eth_link_config *link_config = &adapter->link_config;
	struct phy_device *phydev = adapter->phydev;
	enum al_eth_mac_mode mac_mode_needed = AL_ETH_MAC_MODE_RGMII;
	int new_state = 0;
	int force_1000_base_x = false;

	if (phydev->link) {
		if (phydev->duplex != link_config->active_duplex) {
			new_state = 1;
			link_config->active_duplex = phydev->duplex;
		}

		if (phydev->speed != link_config->active_speed) {
			new_state = 1;
			switch (phydev->speed) {
			case SPEED_1000:
			case SPEED_100:
			case SPEED_10:
				if (adapter->mac_mode == AL_ETH_MAC_MODE_RGMII)
					mac_mode_needed = AL_ETH_MAC_MODE_RGMII;
				else {
					if (adapter->mac_mode == AL_ETH_MAC_MODE_SGMII)
						mac_mode_needed = AL_ETH_MAC_MODE_SGMII;
					else
						mac_mode_needed = AL_ETH_MAC_MODE_10GbE_Serial;
				}
				break;
			case SPEED_10000:
			case SPEED_2500:
				mac_mode_needed = AL_ETH_MAC_MODE_10GbE_Serial;
				break;
			default:
				if (netif_msg_link(adapter))
					netdev_warn(adapter->netdev,
					"Ack!  Speed (%d) is not 10M/100M/1G/2.5G/10G!",
						phydev->speed);
				break;
			}
			link_config->active_speed = phydev->speed;
		}

		if (!link_config->old_link) {
			new_state = 1;
			link_config->old_link = 1;
		}

		if (new_state) {
			int rc;

			if (adapter->mac_mode != mac_mode_needed) {
				al_eth_down(adapter);
				adapter->mac_mode = mac_mode_needed;
#ifdef CONFIG_ARCH_ALPINE
				if (link_config->active_speed > 1000)
					al_eth_serdes_mode_set(adapter);
				else {
					al_eth_serdes_mode_set(adapter);
					force_1000_base_x = true;
				}
#else
				if (link_config->active_speed <= 1000)
					force_1000_base_x = true;
#endif /** CONFIG_ARCH_ALPINE */
				al_eth_up(adapter);
			}

			if (adapter->mac_mode != AL_ETH_MAC_MODE_10GbE_Serial) {
				/* change the MAC link configuration */
				rc = al_eth_mac_link_config(&adapter->hal_adapter,
						force_1000_base_x,
						link_config->autoneg,
						link_config->active_speed,
						link_config->active_duplex ? AL_TRUE : AL_FALSE);
				if (rc) {
					netdev_warn(adapter->netdev,
					"Failed to config the mac with the new link settings!");
				}
			}
		}

		if (link_config->flow_ctrl_supported & AL_ETH_FLOW_CTRL_AUTONEG) {
			uint8_t new_flow_ctrl =
				al_eth_flow_ctrl_mutual_cap_get(adapter);

			if (new_flow_ctrl != link_config->flow_ctrl_active) {
				link_config->flow_ctrl_active = new_flow_ctrl;
				al_eth_flow_ctrl_config(adapter);
			}

		}
	} else if (adapter->link_config.old_link) {
			new_state = 1;
			link_config->old_link = 0;
			link_config->active_duplex = DUPLEX_UNKNOWN;
			link_config->active_speed = SPEED_UNKNOWN;
	}

	if (new_state && netif_msg_link(adapter))
		phy_print_status(phydev);
}

static int al_eth_phy_init(struct al_eth_adapter *adapter)
{
	struct phy_device *phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];

	adapter->link_config.old_link = 0;
	adapter->link_config.active_duplex = DUPLEX_UNKNOWN;
	adapter->link_config.active_speed = SPEED_UNKNOWN;

	/* Attach the MAC to the PHY. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 9, 0)
	phydev = phy_connect(adapter->netdev, dev_name(&phydev->dev), al_eth_adjust_link,
		PHY_INTERFACE_MODE_RGMII);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	phydev = phy_connect(adapter->netdev, dev_name(&phydev->dev), al_eth_adjust_link,
			     0, PHY_INTERFACE_MODE_RGMII);
#else
	phydev = phy_connect(adapter->netdev, dev_name(&phydev->dev), al_eth_adjust_link, 0);
#endif
	if (IS_ERR(phydev)) {
		netdev_err(adapter->netdev, "Could not attach to PHY\n");
		return PTR_ERR(phydev);
	}

	netdev_info(adapter->netdev, "phy[%d]: device %s, driver %s\n",
		phydev->addr, dev_name(&phydev->dev),
		phydev->drv ? phydev->drv->name : "unknown");

	/* Mask with MAC supported features. */
	phydev->supported &= (PHY_GBIT_FEATURES |
		SUPPORTED_Pause |
		SUPPORTED_Asym_Pause);

	phydev->advertising = phydev->supported;

	netdev_info(adapter->netdev, "phy[%d]:supported %x adv %x\n",
		phydev->addr, phydev->supported, phydev->advertising);

	adapter->phydev = phydev;
	/* Bring the PHY up */
	phy_start(adapter->phydev);

	return 0;
}
#endif /* CONFIG_PHYLIB */

static int al_eth_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
#if defined(CONFIG_PHYLIB)
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct mii_ioctl_data *mdio = if_mii(ifr);
	struct phy_device *phydev;

	if (adapter->phy_exist == false)
		return -EOPNOTSUPP;

	netdev_info(adapter->netdev, "ioctl: phy id 0x%x, reg 0x%x, val_in 0x%x\n",
			mdio->phy_id, mdio->reg_num, mdio->val_in);

	if (adapter->mdio_bus) {
		phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
		if (phydev)
			return phy_mii_ioctl(phydev, ifr, cmd);
	}

	return -EOPNOTSUPP;
#else
	return -EOPNOTSUPP;
#endif
}

static void al_eth_tx_timeout(struct net_device *dev)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);

	u64_stats_update_begin(&adapter->syncp);
	adapter->dev_stats.tx_timeout++;
	u64_stats_update_end(&adapter->syncp);

	if (netif_msg_tx_err(adapter))
		netdev_err(dev, "transmit timed out!!!!\n");
	schedule_work(&adapter->reset_task);
}

int al_eth_read_pci_config(void *handle, int where, uint32_t *val)
{
	/* handle is a pointer to the pci_dev */
	pci_read_config_dword((struct pci_dev *)handle, where, val);
	return 0;
}

int al_eth_write_pci_config(void *handle, int where, uint32_t val)
{
	/* handle is a pointer to the pci_dev */
	pci_write_config_dword((struct pci_dev *)handle, where, val);
	return 0;
}

static void al_eth_reset_task(struct work_struct *work)
{
	struct al_eth_adapter *adapter;
	adapter = container_of(work, struct al_eth_adapter, reset_task);
	netdev_err(adapter->netdev, "%s restarting interface\n", __func__);
	/*restart interface*/
	rtnl_lock();
	al_eth_down(adapter);
	al_eth_up(adapter);
	rtnl_unlock();
}

static int
al_eth_function_reset(struct al_eth_adapter *adapter)
{
	struct al_eth_board_params params;
	int rc;
	uint32_t adapter_pci_cmd;

	/** FLR is different for VF */
	if (adapter->udma_num) {
		writel(AL_PCI_EXP_DEVCTL_BCR_FLR, adapter->internal_pcie_base + 0x1000 * adapter->udma_num + AL_PCI_EXP_CAP_BASE + AL_PCI_EXP_DEVCTL);
		/* make sure flr is done */
		readl(adapter->internal_pcie_base + 0x1000 * adapter->udma_num + 0x48);
		mdelay(1);
		/* enable master/slave in the adapter conf */
		adapter_pci_cmd = readw(adapter->internal_pcie_base + 0x1000 * (adapter->udma_num) + PCI_COMMAND);
		adapter_pci_cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
		writew(adapter_pci_cmd, adapter->internal_pcie_base + 0x1000 * (adapter->udma_num) + PCI_COMMAND);

		return 0;
	}

	/* save board params so we restore it after reset */
	al_eth_board_params_get(adapter->mac_base, &params);
	al_eth_mac_addr_read(adapter->ec_base, 0, adapter->mac_addr);

	rc = al_eth_flr_rmn(&al_eth_read_pci_config,
		&al_eth_write_pci_config,
		adapter->pdev, adapter->mac_base);

	/* restore params */
	al_eth_board_params_set(adapter->mac_base, &params);
	al_eth_mac_addr_store(adapter->ec_base, 0, adapter->mac_addr);
	return rc;
}

static int
al_eth_hal_adapter_init(struct al_eth_adapter *adapter)
{
	struct al_hal_eth_adapter *hal_adapter = &adapter->hal_adapter;
	struct al_udma_params udma_params;
	int rc;
	int i;

	hal_adapter->name = adapter->name;
	hal_adapter->rev_id = adapter->rev_id;
	hal_adapter->udma_id = adapter->udma_num;
	hal_adapter->udma_regs_base = adapter->udma_base;
	hal_adapter->ec_regs_base = (struct al_ec_regs __iomem*)adapter->ec_base;
	hal_adapter->mac_regs_base = (struct al_eth_mac_regs __iomem*)adapter->mac_base;
	hal_adapter->unit_regs = (struct unit_regs __iomem *)adapter->udma_base;
	hal_adapter->enable_rx_parser = 1;
	hal_adapter->serdes_lane = adapter->serdes_lane;
	hal_adapter->ec_ints_base = ((void __iomem *)hal_adapter->ec_regs_base) + 0x1c00;
	hal_adapter->mac_ints_base = ((void __iomem *)hal_adapter->mac_regs_base) + 0x800;

	/* initialize Tx udma */
	udma_params.udma_regs_base = hal_adapter->unit_regs;
	udma_params.type = UDMA_TX;
	udma_params.num_of_queues = AL_ETH_UDMA_TX_QUEUES;
	udma_params.name = "eth tx";
	rc = al_udma_init(&hal_adapter->tx_udma, &udma_params);

	if (rc != 0) {
		al_err("failed to initialize %s, error %d\n",
			 udma_params.name, rc);
		return rc;
	}
	/* initialize Rx udma */
	udma_params.udma_regs_base = hal_adapter->unit_regs;
	udma_params.type = UDMA_RX;
	udma_params.num_of_queues = AL_ETH_UDMA_RX_QUEUES;
	udma_params.name = "eth rx";
	rc = al_udma_init(&hal_adapter->rx_udma, &udma_params);

	if (rc != 0) {
		al_err("failed to initialize %s, error %d\n",
			 udma_params.name, rc);
		return rc;
	}

	/* init queue structure fields required for udma_q_reset */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct al_eth_ring *ring = &adapter->tx_ring[i];

		al_udma_q_handle_get(&hal_adapter->tx_udma, i, &ring->dma_q);
		ring->dma_q->q_regs = (union udma_q_regs __iomem *)
				&hal_adapter->tx_udma.udma_regs->m2s.m2s_q[i];
		ring->dma_q->qid = i;
		ring->dma_q->udma = &hal_adapter->tx_udma;
	}

	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct al_eth_ring *ring = &adapter->rx_ring[i];

		al_udma_q_handle_get(&hal_adapter->rx_udma, i, &ring->dma_q);
		ring->dma_q->q_regs = (union udma_q_regs __iomem *)
				&hal_adapter->rx_udma.udma_regs->s2m.s2m_q[i];
		ring->dma_q->qid = i;
		ring->dma_q->udma = &hal_adapter->rx_udma;
	}

	return 0;
}

static void
al_eth_init_rings(struct al_eth_adapter *adapter)
{
	int i;
	uint sb_clk_freq =  AL_ETH_REF_CLK_FREQ_TO_HZ(adapter->ref_clk_freq) / 1000;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct al_eth_ring *ring = &adapter->tx_ring[i];

		ring->dev = &adapter->pdev->dev;
		ring->netdev = adapter->netdev;
		al_udma_q_handle_get(&adapter->hal_adapter.tx_udma, i, &ring->dma_q);
		ring->sw_count = adapter->tx_ring_count;
		ring->hw_count = adapter->tx_descs_count;
		ring->unmask_reg_offset = al_udma_iofic_unmask_offset_get(
						(struct unit_regs *)adapter->udma_base,
						AL_UDMA_IOFIC_LEVEL_PRIMARY,
						AL_INT_GROUP_C);
		ring->unmask_val = ~(1 << i);

		ring->moderation_table_indx = AL_ETH_INTR_MODERATION_LOWEST;
		ring->smoothed_interval = AL_ETH_INTR_LOWEST_VALUE(sb_clk_freq);
		ring->packets = 0;
		ring->bytes = 0;
		u64_stats_init(&ring->syncp);
	}

	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct al_eth_ring *ring = &adapter->rx_ring[i];

		ring->dev = &adapter->pdev->dev;
		ring->netdev = adapter->netdev;
		ring->napi = &adapter->al_napi[AL_ETH_RXQ_NAPI_IDX(adapter,
				i)].napi;
		al_udma_q_handle_get(&adapter->hal_adapter.rx_udma, i, &ring->dma_q);
		ring->sw_count = adapter->rx_ring_count;
		ring->hw_count = adapter->rx_descs_count;
		ring->unmask_reg_offset = al_udma_iofic_unmask_offset_get(
						(struct unit_regs *)adapter->udma_base,
						AL_UDMA_IOFIC_LEVEL_PRIMARY,
						AL_INT_GROUP_B);
		ring->unmask_val = ~(1 << i);

		ring->moderation_table_indx = AL_ETH_INTR_MODERATION_LOWEST;
		ring->smoothed_interval = AL_ETH_INTR_LOWEST_VALUE(sb_clk_freq);
		ring->packets = 0;
		ring->bytes = 0;
		u64_stats_init(&ring->syncp);
	}
}

/**
 * al_eth_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: network interface device structure
 * @qid: queue index
 *
 * Return 0 on success, negative on failure
 **/
static int
al_eth_setup_tx_resources(struct al_eth_adapter *adapter, int qid)
{
	struct al_eth_ring *tx_ring = &adapter->tx_ring[qid];
	struct device *dev = tx_ring->dev;
	struct al_udma_q_params *q_params = &tx_ring->q_params;
	int size;

	size = sizeof(struct al_eth_tx_buffer) * tx_ring->sw_count;

	tx_ring->tx_buffer_info = kzalloc(size, GFP_KERNEL);
	if (!tx_ring->tx_buffer_info)
		return -ENOMEM;

	/* TODO: consider ALIGN to page size */
	tx_ring->descs_size = tx_ring->hw_count * sizeof(union al_udma_desc);
	q_params->size = tx_ring->hw_count;

	q_params->desc_base = dma_alloc_coherent(dev,
					tx_ring->descs_size,
					&q_params->desc_phy_base,
					GFP_KERNEL);

	if (!q_params->desc_base)
		return -ENOMEM;

	q_params->cdesc_base = NULL; /* completion queue not used for tx */
	q_params->cdesc_size = 8;

	/* Reset ethtool TX statistics */
	memset(&tx_ring->tx_stats, 0x0, sizeof(tx_ring->tx_stats));

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	return 0;
}

/**
 * al_eth_free_tx_resources - Free Tx Resources per Queue
 * @adapter: network interface device structure
 * @qid: queue index
 *
 * Free all transmit software resources
 **/
static void
al_eth_free_tx_resources(struct al_eth_adapter *adapter, int qid)
{
	struct al_eth_ring *tx_ring = &adapter->tx_ring[qid];
	struct al_udma_q_params *q_params = &tx_ring->q_params;

	netdev_dbg(adapter->netdev, "%s qid %d\n", __func__, qid);

	kfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!q_params->desc_base)
		return;

	dma_free_coherent(tx_ring->dev, tx_ring->descs_size,
			  q_params->desc_base,
			  q_params->desc_phy_base);

	q_params->desc_base = NULL;
}


/**
 * al_eth_setup_all_tx_resources - allocate all queues Tx resources
 * @adapter: private structure
 *
 * Return 0 on success, negative on failure
 **/
static int
al_eth_setup_all_tx_resources(struct al_eth_adapter *adapter)
{
	int i, rc = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		rc = al_eth_setup_tx_resources(adapter, i);
		if (!rc)
			continue;

		netdev_err(adapter->netdev, "Allocation for Tx Queue %u failed\n", i);
		goto err_setup_tx;
	}

	return 0;
err_setup_tx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		al_eth_free_tx_resources(adapter, i);
	return rc;
}

/**
 * al_eth_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void
al_eth_free_all_tx_resources(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		if (adapter->tx_ring[i].q_params.desc_base)
			al_eth_free_tx_resources(adapter, i);
}


/**
 * al_eth_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: network interface device structure
 * @qid: queue index
 *
 * Returns 0 on success, negative on failure
 **/
static int
al_eth_setup_rx_resources(struct al_eth_adapter *adapter, unsigned int qid)
{
	struct al_eth_ring *rx_ring = &adapter->rx_ring[qid];
	struct device *dev = rx_ring->dev;
	struct al_udma_q_params *q_params = &rx_ring->q_params;
	int size;

	size = sizeof(struct al_eth_rx_buffer) * rx_ring->sw_count;

	/* alloc extra element so in rx path we can always prefetch rx_info + 1*/
	size += 1;

	rx_ring->rx_buffer_info = kzalloc(size, GFP_KERNEL);
	if (!rx_ring->rx_buffer_info)
		return -ENOMEM;

	/* TODO: consider Round up to nearest 4K */
	rx_ring->descs_size = rx_ring->hw_count * sizeof(union al_udma_desc);
	q_params->size = rx_ring->hw_count;

	q_params->desc_base = dma_alloc_coherent(dev, rx_ring->descs_size,
					&q_params->desc_phy_base,
					GFP_KERNEL);
	if (!q_params->desc_base)
		return -ENOMEM;

	q_params->cdesc_size = 16;
	rx_ring->cdescs_size = rx_ring->hw_count * q_params->cdesc_size;
	q_params->cdesc_base = dma_alloc_coherent(dev,
					   rx_ring->cdescs_size,
					   &q_params->cdesc_phy_base,
					   GFP_KERNEL);
	if (!q_params->cdesc_base)
		return -ENOMEM;

	/* Zero out the descriptor ring */
	memset(q_params->cdesc_base, 0, rx_ring->cdescs_size);

	/* Reset ethtool RX statistics */
	memset(&rx_ring->rx_stats, 0x0, sizeof(rx_ring->rx_stats));

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;
}

/**
 * al_eth_free_rx_resources - Free Rx Resources
 * @adapter: network interface device structure
 * @qid: queue index
 *
 * Free all receive software resources
 **/
static void
al_eth_free_rx_resources(struct al_eth_adapter *adapter, unsigned int qid)
{
	struct al_eth_ring *rx_ring = &adapter->rx_ring[qid];
	struct al_udma_q_params *q_params = &rx_ring->q_params;

	kfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!q_params->desc_base)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->descs_size,
			  q_params->desc_base,
			  q_params->desc_phy_base);

	q_params->desc_base = NULL;

	/* if not set, then don't free */
	if (!q_params->cdesc_base)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->cdescs_size,
			  q_params->cdesc_base,
			  q_params->cdesc_phy_base);

	q_params->cdesc_phy_base = 0;
}

/**
 * al_eth_setup_all_rx_resources - allocate all queues Rx resources
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static int al_eth_setup_all_rx_resources(struct al_eth_adapter *adapter)
{
	int i, rc = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		rc = al_eth_setup_rx_resources(adapter, i);
		if (!rc)
			continue;

		netdev_err(adapter->netdev, "Allocation for Rx Queue %u failed\n", i);
		goto err_setup_rx;
	}
	return 0;

err_setup_rx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		al_eth_free_rx_resources(adapter, i);
	return rc;
}

/**
 * al_eth_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
static void al_eth_free_all_rx_resources(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		if (adapter->rx_ring[i].q_params.desc_base)
			al_eth_free_rx_resources(adapter, i);
}

#ifdef CONFIG_AL_ETH_ALLOC_PAGE
static inline int
al_eth_alloc_rx_page(struct al_eth_adapter *adapter,
		     struct al_eth_rx_buffer *rx_info, gfp_t gfp)
{
	struct al_buf *al_buf;
	struct page *page;
	dma_addr_t dma;
	struct al_eth_ring *rx_ring = container_of(&rx_info, struct al_eth_ring, rx_buffer_info);

	/* if previous allocated page is not used */
	if (rx_info->page != NULL)
		return 0;

	page = alloc_page(gfp);
	if (unlikely(!page))
		return -ENOMEM;

	dma = dma_map_page(&adapter->pdev->dev, page, 0, PAGE_SIZE,
				DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(&adapter->pdev->dev, dma))) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.dma_mapping_err++;
		u64_stats_update_end(&rx_ring->syncp);

		__free_page(page);
		return -EIO;
	}
	dev_dbg(&adapter->pdev->dev, "alloc page %p, rx_info %p\n",
		page, rx_info);

	rx_info->page = page;
	rx_info->page_offset = 0;
	al_buf = &rx_info->al_buf;
	dma_unmap_addr_set(al_buf, addr, dma);
	dma_unmap_addr_set(rx_info, dma, dma);
	dma_unmap_len_set(al_buf, len, PAGE_SIZE);
	return 0;
}

static void
al_eth_free_rx_page(struct al_eth_adapter *adapter,
		    struct al_eth_rx_buffer *rx_info)
{
	struct page *page = rx_info->page;
	struct al_buf *al_buf = &rx_info->al_buf;

	if (!page)
		return;

	dma_unmap_page(&adapter->pdev->dev, dma_unmap_addr(al_buf, addr),
		       PAGE_SIZE, DMA_FROM_DEVICE);

	__free_page(page);
	rx_info->page = NULL;
}

#elif defined(CONFIG_AL_ETH_ALLOC_FRAG)

static inline int
al_eth_alloc_rx_frag(struct al_eth_adapter *adapter,
		     struct al_eth_ring *rx_ring,
		     struct al_eth_rx_buffer *rx_info)
{
	struct al_buf *al_buf;
	dma_addr_t dma;
	u8 *data;

	/* if previous allocated frag is not used */
	if (rx_info->data != NULL)
		return 0;

	rx_info->data_size = min_t(unsigned int,
				  (rx_ring->netdev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN),
				   adapter->max_rx_buff_alloc_size);

	rx_info->data_size = max_t(unsigned int,
				   rx_info->data_size,
				   AL_ETH_DEFAULT_MIN_RX_BUFF_ALLOC_SIZE);

	rx_info->frag_size = SKB_DATA_ALIGN(rx_info->data_size + AL_ETH_RX_OFFSET) +
			     SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	data = netdev_alloc_frag(rx_info->frag_size);

	if (!data)
		return -ENOMEM;

	dma = dma_map_single(rx_ring->dev, data + AL_ETH_RX_OFFSET,
			rx_info->data_size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(rx_ring->dev, dma))) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.dma_mapping_err++;
		u64_stats_update_end(&rx_ring->syncp);

		put_page(virt_to_head_page(data));
		return -EIO;
	}
	netdev_dbg(rx_ring->netdev, "alloc frag %p, rx_info %p len %x skb size %x\n",
		data, rx_info, rx_info->data_size, rx_info->frag_size);

	rx_info->data = data;

	BUG_ON(!virt_addr_valid(rx_info->data));
	rx_info->page = virt_to_head_page(rx_info->data);
	rx_info->page_offset = (uintptr_t)rx_info->data -
			       (uintptr_t)page_address(rx_info->page);
	al_buf = &rx_info->al_buf;
	dma_unmap_addr_set(al_buf, addr, dma);
	dma_unmap_addr_set(rx_info, dma, dma);
	dma_unmap_len_set(al_buf, len, rx_info->data_size);
	return 0;
}

static void
al_eth_free_rx_frag(struct al_eth_adapter *adapter,
		    struct al_eth_rx_buffer *rx_info)
{
	u8 *data = rx_info->data;
	struct al_buf *al_buf = &rx_info->al_buf;

	if (!data)
		return;

	dma_unmap_single(&adapter->pdev->dev, dma_unmap_addr(al_buf, addr),
		       rx_info->data_size, DMA_FROM_DEVICE);

	put_page(virt_to_head_page(data));
	rx_info->data = NULL;
}

#elif defined(CONFIG_AL_ETH_ALLOC_SKB)

static inline int
al_eth_alloc_rx_skb(struct al_eth_adapter *adapter,
		     struct al_eth_ring *rx_ring,
		     struct al_eth_rx_buffer *rx_info)
{
	struct sk_buff *skb;
	struct al_buf *al_buf;
	dma_addr_t dma;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	struct sk_buff_head *rx_rc = this_cpu_ptr(&rx_recycle_cache);
#else
	struct sk_buff_head *rx_rc = &__get_cpu_var(rx_recycle_cache);
#endif

	if (rx_info->skb)
		return 0;

	rx_info->data_size = rx_ring->netdev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;

	rx_info->data_size = max_t(unsigned int,
				   rx_info->data_size,
				   AL_ETH_DEFAULT_MIN_RX_BUFF_ALLOC_SIZE);

	skb = __skb_dequeue(rx_rc);
	if (skb == NULL)
		skb = __netdev_alloc_skb_ip_align(rx_ring->netdev, rx_info->data_size, GFP_DMA);

	if (!skb) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.skb_alloc_fail++;
		u64_stats_update_end(&rx_ring->syncp);
		return -ENOMEM;
	}

	dma = dma_map_single(rx_ring->dev, skb->data + AL_ETH_RX_OFFSET,
			rx_info->data_size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(rx_ring->dev, dma))) {
		u64_stats_update_begin(&rx_ring->syncp);
		rx_ring->rx_stats.dma_mapping_err++;
		u64_stats_update_end(&rx_ring->syncp);

		return -EIO;
	}

	rx_info->data = skb->data;
	rx_info->skb = skb;

	BUG_ON(!virt_addr_valid(rx_info->data));
	al_buf = &rx_info->al_buf;
	dma_unmap_addr_set(al_buf, addr, dma);
	dma_unmap_addr_set(rx_info, dma, dma);
	dma_unmap_len_set(al_buf, len, rx_info->data_size);
	return 0;
}

static void
al_eth_free_rx_skb(struct al_eth_adapter *adapter,
		    struct al_eth_rx_buffer *rx_info)
{
	struct al_buf *al_buf = &rx_info->al_buf;

	if (!rx_info->skb)
		return;

	dma_unmap_single(&adapter->pdev->dev, dma_unmap_addr(al_buf, addr),
		       rx_info->data_size, DMA_FROM_DEVICE);
	dev_kfree_skb_any(rx_info->skb);
	rx_info->skb = NULL;
}

/* the following 3 functions taken from old kernels */
static bool skb_is_recycleable(const struct sk_buff *skb, int skb_size)
{
	if (irqs_disabled())
		return false;

	if (skb_shinfo(skb)->tx_flags & SKBTX_DEV_ZEROCOPY)
		return false;

	if (skb_is_nonlinear(skb) || skb->fclone != SKB_FCLONE_UNAVAILABLE)
		return false;

	skb_size = SKB_DATA_ALIGN(skb_size + NET_SKB_PAD);
	if (skb_end_offset(skb) < skb_size)
		return false;

	if (skb_shared(skb) || skb_cloned(skb))
		return false;

	return true;
}

/**
 *     skb_recycle - clean up an skb for reuse
 *     @skb: buffer
 *
 *     Recycles the skb to be reused as a receive buffer. This
 *     function does any necessary reference count dropping, and
 *     cleans up the skbuff as if it just came from __alloc_skb().
 */
void skb_recycle(struct sk_buff *skb)
{
	struct skb_shared_info *shinfo;

	skb_release_head_state(skb);
	shinfo = skb_shinfo(skb);
	memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
	atomic_set(&shinfo->dataref, 1);

	memset(skb, 0, offsetof(struct sk_buff, tail));
	skb->data = skb->head + NET_SKB_PAD;
	skb_reset_tail_pointer(skb);
}

/**
 *     skb_recycle_check - check if skb can be reused for receive
 *     @skb: buffer
 *     @skb_size: minimum receive buffer size
 *
 *     Checks that the skb passed in is not shared or cloned, and
 *     that it is linear and its head portion at least as large as
 *     skb_size so that it can be recycled as a receive buffer.
 *     If these conditions are met, this function does any necessary
 *     reference count dropping and cleans up the skbuff as if it
 *     just came from __alloc_skb().
*/
bool skb_recycle_check(struct sk_buff *skb, int skb_size)
{
	if (!skb_is_recycleable(skb, skb_size))
		return false;

	skb_recycle(skb);

	return true;
}
#endif	/* CONFIG_AL_ETH_ALLOC_SKB */

static int
al_eth_refill_rx_bufs(struct al_eth_adapter *adapter, unsigned int qid,
		      unsigned int num)
{
	struct al_eth_ring *rx_ring = &adapter->rx_ring[qid];
	u16 next_to_use;
	unsigned int i;

	next_to_use = rx_ring->next_to_use;

	for (i = 0; i < num; i++) {
		int rc;
		struct al_eth_rx_buffer *rx_info = &rx_ring->rx_buffer_info[next_to_use];

#ifdef CONFIG_AL_ETH_ALLOC_PAGE
		if (unlikely(al_eth_alloc_rx_page(adapter, rx_info,
						  __GFP_COLD | GFP_ATOMIC | __GFP_COMP) < 0)) {
#elif defined(CONFIG_AL_ETH_ALLOC_FRAG)
		if (unlikely(al_eth_alloc_rx_frag(adapter, rx_ring, rx_info) < 0)) {
#elif defined(CONFIG_AL_ETH_ALLOC_SKB)
		if (unlikely(al_eth_alloc_rx_skb(adapter, rx_ring, rx_info) < 0)) {
#endif
			netdev_warn(adapter->netdev, "failed to alloc buffer for rx queue %d\n",
				    qid);
			break;
		}
		rc = al_eth_rx_buffer_add(rx_ring->dma_q,
					 &rx_info->al_buf, AL_ETH_RX_FLAGS_INT,
					 NULL);
		if (unlikely(rc)) {
			netdev_warn(adapter->netdev, "failed to add buffer for rx queue %d\n",
				    qid);
			break;

		}
		next_to_use = AL_ETH_RX_RING_IDX_NEXT(rx_ring, next_to_use);
	}

	if (unlikely(i < num)) {
		netdev_warn(adapter->netdev, "refilled rx queue %d with %d pages only - available %d\n",
			    qid, i, al_udma_available_get(rx_ring->dma_q));
	}

	if (likely(i))
		al_eth_rx_buffer_action(rx_ring->dma_q, i);

	rx_ring->next_to_use = next_to_use;

	return i;
}

static void
al_eth_free_rx_bufs(struct al_eth_adapter *adapter, unsigned int qid)
{
	struct al_eth_ring *rx_ring = &adapter->rx_ring[qid];
	unsigned int i;

	for (i = 0; i < AL_ETH_DEFAULT_RX_DESCS; i++) {
		struct al_eth_rx_buffer *rx_info = &rx_ring->rx_buffer_info[i];

#ifdef CONFIG_AL_ETH_ALLOC_PAGE
		if (rx_info->page)
			al_eth_free_rx_page(adapter, rx_info);
#elif defined(CONFIG_AL_ETH_ALLOC_FRAG)
		if (rx_info->data)
			al_eth_free_rx_frag(adapter, rx_info);
#elif defined(CONFIG_AL_ETH_ALLOC_SKB)
		if (rx_info->skb)
			al_eth_free_rx_skb(adapter, rx_info);
#endif
	}
}

/**
 * al_eth_refill_all_rx_bufs - allocate all queues Rx buffers
 * @adapter: board private structure
 *
 **/
static void
al_eth_refill_all_rx_bufs(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		al_eth_refill_rx_bufs(adapter, i, AL_ETH_DEFAULT_RX_DESCS - 1);
}

static void
al_eth_free_all_rx_bufs(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		al_eth_free_rx_bufs(adapter, i);
}

/**
 * al_eth_free_tx_bufs - Free Tx Buffers per Queue
 * @adapter: network interface device structure
 * @qid: queue index
 **/
static void
al_eth_free_tx_bufs(struct al_eth_adapter *adapter, unsigned int qid)
{
	struct al_eth_ring *tx_ring = &adapter->tx_ring[qid];
	unsigned int i;
	bool udma_debug_printed = 0;

	for (i = 0; i < AL_ETH_DEFAULT_TX_SW_DESCS; i++) {
		struct al_eth_tx_buffer *tx_info = &tx_ring->tx_buffer_info[i];
		struct al_buf *al_buf;
		int nr_frags;
		int j;

		if (tx_info->skb == NULL)
			continue;

		if (!udma_debug_printed) {
			al_udma_regs_print(tx_ring->dma_q->udma, AL_UDMA_DEBUG_QUEUE(qid));
			al_udma_q_struct_print(tx_ring->dma_q->udma, qid);
			udma_debug_printed = 1;
		}
		netdev_warn(adapter->netdev, "free uncompleted tx skb qid %d idx 0x%x\n",
				qid, i);

		al_buf = tx_info->hal_pkt.bufs;
		dma_unmap_single(&adapter->pdev->dev,
				 dma_unmap_addr(al_buf, addr),
				 dma_unmap_len(al_buf, len), DMA_TO_DEVICE);

		/* unmap remaining mapped pages */
		nr_frags = tx_info->hal_pkt.num_of_bufs - 1;
		for (j = 0; j < nr_frags; j++) {
			al_buf++;
			dma_unmap_page(&adapter->pdev->dev,
				       dma_unmap_addr(al_buf, addr),
				       dma_unmap_len(al_buf, len),
				       DMA_TO_DEVICE);
		}

		dev_kfree_skb_any(tx_info->skb);
	}
	netdev_tx_reset_queue(netdev_get_tx_queue(adapter->netdev, qid));
}

static void
al_eth_free_all_tx_bufs(struct al_eth_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		al_eth_free_tx_bufs(adapter, i);
}

static int al_eth_request_irq(struct al_eth_adapter *adapter)
{
	unsigned long flags;
	struct al_eth_irq *irq;
	int rc = 0, i;

	if (adapter->flags & AL_ETH_FLAG_MSIX_ENABLED)
		flags = 0;
	else
		flags = IRQF_SHARED;

	for (i = 0; i < adapter->irq_vecs; i++) {
		irq = &adapter->irq_tbl[i];
		rc = request_irq(irq->vector, irq->handler, flags, irq->name, irq->data);
		if (rc) {
			netdev_err(adapter->netdev,
				"failed to request irq. index %d rc %d\n", i, rc);
			break;
		}
		irq->requested = 1;

		netdev_dbg(adapter->netdev,
			"set affinity hint of irq. index %d to 0x%lx (irq vector: %d)\n",
			i, irq->affinity_hint_mask.bits[0], irq->vector);

#ifdef HAVE_IRQ_AFFINITY_HINT
		irq_set_affinity_hint(irq->vector, &irq->affinity_hint_mask);
#endif
	}
	return rc;
}

static void __al_eth_free_irq(struct al_eth_adapter *adapter)
{
	struct al_eth_irq *irq;
	int i;

	for (i = 0; i < adapter->irq_vecs; i++) {
		irq = &adapter->irq_tbl[i];
		if (irq->requested) {
#ifdef HAVE_IRQ_AFFINITY_HINT
			irq_set_affinity_hint(irq->vector, NULL);
#endif
			free_irq(irq->vector, irq->data);
		}
		irq->requested = 0;
	}
}

static void
al_eth_free_irq(struct al_eth_adapter *adapter)
{
#ifdef CONFIG_RFS_ACCEL
	if (adapter->msix_vecs >= 1) {
		free_irq_cpu_rmap(adapter->netdev->rx_cpu_rmap);
		adapter->netdev->rx_cpu_rmap = NULL;
	}
#endif

	__al_eth_free_irq(adapter);
	if (adapter->flags & AL_ETH_FLAG_MSIX_ENABLED)
		pci_disable_msix(adapter->pdev);

	adapter->flags &= ~AL_ETH_FLAG_MSIX_ENABLED;

	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;
}

/**
 * al_eth_tx_poll - NAPI Tx polling callback
 * @napi: structure for representing this polling device
 * @budget: how many packets driver is allowed to clean
 *
 * This function is used for legacy and MSI, NAPI mode
 **/
static int
al_eth_tx_poll(struct napi_struct *napi, int budget)
{
	struct al_eth_napi *al_napi = container_of(napi, struct al_eth_napi, napi);
	struct al_eth_adapter *adapter = al_napi->adapter;
	unsigned int qid = al_napi->qid;
	struct al_eth_ring *tx_ring = &adapter->tx_ring[qid];
	struct netdev_queue *txq;
	unsigned int tx_bytes = 0;
	unsigned int total_done;
	u16 next_to_clean;
	int tx_pkt = 0;
#ifdef CONFIG_AL_ETH_ALLOC_SKB
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	struct sk_buff_head *rx_rc = this_cpu_ptr(&rx_recycle_cache);
#else
	struct sk_buff_head *rx_rc = &__get_cpu_var(rx_recycle_cache);
#endif
#endif
	total_done = al_eth_comp_tx_get(tx_ring->dma_q);
	dev_dbg(&adapter->pdev->dev, "tx_poll: q %d total completed descs %x\n",
		qid, total_done);
	next_to_clean = tx_ring->next_to_clean;
	txq = netdev_get_tx_queue(adapter->netdev, qid);

	while (total_done) {
		struct al_eth_tx_buffer *tx_info;
		struct sk_buff *skb;
		struct al_buf *al_buf;
		int i, nr_frags;

		tx_info = &tx_ring->tx_buffer_info[next_to_clean];
		/* stop if not all descriptors of the packet are completed */
		if (tx_info->tx_descs > total_done)
			break;

		skb = tx_info->skb;

		/* prefetch skb_end_pointer() to speedup skb_shinfo(skb) */
		prefetch(&skb->end);

		tx_info->skb = NULL;
		al_buf = tx_info->hal_pkt.bufs;
		dma_unmap_single(tx_ring->dev, dma_unmap_addr(al_buf, addr),
				 dma_unmap_len(al_buf, len), DMA_TO_DEVICE);

		/* unmap remaining mapped pages */
		nr_frags = tx_info->hal_pkt.num_of_bufs - 1;
		for (i = 0; i < nr_frags; i++) {
			al_buf++;
			dma_unmap_page(tx_ring->dev, dma_unmap_addr(al_buf, addr),
				       dma_unmap_len(al_buf, len), DMA_TO_DEVICE);
		}

		tx_bytes += skb->len;
		dev_dbg(&adapter->pdev->dev, "tx_poll: q %d skb %p completed\n",
				qid, skb);
#ifdef CONFIG_AL_ETH_ALLOC_SKB
               if ((skb_queue_len(rx_rc) <
                       AL_ETH_DEFAULT_RX_DESCS) &&
                       skb_recycle_check(skb, tx_ring->netdev->mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN))
                       __skb_queue_head(rx_rc, skb);
               else
#endif
		       dev_kfree_skb(skb);
		tx_pkt++;

		/** Increase counters, relevent in adaptive mode only */
		tx_ring->packets += tx_pkt;
		tx_ring->bytes += tx_bytes;

		total_done -= tx_info->tx_descs;
		next_to_clean = AL_ETH_TX_RING_IDX_NEXT(tx_ring, next_to_clean);
	}

	netdev_tx_completed_queue(txq, tx_pkt, tx_bytes);

	tx_ring->next_to_clean = next_to_clean;

	dev_dbg(&adapter->pdev->dev, "tx_poll: q %d done next to clean %x\n",
		qid, next_to_clean);

	/* need to make the rings circular update visible to
	 * al_eth_start_xmit() before checking for netif_queue_stopped().
	 */
	smp_mb();

	if (unlikely(netif_tx_queue_stopped(txq) &&
		     (al_udma_available_get(tx_ring->dma_q) > AL_ETH_TX_WAKEUP_THRESH))) {
		__netif_tx_lock(txq, smp_processor_id());
		if (netif_tx_queue_stopped(txq) &&
		    (al_udma_available_get(tx_ring->dma_q) > AL_ETH_TX_WAKEUP_THRESH))
			netif_tx_wake_queue(txq);
		__netif_tx_unlock(txq);
	}

	/* all work done, exit the polling mode */
	napi_complete(napi);
	al_reg_write32_relaxed(tx_ring->unmask_reg_offset, tx_ring->unmask_val);
	return 0;
}

/**
 * al_eth_rx_checksum - indicate in skb if hw indicated a good cksum
 * @adapter: structure containing adapter specific data
 * @hal_pkt: HAL structure for the packet
 * @skb: skb currently being received and modified
 **/
static inline void al_eth_rx_checksum(struct al_eth_adapter *adapter,
				      struct al_eth_pkt *hal_pkt,
				      struct sk_buff *skb)
{
	skb_checksum_none_assert(skb);

	/* Rx csum disabled */
	if (unlikely(!(adapter->netdev->features & NETIF_F_RXCSUM))) {
		netdev_dbg(adapter->netdev, "hw checksum offloading disabled\n");
		return;
	}

	/* if IP and error */
	if (unlikely((hal_pkt->l3_proto_idx == AL_ETH_PROTO_ID_IPv4) &&
		(hal_pkt->flags & AL_ETH_RX_FLAGS_L3_CSUM_ERR))) {
		/* ipv4 checksum error */
		netdev_dbg(adapter->netdev, "rx ipv4 header checksum error\n");
		return;
	}

	/* if TCP/UDP */
	if (likely((hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_TCP) ||
	   (hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_UDP))) {
		/* TODO: check if we need the test above for TCP/UDP */
		if (unlikely(hal_pkt->flags & AL_ETH_RX_FLAGS_L4_CSUM_ERR)) {
			/* TCP/UDP checksum error */
			netdev_dbg(adapter->netdev, "rx L4 checksum error\n");
			return;
		} else {
			netdev_dbg(adapter->netdev, "rx checksum correct\n");
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		}
	}
}

/**
 * al_eth_rx_poll - NAPI Rx polling callback
 * @napi: structure for representing this polling device
 * @budget: how many packets driver is allowed to clean
 *
 * This function is used for legacy and MSI, NAPI mode
 **/
static int
al_eth_rx_poll(struct napi_struct *napi, int budget)
{
	struct al_eth_napi *al_napi = container_of(napi, struct al_eth_napi, napi);
	struct al_eth_adapter *adapter = al_napi->adapter;
	unsigned int qid = al_napi->qid;
	struct al_eth_ring *rx_ring = &adapter->rx_ring[qid];
	struct al_eth_pkt *hal_pkt = &rx_ring->hal_pkt;
	int work_done = 0;
	u16 next_to_clean = rx_ring->next_to_clean;
	int refill_required;
	int refill_actual;
	unsigned int total_len = 0;
	unsigned int small_copy_len_pkt = 0;

	netdev_dbg(adapter->netdev, "%s qid %d\n", __func__, qid);

	do {
		struct sk_buff *skb;
		unsigned int descs;

		descs = al_eth_pkt_rx(rx_ring->dma_q, hal_pkt);
		if (unlikely(descs == 0))
			break;

		netdev_dbg(adapter->netdev, "rx_poll: q %d got packet from hal. descs %d\n",
					   qid, descs);
		netdev_dbg(adapter->netdev, "rx_poll: q %d flags %x. l3 proto %d l4 proto %d\n",
					qid, hal_pkt->flags, hal_pkt->l3_proto_idx,
					hal_pkt->l4_proto_idx);
		/**
		 * Increase packets counter
		 * will be used to adjust threshold if using adaptive interrupt moderation
		 **/
		rx_ring->packets++;

		/* ignore if detected dma or eth controller errors */
		if (hal_pkt->flags & (AL_ETH_RX_ERROR | AL_UDMA_CDESC_ERROR)) {
			netdev_dbg(adapter->netdev, "receive packet with error. flags = 0x%x\n", hal_pkt->flags);
			next_to_clean = AL_ETH_RX_RING_IDX_ADD(rx_ring, next_to_clean, descs);
			goto next;
		}

		/* allocate skb and fill it */
		skb = al_eth_rx_skb(adapter, rx_ring, hal_pkt, descs,
				&next_to_clean);

		/* exit if we failed to retrieve a buffer */
		if (unlikely(!skb)) {
			next_to_clean = AL_ETH_RX_RING_IDX_ADD(rx_ring,
					next_to_clean, descs);
			break;
		}

		al_eth_rx_checksum(adapter, hal_pkt, skb);

#if defined(NETIF_F_RXHASH) || defined(CONFIG_ARCH_ALPINE)
		/** Always do in linux kernel driver, on host-driver it is platform related */
		if (likely(adapter->netdev->features & NETIF_F_RXHASH)) {
			uint32_t rxhash;
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0) && !(RHEL_RELEASE_CODE)) || (RHEL_RELEASE_CODE && RHEL_RELEASE_CODE > RHEL_RELEASE_VERSION(7, 1)))
			enum pkt_hash_types type;
			/* This is necessary since the kernel expects a 32-bit hash, while our HW
			 * performs a 16-bit hash. This ensures a unique "final" hash for 16-bit
			 * and 32-bit hashes (if we add support in future HW).
			 * NOTICE: This hash isn't equal to the 32-bit hash which would
			 * be calculated by the kernel.
			 */
			rxhash = (((uint32_t)hal_pkt->rxhash) << 16) ^ hal_pkt->rxhash;
			if (likely((hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_TCP) ||
				   (hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_UDP)))
				/** Hash on L4 + L3 params : src_IP, dst_IP, src_port, dst_port */
				type = PKT_HASH_TYPE_L4;
			else
				/** Hash on L3 params : src_IP, dst_IP */
				type = PKT_HASH_TYPE_L3;
			/** set RX hash & L4/L3 hash */
			skb_set_hash(skb, rxhash, type);
#else
			rxhash = (((uint32_t)hal_pkt->rxhash) << 16) ^ hal_pkt->rxhash;
			skb->rxhash = rxhash;
			if (likely((hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_TCP) ||
				   (hal_pkt->l4_proto_idx == AL_ETH_PROTO_ID_UDP)))
				skb->l4_rxhash = 1;
#endif
		}
#endif /* defined(NETIF_F_RXHASH) || defined(CONFIG_ARCH_ALPINE) */

		skb_record_rx_queue(skb, qid);

		total_len += skb->len;
#ifdef CONFIG_AL_ETH_ALLOC_SKB
		netif_receive_skb(skb);
#else
		if (hal_pkt->bufs[0].len <= adapter->small_copy_len) {
			small_copy_len_pkt++;
			napi_gro_receive(napi, skb);
		} else
			napi_gro_frags(napi);
#endif

next:
		budget--;
		work_done++;
	} while (likely(budget));

	rx_ring->next_to_clean = next_to_clean;

	u64_stats_update_begin(&rx_ring->syncp);
	rx_ring->rx_stats.bytes += total_len;
	rx_ring->rx_stats.packets += work_done;
	rx_ring->rx_stats.small_copy_len_pkt += small_copy_len_pkt;
	u64_stats_update_end(&rx_ring->syncp);

	refill_required = al_udma_available_get(rx_ring->dma_q);
	refill_actual = al_eth_refill_rx_bufs(adapter, qid, refill_required);

	if (unlikely(refill_actual < refill_required)) {
		netdev_warn(adapter->netdev,
			"%s: rescheduling rx queue %d\n", __func__, qid);
		napi_reschedule(napi);
	} else if (budget > 0) {
		dev_dbg(&adapter->pdev->dev, "rx_poll: q %d done next to clean %x\n",
			qid, next_to_clean);
		napi_complete(napi);
		if(adapter->adaptive_intr_rate)
			al_eth_update_intr_moderation(adapter, qid, AL_ETH_RX);
		al_reg_write32_relaxed(rx_ring->unmask_reg_offset, rx_ring->unmask_val);
	}

	return work_done;
}

static void
al_eth_del_napi(struct al_eth_adapter *adapter)
{
	int i;
	int napi_num = adapter->num_rx_queues + adapter->num_tx_queues;

	for (i = 0; i < napi_num; i++)
		netif_napi_del(&adapter->al_napi[i].napi);
}

static void
al_eth_init_napi(struct al_eth_adapter *adapter)
{
	int i;
	int napi_num = adapter->num_rx_queues + adapter->num_tx_queues;

	for (i = 0; i < napi_num; i++) {
		struct al_eth_napi *napi = &adapter->al_napi[i];
		int (*poll)(struct napi_struct *, int);

		if (i < adapter->num_rx_queues) {
			poll = al_eth_rx_poll;
			napi->qid = i;
		} else {
			poll = al_eth_tx_poll;
			napi->qid = i - adapter->num_rx_queues;
		}
		netif_napi_add(adapter->netdev, &adapter->al_napi[i].napi, poll, 64);
		napi->adapter = adapter;
	}
}


static void
al_eth_napi_disable_all(struct al_eth_adapter *adapter)
{
	int i;
	int napi_num = adapter->num_rx_queues + adapter->num_tx_queues;

	for (i = 0; i < napi_num; i++)
		napi_disable(&adapter->al_napi[i].napi);
}

static void
al_eth_napi_enable_all(struct al_eth_adapter *adapter)

{
	int i;
	int napi_num = adapter->num_rx_queues + adapter->num_tx_queues;

	for (i = 0; i < napi_num; i++)
		napi_enable(&adapter->al_napi[i].napi);
}

static void al_eth_restore_ethtool_params(struct al_eth_adapter *adapter)
{
	int i;
	unsigned int tx_usecs = adapter->tx_usecs;
	unsigned int rx_usecs = adapter->rx_usecs;

	adapter->tx_usecs = 0;
	adapter->rx_usecs = 0;

	al_eth_set_coalesce(adapter, tx_usecs, rx_usecs);

	for (i = 0; i < AL_ETH_RX_RSS_TABLE_SIZE; i++)
		al_eth_thash_table_set(&adapter->hal_adapter, i, adapter->udma_num, adapter->rss_ind_tbl[i]);
}

static void al_eth_up_complete(struct al_eth_adapter *adapter)
{
	al_eth_configure_int_mode(adapter);

	/*config rx fwd*/
	al_eth_config_rx_fwd(adapter);

	al_eth_init_napi(adapter);
	al_eth_napi_enable_all(adapter);

	al_eth_change_mtu(adapter->netdev, adapter->netdev->mtu);
	/* enable hw queues */
	al_eth_udma_queues_enable_all(adapter);

	al_eth_refill_all_rx_bufs(adapter);

	al_eth_interrupts_unmask(adapter);

#if !defined(CONFIG_ARCH_ALPINE) && defined(CONFIG_AL_ETH_SRIOV)
	if (adapter->pdev->is_physfn) {
		/* enable forwarding interrupts from eth through pci end point*/
		if (IS_NIC(adapter->board_type))
			writel(0x1FFFF, adapter->internal_pcie_base + 0x1800000
					+ 0x1210);
	}
#else
	/** Linux kernel / Alpine integrated driver */
	/* enable forwarding interrupts from eth through pci end point*/
	if (IS_NIC(adapter->board_type))
		writel(0x1FFFF, adapter->internal_pcie_base + 0x1800000
				+ 0x1210);
#endif

	/* enable transmits */
	netif_tx_start_all_queues(adapter->netdev);

	/* enable flow control */
	al_eth_flow_ctrl_enable(adapter);

	al_eth_restore_ethtool_params(adapter);

	/* enable the mac tx and rx paths */
	al_eth_mac_start(&adapter->hal_adapter);
}

static int al_eth_up(struct al_eth_adapter *adapter)
{
	int rc;

	netdev_dbg(adapter->netdev, "%s\n", __func__);

	if (adapter->flags & AL_ETH_FLAG_RESET_REQUESTED) {
		al_eth_function_reset(adapter);
		adapter->flags &= ~AL_ETH_FLAG_RESET_REQUESTED;
	}

	rc = al_eth_hw_init(adapter);
	if (rc)
		goto err_hw_init_open;

	rc = al_eth_setup_int_mode(adapter, disable_msi);
	if (rc) {
		dev_err(&adapter->pdev->dev, "%s failed at setup interrupt mode!\n", __func__);
		goto err_setup_int;
	}

	/* allocate transmit descriptors */
	rc = al_eth_setup_all_tx_resources(adapter);
	if (rc)
		goto err_setup_tx;

	/* allocate receive descriptors */
	rc = al_eth_setup_all_rx_resources(adapter);
	if (rc)
		goto err_setup_rx;

	rc = al_eth_request_irq(adapter);
	if (rc)
		goto err_req_irq;

	al_eth_up_complete(adapter);

	adapter->up = true;

	return rc;

err_req_irq:
	al_eth_free_all_rx_resources(adapter);
err_setup_rx:
	al_eth_free_all_tx_resources(adapter);
err_setup_tx:
	al_eth_free_irq(adapter);
err_setup_int:
	al_eth_hw_stop(adapter);
err_hw_init_open:
	al_eth_function_reset(adapter);

	return rc;
}

#ifdef CONFIG_RFS_ACCEL
static int
al_eth_flow_steer(struct net_device *netdev, const struct sk_buff *skb,
	       u16 rxq_index, u32 flow_id)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	int rc = 0;

	if ((skb->protocol != htons(ETH_P_IP)) && (skb->protocol != htons(ETH_P_IPV6)))
		return -EPROTONOSUPPORT;

	if (skb->protocol == htons(ETH_P_IP)) {
		if (ip_is_fragment(ip_hdr(skb)))
			return -EPROTONOSUPPORT;
		if ((ip_hdr(skb)->protocol != IPPROTO_TCP) && (ip_hdr(skb)->protocol != IPPROTO_UDP))
			return -EPROTONOSUPPORT;
	}

	if (skb->protocol == htons(ETH_P_IPV6)) {
		/* ipv6 with extension not supported yet */
		if ((ipv6_hdr(skb)->nexthdr != IPPROTO_TCP) && (ipv6_hdr(skb)->nexthdr != IPPROTO_UDP))
			return -EPROTONOSUPPORT;
	}
	rc = flow_id & (AL_ETH_RX_THASH_TABLE_SIZE - 1);

	adapter->rss_ind_tbl[rc] = rxq_index;
	al_eth_thash_table_set(&adapter->hal_adapter, rc, adapter->udma_num, rxq_index);
	if (skb->protocol == htons(ETH_P_IP)) {
		int nhoff = skb_network_offset(skb);
		const struct iphdr *ip = (const struct iphdr *)(skb->data + nhoff);
		const __be16 *ports = (const __be16 *)(skb->data + nhoff + 4 * ip->ihl);

		netdev_info(adapter->netdev, "steering %s %pI4:%u:%pI4:%u to queue %u [flow %u filter %d]\n",
		   (ip->protocol == IPPROTO_TCP) ? "TCP" : "UDP",
		   &ip->saddr, ntohs(ports[0]), &ip->daddr, ntohs(ports[1]),
		   rxq_index, flow_id, rc);
	} else {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);
		const __be16 *ports = (const __be16 *)skb_transport_header(skb);

		netdev_info(adapter->netdev, "steering %s %pI6c:%u:%pI6c:%u to queue %u [flow %u filter %d]\n",
		   (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP) ? "TCP" : "UDP",
		   &ip6h->saddr, ntohs(ports[0]), &ip6h->daddr, ntohs(ports[1]),
		   rxq_index, flow_id, rc);

	}

	return rc;
}
#endif


#ifdef HAVE_NDO_SET_FEATURES
/** For kernels above KERNEL_VERSION(2,6,39) */
static int al_set_features(struct net_device *dev,
		    netdev_features_t features) {
#if defined(NETIF_F_MQ_TX_LOCK_OPT)
	if (((features ^ dev->features) & NETIF_F_MQ_TX_LOCK_OPT) && netif_running(dev)) {
		netdev_warn(dev,
			    "Can't toggle NETIF_F_MQ_TX_LOCK_OPT : device is running!  \n");
		return -EINVAL;
	}
#endif
	return 0;
}
#endif

/************************ Link management ************************/

#if defined(CONFIG_ARCH_ALPINE)
static int al_eth_i2c_data_read(void *context,
				uint8_t bus_id,
				uint8_t i2c_addr,
				uint8_t reg_addr,
				uint8_t *val,
				size_t len)
{
	struct i2c_adapter *i2c_adapter;
	struct al_eth_adapter *adapter = context;
	int rc = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},
		{
			.addr = i2c_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		}
	};

	i2c_adapter = i2c_get_adapter(bus_id);

	if (i2c_adapter == NULL) {
		netdev_err(adapter->netdev,
			   "Failed to get i2c adapter. "
			   "probably caused by wrong i2c bus id in the device tree, "
			   "wrong i2c mux implementation, or the port is configured wrongly as SFP+\n");
		return -EINVAL;
	}

	rc = i2c_transfer(i2c_adapter, msgs, ARRAY_SIZE(msgs));
	i2c_put_adapter(i2c_adapter);
	if (rc != ARRAY_SIZE(msgs)) {
		netdev_dbg(adapter->netdev, "Failed to read sfp+ parameters\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int al_eth_i2c_data_write(void *context,
				 uint8_t bus_id,
				 uint8_t i2c_addr,
				 uint8_t reg_addr,
				 uint8_t *val,
				 size_t len)
{
	struct i2c_adapter *i2c_adapter;
	struct al_eth_adapter *adapter = context;
	struct i2c_msg msgs[1];
	int rc = 0;

	msgs[0].addr = i2c_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1 + len;
	msgs[0].buf = kmalloc(1 + len, GFP_KERNEL);

	if(NULL == msgs[0].buf) {
		netdev_err(adapter->netdev, "Unable to allocate i2c msg.\n");
		return -ENOMEM;
	}
	msgs[0].buf[0] = reg_addr;
	memcpy(&msgs[0].buf[1], val, len);

	i2c_adapter = i2c_get_adapter(bus_id);

	if (i2c_adapter == NULL) {
		kfree(msgs[0].buf);
		netdev_err(adapter->netdev,
			   "Failed to get i2c adapter. "
			   "probably caused by wrong i2c bus id in the device tree, "
			   "wrong i2c mux implementation, or the port is configured wrongly as SFP+\n");
		return -EINVAL;
	}

	rc = i2c_transfer(i2c_adapter, msgs, ARRAY_SIZE(msgs));
	i2c_put_adapter(i2c_adapter);
	kfree(msgs[0].buf);
	if (rc != ARRAY_SIZE(msgs)) {
		netdev_dbg(adapter->netdev, "Failed to read sfp+ parameters\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int al_eth_i2c_byte_read(void *context,
				uint8_t bus_id,
				uint8_t i2c_addr,
				uint8_t reg_addr,
				uint8_t *val)
{

	return al_eth_i2c_data_read(context, bus_id, i2c_addr, reg_addr, val, 1);
}

static int al_eth_i2c_byte_write(void *context,
				 uint8_t bus_id,
				 uint8_t i2c_addr,
				 uint8_t reg_addr,
				 uint8_t val)
{
	return al_eth_i2c_data_write(context, bus_id, i2c_addr, reg_addr, &val, 1);
}

static uint8_t al_eth_get_rand_byte(void)
{
	uint8_t byte;
	get_random_bytes(&byte, 1);
	return byte;
}

static void al_eth_lm_led_config_init_single(unsigned int gpio, const char *name)
{
	int err;

	if (!gpio)
		return;

	err = gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, name);
	if (err)
		pr_err("[%s] LED %s (%u): unable to request GPIO\n", __func__, name, gpio);
}

static void al_eth_lm_mode_apply(struct al_eth_adapter		*adapter,
				 enum al_eth_lm_link_mode	new_mode)
{

	if (new_mode == AL_ETH_LM_MODE_DISCONNECTED)
		return;

	if (new_mode == AL_ETH_LM_MODE_1G || new_mode == AL_ETH_LM_MODE_1G_DA) {
		adapter->mac_mode = AL_ETH_MAC_MODE_SGMII;
		adapter->link_config.active_speed = SPEED_1000;
	} else if (new_mode == AL_ETH_LM_MODE_25G) {
		adapter->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
		adapter->link_config.active_speed = SPEED_25000;
	} else {
		/* force 25G MAC mode when using 25G SerDes */
		if (adapter->serdes_obj->type_get() == AL_SRDS_TYPE_25G)
			adapter->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
		else
			adapter->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;

		adapter->link_config.active_speed = SPEED_10000;
	}

	adapter->link_config.active_duplex = DUPLEX_FULL;
}

static void al_eth_lm_led_config_init(struct al_eth_adapter *adapter)
{
	al_eth_lm_led_config_init_single(
		adapter->gpio_spd_1g, "1g");
	al_eth_lm_led_config_init_single(
		adapter->gpio_spd_10g, "10g");
	al_eth_lm_led_config_init_single(
		adapter->gpio_spd_25g, "25g");
}

static void al_eth_lm_led_config_terminate(struct al_eth_adapter *adapter)
{
	if (adapter->gpio_spd_1g)
		gpio_free(adapter->gpio_spd_1g);
	if (adapter->gpio_spd_10g)
		gpio_free(adapter->gpio_spd_10g);
	if (adapter->gpio_spd_25g)
		gpio_free(adapter->gpio_spd_25g);
}

static void al_eth_lm_led_config_single(unsigned int gpio, unsigned int val, const char *name)
{
	if (gpio) {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
		pr_debug("[%s] LED %s (%u) set to %u\n", __func__, name, gpio, val);
	}
}

static void al_eth_lm_led_config(void *context, struct al_eth_lm_led_config_data *data)
{
	struct al_eth_adapter *adapter = context;

	al_eth_lm_led_config_single(
		adapter->gpio_spd_1g, (data->speed == AL_ETH_LM_LED_CONFIG_1G), "1g");
	al_eth_lm_led_config_single(
		adapter->gpio_spd_10g, (data->speed == AL_ETH_LM_LED_CONFIG_10G), "10g");
	al_eth_lm_led_config_single(
		adapter->gpio_spd_25g, (data->speed == AL_ETH_LM_LED_CONFIG_25G), "25g");
}

static void al_eth_serdes_mode_set(struct al_eth_adapter *adapter)
{
	enum alpine_serdes_eth_mode mode =
		(adapter->mac_mode == AL_ETH_MAC_MODE_SGMII) ?
		ALPINE_SERDES_ETH_MODE_SGMII :
		ALPINE_SERDES_ETH_MODE_KR;

	if (alpine_serdes_eth_mode_set(adapter->serdes_grp, mode))
		netdev_err(
			adapter->netdev,
			"%s: alpine_serdes_eth_mode_set(%d, %d) failed!\n",
			__func__,
			adapter->serdes_grp,
			mode);

	al_udelay(1000);
}

static int al_eth_serdes_init(struct al_eth_adapter *adapter)
{
	void __iomem			*serdes_base;

	adapter->serdes_init = false;

	/*
	 * always call with group A to get the base address of
	 * all groups.
	 */
	serdes_base = alpine_serdes_resource_get(adapter->serdes_grp);

	if (!serdes_base) {
		netdev_err(adapter->netdev, "serdes_base get failed!\n");
		return -EIO;
	}

	adapter->serdes_obj = alpine_serdes_grp_obj_get(adapter->serdes_grp);

	if (!adapter->serdes_obj) {
		netdev_err(adapter->netdev, "serdes_obj get failed!\n");
		return -EIO;
	}

	adapter->serdes_init = true;

	return 0;
}
#endif /** CONFIG_ARCH_ALPINE */

static void al_eth_down(struct al_eth_adapter *adapter)
{
	netdev_info(adapter->netdev, "%s\n", __func__);

	adapter->up = false;
	adapter->last_link = false;

	netif_carrier_off(adapter->netdev);
	al_eth_disable_int_sync(adapter);
	al_eth_napi_disable_all(adapter);
	netif_tx_disable(adapter->netdev);
	al_eth_free_irq(adapter);
	al_eth_hw_stop(adapter);
	al_eth_del_napi(adapter);

	al_eth_free_all_tx_bufs(adapter);
	al_eth_free_all_rx_bufs(adapter);
	al_eth_free_all_tx_resources(adapter);
	al_eth_free_all_rx_resources(adapter);
}

#if defined(CONFIG_ARCH_ALPINE)
static void al_eth_link_status_task(struct work_struct *work)
{
	struct al_eth_adapter *adapter = container_of(to_delayed_work(work),
				       struct al_eth_adapter, link_status_task);
	int				rc;

	do {
		rc = al_eth_group_lm_link_manage(adapter->group_lm_context,
						 adapter->serdes_lane, (void *)adapter);
		/* sleep so another port can run link_status, to avoid deadlocks */
		usleep_range(1, 10);
	} while (rc == -EINPROGRESS);

	/* setting link status delay to 0 (through sysfs) will stop the task */
	if (adapter->link_poll_interval != 0) {
		unsigned long delay;

		delay = msecs_to_jiffies(adapter->link_poll_interval);

		schedule_delayed_work(&adapter->link_status_task, delay);
	}
}
#else /* defined(CONFIG_ARCH_ALPINE) */
static void al_eth_link_status_task_nic(struct work_struct *work)
{
	struct al_eth_adapter *adapter = container_of(to_delayed_work(work),
			       struct al_eth_adapter, link_status_task);
	struct al_eth_link_status status;

	al_eth_link_status_get(&adapter->hal_adapter, &status);

	if ((adapter->last_link == true) && (status.link_up == false)) {
		netdev_info(adapter->netdev, "%s link down\n", __func__);

		netif_carrier_off(adapter->netdev);
		adapter->last_link = false;
	} else if ((adapter->last_link == false) && (status.link_up == true)) {
		netdev_info(adapter->netdev, "%s link up\n", __func__);

		netif_carrier_on(adapter->netdev);
		adapter->last_link = true;
	}


	if (adapter->link_poll_interval != 0) {
		unsigned long delay;

		delay = msecs_to_jiffies(adapter->link_poll_interval);

		schedule_delayed_work(&adapter->link_status_task, delay);
	}
}
#endif /* defined(CONFIG_ARCH_ALPINE) */

#ifdef CONFIG_ARCH_ALPINE
static unsigned int al_eth_systime_msec_get(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return (unsigned int)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
}

static void al_eth_lm_config(struct al_eth_adapter *adapter)
{
	struct al_eth_lm_init_params	params = {0};
	int err;

	params.adapter = &adapter->hal_adapter;
	params.serdes_obj = adapter->serdes_obj;
	params.lane = adapter->serdes_lane;
	params.sfp_detection = adapter->sfp_detection_needed;
	if (adapter->sfp_detection_needed) {
		params.sfp_bus_id = adapter->i2c_adapter_id;
		params.sfp_i2c_addr = SFP_I2C_ADDR;
	}

	params.rx_equal = true;
	params.max_speed = adapter->max_speed;

	switch (adapter->max_speed) {
	case AL_ETH_LM_MAX_SPEED_MAX:
	case AL_ETH_LM_MAX_SPEED_25G:
		params.default_mode = AL_ETH_LM_MODE_25G;
		params.rx_equal = false;
		params.sfp_detect_force_mode = true;
		break;
	case AL_ETH_LM_MAX_SPEED_10G:
		if (adapter->lt_en && adapter->an_en)
			params.default_mode = AL_ETH_LM_MODE_10G_DA;
		else
			params.default_mode = AL_ETH_LM_MODE_10G_OPTIC;
		break;
	case AL_ETH_LM_MAX_SPEED_1G:
		params.default_mode = AL_ETH_LM_MODE_1G;
		break;
	default:
		netdev_warn(adapter->netdev, "Unknown max speed, using default\n");
		params.default_mode = AL_ETH_LM_MODE_10G_DA;
	}

	params.link_training = adapter->lt_en;
	params.static_values = !adapter->dont_override_serdes;
	params.i2c_read = &al_eth_i2c_byte_read;
	params.i2c_write = &al_eth_i2c_byte_write;
	params.i2c_read_data = &al_eth_i2c_data_read;
	params.i2c_write_data = &al_eth_i2c_data_write;
	params.i2c_context = adapter;
	params.get_random_byte = &al_eth_get_rand_byte;
	params.kr_fec_enable = adapter->kr_fec_enable;

	params.retimer_exist = adapter->retimer.exist;
	params.retimer_type = adapter->retimer.type;
	params.retimer_bus_id = adapter->retimer.bus_id;
	params.retimer_i2c_addr = adapter->retimer.i2c_addr;
	params.retimer_channel = adapter->retimer.channel;
	params.retimer_tx_channel = adapter->retimer.tx_channel;
	params.speed_detection = adapter->speed_detection;

	params.auto_fec_enable = adapter->auto_fec_enable;
	params.auto_fec_initial_timeout = AUTO_FEC_INITIAL_TIMEOUT;
	params.auto_fec_toggle_timeout = AUTO_FEC_TOGGLE_TIMEOUT;
	params.get_msec = al_eth_systime_msec_get;
	params.led_config = &al_eth_lm_led_config;

	if (adapter->gpio_sfp_present) {
		err = gpio_request_one(adapter->gpio_sfp_present, GPIOF_IN, "sfp_present");
		if (err) {
			netdev_err(adapter->netdev,
				"Unable to request SFP present gpio %d, falling back to i2c polling\n",
				adapter->gpio_sfp_present);
		} else {
			params.gpio_present = adapter->gpio_sfp_present;
			params.gpio_get = gpio_get_value;
		}
	}

	al_eth_lm_led_config_init(adapter);

	al_eth_lm_init(&adapter->lm_context, &params);

}
#endif /* CONFIG_ARCH_ALPINE */

#define AQUANTIA_AQR105_ID			0x3a1b4a2

static int al_eth_aq_phy_fixup(struct phy_device *phydev)
{
	int temp = 0;

	temp = phy_read(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0x20));
	temp &= ~(1 << 12);

	phy_write(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0x20), temp);

	temp = phy_read(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0xc400));
	temp |= ((1 << 15) | (1 << 11) | (1 << 10));
	phy_write(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0xc400), temp);

	temp = phy_read(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0));
	temp |= (1 << 9);
	temp &= ~(1 << 15);

	phy_write(phydev, (MII_ADDR_C45 | (7 * 0x10000) | 0), temp);

	return 0;
}

#if defined(CONFIG_ARCH_ALPINE)
/* This function is invoked in group_lm before changing serdes speed
 * (even if there is no need to change speed)
 */
static int al_eth_lm_mode_change(void *handle, enum al_eth_lm_link_mode old_mode,
				 enum al_eth_lm_link_mode new_mode)
{
	struct al_eth_adapter		*adapter = (struct al_eth_adapter *)handle;

	if (old_mode != AL_ETH_LM_MODE_DISCONNECTED) {
		netdev_info(adapter->netdev, "%s link down\n", __func__);
		adapter->last_link = false;
		al_eth_down(adapter);
	}

	al_eth_lm_mode_apply(adapter, new_mode);

	return 0;
}

/* This function is invoked in group_lm after serdes speed change (even if speed wasn't changed) */
static int al_eth_group_lm_pre_establish(void *handle, enum al_eth_lm_link_mode old_mode,
					 enum al_eth_lm_link_mode new_mode)
{
	struct al_eth_adapter		*adapter = (struct al_eth_adapter *)handle;
	int rc;

	if (new_mode != AL_ETH_LM_MODE_DISCONNECTED) {
		if (!adapter->up) {
			/* pre_establish can be called several times until link is established
			 * but we must call al_eth_up only once
			 */
			rc = al_eth_up(adapter);
			if (rc)
				return rc;
		}
	} else {
		return -ENETDOWN;
	}

	return 0;
}

static void al_eth_group_lm_update_port_status(void *handle, int link_up)
{
	struct al_eth_adapter		*adapter = (struct al_eth_adapter *)handle;

	if (link_up)
		netif_carrier_on(adapter->netdev);
	else
		netif_carrier_off(adapter->netdev);
}
#endif /* defined(CONFIG_ARCH_ALPINE) */

/**
 * al_eth_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int al_eth_open(struct net_device *netdev)
{
	struct al_eth_adapter		*adapter = netdev_priv(netdev);
	int				rc;
	unsigned long delay;
#if defined(CONFIG_ARCH_ALPINE)
	struct al_eth_group_lm_context	*group_lm_context = adapter->group_lm_context;
	struct al_eth_group_lm_link_params group_lm_link_params;
#endif
	netdev_dbg(adapter->netdev, "%s\n", __func__);
	netif_carrier_off(netdev);

	/* Notify the stack of the actual queue counts. */
	rc = netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
	if (rc)
		return rc;

	rc = netif_set_real_num_rx_queues(netdev, adapter->num_rx_queues);
	if (rc)
		return rc;

#ifdef CONFIG_ARCH_ALPINE
	al_eth_serdes_init(adapter);
#endif

	adapter->last_establish_failed = false;

	if (adapter->phy_exist == false) {
		if (adapter->use_lm) {
#if defined(CONFIG_ARCH_ALPINE)
			/** Driver is running LM */
			al_eth_lm_config(adapter);

			group_lm_link_params.lm_context = &adapter->lm_context;
			group_lm_link_params.eth_port_num = adapter->id_number;
			group_lm_link_params.auto_speed = adapter->auto_speed;
			/* Since each eth port exists independently in Linux,
			 * we want each to run the group_lm_flow
			 */
			group_lm_link_params.skip_group_flow = AL_FALSE;
			group_lm_link_params.init_cb = NULL;
			group_lm_link_params.lm_mode_change_cb = &al_eth_lm_mode_change;
			group_lm_link_params.update_link_status_cb =
				&al_eth_group_lm_update_port_status;
			group_lm_link_params.pre_establish_cb = &al_eth_group_lm_pre_establish;

			while (!group_lm_context->common_params.try_lock_cb(
						group_lm_context->common_params.serdes_grp))
				usleep_range(1, 10);

			al_eth_group_lm_port_register(group_lm_context, adapter->serdes_lane,
						      &group_lm_link_params);

			group_lm_context->common_params.unlock_cb(
				group_lm_context->common_params.serdes_grp);
#endif
		} else {
			/** Someone else is running LM */
			rc = al_eth_up(adapter);
			if (rc)
				return rc;
		}
	} else {
		rc = al_eth_up(adapter);
		if (rc)
			return rc;

		if (adapter->phy_fixup_needed) {
			rc = phy_register_fixup_for_uid(AQUANTIA_AQR105_ID, 0xffffffff,
							al_eth_aq_phy_fixup);
			if (rc)
				netdev_warn(adapter->netdev, "failed to register PHY fixup\n");
		}

#ifdef CONFIG_PHYLIB
		rc = al_eth_mdiobus_setup(adapter);
		if (rc) {
			netdev_err(netdev, "failed at mdiobus setup!\n");
			al_eth_down(adapter);
			return rc;
		}
#endif
	}

#ifdef CONFIG_PHYLIB
	if (adapter->mdio_bus) {
		rc = al_eth_phy_init(adapter);
		return rc;
	}
#endif

	u64_stats_update_begin(&adapter->syncp);
	adapter->dev_stats.interface_up++;
	u64_stats_update_end(&adapter->syncp);

	delay = msecs_to_jiffies(AL_ETH_FIRST_LINK_POLL_INTERVAL);
#ifdef CONFIG_ARCH_ALPINE
	if ((adapter->board_type == ALPINE_INTEGRATED) && (adapter->use_lm)) {
		INIT_DELAYED_WORK(&adapter->link_status_task,
				al_eth_link_status_task);
		schedule_delayed_work(&adapter->link_status_task, delay);
	} else
		netif_carrier_on(adapter->netdev);
#else
	if (IS_NIC(adapter->board_type)) {
		INIT_DELAYED_WORK(&adapter->link_status_task,
				al_eth_link_status_task_nic);
		schedule_delayed_work(&adapter->link_status_task, delay);
	}
#endif

	return rc;
}

/**
 * al_eth_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 */
static int al_eth_close(struct net_device *netdev)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	netdev_dbg(adapter->netdev, "%s\n", __func__);

	cancel_delayed_work_sync(&adapter->link_status_task);
	cancel_work_sync(&adapter->reset_task);

#if defined(CONFIG_ARCH_ALPINE)
	if (adapter->use_lm) {
		struct al_eth_group_lm_context *group_lm_context = adapter->group_lm_context;
		while (!group_lm_context->common_params.try_lock_cb(
				group_lm_context->common_params.serdes_grp))
			usleep_range(1, 10);

		al_eth_group_lm_port_unregister(group_lm_context,
						adapter->serdes_lane);

		group_lm_context->common_params.unlock_cb(
			group_lm_context->common_params.serdes_grp);
	}
#endif

#ifdef CONFIG_PHYLIB
	/** Stop PHY & MDIO BUS */
	if (adapter->phydev) {
		phy_stop(adapter->phydev);
		phy_disconnect(adapter->phydev);
		al_eth_mdiobus_teardown(adapter);
	}
#endif

	if (adapter->up)
		al_eth_down(adapter);

	u64_stats_update_begin(&adapter->syncp);
	adapter->dev_stats.interface_down++;
	u64_stats_update_end(&adapter->syncp);

#if defined(CONFIG_ARCH_ALPINE)
	if (adapter->use_lm) {
		al_eth_lm_led_config_terminate(adapter);
		if (adapter->gpio_sfp_present)
			gpio_free(adapter->gpio_sfp_present);
	}
#endif

	return 0;
}

static int
al_eth_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_board_params params;
	int rc;

#ifdef CONFIG_PHYLIB
	struct phy_device *phydev = adapter->phydev;

	if (phydev)
		return phy_ethtool_gset(phydev, ecmd);
#endif

	rc = al_eth_board_params_get(adapter->mac_base, &params);
	if (rc) {
		netdev_err(adapter->netdev, "Board info not available\n");
		return rc;
	}

	/* If we are in 25g_10g_autodetect, need to check the speed on the fly */
	if (params.media_type == AL_ETH_BOARD_MEDIA_TYPE_25G_10G_AUTO) {
		rc = al_eth_get_serdes_25g_speed(adapter, &adapter->link_config.active_speed);
		if (rc) {
			netdev_err(adapter->netdev, "Invalid speed read from 25G serdes\n");
			return rc;
		}
	}

	ecmd->speed = adapter->link_config.active_speed;
	ecmd->duplex = adapter->link_config.active_duplex;
	ecmd->autoneg = adapter->link_config.autoneg;

	return 0;
}

static int
al_eth_set_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	int rc = 0;
	struct al_eth_group_lm_context *group_lm_context = adapter->group_lm_context;
	struct al_eth_lm_link_config lm_link_config = {0};
#if defined(CONFIG_PHYLIB)
	struct phy_device *phydev = adapter->phydev;

	if (phydev)
		return phy_ethtool_sset(phydev, ecmd);
#endif

	/* in case no phy exist set only mac parameters */
	adapter->link_config.active_speed = ecmd->speed;
	adapter->link_config.active_duplex = ecmd->duplex;
	adapter->link_config.autoneg = ecmd->autoneg;

	if (adapter->use_lm) {
		lm_link_config.autoneg = adapter->link_config.autoneg;
		lm_link_config.duplex = adapter->link_config.active_duplex;
		lm_link_config.speed = (adapter->link_config.autoneg) ? 0 : adapter->link_config.active_speed;
		/* re-apply configuration on the fly */
		return al_eth_group_lm_link_conf_apply(group_lm_context, &lm_link_config);
	} else {
		if (adapter->up)
			dev_warn(&adapter->pdev->dev,
				"%s this action will take place in the next activation (up)\n",
				__func__);
	}

	return rc;
}

static int
al_eth_module_info(struct net_device *netdev,
				struct ethtool_modinfo *modinfo)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	return al_eth_lm_get_module_info(&adapter->lm_context, modinfo);
}

static int
al_eth_module_eeprom(struct net_device *netdev,
				  struct ethtool_eeprom *eeprom, u8 *data)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	return al_eth_lm_get_module_eeprom(&adapter->lm_context, eeprom, data);
}

static int al_eth_ethtool_set_coalesce(
			struct net_device *net_dev,
			struct ethtool_coalesce *coalesce)
{
	struct al_eth_adapter *adapter = netdev_priv(net_dev);
	unsigned int tx_usecs = adapter->tx_usecs;
	unsigned int rx_usecs = adapter->rx_usecs;
	uint sb_clk_freq = AL_ETH_REF_CLK_FREQ_TO_HZ(adapter->ref_clk_freq) / 1000;

	/* allow setting coalescing parameters only if adaptive coalescing  isn't enabled*/
	if (adapter->adaptive_intr_rate)
		return -EINVAL;

	if (coalesce->use_adaptive_tx_coalesce)
		return -EINVAL;

	if (coalesce->rx_coalesce_usecs != rx_usecs)
		rx_usecs = coalesce->rx_coalesce_usecs;
	else
		rx_usecs = coalesce->rx_coalesce_usecs_irq;

	if (coalesce->tx_coalesce_usecs != tx_usecs)
		tx_usecs = coalesce->tx_coalesce_usecs;
	else
		tx_usecs = coalesce->tx_coalesce_usecs_irq;

	if (tx_usecs > (uint)(255 * AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq) / 1000))
		return -EINVAL;
	if (rx_usecs > (uint)(255 * AL_ETH_INTR_MODERATION_RESOLUTION_NSECS(sb_clk_freq) / 1000))
		return -EINVAL;

	al_eth_set_coalesce(adapter, tx_usecs, rx_usecs);

	return 0;
}

static int al_eth_nway_reset(struct net_device *netdev)
{
#if defined(CONFIG_PHYLIB)
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct phy_device *phydev = adapter->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_start_aneg(phydev);
#else
	return -ENODEV;
#endif
}

static u32 al_eth_get_msglevel(struct net_device *netdev)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	return adapter->msg_enable;
}

static void al_eth_set_msglevel(struct net_device *netdev, u32 value)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	adapter->msg_enable = value;
}


#ifdef HAVE_NDO_GET_STATS64
static struct rtnl_link_stats64 *al_eth_get_stats64(struct net_device *netdev,
						    struct rtnl_link_stats64 *stats)
#else
static struct net_device_stats *al_eth_get_stats(struct net_device *netdev)
#endif
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_mac_stats *mac_stats = &adapter->mac_stats;
#ifdef HAVE_NDO_GET_STATS64
	if (!adapter->up)
		return NULL;
#else
	struct net_device_stats *stats;
#ifdef HAVE_NETDEV_STATS_IN_NETDEV
	stats = &netdev->net_stats;
#else
	stats = &adapter->net_stats;
#endif /* HAVE_NETDEV_STATS_IN_NETDEV */
	if (!adapter->up) {
		memset(stats, 0, sizeof(*stats));
		return stats;
	}
#endif /* HAVE_NDO_GET_STATS64 */

	al_eth_mac_stats_get(&adapter->hal_adapter, mac_stats);

	stats->rx_packets = mac_stats->aFramesReceivedOK; /* including pause frames */
	stats->tx_packets = mac_stats->aFramesTransmittedOK; /* including pause frames */
	stats->rx_bytes = mac_stats->aOctetsReceivedOK;
	stats->tx_bytes = mac_stats->aOctetsTransmittedOK;
	stats->rx_dropped = 0;
	stats->multicast = mac_stats->ifInMulticastPkts;
	stats->collisions = 0;

	stats->rx_length_errors = (mac_stats->etherStatsUndersizePkts + /* good but short */
				   mac_stats->etherStatsFragments + /* short and bad*/
				   mac_stats->etherStatsJabbers + /* with crc errors */
				   mac_stats->etherStatsOversizePkts);
	stats->rx_crc_errors = mac_stats->aFrameCheckSequenceErrors;
	stats->rx_frame_errors = mac_stats->aAlignmentErrors;
	stats->rx_fifo_errors = mac_stats->etherStatsDropEvents;
	stats->rx_missed_errors = 0;
	stats->tx_window_errors = 0;

	stats->rx_errors = mac_stats->ifInErrors;
	stats->tx_errors = mac_stats->ifOutErrors;

	return stats;
}

static void
rtl83xx_read(struct net_device *dev, struct ifreq *ifr)
{
	int rc = 0;
	struct al_eth_adapter *adapter = netdev_priv(dev);
	struct mii_ioctl_data *mdio = if_mii(ifr);

	rc = al_eth_mdio_read(&adapter->hal_adapter, mdio->phy_id,
			MDIO_DEVAD_NONE, mdio->reg_num, &mdio->val_out);
	if (rc < 0)
		netdev_err(adapter->netdev, "%s: failed rc = %d\n", __func__, rc);
}

static void
rtl83xx_write(struct net_device *dev, struct ifreq *ifr)
{
	int rc = 0;
	struct al_eth_adapter *adapter = netdev_priv(dev);
	struct mii_ioctl_data *mdio = if_mii(ifr);

	rc = al_eth_mdio_write(&adapter->hal_adapter, mdio->phy_id,
			MDIO_DEVAD_NONE, mdio->reg_num, mdio->val_in);
	if (rc < 0)
		netdev_err(adapter->netdev, "%s: failed rc = %d\n", __func__, rc);
}

static void
al_eth_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);

	strlcpy(info->driver, DRV_MODULE_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, pci_name(adapter->pdev), sizeof(info->bus_info));
}

static void
al_eth_get_ringparam(struct net_device *netdev, struct ethtool_ringparam *ring)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_ring *tx_ring = &adapter->tx_ring[0];
	struct al_eth_ring *rx_ring = &adapter->rx_ring[0];

	ring->rx_max_pending = AL_ETH_DEFAULT_RX_DESCS;
	ring->tx_max_pending = AL_ETH_DEFAULT_TX_SW_DESCS;
	ring->rx_pending = rx_ring->sw_count;
	ring->tx_pending = tx_ring->sw_count;
}

static void
al_eth_get_pauseparam(struct net_device *netdev,
			 struct ethtool_pauseparam *pause)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_link_config *link_config = &adapter->link_config;

	pause->autoneg = ((link_config->flow_ctrl_active &
					AL_ETH_FLOW_CTRL_AUTONEG) != 0);
	pause->rx_pause = ((link_config->flow_ctrl_active &
					AL_ETH_FLOW_CTRL_RX_PAUSE) != 0);
	pause->tx_pause = ((link_config->flow_ctrl_active &
					AL_ETH_FLOW_CTRL_TX_PAUSE) != 0);
}

static int
al_eth_set_pauseparam(struct net_device *netdev,
			 struct ethtool_pauseparam *pause)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_link_config *link_config = &adapter->link_config;
#ifdef CONFIG_PHYLIB
	uint32_t newadv;
#endif

	/* auto negotiation and receive pause are currently not supported */
	if (pause->autoneg == AUTONEG_ENABLE)
		return -EINVAL;

	link_config->flow_ctrl_supported = 0;

	if (pause->rx_pause)
		link_config->flow_ctrl_supported |= AL_ETH_FLOW_CTRL_RX_PAUSE;

	if (pause->tx_pause)
		link_config->flow_ctrl_supported |= AL_ETH_FLOW_CTRL_TX_PAUSE;

#ifdef CONFIG_PHYLIB
	if (pause->tx_pause & pause->rx_pause)
		newadv = ADVERTISED_Pause;
	else if (pause->rx_pause)
		newadv = ADVERTISED_Pause | ADVERTISED_Asym_Pause;
	else if (pause->tx_pause)
		newadv = ADVERTISED_Asym_Pause;
	else
		newadv = 0;

	if (pause->autoneg) {
		struct phy_device *phydev;
		uint32_t oldadv;

		phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
		oldadv = phydev->advertising &
				     (ADVERTISED_Pause | ADVERTISED_Asym_Pause);
		link_config->flow_ctrl_supported |= AL_ETH_FLOW_CTRL_AUTONEG;

		if (oldadv != newadv) {
			phydev->advertising &= ~(ADVERTISED_Pause |
							ADVERTISED_Asym_Pause);
			phydev->advertising |= newadv;

			if (phydev->autoneg)
				return phy_start_aneg(phydev);
		}
	} else {
		link_config->flow_ctrl_active = link_config->flow_ctrl_supported;
		al_eth_flow_ctrl_config(adapter);
	}
#else
	link_config->flow_ctrl_active = link_config->flow_ctrl_supported;
	al_eth_flow_ctrl_config(adapter);
#endif
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static int
al_eth_get_rxnfc(struct net_device *netdev, struct ethtool_rxnfc *info,
		 u32 *rules __always_unused)
{
	/*struct al_eth_adapter *adapter = netdev_priv(netdev);*/

	switch (info->cmd) {
	case ETHTOOL_GRXRINGS:
		info->data = AL_ETH_NUM_QUEUES;
		return 0;
/*	case ETHTOOL_GRXFH:
		return bnx2x_get_rss_flags(bp, info);
*/
	default:
		netdev_err(netdev, "Command parameters not supported\n");
		return -EOPNOTSUPP;
	}
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static u32 al_eth_get_rxfh_indir_size(struct net_device *netdev)
{
	return AL_ETH_RX_RSS_TABLE_SIZE;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
static int al_eth_get_rxfh_indir(struct net_device *netdev, u32 *indir, u8 *key, u8 *hfunc)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
static int al_eth_get_rxfh_indir(struct net_device *netdev, u32 *indir, u8 *key)
#else
static int al_eth_get_rxfh_indir(struct net_device *netdev, u32 *indir)
#endif
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	int i;

	for (i = 0; i < AL_ETH_RX_RSS_TABLE_SIZE; i++)
		indir[i] = adapter->rss_ind_tbl[i];

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
static int al_eth_set_rxfh_indir(struct net_device *netdev, const u32 *indir, const u8 *key,
	const u8 hfunc)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
static int al_eth_set_rxfh_indir(struct net_device *netdev, const u32 *indir, const u8 *key)
#else
static int al_eth_set_rxfh_indir(struct net_device *netdev, const u32 *indir)
#endif
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	size_t i;

	for (i = 0; i < AL_ETH_RX_RSS_TABLE_SIZE; i++) {
		adapter->rss_ind_tbl[i] = indir[i];
		al_eth_thash_table_set(&adapter->hal_adapter, i, adapter->udma_num, indir[i]);
	}

	return 0;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static void al_eth_get_channels(struct net_device *netdev,
				struct ethtool_channels *channels)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	channels->max_rx = AL_ETH_NUM_QUEUES;
	channels->max_tx = AL_ETH_NUM_QUEUES;
	channels->max_other = 0;
	channels->max_combined = 0;
	channels->rx_count = adapter->num_rx_queues;
	channels->tx_count = adapter->num_tx_queues;
	channels->other_count = 0;
	channels->combined_count = 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
#if defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE)
static int al_eth_get_eee(struct net_device *netdev,
			  struct ethtool_eee *edata)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_eee_params params;


	if (!adapter->phy_exist)
		return -EOPNOTSUPP;

	al_eth_eee_get(&adapter->hal_adapter, &params);

	edata->eee_enabled = params.enable;
	edata->tx_lpi_timer = params.tx_eee_timer;

	return phy_ethtool_get_eee(adapter->phydev, edata);
}


static int al_eth_set_eee(struct net_device *netdev,
			  struct ethtool_eee *edata)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct al_eth_eee_params params;

	struct phy_device *phydev;

	if (!adapter->phy_exist)
		return -EOPNOTSUPP;

	phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];

	phy_init_eee(phydev, 1);

	params.enable = edata->eee_enabled;
	params.tx_eee_timer = edata->tx_lpi_timer;
	params.min_interval = 10;

	al_eth_eee_config(&adapter->hal_adapter, &params);

	return phy_ethtool_set_eee(phydev, edata);
}
#else /* defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE) */
static int al_eth_get_eee(struct net_device *netdev,
			  struct ethtool_eee *edata)
{
	return -EOPNOTSUPP;
}

static int al_eth_set_eee(struct net_device *netdev,
			  struct ethtool_eee *edata)
{
	return -EOPNOTSUPP;
}
#endif /* defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE) */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
#if defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE)
static void al_eth_get_wol(struct net_device *netdev,
			   struct ethtool_wolinfo *wol)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct phy_device *phydev;

	wol->wolopts = adapter->wol;

	if ((adapter) && (adapter->phy_exist) && (adapter->mdio_bus)) {
		phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
		if (phydev) {
			phy_ethtool_get_wol(phydev, wol);
			wol->supported |= WAKE_PHY;
			return;
		}
	}

	wol->supported |= WAKE_UCAST | WAKE_MCAST | WAKE_BCAST;
}

static int al_eth_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	struct phy_device *phydev;

	if (wol->wolopts & (WAKE_ARP | WAKE_MAGICSECURE))
		return -EOPNOTSUPP;

	adapter->wol = wol->wolopts;

	if ((adapter) && (adapter->phy_exist) && (adapter->mdio_bus)) {
		phydev = adapter->mdio_bus->phy_map[adapter->phy_addr];
		if (phydev)
			return phy_ethtool_set_wol(phydev, wol);
	}

	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	return 0;
}
#else /* defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE) */
static void al_eth_get_wol(struct net_device *netdev,
			   struct ethtool_wolinfo *wol)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	wol->wolopts = adapter->wol;

	wol->supported |= WAKE_UCAST | WAKE_MCAST | WAKE_BCAST;
}

static int al_eth_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	if (wol->wolopts & (WAKE_ARP | WAKE_MAGICSECURE))
		return -EOPNOTSUPP;

	adapter->wol = wol->wolopts;

	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	return 0;
}
#endif /* defined(CONFIG_PHYLIB) || defined(CONFIG_ARCH_ALPINE) */
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)) */


/****************** ETHTOOL_STATS BEGIN ******************/

struct al_eth_ethtool_stats {
	char name[ETH_GSTRING_LEN];
	int stat_offset;
};

#define AL_ETH_ETHTOOL_STAT_ENTRY(stat, stat_type) { \
	.name = #stat, \
	.stat_offset = offsetof(struct al_eth_stats_##stat_type, stat) \
}

#define AL_ETH_ETHTOOL_STAT_RX_ENTRY(stat) \
	AL_ETH_ETHTOOL_STAT_ENTRY(stat, rx)

#define AL_ETH_ETHTOOL_STAT_TX_ENTRY(stat) \
	AL_ETH_ETHTOOL_STAT_ENTRY(stat, tx)

#define AL_ETH_ETHTOOL_STAT_GLOBAL_ENTRY(stat) \
	AL_ETH_ETHTOOL_STAT_ENTRY(stat, dev)

static const struct al_eth_ethtool_stats al_eth_ethtool_stats_global_strings[] = {
	AL_ETH_ETHTOOL_STAT_GLOBAL_ENTRY(tx_timeout),
	AL_ETH_ETHTOOL_STAT_GLOBAL_ENTRY(interface_up),
	AL_ETH_ETHTOOL_STAT_GLOBAL_ENTRY(interface_down),
};

static const struct al_eth_ethtool_stats al_eth_ethtool_stats_tx_strings[] = {
	AL_ETH_ETHTOOL_STAT_TX_ENTRY(packets),
	AL_ETH_ETHTOOL_STAT_TX_ENTRY(bytes),
};

static const struct al_eth_ethtool_stats al_eth_ethtool_stats_rx_strings[] = {
	AL_ETH_ETHTOOL_STAT_RX_ENTRY(packets),
	AL_ETH_ETHTOOL_STAT_RX_ENTRY(bytes),
	AL_ETH_ETHTOOL_STAT_RX_ENTRY(skb_alloc_fail),
	AL_ETH_ETHTOOL_STAT_RX_ENTRY(dma_mapping_err),
	AL_ETH_ETHTOOL_STAT_RX_ENTRY(small_copy_len_pkt),
};

#define AL_ETH_ETHTOOL_STATS_ARRAY_GLOBAL	ARRAY_SIZE(al_eth_ethtool_stats_global_strings)
#define AL_ETH_ETHTOOL_STATS_ARRAY_TX	ARRAY_SIZE(al_eth_ethtool_stats_tx_strings)
#define AL_ETH_ETHTOOL_STATS_ARRAY_RX	ARRAY_SIZE(al_eth_ethtool_stats_rx_strings)

static void al_eth_safe_update_stat(u64 *src, u64 *dst,
				 struct u64_stats_sync *syncp)
{
	unsigned int start;

	do {
		start = u64_stats_fetch_begin_irq(syncp);
		*(dst) = *src;
	} while (u64_stats_fetch_retry_irq(syncp, start));
}

static void al_eth_ethtool_queue_stats(struct al_eth_adapter *adapter, u64 **data)
{
	const struct al_eth_ethtool_stats *al_eth_ethtool_stats;
	struct al_eth_ring *ring;

	u64 *ptr;
	int i, j;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		/* Tx stats */
		ring = &adapter->tx_ring[i];

		for (j = 0; j < AL_ETH_ETHTOOL_STATS_ARRAY_TX; j++) {
			al_eth_ethtool_stats = &al_eth_ethtool_stats_tx_strings[j];

			ptr = (u64 *)((uintptr_t)&ring->tx_stats +
				(uintptr_t)al_eth_ethtool_stats->stat_offset);

			al_eth_safe_update_stat(ptr, (*data)++, &ring->syncp);
		}
	}

	for (i = 0; i < adapter->num_rx_queues; i++) {
		/* Rx stats */
		ring = &adapter->rx_ring[i];

		for (j = 0; j < AL_ETH_ETHTOOL_STATS_ARRAY_RX; j++) {
			al_eth_ethtool_stats = &al_eth_ethtool_stats_rx_strings[j];

			ptr = (u64 *)((uintptr_t)&ring->rx_stats +
				(uintptr_t)al_eth_ethtool_stats->stat_offset);

			al_eth_safe_update_stat(ptr, (*data)++, &ring->syncp);
		}
	}
}

static void al_eth_get_ethtool_stats(struct net_device *netdev,
				  struct ethtool_stats *stats,
				  u64 *data)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	const struct al_eth_ethtool_stats *al_eth_ethtool_stats;
	u64 *ptr;
	int i;

	for (i = 0; i < AL_ETH_ETHTOOL_STATS_ARRAY_GLOBAL; i++) {
		al_eth_ethtool_stats = &al_eth_ethtool_stats_global_strings[i];

		ptr = (u64 *)((uintptr_t)&adapter->dev_stats +
			(uintptr_t)al_eth_ethtool_stats->stat_offset);

		al_eth_safe_update_stat(ptr, data++, &adapter->syncp);
	}

	al_eth_ethtool_queue_stats(adapter, &data);
}

int al_eth_get_sset_count(struct net_device *netdev, int sset)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return  (adapter->num_tx_queues * AL_ETH_ETHTOOL_STATS_ARRAY_TX)
		+ (adapter->num_rx_queues * AL_ETH_ETHTOOL_STATS_ARRAY_RX)
		+ AL_ETH_ETHTOOL_STATS_ARRAY_GLOBAL;
}

static void al_eth_queue_strings(struct al_eth_adapter *adapter, u8 **data)
{
	const struct al_eth_ethtool_stats *al_eth_ethtool_stats;
	int i, j;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		/* Tx stats */
		for (j = 0; j < AL_ETH_ETHTOOL_STATS_ARRAY_TX; j++) {
			al_eth_ethtool_stats = &al_eth_ethtool_stats_tx_strings[j];

			snprintf(*data, ETH_GSTRING_LEN,
				 "queue_%u_tx_%s", i, al_eth_ethtool_stats->name);
			 (*data) += ETH_GSTRING_LEN;
		}
	}

	for (i = 0; i < adapter->num_rx_queues; i++) {
		/* Rx stats */
		for (j = 0; j < AL_ETH_ETHTOOL_STATS_ARRAY_RX; j++) {
			al_eth_ethtool_stats = &al_eth_ethtool_stats_rx_strings[j];

			snprintf(*data, ETH_GSTRING_LEN,
				 "queue_%u_rx_%s", i, al_eth_ethtool_stats->name);
			(*data) += ETH_GSTRING_LEN;
		}
	}
}

static void al_eth_get_strings(struct net_device *netdev, u32 sset, u8 *data)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);
	const struct al_eth_ethtool_stats *al_eth_ethtool_stats;
	int i;

	if (sset != ETH_SS_STATS)
		return;

	for (i = 0; i < AL_ETH_ETHTOOL_STATS_ARRAY_GLOBAL; i++) {
		al_eth_ethtool_stats = &al_eth_ethtool_stats_global_strings[i];

		memcpy(data, al_eth_ethtool_stats->name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}

	al_eth_queue_strings(adapter, &data);
}

/****************** ETHTOOL_STATS END ******************/

static const struct ethtool_ops al_eth_ethtool_ops = {
	.get_settings		= al_eth_get_settings,
	.set_settings		= al_eth_set_settings,
	.get_drvinfo		= al_eth_get_drvinfo,
/*	.get_regs_len		= al_eth_get_regs_len,*/
/*	.get_regs		= al_eth_get_regs,*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	.get_wol		= al_eth_get_wol,
	.set_wol		= al_eth_set_wol,
#endif
	.get_msglevel		= al_eth_get_msglevel,
	.set_msglevel		= al_eth_set_msglevel,

	.nway_reset		= al_eth_nway_reset,
	.get_link		= ethtool_op_get_link,
	.get_coalesce		= al_eth_get_coalesce,
	.set_coalesce		= al_eth_ethtool_set_coalesce,
	.get_ringparam		= al_eth_get_ringparam,
/*	.set_ringparam		= al_set_ringparam,*/
	.get_pauseparam		= al_eth_get_pauseparam,
	.set_pauseparam		= al_eth_set_pauseparam,
/*	.self_test		= al_eth_self_test,*/
	.get_strings		= al_eth_get_strings,
/*	.set_phys_id		= al_eth_set_phys_id,*/
	.get_ethtool_stats	= al_eth_get_ethtool_stats,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
	.get_rxnfc		= al_eth_get_rxnfc,
#endif
	.get_sset_count		= al_eth_get_sset_count,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	.get_rxfh_indir_size    = al_eth_get_rxfh_indir_size,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
	.get_rxfh			= al_eth_get_rxfh_indir,
	.set_rxfh			= al_eth_set_rxfh_indir,
#else
	.get_rxfh_indir		= al_eth_get_rxfh_indir,
	.set_rxfh_indir		= al_eth_set_rxfh_indir,
#endif
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
	.get_channels		= al_eth_get_channels,
/*	.set_channels		= al_eth_set_channels,*/
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
	.get_eee		= al_eth_get_eee,
	.set_eee		= al_eth_set_eee,
#endif
	.get_module_info	= al_eth_module_info,
	.get_module_eeprom	= al_eth_module_eeprom,
};

static const struct rtl83xx_ops al_eth_rtl83xx_ops = {
	.mdio_write		= rtl83xx_write,
	.mdio_read		= rtl83xx_read,
};

/* These numbers were obtained empirically by simulation of valid mss, header length
 * and payload length values
 */
#define AL_ETH_TSO_FIX_MIN_MSS 64
#define AL_ETH_TSO_FIX_FAST_MSS_THRESOLD 260

static int al_eth_is_mss_valid(u32 mss, u32 total_payload_len, u32 header_len)
{
	int res_payload_len;

	res_payload_len = total_payload_len % mss;

	/* check that the last packet which will be semgented by TSO, is larger than the
	 * ETH minimum length
	 */
	return (((res_payload_len) == 0) || ((header_len + res_payload_len) >= ETH_ZLEN));
}

/* This fix modifies (as little as possible) the MSS value so the last packet which
 * will be created by the TSO engine will be at least the ETH min frame size
 * NOTICE: In some cases this will create more packets than what originally would have been
 * segmented by the TSO engine with the original mss val.
 */
static int al_eth_tx_tso_mss_fix(struct sk_buff *skb, u32 orig_mss)
{
	int header_len, total_payload_len;
	int new_mss = orig_mss;
	int last_frame_pad;
	int i;

	header_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
	total_payload_len = skb->len - header_len;

	if (likely(al_eth_is_mss_valid(orig_mss, total_payload_len, header_len)))
		return 0;

	if (orig_mss < AL_ETH_TSO_FIX_MIN_MSS)
		return -EINVAL;

	/* last_frame_pad is always <=6. (60 - (ETH_HLEN + MIN_IP_HDR + MIN_TCP_HDR)) */
	last_frame_pad = ETH_ZLEN - header_len;
	if (orig_mss >= AL_ETH_TSO_FIX_FAST_MSS_THRESOLD) {
		/* if the number of packets is smaller then the pad length we directly set the mss
		 * so it will be enough for the case that the segment will be fragmented into just
		 * 2 frames.
		 */
		if ((total_payload_len / orig_mss) < last_frame_pad)
			new_mss = orig_mss - last_frame_pad;
		else
			new_mss = orig_mss - 1;
	} else {
		for (i = 1; i <= last_frame_pad; i++)
			if (al_eth_is_mss_valid(orig_mss - i, total_payload_len, header_len)) {
				new_mss = orig_mss - i;
				break;
			}
	}

	skb_shinfo(skb)->gso_size = new_mss;

	return 0;
}

static int
al_eth_tx_csum(struct al_eth_ring *tx_ring, struct al_eth_tx_buffer *tx_info,
	struct al_eth_pkt *hal_pkt, struct sk_buff *skb)
{
	u32 mss = skb_shinfo(skb)->gso_size;
	u8 l4_protocol = 0;
	int rc;

	if ((skb->ip_summed == CHECKSUM_PARTIAL) || mss) {
		struct al_eth_meta_data *meta = &tx_ring->hal_meta;
		if (mss)
			hal_pkt->flags |= AL_ETH_TX_FLAGS_TSO|AL_ETH_TX_FLAGS_L4_CSUM;
		else
			hal_pkt->flags |= AL_ETH_TX_FLAGS_L4_CSUM|AL_ETH_TX_FLAGS_L4_PARTIAL_CSUM;

		switch (ip_hdr(skb)->version) {
		case IPVERSION:
			hal_pkt->l3_proto_idx = AL_ETH_PROTO_ID_IPv4;
			if (mss)
				hal_pkt->flags |= AL_ETH_TX_FLAGS_IPV4_L3_CSUM;
			l4_protocol = ip_hdr(skb)->protocol;
			break;
		case 6:
			hal_pkt->l3_proto_idx = AL_ETH_PROTO_ID_IPv6;
			/* TODO: add support for csum offloading for ipv6 with options */
			l4_protocol = ipv6_hdr(skb)->nexthdr;
			break;
		default:
			break;
		}

		if (l4_protocol == IPPROTO_TCP) {
			hal_pkt->l4_proto_idx = AL_ETH_PROTO_ID_TCP;
			if (mss) {
				rc = al_eth_tx_tso_mss_fix(skb, mss);
				if (rc != 0) {
					pr_warn("Dropping skb due to invalid invalid MSS size (%dB) for TSO. skb_len:%d, header_len:%d\n",
						mss, skb->len,
						skb_transport_offset(skb) + tcp_hdrlen(skb));
					return rc;
				}
			}
		} else if (l4_protocol == IPPROTO_UDP)
			hal_pkt->l4_proto_idx = AL_ETH_PROTO_ID_UDP;
		else
			/* TODO: Need to add support for ipv6 when nexthdr is an ipv6 option */
			pr_warn("%s: l4 protocol isn't TCP(6) or UDP(17). proto:%d\n",
				__func__, l4_protocol);

		if (skb->protocol == __constant_htons(ETH_P_8021Q))
			hal_pkt->source_vlan_count = 1;
		else
			hal_pkt->source_vlan_count = 0;

		meta->words_valid = 4;
		meta->l3_header_len = skb_network_header_len(skb);
		meta->l3_header_offset = skb_network_offset(skb) -
			(VLAN_HLEN * hal_pkt->source_vlan_count);
		meta->l4_header_len = tcp_hdr(skb)->doff; /* this param needed only for TSO */
		meta->mss_idx_sel = 0; /* TODO: check how to select MSS */
		meta->mss_val = skb_shinfo(skb)->gso_size;
		hal_pkt->meta = meta;
	} else
		hal_pkt->meta = NULL;

	return 0;
}

/* Called with netif_tx_lock.
 */
static netdev_tx_t
al_eth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);
	dma_addr_t dma;
	struct al_eth_tx_buffer *tx_info;
	struct al_eth_pkt *hal_pkt;
	struct al_buf *al_buf;
	u32 len, last_frag;
	u16 next_to_use;
	int i = 0, qid;
	struct al_eth_ring *tx_ring;
	struct netdev_queue *txq;
	int rc;

	netdev_dbg(adapter->netdev, "%s skb %p\n", __func__, skb);
	/*  Determine which tx ring we will be placed on */
	qid = skb_get_queue_mapping(skb);
	tx_ring = &adapter->tx_ring[qid];
	txq = netdev_get_tx_queue(dev, qid);

	/* Need to pad the frame to ETH minimal, since MAC padding isn't zeroed
	 * when packet size is <60 and not 4-byte aligned
	 */
	if (eth_skb_pad(skb)) {
		netdev_dbg(adapter->netdev, "failed to pad skb to minimum ETH size (60B). skb->len = %d, skb->data_len= %d\n",
			   skb->len, skb->data_len);
		/* eth_skb_pad frees skb on error */
		return NETDEV_TX_OK;
	}

	next_to_use = tx_ring->next_to_use;
	tx_info = &tx_ring->tx_buffer_info[next_to_use];
	tx_info->skb = skb;
	hal_pkt = &tx_info->hal_pkt;

	/* set flags and meta data */
	hal_pkt->flags = AL_ETH_TX_FLAGS_INT;
	rc = al_eth_tx_csum(tx_ring, tx_info, hal_pkt, skb);
	if (rc != 0)
		goto tx_csum_error;

	al_buf = hal_pkt->bufs;

	len = skb_headlen(skb);
	dma = dma_map_single(tx_ring->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_ring->dev, dma)) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	dma_unmap_addr_set(al_buf, addr, dma);
	dma_unmap_len_set(al_buf, len, len);

	last_frag = skb_shinfo(skb)->nr_frags;

	for (i = 0; i < last_frag; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		al_buf++;

		len = skb_frag_size(frag);
		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(tx_ring->dev, dma))
			goto dma_error;
		dma_unmap_addr_set(al_buf, addr, dma);
		dma_unmap_len_set(al_buf, len, len);
	}

	hal_pkt->num_of_bufs = 1 + last_frag;
	if (unlikely(last_frag > (AL_ETH_PKT_MAX_BUFS - 2))) {
		int i;
		netdev_err(adapter->netdev, "too much descriptors. last_frag %d!\n", last_frag);
		for (i = 0; i <= last_frag; i++)
			netdev_err(adapter->netdev, "frag[%d]: addr:0x%llx, len 0x%x\n",
					i, (unsigned long long)hal_pkt->bufs[i].addr, hal_pkt->bufs[i].len);
		BUG();
	}
	netdev_tx_sent_queue(txq, skb->len);

	u64_stats_update_begin(&tx_ring->syncp);
	tx_ring->tx_stats.packets++;
	tx_ring->tx_stats.bytes += skb->len;
	u64_stats_update_end(&tx_ring->syncp);

	/*smp_wmb();*/ /* commit the item before incrementing the head */
	tx_ring->next_to_use = AL_ETH_TX_RING_IDX_NEXT(tx_ring, next_to_use);

	/* prepare the packet's descriptors to dma engine */
	tx_info->tx_descs = al_eth_tx_pkt_prepare(tx_ring->dma_q, hal_pkt);

	/* stop the queue when no more space available, the packet can have up
	 * to MAX_SKB_FRAGS + 1 buffers and a meta descriptor */
	if (unlikely(al_udma_available_get(tx_ring->dma_q) <
				(MAX_SKB_FRAGS + 2))) {
		dev_dbg(&adapter->pdev->dev, "%s stop queue %d\n", __func__, qid);
		netif_tx_stop_queue(txq);
	}

	/* trigger the dma engine */
	al_eth_tx_dma_action(tx_ring->dma_q, tx_info->tx_descs);

	return NETDEV_TX_OK;

tx_csum_error:
dma_error:
	/* save value of frag that failed */
	last_frag = i;

	/* start back at beginning and unmap skb */
	tx_info->skb = NULL;
	al_buf = hal_pkt->bufs;
	dma_unmap_single(tx_ring->dev, dma_unmap_addr(al_buf, addr),
			 dma_unmap_len(al_buf, len), DMA_TO_DEVICE);

	/* unmap remaining mapped pages */
	for (i = 0; i < last_frag; i++) {
		al_buf++;
		dma_unmap_page(tx_ring->dev, dma_unmap_addr(al_buf, addr),
			       dma_unmap_len(al_buf, len), DMA_TO_DEVICE);
	}

	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

#if (defined HAVE_NETDEV_SELECT_QUEUE) || (defined HAVE_NET_DEVICE_OPS)
/* Return subqueue id on this core (one per core). */
static u16 al_eth_select_queue(struct net_device *dev, struct sk_buff *skb
#if (defined HAVE_NDO_SELECT_QUEUE_ACCEL) || (defined HAVE_NDO_SELECT_QUEUE_ACCEL_FALLBACK)
	, void *accel_priv
#endif
#if (defined HAVE_NDO_SELECT_QUEUE_ACCEL_FALLBACK)
	, select_queue_fallback_t fallback
#endif
	)
{
	u16 qid;
	/* we suspect that this is good for in--kernel network services that want to loop incoming skb rx to tx
	 * in normal user generated traffic, most probably we will not get to this */
	if(skb_rx_queue_recorded(skb)) {
		qid = skb_get_rx_queue(skb);
		pr_debug("sel_rec_qid=%d\n", qid);
	}
	else {
#ifdef CONFIG_ARCH_ALPINE
		/** Is an integrated networking driver */
		qid = smp_processor_id();
#else
		/** Is a host/NIC mode driver */
#if (defined HAVE_NDO_SELECT_QUEUE_ACCEL_FALLBACK)
		qid = fallback(dev, skb);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		qid = __netdev_pick_tx(dev, skb);
#else
		qid = skb_tx_hash(dev, skb);
#endif /* HAVE_NDO_SELECT_QUEUE_ACCEL_FALLBACK */
#endif /* CONFIG_ARCH_ALPINE */
		pr_debug("sel_smp_qid=%d\n", qid);
	}
	return qid;
}
#endif /* (defined HAVE_NETDEV_SELECT_QUEUE) || (defined HAVE_NET_DEVICE_OPS) */


static int al_eth_set_mac_addr(struct net_device *dev, void *p)
{
	struct al_eth_adapter *adapter = netdev_priv(dev);
	struct sockaddr *addr = p;
	int err = 0;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	memcpy(adapter->mac_addr, addr->sa_data, dev->addr_len);
	al_eth_mac_table_unicast_add(adapter, AL_ETH_MAC_TABLE_UNICAST_IDX_BASE,
				     adapter->mac_addr, 1 << adapter->udma_num);

	if (!netif_running(dev))
		return 0;

	return err;
}

#ifdef HAVE_SET_RX_MODE
/**
 *  Unicast, Multicast and Promiscuous mode set
 *  @netdev: network interface device structure
 *
 *  The set_rx_mode entry point is called whenever the unicast or multicast
 *  address lists or the network interface flags are updated.  This routine is
 *  responsible for configuring the hardware for proper unicast, multicast,
 *  promiscuous mode, and all-multi behavior.
 **/
static void al_eth_set_rx_mode(struct net_device *netdev)
{
	struct al_eth_adapter *adapter = netdev_priv(netdev);

	if (netdev->flags & IFF_PROMISC) {
		al_eth_mac_table_promiscuous_set(adapter, true, 1 << adapter->udma_num);
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			al_eth_mac_table_all_multicast_add(adapter,
					AL_ETH_MAC_TABLE_ALL_MULTICAST_IDX, 1 << adapter->udma_num);
		} else {
			if (netdev_mc_empty(netdev))
				al_eth_mac_table_entry_clear(adapter,
					AL_ETH_MAC_TABLE_ALL_MULTICAST_IDX);
			else
				al_eth_mac_table_all_multicast_add(adapter,
					AL_ETH_MAC_TABLE_ALL_MULTICAST_IDX, 1 << adapter->udma_num);
		}

		if (!netdev_uc_empty(netdev)) {
			struct netdev_hw_addr *ha;
			uint8_t i = AL_ETH_MAC_TABLE_UNICAST_IDX_BASE + 1;

			if (netdev_uc_count(netdev) >
				AL_ETH_MAC_TABLE_UNICAST_MAX_COUNT) {
				/* In this case there are more addresses then
				 * entries in the mac table - set promiscuous */
				al_eth_mac_table_promiscuous_set(adapter, true, 1 << adapter->udma_num);
				return;
			}

			/* clear the last configuration */
			while (i < (AL_ETH_MAC_TABLE_UNICAST_IDX_BASE + 1 +
				    AL_ETH_MAC_TABLE_UNICAST_MAX_COUNT)) {
				al_eth_mac_table_entry_clear(adapter, i);
				i++;
			}

			/* set new addresses */
			i = AL_ETH_MAC_TABLE_UNICAST_IDX_BASE + 1;
			netdev_for_each_uc_addr(ha, netdev) {
				al_eth_mac_table_unicast_add(adapter, i,
							     ha->addr, 1 << adapter->udma_num);
				i++;
			}
		}

		al_eth_mac_table_promiscuous_set(adapter, false, 1 << adapter->udma_num);
	}
}
#else /* HAVE_SET_RX_MODE */
static void al_eth_set_rx_mode(struct net_device *netdev)
{
}
#endif /* HAVE_SET_RX_MODE */

/** HAVE_NET_DEVICE_OPS is defined above KERNEL_VERSION(2,6,29) */
/** CONFIG_ARCH_ALPINE assumes that we are on a kernel above that! */
#ifdef HAVE_NET_DEVICE_OPS
static const struct net_device_ops al_eth_netdev_ops = {
	.ndo_open		= al_eth_open,
	.ndo_stop		= al_eth_close,
	.ndo_start_xmit		= al_eth_start_xmit,
	.ndo_select_queue	= al_eth_select_queue,
#ifdef HAVE_NDO_GET_STATS64
	.ndo_get_stats64	= al_eth_get_stats64,
#else
	.ndo_get_stats		= al_eth_get_stats,
#endif
	.ndo_do_ioctl		= al_eth_ioctl,
	.ndo_tx_timeout		= al_eth_tx_timeout,
	.ndo_change_mtu		= al_eth_change_mtu,
	.ndo_set_mac_address	= al_eth_set_mac_addr,
#ifdef HAVE_SET_RX_MODE
	.ndo_set_rx_mode	= al_eth_set_rx_mode,
#endif
#if 0
	.ndo_validate_addr	= eth_validate_addr,

#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= al_eth_netpoll,
#endif
#endif
#ifdef CONFIG_RFS_ACCEL
	.ndo_rx_flow_steer      = al_eth_flow_steer,
#endif
	.ndo_set_features       = al_set_features,
};
#endif /* HAVE_NET_DEVICE_OPS */

/**
 * al_eth_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in al_eth_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * al_eth_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int
al_eth_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	static int version_printed;
	struct net_device *netdev;
	struct al_eth_adapter *adapter;
	void __iomem * const *iomap;
	struct al_hal_eth_adapter *hal_adapter;
	static int adapters_found;
	u16 dev_id;
	u8 rev_id;
	int i;

	int rc;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (version_printed++ == 0)
		dev_info(&pdev->dev, "%s", version);

	rc = pcim_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "pcim_enable_device failed!\n");
		return rc;
	}
	dev_info(&pdev->dev, "driver_data is 0x%lx", ent->driver_data);

	if (ent->driver_data == ALPINE_INTEGRATED)
		rc = pcim_iomap_regions(pdev, (1 << 0) | (1 << 2) | (1 << 4), DRV_MODULE_NAME);
	else {
		int bar;
#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
		if (!pdev->is_physfn)
			bar = 0;
		else
			bar = board_info[ent->driver_data].bar;
		dev_dbg(&pdev->dev, "bar is %d phys dev is %d\n", bar, pdev->is_physfn);
#else
		bar = board_info[ent->driver_data].bar;
		dev_dbg(&pdev->dev, "bar is %d\n", bar);
#endif
		rc = pcim_iomap_regions(pdev, (1 << bar), DRV_MODULE_NAME);
	}
	if (rc) {
		dev_err(&pdev->dev,
			"pci_request_selected_regions failed 0x%x\n", rc);
		return rc;
	}

	iomap = pcim_iomap_table(pdev);
	if (!iomap) {
		dev_err(&pdev->dev, "pcim_iomap_table failed\n");
		return -ENOMEM;
	}

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(40));
	if (rc) {
		dev_err(&pdev->dev, "pci_set_dma_mask failed 0x%x\n", rc);
		return rc;
	}

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(40));
	if (rc) {
		dev_err(&pdev->dev,
			"err_pci_set_consistent_dma_mask failed 0x%x\n", rc);
		return rc;
	}

	pci_set_master(pdev);
	pci_save_state(pdev);

	/* dev zeroed in init_etherdev */
	netdev = alloc_etherdev_mq(sizeof(struct al_eth_adapter), AL_ETH_NUM_QUEUES);
	if (!netdev) {
		dev_err(&pdev->dev, "alloc_etherdev_mq failed\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	adapter = netdev_priv(netdev);
	pci_set_drvdata(pdev, adapter);

	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->board_type = ent->driver_data;
	hal_adapter = &adapter->hal_adapter;
	adapter->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

	if (adapter->board_type == ALPINE_INTEGRATED) {
		/** For integrated networking always use udma 0 in host & kernel driver */
		adapter->udma_num = 0;
		adapter->udma_base = iomap[AL_ETH_UDMA_BAR];
		adapter->ec_base = iomap[AL_ETH_EC_BAR];
		adapter->mac_base = iomap[AL_ETH_MAC_BAR];

		pci_read_config_word(pdev, PCI_DEVICE_ID, &dev_id);
		pci_read_config_byte(pdev, PCI_REVISION_ID, &rev_id);
	} else {
		u16 adapter_pci_cmd;

		int bar = board_info[ent->driver_data].bar;
#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
		if (!pdev->is_physfn)
			bar = 0;
#endif

		if (adapter->board_type == ALPINE_NIC_V2_25_DUAL) {
			if (PCI_FUNC(pdev->devfn) == 1)
				port_num = 2;
			else
				port_num = 0;
			netdev_info(adapter->netdev, "eth port %d\n", port_num);
		}

		adapter->udma_num = udma_num;
		/*
		 * pci adapter configuration space: 0-4K
		 * BAR0-ETH_CTL: 20K-36K (start offset 0x5000)
		 * BAR1-MAC_CTL: 36K-40K (start offset 0x9000)
		 * BAR2-UDMA: 128K-256K
		 */
		adapter->internal_pcie_base = iomap[bar] + port_num * 0x100000;
		dev_id = readw(adapter->internal_pcie_base + PCI_DEVICE_ID);
		rev_id = readb(adapter->internal_pcie_base + PCI_REVISION_ID);

		netdev_info(adapter->netdev, "eth rev_id %d\n", rev_id);

		/** In general, offsets should come from HAL functions! take care of that */
		adapter->udma_base =
			iomap[bar] + port_num * 0x100000 + (128 * 0x400)*(1 + udma_num);
		adapter->ec_base = iomap[bar] + port_num * 0x100000 + 20 * 0x400;
		adapter->serdes_base = iomap[bar] + 0x18c0000;

		if (rev_id < AL_ETH_REV_ID_3)
			adapter->mac_base = iomap[bar] + port_num * 0x100000 + 36 * 0x400;
		else
			adapter->mac_base = iomap[bar] + port_num * 0x100000 + 96 * 0x400;

		netdev_info(adapter->netdev,
			"Mapped addresses - port base %p udma %p ex %p mac %p\n",
			adapter->internal_pcie_base, adapter->udma_base, adapter->ec_base,
			adapter->mac_base);

		/* enable sriov*/
		if (adapter->udma_num > 0) {
			/* enable all 3 VF's */
			writel(3, adapter->internal_pcie_base + 0x300 + PCI_SRIOV_NUM_VF);
			writel(PCI_SRIOV_CTRL_VFE | PCI_SRIOV_CTRL_MSE,
				adapter->internal_pcie_base + 0x300 + PCI_SRIOV_CTRL);
			/* enable master/slave in the adapter conf */
			adapter_pci_cmd =
				readw(adapter->internal_pcie_base + 0x1000 * (adapter->udma_num) +\
				PCI_COMMAND);
			adapter_pci_cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
			writew(adapter_pci_cmd,
				adapter->internal_pcie_base + 0x1000 * (adapter->udma_num) +\
				PCI_COMMAND);
		} else {
			/* enable master/slave in the adapter conf */
			adapter_pci_cmd = readw(adapter->internal_pcie_base + PCI_COMMAND);
			adapter_pci_cmd |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
			adapter_pci_cmd &= ~PCI_COMMAND_INTX_DISABLE;
			writew(adapter_pci_cmd, adapter->internal_pcie_base + PCI_COMMAND);
		}
	}

	adapter->rev_id = rev_id;
	adapter->dev_id = dev_id;
	adapter->id_number = adapters_found;

	/* set default ring sizes */
	adapter->tx_ring_count = AL_ETH_DEFAULT_TX_SW_DESCS;
	adapter->tx_descs_count = AL_ETH_DEFAULT_TX_HW_DESCS;
	adapter->rx_ring_count = AL_ETH_DEFAULT_RX_DESCS;
	adapter->rx_descs_count = AL_ETH_DEFAULT_RX_DESCS;

	adapter->num_tx_queues = AL_ETH_NUM_QUEUES;
	adapter->num_rx_queues = AL_ETH_NUM_QUEUES;

	adapter->small_copy_len = AL_ETH_DEFAULT_SMALL_PACKET_LEN;
	adapter->link_poll_interval = AL_ETH_DEFAULT_LINK_POLL_INTERVAL;
	adapter->max_rx_buff_alloc_size = AL_ETH_DEFAULT_MAX_RX_BUFF_ALLOC_SIZE;
	adapter->link_config.force_1000_base_x = AL_ETH_DEFAULT_FORCE_1000_BASEX;

	spin_lock_init(&adapter->serdes_config_lock);

	snprintf(adapter->name, AL_ETH_NAME_MAX_LEN, "al_eth_%d", adapter->id_number);

#ifndef CONFIG_ARCH_ALPINE
	/** Init serdes handle if board_params need to determine speed on the fly */
	switch (adapter->board_type) {
	case ALPINE_NIC:
	case ALPINE_NIC_V2_10:
		adapter->serdes_obj = kzalloc(sizeof(struct al_serdes_grp_obj), GFP_KERNEL);
		al_serdes_hssp_handle_init(adapter->serdes_base, adapter->serdes_obj);
		break;
	case ALPINE_NIC_V2_25:
	case ALPINE_NIC_V2_25_DUAL:
		adapter->serdes_obj = kzalloc(sizeof(struct al_serdes_grp_obj), GFP_KERNEL);
		al_serdes_25g_handle_init(adapter->serdes_base + AL_ETH_SERDES_25G_OFFSET,
			adapter->serdes_obj);
		break;
	default:
		break;
	}
#endif
	rc = al_eth_board_params_init(adapter);
	if (rc)
		goto err_hw_init;

	/** Perform HW stop to clean garbage left-overs from PXE / Uboot driver */
	al_eth_hal_adapter_init(adapter);
	adapter->hal_adapter.mac_mode = adapter->mac_mode;
	al_eth_hw_stop(adapter);

#if defined(CONFIG_ARCH_ALPINE)
	adapter->group_lm_context = alpine_group_lm_get(adapter->serdes_grp);
#endif

	al_eth_function_reset(adapter);

	rc = al_eth_hw_init_adapter(adapter);
	if (rc)
		goto err_hw_init;

	al_eth_init_rings(adapter);
	INIT_WORK(&adapter->reset_task, al_eth_reset_task);

#ifdef HAVE_NET_DEVICE_OPS
	netdev->netdev_ops = &al_eth_netdev_ops;
#else
	netdev->open = &al_eth_open;
	netdev->stop = &al_eth_close;
	netdev->hard_start_xmit = &al_eth_start_xmit;
	netdev->get_stats = &al_eth_get_stats;
#ifdef HAVE_SET_RX_MODE
	netdev->set_rx_mode = &al_eth_set_rx_mode;
#endif
	netdev->do_ioctl = &al_eth_ioctl;
	netdev->set_mac_address = &al_eth_set_mac_addr;
	netdev->change_mtu = &al_eth_change_mtu;
	netdev->tx_timeout = &al_eth_tx_timeout;
#ifdef HAVE_NETDEV_SELECT_QUEUE
	netdev->select_queue = &al_eth_select_queue;
#endif
#endif /* HAVE_NET_DEVICE_OPS */

	netdev->watchdog_timeo = TX_TIMEOUT;
	netdev->ethtool_ops = &al_eth_ethtool_ops;
	netdev->rtl83xx_ops = &al_eth_rtl83xx_ops;

	if (!is_valid_ether_addr(adapter->mac_addr)) {
		eth_hw_addr_random(netdev);
		memcpy(adapter->mac_addr, netdev->dev_addr, ETH_ALEN);
	} else {
		memcpy(netdev->dev_addr, adapter->mac_addr, ETH_ALEN);
	}

	memcpy(adapter->netdev->perm_addr, adapter->mac_addr, netdev->addr_len);

	netdev->features =	NETIF_F_SG |
				NETIF_F_IP_CSUM |
#if defined(NETIF_F_IPV6_CSUM) || defined(CONFIG_ARCH_ALPINE)
				NETIF_F_IPV6_CSUM |
#endif
				NETIF_F_TSO |
				NETIF_F_TSO_ECN |
				NETIF_F_TSO6 |
				NETIF_F_RXCSUM |
				NETIF_F_NTUPLE |
#if defined(NETIF_F_RXHASH) || defined(CONFIG_ARCH_ALPINE)
				NETIF_F_RXHASH |
#endif
#ifdef CONFIG_NET_MQ_TX_LOCK_OPT
				NETIF_F_MQ_TX_LOCK_OPT |
#endif
				NETIF_F_HIGHDMA;

/** HAVE_NDO_SET_FEATURES defined for kernels above KERNEL_VERSION(2,6,39) */
#ifdef HAVE_NDO_SET_FEATURES
	netdev->hw_features |= netdev->features;
#endif

	netdev->vlan_features |= netdev->features;

/* set this after vlan_features since it cannot be part of vlan_features */
#if defined(CONFIG_AL_ETH_ALLOC_PAGE) || defined(CONFIG_AL_ETH_ALLOC_FRAG)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	netdev->features |= NETIF_F_HW_VLAN_CTAG_RX;
#else
	netdev->features |=  NETIF_F_HW_VLAN_RX;
#endif
#endif

#if defined(NETIF_F_MQ_TX_LOCK_OPT)
	netdev->features &= ~NETIF_F_MQ_TX_LOCK_OPT;
#endif
#if defined(IFF_UNICAST_FLT) || defined(CONFIG_ARCH_ALPINE)
	netdev->priv_flags |= IFF_UNICAST_FLT;
#endif

	for (i = 0; i < AL_ETH_RX_RSS_TABLE_SIZE; i++)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		adapter->rss_ind_tbl[i] =
			ethtool_rxfh_indir_default(i, AL_ETH_NUM_QUEUES);
#else
		adapter->rss_ind_tbl[i] = i % AL_ETH_NUM_QUEUES;
#endif

	u64_stats_init(&adapter->syncp);

	rc = register_netdev(netdev);
	if (rc) {
		dev_err(&pdev->dev, "Cannot register net device\n");
		goto err_register;
	}

	rc = al_eth_sysfs_init(&adapter->pdev->dev);
	if (rc)
		return rc;

	adapter->adaptive_intr_rate = adaptive_int_moderation;
	al_eth_init_intr_default_moderation_table_intervals(adapter);
	/* assign initial_moderation_table the accurate interval values,according to SB frequency */
	al_eth_initial_moderation_table_restore_default();

	netdev_info(netdev, "%s found at mem %lx, mac addr %pM\n",
		board_info[ent->driver_data].name,
		(long)pci_resource_start(pdev, 0), netdev->dev_addr);

adapters_found++;
#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
	if (pdev->is_physfn) {
		int err = 0;
		dev_info(&pdev->dev, "disable sriov");
		pci_disable_sriov(pdev);

		dev_info(&pdev->dev, "enable sriov, vfs: %d", 1);
		err = pci_enable_sriov(pdev, 1);
		if (err)
			dev_info(&pdev->dev, "pci_enable_sriov() failed. err=%d\n", err);
	}
#endif
	return 0;
err_register:
err_hw_init:
	free_netdev(netdev);
	return rc;
}

/**
 * al_eth_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * al_eth_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 **/
static void
al_eth_remove(struct pci_dev *pdev)
{
	struct al_eth_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *dev = adapter->netdev;

	al_eth_hw_stop(adapter);

	unregister_netdev(dev);

	al_eth_sysfs_terminate(&pdev->dev);
#ifndef CONFIG_ARCH_ALPINE
	if (IS_NIC(adapter->board_type))
		kzfree(adapter->serdes_obj);
#endif
	free_netdev(dev);

#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
	if (pdev->is_physfn) {
		dev_info(&pdev->dev, "disable sriov");
		pci_disable_sriov(pdev);
	}
#endif

	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM
static int al_eth_resume(struct pci_dev *pdev)
{
	struct al_eth_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	/*
	 * pci_restore_state clears dev->state_saved so call
	 * pci_save_state to restore it.
	 */
	pci_save_state(pdev);

	err = pci_enable_device_mem(pdev);
	if (err) {
		pr_err("Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_wake_from_d3(pdev, false);

#if defined(CONFIG_AL_ETH_SRIOV) && !defined(CONFIG_ARCH_ALPINE)
	if (pdev->is_physfn) {
	   int err = 0;
	   dev_info(&pdev->dev, "disable sriov");
	   pci_disable_sriov(pdev);

	   dev_info(&pdev->dev, "enable sriov, vfs: %d", 1);
	   err = pci_enable_sriov(pdev, 1);
	   if (err) {
		   dev_info(&pdev->dev, "pci_enable_sriov() failed. err=%d\n", err);
	   }
	}
#endif

	al_eth_wol_disable(&adapter->hal_adapter);

	netif_device_attach(netdev);

	return 0;
}

static int al_eth_wol_config(struct al_eth_adapter *adapter)
{
	struct al_eth_wol_params wol = {0};

	if (adapter->wol & WAKE_UCAST) {
		wol.int_mask = AL_ETH_WOL_INT_UNICAST;
		wol.forward_mask = AL_ETH_WOL_FWRD_UNICAST;
	}

	if (adapter->wol & WAKE_MCAST) {
		wol.int_mask = AL_ETH_WOL_INT_MULTICAST;
		wol.forward_mask = AL_ETH_WOL_FWRD_MULTICAST;
	}

	if (adapter->wol & WAKE_BCAST) {
		wol.int_mask = AL_ETH_WOL_INT_BROADCAST;
		wol.forward_mask = AL_ETH_WOL_FWRD_BROADCAST;
	}

	if (wol.int_mask != 0) {
		al_eth_wol_enable(&adapter->hal_adapter, &wol);
		return 1;
	}

	return 0;
}

static int al_eth_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct al_eth_adapter *adapter = pci_get_drvdata(pdev);

	if (al_eth_wol_config(adapter)) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}
#endif /* CONFIG_PM */


static struct pci_driver al_eth_pci_driver = {
	.name		= DRV_MODULE_NAME,
	.id_table	= al_eth_pci_tbl,
	.probe		= al_eth_probe,
	.remove		= al_eth_remove,
	.shutdown	= al_eth_remove,
#ifdef CONFIG_PM
	.suspend	= al_eth_suspend,
	.resume		= al_eth_resume,
#endif
};

static int __init al_eth_init(void)
{
#ifdef CONFIG_AL_ETH_ALLOC_SKB
	struct sk_buff_head *rx_rc;
	int cpu;

	for_each_possible_cpu(cpu) {
		rx_rc =  &per_cpu(rx_recycle_cache, cpu);
		skb_queue_head_init(rx_rc);
	}
#endif
	return pci_register_driver(&al_eth_pci_driver);
}

static void __exit al_eth_cleanup(void)
{
#ifdef CONFIG_AL_ETH_ALLOC_SKB
	struct sk_buff_head *rx_rc;
	int cpu;

	for_each_possible_cpu(cpu) {
		rx_rc =  &per_cpu(rx_recycle_cache, cpu);
		skb_queue_purge(rx_rc);
	}
#endif
	pci_unregister_driver(&al_eth_pci_driver);
}

module_init(al_eth_init);
module_exit(al_eth_cleanup);
