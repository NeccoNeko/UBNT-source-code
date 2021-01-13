 /*
   * drivers/net/al_eth.c
   *
   * Thie file contains Annapurna Labs Ethernet driver for U-Boot
   *
   * Copyright (C) 2012 Annapurna Labs Ltd.
   *
   * This program is free software; you can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation; either version 2 of the License, or
   * (at your option) any later version.
   *
   * This program is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with this program; if not, write to the Free Software
   * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   */

/*#define DEBUG*/

/** TODO:
 * - Proper error handling
 * - Non RGMII support
 * - 10G support
 * - Shutdown support
 * - Packet size limit cleanup
 */

#include <common.h>
#include <net.h>
#include <phy.h>
#include <miiphy.h>
#include <malloc.h>
#include <pci.h>

#include <al_globals.h>
#include <eeprom_per_device.h>
#include <asm/arch/timer.h>
#include <i2c.h>
#include <asm/gpio.h>

#include "al_eth.h"
#include "al_hal_eth.h"
#include "al_init_eth_kr.h"
#include "al_init_eth_lm.h"
#include "al_hal_nb_regs.h"
#include "al_hal_iomap.h"

#ifdef DEBUG
#define DEBUG_PRINT_RX_BUFF	0
#define DEBUG_PRINT_TX_BUFF	0
#else
#define DEBUG_PRINT_RX_BUFF	0
#define DEBUG_PRINT_TX_BUFF	0
#endif

#define AL_ETH_MAX_NUM_INTERFACES	4

#define PACKET_SIZE		PKTSIZE_ALIGN
#define NUM_RX_BUFFERS		PKTBUFSRX

#define RX_BUFF(i)		\
	net_rx_packets[i]

#define DESCRIPTORS_PER_QUEUE	(NUM_RX_BUFFERS + 1)
#define COMPLETION_DESC_SIZE	16

#define QUEUE_DESCS_SIZE	(DESCRIPTORS_PER_QUEUE * COMPLETION_DESC_SIZE)

#define TX_SDESC_OFFSET		(0 * QUEUE_DESCS_SIZE)
#define TX_CDESC_OFFSET		(1 * QUEUE_DESCS_SIZE)
#define RX_SDESC_OFFSET		(2 * QUEUE_DESCS_SIZE)
#define RX_CDESC_OFFSET		(3 * QUEUE_DESCS_SIZE)
#define QUEUE_DESC_OFFSET	(4 * QUEUE_DESCS_SIZE)

#define LT_ITERS_NUM		5

#define SFP_I2C_ADDR		0x50

/* CAUTION: These following DEFINES shouldn't be made smaller, without
 * checking the impact on link establishment in 25g-10g-auto-detect mode
 */
#define LINK_MANAGEMENT_NUM_OF_ITERS		30
#define LINK_MANAGEMENT_DELAY_BETWEEN_ITERS	100 /* delay is in msec */

#define LINK_MANAGEMENT_CONSECUTIVE_SUCCESS	5

/* Time in mSec to switch auto-FEC on init */
#define AUTO_FEC_INITIAL_TIMEOUT		1000

/* Time in mSec to switch auto-FEC on toggle */
#define AUTO_FEC_TOGGLE_TIMEOUT			500

/* Number of times to poll the TX completion queue. Each time ~= 1 uSec */
#define TX_DONE_POLL_RETRIES			500000

struct al_eth_link_config {
	al_bool		autoneg;
	al_bool		sgmii;
	uint32_t	speed;
	al_bool		full_duplex;
};

struct al_eth_priv {
	int				is_initialized;
	int				phy_is_connected;
	int				phy_startup_skip;
	int				bus_index;
	void __iomem			*udma_regs_base;
	void __iomem			*ec_regs_base;
	void __iomem			*mac_regs_base;
	struct al_hal_eth_adapter	eth_adapter;
	struct al_udma_q *tx_dma_q; /* udma tx queue handler */
	struct al_udma_q *rx_dma_q; /* udma rx queue handler */
	uintptr_t			desc_base;
	unsigned int			rx_buf_tail_idx;
	enum al_eth_mac_mode		mac_mode;
	phy_interface_t			phy_if;
	al_bool				ext_phy_exist;
	enum al_eth_board_ext_phy_if	ext_phy_if;
	unsigned int			phy_addr;
	enum al_eth_ref_clk_freq	ref_clk_freq;
	unsigned int			mdio_freq;
	struct mii_dev			*phy_bus;
	struct phy_device		*phy_dev;
	pci_dev_t			pci_devno;
	uint8_t				rev_id;
	uint8_t				serdes_grp;
	uint8_t				serdes_lane;
	al_bool				sfp_detect_needed;
	al_bool				auto_speed_needed;
	al_bool				link_training_needed;
	struct al_eth_link_config	link_config;
	al_bool				dont_override_serdes;
	struct al_eth_lm_context	lm_context;
	al_bool				use_lm;
	al_bool				lm_debug;
	al_bool				dac;
	uint8_t				dac_len;
	al_bool				retimer_exist;
	al_bool				retimer_type;
	uint8_t				retimer_bus_id;
	uint8_t				retimer_i2c_addr;
	enum al_eth_retimer_channel	retimer_channel;
	enum al_eth_retimer_channel	retimer_tx_channel;
	enum al_eth_lm_max_speed	max_speed;
	al_bool				speed_detection;
	al_bool				auto_fec_enable;
	unsigned int			gpio_spd_1g;
	unsigned int			gpio_spd_10g;
	unsigned int			gpio_spd_25g;
};

static struct al_eth_priv *ifs[AL_ETH_MAX_NUM_INTERFACES];

static int al_eth_dev_init(
	struct eth_device	*dev);

static int al_eth_init(
	struct eth_device *dev,
	bd_t *bis);

static int al_eth_send(
	struct eth_device	*dev,
	void			*packet,
	int			length);

static int al_eth_recv(
	struct eth_device *dev);

static void al_eth_halt(
	struct eth_device *dev);

static int al_eth_write_hwaddr(
	struct eth_device *dev);

#ifdef CONFIG_MII
static int al_eth_phy_mdio_register(
	struct eth_device	*eth_dev,
	struct mii_dev		**bus);

static int al_eth_phy_mdio_read(
	struct mii_dev *bus,
	int phy_addr,
	int dev_addr,
	int regnum);

static int al_eth_phy_mdio_write(
	struct mii_dev *bus,
	int phy_addr,
	int dev_addr,
	int regnum,
	u16 value);
#endif

static int al_eth_flr_read_config_u32(
	void *handle,
	int where,
	uint32_t *val);

static int al_eth_flr_write_config_u32(
	void *handle,
	int where,
	uint32_t val);

static void al_eth_random_enetaddr(
	uchar *enetaddr);

uint32_t sys_counter_cnt_get_low(void)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)AL_NB_SERVICE_BASE;

	return readl(&nb_regs->system_counter.cnt_low);
}

int al_eth_register(
	int		index,
	pci_dev_t	pci_devno,
	uint8_t		rev_id,
	void __iomem	*udma_regs_base,
	void __iomem	*ec_regs_base,
	void __iomem	*mac_regs_base)
{
	int err;
	struct al_eth_priv *priv;
	struct eth_device *dev;
	int eeprom_per_device_mac_addr_dirty;
	unsigned int board_i2c_bus_id_convert(unsigned int id);

	debug(
		"%s(%p, %p, %p)\n",
		__func__,
		udma_regs_base,
		ec_regs_base,
		mac_regs_base);

	priv = malloc(sizeof(*priv));
	if (priv == NULL)
		return -ENOMEM;

	dev = malloc(sizeof(*dev));
	if (dev == NULL) {
		free(priv);
		return -ENOMEM;
	}

	/* setup whatever private state you need */
	memset(priv, 0, sizeof(*priv));

	priv->pci_devno = pci_devno;

	priv->desc_base = (uintptr_t)memalign(16, QUEUE_DESC_OFFSET);
	if (!priv->desc_base) {
		printf("%s: desc_base memalign failed!\n", __func__);
		return -ENOMEM;
	}

	debug("%s: desc_base = %p\n", __func__, (void *)priv->desc_base);

	memset(dev, 0, sizeof(*dev));
	sprintf(dev->name, "al_eth%d", index);

	/* Set MAC address */
	eeprom_per_device_mac_addr_get(index, dev->enetaddr, &eeprom_per_device_mac_addr_dirty);
	if (is_valid_ethaddr(dev->enetaddr))
		eth_setenv_enetaddr_by_index("eth", index, dev->enetaddr);
	else if (!eth_getenv_enetaddr_by_index("eth", index, dev->enetaddr))
		al_eth_random_enetaddr(dev->enetaddr);

	dev->iobase = 0;
	dev->priv = priv;
	dev->init = al_eth_init;
	dev->halt = al_eth_halt;
	dev->send = al_eth_send;
	dev->recv = al_eth_recv;
	dev->write_hwaddr = al_eth_write_hwaddr;

	priv->rev_id = rev_id;
	priv->udma_regs_base = udma_regs_base;
	priv->ec_regs_base = ec_regs_base;
	priv->mac_regs_base = mac_regs_base;
	priv->phy_if = PHY_INTERFACE_MODE_RGMII;

	err = eth_register(dev);
	if (err) {
		printf("%s: eth_register failed!\n", __func__);
		return err;
	}

	priv->ext_phy_exist = al_globals.eth_board_params[index].phy_exist;
	priv->ext_phy_if = al_globals.eth_board_params[index].phy_if;
	priv->phy_addr = al_globals.eth_board_params[index].phy_mdio_addr;
	priv->ref_clk_freq = al_globals.eth_board_params[index].ref_clk_freq;
	priv->retimer_exist = al_globals.eth_board_params[index].retimer_exist;
	priv->retimer_type = al_globals.eth_board_params[index].retimer_type;
	priv->retimer_bus_id =
		board_i2c_bus_id_convert(al_globals.eth_board_params[index].retimer_bus_id);
	priv->retimer_i2c_addr = al_globals.eth_board_params[index].retimer_i2c_addr;
	priv->retimer_channel = al_globals.eth_board_params[index].retimer_channel;
	priv->retimer_tx_channel = al_globals.eth_board_params[index].retimer_tx_channel;
	priv->dac = al_globals.eth_board_params[index].dac;
	priv->dac_len = al_globals.eth_board_params[index].dac_len;
	priv->link_config.sgmii = !al_globals.eth_board_params[index].force_1000_base_x;
	priv->link_config.autoneg = !al_globals.eth_board_params[index].an_disable;
	priv->auto_fec_enable = al_globals.eth_board_params[index].auto_fec_enable;
	priv->gpio_spd_1g = al_globals.eth_board_params[index].gpio_spd_1g;
	priv->gpio_spd_10g = al_globals.eth_board_params[index].gpio_spd_10g;
	priv->gpio_spd_25g = al_globals.eth_board_params[index].gpio_spd_25g;

	switch (al_globals.eth_board_params[index].speed) {
		default:
			printf("%s: invalid speed (%d)\n", __func__,
			       al_globals.eth_board_params[index].speed);
		case AL_ETH_BOARD_1G_SPEED_1000M:
			priv->link_config.speed = 1000;
			break;
		case AL_ETH_BOARD_1G_SPEED_100M:
			priv->link_config.speed = 100;
			break;
		case AL_ETH_BOARD_1G_SPEED_10M:
			priv->link_config.speed = 10;
			break;
	}

	priv->link_config.full_duplex = !al_globals.eth_board_params[index].half_duplex;

	switch (al_globals.eth_board_params[index].mdio_freq) {
	default:
		printf("%s: invalid mdio freq (%d)\n",	__func__,
			al_globals.eth_board_params[index].mdio_freq);
	case AL_ETH_BOARD_MDIO_FREQ_2_5_MHZ:
		priv->mdio_freq = 2500;
		break;
	case AL_ETH_BOARD_MDIO_FREQ_1_MHZ:
		priv->mdio_freq = 1000;
		break;
	}

	priv->bus_index    =
		board_i2c_bus_id_convert(al_globals.eth_board_params[index].i2c_adapter_id);
	priv->serdes_grp   = al_globals.eth_board_params[index].serdes_grp;
	priv->serdes_lane  = al_globals.eth_board_params[index].serdes_lane;
	priv->dont_override_serdes =
		al_globals.eth_board_params[index].dont_override_serdes;
	priv->sfp_detect_needed    = AL_FALSE;
	priv->link_training_needed =
		al_globals.eth_board_params[index].kr_lt_enable &&
		al_globals.eth_board_params[index].autoneg_enable;

	if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_SGMII) {
		priv->mac_mode = AL_ETH_MAC_MODE_SGMII;
		priv->use_lm = AL_FALSE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_1G;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_RGMII) {
		/* backward compatibility */
		if (al_globals.eth_board_params[index].sfp_plus_module_exist)
			priv->mac_mode = AL_ETH_MAC_MODE_SGMII;
		else
			priv->mac_mode = AL_ETH_MAC_MODE_RGMII;
		priv->use_lm = AL_FALSE;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_SGMII_2_5G) {
		priv->mac_mode = AL_ETH_MAC_MODE_SGMII_2_5G;
		priv->use_lm = AL_FALSE;
		priv->link_config.autoneg = AL_FALSE;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_10GBASE_SR) {
		priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_10G;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT) {
		priv->sfp_detect_needed = AL_TRUE;
		priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_10G;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT_AUTO_SPEED) {
		priv->sfp_detect_needed = AL_TRUE;
		priv->auto_speed_needed = AL_TRUE;
		priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_10G;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_NBASE_T) {
		priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_25G) {
		priv->sfp_detect_needed = AL_TRUE;
		priv->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_25G;
	} else if (al_globals.eth_board_params[index].media_type ==
			AL_ETH_BOARD_MEDIA_TYPE_25G_10G_AUTO) {
		priv->sfp_detect_needed = AL_TRUE;
		priv->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_25G;
		priv->speed_detection = AL_TRUE;
	} else if (al_globals.eth_board_params[index].media_type ==
		AL_ETH_BOARD_MEDIA_TYPE_40G) {
		priv->mac_mode = AL_ETH_MAC_MODE_XLG_LL_40G;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_MAX;
	} else {
		printf("%s: Non supported media type (%d)!",
			__func__,
			al_globals.eth_board_params[index].media_type);
		priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
		priv->use_lm = AL_TRUE;
		priv->max_speed = AL_ETH_LM_MAX_SPEED_10G;
	}

	err = al_eth_board_params_set(mac_regs_base,
			&al_globals.eth_board_params[index]);
	if (err) {
		printf("%s: al_eth_board_params_set failed!\n", __func__);
		return err;
	}

#ifdef CONFIG_MII
	err = al_eth_phy_mdio_register(dev, &priv->phy_bus);
	if (err) {
		printf("%s: al_eth_mdio_register failed!\n", __func__);
		return err;
	}
#endif

	ifs[index] = priv;

	return 0;
}

int al_eth_i2c_byte_read(void *context, uint8_t bus_id, uint8_t i2c_addr, uint8_t addr, uint8_t *val)
{
	/*struct al_eth_priv *priv = context;*/
	int ret;

	/* select the correct i2c module by muxing the i2c switch */
	ret = i2c_set_bus_num(bus_id);
	if (ret)
		return -EINVAL;

	/* read from the i2c module, chosen by the switch configuration */
	ret = i2c_read(i2c_addr, addr, 1, val, 1);
	if (ret) {
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		return -ETIMEDOUT;
	}

	return 0;
}

int al_eth_i2c_byte_write(void *context, uint8_t bus_id, uint8_t i2c_addr, uint8_t addr, uint8_t val)
{
	/*struct al_eth_priv *priv = context;*/
	int ret;

	/* select the correct i2c module by muxing the i2c switch */
	ret = i2c_set_bus_num(bus_id);
	if (ret)
		return -EINVAL;

	/* read from the i2c module, chosen by the switch configuration */
	ret = i2c_write(i2c_addr, addr, 1, &val, 1);
	if (ret) {
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		return -ETIMEDOUT;
	}

	return 0;
}

static uint8_t al_eth_get_rand_byte(void)
{
	uint8_t byte;
	static int first = 1;

	if (first) {
		srand(sys_counter_cnt_get_low());
		first = 0;
	}

	byte = rand();

	return byte;
}

static unsigned int al_eth_time_msec_get(void)
{
	return (unsigned int) get_timer(0);
}

static void al_eth_lm_led_config_init_single(unsigned int gpio, const char *name)
{
	int err;

	if (!gpio)
		return;

	err = gpio_request(gpio, name);
	if (err)
		printf("[%s] LED %s (%u): unable to request GPIO\n", __func__, name, gpio);
}

static void al_eth_lm_led_config_init(struct al_eth_priv *priv)
{
	al_eth_lm_led_config_init_single(
		priv->gpio_spd_1g, "1g");
	al_eth_lm_led_config_init_single(
		priv->gpio_spd_10g, "10g");
	al_eth_lm_led_config_init_single(
		priv->gpio_spd_25g, "25g");
}

static void al_eth_lm_led_config_terminate(struct al_eth_priv *priv)
{
	if (priv->gpio_spd_1g)
		gpio_free(priv->gpio_spd_1g);
	if (priv->gpio_spd_10g)
		gpio_free(priv->gpio_spd_10g);
	if (priv->gpio_spd_25g)
		gpio_free(priv->gpio_spd_25g);
}

static void al_eth_lm_led_config_single(unsigned int gpio, unsigned int val, const char *name)
{
	if (gpio) {
		gpio_set_value(gpio, val);
		debug("[%s] LED %s (%u) set to %u\n", __func__, name, gpio, val);
	}
}

static void al_eth_lm_led_config(void *context, struct al_eth_lm_led_config_data *data)
{
	struct al_eth_priv *priv = context;

	al_eth_lm_led_config_single(
		priv->gpio_spd_1g, (data->speed == AL_ETH_LM_LED_CONFIG_1G), "1g");
	al_eth_lm_led_config_single(
		priv->gpio_spd_10g, (data->speed == AL_ETH_LM_LED_CONFIG_10G), "10g");
	al_eth_lm_led_config_single(
		priv->gpio_spd_25g, (data->speed == AL_ETH_LM_LED_CONFIG_25G), "25g");
}

static void al_eth_lm_config(struct al_eth_priv *priv)
{
	struct al_eth_lm_init_params	params = {0};

	params.adapter = &priv->eth_adapter;
	params.serdes_obj = &al_globals.serdes[priv->serdes_grp];
	params.lane = priv->serdes_lane;
	params.sfp_detection = priv->sfp_detect_needed;
	params.rx_equal = true;

	switch (priv->max_speed) {
	case AL_ETH_LM_MAX_SPEED_10G:
		if (priv->link_training_needed || priv->dac) {
			params.default_mode = AL_ETH_LM_MODE_10G_DA;
			params.default_dac_len = priv->dac_len;
		} else {
			params.default_mode = AL_ETH_LM_MODE_10G_OPTIC;
		}
		break;
	case AL_ETH_LM_MAX_SPEED_MAX:
	case AL_ETH_LM_MAX_SPEED_25G:
		params.default_mode = AL_ETH_LM_MODE_25G;
		params.default_dac_len = priv->dac_len;
		params.sfp_detect_force_mode = true;
		params.rx_equal = false;
		break;
	case AL_ETH_LM_MAX_SPEED_1G:
		params.default_mode = AL_ETH_LM_MODE_1G;
		break;
	default:
		printf("mac mode not supported!\n");
		params.default_mode = AL_ETH_LM_MODE_10G_DA;
		params.default_dac_len = priv->dac_len;
	}

	if (params.sfp_detection) {
		params.sfp_bus_id = priv->bus_index;
		params.sfp_i2c_addr = SFP_I2C_ADDR;
	}

	params.link_training = priv->link_training_needed;
	params.static_values = !priv->dont_override_serdes;
	params.i2c_read = &al_eth_i2c_byte_read;
	params.i2c_write = &al_eth_i2c_byte_write;
	params.i2c_context = priv;
	params.get_random_byte = &al_eth_get_rand_byte;
	params.kr_fec_enable = false;

	params.retimer_exist = priv->retimer_exist;
	params.retimer_type = priv->retimer_type;
	params.retimer_bus_id = priv->retimer_bus_id;
	params.retimer_i2c_addr = priv->retimer_i2c_addr;
	params.retimer_channel = priv->retimer_channel;
	params.retimer_tx_channel = priv->retimer_tx_channel;
	params.speed_detection = priv->speed_detection;

	params.auto_fec_enable = priv->auto_fec_enable;
	params.auto_fec_initial_timeout = AUTO_FEC_INITIAL_TIMEOUT;
	params.auto_fec_toggle_timeout = AUTO_FEC_TOGGLE_TIMEOUT;
	params.get_msec = al_eth_time_msec_get;
	params.led_config = &al_eth_lm_led_config;

	al_eth_lm_led_config_init(priv);

	al_eth_lm_init(&priv->lm_context, &params);
}

static int al_eth_dev_init(struct eth_device *dev)
{
	int err = 0;
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;
	int i;
	enum al_eth_lm_link_mode	old_mode;
	enum al_eth_lm_link_mode	new_mode;

	struct al_eth_adapter_params eth_adapter_params = {
		.rev_id = priv->rev_id,
		.udma_id  = 0,
		.enable_rx_parser = 0,
		.udma_regs_base	= priv->udma_regs_base,
		.ec_regs_base = priv->ec_regs_base,
		.mac_regs_base = priv->mac_regs_base,
		.name = dev->name,
		.serdes_lane = priv->serdes_lane,
	};

	struct al_udma_q_params  eth_tx_params = {
			.size = DESCRIPTORS_PER_QUEUE,
			.desc_base = 0,
			.desc_phy_base = 0,
			.cdesc_base = 0,
			.cdesc_phy_base = 0,
			.cdesc_size = COMPLETION_DESC_SIZE
	};

	struct al_udma_q_params eth_rx_params = {
			.size = DESCRIPTORS_PER_QUEUE,
			.desc_base = 0,
			.desc_phy_base = 0,
			.cdesc_base = 0,
			.cdesc_phy_base = 0,
			.cdesc_size = COMPLETION_DESC_SIZE
	};

	debug("%s(%p)\n", __func__, dev);

	eth_tx_params.desc_base =
		(union al_udma_desc *)(priv->desc_base + TX_SDESC_OFFSET);
	eth_tx_params.desc_phy_base =  priv->desc_base + TX_SDESC_OFFSET;
	eth_tx_params.cdesc_base =
		(uint8_t *)(priv->desc_base + TX_CDESC_OFFSET);
	eth_tx_params.cdesc_phy_base =  priv->desc_base + TX_CDESC_OFFSET;

	memset(
		eth_tx_params.cdesc_base,
		0,
		(DESCRIPTORS_PER_QUEUE*COMPLETION_DESC_SIZE));

	eth_rx_params.desc_base =
		(union al_udma_desc *)(priv->desc_base + RX_SDESC_OFFSET);
	eth_rx_params.desc_phy_base = (priv->desc_base + RX_SDESC_OFFSET);
	eth_rx_params.cdesc_base =
		(uint8_t *)(priv->desc_base + RX_CDESC_OFFSET);
	eth_rx_params.cdesc_phy_base = (priv->desc_base + RX_CDESC_OFFSET);

	memset(
		eth_rx_params.cdesc_base,
		0,
		(DESCRIPTORS_PER_QUEUE*COMPLETION_DESC_SIZE));

	al_eth_adapter_init(&priv->eth_adapter, &eth_adapter_params);
	al_eth_queue_config(&priv->eth_adapter, UDMA_TX, 0, &eth_tx_params);
	al_eth_queue_enable(&priv->eth_adapter, UDMA_TX, 0);
	al_eth_queue_config(&priv->eth_adapter, UDMA_RX, 0, &eth_rx_params);
	al_eth_queue_enable(&priv->eth_adapter, UDMA_RX, 0);
	al_udma_q_handle_get(&priv->eth_adapter.tx_udma, 0, &priv->tx_dma_q);
	al_udma_q_handle_get(&priv->eth_adapter.rx_udma, 0, &priv->rx_dma_q);

	if ((!priv->ext_phy_exist) && (priv->use_lm)) {
		al_eth_lm_config(priv);

		al_eth_lm_debug_mode_set(&priv->lm_context, priv->lm_debug);

		err = al_eth_lm_link_detection(&priv->lm_context, NULL, &old_mode, &new_mode);

		if ((err) || (new_mode == AL_ETH_LM_MODE_DISCONNECTED)) {
			printf("%s: SFP doesn't exist or can't be accessed\n", __func__);
			return -ENETDOWN;
		}

		switch (new_mode) {
		case AL_ETH_LM_MODE_1G:
			priv->mac_mode = AL_ETH_MAC_MODE_SGMII;
			break;
		case AL_ETH_LM_MODE_25G:
			priv->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
			break;
		case AL_ETH_LM_MODE_10G_DA:
		case AL_ETH_LM_MODE_10G_OPTIC:
		default:
			if (al_globals.serdes[priv->serdes_grp].type_get() == AL_SRDS_TYPE_25G)
				priv->mac_mode = AL_ETH_MAC_MODE_KR_LL_25G;
			else
				priv->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
			break;
		}

		if (priv->auto_speed_needed) {
			struct al_serdes_adv_tx_params tx_params[AL_SRDS_NUM_LANES];
			struct al_serdes_adv_rx_params rx_params[AL_SRDS_NUM_LANES];

			for (i = 0; i < AL_SRDS_NUM_LANES; i++) {
				al_globals.serdes[priv->serdes_grp].tx_advanced_params_get(
						&al_globals.serdes[priv->serdes_grp],
						i,
						&tx_params[i]);
				al_globals.serdes[priv->serdes_grp].rx_advanced_params_get(
						&al_globals.serdes[priv->serdes_grp],
						i,
						&rx_params[i]);

			}

			if (priv->mac_mode == AL_ETH_MAC_MODE_SGMII)
				al_globals.serdes[priv->serdes_grp].mode_set_sgmii(
						&al_globals.serdes[priv->serdes_grp]);
			else
				al_globals.serdes[priv->serdes_grp].mode_set_kr(
						&al_globals.serdes[priv->serdes_grp]);

			for (i = 0; i < AL_SRDS_NUM_LANES; i++) {
				al_globals.serdes[priv->serdes_grp].tx_advanced_params_set(
						&al_globals.serdes[priv->serdes_grp],
						i,
						&tx_params[i]);
				al_globals.serdes[priv->serdes_grp].rx_advanced_params_set(
						&al_globals.serdes[priv->serdes_grp],
						i,
						&rx_params[i]);
			}
		}

		if (priv->speed_detection && (new_mode != old_mode)) {
			if (new_mode == AL_ETH_LM_MODE_25G) {
				al_info("%s: set SerDes to 25G\n", __func__);
				al_globals.serdes[priv->serdes_grp].mode_set_25g(
					&al_globals.serdes[priv->serdes_grp]);
			} else if ((new_mode == AL_ETH_LM_MODE_10G_DA) ||
					(new_mode == AL_ETH_LM_MODE_10G_OPTIC)) {
				al_info("%s: set SerDes to 10G\n", __func__);
				al_globals.serdes[priv->serdes_grp].mode_set_kr(
					&al_globals.serdes[priv->serdes_grp]);
			}
		}
	}

	al_eth_mac_config(&priv->eth_adapter, priv->mac_mode);

	if ((priv->mac_mode == AL_ETH_MAC_MODE_SGMII) ||
		(priv->mac_mode == AL_ETH_MAC_MODE_SGMII_2_5G) ||
	    ((priv->mac_mode == AL_ETH_MAC_MODE_RGMII) && (!priv->ext_phy_exist)))
		al_eth_mac_link_config(&priv->eth_adapter,
				       !priv->link_config.sgmii,
				       priv->link_config.autoneg,
				       priv->link_config.speed,
				       priv->link_config.full_duplex);

	al_eth_rx_pkt_limit_config(&priv->eth_adapter, 30, 1518);

	/* Init Rx */
	for (i = 0; i < NUM_RX_BUFFERS; i++) {
		struct al_buf buf = {
			.addr = (al_phys_addr_t)(uintptr_t)RX_BUFF(i),
			.len = PACKET_SIZE,
		};

		err = al_eth_rx_buffer_add(
			priv->rx_dma_q, &buf, AL_ETH_RX_FLAGS_INT, NULL);
		if (err) {
			printf("%s: al_eth_rx_buffer_add failed!\n", __func__);
			return err;
		}
	}

	priv->rx_buf_tail_idx = 0;

	al_eth_rx_buffer_action(priv->rx_dma_q, NUM_RX_BUFFERS);

	al_eth_mac_start(&priv->eth_adapter);

	if ((!priv->ext_phy_exist) && (priv->use_lm)) {
		al_bool				link_up;
		int i = 0;
		int consecutive_success = 0;

		for (i = 0; i < LINK_MANAGEMENT_NUM_OF_ITERS; i++) {
			/* the reason for the loop is beacuse in the case of 25g-10g-auto-detect,
			 * the link_state toggles sometimes between DOWN_RF and UP (due to time
			 * diffrences from when opp side performs link_management) therefore we
			 * run several times until the link stablizes on UP
			 */
			err = al_eth_lm_link_detection(&priv->lm_context, NULL, &old_mode,
						&new_mode);
			/* not setting the serdes since the assumption is that the link mode
			 * won't change during this specific network transaction
			 */
			err = al_eth_lm_link_establish(&priv->lm_context, &link_up);

			al_msleep(LINK_MANAGEMENT_DELAY_BETWEEN_ITERS);

			al_dbg("%s: Link management loop: iteration %d, ret val from link_establish is %d, link_up is %d\n",
			       __func__, i, err, link_up);

			if (link_up) {
				consecutive_success++;
				if (consecutive_success == LINK_MANAGEMENT_CONSECUTIVE_SUCCESS) {
					al_dbg("%s: Exited LM loop after %d consecutive times link was up\n",
					       __func__, consecutive_success);
					break;
				}
			} else {
				consecutive_success = 0;
			}
		}

		if (link_up == 0) {
			al_info("%s: link is %s\n", __func__, err ? "DOWN" : "DOWN_RF");
			return -ENETDOWN;
		}
	}

	return err;
}

/**
 * The init function checks the hardware (probing/identifying) and gets it ready
 * for send/recv operations.  You often do things here such as resetting the MAC
 * and/or PHY, and waiting for the link to autonegotiate.  You should also take
 * the opportunity to program the device's MAC address with the dev->enetaddr
 * member.  This allows the rest of U-Boot to dynamically change the MAC address
 * and have the new settings be respected.
 */
static int al_eth_init(struct eth_device *dev, bd_t *bis)
{
	int err;
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;
	struct phy_device *phydev;
	u32 supported = (SUPPORTED_10baseT_Half |
			SUPPORTED_10baseT_Full |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full |
			SUPPORTED_1000baseT_Full);

	debug("%s(%p, %p)\n", __func__, dev, bis);

	if (priv->is_initialized)
		return 0;

	priv->is_initialized = 1;

	err = al_eth_dev_init(dev);
	if (err)
		return err;

#ifdef CONFIG_MII
	err = al_eth_mdio_config(
		&priv->eth_adapter,
		(priv->ext_phy_if == AL_ETH_BOARD_PHY_IF_XMDIO) ?
				AL_ETH_MDIO_TYPE_CLAUSE_45 : AL_ETH_MDIO_TYPE_CLAUSE_22,
		AL_TRUE, /* shared bus */
		priv->ref_clk_freq,
		priv->mdio_freq);
	if (err) {
		printf("%s: al_eth_mdio_config failed!\n", __func__);
		return err;
	}

	if ((!priv->phy_is_connected) &&
	    (priv->ext_phy_exist == AL_TRUE) &&
	    (priv->ext_phy_if != AL_ETH_BOARD_PHY_IF_I2C) && 
	    (!priv->phy_startup_skip)) {
		phydev = phy_connect(
			priv->phy_bus, priv->phy_addr, dev, priv->phy_if);
		if (!phydev) {
			printf("%s: phy_connect failed!\n", __func__);
			return -ENODEV;
		}

		phydev->supported &= supported;
		phydev->advertising = phydev->supported;

		priv->phy_dev = phydev;

		err = phy_config(phydev);
		if (err) {
			printf("%s: phy_config failed!\n", __func__);
			return err;
		}

		priv->phy_is_connected = 1;
	}

	/* Start up the PHY */
	if ((priv->ext_phy_exist == AL_TRUE) &&
	    (priv->ext_phy_if != AL_ETH_BOARD_PHY_IF_I2C) &&
	    (!priv->phy_startup_skip)) {
		err = phy_startup(priv->phy_dev);
		if (err) {
			printf("%s: phy_startup failed!\n",
				__func__);
			return err;
		}

		/* If there's no link, fail */
		if (!priv->phy_dev->link) {
			printf("%s: No link!\n", __func__);
			return -EIO;
		}

		/** TODO:
		 * Configure based on negotiated speed and duplex reported by PHY
		 */
	}
#endif

	return 0;
}

/**
 * The send function does what you think -- transmit the specified packet whose
 * size is specified by length (in bytes).  You should not return until the
 * transmission is complete, and you should leave the state such that the send
 * function can be called multiple times in a row.
 */
static int al_eth_send(
	struct eth_device	*dev,
	void			*packet,
	int			length)
{
	int err = 0;
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;
	struct	al_eth_pkt eth_tx_packet = {
		.flags = 0,
		.l3_proto_idx = (0),
		.l4_proto_idx = (0),
		.source_vlan_count = (0),
		.vlan_mod_add_count = (0),
		.vlan_mod_del_count = (0),
		.vlan_mod_v1_ether_sel = (0),
		.vlan_mod_v1_vid_sel = (0),
		.vlan_mod_v1_pbits_sel = (0),
		.num_of_bufs = (1),
		.rx_header_len = (0), /* header buffer lenght in bytes */
		.meta = (NULL) /* if null, then no meta added */
	};
	int num_descs;
	int num_descs_comp;
	int poll_count = 0;
	struct al_hal_eth_adapter *eth_adapter =
		&priv->eth_adapter;

#if DEBUG_PRINT_TX_BUFF
	debug("%s(%p, %p, %d)\n", __func__, dev, packet, length);

	{
		uint8_t *packet_buff = (uint8_t *)packet;
		int i;

		debug("Packet data:\n");

		for (i = 0; i < length; i++) {
			if (i && (!(i % 16)))
				debug("\n");
			debug("%02x, ", packet_buff[i]);
		}

		debug("\n");
	}
#endif

	eth_tx_packet.bufs[0].addr = (al_phys_addr_t)(uintptr_t)packet;
	eth_tx_packet.bufs[0].len  = length;

	num_descs = al_eth_tx_pkt_prepare(priv->tx_dma_q, &eth_tx_packet);
	if (!num_descs) {
		printf("%s: al_eth_tx_pkt_prepare failed!\n", __func__);
		goto done;
	}
	/* trigger the dma engine */
	al_eth_tx_dma_action(priv->tx_dma_q, num_descs);

	while (num_descs) {
		num_descs_comp = al_eth_comp_tx_get(priv->tx_dma_q);

		assert(num_descs_comp <= num_descs);

		num_descs -= num_descs_comp;

		if (num_descs)
			udelay(1);

		if (poll_count++ >= TX_DONE_POLL_RETRIES) {
			/* Reset the DMA queue by calling this flush function and then fetch all
			   remaining completions */
			al_eth_mac_tx_flush_config(eth_adapter, AL_TRUE);
			poll_count = 0;
			err = -EIO;
		}

	}

	if (err == -EIO)
		/* End of flush */
		al_eth_mac_tx_flush_config(eth_adapter, AL_FALSE);

done:
	return err;
}

/**
 * The recv function should process packets as long as the hardware has them
 * readily available before returning.  i.e. you should drain the hardware fifo.
 * For each packet you receive, you should call the net_process_received_packet() function on it
 * along with the packet length.  The common code sets up packet buffers for you
 * already in the .bss (net_rx_packets), so there should be no need to allocate
 * your own.  This doesn't mean you must use the net_rx_packets array however;
 * you're free to call the net_process_received_packet() function with any buffer you wish.
 */
static int al_eth_recv(
	struct eth_device *dev)
{
	int err = 0;
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;

	/* debug("%s(%p)\n", __func__, dev); */

	/* TODO: */
	while (!err) {
		struct al_eth_pkt pkt;
		struct al_buf buf;
		int num_descs = al_eth_pkt_rx(priv->rx_dma_q, &pkt);
#if DEBUG_PRINT_RX_BUFF
		int i;
#endif

		if (!num_descs)
			break;

		assert(num_descs == 1);

#if DEBUG_PRINT_RX_BUFF
		debug(
			"%s: received packet at %p, len = %d:\n",
			__func__,
			RX_BUFF(priv->rx_buf_tail_idx),
			pkt.bufs[0].len);

		for (i = 0; i < pkt.bufs[0].len; i++) {
			if (i && (!(i % 16)))
				debug("\n");
			debug(
				"%02x, ",
				*(RX_BUFF(priv->rx_buf_tail_idx) + i));
		}

		debug("\n");
#endif

		if (pkt.flags & (AL_ETH_RX_ERROR | AL_UDMA_CDESC_ERROR))
			debug(
				"%s: received packet with error. flags = 0x%x\n",
				__func__, pkt.flags);
		else
			net_process_received_packet(
				RX_BUFF(priv->rx_buf_tail_idx), pkt.bufs[0].len);

		/* Quit Rx processing loop if incoming buffer caused eth halt */
		if (!priv->is_initialized)
			break;


		buf.addr = (al_phys_addr_t)(uintptr_t)RX_BUFF(priv->rx_buf_tail_idx);
		buf.len = PACKET_SIZE;

		err = al_eth_rx_buffer_add(
			priv->rx_dma_q, &buf, AL_ETH_RX_FLAGS_INT, NULL);
		if (err) {
			printf("%s: al_eth_rx_buffer_add failed!\n", __func__);
			continue;
		}

		al_eth_rx_buffer_action(priv->rx_dma_q, 1);

		priv->rx_buf_tail_idx++;
		if (priv->rx_buf_tail_idx == NUM_RX_BUFFERS)
			priv->rx_buf_tail_idx = 0;
	}

	return err;
}

/**
 * The halt function should turn off / disable the hardware and place it back in
 * its reset state.  It can be called at any time (before any call to the
 * related init function), so make sure it can handle this sort of thing.
 */
static void al_eth_halt(
	struct eth_device *dev)
{
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;
	struct al_eth_board_params board_params;
	char*  skip_eth_halt_env = NULL;

	debug("%s(%p)\n", __func__, dev);

	skip_eth_halt_env = getenv("skip_eth_halt");

	if (skip_eth_halt_env != NULL && strcmp(skip_eth_halt_env, "0"))
		return;

	if (!priv->is_initialized)
		return;

	al_eth_mac_stop(&priv->eth_adapter);

	/* wait till pending rx packets written and UDMA becomes idle,
	 * the MAC has ~10KB fifo, 10us should be enought time for the
	 * UDMA to write to the memory
	 */
	udelay(10);

	if (priv->rev_id >= AL_ETH_REV_ID_2) {
		if (al_udma_q_reset(priv->tx_dma_q))
			printf("al_udma_q_reset(tx) failed!\n");
		if (al_udma_q_reset(priv->rx_dma_q))
			printf("al_udma_q_reset(rx) failed!\n");
	}

	al_eth_adapter_stop(&priv->eth_adapter);

	if (al_eth_board_params_get(
			((struct al_eth_priv *)dev->priv)->mac_regs_base,
			&board_params))
		printf("%s: al_eth_board_params_get failed!\n", __func__);

	if (al_eth_flr_rmn(
		al_eth_flr_read_config_u32,
		al_eth_flr_write_config_u32,
		(void *)(uintptr_t)((struct al_eth_priv *)dev->priv)->pci_devno,
		((struct al_eth_priv *)dev->priv)->mac_regs_base))
		printf("%s: al_eth_flr_workaround failed!\n", __func__);

	if (al_eth_board_params_set(
			((struct al_eth_priv *)dev->priv)->mac_regs_base,
			&board_params))
		printf("%s: al_eth_board_params_set failed!\n", __func__);

	if (al_eth_write_hwaddr(dev))
		printf("%s: al_eth_write_hwaddr failed!\n", __func__);

	if (priv->use_lm)
		al_eth_lm_led_config_terminate(priv);

	priv->is_initialized = 0;
}

/**
 * The write_hwaddr function should program the MAC address stored in
 * dev->enetaddr into the Ethernet controller.
 */
static int al_eth_write_hwaddr(
	struct eth_device *dev)
{
	struct al_eth_priv *priv = (struct al_eth_priv *)dev->priv;
	unsigned char *enetaddr = dev->enetaddr;
	int err;

	debug("%s(%p, %02x:%02x:%02x:%02x:%02x:%02x)\n", __func__, dev,
		enetaddr[0], enetaddr[1], enetaddr[2], enetaddr[3],
		enetaddr[4], enetaddr[5]);

	err = al_eth_mac_addr_store(priv->ec_regs_base, 0, enetaddr);
	if (err) {
		printf("%s: al_eth_mac_addr_store failed!\n", __func__);
		return -1;
	}

	return 0;
}

#ifdef CONFIG_MII
static int al_eth_phy_mdio_register(
	struct eth_device	*eth_dev,
	struct mii_dev		**bus)
{
	int err;

	debug("%s(%s)\n", __func__, eth_dev->name);

	*bus = mdio_alloc();
	if (!(*bus)) {
		printf("%s: mdio_alloc failed!\n", __func__);
		return -ENOMEM;
	}

	(*bus)->read = al_eth_phy_mdio_read;
	(*bus)->write = al_eth_phy_mdio_write;
	(*bus)->priv = eth_dev;
	sprintf((*bus)->name, eth_dev->name);

	err = mdio_register(*bus);
	if (err) {
		printf("%s: mdio_register failed!\n", __func__);
		return err;
	}

	return 0;
}

static int al_eth_phy_mdio_read(
	struct mii_dev *bus,
	int phy_addr,
	int dev_addr,
	int regnum)
{
	uint16_t val;
	int err;

#ifndef CONFIG_AL_ETH_PHY_MDIO_FAKED
	struct eth_device *eth_device =
		(struct eth_device *)(struct eth_device *)bus->priv;
	struct al_eth_priv *eth_priv =
		(struct al_eth_priv *)eth_device->priv;
	struct al_hal_eth_adapter *eth_adapter =
		&eth_priv->eth_adapter;
	int should_init = 0;
#endif

	debug(
		"%s(%s, %d, %d, %d)\n",
		__func__,
		bus->name,
		phy_addr,
		dev_addr,
		regnum);

#ifndef CONFIG_AL_ETH_PHY_MDIO_FAKED
	if (!eth_priv->is_initialized)
		should_init = 1;

	if (should_init) {
		eth_priv->phy_startup_skip = 1;
		err = al_eth_init(eth_device, NULL);
		eth_priv->phy_startup_skip = 0;
		if (err) {
			printf("%s: al_eth_init failed!\n", __func__);
			return err;
		}
	}

	err = al_eth_mdio_read(
			eth_adapter,
			phy_addr,
			dev_addr,
			regnum,
			&val);
	if (err) {
		printf("%s: al_eth_mdio_read failed!\n", __func__);
		return err;
	}

	if (should_init)
		al_eth_halt(eth_device);
#else
	/* fake link up */
	if (regnum == MII_BMSR)
		val = BMSR_LSTATUS;
	/* fake atteros ID */
	else if (regnum == MII_PHYSID1)
		val = 0x004d;
	else if (regnum == MII_PHYSID2)
		val = 0xd040;
	else
		val = 0;
	err = 0;
#endif

	debug(
		"%s: return(%04x)\n",
		__func__,
		val);

	return val;
}

static int al_eth_phy_mdio_write(
	struct mii_dev *bus,
	int phy_addr,
	int dev_addr,
	int regnum,
	u16 value)
{
	int err;

#ifndef CONFIG_AL_ETH_PHY_MDIO_FAKED
	struct eth_device *eth_device =
		(struct eth_device *)(struct eth_device *)bus->priv;
	struct al_eth_priv *eth_priv =
		(struct al_eth_priv *)eth_device->priv;
	struct al_hal_eth_adapter *eth_adapter =
		&eth_priv->eth_adapter;
	int should_init = 0;
#endif

	debug(
		"%s(%s, %d, %d, %d, %04x)\n",
		__func__,
		bus->name,
		phy_addr,
		dev_addr,
		regnum,
		value);

#ifndef CONFIG_AL_ETH_PHY_MDIO_FAKED
	if (!eth_priv->is_initialized)
		should_init = 1;

	if (should_init) {
		eth_priv->phy_startup_skip = 1;
		err = al_eth_init(eth_device, NULL);
		eth_priv->phy_startup_skip = 0;
		if (err) {
			printf("%s: al_eth_init failed!\n", __func__);
			return err;
		}
	}

	err = al_eth_mdio_write(
			eth_adapter,
			phy_addr,
			dev_addr,
			regnum,
			value);
	if (err) {
		printf("%s: al_eth_phy_mdio_write failed!\n", __func__);
		return err;
	}

	if (should_init)
		al_eth_halt(eth_device);
#else
	err = 0;
#endif

	return 0;
}
#endif

static int al_eth_flr_read_config_u32(
	void *handle,
	int where,
	uint32_t *val)
{
	pci_dev_t devno = (pci_dev_t)(uintptr_t)handle;

	pci_read_config_dword(devno, where, val);

	debug("%s(%d.%d.%d, %d) --> %08x\n", __func__, PCI_BUS(devno),
		PCI_DEV(devno),	PCI_FUNC(devno), where, *val);

	return 0;
}

static int al_eth_flr_write_config_u32(
	void *handle,
	int where,
	uint32_t val)
{
	pci_dev_t devno = (pci_dev_t)(uintptr_t)handle;

	debug("%s(%d.%d.%d, %d) <-- %08x\n", __func__, PCI_BUS(devno),
		PCI_DEV(devno),	PCI_FUNC(devno), where, val);

	pci_write_config_dword(devno, where, val);

	return 0;
}

static void al_eth_random_enetaddr(
	uchar *enetaddr)
{
	static int first = 1;
	uint32_t rval;

	if (first) {
		srand(sys_counter_cnt_get_low());
		first = 0;
	}

	rval = rand();
	enetaddr[0] = rval & 0xff;
	enetaddr[1] = (rval >> 8) & 0xff;
	enetaddr[2] = (rval >> 16) & 0xff;

	rval = rand();
	enetaddr[3] = rval & 0xff;
	enetaddr[4] = (rval >> 8) & 0xff;
	enetaddr[5] = (rval >> 16) & 0xff;

	/* make sure it's local and unicast */
	enetaddr[0] = (enetaddr[0] | 0x02) & ~0x01;
}

int al_eth_board_params_load(int index, struct al_eth_board_params *params)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	al_eth_board_params_get(ifs[index]->mac_regs_base, params);

	return 0;
}

int al_eth_board_params_store(int index, struct al_eth_board_params *params)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	al_eth_board_params_set(ifs[index]->mac_regs_base, params);

	return 0;
}

int al_eth_mac_link_set(int index, al_bool sgmii, al_bool autoneg,
			uint32_t speed, al_bool full_duplex)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	ifs[index]->link_config.autoneg = autoneg;
	ifs[index]->link_config.sgmii = sgmii;
	ifs[index]->link_config.speed = speed;
	ifs[index]->link_config.full_duplex = full_duplex;

	return 0;
}

int al_eth_serdes_override_set(int index, al_bool override)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	ifs[index]->dont_override_serdes = (override == AL_TRUE) ? AL_FALSE : AL_TRUE;

	return 0;
}

int al_eth_mac_mode_set(int index, enum al_eth_board_media_type mac_mode)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	if (mac_mode ==	AL_ETH_BOARD_MEDIA_TYPE_SGMII) {
		ifs[index]->sfp_detect_needed = AL_FALSE;
		ifs[index]->mac_mode = AL_ETH_MAC_MODE_SGMII;
	} else if (mac_mode == AL_ETH_BOARD_MEDIA_TYPE_RGMII) {
		ifs[index]->sfp_detect_needed = AL_FALSE;
		ifs[index]->mac_mode = AL_ETH_MAC_MODE_RGMII;
	} else if (mac_mode == AL_ETH_BOARD_MEDIA_TYPE_10GBASE_SR) {
		ifs[index]->sfp_detect_needed = AL_FALSE;
		ifs[index]->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
	} else if (mac_mode == AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT) {
		ifs[index]->sfp_detect_needed = AL_TRUE;
		ifs[index]->auto_speed_needed = AL_FALSE;
		ifs[index]->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
	} else if (mac_mode == AL_ETH_BOARD_MEDIA_TYPE_AUTO_DETECT_AUTO_SPEED) {
		ifs[index]->sfp_detect_needed = AL_TRUE;
		ifs[index]->auto_speed_needed = AL_TRUE;
		ifs[index]->mac_mode = AL_ETH_MAC_MODE_10GbE_Serial;
	} else {
		printf("%s: Non supported media type (%d)!",
			__func__,
			mac_mode);
		return -1;
	}

	return 0;
}

int al_eth_link_training_enable(int index, al_bool enable)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	ifs[index]->link_training_needed = enable;

	return 0;
}

int al_eth_link_management_debug_enable(int index, al_bool enable)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	ifs[index]->lm_debug = enable;

	return 0;
}

int al_eth_retimer_config_override(int index,
				   al_bool exist,
				   uint8_t i2c_bus_id,
				   uint8_t i2c_addr,
				   enum al_eth_retimer_channel channel)
{
	if ((index < 0) || (index >= AL_ETH_MAX_NUM_INTERFACES) ||
	    (ifs[index] == NULL))
		return -EINVAL;

	ifs[index]->retimer_exist = exist;
	ifs[index]->retimer_bus_id = i2c_bus_id;
	ifs[index]->retimer_i2c_addr = i2c_addr;
	ifs[index]->retimer_channel = channel;

	return 0;
}
