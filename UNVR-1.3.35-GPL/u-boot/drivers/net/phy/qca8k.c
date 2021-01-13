/*
 * (C) Copyright 2018
 * Ubiquiti Networks Inc.
 * Matt Hsu <matt.hsu@ubnt.com>
 *
 */

#include <common.h>
#include <netdev.h>
#include <miiphy.h>
#include <phy.h>

#include "qca8k.h"

static u16 qca8k_current_page = 0xffff;

static void
qca8k_split_addr(u32 regaddr, u16 *r1, u16 *r2, u16 *page)
{
	regaddr >>= 1;
	*r1 = regaddr & 0x1e;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0x3ff;
}

static u32
qca8k_mii_read32(char *devname, int phy_id, u32 regnum)
{
	u32 val;
	int ret;
	struct mii_dev *bus;

	bus = miiphy_get_dev_by_name(eth_get_name());

	ret = bus->read(bus, phy_id, MDIO_DEVAD_NONE, regnum);
	if (ret >= 0) {
		val = ret;
		ret = bus->read(bus, phy_id, MDIO_DEVAD_NONE, regnum + 1);
		val |= ret << 16;
	}

	if (ret < 0) {
		printf("failed to read qca8k 32bit register\n");
		return ret;
	}

	return val;
}

static void
qca8k_mii_write32(char *devname, int phy_id, u32 regnum, u32 val)
{
	u16 lo, hi;
	int ret;
	struct mii_dev *bus;

	bus = miiphy_get_dev_by_name(eth_get_name());

	lo = val & 0xffff;
	hi = (u16)(val >> 16);

	ret = bus->write(bus, phy_id, MDIO_DEVAD_NONE, regnum, lo);
	if (ret >= 0)
		ret = bus->write(bus, phy_id, MDIO_DEVAD_NONE, regnum + 1, hi);
	if (ret < 0)
		printf("failed to write qca8k 32bit register\n");
}

static void
qca8k_set_page(char *devname, u16 page)
{
	struct mii_dev *bus;

	bus = miiphy_get_dev_by_name(eth_get_name());

	if (page == qca8k_current_page)
		return;

	if (bus->write(bus, 0x18, MDIO_DEVAD_NONE, 0, page) < 0)
		printf("failed to set qca8k page\n");

	qca8k_current_page = page;
}

static u32
qca8k_read(char *devname, u32 reg)
{
	u16 r1, r2, page;
	u32 val;

	qca8k_split_addr(reg, &r1, &r2, &page);

	qca8k_set_page(devname, page);
	val = qca8k_mii_read32(devname, 0x10 | r2, r1);

	return val;
}

static void
qca8k_write(char *devname, u32 reg, u32 val)
{
	u16 r1, r2, page;

	qca8k_split_addr(reg, &r1, &r2, &page);

	qca8k_set_page(devname, page);
	qca8k_mii_write32(devname, 0x10 | r2, r1, val);
}

static u32
qca8k_rmw(char *devname, u32 reg, u32 mask, u32 val)
{
	u16 r1, r2, page;
	u32 ret;

	qca8k_split_addr(reg, &r1, &r2, &page);

	qca8k_set_page(devname, page);
	ret = qca8k_mii_read32(devname, 0x10 | r2, r1);
	ret &= ~mask;
	ret |= val;
	qca8k_mii_write32(devname, 0x10 | r2, r1, ret);

	return ret;
}

static void
qca8k_reg_set(char *devname, u32 reg, u32 val)
{
	qca8k_rmw(devname, reg, 0, val);
}

static void
qca8k_reg_clear(char *devname, u32 reg, u32 val)
{
	qca8k_rmw(devname, reg, val, 0);
}

static int
qca8k_set_pad_ctrl(char *devname, int port, int mode)
{
	u32 reg;

	switch (port) {
	case 0:
		reg = QCA8K_REG_PORT0_PAD_CTRL;
		break;
	case 6:
		reg = QCA8K_REG_PORT6_PAD_CTRL;
		break;
	default:
		printf("Can't set PAD_CTRL on port %d\n", port);
		return -1;
	}

    /* Configure a port to be directly connected to an external
     * PHY or MAC.
     */
    switch (mode) {
    case PHY_INTERFACE_MODE_RGMII:
        qca8k_write(devname, reg,
                QCA8K_PORT_PAD_RGMII_EN |
                QCA8K_PORT_PAD_RGMII_TX_DELAY(3) |
                QCA8K_PORT_PAD_RGMII_RX_DELAY(3));
        /* According to the datasheet, RGMII delay is enabled through
         * PORT5_PAD_CTRL for all ports, rather than individual port
         * registers
         */
        qca8k_write(devname, QCA8K_REG_PORT5_PAD_CTRL,
                QCA8K_PORT_PAD_RGMII_RX_DELAY_EN);
        break;
    case PHY_INTERFACE_MODE_SGMII:
        qca8k_write(devname, reg, QCA8K_PORT_PAD_SGMII_EN);
        break;
    default:
        printf("xMII mode %d not supported\n", mode);
        return -1;
    }

	return 0;
}

static void
qca8k_port_set_status(char *devname, int port, int enable)
{
	u32 mask = QCA8K_PORT_STATUS_TXMAC;

	/* Port 0 and 6 have no internal PHY */
	if ((port > 0) && (port < 6))
		mask |= QCA8K_PORT_STATUS_LINK_AUTO;
	else {
		mask = QCA8K_PORT_STATUS_FLOW_LINK_EN;
		qca8k_reg_clear(devname, QCA8K_REG_PORT_STATUS(port), mask);

		mask = (QCA8K_PORT_STATUS_TXFLOW
			| QCA8K_PORT_STATUS_RXFLOW
			| QCA8K_PORT_STATUS_DUPLEX
			| QCA8K_PORT_STATUS_SPEED_1000M
			| QCA8K_PORT_STATUS_TXMAC
			| QCA8K_PORT_STATUS_RXMAC);
	}

	if (enable)
		qca8k_reg_set(devname, QCA8K_REG_PORT_STATUS(port), mask);
	else
		qca8k_reg_clear(devname, QCA8K_REG_PORT_STATUS(port), mask);
}

static bool dev_initialized = false;
/*
 * QCA833x Switch initialization
 */
int qca8k_switch_initialize(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	u32 id;

	if (!dev_initialized) {
		eth_init();

		/* read the switches ID register */
		id = qca8k_read(eth_get_name(), QCA8K_REG_MASK_CTRL);
		id >>= QCA8K_MASK_CTRL_ID_S;
		id &= QCA8K_MASK_CTRL_ID_M;
		printf("%s: QCA8K_ID_QCA8337 0x%02x\n", eth_get_name(), id);

		if (id != QCA8K_ID_QCA8337)
			return -1;

		qca8k_set_pad_ctrl(eth_get_name(), QCA8K_CPU_PORT, QCA8K_CPU_PORT_INTERFACE);
		qca8k_set_pad_ctrl(eth_get_name(), QCA8K_CPU_PORT6, QCA8K_CPU_PORT6_INTERFACE);

		/* Enable CPU Port */
		qca8k_reg_set(eth_get_name(), QCA8K_REG_GLOBAL_FW_CTRL0,
			QCA8K_GLOBAL_FW_CTRL0_CPU_PORT_EN);

		qca8k_reg_set(eth_get_name(), QCA8K_REG_GLOBAL_FW_CTRL1,
			BIT(0) << QCA8K_GLOBAL_FW_CTRL1_IGMP_DP_S |
            BIT(0) << QCA8K_GLOBAL_FW_CTRL1_BC_DP_S |
            BIT(0) << QCA8K_GLOBAL_FW_CTRL1_MC_DP_S |
            BIT(0) << QCA8K_GLOBAL_FW_CTRL1_UC_DP_S);

		qca8k_port_set_status(eth_get_name(), QCA8K_CPU_PORT, 1);
		qca8k_port_set_status(eth_get_name(), QCA8K_CPU_PORT6, 1);

		/* SGMII enablement */
		qca8k_write(eth_get_name(),
			QCA8K_REG_SGMII_CTRL , QCA8K_SGMII_EN_RX |
								QCA8K_SGMII_EN_TX |
								QCA8K_SGMII_EN_SD |
								QCA8K_SGMII_EN_PLL|
								QCA8K_SGMII_BW_HIGH |
								QCA8K_SGMII_SEL_CLK125M |
								QCA8K_SGMII_TXDR_CTRL_600mV |
								QCA8K_SGMII_CDR_BW_8 |
								QCA8K_SGMII_DIS_AUTO_LPI_25M |
								QCA8K_SGMII_MODE_CTRL_SGMII_PHY |
								QCA8K_SGMII_PAUSE_SG_TX_EN_25M |
								QCA8K_SGMII_ASYM_PAUSE_25M |
								QCA8K_SGMII_PAUSE_25M |
								QCA8K_SGMII_HALF_DUPLEX_25M |
								QCA8K_SGMII_FULL_DUPLEX_25M);

		dev_initialized = true;
	}

	return 0;
}

U_BOOT_CMD(qca8k, 1, 0, qca8k_switch_initialize,
	"initialize QCA8334/QCA8337 switch device", "qca8k"
);
