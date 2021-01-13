#ifndef _qca8k_h_
#define _qca8k_h_

#define BIT(x)			(1 << (x))

#define PHY_ID_QCA8337                  0x004dd036
#define QCA8K_ID_QCA8337                0x13

#define QCA8K_CPU_PORT                  0
#define QCA8K_CPU_PORT6                 6

/* Global control registers */
#define QCA8K_REG_MASK_CTRL             0x000
#define   QCA8K_MASK_CTRL_ID_M              0xff
#define   QCA8K_MASK_CTRL_ID_S              8
#define QCA8K_REG_PORT0_PAD_CTRL            0x004
#define QCA8K_REG_PORT5_PAD_CTRL            0x008
#define QCA8K_REG_PORT6_PAD_CTRL            0x00c
#define QCA8K_REG_SGMII_CTRL            0x00e0

#define   QCA8K_PORT_PAD_RGMII_EN           BIT(26)
#define   QCA8K_PORT_PAD_RGMII_TX_DELAY(x)      \
				((0x8 + (x & 0x3)) << 22)
#define   QCA8K_PORT_PAD_RGMII_RX_DELAY(x)      \
				((0x10 + (x & 0x3)) << 20)
#define   QCA8K_PORT_PAD_RGMII_RX_DELAY_EN      BIT(24)
#define   QCA8K_PORT_PAD_SGMII_EN           BIT(7)

#define   QCA8K_PORT_STATUS_SPEED_1000M 	BIT(1)
#define   QCA8K_PORT_STATUS_TXMAC           BIT(2)
#define   QCA8K_PORT_STATUS_RXMAC           BIT(3)
#define   QCA8K_PORT_STATUS_TXFLOW          BIT(4)
#define   QCA8K_PORT_STATUS_RXFLOW          BIT(5)
#define   QCA8K_PORT_STATUS_DUPLEX          BIT(6)
#define   QCA8K_PORT_STATUS_LINK_UP         BIT(8)
#define   QCA8K_PORT_STATUS_LINK_AUTO           BIT(9)
#define   QCA8K_PORT_STATUS_LINK_PAUSE          BIT(10)
#define   QCA8K_PORT_STATUS_FLOW_LINK_EN          BIT(12)

#define QCA8K_REG_PORT_STATUS(_i)     (0x07c + (_i) * 4)
#define QCA8K_REG_GLOBAL_FW_CTRL0           0x620
#define QCA8K_REG_GLOBAL_FW_CTRL1           0x624
#define   QCA8K_GLOBAL_FW_CTRL1_IGMP_DP_S       24
#define   QCA8K_GLOBAL_FW_CTRL1_BC_DP_S         16
#define   QCA8K_GLOBAL_FW_CTRL1_MC_DP_S         8
#define   QCA8K_GLOBAL_FW_CTRL1_UC_DP_S         0
#define QCA8K_GLOBAL_FW_CTRL0_CPU_PORT_EN     BIT(10)

/* SGMII_CTRL bit definitions */
#define QCA8K_SGMII_EN_LCKDT           BIT(0)
#define QCA8K_SGMII_EN_PLL             BIT(1)
#define QCA8K_SGMII_EN_RX              BIT(2)
#define QCA8K_SGMII_EN_TX              BIT(3)
#define QCA8K_SGMII_EN_SD              BIT(4)
#define QCA8K_SGMII_BW_HIGH            BIT(6)
#define QCA8K_SGMII_SEL_CLK125M        BIT(7)
#define QCA8K_SGMII_TXDR_CTRL_600mV    BIT(10)
#define QCA8K_SGMII_CDR_BW_8           BIT(13)
#define QCA8K_SGMII_DIS_AUTO_LPI_25M       BIT(16)
#define QCA8K_SGMII_MODE_CTRL_SGMII_PHY    BIT(22)
#define QCA8K_SGMII_PAUSE_SG_TX_EN_25M     BIT(24)
#define QCA8K_SGMII_ASYM_PAUSE_25M         BIT(25)
#define QCA8K_SGMII_PAUSE_25M              BIT(26)
#define QCA8K_SGMII_HALF_DUPLEX_25M        BIT(30)
#define QCA8K_SGMII_FULL_DUPLEX_25M        BIT(31)

#endif
