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

#ifndef __AL_HAL_UNIT_ADAPTER_REGS_H__
#define __AL_HAL_UNIT_ADAPTER_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *                       PCIe offsets/masks (by spec)                         *
 ******************************************************************************/

/**
 * Device-ID/Vendor-ID
 */
#define AL_PCI_DEV_ID_VEN_ID			0x00
#define AL_PCI_DEV_ID_VEN_ID_DEV_ID_MASK	0xffff0000
#define AL_PCI_DEV_ID_VEN_ID_DEV_ID_SHIFT	16
#define AL_PCI_DEV_ID_VEN_ID_VEN_ID_MASK	0x0000ffff
#define AL_PCI_DEV_ID_VEN_ID_VEN_ID_SHIFT	0

/**
 * Command
 */
#define AL_PCI_COMMAND				0x04       /* 16 bits */
#define AL_PCI_COMMAND_IO			0x1        /* Enable response in I/O space */
#define AL_PCI_COMMAND_MEMORY			0x2        /* Enable response in Memory space */
#define AL_PCI_COMMAND_MASTER			0x4        /* Enable bus mastering */
#define AL_PCI_COMMAND_INTR_DISABLE		AL_BIT(10) /* Adapter Interrupt disable */
#define AL_PCI_COMMAND_SERR			AL_BIT(8) /* SERR enables reporting of errors */

/**
 * Class
 */
#define PCI_CLASS_REVISION			0x08 /* High 24 bits are class, low 8 revision */
#define PCI_CLASS_REVISION_CLASS_MASK		0xffffff00
#define PCI_CLASS_REVISION_CLASS_SHIFT		8
#define PCI_CLASS_REVISION_REVISION_MASK	0x000000ff
#define PCI_CLASS_REVISION_REVISION_SHIFT	0

/**
 * Bars (Base Address Registers)
 */
#define  AL_PCI_BASE_ADDRESS_SPACE_IO		0x01
#define  AL_PCI_BASE_ADDRESS_MEM_TYPE_64	0x04 /* 64 bit address */
#define  AL_PCI_BASE_ADDRESS_MEM_PREFETCH	0x08 /* prefetchable? */
#define  AL_PCI_BASE_ADDRESS_DEVICE_ID		0x0c

#define  AL_PCI_BASE_ADDRESS_0			0x10
#define  AL_PCI_BASE_ADDRESS_0_HI		0x14
#define  AL_PCI_BASE_ADDRESS_2			0x18
#define  AL_PCI_BASE_ADDRESS_2_HI		0x1c
#define  AL_PCI_BASE_ADDRESS_4			0x20
#define  AL_PCI_BASE_ADDRESS_4_HI		0x24

/**
 * Expansion ROM
 */
#define	AL_PCI_EXP_ROM_BASE_ADDRESS		0x30

/**
 * Subsystem-ID/Vendor-ID
 */
#define  AL_PCI_SUBSYSTEM_VENDOR_ID		0x2c
#define  AL_PCI_SUBSYSTEM_ID			0x2e

/**
 * PCI Express capability
 */
#define AL_PCI_EXP_DEVCAP			4         /* Device capabilities */
#define  AL_PCI_EXP_DEVCAP_PAYLOAD		0x07      /* Max_Payload_Size */
#define  AL_PCI_EXP_DEVCAP_PHANTOM		0x18      /* Phantom functions */
#define  AL_PCI_EXP_DEVCAP_EXT_TAG		0x20      /* Extended tags */
#define  AL_PCI_EXP_DEVCAP_L0S			0x1c0     /* L0s Acceptable Latency */
#define  AL_PCI_EXP_DEVCAP_L1			0xe00     /* L1 Acceptable Latency */
#define  AL_PCI_EXP_DEVCAP_ATN_BUT		0x1000    /* Attention Button Present */
#define  AL_PCI_EXP_DEVCAP_ATN_IND		0x2000    /* Attention Indicator Present */
#define  AL_PCI_EXP_DEVCAP_PWR_IND		0x4000    /* Power Indicator Present */
#define  AL_PCI_EXP_DEVCAP_RBER			0x8000    /* Role-Based Error Reporting */
#define  AL_PCI_EXP_DEVCAP_PWR_VAL		0x3fc0000 /* Slot Power Limit Value */
#define  AL_PCI_EXP_DEVCAP_PWR_SCL		0xc000000 /* Slot Power Limit Scale */
#define  AL_PCI_EXP_DEVCAP_FLR			0x10000000 /* Function Level Reset */
#define AL_PCI_EXP_DEVCTL			8         /* Device Control */
#define  AL_PCI_EXP_DEVCTL_CERE			0x0001    /* Correctable Error Reporting En */
#define  AL_PCI_EXP_DEVCTL_NFERE		0x0002    /* Non-Fatal Error Reporting Enable */
#define  AL_PCI_EXP_DEVCTL_FERE			0x0004    /* Fatal Error Reporting Enable */
#define  AL_PCI_EXP_DEVCTL_URRE			0x0008    /* Unsupported Request Reporting En */
#define  AL_PCI_EXP_DEVCTL_RELAX_EN		0x0010    /* Enable relaxed ordering */
#define  AL_PCI_EXP_DEVCTL_PAYLOAD		0x00e0    /* Max_Payload_Size */
#define  AL_PCI_EXP_DEVCTL_EXT_TAG		0x0100    /* Extended Tag Field Enable */
#define  AL_PCI_EXP_DEVCTL_PHANTOM		0x0200    /* Phantom Functions Enable */
#define  AL_PCI_EXP_DEVCTL_AUX_PME		0x0400    /* Auxiliary Power PM Enable */
#define  AL_PCI_EXP_DEVCTL_NOSNOOP_EN		0x0800    /* Enable No Snoop */
#define  AL_PCI_EXP_DEVCTL_READRQ		0x7000    /* Max_Read_Request_Size */
#define  AL_PCI_EXP_DEVCTL_BCR_FLR		0x8000    /* Bridge Configuration Retry / FLR */
#define AL_PCI_EXP_DEVSTA			0xA       /* Device Status */
#define AL_PCI_EXP_DEVSTA_SHIFT			16        /* Device Status field shift*/
#define AL_PCI_EXP_DEVSTA_MASK			0xFFFF0000 /* Device Status field mask*/
#define  AL_PCI_EXP_DEVSTA_CED			0x01      /* Correctable Error Detected */
#define  AL_PCI_EXP_DEVSTA_NFED			0x02      /* Non-Fatal Error Detected */
#define  AL_PCI_EXP_DEVSTA_FED			0x04      /* Fatal Error Detected */
#define  AL_PCI_EXP_DEVSTA_URD			0x08      /* Unsupported Request Detected */
#define  AL_PCI_EXP_DEVSTA_AUXPD		0x10      /* AUX Power Detected */
#define  AL_PCI_EXP_DEVSTA_TRPND		0x20      /* Transactions Pending */
#define AL_PCI_EXP_LNKCAP			0xC	  /* Link Capabilities */
#define  AL_PCI_EXP_LNKCAP_SLS			0xf	  /* Supported Link Speeds */
#define  AL_PCI_EXP_LNKCAP_SLS_2_5GB		0x1       /* LNKCAP2 SLS Vector bit 0 (2.5GT/s) */
#define  AL_PCI_EXP_LNKCAP_SLS_5_0GB		0x2       /* LNKCAP2 SLS Vector bit 1 (5.0GT/s) */
#define  AL_PCI_EXP_LNKCAP_MLW			0x3f0     /* Maximum Link Width */
#define  AL_PCI_EXP_LNKCAP_ASPMS		0xc00     /* ASPM Support */
#define  AL_PCI_EXP_LNKCAP_L0SEL		0x7000    /* L0s Exit Latency */
#define  AL_PCI_EXP_LNKCAP_L1EL			0x38000   /* L1 Exit Latency */
#define  AL_PCI_EXP_LNKCAP_CLKPM		0x40000   /* L1 Clock Power Management */
#define  AL_PCI_EXP_LNKCAP_SDERC		0x80000   /* Surprise Down Err Reporting Capable */
#define  AL_PCI_EXP_LNKCAP_DLLLARC		0x100000  /* DLL Link Active Reporting Capable */
#define  AL_PCI_EXP_LNKCAP_LBNC			0x200000  /* Link BW Notification Capability */
#define  AL_PCI_EXP_LNKCAP_PN			0xff000000 /* Port Number */
#define AL_PCI_EXP_LNKCTL			0x10      /* Link Control */
#define  AL_PCI_EXP_LNKCTL_LNK_DIS		0x10      /* Link Disable Status */
#define  AL_PCI_EXP_LNKCTL_LNK_RTRN		0x20      /* Link Retrain Status */
#define  AL_PCI_EXP_LNKCTL_CCC			0x40      /* Common Clock Configuration */
#define  AL_PCI_EXP_LNKCTL_ES			0x80      /* Extended Synch */
#define AL_PCI_EXP_LNKSTA			0x12      /* Link Status */
#define  AL_PCI_EXP_LNKSTA_CLS			0x000f    /* Current Link Speed */
#define  AL_PCI_EXP_LNKSTA_CLS_2_5GB		0x01      /* Current Link Speed 2.5GT/s */
#define  AL_PCI_EXP_LNKSTA_CLS_5_0GB		0x02      /* Current Link Speed 5.0GT/s */
#define  AL_PCI_EXP_LNKSTA_CLS_8_0GB		0x03      /* Current Link Speed 8.0GT/s */
#define  AL_PCI_EXP_LNKSTA_NLW			0x03f0    /* Negotiated Link Width */
#define  AL_PCI_EXP_LNKSTA_NLW_SHIFT		4         /* start of NLW mask in link status */
#define  AL_PCI_EXP_LNKSTA_LT			0x0800    /* Link Training */
#define  AL_PCI_EXP_LNKSTA_SLC			0x1000    /* Slot Clock Configuration */
#define  AL_PCI_EXP_LNKSTA_DLLLA		0x2000    /* Data Link Layer Link Active */
#define  AL_PCI_EXP_LNKSTA_LBMS			0x4000    /* Link Bandwidth Management Status */
#define  AL_PCI_EXP_LNKSTA_LABS			0x8000    /* Link Autonomous Bandwidth Status */
#define AL_PCI_EXP_DEVCTL2			0x28      /* Device Control 2 */
#define AL_PCI_EXP_LNKCTL2			0x30      /* Link Control 2 */
#define AL_PCI_EXP_LNKCTL3			0x4       /* Link Control 3 */
#define  AL_PCI_EXP_LNKCTL3_PRFRM_EQ		AL_BIT(0) /* Bit0: Perform Eq */
#define  AL_PCI_EXP_LNKCTL3_EQ_INT_EN		AL_BIT(1) /* Bit1: Link Eq Request Int Enable */

/**
 * MSI-x capability
 */
#define AL_PCI_MSIX_MSGCTRL			0          /* MSIX message control reg */
#define  AL_PCI_MSIX_MSGCTRL_TBL_SIZE		0x7ff      /* MSIX table size */
#define  AL_PCI_MSIX_MSGCTRL_TBL_SIZE_SHIFT	16         /* MSIX table size shift */
#define  AL_PCI_MSIX_MSGCTRL_EN			0x80000000 /* MSIX enable */
#define  AL_PCI_MSIX_MSGCTRL_MASK		0x40000000 /* MSIX mask */
#define AL_PCI_MSIX_TABLE			0x4        /* MSIX table offset and bar reg */
#define  AL_PCI_MSIX_TABLE_OFFSET		0xfffffff8 /* MSIX table offset */
#define  AL_PCI_MSIX_TABLE_BAR			0x7        /* MSIX table BAR */
#define AL_PCI_MSIX_PBA				0x8        /* MSIX pba offset and bar reg */
#define  AL_PCI_MSIX_PBA_OFFSET			0xfffffff8 /* MSIX pba offset */
#define  AL_PCI_MSIX_PBA_BAR			0x7        /* MSIX pba BAR */

/**
 * TPH capability
 */
#define AL_PCI_TPH_REQ_CTRL_OFFSET		0x8   /* offset from TPH cap */
#define AL_PCI_TPH_REQ_ENABLE_MASK		0x300 /* req enable mask in request control reg */
#define AL_PCI_TPH_REQ_ENABLE_SHIFT		0x8   /* req enable shift in request control reg */
#define AL_PCI_TPH_REQ_ENABLE_REG		0x01  /* regular req control enable */
#define AL_PCI_TPH_REQ_ENABLE_REG_EXT		0x11  /* regular/extended req control enable */

 /**
 * Device Serial Number capability
 */
#define  AL_PCI_EXP_DSN_DW1	4	/* Device Serial Number lower dword */
#define  AL_PCI_EXP_DSN_DW2	8	/* Device Serial Number upper dword*/

/**
 * PM capability
 */
#define AL_ADAPTER_PM_0_PM_NEXT_CAP_MASK	0xff00
#define AL_ADAPTER_PM_0_PM_NEXT_CAP_SHIFT	8
#define AL_ADAPTER_PM_0_PM_NEXT_CAP_VAL_MSIX	0x90
#define AL_ADAPTER_PM_1_PME_EN			0x100	/* PM enable */
#define AL_ADAPTER_PM_1_PWR_STATE_MASK		0x3	/* PM state mask */
#define AL_ADAPTER_PM_1_PWR_STATE_D0		0x0	/* PM D0 state */
#define AL_ADAPTER_PM_1_PWR_STATE_D3		0x3	/* PM D3 state */

/******************************************************************************
 *          AL specific PCIe controller config space offsets/masks            *
 ******************************************************************************/

/**
 * Capabilities offsets
 */
#define AL_PCIE_CFG_EXP_CAP_BASE_OFFSET		0x70
#define AL_PCIE_CFG_MSI_CAP_BASE_OFFSET		0xb0
#define AL_PCIE_CFG_EXP_DSN			0x140
#define AL_PCIE_CFG_TPH_CAP_BASE_OFFSET		0x178

/******************************************************************************
 *          AL Specific Embedded PCIe Adapter config space offset/masks       *
 ******************************************************************************/

/**
 * Capabilities offsets
 */
#define AL_PCI_EXP_CAP_BASE			0x40
#define AL_PCI_MSI_CAP_BASE			0x50
#define AL_PCI_MSIX_CAP_BASE			0xb0
#define AL_ADAPTER_PM_0				0x80
#define AL_ADAPTER_PM_1				0x84

/**
 * AXI configuration and control
 */
#define  AL_PCI_AXI_CFG_AND_CTR_0		0x110
#define  AL_PCI_AXI_CFG_AND_CTR_1		0x130
#define  AL_PCI_AXI_CFG_AND_CTR_2		0x150
#define  AL_PCI_AXI_CFG_AND_CTR_3		0x170

/**
 * Adapter Application control
 */
#define  AL_PCI_APP_CONTROL			0x220

/**
 * SR/IOV
 */
#define  AL_PCI_SRIOV_TOTAL_AND_INITIAL_VFS	0x30c
#define  AL_PCI_VF_BASE_ADDRESS_0		0x324

/* Sub Master Configuration & Control registers */
/* AXI_Sub_Master_Configuration */
#define AL_ADAPTER_SMCC					0x110
#define AL_ADAPTER_SMCC_BUNDLE_SIZE			0x20
#define AL_ADAPTER_SMCC_CONF_SNOOP_OVR		AL_BIT(0) /* Snoop Override*/
#define AL_ADAPTER_SMCC_CONF_SNOOP_ENABLE	AL_BIT(1) /* Snoop Enable*/

/* AXI_Sub_Master_Configuration_2 */
#define AL_ADAPTER_SMCC_CONF_2					0x114
#define AL_ADAPTER_SMCC_CONF_2_DIS_ERROR_TRACK		AL_BIT(8)

/* Interrupt_Cause register */
#define AL_ADAPTER_INT_CAUSE			0x1B0
#define AL_ADAPTER_INT_CAUSE_WR_ERR		AL_BIT(1)
#define AL_ADAPTER_INT_CAUSE_RD_ERR		AL_BIT(0)
#define AL_ADAPTER_INT_CAUSE_APP_PARITY_ERR	AL_BIT(3)

/* AXI_Master_Write_Error_Attribute_Latch register */
/* AXI_Master_Read_Error_Attribute_Latch register */
#define AL_ADAPTER_AXI_MSTR_WR_ERR_ATTR			0x1B4
#define AL_ADAPTER_AXI_MSTR_RD_ERR_ATTR			0x1B8

#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_STAT_MASK	AL_FIELD_MASK(1, 0)
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_STAT_SHIFT	0
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_MASK		AL_FIELD_MASK(4, 2)
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_MSTR_ID_SHIFT	2
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_ADDR_TO		AL_BIT(8)
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_ERR		AL_BIT(9)
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_COMP_TO		AL_BIT(10)
#define AL_ADAPTER_AXI_MSTR_RD_WR_ERR_ATTR_ERR_BLK		AL_BIT(11)
#define AL_ADAPTER_AXI_MSTR_RD_ERR_ATTR_RD_PARITY_ERR		AL_BIT(12)

/* Interrupt_Cause_mask register */
#define AL_ADAPTER_INT_CAUSE_MASK			0x1BC
#define AL_ADAPTER_INT_CAUSE_MASK_WR_ERR		AL_BIT(1)
#define AL_ADAPTER_INT_CAUSE_MASK_RD_ERR		AL_BIT(0)
#define AL_ADAPTER_INT_CAUSE_MASK_APP_PARITY_ERR	AL_BIT(3)

/* AXI_Master_write_error_address_Latch register */
#define AL_ADAPTER_AXI_MSTR_WR_ERR_LO_LATCH	0x1C0

/* AXI_Master_write_error_address_high_Latch register */
#define AL_ADAPTER_AXI_MSTR_WR_ERR_HI_LATCH	0x1C4

/* AXI_Master_read_error_address_Latch register */
#define AL_ADAPTER_AXI_MSTR_RD_ERR_LO_LATCH	0x1C8

/* AXI_Master_read_error_address_high_Latch register */
#define AL_ADAPTER_AXI_MSTR_RD_ERR_HI_LATCH	0x1CC

/* AXI_Master_Timeout register */
#define AL_ADAPTER_AXI_MSTR_TO			0x1D0
#define AL_ADAPTER_AXI_MSTR_TO_WR_MASK		AL_FIELD_MASK(31, 16)
#define AL_ADAPTER_AXI_MSTR_TO_WR_SHIFT		16
#define AL_ADAPTER_AXI_MSTR_TO_RD_MASK		AL_FIELD_MASK(15, 0)
#define AL_ADAPTER_AXI_MSTR_TO_RD_SHIFT		0

/* Application_Parity_Status register */
#define AL_ADAPTER_APP_PARITY_STATUS				0x1D4
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_TX_RAM		AL_BIT(0)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_RX_RAM		AL_BIT(1)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_FB_LOW_RAM	AL_BIT(2)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_0_FB_HIGH_RAM	AL_BIT(3)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_TX_RAM		AL_BIT(4)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_RX_RAM		AL_BIT(5)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_FB_LOW_RAM	AL_BIT(6)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_1_FB_HIGH_RAM	AL_BIT(7)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_TX_RAM		AL_BIT(8)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_RX_RAM		AL_BIT(9)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_FB_LOW_RAM	AL_BIT(10)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_2_FB_HIGH_RAM	AL_BIT(11)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_TX_RAM		AL_BIT(12)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_RX_RAM		AL_BIT(13)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_FB_LOW_RAM	AL_BIT(14)
#define AL_ADAPTER_APP_PARITY_STATUS_SATA_PORT_3_FB_HIGH_RAM	AL_BIT(15)
#define AL_ADAPTER_APP_PARITY_STATUS_DBG_AXI_UROB_SRAM_OUT	AL_BIT(30)
#define AL_ADAPTER_APP_PARITY_STATUS_DBG_AXI_UROB_SRAM_IN	AL_BIT(31)

/*
 * Generic control registers
 */

/* Control 0 */
#define AL_ADAPTER_GENERIC_CONTROL_0			0x1E0
/* Control 2 */
#define AL_ADAPTER_GENERIC_CONTROL_2			0x1E8
/* Control 3 */
#define AL_ADAPTER_GENERIC_CONTROL_3			0x1EC
/* Control 9 */
#define AL_ADAPTER_GENERIC_CONTROL_9			0x218
/* Control 10 */
#define AL_ADAPTER_GENERIC_CONTROL_10			0x21C
/* Control 11 */
#define AL_ADAPTER_GENERIC_CONTROL_11			0x220
/* Control 12 */
#define AL_ADAPTER_GENERIC_CONTROL_12			0x224
/* Control 13 */
#define AL_ADAPTER_GENERIC_CONTROL_13			0x228
/* Control 14 */
#define AL_ADAPTER_GENERIC_CONTROL_14			0x22C
/* Control 15 */
#define AL_ADAPTER_GENERIC_CONTROL_15			0x230
/* Control 16 */
#define AL_ADAPTER_GENERIC_CONTROL_16			0x234
/* Control 17 */
#define AL_ADAPTER_GENERIC_CONTROL_17			0x238
/* Control 18 */
#define AL_ADAPTER_GENERIC_CONTROL_18			0x23C
/* Control 19 */
#define AL_ADAPTER_GENERIC_CONTROL_19			0x240

/* Enable clock gating */
#define AL_ADAPTER_GENERIC_CONTROL_0_CLK_GATE_EN	0x01
/* When set, all transactions through the PCI conf & mem BARs get timeout */
#define AL_ADAPTER_GENERIC_CONTROL_0_ADAPTER_DIS	0x40
#define AL_ADAPTER_GENERIC_CONTROL_0_ETH_RESET_1GMAC	AL_BIT(18)
#define AL_ADAPTER_GENERIC_CONTROL_0_ETH_RESET_1GMAC_ON_FLR	AL_BIT(26)

/*
 * SATA registers only
 */
/* Select 125MHz free running clock from IOFAB main PLL as SATA OOB clock
 * instead of using power management ref clock
 */
#define AL_ADAPTER_GENERIC_CONTROL_10_SATA_OOB_CLK_SEL	AL_BIT(26)
/* AXUSER selection and value per bit (1 = address, 0 = register) */
/* Rx */
#define AL_ADPTR_GEN_CTL_12_SATA_AWUSER_VAL_MASK	AL_FIELD_MASK(15, 0)
#define AL_ADPTR_GEN_CTL_12_SATA_AWUSER_VAL_SHIFT	0
#define AL_ADPTR_GEN_CTL_12_SATA_AWUSER_SEL_MASK	AL_FIELD_MASK(31, 16)
#define AL_ADPTR_GEN_CTL_12_SATA_AWUSER_SEL_SHIFT	16
/* Tx */
#define AL_ADPTR_GEN_CTL_13_SATA_ARUSER_VAL_MASK	AL_FIELD_MASK(15, 0)
#define AL_ADPTR_GEN_CTL_13_SATA_ARUSER_VAL_SHIFT	0
#define AL_ADPTR_GEN_CTL_13_SATA_ARUSER_SEL_MASK	AL_FIELD_MASK(31, 16)
#define AL_ADPTR_GEN_CTL_13_SATA_ARUSER_SEL_SHIFT	16
/* Central Target-ID enabler. If set, then each entry will be used as programmed */
#define AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_SEL		AL_BIT(0)
/* Allow access to store Target-ID values per entry */
#define AL_ADPTR_GEN_CTL_14_SATA_MSIX_TGTID_ACCESS_EN	AL_BIT(1)
/* Target-ID Address select */
/* Tx */
#define AL_ADPTR_GEN_CTL_14_SATA_VM_ARADDR_SEL_MASK	AL_FIELD_MASK(13, 8)
#define AL_ADPTR_GEN_CTL_14_SATA_VM_ARADDR_SEL_SHIFT	8
/* Rx */
#define AL_ADPTR_GEN_CTL_14_SATA_VM_AWADDR_SEL_MASK	AL_FIELD_MASK(21, 16)
#define AL_ADPTR_GEN_CTL_14_SATA_VM_AWADDR_SEL_SHIFT	16
/* Address Value */
/* Rx */
#define AL_ADPTR_GEN_CTL_15_SATA_VM_AWDDR_HI	AL_FIELD_MASK(31, 0)
/* Tx */
#define AL_ADPTR_GEN_CTL_16_SATA_VM_ARDDR_HI	AL_FIELD_MASK(31, 0)

/*
 * ROB registers
 */
/* Read ROB Enable, when disabled the read ROB is bypassed */
#define AL_ADPTR_GEN_CTL_19_READ_ROB_EN			AL_BIT(0)
/* Read force in-order of every read transaction */
#define AL_ADPTR_GEN_CTL_19_READ_ROB_FORCE_INORDER	AL_BIT(1)
/* Read software reset */
#define AL_ADPTR_GEN_CTL_19_READ_ROB_SW_RESET		AL_BIT(15)
/* Write ROB Enable, when disabled the Write ROB is bypassed */
#define AL_ADPTR_GEN_CTL_19_WRITE_ROB_EN		AL_BIT(16)
/* Write force in-order of every write transaction */
#define AL_ADPTR_GEN_CTL_19_WRITE_ROB_FORCE_INORDER	AL_BIT(17)
/* Write software reset */
#define AL_ADPTR_GEN_CTL_19_WRITE_ROB_SW_RESET		AL_BIT(31)

#ifdef __cplusplus
}
#endif

#endif
