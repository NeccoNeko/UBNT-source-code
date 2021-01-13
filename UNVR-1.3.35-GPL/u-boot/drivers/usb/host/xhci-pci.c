/*
 * Copyright (c) 2015, Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 * All rights reserved.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <errno.h>
#include <pci.h>
#ifdef CONFIG_DM_USB
#include <dm.h>
#endif /* CONFIG_DM_USB */
#include <usb.h>

#include "xhci.h"

/*
 * Create the appropriate control structures to manage a new XHCI host
 * controller.
 */
int xhci_hcd_init(int index, struct xhci_hccr **ret_hccr,
		  struct xhci_hcor **ret_hcor)
{
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	pci_dev_t pdev;
	uint32_t cmd;
	int len;
	uint16_t word;

	pdev = pci_find_class(PCI_CLASS_SERIAL_USB_XHCI, index);
	if (pdev < 0) {
		printf("XHCI host controller not found\n");
		return -1;
	}

	hccr = (struct xhci_hccr *)pci_map_bar(pdev,
			PCI_BASE_ADDRESS_0, PCI_REGION_MEM);
	/* Enable Bus Mastering and memory region */
	pci_write_config_word(pdev, PCI_COMMAND,
			      PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
	/* Check if mem accesses and Bus Mastering are enabled. */
	pci_read_config_word(pdev, PCI_COMMAND, &word);
	if ( !(word & PCI_COMMAND_MEMORY) ||
	    (!(word & PCI_COMMAND_MASTER))) {
		printf(
			"%s: Failed enabling mem access or bus "
			"mastering!\n",
			__func__);

		debug("%s: PCI command: %04x\n", __func__, word);

		return -EIO;
	}

	len = HC_LENGTH(xhci_readl(&hccr->cr_capbase));
	hcor = (struct xhci_hcor *)((unsigned long)hccr + len);

	debug("XHCI-PCI init hccr 0x%lx and hcor 0x%lx hc_length %d\n",
	      (unsigned long)hccr, (unsigned long)hcor, len);

	*ret_hccr = hccr;
	*ret_hcor = hcor;

	/* enable busmaster */
	pci_read_config_dword(pdev, PCI_COMMAND, &cmd);
	cmd |= PCI_COMMAND_MASTER;
	pci_write_config_dword(pdev, PCI_COMMAND, cmd);

	return 0;
}

/*
 * Destroy the appropriate control structures corresponding * to the XHCI host
 * controller
 */
void xhci_hcd_stop(int index)
{
}

#ifdef CONFIG_DM_USB
static int xhci_pci_probe(struct udevice *dev)
{
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct xhci_ctrl *ctrl = dev->priv;
	int ret, index = 0;

	if (xhci_hcd_init(index, &hccr, (struct xhci_hcor **)&hcor) != 0)
		return -ENODEV;

	if (xhci_reset(hcor) != 0) {
		return -ENODEV;
	}

	ctrl->hccr = hccr;
	ctrl->hcor = hcor;

	ret = xhci_lowlevel_init(ctrl);

	return ret;
}

U_BOOT_DRIVER(usb_xhci) = {
	.name   = "xhci_pci",
	.id     = UCLASS_USB,
	.probe = xhci_pci_probe,
	.ops    = &xhci_usb_ops,
	.platdata_auto_alloc_size = 0,
	.priv_auto_alloc_size = sizeof(struct xhci_ctrl),
	.flags  = DM_FLAG_ALLOC_PRIV_DMA,
};

U_BOOT_DEVICE(usb_xhci) = {
  .name = "xhci_pci",
  .platdata = (void *)1,
};
#endif /* CONFIG_DM_USB */
