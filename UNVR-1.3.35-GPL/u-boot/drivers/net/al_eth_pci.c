 /*
   * drivers/net/al_eth_pci.c
   *
   * Thie file contains Annapurna Labs Ethernet driver PCI handler
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
#include <common.h>
#include <pci.h>

#include <al_hal_eth.h>
#include "al_hal_iomap.h"
#include "al_eth.h"

static struct pci_device_id supported[] = {
	{ PCI_VENDOR_ID_ANNAPURNALABS, PCI_DEVICE_ID_AL_ETH },
	{ PCI_VENDOR_ID_ANNAPURNALABS, PCI_DEVICE_ID_AL_ETH_ADVANCED },
	{}
};

int al_eth_pci_probe(
	void)
{
	int err;
	int idx;
	int eth_idx_prev = -1;

	debug("%s()\n", __func__);

	for (idx = 0; ; idx++) {
		int devno;
		uintptr_t iob[6];
		uint16_t word;
		int eth_idx;
		uint8_t rev_id;

		/* Find PCI device(s) */
		devno = pci_find_devices(supported, idx);
		if (devno < 0) {
			debug(
				"%s: pci_find_devices found no more devices\n",
				__func__);
			break;
		}

		debug(
			"%s: pci_find_devices found %d:%d:%d\n",
			__func__,
			PCI_BUS(devno),
			PCI_DEV(devno),
			PCI_FUNC(devno));

		eth_idx = PCI_DEV(devno);

		/* Skip non existing devices while keeping the index */
		for (; (eth_idx_prev + 1) < eth_idx; eth_idx_prev++)
			eth_register(NULL);
		eth_idx_prev = eth_idx;

		if (!((1 << eth_idx) & AL_ETH_ENABLE_VECTOR)) {
			debug(
				"%s: skipping al_eth%d\n",
				__func__,
				eth_idx);
			continue;
		}

		/* Read out device ID and revision ID */
		pci_read_config_byte(devno, PCI_REVISION_ID, &rev_id);

		/* Read out all BARs */
		iob[0] = (uintptr_t)pci_map_bar(devno,
				PCI_BASE_ADDRESS_0, PCI_REGION_MEM);
		iob[2] = (uintptr_t)pci_map_bar(devno,
				PCI_BASE_ADDRESS_2, PCI_REGION_MEM);
		iob[4] = (uintptr_t)pci_map_bar(devno,
				PCI_BASE_ADDRESS_4, PCI_REGION_MEM);

		/* Enable Bus Mastering and memory region */
		pci_write_config_word(devno, PCI_COMMAND,
				PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

		/* Check if mem accesses and Bus Mastering are enabled. */
		pci_read_config_word(devno, PCI_COMMAND, &word);
		if (!(word & PCI_COMMAND_MEMORY) ||
				(!(word & PCI_COMMAND_MASTER))) {
			printf(
				"%s: Failed enabling mem access or bus "
				"mastering!\n",
				__func__);

			debug("%s: PCI command: %04x\n", __func__, word);

			return -EIO;
		}

		err = al_eth_register(
			eth_idx,
			devno,
			rev_id,
			(void __iomem *)iob[AL_ETH_UDMA_BAR],
			(void __iomem *)iob[AL_ETH_EC_BAR],
			(void __iomem *)iob[AL_ETH_MAC_BAR]);
		if (err) {
			printf("%s: al_eth_register failed!\n", __func__);
			return err;
		}
	}

	return 0;
}

