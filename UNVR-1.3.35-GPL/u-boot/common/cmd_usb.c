/*
 * (C) Copyright 2001
 * Denis Peter, MPL AG Switzerland
 *
 * Adapted for U-Boot driver model
 * (C) Copyright 2015 Google, Inc
 *
 * Most of this source has been derived from the Linux USB
 * project.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <part.h>
#include <usb.h>

#ifdef CONFIG_USB_STORAGE
static int usb_stor_curr_dev = -1; /* current device */
#endif
#ifdef CONFIG_USB_HOST_ETHER
static int usb_ether_curr_dev = -1; /* current ethernet device */
#endif

/* some display routines (info command) */
static char *usb_get_class_desc(unsigned char dclass)
{
	switch (dclass) {
	case USB_CLASS_PER_INTERFACE:
		return "See Interface";
	case USB_CLASS_AUDIO:
		return "Audio";
	case USB_CLASS_COMM:
		return "Communication";
	case USB_CLASS_HID:
		return "Human Interface";
	case USB_CLASS_PRINTER:
		return "Printer";
	case USB_CLASS_MASS_STORAGE:
		return "Mass Storage";
	case USB_CLASS_HUB:
		return "Hub";
	case USB_CLASS_DATA:
		return "CDC Data";
	case USB_CLASS_VENDOR_SPEC:
		return "Vendor specific";
	default:
		return "";
	}
}

static void usb_display_class_sub(unsigned char dclass, unsigned char subclass,
				  unsigned char proto)
{
	switch (dclass) {
	case USB_CLASS_PER_INTERFACE:
		printf("See Interface");
		break;
	case USB_CLASS_HID:
		printf("Human Interface, Subclass: ");
		switch (subclass) {
		case USB_SUB_HID_NONE:
			printf("None");
			break;
		case USB_SUB_HID_BOOT:
			printf("Boot ");
			switch (proto) {
			case USB_PROT_HID_NONE:
				printf("None");
				break;
			case USB_PROT_HID_KEYBOARD:
				printf("Keyboard");
				break;
			case USB_PROT_HID_MOUSE:
				printf("Mouse");
				break;
			default:
				printf("reserved");
				break;
			}
			break;
		default:
			printf("reserved");
			break;
		}
		break;
	case USB_CLASS_MASS_STORAGE:
		printf("Mass Storage, ");
		switch (subclass) {
		case US_SC_RBC:
			printf("RBC ");
			break;
		case US_SC_8020:
			printf("SFF-8020i (ATAPI)");
			break;
		case US_SC_QIC:
			printf("QIC-157 (Tape)");
			break;
		case US_SC_UFI:
			printf("UFI");
			break;
		case US_SC_8070:
			printf("SFF-8070");
			break;
		case US_SC_SCSI:
			printf("Transp. SCSI");
			break;
		default:
			printf("reserved");
			break;
		}
		printf(", ");
		switch (proto) {
		case US_PR_CB:
			printf("Command/Bulk");
			break;
		case US_PR_CBI:
			printf("Command/Bulk/Int");
			break;
		case US_PR_BULK:
			printf("Bulk only");
			break;
		default:
			printf("reserved");
			break;
		}
		break;
	default:
		printf("%s", usb_get_class_desc(dclass));
		break;
	}
}

static void usb_display_string(struct usb_device *dev, int index)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, buffer, 256);

	if (index != 0) {
		if (usb_string(dev, index, &buffer[0], 256) > 0)
			printf("String: \"%s\"", buffer);
	}
}

static void usb_display_desc(struct usb_device *dev)
{
	if (dev->descriptor.bDescriptorType == USB_DT_DEVICE) {
		printf("%d: %s,  USB Revision %x.%x\n", dev->devnum,
		usb_get_class_desc(dev->config.if_desc[0].desc.bInterfaceClass),
				   (dev->descriptor.bcdUSB>>8) & 0xff,
				   dev->descriptor.bcdUSB & 0xff);

		if (strlen(dev->mf) || strlen(dev->prod) ||
		    strlen(dev->serial))
			printf(" - %s %s %s\n", dev->mf, dev->prod,
				dev->serial);
		if (dev->descriptor.bDeviceClass) {
			printf(" - Class: ");
			usb_display_class_sub(dev->descriptor.bDeviceClass,
					      dev->descriptor.bDeviceSubClass,
					      dev->descriptor.bDeviceProtocol);
			printf("\n");
		} else {
			printf(" - Class: (from Interface) %s\n",
			       usb_get_class_desc(
				dev->config.if_desc[0].desc.bInterfaceClass));
		}
		printf(" - PacketSize: %d  Configurations: %d\n",
			dev->descriptor.bMaxPacketSize0,
			dev->descriptor.bNumConfigurations);
		printf(" - Vendor: 0x%04x  Product 0x%04x Version %d.%d\n",
			dev->descriptor.idVendor, dev->descriptor.idProduct,
			(dev->descriptor.bcdDevice>>8) & 0xff,
			dev->descriptor.bcdDevice & 0xff);
	}

}

static void usb_display_conf_desc(struct usb_config_descriptor *config,
				  struct usb_device *dev)
{
	printf("   Configuration: %d\n", config->bConfigurationValue);
	printf("   - Interfaces: %d %s%s%dmA\n", config->bNumInterfaces,
	       (config->bmAttributes & 0x40) ? "Self Powered " : "Bus Powered ",
	       (config->bmAttributes & 0x20) ? "Remote Wakeup " : "",
		config->bMaxPower*2);
	if (config->iConfiguration) {
		printf("   - ");
		usb_display_string(dev, config->iConfiguration);
		printf("\n");
	}
}

static void usb_display_if_desc(struct usb_interface_descriptor *ifdesc,
				struct usb_device *dev)
{
	printf("     Interface: %d\n", ifdesc->bInterfaceNumber);
	printf("     - Alternate Setting %d, Endpoints: %d\n",
		ifdesc->bAlternateSetting, ifdesc->bNumEndpoints);
	printf("     - Class ");
	usb_display_class_sub(ifdesc->bInterfaceClass,
		ifdesc->bInterfaceSubClass, ifdesc->bInterfaceProtocol);
	printf("\n");
	if (ifdesc->iInterface) {
		printf("     - ");
		usb_display_string(dev, ifdesc->iInterface);
		printf("\n");
	}
}

static void usb_display_ep_desc(struct usb_endpoint_descriptor *epdesc)
{
	printf("     - Endpoint %d %s ", epdesc->bEndpointAddress & 0xf,
		(epdesc->bEndpointAddress & 0x80) ? "In" : "Out");
	switch ((epdesc->bmAttributes & 0x03)) {
	case 0:
		printf("Control");
		break;
	case 1:
		printf("Isochronous");
		break;
	case 2:
		printf("Bulk");
		break;
	case 3:
		printf("Interrupt");
		break;
	}
	printf(" MaxPacket %d", get_unaligned(&epdesc->wMaxPacketSize));
	if ((epdesc->bmAttributes & 0x03) == 0x3)
		printf(" Interval %dms", epdesc->bInterval);
	printf("\n");
}

/* main routine to diasplay the configs, interfaces and endpoints */
static void usb_display_config(struct usb_device *dev)
{
	struct usb_config *config;
	struct usb_interface *ifdesc;
	struct usb_endpoint_descriptor *epdesc;
	int i, ii;

	config = &dev->config;
	usb_display_conf_desc(&config->desc, dev);
	for (i = 0; i < config->no_of_if; i++) {
		ifdesc = &config->if_desc[i];
		usb_display_if_desc(&ifdesc->desc, dev);
		for (ii = 0; ii < ifdesc->no_of_ep; ii++) {
			epdesc = &ifdesc->ep_desc[ii];
			usb_display_ep_desc(epdesc);
		}
	}
	printf("\n");
}

/*
 * With driver model this isn't right since we can have multiple controllers
 * and the device numbering starts at 1 on each bus.
 * TODO(sjg@chromium.org): Add a way to specify the controller/bus.
 */
static struct usb_device *usb_find_device(int devnum)
{
#ifdef CONFIG_DM_USB
	struct usb_device *udev;
	struct udevice *hub;
	struct uclass *uc;
	int ret;

	/* Device addresses start at 1 */
	devnum++;
	ret = uclass_get(UCLASS_USB_HUB, &uc);
	if (ret)
		return NULL;

	uclass_foreach_dev(hub, uc) {
		struct udevice *dev;

		if (!device_active(hub))
			continue;
		udev = dev_get_parentdata(hub);
		if (udev->devnum == devnum)
			return udev;

		for (device_find_first_child(hub, &dev);
		     dev;
		     device_find_next_child(&dev)) {
			if (!device_active(hub))
				continue;

			udev = dev_get_parentdata(dev);
			if (udev->devnum == devnum)
				return udev;
		}
	}
#else
	struct usb_device *udev;
	int d;

	for (d = 0; d < USB_MAX_DEVICE; d++) {
		udev = usb_get_dev_index(d);
		if (udev == NULL)
			return NULL;
		if (udev->devnum == devnum)
			return udev;
	}
#endif

	return NULL;
}

static inline char *portspeed(int speed)
{
	char *speed_str;

	switch (speed) {
	case USB_SPEED_SUPER:
		speed_str = "5 Gb/s";
		break;
	case USB_SPEED_HIGH:
		speed_str = "480 Mb/s";
		break;
	case USB_SPEED_LOW:
		speed_str = "1.5 Mb/s";
		break;
	default:
		speed_str = "12 Mb/s";
		break;
	}

	return speed_str;
}

/* shows the device tree recursively */
static void usb_show_tree_graph(struct usb_device *dev, char *pre)
{
	int index;
	int has_child, last_child;

	index = strlen(pre);
	printf(" %s", pre);
#ifdef CONFIG_DM_USB
	has_child = device_has_active_children(dev->dev);
#else
	/* check if the device has connected children */
	int i;

	has_child = 0;
	for (i = 0; i < dev->maxchild; i++) {
		if (dev->children[i] != NULL)
			has_child = 1;
	}
#endif
	/* check if we are the last one */
#ifdef CONFIG_DM_USB
	last_child = device_is_last_sibling(dev->dev);
#else
	last_child = (dev->parent != NULL);
#endif
	if (last_child) {
#ifndef CONFIG_DM_USB
		for (i = 0; i < dev->parent->maxchild; i++) {
			/* search for children */
			if (dev->parent->children[i] == dev) {
				/* found our pointer, see if we have a
				 * little sister
				 */
				while (i++ < dev->parent->maxchild) {
					if (dev->parent->children[i] != NULL) {
						/* found a sister */
						last_child = 0;
						break;
					} /* if */
				} /* while */
			} /* device found */
		} /* for all children of the parent */
#endif
		printf("\b+-");
		/* correct last child */
		if (last_child && index)
			pre[index-1] = ' ';
	} /* if not root hub */
	else
		printf(" ");
	printf("%d ", dev->devnum);
	pre[index++] = ' ';
	pre[index++] = has_child ? '|' : ' ';
	pre[index] = 0;
	printf(" %s (%s, %dmA)\n", usb_get_class_desc(
					dev->config.if_desc[0].desc.bInterfaceClass),
					portspeed(dev->speed),
					dev->config.desc.bMaxPower * 2);
	if (strlen(dev->mf) || strlen(dev->prod) || strlen(dev->serial))
		printf(" %s  %s %s %s\n", pre, dev->mf, dev->prod, dev->serial);
	printf(" %s\n", pre);
#ifdef CONFIG_DM_USB
	struct udevice *child;

	for (device_find_first_child(dev->dev, &child);
	     child;
	     device_find_next_child(&child)) {
		struct usb_device *udev;

		if (!device_active(child))
			continue;

		udev = dev_get_parentdata(child);

		/* Ignore emulators, we only want real devices */
		if (device_get_uclass_id(child) != UCLASS_USB_EMUL) {
			usb_show_tree_graph(udev, pre);
			pre[index] = 0;
		}
	}
#else
	if (dev->maxchild > 0) {
		for (i = 0; i < dev->maxchild; i++) {
			if (dev->children[i] != NULL) {
				usb_show_tree_graph(dev->children[i], pre);
				pre[index] = 0;
			}
		}
	}
#endif
}

/* main routine for the tree command */
static void usb_show_tree(struct usb_device *dev)
{
	char preamble[32];

	memset(preamble, '\0', sizeof(preamble));
	usb_show_tree_graph(dev, &preamble[0]);
}

static int usb_test(struct usb_device *dev, int port, char* arg)
{
	int mode;

	if (port > dev->maxchild) {
		printf("Device is no hub or does not have %d ports.\n", port);
		return 1;
	}

	switch (arg[0]) {
	case 'J':
	case 'j':
		printf("Setting Test_J mode");
		mode = USB_TEST_MODE_J;
		break;
	case 'K':
	case 'k':
		printf("Setting Test_K mode");
		mode = USB_TEST_MODE_K;
		break;
	case 'S':
	case 's':
		printf("Setting Test_SE0_NAK mode");
		mode = USB_TEST_MODE_SE0_NAK;
		break;
	case 'P':
	case 'p':
		printf("Setting Test_Packet mode");
		mode = USB_TEST_MODE_PACKET;
		break;
	case 'F':
	case 'f':
		printf("Setting Test_Force_Enable mode");
		mode = USB_TEST_MODE_FORCE_ENABLE;
		break;
	default:
		printf("Unrecognized test mode: %s\nAvailable modes: "
		       "J, K, S[E0_NAK], P[acket], F[orce_Enable]\n", arg);
		return 1;
	}

	if (port)
		printf(" on downstream facing port %d...\n", port);
	else
		printf(" on upstream facing port...\n");

	if (usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_REQ_SET_FEATURE,
			    port ? USB_RT_PORT : USB_RECIP_DEVICE,
			    port ? USB_PORT_FEAT_TEST : USB_FEAT_TEST,
			    (mode << 8) | port,
			    NULL, 0, USB_CNTL_TIMEOUT) == -1) {
		printf("Error during SET_FEATURE.\n");
		return 1;
	} else {
		printf("Test mode successfully set. Use 'usb start' "
		       "to return to normal operation.\n");
		return 0;
	}
}


#define BD_ADDR_0_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            24
#define BD_ADDR_1_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            23
#define BD_ADDR_2_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            21
#define BD_ADDR_3_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            17
#define BD_ADDR_4_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            20
#define BD_ADDR_5_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD            19

#define BCCMD_SET_BD_ADDR_CMD_INDEX                          1
#define BCCMD_RESET_CSR_CMD_INDEX                            2
#define BLE_HCI_LE_SET_ADVERTISING_DATA_INDEX                3
#define BLE_HCI_LE_SET_SCAN_RESPONSE_DATA_INDEX              4

#define BLE_UUID_OFFSET_IN_ADVERTISING_DATA                    6
#define BLE_SERIAL_NUMBER_STR_IN_ADVERTISING_DATA             26
#define BLE_SHORTENED_LOCAL_NAME_OFFSET_IN_SCAN_RESPONSE_DATA  3
#define BLE_BT_MAC_OFFSET_IN_SCAN_RESPONSE_DATA               17

/* UUID of UNVR */
/* 9fb9ae81-e7fb-49b2-b7f8-dfef4e09e3f4 */
static const uint8_t UUID[] =
	{0xf4, 0xe3, 0x09, 0x4e, 0xef, 0xdf, 0xf8, 0xb7, 0xb2, 0x49, 0xfb, 0xe7, 0x81, 0xae, 0xb9, 0x9f};

typedef struct BLE_hci_command {
	const uint8_t opcode[2];
	const uint8_t size;
	uint8_t command[32];
} ble_hci_command_t;

static ble_hci_command_t BLE_hci_commands[] = {
	/*CSR command - bccmd -d hci0 psset -r to set BD addr*/
	{.opcode= {0x0,0xfc}, .size = 0x13, .command = {0xc2,0x0,0x0,0x9,0x0,0x0,0x0,0x6,0x30,0x0,0x0,0x1,0x0,0x8,0x0,0x0,0x0,0x0,0x0}},
	 /*Write new BD addr: MAC[0]=command[24], MAC[1]=command[23],
	* MAC[2]=command[21], MAC[3]=command[17], MAC[4]=command[20],
	* MAC[5]=command[19]*/
	{.opcode= {0x0,0xfc}, .size = 0x19, .command = {0xc2,0x2,0x0,0xc,0x0,0x1,0x0,0x3,0x70,0x0,0x0,0x1,0x0,0x4,0x0,0x8,0x0,0x33,0x0,0x55,0x44,0x22,0x0,0x11,0x5}},
	/*Reset CSR chip - USB  will re-connect*/
	{.opcode= {0x0,0xfc}, .size = 0x13, .command = {0xc2,0x2,0x0,0x9,0x0,0x2,0x0,0x2,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}},
	/*End - bccmd -d hci0 psset -r */

	/*LE_Set_Advertising_Data*/
	//f9f0fc99-0867-427c-a231-6a028a104efd - default uuid
	{.opcode= {0x8,0x20}, .size = 0x20, .command = {0x15,0x2,0x1,0x2,0x11,0x6,0xfd,0x4e,0x10,0x8a,0x02,0x6a,0x31,0xa2,0x7c,0x42,0x67,0x08,0x99,0xfc,0xf0,0xf9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0}},

	/*LE_Set_Scan_Responce_Data*/
	{.opcode= {0x9,0x20}, .size = 0x20, .command = { 0x16,0x0b,0x08,0x55,0x4e,0x56,0x52,0x00,0x00,0x00,0x00,0x00,0x00, \
			0x09,0x16,0x2a,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
			0x00,0x00,0x00}},

	/*LE_Set_Advertising_Parameters*/
	{.opcode= {0x6,0x20}, .size = 0xf, .command = {0xa0,0x00,0xa0,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00}},

	/*LE_Set_Advertise_Enable*/
	{.opcode= {0xa,0x20}, .size = 0x1, .command = {0x1}},
};
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#endif

#define MAX_USB_PORT_NUM 4
static struct usb_device *found_csr8811(void)
{
	struct usb_device *udev;
	int i;

	for(i = 0; i < MAX_USB_PORT_NUM; i++) {
		udev = usb_find_device(i);
		if(udev == NULL)
			continue;

		printf("usb vid=0x%x\n", udev->descriptor.idVendor);
		if(udev->descriptor.idVendor == 0x0a12) {
			printf("Found csr8811\n");
			return udev;
		}
	}
}

static int usb_set_bt(void)
{
	int data_len = 0;
	int return_val= 0;
	int expected_data_count = 0;
	//struct usb_device *udev = usb_get_dev_index(2); /*BT device is 2 which is count from 0,1,2*/
	struct usb_device *udev;
	int cnt;
	int i;

	udev = found_csr8811();

	if (udev == NULL) {
		printf("UDEV = NULL\n");
		return -2;
	}

	uchar *eth_mac=eth_get_dev()->enetaddr;
	/* btaddress should be different from ethaddr */
	*(eth_mac+5) = *(eth_mac+5) + 3;
	printf("MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",*(eth_mac+0),*(eth_mac+1),*(eth_mac+2),*(eth_mac+3),*(eth_mac+4),*(eth_mac+5));

	for (cnt=0; cnt<ARRAY_SIZE(BLE_hci_commands); cnt++){
		ble_hci_command_t *cmd = &BLE_hci_commands[cnt];
#if 1
		switch(cnt)
		{
			case BCCMD_SET_BD_ADDR_CMD_INDEX:
			{
				cmd->command[BD_ADDR_0_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+0);
				cmd->command[BD_ADDR_1_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+1);
				cmd->command[BD_ADDR_2_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+2);
				cmd->command[BD_ADDR_3_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+3);
				cmd->command[BD_ADDR_4_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+4);
				cmd->command[BD_ADDR_5_OFFSET_IN_BCCMD_SET_BD_ADDR_CMD] = *(eth_mac+5);
				break;
			}
			case BCCMD_RESET_CSR_CMD_INDEX:
			{
				data_len = sizeof(BLE_hci_commands[cnt].opcode) + sizeof(BLE_hci_commands[cnt].size) + BLE_hci_commands[cnt].size;
				usb_control_msg(udev, usb_sndctrlpipe(udev, 0),0, 0x20, 0, 0, cmd, data_len,USB_CNTL_TIMEOUT);
				printf("0x%x 0x%x 0x%x ", cmd->opcode[0], cmd->opcode[1], cmd->size);
				for (i = 0;i < cmd->size; i++) {
					printf("0x%x ", cmd->command[i]);
				}
				printf("\n");
				printf("return_val: %d, cmd: 0x%x, 0x%x\n", return_val, cmd->opcode[0], cmd->opcode[1]);

				usb_stop();
				usb_init();
				//udev = usb_get_dev_index(2); /*BT device is 2 which is count from 0,1,2*/
				udev = found_csr8811();
				if (udev == NULL) {
					printf("UDEV = NULL\n");
					return -2;
				}
				mdelay(100);
				continue;
			}
			case BLE_HCI_LE_SET_ADVERTISING_DATA_INDEX:
			{
				memcpy(&cmd->command[BLE_UUID_OFFSET_IN_ADVERTISING_DATA],
							UUID, ARRAY_SIZE(UUID));
				break;
			}
			case BLE_HCI_LE_SET_SCAN_RESPONSE_DATA_INDEX:
			{
				char board_name[32];
				strcpy(board_name, "UNVR");
				memcpy(&cmd->command[BLE_SHORTENED_LOCAL_NAME_OFFSET_IN_SCAN_RESPONSE_DATA],
					board_name, strlen(board_name));
				memcpy(&cmd->command[BLE_BT_MAC_OFFSET_IN_SCAN_RESPONSE_DATA],
					eth_mac, 6);
				break;
			}
			default:
				break;
		}
#endif
		data_len = sizeof(BLE_hci_commands[cnt].opcode) + sizeof(BLE_hci_commands[cnt].size) + BLE_hci_commands[cnt].size;
			expected_data_count += data_len;
		printf("0x%x 0x%x 0x%x ", cmd->opcode[0], cmd->opcode[1], cmd->size);
		for (i = 0;i < cmd->size; i++) {
			printf("0x%x ", cmd->command[i]);
		}
		printf("\n");
		return_val += usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			0, 0x20, 0, 0, cmd, data_len,
			USB_CNTL_TIMEOUT);
		printf("return_val: %d, cmd: 0x%x, 0x%x\n", return_val, cmd->opcode[0], cmd->opcode[1]);
	}
	if (expected_data_count == return_val)
		return 0;

	printf("expected: %d, ret: %d\n", expected_data_count, return_val);
	return -1;
}

/******************************************************************************
 * usb boot command intepreter. Derived from diskboot
 */
#ifdef CONFIG_USB_STORAGE
static int do_usbboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return common_diskboot(cmdtp, "usb", argc, argv);
}
#endif /* CONFIG_USB_STORAGE */

static int do_usb_stop_keyboard(int force)
{
#ifdef CONFIG_USB_KEYBOARD
	if (usb_kbd_deregister(force) != 0) {
		printf("USB not stopped: usbkbd still using USB\n");
		return 1;
	}
#endif
	return 0;
}

static void do_usb_start(void)
{
	bootstage_mark_name(BOOTSTAGE_ID_USB_START, "usb_start");

	if (usb_init() < 0)
		return;

	/* Driver model will probe the devices as they are found */
#ifndef CONFIG_DM_USB
#ifdef CONFIG_USB_STORAGE
	/* try to recognize storage devices immediately */
	usb_stor_curr_dev = usb_stor_scan(1);
#endif
#endif
#ifdef CONFIG_USB_HOST_ETHER
	/* try to recognize ethernet devices immediately */
	usb_ether_curr_dev = usb_host_eth_scan(1);
#endif
#ifdef CONFIG_USB_KEYBOARD
	drv_usb_kbd_init();
#endif
}

#ifdef CONFIG_DM_USB
static void show_info(struct udevice *dev)
{
	struct udevice *child;
	struct usb_device *udev;

	udev = dev_get_parentdata(dev);
	usb_display_desc(udev);
	usb_display_config(udev);
	for (device_find_first_child(dev, &child);
	     child;
	     device_find_next_child(&child)) {
		if (device_active(child))
			show_info(child);
	}
}

static int usb_device_info(void)
{
	struct udevice *bus;

	for (uclass_first_device(UCLASS_USB, &bus);
	     bus;
	     uclass_next_device(&bus)) {
		struct udevice *hub;

		device_find_first_child(bus, &hub);
		if (device_get_uclass_id(hub) == UCLASS_USB_HUB &&
		    device_active(hub)) {
			show_info(hub);
		}
	}

	return 0;
}
#endif

/******************************************************************************
 * usb command intepreter
 */
static int do_usb(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct usb_device *udev = NULL;
	int i;
	extern char usb_started;
#ifdef CONFIG_USB_STORAGE
	block_dev_desc_t *stor_dev;
#endif

	if (argc < 2)
		return CMD_RET_USAGE;

	if (strncmp(argv[1], "start", 5) == 0) {
		if (usb_started)
			return 0; /* Already started */
		printf("starting USB...\n");
		do_usb_start();
		return 0;
	}

	if (strncmp(argv[1], "reset", 5) == 0) {
		printf("resetting USB...\n");
		if (do_usb_stop_keyboard(1) != 0)
			return 1;
		usb_stop();
		do_usb_start();
		return 0;
	}
	if (strncmp(argv[1], "stop", 4) == 0) {
		if (argc != 2)
			console_assign(stdin, "serial");
		if (do_usb_stop_keyboard(0) != 0)
			return 1;
		printf("stopping USB..\n");
		usb_stop();
		return 0;
	}
	if (!usb_started) {
		printf("USB is stopped. Please issue 'usb start' first.\n");
		return 1;
	}
	if (strncmp(argv[1], "tree", 4) == 0) {
		puts("USB device tree:\n");
#ifdef CONFIG_DM_USB
		struct udevice *bus;

		for (uclass_first_device(UCLASS_USB, &bus);
		     bus;
		     uclass_next_device(&bus)) {
			struct usb_device *udev;
			struct udevice *hub;

			device_find_first_child(bus, &hub);
			if (device_get_uclass_id(hub) == UCLASS_USB_HUB &&
			    device_active(hub)) {
				udev = dev_get_parentdata(hub);
				usb_show_tree(udev);
			}
		}
#else
		for (i = 0; i < USB_MAX_DEVICE; i++) {
			udev = usb_get_dev_index(i);
			if (udev == NULL)
				break;
			if (udev->parent == NULL)
				usb_show_tree(udev);
		}
#endif
		return 0;
	}
	if (strncmp(argv[1], "inf", 3) == 0) {
		if (argc == 2) {
#ifdef CONFIG_DM_USB
			usb_device_info();
#else
			int d;
			for (d = 0; d < USB_MAX_DEVICE; d++) {
				udev = usb_get_dev_index(d);
				if (udev == NULL)
					break;
				usb_display_desc(udev);
				usb_display_config(udev);
			}
#endif
			return 0;
		} else {
			/*
			 * With driver model this isn't right since we can
			 * have multiple controllers and the device numbering
			 * starts at 1 on each bus.
			 */
			i = simple_strtoul(argv[2], NULL, 10);
			printf("config for device %d\n", i);
			udev = usb_find_device(i);
			if (udev == NULL) {
				printf("*** No device available ***\n");
				return 0;
			} else {
				usb_display_desc(udev);
				usb_display_config(udev);
			}
		}
		return 0;
	}
	if (strncmp(argv[1], "test", 4) == 0) {
		if (argc < 5)
			return CMD_RET_USAGE;
		i = simple_strtoul(argv[2], NULL, 10);
		udev = usb_find_device(i);
		if (udev == NULL) {
			printf("Device %d does not exist.\n", i);
			return 1;
		}
		i = simple_strtoul(argv[3], NULL, 10);
		return usb_test(udev, i, argv[4]);
	}
#ifdef CONFIG_USB_STORAGE
	if (strncmp(argv[1], "stor", 4) == 0)
		return usb_stor_info();

	if (strncmp(argv[1], "part", 4) == 0) {
		int devno, ok = 0;
		if (argc == 2) {
			for (devno = 0; ; ++devno) {
				stor_dev = usb_stor_get_dev(devno);
				if (stor_dev == NULL)
					break;
				if (stor_dev->type != DEV_TYPE_UNKNOWN) {
					ok++;
					if (devno)
						printf("\n");
					debug("print_part of %x\n", devno);
					print_part(stor_dev);
				}
			}
		} else {
			devno = simple_strtoul(argv[2], NULL, 16);
			stor_dev = usb_stor_get_dev(devno);
			if (stor_dev != NULL &&
			    stor_dev->type != DEV_TYPE_UNKNOWN) {
				ok++;
				debug("print_part of %x\n", devno);
				print_part(stor_dev);
			}
		}
		if (!ok) {
			printf("\nno USB devices available\n");
			return 1;
		}
		return 0;
	}
	if (strcmp(argv[1], "read") == 0) {
		if (usb_stor_curr_dev < 0) {
			printf("no current device selected\n");
			return 1;
		}
		if (argc == 5) {
			unsigned long addr = simple_strtoul(argv[2], NULL, 16);
			unsigned long blk  = simple_strtoul(argv[3], NULL, 16);
			unsigned long cnt  = simple_strtoul(argv[4], NULL, 16);
			unsigned long n;
			printf("\nUSB read: device %d block # %ld, count %ld"
				" ... ", usb_stor_curr_dev, blk, cnt);
			stor_dev = usb_stor_get_dev(usb_stor_curr_dev);
			n = stor_dev->block_read(usb_stor_curr_dev, blk, cnt,
						 (ulong *)addr);
			printf("%ld blocks read: %s\n", n,
				(n == cnt) ? "OK" : "ERROR");
			if (n == cnt)
				return 0;
			return 1;
		}
	}
	if (strcmp(argv[1], "write") == 0) {
		if (usb_stor_curr_dev < 0) {
			printf("no current device selected\n");
			return 1;
		}
		if (argc == 5) {
			unsigned long addr = simple_strtoul(argv[2], NULL, 16);
			unsigned long blk  = simple_strtoul(argv[3], NULL, 16);
			unsigned long cnt  = simple_strtoul(argv[4], NULL, 16);
			unsigned long n;
			printf("\nUSB write: device %d block # %ld, count %ld"
				" ... ", usb_stor_curr_dev, blk, cnt);
			stor_dev = usb_stor_get_dev(usb_stor_curr_dev);
			n = stor_dev->block_write(usb_stor_curr_dev, blk, cnt,
						(ulong *)addr);
			printf("%ld blocks write: %s\n", n,
				(n == cnt) ? "OK" : "ERROR");
			if (n == cnt)
				return 0;
			return 1;
		}
	}
	if (strncmp(argv[1], "dev", 3) == 0) {
		if (argc == 3) {
			int dev = (int)simple_strtoul(argv[2], NULL, 10);
			printf("\nUSB device %d: ", dev);
			stor_dev = usb_stor_get_dev(dev);
			if (stor_dev == NULL) {
				printf("unknown device\n");
				return 1;
			}
			printf("\n    Device %d: ", dev);
			dev_print(stor_dev);
			if (stor_dev->type == DEV_TYPE_UNKNOWN)
				return 1;
			usb_stor_curr_dev = dev;
			printf("... is now current device\n");
			return 0;
		} else {
			printf("\nUSB device %d: ", usb_stor_curr_dev);
			stor_dev = usb_stor_get_dev(usb_stor_curr_dev);
			dev_print(stor_dev);
			if (stor_dev->type == DEV_TYPE_UNKNOWN)
				return 1;
			return 0;
		}
		return 0;
	}

#endif /* CONFIG_USB_STORAGE */
	if (strncmp(argv[1], "bt", 2) == 0) {
		if(argc == 2)
			return usb_set_bt();

		return 0;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(
	usb,	5,	1,	do_usb,
	"USB sub-system",
	"start - start (scan) USB controller\n"
	"usb reset - reset (rescan) USB controller\n"
	"usb stop [f] - stop USB [f]=force stop\n"
	"usb tree - show USB device tree\n"
	"usb info [dev] - show available USB devices\n"
	"usb test [dev] [port] [mode] - set USB 2.0 test mode\n"
	"    (specify port 0 to indicate the device's upstream port)\n"
	"    Available modes: J, K, S[E0_NAK], P[acket], F[orce_Enable]\n"
#ifdef CONFIG_USB_STORAGE
	"usb storage - show details of USB storage devices\n"
	"usb dev [dev] - show or set current USB storage device\n"
	"usb part [dev] - print partition table of one or all USB storage"
	"    devices\n"
	"usb read addr blk# cnt - read `cnt' blocks starting at block `blk#'\n"
	"    to memory address `addr'\n"
	"usb write addr blk# cnt - write `cnt' blocks starting at block `blk#'\n"
	"    from memory address `addr'"
#endif /* CONFIG_USB_STORAGE */
);


#ifdef CONFIG_USB_STORAGE
U_BOOT_CMD(
	usbboot,	3,	1,	do_usbboot,
	"boot from USB device",
	"loadAddr dev:part"
);
#endif /* CONFIG_USB_STORAGE */
