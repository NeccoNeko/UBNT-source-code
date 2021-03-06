#ifndef __EEPROM_PER_DEVICE_LAYOUT_H__
#define __EEPROM_PER_DEVICE_LAYOUT_H__

#define EEPROM_PER_DEVICE_LAYOUT_MAGIC_NUM				0xCC

#define EEPROM_PER_DEVICE_LAYOUT_OFFSET					0x600

#define EEPROM_PER_DEVICE_LAYOUT_SIZE					0x40

#define EEPROM_PER_DEVICE_LAYOUT_MAGIC_NUM_OFFSET			0x00

#define EEPROM_PER_DEVICE_LAYOUT_EEPROM_REV_ID				0x01

#define EEPROM_PER_DEVICE_LAYOUT_MAC_ADDRESS_OFFSET(idx)		\
	(0x02 + ((idx) * 0x06))

#define EEPROM_PER_DEVICE_LAYOUT_PCB_REV_ID				0x1a

#define EEPROM_PER_DEVICE_LAYOUT_BOARD_SN				0x21

#define EEPROM_PER_DEVICE_LAYOUT_RESERVED				0x31

#endif

