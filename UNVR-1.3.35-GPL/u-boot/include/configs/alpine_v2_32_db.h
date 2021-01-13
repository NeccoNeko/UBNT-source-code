#ifndef __AL_ALPINE_V2_32_DB_H
#define __AL_ALPINE_V2_32_DB_H

#include "alpine_db_common.h"

#define AL_DEV_ID			AL_DEV_ID_ALPINE_V2
#define AL_DEV_REV_ID			0

#define CONFIG_SYS_THUMB_BUILD

#define CONFIG_SYS_TEXT_BASE		0x01100000
#define CONFIG_SYS_INIT_SP_ADDR		0x01800000
#define AL_RECOVERY_OFFSET		0x01800000

#define CONFIG_LOADADDR			0x08000000
#define AL_ENV_LOADADDR_PAYLOAD		"0x08000004"
#define AL_ENV_LOADADDR_DT		"0x04078000"
#define AL_ENV_LOADADDR_ROOTFS_CHK	"0x07000000"
#define AL_ENV_FDT_HIGH			"fdt_high=0xffffffffffffffff\0"
#define CONFIG_BOOTCOMMAND		\
	"cp.b $fdtaddr $loadaddr_dt 7ffc; fdt addr $loadaddr_dt; run bootargsnand; run bootnand"

#define CONFIG_AL_PCIE_3

#endif
