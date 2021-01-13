#ifndef __AL_ALPINE_V2_64_UBNT_U6WALL_H
#define __AL_ALPINE_V2_64_UBNT_U6WALL_H

#include "alpine_db_common.h"

#define AL_DEV_ID			AL_DEV_ID_ALPINE_V2
#define AL_DEV_REV_ID			0

#define GICD_BASE			AL_NB_GIC_DIST_BASE_MAIN
#define COUNTER_FREQUENCY		50000000
#define CPU_RELEASE_ADDR		0

#define CONFIG_SYS_TEXT_BASE		0x01100000
#define CONFIG_SYS_INIT_SP_ADDR		0x01800000
#define AL_RECOVERY_OFFSET		0x01800000

#define CONFIG_LOADADDR			0x08000000
#define AL_ENV_LOADADDR_PAYLOAD		"0x08000004"
#define AL_ENV_LOADADDR_DT		"0x04078000"
#define AL_ENV_LOADADDR_ROOTFS_CHK	"0x07000000"
#define AL_ENV_FDT_HIGH			"fdt_high=0xffffffffffffffff\0"
#define CONFIG_BOOTCOMMAND		\
	"cp.b $fdtaddr $loadaddr_dt 7ffc; fdt addr $loadaddr_dt; run bootemmcdual"

#define CONFIG_AL_PCIE_3

#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT              "ALPINE_UBNT_U6WALL> "
#endif

#undef CONFIG_AL_NAND_ENV_COMMANDS

#define CONFIG_AR8033_SEL_1P8

#define CONFIG_AL_NAND_ENV_COMMANDS					\
	"nand_pt_addr_al_boot=0x0\0"					\
	"nand_pt_size_al_boot=0x00400000\0"				\
	"nand_pt_addr_kernel=0x00400000\0"				\
	"nand_pt_size_kernel=0x01000000\0"				\
	"nand_pt_addr_fs=0x01300000\0"					\
	"nand_pt_size_fs=0x1e600000\0"					\
	"nand_pt_desc_kernel_1=Test kernel A\0"				\
	"nand_pt_addr_kernel_1=0x00400000\0"				\
	"nand_pt_desc_kernel_2=Test kernel B\0"				\
	"nand_pt_addr_kernel_2=0x00a00000\0"

#endif /* __AL_ALPINE_V2_64_UBNT_U6WALL_H */
