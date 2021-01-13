#ifndef __AL_ALPINE_V2_64_UBNT_UDC_H
#define __AL_ALPINE_V2_64_UBNT_UDC_H

#include "alpine_db_common.h"

#define AL_DEV_ID					AL_DEV_ID_ALPINE_V2
#define AL_DEV_REV_ID				0

#define GICD_BASE					AL_NB_GIC_DIST_BASE_MAIN
#define COUNTER_FREQUENCY			50000000
#define CPU_RELEASE_ADDR			0

#define CONFIG_SYS_TEXT_BASE		0x01100000
#define CONFIG_SYS_INIT_SP_ADDR		0x01800000
#define AL_RECOVERY_OFFSET			0x01800000

#define CONFIG_LOADADDR				0x08000000
#define AL_ENV_LOADADDR_PAYLOAD		"0x08000004"
#define AL_ENV_LOADADDR_DT			"0x04078000"
#define AL_ENV_LOADADDR_ROOTFS_CHK	"0x07000000"
#define AL_ENV_FDT_HIGH				"fdt_high=0xffffffffffffffff\0"
#define CONFIG_BOOTCOMMAND		\
	"cp.b $fdtaddr $loadaddr_dt 7ffc; fdt addr $loadaddr_dt; run bootext4"

#define CONFIG_AL_PCIE_3

#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT			"ALPINE_UBNT_UDC> "
#endif

#define CONFIG_AR8033_SEL_1P8

/* Customized default env for UDC-48 */
#ifdef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_EXTRA_ENV_SETTINGS
#endif

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"ethprime=al_eth1\0"						\
	"nfseth=eth1\0"							\
	"nfsrootdir="							\
		"/srv/root/\0"						\
	"autoload="							\
		"n\0"							\
	"fail="								\
		"echo Failed!; lcd_print \"Failed!\"\0"			\
	"hdroot="							\
		"/dev/sda1\0"						\
	"rootargsnfs="							\
		"setenv rootargs " CONFIG_AL_ROOTARGS_NFS "\0"		\
	"rootargshd="							\
		"setenv rootargs " CONFIG_AL_ROOTARGS_HD "\0"		\
	"bootargsnfs="							\
		"run rootargsnfs; setenv bootargs $rootargs " CONFIG_AL_BOOTARGS_COMMON " $bootargsextra; printenv bootargs\0"	\
	"bootargshd="							\
		"run rootargshd; setenv bootargs $rootargs " CONFIG_AL_BOOTARGS_COMMON " $bootargsextra; printenv bootargs\0"	\
	"boottftp="							\
		"lcd_print \"Loading OS...\";"				\
		"tftpboot $loadaddr ${tftpdir}${dt_filename};"		\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"if test ${dt_is_from_toc} != 1; then; else "		\
			"flash_contents_obj_read_mem $loadaddr_dt $loadaddr;"	\
			"if test $? -ne 0; then run fail; exit; fi;"	\
		"fi;"							\
		"tftpboot $loadaddr ${tftpdir}uImage;"			\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"fdt addr $loadaddr_dt;"				\
		"setenv bootargs $rootargs " CONFIG_AL_BOOTARGS_COMMON " $bootargsextra;"	\
		"bootm $loadaddr - $fdtaddr;"				\
		"run fail; exit\0"					\
	"ext4dir=boot/\0"						\
	"ext4dev=0\0"							\
	"ext4part=1\0"							\
	"bootext4="							\
		"lcd_print \"Loading OS...\";"				\
		"scsi init; "						\
		"ext4load scsi ${ext4dev}:${ext4part} $loadaddr ${ext4dir}${dt_filename};"	\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"if test ${dt_is_from_toc} != 1; then; else "		\
			"flash_contents_obj_read_mem $loadaddr_dt $loadaddr;"	\
			"if test $? -ne 0; then run fail; exit; fi;"	\
		"fi;"							\
		"ext4load scsi ${ext4dev}:${ext4part} $loadaddr ${ext4dir}uImage;"	\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"fdt addr $loadaddr_dt;"				\
		"setenv bootargs $rootargs " CONFIG_AL_BOOTARGS_COMMON " $bootargsextra;"	\
		"bootm $loadaddr - $fdtaddr;"				\
		"run fail; exit\0"					\
	"bootupd="							\
		"lcd_print \"Updating al-boot\" \"to SPI\"; "		\
		"tftpboot ${loadaddr} ${tftpdir}boot.img; "		\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"sf probe; "						\
		"sf erase 0 +${filesize}; "				\
		"sf write ${loadaddr} 0 ${filesize}; "			\
		"echo bootupd done;"					\
		"echo Notice: Changes in default environment "		\
		"variables will only take effect once the;"		\
		"echo environment variables are deleted from "		\
		"flash using the 'delenv' script;"			\
		"lcd_print \"Done\"\0"					\
	"bootupdy="							\
		"lcd_print \"Updating al-boot\" \"to SPI\"; "		\
		"echo >> Use YModem to upload the boot image binary...;"	\
		"loady ${loadaddr};"					\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"sf probe; "						\
		"sf erase 0 +${filesize}; "				\
		"sf write ${loadaddr} 0 ${filesize}; "			\
		"echo bootupd done;"					\
		"echo Notice: Changes in default environment "		\
		"variables will only take effect once the;"		\
		"echo environment variables are deleted from "		\
		"flash using the 'delenv' script;"			\
		"lcd_print \"Done\"\0"					\
	"dtupd="							\
		"lcd_print \"Updating DT\" \"to SPI\"; "		\
		"tftpboot $loadaddr_dt ${tftpdir}${dt_filename};"	\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"if test ${dt_is_from_toc} != 1; then; else "		\
			"flash_contents_obj_read_mem $loadaddr $loadaddr_dt;"	\
			"if test $? -ne 0; then run fail; exit; fi;"	\
		"fi;"							\
		"sf probe; "						\
		"sf erase ${dt_location} +${filesize}; "		\
		"sf write ${loadaddr_dt} ${dt_location} ${filesize};"	\
		"echo dtupd done;"					\
		"lcd_print \"Done\"\0"					\
	"dtupdy="							\
		"lcd_print \"Updating DT\" \"to SPI\"; "		\
		"echo >> Use YModem to upload the device tree binary...;"	\
		"loady $loadaddr_dt;"				\
		"if test $? -ne 0; then run fail; exit; fi;"		\
		"if test ${dt_is_from_toc} != 1; then; else "		\
			"flash_contents_obj_read_mem $loadaddr $loadaddr_dt;"	\
			"if test $? -ne 0; then run fail; exit; fi;"	\
		"fi;"							\
		"sf probe; "						\
		"sf erase ${dt_location} +${filesize}; "		\
		"sf write ${loadaddr_dt} ${dt_location} ${filesize}; "\
		"echo dtupd done;"					\
		"lcd_print \"Done\"\0"					\
	"delenv="							\
		"lcd_print \"Deleting env...\"; "			\
		"sf probe; "						\
		"sf erase ${env_offset} +2000;"				\
		"if test -n ${env_offset_redund}; then "		\
			"sf erase ${env_offset_redund} +2000;"		\
		"fi;"							\
		"lcd_print \"Done\"\0"			\
	"skip_eth_halt=0\0"						\
	"loadaddr_payload=" AL_ENV_LOADADDR_PAYLOAD "\0"		\
	"loadaddr_dt=" AL_ENV_LOADADDR_DT "\0"					\
	"loadaddr_rootfs_chk=" AL_ENV_LOADADDR_ROOTFS_CHK "\0"				\
	"eepromupd="							\
		"confirm_msg \"Perform EEPROM update? [y/n] \";"	\
		"if test $? -ne 0; then exit; fi;"			\
		"tftpboot ${tftpdir}eeprom.bin;"			\
		"if test $? -ne 0; then exit; fi;"			\
		"i2c probe ${pld_i2c_addr};"				\
		"if test $? -ne 0; then exit; fi;"			\
		"i2c write $fileaddr ${pld_i2c_addr} 0.2 $filesize;"	\
		"if test $? -ne 0; then exit;fi;"			\
		"echo eepromupd done\0"					\
	"eepromupdy="							\
		"confirm_msg \"Perform EEPROM update? [y/n] \";"	\
		"if test $? -ne 0; then exit; fi;"			\
		"echo >> Use YModem to upload the EEPROM binary...;"	\
		"loady $loadaddr;"					\
		"if test $? -ne 0; then exit; fi;"			\
		"i2c probe ${pld_i2c_addr};"				\
		"if test $? -ne 0; then exit; fi;"			\
		"i2c write $loadaddr ${pld_i2c_addr} 0.2 $filesize;"	\
		"if test $? -ne 0; then exit;fi;"			\
		"echo eepromupdy done\0"				\
	CONFIG_CVOS_ENV_COMMANDS					\
	AL_ENV_FDT_HIGH								\
	""


#endif /* __AL_ALPINE_V2_64_UBNT_UDC_H */
