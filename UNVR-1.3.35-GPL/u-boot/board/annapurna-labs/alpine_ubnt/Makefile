-include $(HAL_TOP)/file_list_base.mk

COMMON = ../common

ccflags-y += -Iarch/$(CONFIG_SYS_ARCH)/include/asm/arch-$(CONFIG_SYS_SOC)/
ccflags-y += -Iboard/$(CONFIG_SYS_VENDOR)/common/
ccflags-y += $(HAL_USER_INCLUDE_PATH) $(HAL_DRIVER_INCLUDE_PATH) $(HAL_INIT_INCLUDE_PATH)
ccflags-$(CONFIG_ALPINE) += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V1)
ccflags-$(CONFIG_ALPINE_V2_32) += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V2)
ccflags-$(CONFIG_ALPINE_V2_64) += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V2)
ccflags-$(CONFIG_ALPINE_V3_64) += $(HAL_PLATFORM_INCLUDE_PATH_ALPINE_V3)
ccflags-y += -include config.h

obj-y += board.o
obj-y += lcd.o
obj-y += cmd_lcd.o
obj-y += $(COMMON)/cmd_flash_contents.o
obj-y += $(COMMON)/eeprom_per_device.o
obj-y += $(COMMON)/cmd_eeprom_per_device.o
obj-y += $(COMMON)/cmd_cpu_misc.o
obj-y += $(COMMON)/iodma.o
obj-y += $(COMMON)/cmd_thermal.o
obj-y += $(COMMON)/shared_params.o
obj-y += $(COMMON)/pll_init.o
obj-y += $(COMMON)/gpio_board_init.o
obj-y += $(COMMON)/al_board.o
obj-y += $(COMMON)/al_globals.o
obj-y += $(COMMON)/cmd_scripth.o
obj-y += $(COMMON)/cmd_dev_info.o
obj-y += $(COMMON)/cmd_muio_mux.o
obj-$(CONFIG_ALPINE)$(CONFIG_ALPINE_V2_32)$(CONFIG_ALPINE_V2_64) += $(COMMON)/cmd_serdes.o
obj-$(CONFIG_ALPINE_V3_64) += $(COMMON)/cmd_serdes_v3.o
obj-$(CONFIG_ALPINE_V3_64) += $(COMMON)/cmd_sbus.o
obj-y += $(COMMON)/cmd_iodma.o
obj-y += $(COMMON)/cmd_wfi.o
obj-y += $(COMMON)/cmd_ddr.o
obj-y += $(COMMON)/env_dyn.o
obj-y += $(COMMON)/cmd_confirm_msg.o
obj-y += $(COMMON)/cmd_eth.o
obj-y += $(COMMON)/cmd_dram_margins.o
obj-$(CONFIG_AL_TRACE) += $(COMMON)/al_serial_trace.o $(COMMON)/cmd_trace.o
obj-$(CONFIG_AL_NAND_FLASH) += $(COMMON)/al_nand.o $(COMMON)/early_nand.o
obj-$(CONFIG_AL_NAND_SIMULATE) += $(COMMON)/nand_simulation.o
obj-$(CONFIG_PCI) += $(COMMON)/pci.o $(COMMON)/pci_skip.o
obj-$(CONFIG_CMD_AL_PCI_EXT) += $(COMMON)/cmd_al_pcie_external.o
obj-$(CONFIG_ALPINE_V2_32)$(CONFIG_ALPINE_V2_64)$(CONFIG_ALPINE_V3_64) += $(COMMON)/dt_based_sgpo_init.o
obj-$(CONFIG_ALPINE) += $(COMMON)/dt_based_sgpo_init_stub.o
obj-$(CONFIG_ALPINE_V2_32)$(CONFIG_ALPINE_V2_64)$(CONFIG_ALPINE_V3_64) += $(COMMON)/cmd_sgpo.o

HAL_UNNECESSARY_SOURCES = \
	$(HAL_TOP)/drivers/eth_ddp/al_hal_eth_ddp.c \
	$(HAL_TOP)/drivers/ddr/al_hal_ddr_init.c \
	$(HAL_TOP)/drivers/ddr/al_hal_ddr_init_alpine_v1.c \
	$(HAL_TOP)/drivers/ddr/al_hal_ddr_init_alpine_v2.c \
	$(HAL_TOP)/drivers/ddr/al_hal_ddr_init_alpine_v3.c \
	$(HAL_TOP)/drivers/ddr/al_hal_ddr_init_alpine_v3_tc.c \
	$(HAL_TOP)/drivers/udma/al_hal_msg_ipc.c \
	$(HAL_TOP)/services/dram_margins/al_dram_margins.c \
	$(HAL_TOP)/services/sys_fabric/al_init_l2cache_aarch32.S \
	$(HAL_TOP)/services/sys_fabric/al_init_l2cache_aarch64.S \
	$(HAL_TOP)/services/stats/al_stats.c \
	$(HAL_TOP)/services/stats/al_stats_io_fabric.c \
	$(HAL_TOP)/services/stats/al_stats_pcie.c \
	$(HAL_TOP)/services/stats/al_stats_sys_fabric.c \
	$(HAL_TOP)/services/stats/al_stats_eth.c \
	$(HAL_TOP)/services/stats/al_stats_ssm.c \

HAL_ALL_SOURCES := $(filter-out $(HAL_UNNECESSARY_SOURCES), $(HAL_DRIVER_SOURCES) $(HAL_INIT_SOURCES_GENERIC))

HAL_DRIVER_OBJS = $(patsubst %.c,%.o,$(HAL_ALL_SOURCES))
obj-y += $(HAL_DRIVER_OBJS)
$(addprefix $(obj)/,$(HAL_DRIVER_OBJS)): $(obj)/$(HAL_TOP)/%.o: $(HAL_TOP)/%.c FORCE
	$(call cmd,force_checksrc)
	$(call if_changed_rule,cc_o_c)
