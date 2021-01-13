KBUILD_CFLAGS_KERNEL += -I$(srctree)/include/soc/alpine/ -I$(srctree)/drivers/dma/al/
KBUILD_CFLAGS_MODULE += -I$(srctree)/include/soc/alpine/

obj-y	+= al_hal_udma_config.o
obj-y	+= al_hal_udma_main.o
obj-y	+= al_hal_m2m_udma.o
obj-y   += al_hal_udma_debug.o
obj-y   += al_hal_udma_iofic.o
obj-y   += al_hal_udma_fast.o
obj-y   += al_hal_iofic.o
obj-y   += al_hal_ssm.o
obj-y   += al_hal_unit_adapter.o
obj-y   += al_hal_sgpo.o
obj-y   += al_hal_serdes_hssp.o
obj-y	+= al_hal_sys_fabric_utils.o
obj-y	+= al_hal_pbs_utils.o

obj-$(CONFIG_PCI)	+= al_hal_pcie.o
obj-$(CONFIG_PCI)	+= al_hal_pcie_interrupts.o
obj-$(CONFIG_EDAC_AL_MC)+= al_hal_ddr.o

obj-y	+= al_serdes.o
obj-y	+= alpine_serdes.o
obj-y	+= al_hal_exports.o
obj-y	+= alpine_fabric.o
obj-y	+= alpine_group_lm.o

obj-$(CONFIG_AL_TDM_TEST_MODULE) += al_tdm_test.o
al_tdm_test-objs := al_hal_tdm.o

