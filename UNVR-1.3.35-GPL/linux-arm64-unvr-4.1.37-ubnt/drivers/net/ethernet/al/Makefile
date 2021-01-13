KBUILD_CFLAGS_KERNEL += -I$(srctree)/include/soc/alpine/ -I$(srctree)/drivers/net/ethernet/al/

obj-$(CONFIG_NET_AL_ETH) += al_eth_drv.o
al_eth_drv-objs := al_eth.o al_eth_sysfs.o al_hal_eth_main.o al_hal_eth_kr.o al_init_eth_kr.o al_init_eth_lm.o al_hal_serdes_25g.o al_eth_group_lm.o kcompat.o

