#!/bin/bash

ln -sfn ../../../../AL_ETH/linux/kernel/al_eth/Makefile_linux ./Makefile

COMMAND="SYM"

DIR=../../../../AL_ETH/linux/kernel/al_eth
FILES="al_eth.c	al_eth.h al_eth_sysfs.c al_eth_sysfs.h kcompat.c kcompat.h Kconfig RELEASE_NOTES"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

DIR=../../../../HAL/include/eth
FILES="al_hal_eth.h al_hal_eth_alu.h al_hal_eth_kr.h"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

DIR=../../../../HAL/drivers/eth
FILES="al_hal_an_lt_wrapper_regs.h al_hal_eth_ec_regs.h al_hal_eth_kr.c
       al_hal_eth_mac_regs.h al_hal_eth_main.c"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

DIR=../../../../HAL/include/serdes
FILES="al_hal_serdes_25g_regs.h"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

DIR=../../../../HAL/drivers/serdes
FILES="al_hal_serdes_25g.c al_hal_serdes_25g_internal_regs.h"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

DIR=../../../../HAL/services/eth
FILES="al_init_eth_kr.c al_init_eth_kr.h al_init_eth_lm.c al_init_eth_lm.h
       al_eth_group_lm.c al_eth_group_lm.h"
./al_eth_symlink_aux.sh "$DIR" "$FILES" "$COMMAND"

