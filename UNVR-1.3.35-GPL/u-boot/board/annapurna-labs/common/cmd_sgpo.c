/**
 * board/annapurna-labs/common/cmd_sgpo.c
 *
 * Control SGPO pins on the fly
 *
 * Copyright (C) 2015 Annapurna Labs Ltd.
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
#include <command.h>
#include <errno.h>
#include <al_globals.h>
#include <al_hal_sgpo.h>

enum sgpo_cmd {
	SGPO_SET,
	SGPO_CLEAR,
	SGPO_USER,
	SGPO_HW,
};

static int do_sgpo(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct al_sgpo_if sgpo_if;
	unsigned int sgpo;
	enum sgpo_cmd sub_cmd;
	int set_value = 0;
	int value;
	int set_mode = 0;
	enum al_sgpo_mode mode;
	const char *str_cmd, *str_sgpo;
	int err;

	if (argc < 3)
		return CMD_RET_USAGE;

	str_cmd = argv[1];
	str_sgpo = argv[2];

	if (!str_sgpo)
		return CMD_RET_USAGE;

	/* parse the behavior */
	switch (*str_cmd) {
	case 's':
		sub_cmd = SGPO_SET;
		break;
	case 'c':
		sub_cmd = SGPO_CLEAR;
		break;
	case 'u':
		sub_cmd = SGPO_USER;
		break;
	case 'h':
		sub_cmd = SGPO_HW;
		break;
	default:
		return CMD_RET_USAGE;
	}

	/* turn the sgpo name into a sgpo number */
	sgpo = simple_strtoul(str_sgpo, NULL, 10);
	if (sgpo < 0)
		return CMD_RET_USAGE;

	/* finally, let's do it: set direction and exec command */
	switch (sub_cmd) {
	case SGPO_SET:
		value = 1;
		set_value = 1;
		break;
	case SGPO_CLEAR:
		value = 0;
		set_value = 1;
		break;
	case SGPO_USER:
		mode = AL_SGPO_USER;
		set_mode = 1;
		break;
	case SGPO_HW:
		mode = AL_SGPO_HW;
		set_mode = 1;
		break;
	default:
		return CMD_RET_USAGE;
	}

	err = al_sgpo_handle_init(&sgpo_if, (void __iomem *)AL_PBS_SGPO_BASE,
		al_globals.bootstraps.sb_clk_freq / 1000000);
	if (err) {
		printf("al_sgpo_handle_init failed!\n");
		return CMD_RET_FAILURE;
	}

	if (set_value) {
		al_sgpo_user_val_set(&sgpo_if, sgpo, value);
		printf("sgpo %i value is %d\n", sgpo, value);
	}

	if (set_mode) {
		al_sgpo_pin_mode_set(&sgpo_if, sgpo, mode);
		printf("sgpo %i mode is %s\n", sgpo, (mode == AL_SGPO_USER) ? "user" : "hw");
	}

	return 0;
}

U_BOOT_CMD(sgpo, 3, 0, do_sgpo,
	   "control SGPO pins",
	   "<set|clear|user|hw> <pin>\n"
	   "    - set/clear the specified pin\n"
	   "    - set specified pin mode to user/hw");
