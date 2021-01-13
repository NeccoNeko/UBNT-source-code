 /*
   * Copyright (C) 2016 Annapurna Labs Ltd.
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

#include "al_hal_sbus.h"
#include "al_hal_iomap.h"

static int handle_init(
	const char		*sbus_id_str,
	struct al_sbus_obj	*handle)
{
	if (!strcmp(sbus_id_str, "tc-main")) {
		al_sbus_handle_init(handle, (void *)AL_PBS_SBUS_MASTER_BASE);
	} else {
		printf("Invalid complex \"%s\"!\n", sbus_id_str);
		return -EINVAL;
	}

	return 0;
}

static int do_sbus_wr(
	struct al_sbus_obj *handle, int argc, char *const argv[])
{
	const char *receiver_addr_str;
	const char *data_addr_str;
	const char *data_str;

	unsigned int receiver_addr;
	unsigned int data_addr;
	unsigned int data;

	int err;

	if (argc != 4) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	receiver_addr_str = argv[1];
	data_addr_str = argv[2];
	data_str = argv[3];

	receiver_addr = simple_strtoul(receiver_addr_str, NULL, 0);
	data_addr = simple_strtoul(data_addr_str, NULL, 0);
	data = simple_strtoul(data_str, NULL, 0);

	err = al_sbus_write(handle, receiver_addr, data_addr, data, AL_TRUE);

	printf(
		"SBUS write(0x%x, 0x%x, 0x%x) - %s\n",
		receiver_addr, data_addr, data, err ? "Failed" : "Ok");

	return 0;
}

static int do_sbus_rd(
	struct al_sbus_obj *handle, int argc, char *const argv[])
{
	const char *receiver_addr_str;
	const char *data_addr_str;

	unsigned int receiver_addr;
	unsigned int data_addr;
	unsigned int data;

	int err;

	if (argc != 3) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	receiver_addr_str = argv[1];
	data_addr_str = argv[2];

	receiver_addr = simple_strtoul(receiver_addr_str, NULL, 0);
	data_addr = simple_strtoul(data_addr_str, NULL, 0);

	err = al_sbus_read(handle, receiver_addr, data_addr, &data);
	if (err)
		printf("SBUS read (0x%x, 0x%x) - Failed\n", receiver_addr, data_addr);
	else
		printf("SBUS read (0x%x, 0x%x) - 0x%x\n", receiver_addr, data_addr, data);

	return 0;
}

static int do_sbus_fw_init(
	struct al_sbus_obj *handle, int argc, char *const argv[])
{
	const char *fw_addr_str;
	const char *fw_size_str;

	unsigned int fw_addr;
	unsigned int fw_size;

	int err;

	if (argc != 3) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	fw_addr_str = argv[1];
	fw_size_str = argv[2];

	fw_addr = simple_strtoul(fw_addr_str, NULL, 0);
	fw_size = simple_strtoul(fw_size_str, NULL, 0);

	if (fw_size % sizeof(uint32_t)) {
		printf("Firmware size must be a multiple of 4 bytes!\n\n");
		return -1;
	}

	err = al_sbus_master_fw_init(
		handle, (const uint32_t *)(uintptr_t)fw_addr, fw_size / sizeof(uint32_t));
	if (err)
		printf("SBUS master FW init failed!\n");
	else
		printf("SBUS master FW init done\n");

	return 0;
}

static int do_sbus_fw_rev(
	struct al_sbus_obj *handle, int argc, char *const argv[])
{
	if (argc != 1) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	al_sbus_master_fw_revision_print(handle);

	return 0;
}

static int do_sbus_int(
	struct al_sbus_obj *handle, int argc, char *const argv[])
{
	const char *code_str;
	const char *data_str;

	unsigned int code;
	unsigned int data;

	unsigned int res_status;
	unsigned int res_data;

	int err;

	if (argc != 3) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	code_str = argv[1];
	data_str = argv[2];

	code = simple_strtoul(code_str, NULL, 0);
	data = simple_strtoul(data_str, NULL, 0);

	err = al_sbus_master_interrupt(handle, code, data, &res_status, &res_data);
	if (err)
		printf("SBUS master interrupt %04x, %04x failed!\n", code, data);
	else
		printf(
			"SBUS master interrupt %04x, %04x: status = %04x, data = %04x\n",
			code, data, res_status, res_data);

	return 0;
}

int do_sbus(
	cmd_tbl_t *cmdtp,
	int flag,
	int argc,
	char *const argv[])
{
	const char *op;
	const char *sbus_id_str;

	struct al_sbus_obj handle;

	if (argc < 3) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	op = argv[1];
	sbus_id_str = argv[2];

	if (handle_init(sbus_id_str, &handle)) {
		printf("Syntax error!\n\n");
		return -1;
	}

	argv += 2;
	argc -= 2;

	if (!strcmp(op, "wr")) {
		return do_sbus_wr(&handle, argc, argv);
	} else if (!strcmp(op, "rd")) {
		return do_sbus_rd(&handle, argc, argv);
	} else if (!strcmp(op, "fw-init")) {
		return do_sbus_fw_init(&handle, argc, argv);
	} else if (!strcmp(op, "fw-rev")) {
		return do_sbus_fw_rev(&handle, argc, argv);
	} else if (!strcmp(op, "int")) {
		return do_sbus_int(&handle, argc, argv);
	} else {
		printf("Syntax error - invalid op!\n\n");
		return -1;
	}

	return 0;
}

U_BOOT_CMD(
	sbus, 6, 0, do_sbus,
	"SBUS debug",
	"sbus op <arg1> <arg2> ...\n\n"
	" - op - operation:\n"
	"        wr - SBUS write\n"
	"             arg1 - sbus_id (see below)\n"
	"             arg2 - receiver address\n"
	"             arg3 - data address\n"
	"             arg4 - data\n"
	"\n"
	"        rd - SBUS read\n"
	"             arg1 - sbus_id (see below)\n"
	"             arg2 - receiver address\n"
	"             arg3 - data address\n"
	"\n"
	"        fw-init - FW init\n"
	"             arg1 - sbus_id (see below)\n"
	"             arg2 - FW location in memory\n"
	"             arg3 - FW size\n"
	"\n"
	"        fw-rev - FW revision printout\n"
	"             arg1 - sbus_id (see below)\n"
	"\n"
	"        int - SBUS master interrupt\n"
	"             arg1 - sbus_id (see below)\n"
	"             arg2 - interrupt code\n"
	"             arg3 - interrupt data\n"
	"\n"
	" - sbus_id - the SBUS of interest: tc-main\n"
	"\n\n"
);

