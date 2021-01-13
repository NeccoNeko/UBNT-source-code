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

#include "al_globals.h"

#define MAX_COMPLEX_NAME_STR_LEN	20

static char default_cmplx_str[MAX_COMPLEX_NAME_STR_LEN] = "tc-main";
static int default_lane_idx;

static int lane_handle_get(
	const char			*cmplx_str,
	const char			*lane_str,
	struct al_serdes_grp_obj	**handle)
{
	struct al_serdes_grp_obj *lanes;
	unsigned int num_lanes;
	int lane_idx;

	if (!strcmp(cmplx_str, "default"))
		cmplx_str = default_cmplx_str;

	if (!strcmp(cmplx_str, "tc-main")) {
		lanes = al_globals.srds_cmplx_tc_main_lanes;
		num_lanes = ARRAY_SIZE(al_globals.srds_cmplx_tc_main_lanes);
	} else {
		printf("Invalid complex \"%s\"!\n", cmplx_str);
		return -EINVAL;
	}

	if (!strcmp(lane_str, "default"))
		lane_idx = default_lane_idx;
	else
		lane_idx = simple_strtoul(lane_str, NULL, 0);

	if ((lane_idx < 0) || (lane_idx > num_lanes)) {
		printf("Invalid lane %d!\n", lane_idx);
		return -EINVAL;
	}

	*handle = &lanes[lane_idx];

	return 0;
}

static int do_serdes_default_show(void)
{
	printf("Complex: %s, Lane: %u\n", default_cmplx_str, default_lane_idx);

	return 0;
}

static int do_serdes_default_set(const char *cmplx_str, const char *lane_str)
{
	strncpy(default_cmplx_str, cmplx_str, sizeof(default_cmplx_str) - 1);
	default_lane_idx = simple_strtoul(lane_str, NULL, 0);

	return 0;
}

static int parse_lb_mode(
	const char		*lb_mode_str,
	enum al_serdes_lb_mode	*lb_mode)
{
	if (!strcmp(lb_mode_str, "0")) {
		*lb_mode = AL_SRDS_LB_MODE_OFF;
	} else if (!strcmp(lb_mode_str, "1")) {
		*lb_mode = AL_SRDS_LB_MODE_PMA_SERIAL_TX_IO_TO_RX_IO;
	} else if (!strcmp(lb_mode_str, "2")) {
		*lb_mode = AL_SRDS_LB_MODE_PMA_PARALLEL_RX_TO_TX;
	} else {
		printf("Syntax error - invalid loopback mode!\n\n");
		return -1;
	}

	return 0;
}

static int parse_bist_pattern(
	const char			*bist_pattern_str,
	enum al_serdes_bist_pattern	*bist_pattern)
{
	if (!strcmp(bist_pattern_str, "0")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS7;
	} else if (!strcmp(bist_pattern_str, "1")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS9;
	} else if (!strcmp(bist_pattern_str, "2")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS11;
	} else if (!strcmp(bist_pattern_str, "3")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS15;
	} else if (!strcmp(bist_pattern_str, "4")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS23;
	} else if (!strcmp(bist_pattern_str, "5")) {
		*bist_pattern = AL_SRDS_BIST_PATTERN_PRBS31;
	} else {
		printf("Syntax error - invalid BIST pattern!\n\n");
		return -1;
	}

	return 0;
}

static int do_serdes_pcie_int_en(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	handle->pcs_interrupt_enable_set(handle, AL_FALSE);

	return 0;
}

static int do_serdes_pcie_int_dis(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	handle->pcs_interrupt_enable_set(handle, AL_TRUE);

	return 0;
}

static int do_serdes_rx_equal(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	handle->rx_equalization(handle, 0);

	return 0;
}

static int do_serdes_core_int(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	const char *code_str;
	const char *data_str;

	unsigned int code;
	unsigned int data;
	unsigned int ret_data;

	if (argc != 3) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	code_str = argv[1];
	data_str = argv[2];

	code = simple_strtoul(code_str, NULL, 0);
	data = simple_strtoul(data_str, NULL, 0);

	ret_data = al_serdes_avg_core_interrupt(handle, code, data);

	printf("Core interrupt (0x%x, 0x%x) returned 0x%x\n\n", code, data, ret_data);

	return 0;
}

static int do_serdes_lb(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	const char *lb_mode_str;

	enum al_serdes_lb_mode lb_mode;

	if (argc != 2) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	lb_mode_str = argv[1];

	if (parse_lb_mode(lb_mode_str, &lb_mode))
		return -1;

	handle->loopback_control(handle, 0, lb_mode);

	return 0;
}

static int do_serdes_bist_pat(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	const char *bist_pattern_str;

	enum al_serdes_bist_pattern bist_pattern;

	if (argc != 2) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	bist_pattern_str = argv[1];

	if (parse_bist_pattern(bist_pattern_str, &bist_pattern))
		return -1;

	handle->bist_pattern_select(handle, bist_pattern, NULL);

	return 0;
}

static int do_serdes_bist_tx_en(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	const char *en_str;

	int en;

	if (argc != 2) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	en_str = argv[1];

	en = simple_strtol(en_str, NULL, 10);

	handle->bist_tx_enable(handle, 0, en);

	return 0;
}

static int do_serdes_bist_tx_err(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	if (argc != 1) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	handle->bist_tx_err_inject(handle);

	return 0;
}

static int do_serdes_bist_rx_en(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	const char *en_str;

	int en;

	if (argc != 2) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	en_str = argv[1];

	en = simple_strtol(en_str, NULL, 10);

	handle->bist_rx_enable(handle, 0, en);

	return 0;
}

static int do_serdes_bist_rx_stat(
	struct al_serdes_grp_obj *handle, int argc, char *const argv[])
{
	al_bool is_locked;
	al_bool err_cnt_overflow;
	uint32_t err_cnt;

	if (argc != 1) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	handle->bist_rx_status(handle, 0, &is_locked, &err_cnt_overflow, &err_cnt);

	printf(
		"BIST RX status: is_locked=%d, err_cnt_overflow=%d err_cnt=%d\n",
		is_locked,
		err_cnt_overflow,
		err_cnt);

	return 0;
}

int do_serdes(
	cmd_tbl_t *cmdtp,
	int flag,
	int argc,
	char *const argv[])
{
	const char *op;
	const char *cmplx_str = "default";
	const char *lane_str = "default";

	struct al_serdes_grp_obj *handle;
	int default_show = (argc >= 2) && (!strcmp(argv[1], "default_show"));
	int argc_min = default_show ? 2 : 4;

	if (argc < argc_min) {
		printf("Syntax error - invalid number of parameters!\n\n");
		return -1;
	}

	op = argv[1];

	if (argc >= 4) {
		cmplx_str = argv[2];
		lane_str = argv[3];
		argv += 2;
		argc -= 2;
	}

	if (lane_handle_get(cmplx_str, lane_str, &handle)) {
		printf("Syntax error!\n\n");
		return -1;
	}

	argv++;
	argc--;

	if (default_show) {
		return do_serdes_default_show();
	} else if (!strcmp(op, "default_set")) {
		return do_serdes_default_set(cmplx_str, lane_str);
	} else if (!strcmp(op, "pcie_int_en")) {
		return do_serdes_pcie_int_en(handle, argc, argv);
	} else if (!strcmp(op, "pcie_int_dis")) {
		return do_serdes_pcie_int_dis(handle, argc, argv);
	} else if (!strcmp(op, "rx_equal")) {
		return do_serdes_rx_equal(handle, argc, argv);
	} else if (!strcmp(op, "int")) {
		return do_serdes_core_int(handle, argc, argv);
	} else if (!strcmp(op, "lb")) {
		return do_serdes_lb(handle, argc, argv);
	} else if (!strcmp(op, "bist_pat")) {
		return do_serdes_bist_pat(handle, argc, argv);
	} else if (!strcmp(op, "bist_tx_en")) {
		return do_serdes_bist_tx_en(handle, argc, argv);
	} else if (!strcmp(op, "bist_tx_err")) {
		return do_serdes_bist_tx_err(handle, argc, argv);
	} else if (!strcmp(op, "bist_rx_en")) {
		return do_serdes_bist_rx_en(handle, argc, argv);
	} else if (!strcmp(op, "bist_rx_stat")) {
		return do_serdes_bist_rx_stat(handle, argc, argv);
	} else {
		printf("Syntax error - invalid op!\n\n");
		return -1;
	}

	return 0;
}

U_BOOT_CMD(
	serdes, 6, 0, do_serdes,
	"SerDes debug",
	"serdes op <arg1> <arg2> ...\n\n"
	" - op - operation:\n"
	"        default_show - Show default SerDes complex and lane\n"
	"\n"
	"        default_set - Set default SerDes complex and lane\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane\n"
	"\n"
	"        pcie_int_en - PCIe interrupt enable\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane num (or 'default')\n"
	"\n"
	"        pcie_int_dis - PCIe interrupt disable\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane num (or 'default')\n"
	"\n"
	"        rx_equal - Rx equalization\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane num (or 'default')\n"
	"\n"
	"        int - SerDes core interrupt\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane num (or 'default')\n"
	"             arg3 - code\n"
	"             arg4 - data\n"
	"\n"
	"        lb - loopback control\n"
	"             arg1 - cmplx (see below)\n"
	"             arg2 - lane num (or 'default')\n"
	"             arg3 - loopback mode:\n"
	"                    0 - No loopback\n"
	"                    1 - TX driver IO signal to the RX IO pins\n"
	"                    2 - PMA RX lane data ports to TX lane data ports\n"
	"\n"
	"        bist_pat - BIST pattern selection\n"
	"                   arg1 - cmplx (see below)\n"
	"                   arg2 - lane num (or 'default')\n"
	"                   arg3 - pattern:\n"
	"                          0 - PRBS 2^7\n"
	"                          1 - PRBS 2^9\n"
	"                          2 - PRBS 2^11\n"
	"                          3 - PRBS 2^15\n"
	"                          4 - PRBS 2^23\n"
	"                          5 - PRBS 2^31\n"
	"\n"
	"        bist_tx_en - BIST TX enable/disable\n"
	"                     arg1 - cmplx (see below)\n"
	"                     arg2 - lane num (or 'default')\n"
	"                     arg3 - 1 - enable, 0 - disable\n"
	"\n"
	"        bist_tx_err - BIST TX single bit error injection\n"
	"                      arg1 - cmplx (see below)\n"
	"                      arg2 - lane num (or 'default')\n"
	"\n"
	"        bist_rx_en - BIST RX enable/disable\n"
	"                     arg1 - cmplx (see below)\n"
	"                     arg2 - lane num (or 'default')\n"
	"                     arg3 - 1 - enable, 0 - disable\n"
	"\n"
	"        bist_rx_stat - BIST RX status information\n"
	"                       arg1 - cmplx (see below)\n"
	"                       arg2 - lane num (or 'default')\n"
	"\n"
	" - cmplx - the SerDes complex: tc-main, default\n"
	"\n\n"
);

