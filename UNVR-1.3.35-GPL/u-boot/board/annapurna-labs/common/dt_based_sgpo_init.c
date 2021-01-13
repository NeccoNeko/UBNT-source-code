/**
 * board/annapurna-labs/common/dt_based_sgpo_init.c
 *
 * DT based SGPO initialization
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
#include <errno.h>
#include <common.h>
#include <libfdt_env.h>
#include <fdt_support.h>
#include <libfdt.h>
#include <fdtdec.h>
#include <asm/global_data.h>
#include <al_globals.h>
#include <al_board.h>
#include <al_hal_sgpo.h>
#include <al_hal_iomap.h>

DECLARE_GLOBAL_DATA_PTR;

struct sgpo_grp_cfg {
	enum al_sgpo_mode	mode_mask;
	unsigned int		init_val;
	unsigned int		invert_mask;
	unsigned int		stretch_mask;
	unsigned int		stretch_time_ms;
	unsigned int		blink_mask;
#if 0
	unsigned int		blink_high_time_ms;
	unsigned int		blink_low_time_ms;
#endif
};

struct sgpo_cfg {
	enum al_sgpo_group_mode group_mode;
	enum al_sgpo_sata_mode sata_mode;
	unsigned int clk_setup_time_ns;
	unsigned int update_freq_khz;
	unsigned int clk_freq_mhz;
	enum al_sgpo_blink_base_rate blink_base_rate;

	struct sgpo_grp_cfg grp_cfg[AL_SGPO_NUM_OF_GROUPS];
};

static int dt_based_sgpo_init_group(
	void			*fdt,
	struct sgpo_grp_cfg	*cfg,
	unsigned int		grp_idx)
{
	const struct fdt_property *fdt_prop;
	const u32 *cell;
	char path[80];
	int off;
	int len;

	debug("\tGroup %d:\n", grp_idx);

	sprintf(path, "/soc/board-cfg/sgpo_init/group%d", grp_idx);
	off = fdt_path_offset(fdt, path);
	if (off < 0) {
		printf("%s: sgpo initialization node not found!\n", __func__);
		return -EINVAL;
	}

	/* mode_mask */
	fdt_prop = fdt_get_property(fdt, off, "mode_mask", &len);
	if (!fdt_prop) {
		printf("%s: mode_mask node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->mode_mask = fdt32_to_cpu(cell[0]);
	debug("\t\tmode_mask: %02x\n", cfg->mode_mask);

	/* init_val */
	fdt_prop = fdt_get_property(fdt, off, "init_val", &len);
	if (!fdt_prop) {
		printf("%s: init_val node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->init_val = fdt32_to_cpu(cell[0]);
	debug("\t\tinit_val: %02x\n", cfg->init_val);

	/* invert_mask */
	fdt_prop = fdt_get_property(fdt, off, "invert_mask", &len);
	if (!fdt_prop) {
		printf("%s: invert_mask node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->invert_mask = fdt32_to_cpu(cell[0]);
	debug("\t\tinvert_mask: %02x\n", cfg->invert_mask);

	/* stretch_mask */
	fdt_prop = fdt_get_property(fdt, off, "stretch_mask", &len);
	if (!fdt_prop) {
		printf("%s: stretch_mask node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->stretch_mask = fdt32_to_cpu(cell[0]);
	debug("\t\tstretch_mask: %02x\n", cfg->stretch_mask);

	/* blink_mask */
	fdt_prop = fdt_get_property(fdt, off, "blink_mask", &len);
	if (!fdt_prop) {
		printf("%s: blink_mask node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->blink_mask = fdt32_to_cpu(cell[0]);
	debug("\t\tblink_mask: %02x\n", cfg->blink_mask);

	/* stretch time */
	fdt_prop = fdt_get_property(fdt, off, "stretch_time_ms", &len);
	if (!fdt_prop) {
		printf("%s: stretch_time_ms node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->stretch_time_ms = fdt32_to_cpu(cell[0]);
	debug("\t\tstretch_time_ms: %u\n", cfg->stretch_time_ms);

#if 0
	/* blink high time */
	fdt_prop = fdt_get_property(fdt, off, "blink_high_time_ms", &len);
	if (!fdt_prop) {
		printf("%s: blink_high_time_ms node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->blink_high_time_ms = fdt32_to_cpu(cell[0]);
	debug("\t\tblink_high_time_ms: %u\n", cfg->blink_high_time_ms);

	/* blink low time */
	fdt_prop = fdt_get_property(fdt, off, "blink_low_time_ms", &len);
	if (!fdt_prop) {
		printf("%s: blink_low_time_ms node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg->blink_low_time_ms = fdt32_to_cpu(cell[0]);
	debug("\t\tblink_low_time_ms: %u\n", cfg->blink_low_time_ms);
#endif

	return 0;
}

int dt_based_sgpo_init(
	void *fdt)
{
	const struct fdt_property *fdt_prop;
	const char *prop;
	const u32 *cell;
	char path[80];
	int off;
	int len;
	int err;
	int i;
	struct al_sgpo_if sgpo;
	struct sgpo_cfg cfg;
	const enum al_sgpo_group grp_map[AL_SGPO_NUM_OF_GROUPS] = {
		AL_SGPO_GROUP0,
		AL_SGPO_GROUP1,
		AL_SGPO_GROUP2,
		AL_SGPO_GROUP3,
	};

	debug("SGPO:\n");

	sprintf(path, "/soc/board-cfg/sgpo_init");
	off = fdt_path_offset(fdt, path);
	if (off < 0) {
		debug("%s: sgpo initialization node not found!\n", __func__);
		return 0;
	}

	/* Status */
	prop = (char *)fdt_getprop(fdt, off, "status", NULL);
	debug("\tstatus: %s\n", prop ? prop : "unknown - assuming enabled");
	if (prop && strcmp(prop, "enabled"))
		return 0;

	/* Group mode */
	prop = (char *)fdt_getprop(fdt, off, "group_mode", NULL);
	debug("\tgroup mode: %s\n", prop ? prop : "<unknown>");
	if (!prop) {
		printf("%s: group_mode node not found!\n", __func__);
		return -EINVAL;
	}

	if (!strcmp(prop, "one"))
		cfg.group_mode = AL_SGPO_ONE_GROUP;
	else if (!strcmp(prop, "two"))
		cfg.group_mode = AL_SGPO_TWO_GROUPS;
	else if (!strcmp(prop, "four"))
		cfg.group_mode = AL_SGPO_FOUR_GROUPS;
	else {
		printf("%s: invalid group mode (%s)!\n", __func__, prop);
		return -EINVAL;
	}

	/* SATA mode */
	prop = (char *)fdt_getprop(fdt, off, "sata_mode", NULL);
	debug("\tSATA mode: %s\n", prop ? prop : "<unknown>");
	if (!prop) {
		printf("%s: sata_mode node not found!\n", __func__);
		return -EINVAL;
	}

	if (!strcmp(prop, "active"))
		cfg.sata_mode = AL_SGPO_SATA_ACTIVE;
	else if (!strcmp(prop, "presence"))
		cfg.sata_mode = AL_SGPO_SATA_PRESENCE;
	else if (!strcmp(prop, "active-presence"))
		cfg.sata_mode = AL_SGPO_SATA_ACTTIVE_PRESENCE;
	else {
		printf("%s: invalid sata mode (%s)!\n", __func__, prop);
		return -EINVAL;
	}

	/* Timing */
	sprintf(path, "/soc/board-cfg/sgpo_init/timing/");
	off = fdt_path_offset(fdt, path);

	fdt_prop = fdt_get_property(fdt, off, "clk_setup_time_ns", &len);
	if (!fdt_prop) {
		printf("%s: clk_setup_time_ns node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg.clk_setup_time_ns = fdt32_to_cpu(cell[0]);
	debug("\tclk_setup_time_ns: %u\n", cfg.clk_setup_time_ns);

	fdt_prop = fdt_get_property(fdt, off, "update_freq_khz", &len);
	if (!fdt_prop) {
		printf("%s: update_freq_khz node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg.update_freq_khz = fdt32_to_cpu(cell[0]);
	debug("\tupdate_freq_khz: %u\n", cfg.update_freq_khz);

	fdt_prop = fdt_get_property(fdt, off, "clk_freq_mhz", &len);
	if (!fdt_prop) {
		printf("%s: clk_freq_mhz node not found!\n", __func__);
		return -EINVAL;
	}
	cell = (u32 *)fdt_prop->data;
	cfg.clk_freq_mhz = fdt32_to_cpu(cell[0]);
	debug("\tclk_freq_mhz: %u\n", cfg.clk_freq_mhz);

	prop = (char *)fdt_getprop(fdt, off, "blink_rate", NULL);
	debug("\tblink rate: %s\n", prop ? prop : "<unknown>");
	if (!prop) {
		printf("%s: blink_rate node not found!\n", __func__);
		return -EINVAL;
	}

	if (!strcmp(prop, "normal"))
		cfg.blink_base_rate = AL_SGPO_BLINK_BASE_RATE_NORMAL;
	else if (!strcmp(prop, "fast"))
		cfg.blink_base_rate = AL_SGPO_BLINK_BASE_RATE_FAST;
	else {
		printf("%s: invalid blink rate (%s)!\n", __func__, prop);
		return -EINVAL;
	}

	/* Per group configuration */
	for (i = 0; i < AL_SGPO_NUM_OF_GROUPS; i++) {
		err = dt_based_sgpo_init_group(fdt, &cfg.grp_cfg[i], i);
		if (err) {
			printf("%s: dt_based_sgpo_init_group(%d) failed!\n", __func__, i);
			return err;
		}
	}

	/* HW initialization */
	err = al_sgpo_handle_init(&sgpo, (void __iomem *)AL_PBS_SGPO_BASE,
		al_globals.bootstraps.sb_clk_freq / 1000000);
	if (err) {
		printf("al_sgpo_handle_init failed!\n");
		return err;
	}

	err = al_sgpo_hw_init(&sgpo, cfg.blink_base_rate);
	if (err) {
		printf("al_sgpo_hw_init failed!\n");
		return err;
	}

	al_sgpo_group_mode_set(&sgpo, cfg.group_mode);
	al_sgpo_sata_mode_set(&sgpo, cfg.sata_mode);
	al_sgpo_clk_rise_time_set(&sgpo, cfg.clk_setup_time_ns, AL_SGPO_NS);
	al_sgpo_upd_freq_set(&sgpo, cfg.update_freq_khz, AL_SGPO_KHZ);
	al_sgpo_clk_freq_set(&sgpo, cfg.clk_freq_mhz * 1000, AL_SGPO_KHZ);
	for (i = 0; i < AL_SGPO_NUM_OF_GROUPS; i++) {
		struct sgpo_grp_cfg *grp_cfg = &cfg.grp_cfg[i];
		enum al_sgpo_group grp = grp_map[i];
		unsigned int bit_idx;

		al_sgpo_stretch_len_set(&sgpo, grp, grp_cfg->stretch_time_ms, AL_SGPO_MS);
#if 0
		al_sgpo_blink_high_set(&sgpo, grp, grp_cfg->blink_high_time_ms, AL_SGPO_MS);
		al_sgpo_blink_low_set(&sgpo, grp, grp_cfg->blink_low_time_ms, AL_SGPO_MS);
#endif

		for (bit_idx = 0; bit_idx < AL_SGPO_NUM_OF_PINS_IN_GROUP; bit_idx++) {
			unsigned int id = (i * AL_SGPO_NUM_OF_PINS_IN_GROUP) + bit_idx;

			al_sgpo_pin_mode_set(&sgpo, id,
				(grp_cfg->mode_mask & AL_BIT(bit_idx)) ? AL_SGPO_USER : AL_SGPO_HW);
			al_sgpo_user_val_set(
				&sgpo, id, !!(grp_cfg->init_val & AL_BIT(bit_idx)));
			al_sgpo_invert_mode_en(
				&sgpo, id, !!(grp_cfg->invert_mask & AL_BIT(bit_idx)));
			al_sgpo_stretch_mode_en(
				&sgpo, id, !!(grp_cfg->stretch_mask & AL_BIT(bit_idx)));
			al_sgpo_blink_mode_en(
				&sgpo, id, !!(grp_cfg->blink_mask & AL_BIT(bit_idx)));
		}
	}

	return 0;
}
