/*
   * board/annapurna-labs/common/cmd_thermal.c
   *
   * Thie file contains a U-Boot commands for using the thermal sensor.
   *
   * Copyright (C) 2013 Annapurna Labs Ltd.
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

#include "al_hal_thermal_sensor.h"
#include "al_hal_iomap.h"

#define TIMEOUT_MS	1000

#if (AL_DEV_ID == AL_DEV_ID_ALPINE_V3) && (AL_DEV_REV_ID == 0)
static struct al_sbus_obj sbus_handle;
#endif
static struct al_thermal_sensor_handle handle;

int do_thermal_init(
	cmd_tbl_t *cmdtp,
	int flag,
	int argc,
	char *const argv[])
{
	int timeout;

#if (AL_DEV_ID == AL_DEV_ID_ALPINE_V3) && (AL_DEV_REV_ID == 0)
	struct al_thermal_sensor_handle_init_params init_params = {
		.pbs_regs_base = (void __iomem *)AL_PBS_REGFILE_BASE,
		.sbus_obj = &sbus_handle,
		.sbus_slave_addr = AL_SBUS_ADDR_THERMAL_SENSOR,
	};

	al_sbus_handle_init(&sbus_handle, (void *)AL_PBS_SBUS_MASTER_BASE);
	al_thermal_sensor_handle_init_ex(&handle, &init_params);
#else
	int err;
	err = al_thermal_sensor_handle_init(&handle,
			(void __iomem *)AL_TEMP_SENSOR_BASE,
			(void __iomem *)AL_PBS_REGFILE_BASE);
	if (err) {
		printf("al_thermal_sensor_init failed!\n");
		return -1;
	}
#endif

	al_thermal_sensor_enable_set(&handle, 1);

	for (timeout = 0; timeout < TIMEOUT_MS; timeout++) {
		if (al_thermal_sensor_is_ready(&handle))
			break;
		udelay(1000);
	}
	if (timeout == TIMEOUT_MS) {
		printf("al_thermal_sensor_is_ready timed out!\n");
		return -1;
	}

	al_thermal_sensor_trigger_continuous(&handle);

	return 0;
}

int do_thermal_get(
	cmd_tbl_t *cmdtp,
	int flag,
	int argc,
	char *const argv[])
{
	int timeout;

	for (timeout = 0; timeout < TIMEOUT_MS; timeout++) {
		if (al_thermal_sensor_readout_is_valid(&handle))
			break;
		udelay(1000);
	}
	if (timeout == TIMEOUT_MS) {
		printf("al_thermal_sensor_readout_is_valid timed out!\n");
		return -1;
	}

	printf("temprature: %d degrees\n",
		al_thermal_sensor_readout_get(&handle));

	return 0;
}

U_BOOT_CMD(
	thermal_get, 1, 1, do_thermal_get,
	"Thermal sensor get readout",
	"Before the first use, thermal_init must be called.\n");

U_BOOT_CMD(
	thermal_init, 1, 0, do_thermal_init,
	"Initialize the thermal sensor",
	"This command must be called before using thermal_get\n"
	"This command should be called only once\n");
