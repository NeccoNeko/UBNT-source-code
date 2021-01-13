/*
 *  Copyright (C) 2019, Matt Hsu <matt.hsu@ubnt.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __UBNT_HDD_PWRCTL_H__
#define __UBNT_HDD_PWRCTL_H__

#include <linux/types.h>

#define UBNT_HDD_NUM 	4

struct ubnt_hdd_pwrctl_platform_data {
	const char *name;

	int pwren_gpio;
	int present_gpio;
	int fault_led_gpio;
	/* milliseconds */
	u32 pwren_delay;
};

int ubnt_hdd_fault_led_enabled(int fault_led_gpio);

#endif
