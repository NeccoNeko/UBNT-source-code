/*
 * (C) Copyright 2008
 * Ricado Ribalda-Universidad Autonoma de Madrid, ricardo.ribalda@uam.es
 * This work has been supported by: QTechnology  http://qtec.com/
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <dtt.h>

#define ADT7475_ADDRESS			0x2E
#define ADT7475_REG_DEVICE_ID		0x3D
#define ADT7475_REG_TEMP_BASE		0x25
#define ADT7475_REG_PWM_BASE		0x30
#define ADT7475_REG_PWM_CONFIG_BASE	0x5C
#define ADT7475_REG_REMOTE2_TMIN	0x69

#define ADT7475_INVALID		128

#define ADT7475_DEVICE_ID	0x75
#define ADT7475_TEMP_REG(idx) (ADT7475_REG_TEMP_BASE + (idx))
#define ADT7475_PWM_REG(idx) (ADT7475_REG_PWM_BASE + (idx))
#define ADT7475_PWM_CONFIG_REG(idx) (ADT7475_REG_PWM_CONFIG_BASE + (idx))

#define ADT7475_SENSOR_NUM	3

#define ADT7475_PWM_MANUAL_CONFIG	0xE2
#define ADT7475_PWM_AUTO_REMOTE2_CONFIG	0x42

#define PWM_AUTO	0x100
#define ADT7475_REMOTE2_TMIN	90		//set REMOTE2 to 90'C

#ifndef ADT7475_FAN1_PWM
#define ADT7475_FAN1_PWM	PWM_AUTO	/* PWM1 Current Duty Cycle (0% to 100% Duty Cycle = 0x00 to 0xFF */
#endif
#ifndef ADT7475_FAN2_PWM
#define ADT7475_FAN2_PWM	PWM_AUTO	/* PWM2 Current Duty Cycle (0% to 100% Duty Cycle = 0x00 to 0xFF */
#endif
#ifndef ADT7475_FAN3_PWM
#define ADT7475_FAN3_PWM	PWM_AUTO	/* PWM3 Current Duty Cycle (0% to 100% Duty Cycle = 0x00 to 0xFF */
#endif

#define ADT7475_PWM_CONFIG	{ADT7475_FAN1_PWM, ADT7475_FAN2_PWM, ADT7475_FAN1_PWM}

static int pwm_config[] = ADT7475_PWM_CONFIG;

int dtt_read(int sensor, int reg)
{
	u8 dir = reg;
	u8 data;

	if (i2c_read(ADT7475_ADDRESS, dir, 1, &data, 1) == -1)
		return -1;
	if (data == ADT7475_INVALID)
		return -1;

	return data;
}

int dtt_write(int sensor, int reg, int val)
{
	u8 dir = reg;
	u8 data = val;

	if (i2c_write(ADT7475_ADDRESS, dir, 1, &data, 1) == -1)
		return -1;

	return 0;
}

int dtt_init_one(int sensor)
{
	unsigned char pwm_value = 0;
	int i;

	if (dtt_read( 0, ADT7475_REG_DEVICE_ID) != ADT7475_DEVICE_ID) {
		puts("Error initialiting ADT7475\n");
		return -1;
	}

	for (i = 0; i < ADT7475_SENSOR_NUM; i++) {
		printf("dtt_get_temp[%d] %d\n", i, dtt_get_temp(i));
	}

	dtt_write( 0, ADT7475_REG_REMOTE2_TMIN, ADT7475_REMOTE2_TMIN);

	for (i = 0; i < ADT7475_SENSOR_NUM; i++) {
		if (pwm_config[i] != PWM_AUTO) {
			pwm_value = pwm_config[i] & 0xFF;
			/* set PWM to Manual Mode */
			dtt_write( 0, ADT7475_PWM_CONFIG_REG(i), ADT7475_PWM_MANUAL_CONFIG);
			/* set PWM value */
			dtt_write( 0, ADT7475_PWM_REG(i), pwm_value);
		} else {
			/* set PWM to Manual Mode */
			dtt_write( 0, ADT7475_PWM_CONFIG_REG(i), ADT7475_PWM_AUTO_REMOTE2_CONFIG);
		}
	}
	return 0;
}

int dtt_get_temp(int sensor)
{
	int aux;

	if (sensor >= ADT7475_SENSOR_NUM) {
		puts("DTT sensor does not exist\n");
		return -1;
	}

	aux = dtt_read( 0, ADT7475_TEMP_REG(sensor));
	if (aux == -1) {
		puts("DTT temperature read failed\n");
		return -1;
	}

	return aux;
}
