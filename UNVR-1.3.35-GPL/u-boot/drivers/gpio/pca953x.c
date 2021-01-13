/*
 * Copyright 2008 Extreme Engineering Solutions, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Driver for NXP's 4, 8 and 16 bit I2C gpio expanders pca953x series
 * (eg pca9537, pca9557, pca9539, etc) and pca957x series (eg pca9575).
 */

#include <common.h>
#include <i2c.h>
#include <pca953x.h>

#include <linux/kernel.h>

/* Default to an address that hopefully won't corrupt other i2c devices */
#ifndef CONFIG_SYS_I2C_PCA953X_ADDR
#define CONFIG_SYS_I2C_PCA953X_ADDR	(~0)
#endif

enum {
	PCA953X_CMD_INFO,
	PCA953X_CMD_DEVICE,
	PCA953X_CMD_OUTPUT,
	PCA953X_CMD_INPUT,
	PCA953X_CMD_INVERT,
};

struct pca953x_regs {
	int in;
	int out;
	int pol;
	int dir;
	int bken;
	int pupd;
	int mask;
	int ints;
};

/*
 * Register address (per chip type) database
 * registers which don't exist in a specific type have the value of -1
 */
static const struct pca953x_regs pca953x_type_regs[] = {
	/*PCA953X_TYPE*/
	{PCA953X_IN, PCA953X_OUT, PCA953X_POL, PCA953X_CONF, -1, -1, -1, -1},
	/*PCA957X_TYPE*/
	{PCA957X_IN, PCA957X_OUT, PCA957X_INVRT, PCA957X_CFG, PCA957X_BKEN,
	 PCA957X_PUPD, PCA957X_MSK, PCA957X_INTS}
};

#ifdef CONFIG_SYS_I2C_PCA953X_WIDTH
static struct pca953x_chip_ngpio pca953x_chip_ngpios[] =
    CONFIG_SYS_I2C_PCA953X_WIDTH;

/*
 * Determine the number of GPIO pins supported. If we don't know we assume
 * 8 pins.
 */
static int pca953x_ngpio(uint8_t chip)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pca953x_chip_ngpios); i++)
		if (pca953x_chip_ngpios[i].chip == chip)
			return pca953x_chip_ngpios[i].ngpio;

	return 8;
}
#else
static int pca953x_ngpio(uint8_t chip)
{
	return 8;
}
#endif

#ifdef CONFIG_SYS_I2C_PCA953X_MULTI_TYPE
/*
 * Determine the type of the chip. If we don't know we assume PCA953X_TYPE.
 */
static int pca953x_chip_type(uint8_t chip)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pca953x_chip_ngpios); i++)
		if (pca953x_chip_ngpios[i].chip == chip)
			return pca953x_chip_ngpios[i].chip_type;

	return PCA953X_TYPE;
}
#else
static int pca953x_chip_type(uint8_t chip)
{
	return PCA953X_TYPE;
}
#endif

/*
 * Modify masked bits in register
 */
static int pca953x_reg_write(uint8_t chip, uint addr, uint mask, uint data)
{
	uint8_t valb;
	uint16_t valw;
	uint16_t ngpios;

	assert((mask & data) == data);

	ngpios = pca953x_ngpio(chip);
	if (ngpios <= 8) {
		if (i2c_read(chip, addr, 1, &valb, 1))
			return -1;

		valb &= ~mask;
		valb |= data;

		return i2c_write(chip, addr, 1, &valb, 1);
	} else if (ngpios <= 16) {
		switch (pca953x_chip_type(chip)) {
		case PCA953X_TYPE:
			if (i2c_read(chip, addr << 1, 1,
				     (u8 *)&valw, 2))
				return -1;

			valw &= ~mask;
			valw |= data;

			return i2c_write(chip, addr << 1, 1,
					(u8 *)&valw, 2);
		case PCA957X_TYPE:
			if (i2c_read(chip, addr << 1, 1,
				     (u8 *)&valw, 1))
				return -1;
			if (i2c_read(chip, (addr << 1)+1, 1,
				     (((u8 *)&valw)+1), 1))
				return -1;

			valw &= ~mask;
			valw |= data;

			if (i2c_write(chip, addr << 1, 1, (u8 *)&valw, 1))
				return -1;
			return i2c_write(chip, (addr << 1)+1, 1,
					((u8 *)&valw)+1, 1);
		default: /*if chip_type isn't one of the predefined types*/
			return -1;
		}
	} else {
		return -1;
	}
}

static int pca953x_reg_read(uint8_t chip, uint addr, uint *data)
{
	uint8_t valb;
	uint16_t valw;
	uint16_t ngpios;

	ngpios = pca953x_ngpio(chip);
	if (ngpios <= 8) {
		if (i2c_read(chip, addr, 1, &valb, 1))
			return -1;
		*data = (int)valb;
	} else if (ngpios <= 16) {
		switch (pca953x_chip_type(chip)) {
		case PCA953X_TYPE:
			if (i2c_read(chip, addr << 1, 1, (u8 *)&valw, 2))
				return -1;
			break;
		case PCA957X_TYPE:
			if (i2c_read(chip, addr << 1, 1, (u8 *)&valw, 1))
				return -1;
			if (i2c_read(chip, (addr << 1)+1, 1,
				     ((u8 *)&valw)+1, 1))
				return -1;
			break;
		default: /*if chip_type isn't one of the predefined types*/
			return -1;
		}
		*data = (int)valw;
	} else {
		return -1;
	}

	return 0;
}

/*
 * Set output value of IO pins in 'mask' to corresponding value in 'data'
 * 0 = low, 1 = high
 */
int pca953x_set_val(uint8_t chip, uint mask, uint data)
{
	int reg_db_index;

	if ((mask & data) != data) {
		error("Trying to write data to bits which are 0 in mask!\nDATA = %#x MASK = %#x\n",
		      data, mask);
		return -1;
	}

	reg_db_index = pca953x_chip_type(chip);
	return pca953x_reg_write(chip, pca953x_type_regs[reg_db_index].out,
				mask, data);
}

/*
 * Set read polarity of IO pins in 'mask' to corresponding value in 'data'
 * 0 = read pin value, 1 = read inverted pin value
 */
int pca953x_set_pol(uint8_t chip, uint mask, uint data)
{
	int reg_db_index;

	if ((mask & data) != data) {
		error("Trying to write data to bits which are 0 in mask!\nDATA = %#x MASK = %#x\n",
		      data, mask);
		return -1;
	}

	reg_db_index = pca953x_chip_type(chip);
	return pca953x_reg_write(chip, pca953x_type_regs[reg_db_index].pol,
				mask, data);
}

/*
 * Set direction of IO pins in 'mask' to corresponding value in 'data'
 * 0 = output, 1 = input
 */
int pca953x_set_dir(uint8_t chip, uint mask, uint data)
{
	int reg_db_index;

	if ((mask & data) != data) {
		error("Trying to write data to bits which are 0 in mask!\nDATA = %#x MASK = %#x\n",
		      data, mask);
		return -1;
	}

	reg_db_index = pca953x_chip_type(chip);
	return pca953x_reg_write(chip, pca953x_type_regs[reg_db_index].dir,
				mask, data);
}

/*
 * Read current logic level of all IO pins
 */
int pca953x_get_val(uint8_t chip)
{
	uint val;
	int reg_db_index;

	reg_db_index = pca953x_chip_type(chip);
	if (pca953x_reg_read(chip, pca953x_type_regs[reg_db_index].in, &val)
		< 0)
		return -1;

	return (int)val;
}

#ifdef CONFIG_CMD_PCA953X
#ifdef CONFIG_CMD_PCA953X_INFO
/*
 * Display pca953x information
 */
static int pca953x_info(uint8_t chip)
{
	int i;
	uint data;
	int nr_gpio = pca953x_ngpio(chip);
	int msb = nr_gpio - 1;
	int reg_db_index;

	reg_db_index = pca953x_chip_type(chip);

	printf("pca953x@ 0x%x (%d pins):\n\n", chip, nr_gpio);
	printf("gpio pins: ");
	for (i = msb; i >= 0; i--)
		printf("%x", i);
	printf("\n");
	for (i = 11 + nr_gpio; i > 0; i--)
		printf("-");
	printf("\n");

	if (pca953x_reg_read(chip, pca953x_type_regs[reg_db_index].dir, &data)
		< 0)
		return -1;
	printf("conf:      ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? 'i' : 'o');
	printf("\n");

	if (pca953x_reg_read(chip, pca953x_type_regs[reg_db_index].pol, &data)
		< 0)
		return -1;
	printf("invert:    ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	if (pca953x_reg_read(chip, pca953x_type_regs[reg_db_index].in, &data)
		< 0)
		return -1;
	printf("input:     ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	if (pca953x_reg_read(chip, pca953x_type_regs[reg_db_index].out, &data)
		< 0)
		return -1;
	printf("output:    ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	return 0;
}
#endif /* CONFIG_CMD_PCA953X_INFO */

cmd_tbl_t cmd_pca953x[] = {
	U_BOOT_CMD_MKENT(device, 3, 0, (void *)PCA953X_CMD_DEVICE, "", ""),
	U_BOOT_CMD_MKENT(output, 4, 0, (void *)PCA953X_CMD_OUTPUT, "", ""),
	U_BOOT_CMD_MKENT(input, 3, 0, (void *)PCA953X_CMD_INPUT, "", ""),
	U_BOOT_CMD_MKENT(invert, 4, 0, (void *)PCA953X_CMD_INVERT, "", ""),
#ifdef CONFIG_CMD_PCA953X_INFO
	U_BOOT_CMD_MKENT(info, 2, 0, (void *)PCA953X_CMD_INFO, "", ""),
#endif
};

int do_pca953x(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	static uint8_t chip = CONFIG_SYS_I2C_PCA953X_ADDR;
	int ret = CMD_RET_USAGE, val;
	ulong ul_arg2 = 0;
	ulong ul_arg3 = 0;
	cmd_tbl_t *c;
#ifdef CONFIG_SYS_I2C_PCA953X_WIDTH
	static int chip_initialized;

	if (!chip_initialized) {
		chip = pca953x_chip_ngpios[0].chip;
		chip_initialized = 1;
	}
#endif

	c = find_cmd_tbl(argv[1], cmd_pca953x, ARRAY_SIZE(cmd_pca953x));

	/* All commands but "device" require 'maxargs' arguments */
	if (!c || !((argc == (c->maxargs)) ||
		(((uintptr_t)c->cmd == PCA953X_CMD_DEVICE) &&
		 (argc == (c->maxargs - 1))))) {
		return CMD_RET_USAGE;
	}

	/* arg2 used as chip number or pin number */
	if (argc > 2)
		ul_arg2 = simple_strtoul(argv[2], NULL, 16);

	/* arg3 used as pin or invert value */
	if (argc > 3)
		ul_arg3 = simple_strtoul(argv[3], NULL, 16) & 0x1;

	switch ((uintptr_t)c->cmd) {
#ifdef CONFIG_CMD_PCA953X_INFO
	case PCA953X_CMD_INFO:
		ret = pca953x_info(chip);
		if (ret)
			ret = CMD_RET_FAILURE;
		break;
#endif

	case PCA953X_CMD_DEVICE:
		if (argc == 3)
			chip = (uint8_t)ul_arg2;
		printf("Current device address: 0x%x\n", chip);
		ret = CMD_RET_SUCCESS;
		break;

	case PCA953X_CMD_INPUT:
		ret = pca953x_set_dir(chip, (1 << ul_arg2),
				PCA953X_DIR_IN << ul_arg2);
		val = (pca953x_get_val(chip) & (1 << ul_arg2)) != 0;

		if (ret)
			ret = CMD_RET_FAILURE;
		else
			printf("chip 0x%02x, pin 0x%lx = %d\n", chip, ul_arg2,
									val);
		break;

	case PCA953X_CMD_OUTPUT:
		ret = pca953x_set_dir(chip, (1 << ul_arg2),
				(PCA953X_DIR_OUT << ul_arg2));
		if (!ret)
			ret = pca953x_set_val(chip, (1 << ul_arg2),
						(ul_arg3 << ul_arg2));
		if (ret)
			ret = CMD_RET_FAILURE;
		break;

	case PCA953X_CMD_INVERT:
		ret = pca953x_set_pol(chip, (1 << ul_arg2),
					(ul_arg3 << ul_arg2));
		if (ret)
			ret = CMD_RET_FAILURE;
		break;
	}

	if (ret == CMD_RET_FAILURE)
		eprintf("Error talking to chip at 0x%x\n", chip);

	return ret;
}

U_BOOT_CMD(
	pca953x,	5,	1,	do_pca953x,
	"pca953x gpio access",
	"device [dev]\n"
	"	- show or set current device address\n"
#ifdef CONFIG_CMD_PCA953X_INFO
	"pca953x info\n"
	"	- display info for current chip\n"
#endif
	"pca953x output pin 0|1\n"
	"	- set pin as output and drive low or high\n"
	"pca953x invert pin 0|1\n"
	"	- disable/enable polarity inversion for reads\n"
	"pca953x input pin\n"
	"	- set pin as input and read value"
);

#endif /* CONFIG_CMD_PCA953X */

/* Returns the size of pca953x_chip_ngpios and indirectly returns a pointer to it*/
#ifdef CONFIG_SYS_I2C_PCA953X_WIDTH
uint pca953x_get_chip_array(struct pca953x_chip_ngpio *chip_ngpios[])
{
	*chip_ngpios  = pca953x_chip_ngpios;
	return ARRAY_SIZE(pca953x_chip_ngpios);
}
#endif
