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

#ifndef __PCA953X_H_
#define __PCA953X_H_

/*** PCA TYPES ***/
#define PCA953X_TYPE            0
#define PCA957X_TYPE            1

/*** PCA953X REGISTERS ***/
#define PCA953X_IN		0x00
#define PCA953X_OUT		0x01
#define PCA953X_POL		0x02
#define PCA953X_CONF		0x03


/*** PCA957X REGISTERS ***/
#define PCA957X_IN              0
#define PCA957X_INVRT           1
#define PCA957X_BKEN            2
#define PCA957X_PUPD            3
#define PCA957X_CFG             4
#define PCA957X_OUT             5
#define PCA957X_MSK             6
#define PCA957X_INTS            7

/*** REGISTER BIT VALUES ***/
#define PCA953X_OUT_LOW		0
#define PCA953X_OUT_HIGH	1
#define PCA953X_POL_NORMAL	0
#define PCA953X_POL_INVERT	1
#define PCA953X_DIR_OUT		0
#define PCA953X_DIR_IN		1

#ifdef CONFIG_SYS_I2C_PCA953X_WIDTH
struct pca953x_chip_ngpio {
	uint8_t chip;
	uint8_t ngpio;
#ifdef CONFIG_SYS_I2C_PCA953X_MULTI_TYPE
	int chip_type;
#endif
};
#endif

int pca953x_set_val(u8 chip, uint mask, uint data);
int pca953x_set_pol(u8 chip, uint mask, uint data);
int pca953x_set_dir(u8 chip, uint mask, uint data);
int pca953x_get_val(u8 chip);

uint pca953x_get_chip_array(struct pca953x_chip_ngpio *chip_ngpios[]);

#endif /* __PCA953X_H_ */
