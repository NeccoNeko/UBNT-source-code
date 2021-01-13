#ifndef __EARLY_NAND_H__
#define __EARLY_NAND_H__

int early_nand_init(void);

int early_nand_read(
	unsigned int	address,
	void		*buff_ptr,
	unsigned int	num_bytes);

#endif

