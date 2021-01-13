#include "al_hal_nand.h"
#include "al_hal_iomap.h"

#define PG_READ_SEQ_MAX_SIZE		32
#define BB_MAP_MAX_SIZE			sizeof(uint32_t)

#define ALIGN_DOWN_2_POWER(addr, size)	((addr)&(~((size)-1)))

struct early_nand_bb_map {
	uint32_t	is_bad_map;
	uint32_t	is_known_map;
};

static struct nand_db {
	struct al_nand_ctrl_obj			obj;
	struct al_nand_extra_dev_properties	dev_ext_props;
	struct al_nand_ecc_config		ecc_config;
	struct early_nand_bb_map		bb_map;
} nand_db;

static int early_nand_spare_read(
	unsigned int	pg_addr,
	unsigned int	spare_addr,
	uint8_t	*buff,
	unsigned int	num_bytes);

static int early_nand_is_blk_bad_raw_within_pg_wrd(
	unsigned int	pg_addr,
	unsigned int	spare_wrd_idx,
	int		*bad);

static int early_nand_is_blk_bad_raw_within_pg(
	unsigned int	pg_addr,
	int		*bad);

static int early_nand_is_blk_bad_raw(
	unsigned int	blk_addr,
	int		*bad);

static int early_nand_is_block_bad(
	unsigned int	blk_addr,
	int		*bad);

static int early_nand_addr_logic_to_phys(
	unsigned int	logic_addr,
	unsigned int	*phys_addr);

int early_nand_init(void)
{
	int err;
	struct al_nand_dev_properties dev_props;

	memset(&nand_db, 0, sizeof(struct nand_db));

	err = al_nand_properties_decode(
		(void __iomem *)AL_PBS_REGFILE_BASE,
		&dev_props,
		&nand_db.ecc_config,
		&nand_db.dev_ext_props);
	if (err) {
		printf("al_nand_properties_decode failed\n");
		return err;
	}

#ifdef AL_NAND_BBM_OVERRIDE
	nand_db.dev_ext_props.badBlockMarking.method = NAND_BAD_BLOCK_MARKING_CHECK_1ST_PAGE;
#endif

	err = al_nand_init(&nand_db.obj, (void *)(uintptr_t)AL_NAND_BASE, NULL, 0);
	if (err) {
		printf("nand init failed\n");
		return err;
	}

	al_nand_dev_select(&nand_db.obj, 0);

	err = al_nand_dev_config(&nand_db.obj, &dev_props, &nand_db.ecc_config);
	if (err) {
		printf("al_nand_dev_config failed\n");
		return err;
	}

	return 0;
}

int early_nand_read(
	unsigned int	address,
	void		*buff_ptr,
	unsigned int	num_bytes)
{
	int err = 0;
	uint8_t *buff = buff_ptr;
	uint32_t cmd_seq_pg_rd[PG_READ_SEQ_MAX_SIZE];
	uint32_t cw_size;
	uint32_t cw_count;
	int cmd_seq_pg_rd_num_entries;

	al_dbg("%s(%u, %u)\n", __func__, address, num_bytes);

	if (nand_db.dev_ext_props.eccIsEnabled) {
		al_nand_uncorr_err_clear(&nand_db.obj);
		al_nand_corr_err_clear(&nand_db.obj);
	}

	while (num_bytes) {
		unsigned int pg_base_addr_phys;
		int pg_base_addr = ALIGN_DOWN_2_POWER(address, nand_db.dev_ext_props.pageSize);
		int next_pg_base_addr = pg_base_addr + nand_db.dev_ext_props.pageSize;
		int num_bytes_skip_head = address - pg_base_addr;
		int num_bytes_curr =
			((num_bytes + num_bytes_skip_head) < nand_db.dev_ext_props.pageSize) ?
			(num_bytes) :
			(nand_db.dev_ext_props.pageSize - num_bytes_skip_head);
		int num_bytesSkipTail = next_pg_base_addr - (address + num_bytes_curr);

		err = early_nand_addr_logic_to_phys(
			pg_base_addr,
			&pg_base_addr_phys);
		if (err) {
			al_err("early_nand_addr_logic_to_phys failed!\n");
			return err;
		}

		al_nand_tx_set_enable(&nand_db.obj, 1);
		al_nand_tx_set_enable(&nand_db.obj, 0);

		cmd_seq_pg_rd_num_entries = PG_READ_SEQ_MAX_SIZE;

		if (nand_db.dev_ext_props.eccIsEnabled) {
			err = al_nand_cmd_seq_gen_page_read(
				&nand_db.obj,
				0,
				pg_base_addr_phys / nand_db.dev_ext_props.pageSize,
				nand_db.dev_ext_props.pageSize,
				1,
				cmd_seq_pg_rd,
				&cmd_seq_pg_rd_num_entries,
				&cw_size,
				&cw_count);
			if (err) {
				al_err("al_nand_cmd_seq_gen_page_read failed!\n");
				return err;
			}
		} else {
			int num_bytes;

			num_bytes = num_bytes_curr;

			err = al_nand_cmd_seq_gen_page_read(
				&nand_db.obj,
				num_bytes_skip_head,
				pg_base_addr_phys / nand_db.dev_ext_props.pageSize,
				num_bytes,
				0,
				cmd_seq_pg_rd,
				&cmd_seq_pg_rd_num_entries,
				&cw_size,
				&cw_count);
			if (err) {
				al_err("al_nand_cmd_seq_gen_page_read failed!\n");
				return err;
			}
		}

		al_nand_cw_config(&nand_db.obj, cw_size, cw_count);

		if (nand_db.dev_ext_props.eccIsEnabled)
			al_nand_ecc_set_enabled(&nand_db.obj, 1);

		al_nand_cmd_seq_execute(&nand_db.obj, cmd_seq_pg_rd, cmd_seq_pg_rd_num_entries);

		if (nand_db.dev_ext_props.eccIsEnabled) {
			err = al_nand_data_buff_read(
				&nand_db.obj,
				nand_db.dev_ext_props.pageSize,
				num_bytes_skip_head,
				num_bytesSkipTail,
				buff);
			if (err) {
				al_err("al_nand_data_buff_read failed!\n");
				return err;
			}
		} else {
			int num_bytes_skip_head;

			num_bytes_skip_head = 0;

			err = al_nand_data_buff_read(
				&nand_db.obj,
				num_bytes_curr + num_bytes_skip_head,
				num_bytes_skip_head,
				0,
				buff);
			if (err) {
				al_err("al_nand_data_buff_read failed!\n");
				return err;
			}
		}

		if (nand_db.dev_ext_props.eccIsEnabled)
			al_nand_ecc_set_enabled(&nand_db.obj, 0);

		num_bytes -= num_bytes_curr;
		address = next_pg_base_addr;
		buff += num_bytes_curr;
	}

	if (nand_db.dev_ext_props.eccIsEnabled) {
		if (al_nand_uncorr_err_get(&nand_db.obj)) {
			al_err("uncorrectable errors!\n");
			return err;
		}

		if (al_nand_corr_err_get(&nand_db.obj))
			al_err("correctable errors!\n");
	}

	return 0;
}

static int early_nand_spare_read(
	unsigned int	pg_addr,
	unsigned int	spare_addr,
	uint8_t		*buff,
	unsigned int	num_bytes)
{
	int err = 0;
	uint32_t cmd_seq_pg_rd[PG_READ_SEQ_MAX_SIZE];
	uint32_t cw_size;
	uint32_t cw_count;
	int cmd_seq_pg_rd_num_entries;

	al_dbg("early_nand_spare_read(%u, %u, %u)\n", pg_addr, spare_addr, num_bytes);

	cmd_seq_pg_rd_num_entries = PG_READ_SEQ_MAX_SIZE;

	err = al_nand_cmd_seq_gen_page_read(
		&nand_db.obj,
		nand_db.dev_ext_props.pageSize + spare_addr,
		pg_addr / nand_db.dev_ext_props.pageSize,
		num_bytes,
		0,
		cmd_seq_pg_rd,
		&cmd_seq_pg_rd_num_entries,
		&cw_size,
		&cw_count);
	if (err) {
		al_err("al_nand_cmd_seq_gen_page_read failed!\n");
		return err;
	}

	al_nand_cw_config(&nand_db.obj, cw_size, cw_count);
	al_nand_cmd_seq_execute(&nand_db.obj, cmd_seq_pg_rd, cmd_seq_pg_rd_num_entries);

	err = al_nand_data_buff_read(
		&nand_db.obj,
		num_bytes,
		0,
		0,
		buff);
	if (err) {
		al_err("al_nand_data_buff_read failed!\n");
		return err;
	}

	return 0;
}

static int early_nand_is_blk_bad_raw_within_pg_wrd(
	unsigned int	pg_addr,
	unsigned int	spare_wrd_idx,
	int		*bad)
{
	int err = 0;
	uint16_t word = 0xFFFF;

	err = early_nand_spare_read(pg_addr, spare_wrd_idx, (uint8_t *)&word, 1);
	if (err) {
		printf("early_nand_spare_read failed\n");
		return err;
	}

	*bad = (word != 0xFFFF);

	return 0;
}

static int early_nand_is_blk_bad_raw_within_pg(
	unsigned int	pg_addr,
	int		*bad)
{
	int err = 0;
	struct al_nand_extra_dev_properties *dev_ext_props = &nand_db.dev_ext_props;
	struct al_nand_bad_block_marking *bbm = &dev_ext_props->badBlockMarking;
	int bad_local;

	err = early_nand_is_blk_bad_raw_within_pg_wrd(
			pg_addr,
			bbm->location1,
			&bad_local);
	if (err) {
		al_err("early_nand_is_blk_bad_raw_within_pg_wrd failed!\n");
		return err;
	}

	if ((0 == bad_local) && (bbm->location1 != bbm->location2)) {
		err = early_nand_is_blk_bad_raw_within_pg_wrd(
			pg_addr,
			bbm->location2,
			&bad_local);
		if (err) {
			al_err("early_nand_is_blk_bad_raw_within_pg_wrd failed!\n");
			return err;
		}
	}

	*bad = bad_local;

	return 0;
}

static int early_nand_is_blk_bad_raw(
	unsigned int	blk_addr,
	int		*bad)
{
	int err = 0;
	struct al_nand_extra_dev_properties *dev_ext_props = &nand_db.dev_ext_props;
	struct al_nand_bad_block_marking *bbm = &dev_ext_props->badBlockMarking;
	int num_pgs = dev_ext_props->blockSize / dev_ext_props->pageSize;
	int i;
	int pg_idx_arr[2] = { -1, -1 };

	*bad = 0;

	switch (bbm->method) {
	case NAND_BAD_BLOCK_MARKING_CHECK_1ST_PAGE:
		pg_idx_arr[0] = 0;
		break;

	case NAND_BAD_BLOCK_MARKING_CHECK_1ST_PAGES:
		pg_idx_arr[0] = 0;
		pg_idx_arr[1] = 1;
		break;

	case NAND_BAD_BLOCK_MARKING_CHECK_LAST_PAGE:
		pg_idx_arr[0] = num_pgs - 1;
		break;

	case NAND_BAD_BLOCK_MARKING_CHECK_LAST_PAGES:
		pg_idx_arr[0] = num_pgs - 1;
		pg_idx_arr[1] = num_pgs - 3;
		break;

	default:
		al_err("%s: invalid method (%d)\n", __func__, bbm->method);
		return -EINVAL;
	}

	for (i = 0; (i < 2) && (pg_idx_arr[i] != -1) && (0 == (*bad)); i++) {
		err = early_nand_is_blk_bad_raw_within_pg(
			blk_addr + pg_idx_arr[i] * dev_ext_props->pageSize,
			bad);
		if (err) {
			al_err("early_nand_is_blk_bad_raw_within_pg failed!\n");
			return err;
		}
	}

	return 0;
}

static int early_nand_is_block_bad(
	unsigned int	blk_addr,
	int		*bad)
{
	int err = 0;
	struct early_nand_bb_map *bb_map = &nand_db.bb_map;
	struct al_nand_extra_dev_properties *dev_ext_props = &nand_db.dev_ext_props;
	int blk_num = blk_addr / dev_ext_props->blockSize;

	if ((blk_num >= BB_MAP_MAX_SIZE) || (!(bb_map->is_known_map & AL_BIT(blk_num)))) {
		err = early_nand_is_blk_bad_raw(blk_addr, bad);
		if (err) {
			al_err("early_nand_is_blk_bad_raw failed!\n");
			return err;
		}

		if (blk_num < BB_MAP_MAX_SIZE) {
			bb_map->is_known_map |= AL_BIT(blk_num);
			if (*bad)
				bb_map->is_bad_map |= AL_BIT(blk_num);
			else
				bb_map->is_bad_map &= ~AL_BIT(blk_num);
		}
	} else {
		*bad = !!(bb_map->is_bad_map & AL_BIT(blk_num));
	}

	return 0;
}

static int early_nand_addr_logic_to_phys(
	unsigned int	logic_addr,
	unsigned int	*phys_addr)
{
	int err = 0;
	struct al_nand_extra_dev_properties *dev_ext_props = &nand_db.dev_ext_props;
	struct al_nand_bad_block_marking *bbm = &dev_ext_props->badBlockMarking;
	int num_bad_blks = 0;

	if (bbm->method != NAND_BAD_BLOCK_MARKING_METHOD_DISABLED) {
		int logic_blk_num = logic_addr / dev_ext_props->blockSize;
		int i;

		for (i = 0, num_bad_blks = 0; i <= (logic_blk_num + num_bad_blks); i++) {
			int is_bad;

			err = early_nand_is_block_bad(
				i * dev_ext_props->blockSize,
				&is_bad);
			if (err) {
				al_err("early_nand_is_block_bad failed!\n");
				return err;
			}

			if (is_bad)
				num_bad_blks++;
		}
	}

	*phys_addr = logic_addr + num_bad_blks * dev_ext_props->blockSize;

	if ((*phys_addr) != logic_addr)
		al_err("%s: %u --> %u\n", __func__, logic_addr, *phys_addr);

	return 0;
}

