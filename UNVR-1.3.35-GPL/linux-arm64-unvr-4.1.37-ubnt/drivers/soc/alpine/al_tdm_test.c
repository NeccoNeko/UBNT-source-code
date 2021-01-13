/*******************************************************************************
 ** Includes
 ******************************************************************************/
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#include <al_hal_pbs_iofic.h>
#include <al_hal_iofic.h>
#include <al_hal_tdm.h>
#include <al_hal_reg_utils.h>

#define ACTIVE_SLOTS				((uint32_t)((1ULL << (num_slots)) - 1))

#define EXTERNAL_DEVICES_CONNECTED		0

#define PAD					AL_TRUE
#if (PAD == AL_TRUE)
#define USER_BUFF_NUM_SAMPLES			ARRAY_SIZE(user_buff)
#else
#define USER_BUFF_NUM_SAMPLES			sizeof(user_buff)
#endif

#define USER_BUFF_SIZE				128
#define SB_CLK_FREQ				500000000

#define AL_TDM_AXI_TX_FIFO_OFFSET 		0x00000
#define AL_TDM_AXI_RX_FIFO_OFFSET 		0x01000
#define AL_TDM_AXI_CFG_OFFSET			0x10000

static void *pbs_base;
static void *pbs_iofic_base;
static void *tdm_base;
static int pbs_irq;
struct al_tdm tdm_obj;

#if (PAD == AL_TRUE)
static uint32_t
#else
static uint8_t
#endif
sync[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };

static spinlock_t pcm_lock;
static unsigned long flags;

/*******************************************************************************
 ** Definitions
 ******************************************************************************/
static unsigned int num_buffs_to_send = 1000000;
module_param(num_buffs_to_send, int, 0);

static unsigned int num_slots = 2;
module_param(num_slots, int, 0);

static struct tasklet_struct tdm_tx_tasklet_obj;
static struct tasklet_struct tdm_rx_tasklet_obj;

static void cs_enter_cb(void *cs_ctx)
{
	spin_lock_irqsave(&pcm_lock, flags);
}

void cs_leave_cb(void *cs_ctx)
{
	spin_unlock_irqrestore(&pcm_lock, flags);
}

uint8_t prand(unsigned int idx)
{
	unsigned int val;

	idx &= 0x3ffff;
	val = 1 + idx + idx * idx + 3 * (idx + 1) * (idx + 5);

	val &= val >> 7;
	val |= val << 3;

	return val;
}

static void tdm_tx_work(
	int *work_left_current,
	int *work_left_total)
{
	uint32_t user_buff[32];
	static unsigned int num_buffers_added = 0;
	static unsigned int tx_wrd_cnt = 0;
	static al_bool sync_sent = AL_FALSE;
	static unsigned int work_iteration = 0;
	unsigned int work_iteration_curr = 0;
	int i;

	if (!work_iteration)
		al_err("TX Side\n");

	work_iteration++;

	if (!sync_sent) {
		al_tdm_tx_fifo_samples_add(&tdm_obj, sync, ARRAY_SIZE(sync), AL_TRUE);
		sync_sent = AL_TRUE;
	}

	while (1) {
		struct al_tdm_tx_fifo_status tx_fifo_status;

		al_tdm_tx_fifo_status_get(&tdm_obj, &tx_fifo_status);
		if (tx_fifo_status.underrun) {
			al_err("TX FIFO underrun!\n");
			*work_left_current = 0;
			*work_left_total = 0;
			return;
		}

		if ((num_buffers_added == num_buffs_to_send) || (tx_fifo_status.num_samples_vacant < 256))
			break;

#if 0
		if (work_iteration_curr == 10) {
			*work_left_total = 1;
			*work_left_current = 1;
			return;
		}
#endif

		for (i = 0; i < ARRAY_SIZE(user_buff); i++) {
			user_buff[i] = prand(tx_wrd_cnt);
			tx_wrd_cnt++;
		}

		al_tdm_tx_fifo_samples_add(&tdm_obj, user_buff, USER_BUFF_NUM_SAMPLES, AL_TRUE);
		num_buffers_added++;
		work_iteration_curr++;

		if (num_buffers_added == 7)
			al_tdm_set_enables(&tdm_obj, AL_TRUE, AL_TRUE, AL_TRUE, AL_TRUE);
	}

	al_dbg("%u TX\n", num_buffers_added);

	if (num_buffers_added == num_buffs_to_send) {
		al_err("TX Side terminated\n");
		*work_left_current = 0;
		*work_left_total = 0;
		return;
	}

	*work_left_total = 1;
	*work_left_current = 0;
}

static void tdm_tx_tasklet(unsigned long data)
{
	static int work_left_total = 1;
	int work_left_current = 0;

	if (work_left_total)
		tdm_tx_work(&work_left_current, &work_left_total);

	if (work_left_current) {
		tasklet_schedule(&tdm_tx_tasklet_obj);
	} else if (work_left_total) {
		al_iofic_clear_cause(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY);
		al_iofic_unmask(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY);
	}
}

static int tdm_rx_work(void)
{
	uint32_t user_buff[32];
	static unsigned int num_buffers_added = 0;
	static unsigned int rx_wrd_cnt = 0;
	static unsigned int sync_cnt = 0;
	static unsigned int sync_cnt_attempts = 0;
	static uint32_t sync_buff[ARRAY_SIZE(sync)];
	static unsigned int work_iteration = 0;
	int i;

	if (!work_iteration) {
		al_err("RX Side\n");
		al_err("Waiting for sync...\n");
	}

	work_iteration++;

	while (1) {
		struct al_tdm_rx_fifo_status rx_fifo_status;
		al_bool buff_ok;

		al_tdm_rx_fifo_status_get(&tdm_obj, &rx_fifo_status);
		if (rx_fifo_status.overrun) {
			al_err("RX FIFO overrrun!\n");
			return 0;
		}

		if (sync_cnt < ARRAY_SIZE(sync)) {
			if (!rx_fifo_status.num_samples_avail)
				break;

			al_dbg("Avail: %u\n", rx_fifo_status.num_samples_avail);
			sync_cnt_attempts++;
			if (sync_cnt_attempts > (32 * num_slots)) {
				al_err("RX sync timeout!\n");
				return 0;
			}
			al_tdm_rx_fifo_samples_fetch(
				&tdm_obj, &sync_buff[sync_cnt], 1, AL_TRUE);
			if (num_slots <= 16)
				al_dbg("Got %08x\n", sync_buff[sync_cnt]);

			if (sync_buff[sync_cnt] == sync[sync_cnt]) {
				sync_cnt++;
				al_dbg("Found\n");
			} else {
				int i;

				al_dbg("sync_buff = ");
				for (i = 0; i < sync_cnt; i++)
					sync_buff[i] = sync_buff[i + 1];
				for (i = 0; i < sync_cnt; i++) {
					if (sync_buff[i] == sync[i]) {
						al_dbg("%08x ", sync_buff[i]);
					} else {
						sync_cnt = i;
						break;
					}
				}
				al_dbg("\n");
			}

			continue;
		}

		if (num_buffers_added >= num_buffs_to_send)
			break;

		if (rx_fifo_status.num_samples_avail < USER_BUFF_NUM_SAMPLES)
			break;

		al_memset(user_buff, 0, sizeof(user_buff));
		al_tdm_rx_fifo_samples_fetch(
			&tdm_obj, user_buff, USER_BUFF_NUM_SAMPLES, AL_TRUE);
		buff_ok = AL_TRUE;
		for (i = 0; i < ARRAY_SIZE(user_buff); i++) {
			if (user_buff[i] != (prand(rx_wrd_cnt + i)))
				buff_ok = AL_FALSE;
		}

		if (buff_ok) {
			al_dbg("%u ++\n", num_buffers_added);
		} else {
			al_tdm_set_enables(&tdm_obj, AL_TRUE, AL_TRUE, AL_FALSE, AL_FALSE);

			al_err("%u --\n", num_buffers_added);

			for (i = 0; i < ARRAY_SIZE(user_buff); i++)
				al_err("%u:%u\n", user_buff[i], prand(rx_wrd_cnt + i));
			al_tdm_status_print(&tdm_obj);
			return 0;
		}

		rx_wrd_cnt += ARRAY_SIZE(user_buff);

		num_buffers_added++;
	}

	al_info("%u RX\n", num_buffers_added);

	if (num_buffers_added >= num_buffs_to_send) {
		al_err("RX Side terminated\n");
		return 0;
	}

	return 1;
}

static void tdm_rx_tasklet(unsigned long data)
{
	static int work_left = 1;

	if (work_left)
		work_left = tdm_rx_work();

	if (work_left) {
		al_iofic_clear_cause(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL);
		al_iofic_unmask(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL);
	}
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	uint32_t cause = al_iofic_read_cause(pbs_iofic_base, 0);

	if (!(cause & (AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY | AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL))) {
		al_err("%s: cause = %x)\n", __func__, cause);
	}

	if (cause & (AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY)) {
		al_iofic_mask(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY);

		tasklet_schedule(&tdm_tx_tasklet_obj);

		cause &= ~(AL_PBS_IOFIC_INT_ID_TDM_TX_ALMOST_EMPTY);
	}

	if (cause & (AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL)) {
		al_iofic_mask(pbs_iofic_base, 0, AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL);

		tasklet_schedule(&tdm_rx_tasklet_obj);

		cause &= ~AL_PBS_IOFIC_INT_ID_TDM_RX_ALMOST_FULL;
	}

	if (cause)
    		al_iofic_clear_cause(pbs_iofic_base, 0, cause);

	return IRQ_HANDLED;
}

static struct of_device_id of_pbs_table[] = {
	{.compatible = "annapurna-labs,al-pbs"},
	{ /* end of list */ },
};

static struct of_device_id of_tdm_table[] = {
	{.compatible = "annapurna-labs,al-tdm"},
	{ /* end of list */ },
};

static int hw_init(void)
{
	struct al_tdm_handle_init_params handle_init_params;
	struct device_node *pbs_node;
	struct device_node *tdm_node;
	void *tdm_tx_fifo_base;
	void *tdm_rx_fifo_base;
	void *tdm_cfg_base;
	int err;

	struct al_tdm_cfg tdm_cfg = {
		.zsi_en = AL_FALSE,
		.num_slots_log2 = 7, // This means 128 slots which with 8bits resolution gives 1024 pcm's per frame.
		.slot_size = AL_TDM_SLOT_SIZE_8_BITS,
		.pcm_clk_src = AL_TDM_PCM_CLK_SRC_INTERNAL,
		.pcm_tune_dynamic = AL_TRUE,
		.pcm_tune_period = 36, // This is for the compensation needed in case of 8.192MHz on 500MHz
		.pcm_clk_ratio = (SB_CLK_FREQ / 8192000),  // This is for the 500MHz internal source.
		.window_frame_ratio = (25000000 / 8000), // This must remain as is! = 3125
		.window_hyst_start = 0,
		.window_hyst_level = 0,
		.fsync_edge = AL_TDM_EDGE_RISING,
		.fsync_inv = AL_FALSE,
		.fsync_len = 1, // This means a single cycle of the 1024 pcm's in the 8khz frame.
		.tx_ctrl = {
			.auto_track = AL_TRUE,
			.en_special = AL_TDM_CHAN_EN_SPECIAL_00,
			.data_is_padded = PAD,
			.data_alignment = AL_TDM_DATA_ALIGNMENT_LSB,
			.sample_alignment = AL_TDM_SAMPLE_ALIGNMENT_END,
			.sample_size = AL_TDM_SAMPLE_SIZE_8_BITS,
			.en_edge = AL_TDM_EDGE_RISING,
			.en_inv = AL_FALSE,
			.en_len = 8,
			.en_delay = 0,
			.data_edge = AL_TDM_EDGE_RISING,
			.data_inv = AL_FALSE,
#if EXTERNAL_DEVICES_CONNECTED
			/* Should be set to this value in case external device is connected */
			.data_delay = 1,
#else
			/* Should be set to this value in case NO EXTERNAL DEVICES CONNECTED */
			.data_delay = 0,
#endif
			.active_slots_127_96 = 0,
			.active_slots_95_64 = 0,
			.active_slots_63_32 = 0,
			.active_slots_31_0 = ACTIVE_SLOTS,
			.fifo_empty_threshold = 512,
		},
		.tx_specific_ctrl = {
			.tx_padding_data = AL_TDM_TX_PADDING_DATA_0, // No diff found in dump.
			.tx_inactive_data = AL_TDM_TX_INACTIVE_DATA_TRI_STATE,
			.tx_inactive_data_user_val = 0xf0,
			.tx_underflow_data_user_val = 0x55,
		},
		.rx_ctrl = {
			.auto_track = AL_TRUE,
			.en_special = AL_TDM_CHAN_EN_SPECIAL_00,
			.data_is_padded = PAD,
			.data_alignment = AL_TDM_DATA_ALIGNMENT_LSB,
			.sample_alignment = AL_TDM_SAMPLE_ALIGNMENT_END,
			.sample_size = AL_TDM_SAMPLE_SIZE_8_BITS,
			.en_edge = AL_TDM_EDGE_RISING,
			.en_inv = AL_FALSE,
			.en_len = 8,
			.en_delay = 0,
			.data_edge = AL_TDM_EDGE_RISING,
			.data_inv = AL_FALSE,
			.data_delay = 0,
			.active_slots_127_96 = 0,
			.active_slots_95_64 = 0,
			.active_slots_63_32 = 0,
			.active_slots_31_0 = ACTIVE_SLOTS,
			.fifo_empty_threshold = 512,
		},
		.slverr_en = AL_TRUE,
		.parity_en = AL_TRUE,
	};

	pbs_node = of_find_matching_node(NULL, of_pbs_table);
	pbs_base = of_iomap(pbs_node, 0);
	if (!pbs_base) {
		al_err("%s: Unable to obtain PBS regfile base!\n", __func__);
		return -EINVAL;
	}

	tdm_node = of_find_matching_node(NULL, of_tdm_table);
	tdm_base = of_iomap(tdm_node, 0);
	if (!tdm_base) {
		al_err("%s: Unable to obtain TDM regfile base!\n", __func__);
		return -EINVAL;
	}

	tdm_cfg_base = (uint8_t *)tdm_base + AL_TDM_AXI_CFG_OFFSET;
	al_err("tdm_cfg_base = %p\n", tdm_cfg_base);
	tdm_tx_fifo_base = (uint8_t *)tdm_base + AL_TDM_AXI_TX_FIFO_OFFSET;
	al_err("tdm_tx_fifo_base = %p\n", tdm_tx_fifo_base);
	tdm_rx_fifo_base = (uint8_t *)tdm_base + AL_TDM_AXI_RX_FIFO_OFFSET;
	al_err("tdm_rx_fifo_base = %p\n", tdm_rx_fifo_base);

	memset(&handle_init_params, 0, sizeof(handle_init_params));
	handle_init_params.cfg_regs = (void __iomem*)tdm_cfg_base;
	handle_init_params.rx_fifo_regs = (void __iomem *)tdm_rx_fifo_base;
	handle_init_params.tx_fifo_regs = (void __iomem *)tdm_tx_fifo_base;
	handle_init_params.cs_enter_cb = cs_enter_cb;
	handle_init_params.cs_leave_cb = cs_leave_cb;

	al_tdm_handle_init(&tdm_obj, &handle_init_params);
	al_tdm_config(&tdm_obj, &tdm_cfg);

	/* TDM enable - keep Tx and Rx disabled until ready */
	al_tdm_set_enables( &tdm_obj, AL_TRUE, AL_TRUE, AL_FALSE, AL_FALSE);
	al_tdm_tx_fifo_clear(&tdm_obj);
	al_tdm_rx_fifo_clear(&tdm_obj);

	pbs_iofic_base = al_pbs_iofic_regs_get((void __iomem*)pbs_base);
	al_iofic_config(pbs_iofic_base, 0, 0);
	al_iofic_mask(pbs_iofic_base, 0, 0xffffffff);
	al_iofic_clear_cause(pbs_iofic_base, 0, 0xffffffff);

	pbs_irq = irq_of_parse_and_map(pbs_node, 0);
	err = request_irq(pbs_irq, (irq_handler_t )irq_handler, IRQF_TRIGGER_HIGH, "al-tdm", NULL);
	if (err) {
		al_err("%s: request_irq failed!\n", __func__);
		return err;
	}

	tasklet_init(&tdm_tx_tasklet_obj, tdm_tx_tasklet, 0);
	tasklet_init(&tdm_rx_tasklet_obj, tdm_rx_tasklet, 0);

	return 0;
}

/*******************************************************************************
 ** Test
 ******************************************************************************/
int __init al_tdm_test_init(void)
{
	int err;

	al_info("%s: started\n", __func__);

	err = hw_init();
	if (err)
		return err;

	tdm_tx_tasklet(0);
	tdm_rx_tasklet(0);

	al_info("%s: done\n", __func__);

	return 0;
}

static void __exit al_tdm_test_exit(void)
{
	al_info("%s: started\n", __func__);
	al_tdm_set_enables(&tdm_obj, AL_FALSE, AL_FALSE, AL_FALSE, AL_FALSE);
	al_iofic_mask(pbs_iofic_base, 0, 0xffffffff);
	al_iofic_clear_cause(pbs_iofic_base, 0, 0xffffffff);
	free_irq(pbs_irq, NULL);
	tasklet_disable(&tdm_tx_tasklet_obj);
	tasklet_disable(&tdm_rx_tasklet_obj);
	al_tdm_tx_fifo_clear(&tdm_obj);
	al_tdm_rx_fifo_clear(&tdm_obj);
	iounmap(pbs_base);
	iounmap(tdm_base);
	al_info("%s: done\n", __func__);
}

module_init(al_tdm_test_init);
module_exit(al_tdm_test_exit);
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(irq, "Annapurna Labs TDM loopback test");

