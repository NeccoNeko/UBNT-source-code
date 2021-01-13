#include "al_hal_sys_fabric_utils.h"

#define NUM_CORES_PER_CLUSTER		4
#define NUM_LOCAL_CAUSE			4
#define NUM_SEMAPHORES			64

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_handle_init(
	struct al_sys_fabric_handle	*handle,
	void __iomem			*nb_regs_base)
{
	struct al_nb_regs __iomem *nb_regs =
		(struct al_nb_regs __iomem *)nb_regs_base;

	al_assert(handle);

	handle->ver = (al_reg_read32(&nb_regs->nb_version.version) &
		NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_MASK) >>
		NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_SHIFT;

	handle->nb_regs_base = nb_regs_base;
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_cluster_handle_init(
	struct al_sys_fabric_cluster_handle	*handle,
	struct al_sys_fabric_handle		*fabric_handle,
	void __iomem				*anpa_regs_base)
{
	al_assert(handle);
	al_assert(fabric_handle);
	al_assert(
		((fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) &&
		(!anpa_regs_base)) ||
		((fabric_handle->ver >= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V3) &&
		(anpa_regs_base)));

	handle->fabric_handle = fabric_handle;
	handle->anpa_regs_base = anpa_regs_base;
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_int_local_unmask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			idx,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_assert(handle);
	al_assert(idx < NUM_LOCAL_CAUSE);

	al_reg_write32_masked(&nb_regs->cpun_config_status[idx].local_cause_mask, mask, mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_int_local_mask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			idx,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_assert(handle);
	al_assert(idx < NUM_LOCAL_CAUSE);

	al_reg_write32_masked(&nb_regs->cpun_config_status[idx].local_cause_mask, mask, ~mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_int_clear(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_assert(handle);

	al_reg_write32(&nb_regs->global.nb_int_cause, mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_error_int_unmask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs;

	al_assert(handle);

	nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_reg_write32_masked(&nb_regs->global.error_mask, mask, mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_error_int_mask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs;

	al_assert(handle);

	nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_reg_write32_masked(&nb_regs->global.error_mask, mask, ~mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_error_int_clear(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask)
{
	struct al_nb_regs __iomem *nb_regs;

	al_assert(handle);

	nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_reg_write32(&nb_regs->global.error_cause, ~mask);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_sb_pos_info_get_and_clear(
	struct al_sys_fabric_handle		*handle,
	struct al_sys_fabric_sb_pos_error_info	*pos_info)
{
	struct al_nb_regs __iomem *nb_regs;
	uint32_t error_log_1;

	al_assert(handle);
	al_assert(pos_info);

	nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	/* SB PoS error low address bits */
	pos_info->address = al_reg_read32(&nb_regs->global.sb_pos_error_log_0);

	/* this clears the interrupt */
	error_log_1 = al_reg_read32(&nb_regs->global.sb_pos_error_log_1);

	/*
	 * Addressing RMN: 10628
	 *
	 * RMN description:
	 * on V2 there is a HW synchronization bug, which
	 * Nondeterministically results in interrupt not being
	 * cleared after reading the log_1 register
	 *
	 * Software flow:
	 * read log_1 multiple times until interrupt is cleared
	 */
	if (handle->ver == NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		int i;
		uint32_t nb_int_cause;

		for (i = 0; i < 1000; i++) {
			nb_int_cause = al_reg_read32(&nb_regs->global.nb_int_cause);
			if (!(nb_int_cause & NB_GLOBAL_NB_INT_CAUSE_SB_POS_ERR))
				break;

			al_reg_read32(&nb_regs->global.sb_pos_error_log_1);
		}

		if (i == 1000)
			al_err("%s: tried to clear SB PoS error %d times, giving up...\n",
				__func__,
				i);
	}

	/* Error Log 1
	 * [7:0] address_high
	 * [16:8] request id
	 * [18:17] bresp
	 */
	pos_info->address |= ((uint64_t)(error_log_1 & AL_FIELD_MASK(7, 0)) << 32);
	pos_info->request_id = (error_log_1 & AL_FIELD_MASK(16, 8)) >> 8;
	pos_info->bresp = (error_log_1 & AL_FIELD_MASK(18, 17)) >> 17;
	pos_info->valid = (error_log_1 & NB_GLOBAL_SB_POS_ERROR_LOG_1_VALID) ? AL_TRUE : AL_FALSE;

	/*
	 * clear valid bit so that
	 * Subsequent errors will be captured
	 * and information will be valid
	 */
	if (pos_info->valid)
		al_reg_write32(&nb_regs->global.sb_pos_error_log_1, 0);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_msg_set(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				msg,
	al_bool					concat_reg_addr)
{
	uint32_t *msg_reg;
	uintptr_t address;

	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		msg_reg = &nb_regs->debug.cpu_msg[core];

	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
		return;
	}

	if (concat_reg_addr) {
		address = (uintptr_t)msg_reg;
		al_assert((msg & 0xffff) == msg);
		msg |= ((uint32_t)address << 16);
	}

	al_reg_write32(msg_reg, msg);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_msg_get(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				*msg_valid,
	unsigned int				*msg)
{
	unsigned int val;

	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		val = al_reg_read32(&nb_regs->cpun_config_status[core].cpu_msg_in);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
		return;
	}

	*msg_valid = !!(val & NB_CPUN_CONFIG_STATUS_CPU_MSG_IN_VALID);
	*msg = (val & NB_CPUN_CONFIG_STATUS_CPU_MSG_IN_DATA_MASK) >>
		NB_CPUN_CONFIG_STATUS_CPU_MSG_IN_DATA_SHIFT;
}

/**************************************************************************************************/
/**************************************************************************************************/
unsigned int al_sys_fabric_sys_counter_freq_get(
	struct al_sys_fabric_handle	*handle)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_assert(handle);

	return al_reg_read32(&nb_regs->system_counter.cnt_base_freq);
}

/**************************************************************************************************/
/**************************************************************************************************/
uint64_t al_sys_fabric_sys_counter_get_64(
	struct al_sys_fabric_handle	*handle)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;
	uint32_t timer_low;
	uint32_t timer_high_presample;
	uint32_t timer_high_postsample;
	uint64_t timer;

	al_assert(handle);

	do {
		timer_high_presample = al_reg_read32(&nb_regs->system_counter.cnt_high);
		timer_low = al_reg_read32(&nb_regs->system_counter.cnt_low);
		timer_high_postsample = al_reg_read32(&nb_regs->system_counter.cnt_high);
	/* if cnt_high changed during sampling, re-sample the value */
	} while (timer_high_presample != timer_high_postsample);

	timer = (((uint64_t)timer_high_postsample << 32) | (uint64_t)timer_low);

	return timer;
}

/**************************************************************************************************/
/**************************************************************************************************/
uint32_t al_sys_fabric_sys_counter_get_32(
	struct al_sys_fabric_handle	*handle)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;

	al_assert(handle);

	return al_reg_read32(&nb_regs->system_counter.cnt_low);
}

/**************************************************************************************************/
/**************************************************************************************************/
int al_sys_fabric_hw_semaphore_lock(
	struct al_sys_fabric_handle	*handle,
	unsigned int			id)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;
	struct al_nb_semaphores *regs = &nb_regs->semaphores[id];
	uint32_t val;

	al_assert(handle);
	al_assert(id < NUM_SEMAPHORES);

	val = al_reg_read32(&regs->lockn);
	/* ensure memory barrier */
	if (val)
		return -EIO;

	return 0;
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_hw_semaphore_unlock(
	struct al_sys_fabric_handle	*handle,
	unsigned int			id)
{
	struct al_nb_regs __iomem *nb_regs = (struct al_nb_regs __iomem *)handle->nb_regs_base;
	struct al_nb_semaphores *regs = &nb_regs->semaphores[id];

	al_assert(handle);
	al_assert(id < NUM_SEMAPHORES);

	al_reg_write32(&regs->lockn, 0);
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_cluster_pd_pu_timer_set(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				pd,
	unsigned int				pu)
{
	al_assert(handle);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(&nb_regs->global.cpu_max_pd_timer, pd);
		al_reg_write32(&nb_regs->global.cpu_max_pu_timer, pu);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_cluster_power_ctrl(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				ctrl)
{
	al_assert(handle);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(&nb_regs->global.cpus_power_ctrl, ctrl);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_power_ctrl(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				ctrl)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(&nb_regs->cpun_config_status[core].power_ctrl, ctrl);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_cluster_sw_reset(
	struct al_sys_fabric_cluster_handle	*handle)
{
	al_assert(handle);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(
			&nb_regs->global.cpus_software_reset,
			(3 << NB_GLOBAL_CPUS_SOFTWARE_RESET_LEVEL_SHIFT) |
			NB_GLOBAL_CPUS_SOFTWARE_RESET_SWRESET_REQ);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_sw_reset(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(
			&nb_regs->global.cpus_software_reset,
			(1 << (NB_GLOBAL_CPUS_SOFTWARE_RESET_CORES_SHIFT + core)) |
			NB_GLOBAL_CPUS_SOFTWARE_RESET_SWRESET_REQ);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_power_on_reset(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(
			&nb_regs->global.cpus_software_reset,
			(1 << (NB_GLOBAL_CPUS_SOFTWARE_RESET_CORES_SHIFT + core)) |
			NB_GLOBAL_CPUS_SOFTWARE_RESET_LEVEL_CPU_PORESET |
			NB_GLOBAL_CPUS_SOFTWARE_RESET_SWRESET_REQ);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_reset_deassert(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32_masked(
			&nb_regs->global.cpus_init_control, AL_BIT(core), AL_BIT(core));
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_aarch64_setup(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				entry_high,
	uint32_t				entry_low)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);
	al_assert(entry_high || entry_low);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(
			&nb_regs->cpun_config_status[core].rvbar_low,
			entry_low);
		al_reg_write32(
			&nb_regs->cpun_config_status[core].rvbar_high,
			entry_high);
		al_reg_write32(
			&nb_regs->cpun_config_status[core].config_aarch64,
			NB_CPUN_CONFIG_STATUS_CONFIG_AARCH64_AA64_NAA32);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_aarch32_setup(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				entry_high,
	uint32_t				entry_low)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);
	al_assert(entry_high || entry_low);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		al_reg_write32(
			&nb_regs->cpun_config_status[core].resume_addr_l,
			entry_low);
		al_reg_write32(
			&nb_regs->cpun_config_status[core].resume_addr_h,
			entry_high);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

/**************************************************************************************************/
/**************************************************************************************************/
void al_sys_fabric_core_aarch32_setup_get(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				*entry_high,
	uint32_t				*entry_low)
{
	al_assert(handle);
	al_assert(core < NUM_CORES_PER_CLUSTER);
	al_assert(entry_high || entry_low);

	if (handle->fabric_handle->ver <= NB_NB_VERSION_VERSION_RELEASE_NUM_MAJOR_VAL_ALPINE_V2) {
		struct al_nb_regs __iomem *nb_regs =
			(struct al_nb_regs __iomem *)handle->fabric_handle->nb_regs_base;

		*entry_low = al_reg_read32(&nb_regs->cpun_config_status[core].resume_addr_l);
		*entry_high = al_reg_read32(&nb_regs->cpun_config_status[core].resume_addr_h);
	} else {
		al_err("%s: not supported NB version (%u)\n", __func__, handle->fabric_handle->ver);
		al_assert(0);
	}
}

