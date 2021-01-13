#ifndef __AL_HAL_SYS_FABRIC_UTILS_H__
#define __AL_HAL_SYS_FABRIC_UTILS_H__

#include "al_hal_common.h"
#include "al_hal_nb_regs.h"

/*
 * SB PoS error info
 */
struct al_sys_fabric_sb_pos_error_info {
	/* Set on SB PoS error occurrence */
	al_bool valid;

	/* Write address that caused the error */
	uint64_t address;

	/* Transaction AXI ID */
	uint8_t request_id;

	/*
	 * Response
	 * 0x0: OK
	 * 0x1: Exclusive OK
	 * 0x2: Slave error
	 * 0x3: Decode error
	 */
	uint8_t bresp;
};

/**
 * System fabric handle
 */
struct al_sys_fabric_handle {
	unsigned int	ver;
	void __iomem	*nb_regs_base;
};

/**
 * System fabric cluster handle
 */
struct al_sys_fabric_cluster_handle {
	struct al_sys_fabric_handle	*fabric_handle;
	void __iomem			*anpa_regs_base;
};

/**
 * System fabric handle initialization
 *
 * @param	handle
 *		System fabric clear handle
 * @param	nb_regs_base
 *		NB service registers base address
 */
void al_sys_fabric_handle_init(
	struct al_sys_fabric_handle	*handle,
	void __iomem			*nb_regs_base);

/**
 * System fabric cluster handle initialization
 *
 * @param	handle
 *		System fabric cluster clear handle
 * @param	fabric_handle
 *		System fabric initialized handle
 * @param	anpa_regs_base
 *		ANPA registers base address
 */
void al_sys_fabric_cluster_handle_init(
	struct al_sys_fabric_cluster_handle	*handle,
	struct al_sys_fabric_handle		*fabric_handle,
	void __iomem				*anpa_regs_base);

/**
 * System fabric interrupt local unmasking
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	idx
 *		Local interrupt index
 * @param	mask
 *		Interrupts to unmask (see NB_GLOBAL_NB_INT_CAUSE_*)
 */
void al_sys_fabric_int_local_unmask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			idx,
	unsigned int			mask);

/**
 * System fabric interrupt local masking
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	idx
 *		Local interrupt index
 * @param	mask
 *		Interrupts to mask (see NB_GLOBAL_NB_INT_CAUSE_*)
 */
void al_sys_fabric_int_local_mask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			idx,
	unsigned int			mask);

/**
 * System fabric interrupt clearing
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	mask
 *		Interrupts to clear (see NB_GLOBAL_NB_INT_CAUSE_*)
 */
void al_sys_fabric_int_clear(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask);

/**
 * System fabric error interrupt unmasking
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	mask
 *		Interrupts to unmask (see NB_GLOBAL_NB_ERROR_CAUSE_*)
 */
void al_sys_fabric_error_int_unmask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask);

/**
 * System fabric error interrupt masking
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	mask
 *		Interrupts to mask (see NB_GLOBAL_NB_ERROR_CAUSE_*)
 */
void al_sys_fabric_error_int_mask(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask);

/**
 * System fabric error interrupt clearing
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	mask
 *		Interrupts to clear (see NB_GLOBAL_NB_ERROR_CAUSE_*)
 */
void al_sys_fabric_error_int_clear(
	struct al_sys_fabric_handle	*handle,
	unsigned int			mask);

/**
 * System fabric SB PoS error interrupt clearing and info retrieve
 *
 * @param	handle
 *		System fabric initialized handle
 * @param	pos_info
 *		pos_info instance
 */
void al_sys_fabric_sb_pos_info_get_and_clear(
	struct al_sys_fabric_handle		*handle,
	struct al_sys_fabric_sb_pos_error_info	*pos_info);

/**
 * System fabric core message setting
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	msg
 *		Message to set
 *		Note: message meaning depends on implementation
 * @param	concat_reg_addr
 *		Concatenate the low address halfword to the data, because it simplifies matching
 *		data to address in waves/logs
 */
void al_sys_fabric_core_msg_set(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				msg,
	al_bool					concat_reg_addr);

/**
 * System fabric core message getting
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	msg_valid
 *		An indication whether the message is valid
 * @param	msg
 *		Message received
 *		Note: message meaning depends on implementation
 */
void al_sys_fabric_core_msg_get(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				*msg_valid,
	unsigned int				*msg);

/**
 * System fabric system counter get frequency
 *
 * @param	handle
 *		System fabric initialized handle
 *
 * @returns	System fabric system counter frequency [Hz]
 */
unsigned int al_sys_fabric_sys_counter_freq_get(
	struct al_sys_fabric_handle	*handle);

/**
 * System fabric system counter get 64 bits value
 *
 * @param	handle
 *		System fabric initialized handle
 *
 * @returns	System fabric system counter value (64 bits)
 */
uint64_t al_sys_fabric_sys_counter_get_64(
	struct al_sys_fabric_handle	*handle);

/**
 * System fabric system counter get 32 bits value
 *
 * @param	handle
 *		System fabric initialized handle
 *
 * @returns	System fabric system counter value (32 bits)
 */
uint32_t al_sys_fabric_sys_counter_get_32(
	struct al_sys_fabric_handle	*handle);

/**
 * System fabric semaphore lock
 *
 * @param	handle
 *		System fabric initialized handle
  * @param	id
  *		Semaphore ID
 *
 * @returns	0 upon success
 */
int al_sys_fabric_hw_semaphore_lock(
	struct al_sys_fabric_handle	*handle,
	unsigned int			id);

/**
 * System fabric semaphore unlock
 *
 * @param	handle
 *		System fabric initialized handle
  * @param	id
  *		Semaphore ID
 */
void al_sys_fabric_hw_semaphore_unlock(
	struct al_sys_fabric_handle	*handle,
	unsigned int			id);

/**
 * System fabric cluster powerdown/powerup timer setting
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	pd
 *		Powerdown time [system fabric clocks]
 * @param	pu
 *		Powerup time [system fabric clocks]
 */
void al_sys_fabric_cluster_pd_pu_timer_set(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				pd,
	unsigned int				pu);

/**
 * System fabric cluster power control
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	ctrl
 *		Control value (see NB_GLOBAL_CPUS_POWER_CTRL_*)
 */
void al_sys_fabric_cluster_power_ctrl(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				ctrl);

/**
 * System fabric core power control
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	ctrl
 *		Control value (see NB_CPUN_CONFIG_STATUS_POWER_CTRL_*)
 */
void al_sys_fabric_core_power_ctrl(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	unsigned int				ctrl);

/**
 * System fabric cluster SW reset
 *
 * @param	handle
 *		System fabric cluster initialized handle
 */
void al_sys_fabric_cluster_sw_reset(
	struct al_sys_fabric_cluster_handle	*handle);

/**
 * System fabric core SW reset
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 */
void al_sys_fabric_core_sw_reset(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core);

/**
 * System fabric core power-on reset (POR)
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 */
void al_sys_fabric_core_power_on_reset(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core);

/**
 * System fabric core reset de-assertion
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 */
void al_sys_fabric_core_reset_deassert(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core);

/**
 * System fabric core setup for aarch64
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	entry_high
 *		aarch64 entry point high 32 bits
 * @param	entry_low
 *		aarch64 entry point low 32 bits
 */
void al_sys_fabric_core_aarch64_setup(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				entry_high,
	uint32_t				entry_low);

/**
 * System fabric core setup for aarch32
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	entry_high
 *		aarch32 entry point high 32 bits
 * @param	entry_low
 *		aarch32 entry point low 32 bits
 */
void al_sys_fabric_core_aarch32_setup(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				entry_high,
	uint32_t				entry_low);

/**
 * System fabric core aarch32 setup get
 *
 * @param	handle
 *		System fabric cluster initialized handle
 * @param	core
 *		Core index (within cluster)
 * @param	entry_high
 *		aarch32 entry point high 32 bits
 * @param	entry_low
 *		aarch32 entry point low 32 bits
 */
void al_sys_fabric_core_aarch32_setup_get(
	struct al_sys_fabric_cluster_handle	*handle,
	unsigned int				core,
	uint32_t				*entry_high,
	uint32_t				*entry_low);

#endif

