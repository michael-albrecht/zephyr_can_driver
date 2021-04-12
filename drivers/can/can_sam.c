/*
 * Copyright (c) 2020 Abram Early
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/can.h>
#include <devicetree.h>
#include "can_mcan_int.h"
#include "can_sam.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(can_driver, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT atmel_sam_mcan

static int can_sam_init(const struct device *dev)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;
	int ret;

	soc_pmc_peripheral_enable(cfg->periph_id);

	soc_gpio_list_configure(cfg->pins, cfg->num_pins);

	ret = can_mcan_init(dev, mcan_cfg, msg_ram, mcan_data);
	if (ret) {
		return ret;
	}

	cfg->irq_config();

	return ret;
}

static void can_sam_isr_0(const struct device *dev)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_reg *can = mcan_cfg->can;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;
    
    can_mcan_line_0_isr(mcan_cfg, msg_ram, mcan_data);
}

static void can_sam_isr_1(const struct device *dev)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_reg *can = mcan_cfg->can;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	can_mcan_line_1_isr(mcan_cfg, msg_ram, mcan_data);
}

int can_sam_set_mode(const struct device *dev, enum can_mode mode)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_set_mode(mcan_cfg, mode);
}

int can_sam_set_timing(const struct device *dev,
			const struct can_timing *timing,
			const struct can_timing *timing_data)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_set_timing(mcan_cfg, timing, timing_data);
}

int can_sam_send(const struct device *dev, const struct zcan_frame *frame,
		  k_timeout_t timeout, can_tx_callback_t callback,
		  void *callback_arg)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	return can_mcan_send(mcan_cfg, mcan_data, msg_ram, frame, timeout,
			     callback, callback_arg);
}

int can_sam_attach_isr(const struct device *dev, can_rx_callback_t isr,
			void *cb_arg, const struct zcan_filter *filter)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	return can_mcan_attach_isr(mcan_data, msg_ram, isr, cb_arg, filter);
}

void can_sam_detach(const struct device *dev, int filter_nr)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	struct can_mcan_data *mcan_data = &DEV_DATA(dev)->mcan_data;
	struct can_mcan_msg_sram *msg_ram = cfg->msg_sram;

	can_mcan_detach(mcan_data, msg_ram, filter_nr);
}

enum can_state can_sam_get_state(const struct device *dev,
				  struct can_bus_err_cnt *err_cnt)
{
	const struct can_sam_config *cfg = DEV_CFG(dev);
	const struct can_mcan_config *mcan_cfg = &cfg->mcan_cfg;

	return can_mcan_get_state(mcan_cfg, err_cnt);
}

static int can_sam_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);
	*rate = SOC_ATMEL_SAM_MCK_FREQ_HZ;
	return 0;
}

static const struct can_driver_api can_api_funcs = {
	.set_mode = can_sam_set_mode,
	.set_timing = can_sam_set_timing,
	.send = can_sam_send,
	.attach_isr = can_sam_attach_isr,
	.detach = can_sam_detach,
	.get_state = can_sam_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_mcan_recover,
#endif
	.get_core_clock = can_sam_get_core_clock,
	.register_state_change_isr = NULL,
	.timing_min = { .sjw = 0x7f,
			.prop_seg = 0x00,
			.phase_seg1 = 0x01,
			.phase_seg2 = 0x01,
			.prescaler = 0x01 },
	.timing_max = { .sjw = 0x7f,
			.prop_seg = 0x00,
			.phase_seg1 = 0x100,
			.phase_seg2 = 0x80,
			.prescaler = 0x200 },
#ifdef CONFIG_CAN_FD_MODE
	.timing_min_data = { .sjw = 0x01,
			     .prop_seg = 0x01,
			     .phase_seg1 = 0x01,
			     .phase_seg2 = 0x01,
			     .prescaler = 0x01 },
	.timing_max_data = { .sjw = 0x10,
			     .prop_seg = 0x00,
			     .phase_seg1 = 0x20,
			     .phase_seg2 = 0x10,
			     .prescaler = 0x20 }
#endif /* CONFIG_CAN_FD_MODE */
};

#ifdef CONFIG_CAN_FD_MODE
#define SAM_MCAN_FD_CONFIG(inst)					  \
	.bus_speed_data = DT_INST_PROP(inst, bus_speed_data),		  \
	.sjw_data = DT_INST_PROP(inst, sjw_data),			  \
	.sample_point_data = DT_INST_PROP_OR(inst, sample_point_data, 0), \
	.prop_ts1_data = DT_INST_PROP_OR(inst, prop_seg_data, 0) +	  \
			 DT_INST_PROP_OR(inst, phase_seg1_data, 0),	  \
	.ts2_data = DT_INST_PROP_OR(inst, phase_seg2_data, 0),
#else /* CONFIG_CAN_FD_MODE */
#define SAM_MCAN_FD_CONFIG(inst)
#endif /* CONFIG_CAN_FD_MODE */

#define SAM_MCAN_INIT(inst)						       \
	DEVICE_DECLARE(CONCAT(can_sam_, inst));			       \
	static void CONCAT(can_sam_irq_config_, inst)()		       \
	{								       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, 0, irq),		       \
			    DT_INST_IRQ_BY_IDX(inst, 0, priority),	       \
			    can_sam_isr0, DEVICE_GET(CONCAT(can_sam_, inst)), \
			    0);						       \
		irq_enable(DT_INST_IRQ_BY_IDX(inst, 0, irq));		       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, 1, irq),		       \
			    DT_INST_IRQ_BY_IDX(inst, 1, priority),	       \
			    can_sam_isr1, DEVICE_GET(CONCAT(can_sam_, inst)), \
			    1);						       \
		irq_enable(DT_INST_IRQ_BY_IDX(inst, 1, irq));		       \
	}								       \
									       \
	struct can_mcan_msg_sram __attribute__((__section__(".can_msg_sram"))) \
	CONCAT(can_sam_msg_sram_, inst) __aligned(4);			       \
	static const struct can_sam_config CONCAT(can_sam_cfg_, inst) = {    \
		.msg_sram = &CONCAT(can_sam_msg_sram_, inst),		       \
		.mcan_cfg = { .can = (struct can_mcan_reg *)		       \
				     DT_INST_REG_ADDR_BY_NAME(inst, m_can),    \
			      .bus_speed = DT_INST_PROP(inst, bus_speed),      \
			      .sjw = DT_INST_PROP(inst, sjw),		       \
			      .sample_point =				       \
				      DT_INST_PROP_OR(inst, sample_point, 0),  \
			      .prop_ts1 =				       \
				      DT_INST_PROP_OR(inst, prop_seg, 0) +     \
				      DT_INST_PROP_OR(inst, phase_seg1, 0),    \
			      .ts2 = DT_INST_PROP_OR(inst, phase_seg2, 0),     \
			      SAM_MCAN_FD_CONFIG(inst) },		       \
		.irq_config = CONCAT(can_sam_irq_config_, inst),	       \
		.periph_id = DT_INST_PROP(inst, peripheral_id),		\
		.num_pins = ATMEL_SAM_DT_NUM_PINS(inst),			\
		.pins = ATMEL_SAM_DT_PINS(inst),				\
	};								       \
	static struct can_sam_data CONCAT(can_sam_data_, inst);	       \
	DEVICE_DT_INST_DEFINE(0, &can_sam_init, device_pm_control_nop,	       \
			      &CONCAT(can_sam_data_, inst),		       \
			      &CONCAT(can_sam_cfg_, inst), POST_KERNEL,       \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &can_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(SAM_MCAN_INIT)