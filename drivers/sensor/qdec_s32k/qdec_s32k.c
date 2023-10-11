/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/arch_interface.h>
#include <zephyr/timing/timing.h>

#define LOG_MODULE_NAME nxp_qdec_s32k
#define INDEX_CHANNEL 22U
#define EMIOS_CW_CHANNEL 6U
#define EMIOS_CCW_CHANNEL 7U
#define COUNTS_PER_ENC_PULSE 4U

LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_SENSOR_LOG_LEVEL);

#include <Emios_Icu_Ip.h>
#include <Trgmux_Ip.h>
#include <Lcu_Ip.h>
#include <Siul2_Port_Ip.h>

#define DT_DRV_COMPAT nxp_qdec_s32k

#define EMIOS_NXP_S32_NODE(n) DT_NODELABEL(emios##n)

struct qdec_s32k_config {
	uint8_t trgmux_inst;
	uint8_t lcu_inst;
	uint8_t emios_inst;
	uint16_t encoder_pulses;
	const struct pinctrl_dev_config *pincfg;
	const eMios_Icu_Ip_ConfigType *emios_config;
	const Trgmux_Ip_InitType *trgmux_config;
	const Lcu_Ip_InitType *lcu_config;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint16_t motor_to_gearbox_ratio;
};

struct qdec_s32k_data {
	uint32_t counterCW;
	uint32_t counterCCW;
	int32_t abs_counter;
	uint16_t counts_per_revolution;
	uint32_t counter_index;
	uint32_t Clockwise_overflow_count;
	uint32_t CounterCW_overflow_count;
	uint32_t ticks_per_sec;
};

static int qdec_s32k_fetch(const struct device *dev, enum sensor_channel ch)
{
	const struct qdec_s32k_config *config = dev->config;
	struct qdec_s32k_data *data = dev->data;

	if (ch != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	static uint32_t prev_time = 0;
	static int32_t prev_abs_count = 0;

	data->counterCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(config->emios_inst, EMIOS_CW_CHANNEL)); /* CW counter */
	data->counterCCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(config->emios_inst, EMIOS_CCW_CHANNEL)); /* CCW counter */
	data->counter_index = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(config->emios_inst, INDEX_CHANNEL)); /* Z or Index Pulse */
	data->Clockwise_overflow_count = Emios_Icu_Ip_GetOverflowCount(config->emios_inst, EMIOS_CW_CHANNEL);
	data->CounterCW_overflow_count = Emios_Icu_Ip_GetOverflowCount(config->emios_inst, EMIOS_CCW_CHANNEL);

	uint32_t curr_time = k_uptime_get();
	uint32_t time_diff_ms = curr_time - prev_time;
	prev_time = curr_time;

 	data->abs_counter = (int32_t)(
		+ (data->counterCW  + (EMIOS_ICU_IP_COUNTER_MASK * data->Clockwise_overflow_count))
		- (data->counterCCW + (EMIOS_ICU_IP_COUNTER_MASK * data->CounterCW_overflow_count)));

	data->ticks_per_sec = abs(data->abs_counter - prev_abs_count) * MSEC_PER_SEC / time_diff_ms;
	prev_abs_count = data->abs_counter;

	uint32 freq_enc_hz = data->ticks_per_sec / COUNTS_PER_ENC_PULSE,
		motor_revs_per_sec = freq_enc_hz / config->encoder_pulses,
		gearbox_revs_per_min = motor_revs_per_sec * 10 * SEC_PER_MIN / config->motor_to_gearbox_ratio,
		wheel_revs_per_min = data->ticks_per_sec * SEC_PER_MIN / data->counts_per_revolution;

	printk("ticks_per_sec = %u, freq_enc_hz = %u, motor_revs_per_sec = %u, gearbox_revs_per_min = %u, wheel_revs_per_min = %u\n",
		data->ticks_per_sec, freq_enc_hz, motor_revs_per_sec, gearbox_revs_per_min, wheel_revs_per_min);
	printk("\nABS_COUNT = %d CW = %u OverFlow_CW = %u CCW = %u Overflow_CCW = %u\n",
		data->abs_counter, data->counterCW, data->Clockwise_overflow_count, data->counterCCW, data->CounterCW_overflow_count);

	return 0;
}

static int qdec_s32k_ch_get(const struct device *dev, enum sensor_channel ch,
	struct sensor_value *val)
{
	struct qdec_s32k_data *data = dev->data;

	switch (ch) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_float(val,
			(data->abs_counter * 360.0f) / data->counts_per_revolution);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api qdec_s32k_api = {
	.sample_fetch = &qdec_s32k_fetch,
	.channel_get = &qdec_s32k_ch_get,
};

static int qdec_s32k_initialize(const struct device *dev)
{
	const struct qdec_s32k_config *config = dev->config;
	struct qdec_s32k_data *data = dev->data;
	uint32_t rate;

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (Trgmux_Ip_Init(config->trgmux_config)) {
		 LOG_ERR("Could not initialize Trgmux");
		return -EINVAL;
	}

	/* Assign eMIOS0_CH6 input to TRGMUX_OUT_37 */
	IP_SIUL2->IMCR[54] = SIUL2_IMCR_SSS(3U);

				/* Assign eMIOS0_CH7 input to TRGMUX_OUT_38 */
	IP_SIUL2->IMCR[55] = SIUL2_IMCR_SSS(4U);

				/* Drive eMIOS0 CH6 and CH7 using FlexIO */
				IP_SIUL2->MSCR[17] = SIUL2_MSCR_SSS(7U);
				IP_SIUL2->MSCR[135] = SIUL2_MSCR_SSS(7U);

	printk("\nqdec_s32k_initialize  SIUL2 IMCR SSS for 53 = 0x%x 54 =0x%x 55 =0x%x MSCR 17 =0x%x 135 "
		"=0x%x\n",
		IP_SIUL2->IMCR[53], IP_SIUL2->IMCR[54], IP_SIUL2->IMCR[55], IP_SIUL2->MSCR[17], IP_SIUL2->MSCR[135] );

	printk("\n TRGMUX ACCESS Input[0] =%d Output[0]=%d lcu_inst = %d\n",
		config->trgmux_config->paxLogicTrigger[0]->Input,
		config->trgmux_config->paxLogicTrigger[0]->Output, config->lcu_inst);

	if (Lcu_Ip_Init(config->lcu_config)) {
		 LOG_ERR("Could not initialize Lcu");
		return -EINVAL;
	}

				/* Unmask relevant LCU OUT Channels */
				Lcu_Ip_SyncOutputValueType EncLcuEnable[4U];
	EncLcuEnable[0].LogicOutputId = LCU_LOGIC_OUTPUT_0;
	EncLcuEnable[0].Value = 1U;
	EncLcuEnable[1].LogicOutputId = LCU_LOGIC_OUTPUT_1;
	EncLcuEnable[1].Value = 1U;
	EncLcuEnable[2].LogicOutputId = LCU_LOGIC_OUTPUT_2;
	EncLcuEnable[2].Value = 1U;
	EncLcuEnable[3].LogicOutputId = LCU_LOGIC_OUTPUT_3;
	EncLcuEnable[3].Value = 1U;
	Lcu_Ip_SetSyncOutputEnable(EncLcuEnable, 4U);

	if (Emios_Icu_Ip_Init(0U, config->emios_config)) {
		LOG_ERR("Could not initialize eMIOS");
		return -EINVAL;
	}

	Emios_Icu_Ip_SetInitialCounterValue(config->emios_inst, EMIOS_CW_CHANNEL, (uint32_t)0x1U);
	Emios_Icu_Ip_SetInitialCounterValue(config->emios_inst, EMIOS_CCW_CHANNEL, (uint32_t)0x1U);
	Emios_Icu_Ip_SetInitialCounterValue(config->emios_inst, INDEX_CHANNEL, (uint32_t)0x1U);

	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst, EMIOS_CW_CHANNEL, EMIOS_ICU_IP_COUNTER_MASK);
	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst, EMIOS_CCW_CHANNEL, EMIOS_ICU_IP_COUNTER_MASK);
	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst, INDEX_CHANNEL, EMIOS_ICU_IP_COUNTER_MASK);

	/* This API sets MCB/EMIOS_ICU_MODE_EDGE_COUNTER mode */
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, EMIOS_CW_CHANNEL);
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, EMIOS_CCW_CHANNEL);
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, INDEX_CHANNEL);

	clock_control_get_rate(config->clock_dev, config->clock_subsys, &rate);
	printk("\n qdec_s32k_initialize Success counts_per_revolution =%d core Clock Rate =%d!! Mask Value =%d\n",
		data->counts_per_revolution, rate, EMIOS_ICU_IP_COUNTER_MASK);
	return 0;
}

#define QDEC_S32K_INIT(n)                                                              	 \
																						 \
	static struct qdec_s32k_data qdec_s32k_##n##_data = {                                \
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution),                 \
		.counterCW = 1,                                                                  \
		.counterCCW = 1,                                                                 \
		.Clockwise_overflow_count = 0,                                                   \
		.CounterCW_overflow_count = 0,                                                   \
	};                                                                                   \
																						 \
	PINCTRL_DT_INST_DEFINE(n);                                                           \
																						 \
	static const struct qdec_s32k_config qdec_s32k_##n##_config = {                      \
		.trgmux_inst = DT_INST_PROP(n, trgmux_inst),                                     \
		.lcu_inst = DT_INST_PROP(n, lcu_inst),                                           \
		.emios_inst = DT_INST_PROP(n, emios_inst),                                       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                     \
		.emios_config = &eMios_Icu_Ip_0_Config_PB_Init,                                  \
		.trgmux_config = &Trgmux_Ip_xTrgmuxInitPB,                                       \
		.lcu_config = &Lcu_Ip_xLcuInitPB,                                                \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(EMIOS_NXP_S32_NODE(n))),               \
		.clock_subsys =                                                                  \
			(clock_control_subsys_t)DT_CLOCKS_CELL(EMIOS_NXP_S32_NODE(n), name),         \
		.encoder_pulses = DT_INST_PROP(n, encoder_pulses),                               \
		.motor_to_gearbox_ratio = DT_INST_PROP(n, motor_to_gearbox_ratio),               \
	};                                                                                   \
																						 \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_s32k_initialize, NULL, &qdec_s32k_##n##_data,	 \
		&qdec_s32k_##n##_config, POST_KERNEL,                                            \
		CONFIG_SENSOR_INIT_PRIORITY, &qdec_s32k_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_S32K_INIT)
