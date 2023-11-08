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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EMIOS_CHANNEL_COUNT 2U
#define EMIOS_CW_CH_IDX     0
#define EMIOS_CCW_CH_IDX    1

/* LCU LUT control values for each of the 4 LC outputs */
/* These values decide the direction of motor rotation */
#define LCU_O0_LUT 0xAAAA
#define LCU_O1_LUT 0xCCCC
#define LCU_O2_LUT 0x4182
#define LCU_O3_LUT 0x2814

LOG_MODULE_REGISTER(nxp_qdec_s32, CONFIG_SENSOR_LOG_LEVEL);

#include <Emios_Icu_Ip.h>
#include <Trgmux_Ip.h>
#include <Lcu_Ip.h>
#include <Siul2_Port_Ip.h>

#define DT_DRV_COMPAT nxp_qdec_s32

#define EMIOS_NXP_S32_NODE(n) DT_NODELABEL(emios##n)

/* Configuration variables from eMIOS Icu driver */
extern eMios_Icu_Ip_ChStateType eMios_Icu_Ip_ChState[EMIOS_ICU_IP_NUM_OF_CHANNELS_USED];
extern uint8 eMios_Icu_Ip_IndexInChState[EMIOS_ICU_IP_INSTANCE_COUNT][EMIOS_ICU_IP_NUM_OF_CHANNELS];

static uint32_t Clockwise_overflow_count, CounterCW_overflow_count;

struct qdec_s32_config {
	uint8_t lcu_inst;
	uint8_t emios_inst;
	uint8_t emios_channels[EMIOS_CHANNEL_COUNT];
	const struct pinctrl_dev_config *pincfg;

	const eMios_Icu_Ip_ConfigType *emios_config;

	const Trgmux_Ip_InitType *trgmux_config;

	const Lcu_Ip_InitType *lcu_config;

	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	size_t trgmux_maps_len;
	int trgmux_maps[];
};

struct qdec_s32_data {
	uint32_t counterCW;
	uint32_t counterCCW;
	int32_t abs_counter;
	float micro_ticks_per_rev;
	uint32_t Clockwise_overflow_count;
	uint32_t CounterCW_overflow_count;
	uint32_t ticks_per_sec;
};

static int qdec_s32_fetch(const struct device *dev, enum sensor_channel ch)
{
	const struct qdec_s32_config *config = dev->config;
	struct qdec_s32_data *data = dev->data;

	if (ch != SENSOR_CHAN_ALL)
		return -ENOTSUP;

	static uint32_t prev_time;

	data->counterCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(
		config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX])); /* CW counter */
	data->counterCCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(
		config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX])); /* CCW counter */

	data->Clockwise_overflow_count = Clockwise_overflow_count;
	data->CounterCW_overflow_count = CounterCW_overflow_count;

	uint32_t curr_time = k_uptime_get();

	prev_time = curr_time;

	data->abs_counter = (int32_t)(
		+(data->counterCW  + (EMIOS_ICU_IP_COUNTER_MASK * data->Clockwise_overflow_count))
		-(data->counterCCW + (EMIOS_ICU_IP_COUNTER_MASK * data->CounterCW_overflow_count)));

	LOG_DBG("ABS_COUNT = %d CW = %u OverFlow_CW = %u CCW = %u Overflow_CCW = %u",
		data->abs_counter, data->counterCW, data->Clockwise_overflow_count,
		data->counterCCW, data->CounterCW_overflow_count);

	return 0;
}

static int qdec_s32_ch_get(const struct device *dev, enum sensor_channel ch,
		struct sensor_value *val)
{
	struct qdec_s32_data *data = dev->data;

	double rotation = (data->abs_counter * 2.0 * M_PI) / data->micro_ticks_per_rev;

	switch (ch) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_double(val, rotation);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void eMios_Overflow_Count_CW(void)
{
	Clockwise_overflow_count++;
}

static void eMios_Overflow_Count_CCW(void)
{
	CounterCW_overflow_count++;
}

static const struct sensor_driver_api qdec_s32_api = {
	.sample_fetch = &qdec_s32_fetch,
	.channel_get = &qdec_s32_ch_get,
};

static int qdec_s32_initialize(const struct device *dev)
{
	const struct qdec_s32_config *config = dev->config;
	uint32_t rate;
	uint8_t emios_inst;
	uint8_t emios_hw_ch_cw, emios_hw_ch_ccw;

	if (config->trgmux_maps_len != 4 || config->trgmux_maps[0] > SIUL2_MAX_NUM_OF_IMCR_REG ||
			config->trgmux_maps[2] > SIUL2_MAX_NUM_OF_IMCR_REG) {
		LOG_ERR("Wrong Trgmux config");
		return -EINVAL;
	}

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (Trgmux_Ip_Init(config->trgmux_config)) {
		LOG_ERR("Could not initialize Trgmux");
		return -EINVAL;
	}

	LOG_DBG("TRGMUX ACCESS Input[0] =%d Output[0]=%d",
		config->trgmux_config->paxLogicTrigger[0]->Input,
		config->trgmux_config->paxLogicTrigger[0]->Output);

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

	emios_inst = config->emios_inst;
	emios_hw_ch_cw =  config->emios_channels[EMIOS_CW_CH_IDX];
	emios_hw_ch_ccw =  config->emios_channels[EMIOS_CCW_CH_IDX];

	/* Initialize the positions of the eMios hw channels used for QDEC
         * to be beyond the eMios pwm hw channels. Currently only pwm and qdec
         * are using the eMios channels so qdec ones are the last two.
         */
        eMios_Icu_Ip_IndexInChState[emios_inst][emios_hw_ch_cw]
                = EMIOS_ICU_IP_NUM_OF_CHANNELS_USED - 2;
        eMios_Icu_Ip_IndexInChState[emios_inst][emios_hw_ch_ccw]
                = EMIOS_ICU_IP_NUM_OF_CHANNELS_USED - 1;

	/* Set Overflow Notification for eMIOS channels meant
	 * for Clockwise and Counterclock rotation counters
	 */
	eMios_Icu_Ip_ChState[eMios_Icu_Ip_IndexInChState[emios_inst][emios_hw_ch_cw]]
		.eMiosOverflowNotification = &eMios_Overflow_Count_CW;
	eMios_Icu_Ip_ChState[eMios_Icu_Ip_IndexInChState[emios_inst][emios_hw_ch_ccw]]
		.eMiosOverflowNotification = &eMios_Overflow_Count_CCW;

	Emios_Icu_Ip_SetInitialCounterValue(
		config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX], (uint32_t)0x1U);
	Emios_Icu_Ip_SetInitialCounterValue(
		config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX], (uint32_t)0x1U);

	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst,
		config->emios_channels[EMIOS_CW_CH_IDX],
		EMIOS_ICU_IP_COUNTER_MASK);
	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst,
		config->emios_channels[EMIOS_CCW_CH_IDX],
		EMIOS_ICU_IP_COUNTER_MASK);

	/* This API sets MCB/EMIOS_ICU_MODE_EDGE_COUNTER mode */
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX]);
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX]);

	clock_control_get_rate(config->clock_dev, config->clock_subsys, &rate);
	return 0;
}

#define EMIOS_NXP_S32_INSTANCE_CHECK(idx, node_id)						\
	((DT_REG_ADDR(node_id) == IP_EMIOS_##idx##_BASE) ? idx : 0)

#define EMIOS_NXP_S32_GET_INSTANCE(node_id)							\
	LISTIFY(__DEBRACKET eMIOS_INSTANCE_COUNT, EMIOS_NXP_S32_INSTANCE_CHECK, (|), node_id)

/* LCU Logic Input Configuration */
#define LogicInputCfg_Common(mux_sel)								   \
	{											   \
		.MuxSel = mux_sel,								   \
		.SwSynMode = LCU_IP_SW_SYNC_IMMEDIATE,						   \
		.SwValue = LCU_IP_SW_OVERRIDE_LOGIC_LOW,					   \
	};
#define LogicInput_Config_Common(n, hw_lc_input_id, logic_input_n_cfg)				   \
	{											   \
		.xLogicInputId = {								   \
			.HwInstId = DT_INST_PROP(n, lcu_inst),					   \
			.HwLcInputId = hw_lc_input_id,						   \
		},										   \
		.pxLcInputConfig = &logic_input_n_cfg,						   \
	};

/* LCU Logic Output Configuration */
#define LogicOutputCfg_Common(En_Debug_Mode, Lut_Control, Lut_Rise_Filt, Lut_Fall_Filt)		   \
	{											   \
		.EnDebugMode = (boolean)En_Debug_Mode,						   \
		.LutControl = Lut_Control,							   \
		.LutRiseFilt = Lut_Rise_Filt,							   \
		.LutFallFilt = Lut_Fall_Filt,							   \
		.EnLutDma = (boolean)FALSE,							   \
		.EnForceDma = (boolean)FALSE,							   \
		.EnLutInt = (boolean)FALSE,							   \
		.EnForceInt = (boolean)FALSE,							   \
		.InvertOutput = (boolean)FALSE,							   \
		.ForceSignalSel = 0U,								   \
		.ClearForceMode = LCU_IP_CLEAR_FORCE_SIGNAL_IMMEDIATE,				   \
		.ForceSyncSel = LCU_IP_SYNC_SEL_INPUT0,						   \
	};
#define LogicOutput_Config_Common(n, logic_output_cfg, hw_lc_output_id)				   \
	{											   \
		.xLogicOutputId = {								   \
			.HwInstId = DT_INST_PROP(n, lcu_inst),					   \
			.HwLcOutputId = hw_lc_output_id,					   \
			.IntCallback = NULL_PTR,						   \
		},										   \
		.pxLcOutputConfig = &logic_output_cfg,						   \
	};

#define LCU_IP_INIT_CONFIG(n)									\
	const Lcu_Ip_LogicInputConfigType LogicInput0Cfg =					\
		LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_IN_0)					\
	const Lcu_Ip_LogicInputConfigType LogicInput1Cfg =					\
		LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_IN_1)					\
	const Lcu_Ip_LogicInputConfigType LogicInput2Cfg =					\
		LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_OUT_0)					\
	const Lcu_Ip_LogicInputConfigType LogicInput3Cfg =					\
		LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_OUT_1)					\
												\
	const Lcu_Ip_LogicInputType LogicInput0_Config =					\
		LogicInput_Config_Common(n, 0U, LogicInput0Cfg)					\
	const Lcu_Ip_LogicInputType LogicInput1_Config =					\
		LogicInput_Config_Common(n, 1U, LogicInput1Cfg)					\
	const Lcu_Ip_LogicInputType LogicInput2_Config =					\
		LogicInput_Config_Common(n, 2U, LogicInput2Cfg)					\
	const Lcu_Ip_LogicInputType LogicInput3_Config =					\
		LogicInput_Config_Common(n, 3U, LogicInput3Cfg)					\
												\
	const Lcu_Ip_LogicInputType								\
		*const Lcu_Ip_ppxLogicInputArray_Config[LCU_IP_NOF_CFG_LOGIC_INPUTS] = {	\
			&LogicInput0_Config,							\
			&LogicInput1_Config,							\
			&LogicInput2_Config,							\
			&LogicInput3_Config,							\
		};										\
												\
	const Lcu_Ip_LogicOutputConfigType LogicOutput0Cfg = LogicOutputCfg_Common(		\
		LCU_IP_DEBUG_DISABLE, LCU_O0_LUT,						\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 1),				\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 2))				\
	const Lcu_Ip_LogicOutputConfigType LogicOutput1Cfg = LogicOutputCfg_Common(		\
		LCU_IP_DEBUG_DISABLE, LCU_O1_LUT,						\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 4),				\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 5))				\
	const Lcu_Ip_LogicOutputConfigType LogicOutput2Cfg = LogicOutputCfg_Common(		\
		LCU_IP_DEBUG_ENABLE, LCU_O2_LUT,						\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 7),				\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 8))				\
	const Lcu_Ip_LogicOutputConfigType LogicOutput3Cfg = LogicOutputCfg_Common(		\
		LCU_IP_DEBUG_ENABLE, LCU_O3_LUT,						\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 10),				\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 11))				\
												\
	const Lcu_Ip_LogicOutputType LogicOutput0_Config =					\
		LogicOutput_Config_Common(n, LogicOutput0Cfg,					\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 0))				\
	const Lcu_Ip_LogicOutputType LogicOutput1_Config =					\
		LogicOutput_Config_Common(n, LogicOutput1Cfg,					\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 3))				\
	const Lcu_Ip_LogicOutputType LogicOutput2_Config =					\
		LogicOutput_Config_Common(n, LogicOutput2Cfg,					\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 6))				\
	const Lcu_Ip_LogicOutputType LogicOutput3_Config =					\
		LogicOutput_Config_Common(n, LogicOutput3Cfg,					\
		DT_INST_PROP_BY_IDX(n, lcu_output_filter_config, 9))				\
												\
	const Lcu_Ip_LogicOutputType								\
		*const Lcu_Ip_ppxLogicOutputArray_Config[LCU_IP_NOF_CFG_LOGIC_OUTPUTS] = {	\
			&LogicOutput0_Config,							\
			&LogicOutput1_Config,							\
			&LogicOutput2_Config,							\
			&LogicOutput3_Config,							\
		};										\
												\
	const Lcu_Ip_LogicInputConfigType Lcu_Ip_LogicInputResetConfig = {			\
		.MuxSel = LCU_IP_MUX_SEL_LOGIC_0,						\
		.SwSynMode = LCU_IP_SW_SYNC_IMMEDIATE,						\
		.SwValue = LCU_IP_SW_OVERRIDE_LOGIC_LOW,					\
	};											\
												\
	const Lcu_Ip_LogicOutputConfigType Lcu_Ip_LogicOutputResetConfig =			\
		LogicOutputCfg_Common(LCU_IP_DEBUG_DISABLE, 0U, 0U, 0U)				\
												\
	const Lcu_Ip_LogicInstanceType LcuLogicInstance0Config = {				\
		.HwInstId = DT_INST_PROP(n, lcu_inst),						\
		.NumLogicCellConfig = 0U,							\
		.ppxLogicCellConfigArray = NULL_PTR,						\
		.OperationMode = LCU_IP_INTERRUPT_MODE,						\
	};											\
	const Lcu_Ip_LogicInstanceType								\
		*const Lcu_Ip_ppxLogicInstanceArray_Config[LCU_IP_NOF_CFG_LOGIC_INSTANCES] = {	\
			&LcuLogicInstance0Config,						\
		};										\
												\
	Lcu_Ip_HwOutputStateType HwOutput0State_Config;						\
	Lcu_Ip_HwOutputStateType HwOutput1State_Config;						\
	Lcu_Ip_HwOutputStateType HwOutput2State_Config;						\
	Lcu_Ip_HwOutputStateType HwOutput3State_Config;						\
	Lcu_Ip_HwOutputStateType								\
		*Lcu_Ip_ppxHwOutputStateArray_Config[LCU_IP_NOF_CFG_LOGIC_OUTPUTS] = {		\
			&HwOutput0State_Config,							\
			&HwOutput1State_Config,							\
			&HwOutput2State_Config,							\
			&HwOutput3State_Config,							\
	};											\
												\
	const Lcu_Ip_InitType Lcu_Ip_Init_Config = {						\
		.ppxHwOutputStateArray = &Lcu_Ip_ppxHwOutputStateArray_Config[0],		\
		.ppxLogicInstanceConfigArray = &Lcu_Ip_ppxLogicInstanceArray_Config[0],		\
		.pxLogicOutputResetConfigArray = &Lcu_Ip_LogicOutputResetConfig,		\
		.pxLogicInputResetConfigArray = &Lcu_Ip_LogicInputResetConfig,			\
		.ppxLogicOutputConfigArray = &Lcu_Ip_ppxLogicOutputArray_Config[0],		\
		.ppxLogicInputConfigArray = &Lcu_Ip_ppxLogicInputArray_Config[0],		\
	};

#define Trgmux_Ip_LogicTrigger_Config(n, logic_channel, output, input)				\
	{											\
		.LogicChannel = logic_channel,							\
		.Output = output,								\
		.Input = input,									\
		.HwInstId = DT_INST_PROP(n, trgmux_inst),					\
		.Lock = (boolean)FALSE,								\
	};

#define TRGMUX_IP_INIT_CONFIG(n)								\
	const Trgmux_Ip_LogicTriggerType							\
		Trgmux_Ip_LogicTrigger_0_Config = Trgmux_Ip_LogicTrigger_Config(n,		\
		DT_INST_PROP_BY_IDX(n, trgmux_io_config, 0),					\
		DT_INST_PROP_BY_IDX(n, trgmux_io_config, 1),					\
		DT_INST_PROP_BY_IDX(n, trgmux_io_config, 2))					\
	const Trgmux_Ip_LogicTriggerType							\
		Trgmux_Ip_LogicTrigger_1_Config = Trgmux_Ip_LogicTrigger_Config(n,		\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 3),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 4),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 5))				\
	const Trgmux_Ip_LogicTriggerType							\
		Trgmux_Ip_LogicTrigger_2_Config = Trgmux_Ip_LogicTrigger_Config(n,		\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 6),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 7),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 8))				\
	const Trgmux_Ip_LogicTriggerType							\
		Trgmux_Ip_LogicTrigger_3_Config = Trgmux_Ip_LogicTrigger_Config(n,		\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 9),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 10),				\
			DT_INST_PROP_BY_IDX(n, trgmux_io_config, 11))				\
	const Trgmux_Ip_InitType Trgmux_Ip_Init_Config = {					\
		.paxLogicTrigger = {								\
			&Trgmux_Ip_LogicTrigger_0_Config,					\
			&Trgmux_Ip_LogicTrigger_1_Config,					\
			&Trgmux_Ip_LogicTrigger_2_Config,					\
			&Trgmux_Ip_LogicTrigger_3_Config,					\
		},										\
	};

#define eMios_Icu_Ip_ChannelConfig_Common(channel) {						\
	.hwChannel = channel,									\
	.ucMode = EMIOS_ICU_SAIC,								\
	.FreezeEn = (boolean)FALSE,								\
	.Prescaler = EMIOS_PRESCALER_DIVIDE_1,							\
	.AltPrescaler = EMIOS_PRESCALER_DIVIDE_1,						\
	.CntBus = EMIOS_ICU_BUS_INTERNAL_COUNTER,						\
	.chMode = EMIOS_ICU_MODE_EDGE_COUNTER,							\
	.chSubMode = EMIOS_ICU_MODE_WITHOUT_DMA,						\
	.measurementMode = EMIOS_ICU_NO_MEASUREMENT,						\
	.edgeAlignement = EMIOS_ICU_RISING_EDGE,						\
	.Filter = EMIOS_DIGITAL_FILTER_BYPASSED,						\
	.callback = NULL_PTR,									\
	.logicChStateCallback = NULL_PTR,							\
	.callbackParams = (uint8)255U,								\
	.bWithoutInterrupt = (boolean)FALSE,							\
	.eMiosChannelNotification = NULL_PTR,							\
	.eMiosOverflowNotification = NULL_PTR,							\
}

#define EMIOS_ICU_IP_CONFIG(n)									\
	const eMios_Icu_Ip_ChannelConfigType eMios_Icu_Ip_ChannelConfig[EMIOS_CHANNEL_COUNT] = {\
		eMios_Icu_Ip_ChannelConfig_Common(						\
			DT_INST_PROP_BY_IDX(n, emios_channels, EMIOS_CW_CH_IDX)),		\
		eMios_Icu_Ip_ChannelConfig_Common(						\
			DT_INST_PROP_BY_IDX(n, emios_channels, EMIOS_CCW_CH_IDX)),		\
	};											\
	const eMios_Icu_Ip_ConfigType eMios_Icu_Ip_Config = {					\
		.nNumChannels = EMIOS_CHANNEL_COUNT,						\
		.pChannelsConfig = &eMios_Icu_Ip_ChannelConfig,					\
	};

#define QDEC_NXP_S32_INIT(n)									\
												\
	static struct qdec_s32_data qdec_s32_##n##_data = {					\
		.micro_ticks_per_rev = (float)(DT_INST_PROP(n, micro_ticks_per_rev) / 1000000), \
		.counterCW = 1,									\
		.counterCCW = 1,								\
		.Clockwise_overflow_count = 0,							\
		.CounterCW_overflow_count = 0,							\
	};											\
												\
	PINCTRL_DT_INST_DEFINE(n);								\
	EMIOS_ICU_IP_CONFIG(n)									\
	TRGMUX_IP_INIT_CONFIG(n)								\
	LCU_IP_INIT_CONFIG(n)									\
												\
	static const struct qdec_s32_config qdec_s32_##n##_config = {				\
		.emios_inst = EMIOS_NXP_S32_GET_INSTANCE(DT_INST_PHANDLE(n, emios)),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.emios_config = &eMios_Icu_Ip_Config,						\
		.trgmux_config = &Trgmux_Ip_Init_Config,					\
		.lcu_config = &Lcu_Ip_Init_Config,						\
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(EMIOS_NXP_S32_NODE(n))),		\
		.clock_subsys =									\
			(clock_control_subsys_t)DT_CLOCKS_CELL(EMIOS_NXP_S32_NODE(n), name),	\
		.emios_channels = {DT_INST_PROP_BY_IDX(n, emios_channels, EMIOS_CW_CH_IDX),	\
				   DT_INST_PROP_BY_IDX(n, emios_channels, EMIOS_CCW_CH_IDX)},	\
		.trgmux_maps_len = DT_INST_PROP_LEN(n, trgmux_maps),				\
		.trgmux_maps = DT_INST_PROP(n, trgmux_maps),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_s32_initialize, NULL, &qdec_s32_##n##_data,	\
				     &qdec_s32_##n##_config, POST_KERNEL,			\
				     CONFIG_SENSOR_INIT_PRIORITY, &qdec_s32_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_NXP_S32_INIT)
