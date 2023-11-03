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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EMIOS_CHANNEL_COUNT 2U
#define EMIOS_CW_CH_IDX 0U
#define EMIOS_CCW_CH_IDX 1U

/* LCU LUT control values for each of the 4 LC outputs */
/* These values decide the direction of motor rotation */
#define LCU_O0_LUT 0xAAAA
#define LCU_O1_LUT 0xCCCC
#define LCU_O2_LUT 0x4182
#define LCU_O3_LUT 0x2814

LOG_MODULE_REGISTER(qdec_s32k, CONFIG_SENSOR_LOG_LEVEL);

#include <Emios_Icu_Ip.h>
#include <Trgmux_Ip.h>
#include <Lcu_Ip.h>
#include <Siul2_Port_Ip.h>

#define DT_DRV_COMPAT nxp_qdec_s32k

#define EMIOS_NXP_S32_NODE(n) DT_NODELABEL(emios##n)

struct qdec_s32k_config {
	uint8_t lcu_inst;
	uint8_t emios_inst;
	uint8_t emios_channels[EMIOS_CHANNEL_COUNT];
	uint16_t lcu_rise_fall_filter;
	uint16_t lcu_glitch_filter;
	const struct pinctrl_dev_config *pincfg;
	const eMios_Icu_Ip_ConfigType *emios_config;
	const Trgmux_Ip_InitType *trgmux_config;
	const Lcu_Ip_InitType *lcu_config;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	size_t trgmux_maps_len;
	int trgmux_maps[];
};

struct qdec_s32k_data {
	uint32_t counterCW;
	uint32_t counterCCW;
	int32_t abs_counter;
	float micro_ticks_per_rev;
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

	data->counterCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX])); /* CW counter */
	data->counterCCW = (uint32_t)(Emios_Icu_Ip_GetEdgeNumbers(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX])); /* CCW counter */
	data->Clockwise_overflow_count = Emios_Icu_Ip_GetOverflowCount(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX]); /* CW Counter Overflow Counter */
	data->CounterCW_overflow_count = Emios_Icu_Ip_GetOverflowCount(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX]); /* CCW Counter Overflow Counter */

	uint32_t curr_time = k_uptime_get();
	prev_time = curr_time;

 	data->abs_counter = (int32_t)(
		+ (data->counterCW  + (EMIOS_ICU_IP_COUNTER_MASK * data->Clockwise_overflow_count))
		- (data->counterCCW + (EMIOS_ICU_IP_COUNTER_MASK * data->CounterCW_overflow_count)));

	LOG_DBG("ABS_COUNT = %d CW = %u OverFlow_CW = %u CCW = %u Overflow_CCW = %u",
		data->abs_counter, data->counterCW, data->Clockwise_overflow_count, data->counterCCW, data->CounterCW_overflow_count);

	return 0;
}

static int qdec_s32k_ch_get(const struct device *dev, enum sensor_channel ch,
	struct sensor_value *val)
{
	struct qdec_s32k_data *data = dev->data;

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

static const struct sensor_driver_api qdec_s32k_api = {
	.sample_fetch = &qdec_s32k_fetch,
	.channel_get = &qdec_s32k_ch_get,
};

static int qdec_s32k_initialize(const struct device *dev)
{
	const struct qdec_s32k_config *config = dev->config;
	uint32_t rate;

	if(config->trgmux_maps_len != 4 ||
	   config->trgmux_maps[0] > SIUL2_MAX_NUM_OF_IMCR_REG ||
	   config->trgmux_maps[2] > SIUL2_MAX_NUM_OF_IMCR_REG) {
		LOG_ERR("Wrong Trgmux config");
		return -EINVAL;
	}

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (Trgmux_Ip_Init(config->trgmux_config)) {
		LOG_ERR("Could not initialize Trgmux");
		return -EINVAL;
	}

	/* Assign eMIOS0 input to TRGMUX_OUT */
	IP_SIUL2->IMCR[config->trgmux_maps[0]] = SIUL2_IMCR_SSS(config->trgmux_maps[1]);

	/* Assign eMIOS0 input to TRGMUX_OUT */
	IP_SIUL2->IMCR[config->trgmux_maps[2]] = SIUL2_IMCR_SSS(config->trgmux_maps[3]);

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

	Emios_Icu_Ip_SetInitialCounterValue(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX], (uint32_t)0x1U);
	Emios_Icu_Ip_SetInitialCounterValue(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX], (uint32_t)0x1U);

	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX], EMIOS_ICU_IP_COUNTER_MASK);
	Emios_Icu_Ip_SetMaxCounterValue(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX], EMIOS_ICU_IP_COUNTER_MASK);

	/* This API sets MCB/EMIOS_ICU_MODE_EDGE_COUNTER mode */
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, config->emios_channels[EMIOS_CW_CH_IDX]);
	Emios_Icu_Ip_EnableEdgeCount(config->emios_inst, config->emios_channels[EMIOS_CCW_CH_IDX]);

	clock_control_get_rate(config->clock_dev, config->clock_subsys, &rate);
	return 0;
}

/* LCU Logic Input Configuration */
#define LogicInputCfg_Common(MuxSel)		\
	{		\
		/* uint8 MuxSel         */ MuxSel,		\
		/* boolean SwSynMode    */ LCU_IP_SW_SYNC_IMMEDIATE,		\
		/* uint8 SwValue        */ LCU_IP_SW_OVERRIDE_LOGIC_LOW,		\
	};
#define LogicInput_Config_Common(HwLcInputId, LogicInputnCfg)		\
	{		\
		{		\
			/* uint8 HwInstId      */ LCU_IP_HW_INST_1,		\
			/* uint8 HwLcInputId   */ HwLcInputId,		\
		},		\
		&LogicInputnCfg,		\
	};

/* LCU Logic Output Configuration */
#define LogicOutputCfg_Common(EnDebugMode, LutControl, LutRiseFilt, LutFallFilt)	\
	{		\
		/* boolean EnDebugMode   */ (boolean)EnDebugMode,		\
		/* uint16 LutControl     */ LutControl,			\
		/* uint16 LutRiseFilt    */ LutRiseFilt,		\
		/* uint16 LutFallFilt    */ LutFallFilt,		\
		/* boolean EnLutDma      */ (boolean)FALSE,		\
		/* boolean EnForceDma    */ (boolean)FALSE,		\
		/* boolean EnLutInt      */ (boolean)FALSE,		\
		/* boolean EnForceInt    */ (boolean)FALSE,		\
		/* boolean InvertOutput  */ (boolean)FALSE,		\
		/* uint8 ForceSignalSel  */ 0U,		\
		/* uint8 ClearForceMode  */ LCU_IP_CLEAR_FORCE_SIGNAL_IMMEDIATE,		\
		/* uint8 ForceSyncSel    */ LCU_IP_SYNC_SEL_INPUT0,		\
	};
#define LogicOutput_Config_Common(LogicOutputCfg, HwLcOutputId)		\
	{		\
		{		\
			/* uint8 HwInstId               */ LCU_IP_HW_INST_1,		\
			/* uint8 HwLcOutputId           */ HwLcOutputId,		\
			/* Lcu_Ip_Callback IntCallback  */ NULL_PTR,		\
		},		\
		&LogicOutputCfg,		\
	};

#define LCU_IP_INIT_CONFIG(n)	\
	const uint16_t lcu_rise_fall_filter = DT_INST_PROP(n, lcu_rise_fall_filter);		\
	const uint16_t lcu_glitch_filter = DT_INST_PROP(n, lcu_glitch_filter);		\
	const Lcu_Ip_LogicInputConfigType LogicInput0Cfg = LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_IN_0)		\
	const Lcu_Ip_LogicInputConfigType LogicInput1Cfg = LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_IN_1)		\
	const Lcu_Ip_LogicInputConfigType LogicInput2Cfg = LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_OUT_0)		\
	const Lcu_Ip_LogicInputConfigType LogicInput3Cfg = LogicInputCfg_Common(LCU_IP_MUX_SEL_LU_OUT_1)		\
																											\
	const Lcu_Ip_LogicInputType LogicInput0_Config = LogicInput_Config_Common(0U, LogicInput0Cfg)		\
	const Lcu_Ip_LogicInputType LogicInput1_Config = LogicInput_Config_Common(1U, LogicInput1Cfg)		\
	const Lcu_Ip_LogicInputType LogicInput2_Config = LogicInput_Config_Common(2U, LogicInput2Cfg)		\
	const Lcu_Ip_LogicInputType LogicInput3_Config = LogicInput_Config_Common(3U, LogicInput3Cfg)		\
																											\
	const Lcu_Ip_LogicInputType * const Lcu_Ip_ppxLogicInputArray_Config[LCU_IP_NOF_CFG_LOGIC_INPUTS] = {		\
		&LogicInput0_Config,		\
		&LogicInput1_Config,		\
		&LogicInput2_Config,		\
		&LogicInput3_Config,		\
	};		\
																											\
	const Lcu_Ip_LogicOutputConfigType LogicOutput0Cfg = LogicOutputCfg_Common(LCU_IP_DEBUG_DISABLE, LCU_O0_LUT, lcu_rise_fall_filter, lcu_rise_fall_filter)	\
	const Lcu_Ip_LogicOutputConfigType LogicOutput1Cfg = LogicOutputCfg_Common(LCU_IP_DEBUG_DISABLE, LCU_O1_LUT, lcu_rise_fall_filter, lcu_rise_fall_filter)	\
	const Lcu_Ip_LogicOutputConfigType LogicOutput2Cfg = LogicOutputCfg_Common(LCU_IP_DEBUG_ENABLE, LCU_O2_LUT, lcu_glitch_filter, lcu_glitch_filter)	\
	const Lcu_Ip_LogicOutputConfigType LogicOutput3Cfg = LogicOutputCfg_Common(LCU_IP_DEBUG_ENABLE, LCU_O3_LUT, lcu_glitch_filter, lcu_glitch_filter)	\
																											\
	const Lcu_Ip_LogicOutputType LogicOutput0_Config = LogicOutput_Config_Common(LogicOutput0Cfg, 0U)	\
	const Lcu_Ip_LogicOutputType LogicOutput1_Config = LogicOutput_Config_Common(LogicOutput1Cfg, 1U)	\
	const Lcu_Ip_LogicOutputType LogicOutput2_Config = LogicOutput_Config_Common(LogicOutput2Cfg, 2U)	\
	const Lcu_Ip_LogicOutputType LogicOutput3_Config = LogicOutput_Config_Common(LogicOutput3Cfg, 3U)	\
																											\
	const Lcu_Ip_LogicOutputType * const Lcu_Ip_ppxLogicOutputArray_Config[LCU_IP_NOF_CFG_LOGIC_OUTPUTS] = {	\
		&LogicOutput0_Config,	\
		&LogicOutput1_Config,	\
		&LogicOutput2_Config,	\
		&LogicOutput3_Config,	\
	};	\
																											\
	const Lcu_Ip_LogicInputConfigType Lcu_Ip_LogicInputResetConfig = {			\
		/* uint8 MuxSel         */ LCU_IP_MUX_SEL_LOGIC_0,			\
		/* boolean SwSynMode    */ LCU_IP_SW_SYNC_IMMEDIATE,			\
		/* uint8 SwValue        */ LCU_IP_SW_OVERRIDE_LOGIC_LOW,			\
	};			\
																											\
	const Lcu_Ip_LogicOutputConfigType Lcu_Ip_LogicOutputResetConfig = LogicOutputCfg_Common(LCU_IP_DEBUG_DISABLE, 0U, 0U, 0U)	\
																											\
	const Lcu_Ip_LogicInstanceType LcuLogicInstance0Config = {							\
		/* uint8 HwInstId             */ LCU_IP_HW_INST_1,							\
		/* uint8 NumLogicCellConfig   */ 0U,							\
		/* ppxLogicCellConfigArray    */ NULL_PTR,							\
		/* uint8 OperationMode        */ LCU_IP_INTERRUPT_MODE,							\
	};							\
	const Lcu_Ip_LogicInstanceType * const Lcu_Ip_ppxLogicInstanceArray_Config[LCU_IP_NOF_CFG_LOGIC_INSTANCES] = {		\
		&LcuLogicInstance0Config,		\
	};		\
																											\
	Lcu_Ip_HwOutputStateType HwOutput0State_Config;		\
	Lcu_Ip_HwOutputStateType HwOutput1State_Config;		\
	Lcu_Ip_HwOutputStateType HwOutput2State_Config;		\
	Lcu_Ip_HwOutputStateType HwOutput3State_Config;		\
	Lcu_Ip_HwOutputStateType * Lcu_Ip_ppxHwOutputStateArray_Config[LCU_IP_NOF_CFG_LOGIC_OUTPUTS] = {		\
		&HwOutput0State_Config,		\
		&HwOutput1State_Config,		\
		&HwOutput2State_Config,		\
		&HwOutput3State_Config,		\
	};		\
																											\
	const Lcu_Ip_InitType Lcu_Ip_Init_Config = {	\
		/* Lcu_Ip_HwOutputStateType ** ppxHwOutputStateArray;                       */ &Lcu_Ip_ppxHwOutputStateArray_Config[0],	\
		/* const Lcu_Ip_LogicInstanceType * const * ppxLogicInstanceConfigArray     */ &Lcu_Ip_ppxLogicInstanceArray_Config[0],	\
		/* const Lcu_Ip_LogicOutputConfigType * const pxLogicOutputResetConfigArray */ &Lcu_Ip_LogicOutputResetConfig,		\
		/* const Lcu_Ip_LogicInputConfigType * const pxLogicInputResetConfigArray   */ &Lcu_Ip_LogicInputResetConfig,	\
		/* const Lcu_Ip_LogicOutputType * const * ppxLogicOutputConfigArray         */ &Lcu_Ip_ppxLogicOutputArray_Config[0],	\
		/* const Lcu_Ip_LogicInputType * const * ppxLogicInputConfigArray           */ &Lcu_Ip_ppxLogicInputArray_Config[0],		\
	};

#define Trgmux_Ip_LogicTrigger_Config(LogicChannel, Output, Intput)		\
	{																			\
		/* uint8 LogicChannel; */   LogicChannel,								\
		/* uint8 Output; */         Output,										\
		/* uint8 Input; */          Intput,										\
		/* uint8 HwInstId; */       TRGMUX_IP_HW_INST_0,						\
		/* boolean Lock; */         (boolean)FALSE,								\
	};

#define TRGMUX_IP_INIT_CONFIG(n)											\
	const Trgmux_Ip_LogicTriggerType Trgmux_Ip_LogicTrigger_0_Config = Trgmux_Ip_LogicTrigger_Config	\
		(TRGMUX_LOGIC_GROUP_0_TRIGGER_0, TRGMUX_IP_OUTPUT_EMIOS0_CH5_9_IPP_IND_CH6, TRGMUX_IP_INPUT_LCU1_LC0_OUT_I2)	\
	const Trgmux_Ip_LogicTriggerType Trgmux_Ip_LogicTrigger_1_Config = Trgmux_Ip_LogicTrigger_Config	\
		(TRGMUX_LOGIC_GROUP_0_TRIGGER_1, TRGMUX_IP_OUTPUT_EMIOS0_CH5_9_IPP_IND_CH7, TRGMUX_IP_INPUT_LCU1_LC0_OUT_I3)	\
	const Trgmux_Ip_LogicTriggerType Trgmux_Ip_LogicTrigger_2_Config = Trgmux_Ip_LogicTrigger_Config	\
		(TRGMUX_LOGIC_GROUP_1_TRIGGER_0, TRGMUX_IP_OUTPUT_LCU1_0_INP_I0, TRGMUX_IP_INPUT_SIUL2_IN2)		\
	const Trgmux_Ip_LogicTriggerType Trgmux_Ip_LogicTrigger_3_Config = Trgmux_Ip_LogicTrigger_Config	\
		(TRGMUX_LOGIC_GROUP_1_TRIGGER_1, TRGMUX_IP_OUTPUT_LCU1_0_INP_I1, TRGMUX_IP_INPUT_SIUL2_IN3)		\
	const Trgmux_Ip_InitType Trgmux_Ip_Init_Config = {						\
		{	\
			&Trgmux_Ip_LogicTrigger_0_Config,	\
			&Trgmux_Ip_LogicTrigger_1_Config,	\
			&Trgmux_Ip_LogicTrigger_2_Config,	\
			&Trgmux_Ip_LogicTrigger_3_Config,	\
		},	\
	};

#define eMios_Icu_Ip_ChannelConfig_Common(channel) \
	{	\
		channel,	\
		EMIOS_ICU_SAIC,	\
		(boolean)FALSE,	\
		EMIOS_PRESCALER_DIVIDE_1,	\
		EMIOS_PRESCALER_DIVIDE_1,	\
		EMIOS_ICU_BUS_INTERNAL_COUNTER,	\
		EMIOS_ICU_MODE_EDGE_COUNTER,	\
		EMIOS_ICU_MODE_WITHOUT_DMA,	\
		EMIOS_ICU_NO_MEASUREMENT,	\
		EMIOS_ICU_RISING_EDGE,	\
		EMIOS_DIGITAL_FILTER_BYPASSED,	\
		NULL_PTR,	\
		NULL_PTR,	\
		(uint8)255U,	\
		(boolean)FALSE,	\
		.eMiosChannelNotification = NULL_PTR,	\
		.eMiosOverflowNotification = NULL_PTR,	\
	}

#define EMIOS_ICU_IP_CONFIG(n)								\
	const uint8_t emios_channels[EMIOS_CHANNEL_COUNT] = DT_INST_PROP(n, emios_channels);		\
	const eMios_Icu_Ip_ChannelConfigType eMios_Icu_Ip_ChannelConfig[EMIOS_CHANNEL_COUNT] = {	\
		eMios_Icu_Ip_ChannelConfig_Common(emios_channels[EMIOS_CW_CH_IDX]),			\
		eMios_Icu_Ip_ChannelConfig_Common(emios_channels[EMIOS_CCW_CH_IDX]),			\
	};												\
	const eMios_Icu_Ip_ConfigType eMios_Icu_Ip_Config = {	\
		.nNumChannels = EMIOS_CHANNEL_COUNT,				\
		.pChannelsConfig = &eMios_Icu_Ip_ChannelConfig,		\
	};

#define QDEC_S32K_INIT(n)																		\
																								\
	static struct qdec_s32k_data qdec_s32k_##n##_data = {                               		\
		.micro_ticks_per_rev = (float)(DT_INST_PROP(n, micro_ticks_per_rev)/1000000),                 \
		.counterCW = 1,                                                                 		\
		.counterCCW = 1,                                                                		\
		.Clockwise_overflow_count = 0,                                                  		\
		.CounterCW_overflow_count = 0,                                                  		\
	};                                                                                  		\
																								\
	PINCTRL_DT_INST_DEFINE(n);                                                          		\
	EMIOS_ICU_IP_CONFIG(n)																		\
	TRGMUX_IP_INIT_CONFIG(n)																	\
	LCU_IP_INIT_CONFIG(n)																		\
																								\
	static const struct qdec_s32k_config qdec_s32k_##n##_config = {                     		\
		.emios_inst = DT_INST_PROP(n, emios_inst),                                      		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                    		\
		.emios_config = &eMios_Icu_Ip_Config,													\
		.trgmux_config = &Trgmux_Ip_Init_Config,                                      			\
		.lcu_config = &Lcu_Ip_Init_Config,                                               		\
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(EMIOS_NXP_S32_NODE(n))),              		\
		.clock_subsys =                                                                 		\
			(clock_control_subsys_t)DT_CLOCKS_CELL(EMIOS_NXP_S32_NODE(n), name),        		\
		.lcu_rise_fall_filter = DT_INST_PROP(n, lcu_rise_fall_filter),                          \
		.lcu_glitch_filter = DT_INST_PROP(n, lcu_glitch_filter),                              	\
		.emios_channels = {DT_INST_PROP_BY_IDX(n, emios_channels, 0), DT_INST_PROP_BY_IDX(n, emios_channels, 1) }, \
		.trgmux_maps_len = DT_INST_PROP_LEN(n, trgmux_maps),	\
		.trgmux_maps = DT_INST_PROP(n, trgmux_maps),		\
	};                                                                                  		\
																								\
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_s32k_initialize, NULL, &qdec_s32k_##n##_data,			\
		&qdec_s32k_##n##_config, POST_KERNEL,                                           		\
		CONFIG_SENSOR_INIT_PRIORITY, &qdec_s32k_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_S32K_INIT)
