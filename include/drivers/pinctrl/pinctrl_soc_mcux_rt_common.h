/*
 * Copyright (c) 2021, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_SOC_MCUX_RT_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_SOC_MCUX_RT_COMMON_H_

#include <devicetree.h>
#include <zephyr/types.h>
#include "fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pinctrl_soc_pinmux {
	uint32_t mux_register;
	uint32_t mux_mode;
	uint32_t input_register;
	uint32_t input_daisy;
	uint32_t config_register;
	uint32_t input_on_field;
};

typedef struct pinctrl_soc_pin {
	struct pinctrl_soc_pinmux pinmux;
	uint32_t pin_ctrl_flags;
} pinctrl_soc_pin_t;

#define MCUX_RT_HYS_SHIFT   IOMUXC_SW_PAD_CTL_PAD_HYS_SHIFT
#define MCUX_RT_PUS_SHIFT   IOMUXC_SW_PAD_CTL_PAD_PUS_SHIFT
#define MCUX_RT_PUE_SHIFT   IOMUXC_SW_PAD_CTL_PAD_PUE_SHIFT
#define MCUX_RT_PKE_SHIFT   IOMUXC_SW_PAD_CTL_PAD_PKE_SHIFT
#define MCUX_RT_ODE_SHIFT   IOMUXC_SW_PAD_CTL_PAD_ODE_SHIFT
#define MCUX_RT_SPEED_SHIFT IOMUXC_SW_PAD_CTL_PAD_SPEED_SHIFT
#define MCUX_RT_DSE_SHIFT   IOMUXC_SW_PAD_CTL_PAD_DSE_SHIFT
#define MCUX_RT_SRE_SHIFT   IOMUXC_SW_PAD_CTL_PAD_SRE_SHIFT

#define Z_PINCTRL_MCUX_RT_PINCFG_INIT(node_id)				\
	((DT_PROP(node_id, nxp_mcux_hys) << MCUX_RT_HYS_SHIFT) |	\
	 (DT_ENUM_IDX(node_id, nxp_mcux_pus) << MCUX_RT_PUS_SHIFT) |	\
	 (DT_PROP(node_id, nxp_mcux_pue) << MCUX_RT_PUE_SHIFT) |	\
	 (DT_PROP(node_id, nxp_mcux_pke) << MCUX_RT_PKE_SHIFT) |	\
	 (DT_PROP(node_id, drive_open_drain) << MCUX_RT_ODE_SHIFT) |	\
	 (DT_ENUM_IDX(node_id, nxp_mcux_speed) << MCUX_RT_SPEED_SHIFT) |\
	 (DT_ENUM_IDX(node_id, nxp_mcux_dse) << MCUX_RT_DSE_SHIFT) |	\
	 (DT_ENUM_IDX(node_id, nxp_mcux_sre) << MCUX_RT_SRE_SHIFT))

#define Z_PINCTRL_PIN_INIT(node_id)				\
	{ .pinmux.mux_register = DT_PROP_BY_IDX(node_id, pinmux, 0),	\
	  .pinmux.mux_mode = DT_PROP_BY_IDX(node_id, pinmux, 1),	\
	  .pinmux.input_register = DT_PROP_BY_IDX(node_id, pinmux, 2),	\
	  .pinmux.input_daisy = DT_PROP_BY_IDX(node_id, pinmux, 3),	\
	  .pinmux.config_register = DT_PROP_BY_IDX(node_id, pinmux, 4),	\
	  .pinmux.input_on_field = DT_PROP(node_id, nxp_mcux_input),	\
	  .pin_ctrl_flags = Z_PINCTRL_MCUX_RT_PINCFG_INIT(node_id),	\
	},

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			\
	Z_PINCTRL_PIN_INIT(DT_PROP_BY_IDX(node_id, prop, idx))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/**
 * @brief Define all the states for the given node identifier.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_STATES_DEFINE(node_id)				\
	static const struct pinctrl_state				\
	Z_PINCTRL_STATES_NAME(node_id)[] = {				\
		UTIL_LISTIFY(DT_NUM_PINCTRL_STATES(node_id),		\
			     Z_PINCTRL_STATE_INIT, node_id)		\
	};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_SOC_MCUX_RT_COMMON_H_ */
