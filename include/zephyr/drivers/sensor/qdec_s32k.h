/*
 * Copyright (c) 2023, NAP Semiconductors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_S32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_S32_H_

#include <zephyr/drivers/sensor.h>

enum sensor_attribute_qdec_s32 {
	/* Number of counts per revolution */
	SENSOR_ATTR_QDEC_MOD_VAL = SENSOR_ATTR_PRIV_START,
	/* Single phase counting */
	SENSOR_ATTR_QDEC_ENABLE_SINGLE_PHASE,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_S32_H_ */
