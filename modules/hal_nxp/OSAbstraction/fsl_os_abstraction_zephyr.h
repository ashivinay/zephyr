/**
 * Copyright (c) 2021, NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !defined(__FSL_OS_ABSTRACTION_ZEPHYR_H__)
#define __FSL_OS_ABSTRACTION_ZEPHYR_H__
/* fsl_common.h is included before the kernel headers, and also defines 
 * ARRAY_SIZE(x). Undefine the macro here, so that the kernel definition takes
 * affect
 */
#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#endif
#include <kernel.h>

#define OSA_TASK_HANDLE_SIZE (sizeof(struct k_thread))

/* Zephyr does not have a concept of event groups, so use the result variable
 * from a k_poll_signal to store the flags value of an event
 */

struct zephyr_event {
	struct k_poll_event event;
	struct k_poll_signal signal;
	osa_event_flags_t flag_state;
	bool auto_clear;
};

#define OSA_EVENT_HANDLE_SIZE (sizeof(struct zephyr_event))

#define OSA_SEM_HANDLE_SIZE (sizeof(struct k_sem))

#define OSA_MUTEX_HANDLE_SIZE (sizeof(struct k_mutex))

/* Additional tracking information is required for queues */
struct zephyr_queue {
	struct k_queue queue;
	uint32_t msg_size;
	int msg_cnt;
};

#define OSA_MSGQ_HANDLE_SIZE (sizeof(struct zephyr_queue))

#endif
