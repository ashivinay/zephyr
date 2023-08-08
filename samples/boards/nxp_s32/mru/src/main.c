/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>

#define CH_ID (0)

// static K_SEM_DEFINE(sem, 0U, 1);

static void callback(const struct device *dev, uint32_t channel,
		     void *user_data, struct mbox_msg *msg)
{
	printk("Pong on channel %d, rx data is %d\n\n",
		channel, *((uint32_t *)(msg->data)));

	// k_sem_give(&sem);
}

int main(void)
{
	struct mbox_channel tx_channel;
	struct mbox_channel rx_channel;
	const struct device *dev;
	uint32_t payload = 0;
	const struct mbox_msg msg = {
		.data = &payload,
		.size = sizeof(payload)
	};

	dev = DEVICE_DT_GET(DT_NODELABEL(mru0));
	if (!device_is_ready(dev)) {
		printk("device not ready.\n");
		return 0;
	}

	mbox_init_channel(&tx_channel, dev, CH_ID);
	mbox_init_channel(&rx_channel, dev, CH_ID);

	if (mbox_register_callback(&rx_channel, callback, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}

	if (mbox_set_enabled(&rx_channel, 1)) {
		printk("mbox_set_enable() error\n");
		return 0;
	}

	while (1) {
		payload = k_cycle_get_32();
		printk("Ping on channel %d, tx data is %d\n",
			tx_channel.id, payload);

		if (mbox_send(&tx_channel, &msg) < 0) {
			printk("mbox_send() error\n");
			return 0;
		}

		// k_sem_take(&sem, K_FOREVER);
		k_sleep(K_MSEC(3000));
	}

	return 0;
}
