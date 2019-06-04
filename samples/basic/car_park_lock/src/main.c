/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>

/* size of stack area used by each thread */
#define STACKSIZE 256

/* scheduling priority used by each thread */
#define PRIORITY 7

/* Change this if you have an LED connected to a custom port */
#ifndef LED0_GPIO_CONTROLLER
#define LED0_GPIO_CONTROLLER 	LED0_GPIO_PORT
#endif
#ifndef LED1_GPIO_CONTROLLER
#define LED1_GPIO_CONTROLLER 	LED1_GPIO_PORT
#endif

#define PORT0	 LED0_GPIO_CONTROLLER
#define PORT1	 LED1_GPIO_CONTROLLER


/* Change this if you have an LED connected to a custom pin */
#define LED0    LED0_GPIO_PIN
#define LED1    LED1_GPIO_PIN

void blink(const char *port, u32_t sleep_ms, u32_t led, u32_t id)
{
	int cnt = 0;
	struct device *gpio_dev;

	gpio_dev = device_get_binding(port);
	__ASSERT_NO_MSG(gpio_dev != NULL);

	gpio_pin_configure(gpio_dev, led, GPIO_DIR_OUT);

	while (1) {
		gpio_pin_write(gpio_dev, led, cnt % 2);

		k_sleep(sleep_ms);
		cnt++;
	}
}

void blink1(void)
{
	blink(PORT0, 100, LED0, 0);
}

void blink2(void)
{
	blink(PORT1, 1000, LED1, 1);
}

K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);