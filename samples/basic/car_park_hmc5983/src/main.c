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
#include <sensor.h>

/* size of stack area used by each thread */
#define STACKSIZE 512

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

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	u32_t led;
	u32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

void blink(const char *port, u32_t sleep_ms, u32_t led, u32_t id)
{
	int cnt = 0;
	struct device *gpio_dev;

	gpio_dev = device_get_binding(port);
	__ASSERT_NO_MSG(gpio_dev != NULL);

	gpio_pin_configure(gpio_dev, led, GPIO_DIR_OUT);

	while (1) {
		gpio_pin_write(gpio_dev, led, cnt % 2);

		struct printk_data_t tx_data = { .led = id, .cnt = cnt };

		size_t size = sizeof(struct printk_data_t);
		char *mem_ptr = k_malloc(size);
		__ASSERT_NO_MSG(mem_ptr != 0);

		memcpy(mem_ptr, &tx_data, size);

		k_fifo_put(&printk_fifo, mem_ptr);

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

void uart_out(void)
{
	while (1) {
		struct printk_data_t *rx_data = k_fifo_get(&printk_fifo, K_FOREVER);
		printk("Toggle USR%d LED: Counter = %d\n", rx_data->led, rx_data->cnt);
		k_free(rx_data);
	}
}

void hmc5983(void)
{
	struct device *dev = device_get_binding("HMC5883L");

	if (dev == NULL) {
		printk("Could not get HMC5883L device\n");
		return;
	}

	printk("dev %p name %s\n", dev, dev->config->name);

	while (1) {
		struct sensor_value temp[3];

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, temp);


		printk("x: %d.%08d; y: %d.%08d; z: %d.%08d\n",
		      temp[0].val1, temp[0].val2, temp[1].val1, temp[1].val2,
		      temp[2].val1, temp[2].val2);

		k_sleep(1000);
	}
}


K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(uart_out_id, STACKSIZE, uart_out, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(hmc5983_id, STACKSIZE, hmc5983, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
