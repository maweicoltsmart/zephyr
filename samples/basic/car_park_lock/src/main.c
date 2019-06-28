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
#include "cfg_parm.h"

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
#ifndef LED2_GPIO_CONTROLLER
#define LED2_GPIO_CONTROLLER 	LED2_GPIO_PORT
#endif

#define PORT0	 LED0_GPIO_CONTROLLER
#define PORT1	 LED1_GPIO_CONTROLLER
#define PORT2	 LED2_GPIO_CONTROLLER


/* Change this if you have an LED connected to a custom pin */
#define LED0    LED0_GPIO_PIN
#define LED1    LED1_GPIO_PIN
#define LED2    LED2_GPIO_PIN

void blink(void)
{
	struct device *gpio_dev0,*gpio_dev1,*gpio_dev2;

	gpio_dev0 = device_get_binding(PORT0);
	__ASSERT_NO_MSG(gpio_dev0 != NULL);

	gpio_pin_configure(gpio_dev0, LED0, GPIO_DIR_OUT);

	gpio_dev1 = device_get_binding(PORT1);
	__ASSERT_NO_MSG(gpio_dev1 != NULL);

	gpio_pin_configure(gpio_dev1, LED1, GPIO_DIR_OUT);

	gpio_dev2 = device_get_binding(PORT2);
	__ASSERT_NO_MSG(gpio_dev2 != NULL);

	gpio_pin_configure(gpio_dev2, LED2, GPIO_DIR_OUT);

	while(1)
	{
		if(stTmpCfgParm.led_mask & 0x01)
		{
			gpio_pin_write(gpio_dev0, LED0, 1);
			gpio_pin_write(gpio_dev1, LED1, 0);
			gpio_pin_write(gpio_dev2, LED2, 0);
		}
		k_sleep(1000);
		if(stTmpCfgParm.led_mask & 0x02)
		{
			gpio_pin_write(gpio_dev0, LED0, 0);
			gpio_pin_write(gpio_dev1, LED1, 1);
			gpio_pin_write(gpio_dev2, LED2, 0);
		}
		k_sleep(1000);
		if(stTmpCfgParm.led_mask & 0x04)
		{
			gpio_pin_write(gpio_dev0, LED0, 0);
			gpio_pin_write(gpio_dev1, LED1, 0);
			gpio_pin_write(gpio_dev2, LED2, 1);
		}
		k_sleep(1000);
	}
}

/*void blink2(void)
{
	blink(PORT1, 1000, LED1, 1);
}
*/
K_THREAD_DEFINE(blink_id, STACKSIZE, blink, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
/*K_THREAD_DEFINE(blink2_id, STACKSIZE, blink2, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);*/