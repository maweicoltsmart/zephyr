/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/printk.h>
#include "motor.h"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define PORT_REJOIN_KEY	SW0_GPIO_CONTROLLER
#define PORT_CALIBRATE_KEY	SW1_GPIO_CONTROLLER
#define PORT_MOTOR_KEY1	SW2_GPIO_CONTROLLER
#define PORT_MOTOR_KEY2	SW3_GPIO_CONTROLLER

#define PIN_REJOIN_KEY     SW0_GPIO_PIN
#define PIN_CALIBRATE_KEY     SW1_GPIO_PIN
#define PIN_MOTOR_KEY1     SW2_GPIO_PIN
#define PIN_MOTOR_KEY2     SW3_GPIO_PIN

#define EDGE_REJOIN_KEY    (SW0_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_CALIBRATE_KEY    (SW1_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_MOTOR_KEY1    (SW2_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_MOTOR_KEY2    (SW3_GPIO_FLAGS | GPIO_INT_EDGE)

#define PULL_UP_REJOIN_KEY SW0_GPIO_FLAGS
#define PULL_UP_CALIBRATE_KEY SW1_GPIO_FLAGS
#define PULL_UP_MOTOR_KEY1 SW2_GPIO_FLAGS
#define PULL_UP_MOTOR_KEY2 SW3_GPIO_FLAGS

/* Sleep time */
#define SLEEP_TIME	500


void rejoin_button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	gpio_pin_read(gpiob, PIN_REJOIN_KEY, &val);
	printk("Button pressed at %d,%d\n", k_cycle_get_32(),val);
}
void calibrate_button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	gpio_pin_read(gpiob, PIN_CALIBRATE_KEY, &val);
	printk("Button pressed at %d,%d\n", k_cycle_get_32(),val);
}
void motor_button1_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	struct motor_event_type motor_event;
    motor_event.event = MOTOR_KEY1_EVENT;
    k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
	gpio_pin_read(gpiob, PIN_MOTOR_KEY1, &val);
	printk("Button pressed at %d,%d\n", k_cycle_get_32(),val);
}
void motor_button2_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	struct motor_event_type motor_event;
    motor_event.event = MOTOR_KEY2_EVENT;
    k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
	gpio_pin_read(gpiob, PIN_MOTOR_KEY2, &val);
	printk("Button pressed at %d,%d\n", k_cycle_get_32(),val);
}

static struct gpio_callback gpio_cb;

void my_button_init(void)
{
	struct device *gpiob;

	gpiob = device_get_binding(PORT_REJOIN_KEY);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN_REJOIN_KEY,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_REJOIN_KEY | EDGE_REJOIN_KEY);

	gpio_init_callback(&gpio_cb, rejoin_button_pressed, BIT(PIN_REJOIN_KEY));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN_REJOIN_KEY);

	gpiob = device_get_binding(PORT_CALIBRATE_KEY);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN_CALIBRATE_KEY,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_CALIBRATE_KEY | EDGE_CALIBRATE_KEY);

	gpio_init_callback(&gpio_cb, calibrate_button_pressed, BIT(PIN_CALIBRATE_KEY));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN_CALIBRATE_KEY);

	gpiob = device_get_binding(PORT_MOTOR_KEY1);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN_MOTOR_KEY1,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_MOTOR_KEY1 | EDGE_MOTOR_KEY1);

	gpio_init_callback(&gpio_cb, motor_button1_pressed, BIT(PIN_MOTOR_KEY1));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN_MOTOR_KEY1);

	gpiob = device_get_binding(PORT_MOTOR_KEY2);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN_MOTOR_KEY2,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_MOTOR_KEY2 | EDGE_MOTOR_KEY2);

	gpio_init_callback(&gpio_cb, motor_button2_pressed, BIT(PIN_MOTOR_KEY2));

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, PIN_MOTOR_KEY2);
}