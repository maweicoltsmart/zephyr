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
#include "cfg_parm.h"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define PORT_REJOIN_KEY	SW0_GPIO_CONTROLLER
#define PORT_CALIBRATE_KEY	SW1_GPIO_CONTROLLER
#define PORT_MOTOR_KEY1_DOWN	SW2_GPIO_CONTROLLER
#define PORT_MOTOR_KEY2_UP	SW3_GPIO_CONTROLLER

#define PIN_REJOIN_KEY     SW0_GPIO_PIN
#define PIN_CALIBRATE_KEY     SW1_GPIO_PIN
#define PIN_MOTOR_KEY1_DOWN     SW2_GPIO_PIN
#define PIN_MOTOR_KEY2_UP     SW3_GPIO_PIN

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


void user_button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	if(pins & BIT(PIN_REJOIN_KEY))
	{
    	printk("rejoin key pressed\r\n");
    }
    else if(pins & BIT(PIN_CALIBRATE_KEY))
	{
		printk("calibrate key pressed\r\n");
	}
	else
	{
		return;
	}
	//cfg_parm_factory_reset();
	//cfg_parm_dump_to_ram();
}
/*void calibrate_button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	gpio_pin_read(gpiob, PIN_CALIBRATE_KEY, &val);
	printk("Button pressed at %d,%d\n", k_cycle_get_32(),val);
}*/
/*void motor_button1_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	u32_t val = 0U;
	struct motor_event_type motor_event;
    motor_event.event = MOTOR_KEY1_DOWN_EVENT;
    k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
	gpio_pin_read(gpiob, PIN_MOTOR_KEY1_DOWN, &val);
	printk("%s,%d\n", __func__, val);
}*/
void motor_button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	struct motor_event_type motor_event;
	//printk("0x%08X\r\n",pins);
	u32_t val = 0U;

	gpio_pin_read(gpiob, PIN_MOTOR_KEY1_DOWN, &val);
	if(val == 0)
	{
		//printk("motor down key pressed\r\n");
	}
	gpio_pin_read(gpiob, PIN_MOTOR_KEY2_UP, &val);
	if(val == 0)
	{
		//printk("motor up key pressed\r\n");
	}
	
	/*if(pins & BIT(PIN_MOTOR_KEY1_DOWN))
	{
    	motor_event.event = MOTOR_KEY1_DOWN_EVENT;
    	printk("motor down key pressed\r\n");
    }
    if(pins & BIT(PIN_MOTOR_KEY2_UP))
	{
		motor_event.event = MOTOR_KEY2_UP_EVENT;
		printk("motor up key pressed\r\n");
	}*/
    //k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
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

	gpio_init_callback(&gpio_cb, user_button_pressed, BIT(PIN_REJOIN_KEY));

	gpio_add_callback(gpiob, &gpio_cb);

	gpio_pin_configure(gpiob, PIN_REJOIN_KEY,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_REJOIN_KEY | EDGE_REJOIN_KEY);

	gpio_pin_enable_callback(gpiob, PIN_REJOIN_KEY);


	gpio_pin_configure(gpiob, PIN_CALIBRATE_KEY,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP_CALIBRATE_KEY | EDGE_CALIBRATE_KEY);

	gpio_pin_enable_callback(gpiob, PIN_CALIBRATE_KEY);

	gpiob = device_get_binding(PORT_MOTOR_KEY1_DOWN);
	if (!gpiob) {
		printk("error\n");
		return;
	}
	gpio_init_callback(&gpio_cb, motor_button_pressed, BIT(PIN_MOTOR_KEY1_DOWN));

	gpio_add_callback(gpiob, &gpio_cb);

	gpio_pin_configure(gpiob, PIN_MOTOR_KEY1_DOWN,
			   GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE_MOTOR_KEY1);

	gpio_pin_enable_callback(gpiob, PIN_MOTOR_KEY1_DOWN);

	gpio_pin_configure(gpiob, PIN_MOTOR_KEY2_UP,
			   GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE_MOTOR_KEY2);

	gpio_pin_enable_callback(gpiob, PIN_MOTOR_KEY2_UP);
}