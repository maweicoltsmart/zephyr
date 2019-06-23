/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_QMC5883_QMC5883_H_
#define ZEPHYR_DRIVERS_SENSOR_QMC5883_QMC5883_H_

#include <device.h>
#include <misc/util.h>
#include <zephyr/types.h>
#include <gpio.h>

#define QMC5883_I2C_ADDR		0x0D

#define QMC5883_REG_CONFIG_A		0x00
#define QMC5883_ODR_SHIFT		2

#define QMC5883_REG_CONFIG_B		0x01
#define QMC5883_GAIN_SHIFT		5

#define QMC5883_REG_MODE		0x02
#define QMC5883_MODE_CONTINUOUS	0

#define QMC5883_REG_DATA_START		0x00

#define QMC5883_REG_CHIP_ID		0x0D
#define QMC5883_CHIP_ID_A		0xff

static const char * const qmc5883_odr_strings[] = {
	"10", "50", "100", "200"
};

static const char * const qmc5883_fs_strings[] = {
	"0.88", "1.3", "1.9", "2.5", "4", "4.7", "5.6", "8.1"
};

static const u16_t qmc5883_gain[] = {
	1370, 1090, 820, 660, 440, 390, 330, 230
};

struct qmc5883_data {
	struct device *i2c;
	s16_t x_sample;
	s16_t y_sample;
	s16_t z_sample;
	u8_t gain_idx;

#ifdef CONFIG_QMC5883_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_QMC5883_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_QMC5883_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_QMC5883_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_QMC5883_TRIGGER */
};

#ifdef CONFIG_QMC5883_TRIGGER
int qmc5883_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int qmc5883_init_interrupt(struct device *dev);
#endif

#endif /* __SENSOR_QMC5883__ */
