/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <i2c.h>
#include <misc/__assert.h>
#include <misc/util.h>
#include <kernel.h>
#include <sensor.h>

#include "qmc5883.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_DECLARE(QMC5883);

int qmc5883_trigger_set(struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct qmc5883_data *drv_data = dev->driver_data;

	__ASSERT_NO_MSG(trig->type == SENSOR_TRIG_DATA_READY);

	gpio_pin_disable_callback(drv_data->gpio, CONFIG_QMC5883_GPIO_PIN_NUM);

	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	drv_data->data_ready_trigger = *trig;

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_QMC5883_GPIO_PIN_NUM);

	return 0;
}

static void qmc5883_gpio_callback(struct device *dev,
				   struct gpio_callback *cb, u32_t pins)
{
	struct qmc5883_data *drv_data =
		CONTAINER_OF(cb, struct qmc5883_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_disable_callback(dev, CONFIG_QMC5883_GPIO_PIN_NUM);

#if defined(CONFIG_QMC5883_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_QMC5883_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void qmc5883_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct qmc5883_data *drv_data = dev->driver_data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_QMC5883_GPIO_PIN_NUM);
}

#ifdef CONFIG_QMC5883_TRIGGER_OWN_THREAD
static void qmc5883_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct qmc5883_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		qmc5883_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_QMC5883_TRIGGER_GLOBAL_THREAD
static void qmc5883_work_cb(struct k_work *work)
{
	struct qmc5883_data *drv_data =
		CONTAINER_OF(work, struct qmc5883_data, work);

	qmc5883_thread_cb(drv_data->dev);
}
#endif

int qmc5883_init_interrupt(struct device *dev)
{
	struct qmc5883_data *drv_data = dev->driver_data;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(CONFIG_QMC5883_GPIO_DEV_NAME);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device.",
			    CONFIG_QMC5883_GPIO_DEV_NAME);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, CONFIG_QMC5883_GPIO_PIN_NUM,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&drv_data->gpio_cb,
			   qmc5883_gpio_callback,
			   BIT(CONFIG_QMC5883_GPIO_PIN_NUM));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback.");
		return -EIO;
	}

#if defined(CONFIG_QMC5883_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_QMC5883_THREAD_STACK_SIZE,
			(k_thread_entry_t)qmc5883_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_QMC5883_THREAD_PRIORITY),
			0, 0);
#elif defined(CONFIG_QMC5883_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = qmc5883_work_cb;
	drv_data->dev = dev;
#endif

	gpio_pin_enable_callback(drv_data->gpio, CONFIG_QMC5883_GPIO_PIN_NUM);

	return 0;
}
