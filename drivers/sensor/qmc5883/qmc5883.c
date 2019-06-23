/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <i2c.h>
#include <init.h>
#include <misc/__assert.h>
#include <misc/byteorder.h>
#include <sensor.h>
#include <string.h>
#include <logging/log.h>

#include "qmc5883.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(QMC5883);

static void qmc5883_convert(struct sensor_value *val, s16_t raw_val,
			     u16_t divider)
{
	/* val = raw_val / divider */
	val->val1 = raw_val / divider;
	//val->val2 = (((s64_t)raw_val % divider) * 1000000L) / divider;
}

static int qmc5883_channel_get(struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct qmc5883_data *drv_data = dev->driver_data;

	if (chan == SENSOR_CHAN_MAGN_X) {
		qmc5883_convert(val, drv_data->x_sample,
				 qmc5883_gain[drv_data->gain_idx]);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		qmc5883_convert(val, drv_data->y_sample,
				 qmc5883_gain[drv_data->gain_idx]);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		qmc5883_convert(val, drv_data->z_sample,
				 qmc5883_gain[drv_data->gain_idx]);
	} else { /* chan == SENSOR_CHAN_MAGN_XYZ */
		qmc5883_convert(val, drv_data->x_sample,
				 1);//qmc5883_gain[drv_data->gain_idx]);
		qmc5883_convert(val + 1, drv_data->y_sample,
				 1);//qmc5883_gain[drv_data->gain_idx]);
		qmc5883_convert(val + 2, drv_data->z_sample,
				 1);//qmc5883_gain[drv_data->gain_idx]);
	}

	return 0;
}

static int qmc5883_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct qmc5883_data *drv_data = dev->driver_data;
	s16_t buf[3];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	/* fetch magnetometer sample */
	if (i2c_burst_read(drv_data->i2c, QMC5883_I2C_ADDR,
			   QMC5883_REG_DATA_START, (u8_t *)buf, 6) < 0) {
		LOG_ERR("Failed to fetch megnetometer sample.");
		return -EIO;
	}

	drv_data->x_sample = sys_le16_to_cpu(buf[0]);
	drv_data->y_sample = sys_le16_to_cpu(buf[1]);
	drv_data->z_sample = sys_le16_to_cpu(buf[2]);

	return 0;
}

static const struct sensor_driver_api qmc5883_driver_api = {
#if CONFIG_QMC5883_TRIGGER
	.trigger_set = qmc5883_trigger_set,
#endif
	.sample_fetch = qmc5883_sample_fetch,
	.channel_get = qmc5883_channel_get,
};

int qmc5883_init(struct device *dev)
{
	struct qmc5883_data *drv_data = dev->driver_data;
	u8_t chip_cfg[3], id[3];

	drv_data->i2c = device_get_binding(CONFIG_QMC5883_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device.",
			    CONFIG_QMC5883_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	/* check chip ID */
	if (i2c_burst_read(drv_data->i2c, QMC5883_I2C_ADDR,
			   QMC5883_REG_CHIP_ID, id, 1) < 0) {
		LOG_ERR("Failed to read chip ID.");
		return -EIO;
	}

	if (id[0] != QMC5883_CHIP_ID_A) {
		LOG_ERR("Invalid chip ID.");
		return -EINVAL;
	}

	chip_cfg[0] = 0x0d;	// 0d: 2Ga, 1d: 8Ga
	if (i2c_burst_write(drv_data->i2c, QMC5883_I2C_ADDR,	// ctl register config
			    0x09, chip_cfg, 1) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}
	chip_cfg[0] = 0x01;
	if (i2c_burst_write(drv_data->i2c, QMC5883_I2C_ADDR,	// set clear time register
			    0x0b, chip_cfg, 1) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}
#if 0
	/* check if CONFIG_QMC5883_FS is valid */
	for (idx = 0U; idx < ARRAY_SIZE(qmc5883_fs_strings); idx++) {
		if (!strcmp(qmc5883_fs_strings[idx], CONFIG_QMC5883_FS)) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(qmc5883_fs_strings)) {
		LOG_ERR("Invalid full-scale range value.");
		return -EINVAL;
	}

	drv_data->gain_idx = idx;

	/* check if CONFIG_QMC5883_ODR is valid */
	for (idx = 0U; idx < ARRAY_SIZE(qmc5883_odr_strings); idx++) {
		if (!strcmp(qmc5883_odr_strings[idx], CONFIG_QMC5883_ODR)) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(qmc5883_odr_strings)) {
		LOG_ERR("Invalid ODR value.");
		return -EINVAL;
	}

	/* configure device */
	chip_cfg[0] = idx << QMC5883_ODR_SHIFT;
	chip_cfg[1] = drv_data->gain_idx << QMC5883_GAIN_SHIFT;
	chip_cfg[2] = QMC5883_MODE_CONTINUOUS;

	if (i2c_burst_write(drv_data->i2c, QMC5883_I2C_ADDR,
			    QMC5883_REG_CONFIG_A, chip_cfg, 3) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}
#endif
#ifdef CONFIG_QMC5883_TRIGGER
	if (qmc5883_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	return 0;
}

struct qmc5883_data qmc5883_driver;

DEVICE_AND_API_INIT(qmc5883, CONFIG_QMC5883_NAME, qmc5883_init,
		    &qmc5883_driver, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &qmc5883_driver_api);
