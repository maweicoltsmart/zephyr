/*
 * Copyright (c) 2016 Linaro Limited
 *               2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <flash.h>
#include <device.h>
#include <stdio.h>
#include "nvs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* size of stack area used by each thread */
#define STACKSIZE 256

/* scheduling priority used by each thread */
#define PRIORITY 7

//static uint8_t device_unique_id[8] = {0x20,0x19,0x05,0x06,0x00,0x00,0x00,0x01};
//extern char localip[];

//K_SEM_DEFINE(car_park_lock_nvs_restore, 0, 1);

void car_park_lock_nvs_restore(uint8_t * data,uint32_t len)
{
	struct device *flash_dev = NULL;

	flash_dev = device_get_binding(DT_FLASH_DEV_NAME);

	if (!flash_dev) {
		printk("STM32F030 flash driver was not found!\n");
		return;
	}

	printk("\nFlash erase page at 0x%x\n", NET_PARAM_FLASH_OFFSET);
	flash_write_protection_set(flash_dev, false);
	if (flash_erase(flash_dev, NET_PARAM_FLASH_OFFSET, FLASH_PAGE_SIZE) != 0) {
		printk("   Flash erase failed!\n");
		goto failed;
	} else {
		printk("   Flash erase succeeded!\n");
	}

	printk("\nFlash write CAR_PARK_LOCK NVS Parameters\n");
	flash_write_protection_set(flash_dev, false);
	
	if (flash_write(flash_dev, NET_PARAM_FLASH_OFFSET, data,
		len) != 0) {
		printk("   Flash write failed!\n");
		goto failed;
	}
	/*flash_write_protection_set(flash_dev, true);
	printk("   Attempted to read 0x%x\n", NET_PARAM_FLASH_OFFSET);
	if (flash_read(flash_dev, NET_PARAM_FLASH_OFFSET, temp,
		sizeof(device_unique_id)) != 0) {
		printk("   Flash read failed!\n");
		goto failed;
	}
	if(memcmp(temp,device_unique_id,sizeof(device_unique_id)) == 0)
	{
		printk("   Data read matches data written. Good! len = %d\n",sizeof(device_unique_id));
	}
	else
	{
		printk("   Data read does not match data written! len = %d\n",sizeof(device_unique_id));
	}*/

failed:
	flash_write_protection_set(flash_dev, true);
}

void car_park_lock_nvs_get(uint8_t * data,uint32_t len)
{
	struct device *flash_dev = NULL;

	flash_dev = device_get_binding(DT_FLASH_DEV_NAME);

	if (!flash_dev) {
		printk("STM32F030 flash driver was not found!\n");
		return;
	}

	flash_write_protection_set(flash_dev, true);
	printk("   Attempted to read 0x%x\n", NET_PARAM_FLASH_OFFSET);
	if (flash_read(flash_dev, NET_PARAM_FLASH_OFFSET, data,
		len) != 0) {
		printk("   Flash read failed!\n");
		goto failed;
	}

failed:
	flash_write_protection_set(flash_dev, true);
}


/*K_THREAD_DEFINE(car_park_lock_nvs_restore_id, STACKSIZE, car_park_lock_nvs_init, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);*/