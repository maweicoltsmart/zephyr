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
#include "dtunvs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

static uint8_t device_unique_id[8] = {0x20,0x19,0x05,0x06,0x00,0x00,0x00,0x01};
extern char localip[];
struct dtu_param_nvs_struct dtu_param_nvs;
K_SEM_DEFINE(dtu_param_nvs_restore, 0, 1);
extern struct k_sem got_address;

void dtu_nvs_init(void)
{
	struct dtu_param_nvs_struct *temp;
	struct device *flash_dev = NULL;

	temp = malloc(sizeof(dtu_param_nvs));
	if(temp == NULL)
	{
		printk("malloc failed \r\n");
		return;
	}
	memset(temp,0,sizeof(dtu_param_nvs));
	memset(&dtu_param_nvs,0,sizeof(dtu_param_nvs));

	strcpy(dtu_param_nvs.dev_ip,localip);
	strcpy(dtu_param_nvs.dev_name,"device001");
	for(int i = 0;i < 8;i ++)
	{
		sprintf(&dtu_param_nvs.dev_id[i * 2],"%02X",device_unique_id[i]);
	}
	strcpy(dtu_param_nvs.dev_mac,&dtu_param_nvs.dev_id[4]);
	strcpy(dtu_param_nvs.dev_version,"V1.0");
	strcpy(dtu_param_nvs.dev_type,"ETH_DTU");
	dtu_param_nvs.conn2mj = false;
	strcpy(dtu_param_nvs.iptype,"DHCP");
	strcpy(dtu_param_nvs.user_name,"coltsmart");
	strcpy(dtu_param_nvs.user_pwd,"www.coltsmart.com");
	strcpy(dtu_param_nvs.net_mask,"255.255.255.255");
	strcpy(dtu_param_nvs.gateway,"192.168.1.1");
	strcpy(dtu_param_nvs.work_type,"TCP Server");
	strcpy(dtu_param_nvs.dest_ip,"192.168.1.2");
	dtu_param_nvs.dest_port = 4999;
	dtu_param_nvs.local_port = 5000;

	strcpy(dtu_param_nvs.com[0].protocol,"RS232");
	dtu_param_nvs.com[0].baud = 115200;
	strcpy(dtu_param_nvs.com[0].databit,"8");
	strcpy(dtu_param_nvs.com[0].stopbit,"1");
	strcpy(dtu_param_nvs.com[0].paritybit,"None");
	strcpy(dtu_param_nvs.com[0].flowctrl,"None");
	dtu_param_nvs.com[0].timeout = 5;
	dtu_param_nvs.com[0].maxlen = 1024;
	dtu_param_nvs.com[0].autobaud = false;

	strcpy(dtu_param_nvs.com[1].protocol,"RS485");
	dtu_param_nvs.com[1].baud = 115200;
	strcpy(dtu_param_nvs.com[1].databit,"8");
	strcpy(dtu_param_nvs.com[1].stopbit,"1");
	strcpy(dtu_param_nvs.com[1].paritybit,"None");
	strcpy(dtu_param_nvs.com[1].flowctrl,"None");
	dtu_param_nvs.com[1].timeout = 5;;
	dtu_param_nvs.com[1].maxlen = 1024;
	dtu_param_nvs.com[1].autobaud = false;

	

	printk("\nInit DTU NVS Parameters\n");
	printk("=========================\n");

	//if(flash_dev == NULL)
	{
		flash_dev = device_get_binding(DT_FLASH_DEV_NAME);
	}

	if (!flash_dev) {
		printk("STM32F407 flash driver was not found!\n");
		free(temp);
		return;
	}

	printk("\nFlash erase page at 0x%x\n", DTU_PARAM_FLASH_OFFSET);
	flash_write_protection_set(flash_dev, false);
	if (flash_erase(flash_dev, DTU_PARAM_FLASH_OFFSET, FLASH_PAGE_SIZE) != 0) {
		printk("   Flash erase failed!\n");
		goto failed;
	} else {
		printk("   Flash erase succeeded!\n");
	}

	printk("\nFlash write DTU NVS Parameters\n");
	flash_write_protection_set(flash_dev, false);
	
	if (flash_write(flash_dev, DTU_PARAM_FLASH_OFFSET, &dtu_param_nvs,
		sizeof(dtu_param_nvs)) != 0) {
		printk("   Flash write failed!\n");
		goto failed;
	}
	flash_write_protection_set(flash_dev, true);
	printk("   Attempted to read 0x%x\n", DTU_PARAM_FLASH_OFFSET);
	if (flash_read(flash_dev, DTU_PARAM_FLASH_OFFSET, temp,
		sizeof(dtu_param_nvs)) != 0) {
		printk("   Flash read failed!\n");
		goto failed;
	}
	if(memcmp(temp,&dtu_param_nvs,sizeof(dtu_param_nvs)) == 0)
	{
		printk("   Data read matches data written. Good! len = %d\n",sizeof(dtu_param_nvs));
	}
	else
	{
		printk("   Data read does not match data written! len = %d\n",sizeof(dtu_param_nvs));
	}

failed:
	//flash_write_protection_set(flash_dev, true);
	free(temp);
	/*while(1)
	{
		sleep(1);
		printk("again\r\n");
		dtu_nvs_restore();
	}*/

}

void dtu_nvs_restore(void)
{
	struct dtu_param_nvs_struct *temp;
	struct device *flash_dev = NULL;

	k_sem_take(&got_address,K_FOREVER);
	sleep(1);
	dtu_nvs_init();
	while(1)
	{
		sleep(1);
		printk("again\r\n");
		//k_sem_take(&dtu_param_nvs_restore,K_FOREVER);
		temp = malloc(sizeof(dtu_param_nvs));
		if(temp == NULL)
		{
			printk("malloc failed \r\n");
			return;
		}
		memset(temp,0,sizeof(dtu_param_nvs));

		printk("\nRestore DTU NVS Parameters\n");
		printk("=========================\n");
		flash_dev = device_get_binding(DT_FLASH_DEV_NAME);

		if (!flash_dev) {
			printk("STM32F407 flash driver was not found!\n");
			free(temp);
			return;
		}
		printk("\nFlash erase page at 0x%x\n", DTU_PARAM_FLASH_OFFSET);
		flash_write_protection_set(flash_dev, false);
		if (flash_erase(flash_dev, DTU_PARAM_FLASH_OFFSET, FLASH_PAGE_SIZE) != 0) {
			printk("   Flash erase failed!\n");
			goto failed;
		} else {
			printk("   Flash erase succeeded!\n");
		}

		printk("\nFlash write DTU NVS Parameters\n");
		flash_write_protection_set(flash_dev, false);
		
		if (flash_write(flash_dev, DTU_PARAM_FLASH_OFFSET, &dtu_param_nvs,
			sizeof(dtu_param_nvs)) != 0) {
			printk("   Flash write failed!\n");
			goto failed;
		}
		flash_write_protection_set(flash_dev, true);
		printk("   Attempted to read 0x%x\n", DTU_PARAM_FLASH_OFFSET);
		if (flash_read(flash_dev, DTU_PARAM_FLASH_OFFSET, temp,
			sizeof(dtu_param_nvs)) != 0) {
			printk("   Flash read failed!\n");
			goto failed;
		}
		if(memcmp(temp,&dtu_param_nvs,sizeof(dtu_param_nvs)) == 0)
		{
			printk("   Data read matches data written. Good! len = %d\n",sizeof(dtu_param_nvs));
		}
		else
		{
			printk("   Data read does not match data written! len = %d\n",sizeof(dtu_param_nvs));
		}
	}

failed:
	//flash_write_protection_set(flash_dev, true);
	free(temp);
}

K_THREAD_DEFINE(dtu_nvs_restore_id, STACKSIZE, dtu_nvs_restore, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);