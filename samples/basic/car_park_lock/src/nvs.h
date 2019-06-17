#ifndef __CAR_PARK_LOCK_NVS_H__

#define __CAR_PARK_LOCK_NVS_H__

//#define NVS_STRING_MAX_LEN	32
#define NET_PARAM_FLASH_OFFSET (0xFC00)
#define FLASH_PAGE_SIZE          0x400U

void car_park_lock_nvs_restore(uint8_t * data,uint32_t len);
void car_park_lock_nvs_get(uint8_t * data,uint32_t len);

#endif