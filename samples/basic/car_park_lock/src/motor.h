#ifndef __CAR_PARK_MOTOR_H__
#define __CAR_PARK_MOTOR_H__

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>

typedef enum{
    MOTOR_CMD_DOWN_EVENT = 0,
    MOTOR_CMD_UP_EVENT = 1,
    MOTOR_CURRENT_ADC_BRAK_EVENT,
    MOTOR_KEY1_DOWN_EVENT,
    MOTOR_KEY2_UP_EVENT,
    MOTOR_NOKEY1_NOKEY2_EVENT
}en_MOTOR_EVENT,*pen_MOTOR_EVENT;

typedef union{
	uint8_t value;
	uint8_t motor:1;
	uint8_t sensor1:1;
	uint8_t sensor2:1;
}un_LockStatus,*pun_LockStatus;

struct motor_event_type {
    en_MOTOR_EVENT event;
};

extern struct k_msgq motor_msgq;
extern void my_button_init(void);
extern un_LockStatus unLockStatus;
extern uint16_t destensecopy[];

#endif