#ifndef __CAR_PARK_MOTOR_H__
#define __CAR_PARK_MOTOR_H__

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>

typedef enum{
    MOTOR_CMD_FORWARD_EVENT = 0,
    MOTOR_CMD_BACKWARD_EVENT,
    MOTOR_CMD_BRAK_EVENT,
    MOTOR_KEY1_EVENT,
    MOTOR_KEY2_EVENT,
}en_MOTOR_EVENT,*pen_MOTOR_EVENT;

struct motor_event_type {
    en_MOTOR_EVENT event;
};

extern struct k_msgq motor_msgq;
extern void my_button_init(void);

#endif