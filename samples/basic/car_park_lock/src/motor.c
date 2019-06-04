#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include <stdio.h>
#include "motor.h"
#include <adc.h>

/* size of stack area used by each thread */
#define STACKSIZE 256

/* scheduling priority used by each thread */
#define PRIORITY 7

#define PORT_MOTOR_BRAK		"GPIO_A"
#define PORT_MOTOR_DIR		"GPIO_B"
#define PORT_MOTOR_PWM		"GPIO_A"

#define PIN_MOTOR_BRAK		7
#define PIN_MOTOR_DIR		0
#define PIN_MOTOR_PWM		6

#define ADC_DEVICE_NAME		DT_ADC_1_NAME
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	4

#define BUFFER_SIZE  6
static s16_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};
K_MSGQ_DEFINE(motor_msgq, sizeof(struct motor_event_type), 20, 4);

void motor_process(void)
{
	struct device *gpio_dev_brak,*gpio_dev_dir,*gpio_dev_pwm;
	struct motor_event_type motor_event;

	gpio_dev_brak = device_get_binding(PORT_MOTOR_BRAK);
	__ASSERT_NO_MSG(gpio_dev_brak != NULL);
	gpio_pin_configure(gpio_dev_brak, PIN_MOTOR_BRAK, GPIO_DIR_OUT);

	gpio_dev_dir = device_get_binding(PORT_MOTOR_DIR);
	__ASSERT_NO_MSG(gpio_dev_dir != NULL);
	gpio_pin_configure(gpio_dev_dir, PIN_MOTOR_DIR, GPIO_DIR_OUT);

	gpio_dev_pwm = device_get_binding(PORT_MOTOR_PWM);
	__ASSERT_NO_MSG(gpio_dev_pwm != NULL);
	gpio_pin_configure(gpio_dev_pwm, PIN_MOTOR_PWM, GPIO_DIR_OUT);

	my_button_init();
	
    while(1)
    {
        k_msgq_get(&motor_msgq, &motor_event, K_FOREVER);
        switch(motor_event.event)
        {
            case MOTOR_CMD_FORWARD_EVENT:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 1);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 1);
                break;
            case MOTOR_CMD_BACKWARD_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 1);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                break;
            case MOTOR_KEY1_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                break;
            case MOTOR_KEY2_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                break;
            case MOTOR_CMD_BRAK_EVENT:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
            	break;
            default:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
            	break;
        }
    }
	
}

static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);

	(void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

	return adc_dev;
}

static s16_t check_samples(int expected_count)
{
	int i;
	s16_t sample_value;

	for (i = 0; i < BUFFER_SIZE; i++) {
		sample_value = m_sample_buffer[i];

		printk("0x%04x ", sample_value);
	}
	return sample_value;
}

void motor_adc_process(void)
{
	int ret;
	struct motor_event_type motor_event;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};
	s16_t sample_value;

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		printk("do not find adc device\r\n");
		return;
	}

	while(1)
	{
		ret = adc_read(adc_dev, &sequence);
		sample_value = check_samples(1);
		if(sample_value > 1022)
		{
			motor_event.event = MOTOR_CMD_BRAK_EVENT;
    		k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
		}
		k_sleep(100);
	}
}

void motor_cmd_process(void)
{
	char byte = '\0';
	struct motor_event_type motor_event;
	while(1)
	{
		byte = getchar();
		switch(byte)
		{
			case 'F':
			case 'f':
    			motor_event.event = MOTOR_CMD_FORWARD_EVENT;
    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
				break;
			case 'B':
			case 'b':
				motor_event.event = MOTOR_CMD_BACKWARD_EVENT;
    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
				break;
			case 'S':
			case 's':
				motor_event.event = MOTOR_CMD_BRAK_EVENT;
    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
				break;
			default:
				motor_event.event = MOTOR_CMD_BRAK_EVENT;
    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
		}
	}
}

K_THREAD_DEFINE(motor_process_id, STACKSIZE, motor_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_adc_id, STACKSIZE, motor_adc_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_cmd_id, STACKSIZE, motor_cmd_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);