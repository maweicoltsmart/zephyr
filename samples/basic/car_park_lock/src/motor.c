#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include <stdio.h>
#include "motor.h"
#include <adc.h>
#include <ring_buffer.h>
#include <atomic.h>
#include <uart.h>

/* size of stack area used by each thread */
#define STACKSIZE 300

/* scheduling priority used by each thread */
#define PRIORITY 7

#define PORT_MOTOR_BRAK		"GPIOA"
#define PORT_MOTOR_DIR		"GPIOB"
#define PORT_MOTOR_PWM		"GPIOA"

#define PIN_MOTOR_BRAK		7
#define PIN_MOTOR_DIR		0
#define PIN_MOTOR_PWM		6

#define ADC_DEVICE_NAME		DT_ADC_1_NAME
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	4

#define BUFFER_SIZE  1
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
	gpio_pin_configure(gpio_dev_brak, PIN_MOTOR_BRAK, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	gpio_dev_dir = device_get_binding(PORT_MOTOR_DIR);
	__ASSERT_NO_MSG(gpio_dev_dir != NULL);
	gpio_pin_configure(gpio_dev_dir, PIN_MOTOR_DIR, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	gpio_dev_pwm = device_get_binding(PORT_MOTOR_PWM);
	__ASSERT_NO_MSG(gpio_dev_pwm != NULL);
	gpio_pin_configure(gpio_dev_pwm, PIN_MOTOR_PWM, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	my_button_init();
	
    while(1)
    {
        k_msgq_get(&motor_msgq, &motor_event, K_FOREVER);
        switch(motor_event.event)
        {
            case MOTOR_CMD_FORWARD_EVENT:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 1);
                printk("motor go forward\r\n");
                break;
            case MOTOR_CMD_BACKWARD_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                printk("motor go backward\r\n");
                break;
            case MOTOR_KEY1_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                printk("motor go stop\r\n");
                break;
            case MOTOR_KEY2_EVENT:
                gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                printk("motor go stop\r\n");
                break;
            case MOTOR_CMD_BRAK_EVENT:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                printk("motor go stop\r\n");
            	break;
            default:
            	gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0);
                gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 0);
                gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0);
                printk("motor go stop\r\n");
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

		printk("adc value = %d\r\n", sample_value);
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
		k_sleep(1000);
	}
}

#define UART_DEVICE_NAME    "UART_2"
struct k_sem rx_sem;
#define MAX_READ_SIZE   12
#define DATA_SIZE   (12)
struct ring_buf rx_rb;
static unsigned char  uRxBuffer[DATA_SIZE];
struct ring_buf tx_rb;
static unsigned char uTxBuffer[DATA_SIZE];
struct device *uart_dev = NULL;

static void uart_fifo_callback(struct device *dev)
{
    u8_t recvData;
    int rx,ret;
    static u8_t read_buf[MAX_READ_SIZE];

    /* Verify uart_irq_update() */
    if (!uart_irq_update(dev)) {
        //TC_PRINT("retval should always be 1\n");
        return;
    }

    /* Verify uart_irq_tx_ready() */
    /* Note that TX IRQ may be disabled, but uart_irq_tx_ready() may
     * still return true when ISR is called for another UART interrupt,
     * hence additional check for i < DATA_SIZE.
     */
    if (uart_irq_tx_ready(dev)) {
        //printk("tx isr\r\n");
        if((ret = ring_buf_get(&tx_rb,read_buf,1)) > 0)
        {
            ret = uart_fifo_fill(dev,read_buf, ret);
            //printk("ret = %d\r\n",ret);
        }else
        {
            uart_irq_tx_disable(dev);
        }
    }
    /* get all of the data off UART as fast as we can */
    while (uart_irq_update(dev) &&
        uart_irq_rx_ready(dev)) {
        rx = uart_fifo_read(dev, read_buf, sizeof(read_buf));
        if (rx > 0) {
            ret = ring_buf_put(&rx_rb, read_buf, rx);
            if (ret != rx) {
                printk("Rx buffer doesn't have enough space. "
                "Bytes pending: %d, written: %d",
                rx, ret);
                while (uart_fifo_read(dev, &recvData, 1) > 0) {
                    continue;
                }
		k_sem_give(&rx_sem);
		break;
            }
            k_sem_give(&rx_sem);
        }
    }
    /* Verify uart_irq_rx_ready() */
    /*if (uart_irq_rx_ready(dev)) {
        rx = uart_fifo_read(dev, read_buf, sizeof(read_buf));
        if (rx > 0) {
            ret = ring_buf_put(&rx_rb, read_buf, rx);
            if (ret != rx) {
                printk("Rx buffer doesn't have enough space. "
                        "Bytes pending: %d, written: %d",
                        rx, ret);
                while (uart_fifo_read(dev, &recvData, 1) > 0) {
                    continue;
                }
            }
            k_sem_give(&rx_sem);
        }
    }*/
}

void motor_cmd_process(void)
{
	char byte;
	struct motor_event_type motor_event;

	uart_dev = device_get_binding(UART_DEVICE_NAME);
    k_sem_init(&rx_sem, 0, 1);
    ring_buf_init(&rx_rb, DATA_SIZE, uRxBuffer);
    ring_buf_init(&tx_rb, DATA_SIZE, uTxBuffer);

    /* Verify uart_irq_callback_set() */
    uart_irq_callback_set(uart_dev, uart_fifo_callback);

    /* Enable Tx/Rx interrupt before using fifo */
    /* Verify uart_irq_rx_enable() */
    uart_irq_rx_enable(uart_dev);

	while(1)
	{
		k_sem_take(&rx_sem,K_FOREVER);
		while(ring_buf_get(&rx_rb, &byte, 1) > 0)
		{
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
}

K_THREAD_DEFINE(motor_process_id, STACKSIZE, motor_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_adc_id, 512, motor_adc_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_cmd_id, STACKSIZE, motor_cmd_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);