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
#include "cfg_parm.h"
#include "LoRaMac.h"

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

#define PORT_REJOIN_KEY SW0_GPIO_CONTROLLER
#define PORT_CALIBRATE_KEY  SW1_GPIO_CONTROLLER
#define PORT_MOTOR_KEY1_DOWN    SW2_GPIO_CONTROLLER
#define PORT_MOTOR_KEY2_UP  SW3_GPIO_CONTROLLER

#define PIN_REJOIN_KEY     SW0_GPIO_PIN
#define PIN_CALIBRATE_KEY     SW1_GPIO_PIN
#define PIN_MOTOR_KEY1_DOWN     SW2_GPIO_PIN
#define PIN_MOTOR_KEY2_UP     SW3_GPIO_PIN

/*#define EDGE_REJOIN_KEY    (SW0_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_CALIBRATE_KEY    (SW1_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_MOTOR_KEY1    (SW2_GPIO_FLAGS | GPIO_INT_EDGE)
#define EDGE_MOTOR_KEY2    (SW3_GPIO_FLAGS | GPIO_INT_EDGE)

#define PULL_UP_REJOIN_KEY SW0_GPIO_FLAGS
#define PULL_UP_CALIBRATE_KEY SW1_GPIO_FLAGS
#define PULL_UP_MOTOR_KEY1 SW2_GPIO_FLAGS
#define PULL_UP_MOTOR_KEY2 SW3_GPIO_FLAGS*/

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};

typedef enum{
    EN_LOCK_CURRENT_STATUS_DOWN = 0,
    EN_LOCK_CURRENT_STATUS_UP = 1,
    EN_LOCK_GOING_DOWN = 2,
    EN_LOCK_GOING_UP = 3,
}en_LockCurrentStatus,*pen_LockCurrentStatus;

en_LockCurrentStatus enLockCurrentStatus = EN_LOCK_CURRENT_STATUS_DOWN;
un_LockStatus unLockStatus;
uint16_t destensecopy[2] = {0};

K_MSGQ_DEFINE(motor_msgq, sizeof(struct motor_event_type), 20, 4);
struct device *gpio_dev_brak,*gpio_dev_dir,*gpio_dev_pwm,*gpio_dev_key1,*gpio_dev_key2;

void motor_going_down(void)
{
    gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 1); // IN2
    gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 1); // EN
    gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 0); // IN1
    //printk("motor going down\r\n");
}

void motor_going_up(void)
{
    gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 0); // IN2
    gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 1); // EN
    gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 1); // IN1
    printk("motor going up\r\n");
}

void motor_going_stop(void)
{
    gpio_pin_write(gpio_dev_pwm, PIN_MOTOR_PWM, 1); // IN2
    gpio_pin_write(gpio_dev_brak, PIN_MOTOR_BRAK, 1); // EN
    gpio_pin_write(gpio_dev_dir, PIN_MOTOR_DIR, 1); // IN1
    printk("motor going stop\r\n");
}

void motor_process(void)
{
	struct motor_event_type motor_event;
    int ret;
    u32_t val = 0U;

	gpio_dev_brak = device_get_binding(PORT_MOTOR_BRAK);
	__ASSERT_NO_MSG(gpio_dev_brak != NULL);
	gpio_pin_configure(gpio_dev_brak, PIN_MOTOR_BRAK, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	gpio_dev_dir = device_get_binding(PORT_MOTOR_DIR);
	__ASSERT_NO_MSG(gpio_dev_dir != NULL);
	gpio_pin_configure(gpio_dev_dir, PIN_MOTOR_DIR, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	gpio_dev_pwm = device_get_binding(PORT_MOTOR_PWM);
	__ASSERT_NO_MSG(gpio_dev_pwm != NULL);
	gpio_pin_configure(gpio_dev_pwm, PIN_MOTOR_PWM, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

	//my_button_init();
    
    
    while(1)
    {
        switch(enLockCurrentStatus)
        {
            case EN_LOCK_CURRENT_STATUS_DOWN:
                unLockStatus.motor = 0;
                ret = k_msgq_get(&motor_msgq, &motor_event, 1000);
                if(ret == -EAGAIN)
                {
                    gpio_pin_read(gpio_dev_key1, PIN_MOTOR_KEY1_DOWN, &val);
                    if(val != 0)
                    {
                        motor_going_down();
                        enLockCurrentStatus = EN_LOCK_GOING_DOWN;
                    }
                }
                else
                {
                    if(motor_event.event == MOTOR_CMD_UP_EVENT)
                    {
                        motor_going_up();
                        enLockCurrentStatus = EN_LOCK_GOING_UP;
                    }
                }
                break;
            case EN_LOCK_GOING_UP:
                ret = k_msgq_get(&motor_msgq, &motor_event, 1000);
                if(ret == -EAGAIN)
                {
                    gpio_pin_read(gpio_dev_key2, PIN_MOTOR_KEY2_UP, &val);
                    if(val != 0)
                    {
                        motor_going_up();
                        enLockCurrentStatus = EN_LOCK_GOING_UP;
                    }
                }
                else
                {
                    if(motor_event.event == MOTOR_CMD_DOWN_EVENT)
                    {
                        motor_going_down();
                        enLockCurrentStatus = EN_LOCK_GOING_DOWN;
                    }
                    else if(motor_event.event == MOTOR_CURRENT_ADC_BRAK_EVENT)
                    {
                        motor_going_stop();
                    }
                    else if(motor_event.event == MOTOR_CMD_UP_EVENT)
                    {
                        motor_going_up();
                    }
                    else if(motor_event.event == MOTOR_KEY2_UP_EVENT)
                    {
                        motor_going_stop();
                        enLockCurrentStatus = EN_LOCK_CURRENT_STATUS_UP;
                        unLockStatus.motor = 1;
                        if(stTmpCfgParm.netState >= LORAMAC_JOINED)
                        {
                            struct msg_up_event_type msg_up_event;
                            msg_up_event.event = MSG_UP_LOCK_STATUS_CHANGE_EVENT;
                            k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                            printk("send to server\r\n");
                        }
                        printk("up key detect\r\n");
                    }
                }
                break;
            case EN_LOCK_CURRENT_STATUS_UP:
                unLockStatus.motor = 1;
                ret = k_msgq_get(&motor_msgq, &motor_event, 1000);
                if(ret == -EAGAIN)
                {
                    gpio_pin_read(gpio_dev_key2, PIN_MOTOR_KEY2_UP, &val);
                    if(val != 0)
                    {
                        motor_going_up();
                        enLockCurrentStatus = EN_LOCK_GOING_UP;
                    }
                }
                else
                {
                    if(motor_event.event == MOTOR_CMD_DOWN_EVENT)
                    {
                        motor_going_down();
                        enLockCurrentStatus = EN_LOCK_GOING_DOWN;
                    }
                }
                break;
            case EN_LOCK_GOING_DOWN:
                ret = k_msgq_get(&motor_msgq, &motor_event, 1000);
                if(ret == -EAGAIN)
                {
                    gpio_pin_read(gpio_dev_key1, PIN_MOTOR_KEY1_DOWN, &val);
                    if(val != 0)
                    {
                        motor_going_down();
                        enLockCurrentStatus = EN_LOCK_GOING_DOWN;
                    }
                }
                else
                {
                    if(motor_event.event == MOTOR_CMD_UP_EVENT)
                    {
                        motor_going_up();
                        enLockCurrentStatus = EN_LOCK_GOING_UP;
                    }
                    else if(motor_event.event == MOTOR_CURRENT_ADC_BRAK_EVENT)
                    {
                        motor_going_stop();
                    }
                    else if(motor_event.event == MOTOR_CMD_DOWN_EVENT)
                    {
                        motor_going_down();
                    }
                    else if(motor_event.event == MOTOR_KEY1_DOWN_EVENT)
                    {
                        motor_going_stop();
                        enLockCurrentStatus = EN_LOCK_CURRENT_STATUS_DOWN;
                        unLockStatus.motor = 0;
                        if(stTmpCfgParm.netState >= LORAMAC_JOINED)
                        {
                            struct msg_up_event_type msg_up_event;
                            msg_up_event.event = MSG_UP_LOCK_STATUS_CHANGE_EVENT;
                            k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                            printk("send to server\r\n");
                        }
                        printk("down key detect\r\n");
                    }
                }
                break;
            default:
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

		//printk("adc value = %d\r\n", sample_value);
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

    struct device *gpio_dev_rejoin;
    u32_t val1 = 0U;
    u32_t val2 = 0U;
    u32_t val1last = 1U;
    u32_t val2last = 1U;
    gpio_dev_key1 = device_get_binding(PORT_MOTOR_KEY1_DOWN);
    if (!gpio_dev_key1) {
        printk("error\n");
        return;
    }
    gpio_pin_configure(gpio_dev_key1, PIN_MOTOR_KEY1_DOWN,
               GPIO_DIR_IN |  GPIO_PUD_PULL_UP);
    gpio_dev_key2 = device_get_binding(PORT_MOTOR_KEY2_UP);
    if (!gpio_dev_key2) {
        printk("error\n");
        return;
    }
    gpio_pin_configure(gpio_dev_key2, PIN_MOTOR_KEY2_UP,
               GPIO_DIR_IN |  GPIO_PUD_PULL_UP);

    gpio_dev_rejoin = device_get_binding("GPIOF");
    if (!gpio_dev_rejoin) {
        printk("error\n");
        return;
    }

    gpio_pin_configure(gpio_dev_rejoin, PIN_REJOIN_KEY, GPIO_DIR_IN | GPIO_PUD_PULL_UP);

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		printk("do not find adc device\r\n");
		return;
	}

	while(1)
	{
        gpio_pin_read(gpio_dev_key1, PIN_MOTOR_KEY1_DOWN, &val1);
        gpio_pin_read(gpio_dev_key2, PIN_MOTOR_KEY2_UP, &val2);
        //struct motor_event_type motor_event;
        
        if(val1 == 0)
        {
            if(val1 != val1last)
            {
                motor_event.event = MOTOR_KEY1_DOWN_EVENT;
                k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
                printk("key down detect\r\n");
            }
        }
        val1last = val1;
        if(val2 == 0)
        {
            if(val2 != val2last)
            {
                motor_event.event = MOTOR_KEY2_UP_EVENT;
                k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
                printk("key up detect\r\n");
            }
        }
        val2last = val2;
		ret = adc_read(adc_dev, &sequence);
		sample_value = check_samples(1);
		if(sample_value > 500)
		{
			motor_event.event = MOTOR_CURRENT_ADC_BRAK_EVENT;
    		k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
            printk("adc brak detect\r\n");
		}

        
        u32_t val = 0U;
        gpio_pin_read(gpio_dev_rejoin, PIN_REJOIN_KEY, &val);
        if(val == 0)
        {
            do{
                k_sleep(100);
                gpio_pin_read(gpio_dev_rejoin, PIN_REJOIN_KEY, &val);
            }
            while(val == 0);
            cfg_parm_factory_reset();
            struct msg_up_event_type msg_up_event;
            msg_up_event.event = MSG_UP_PARK_STATUS_CHANGE_EVENT;
            k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
            printk("Rejoin again\r\n");
        }
        //printk("get rejoin key at %d,%d\n", k_cycle_get_32(),val);

		k_sleep(10);
	}
}

#define UART2_DEVICE_NAME    "UART_2"
struct k_sem rx_sem;
#define MAX_READ_SIZE   12
#define DATA_SIZE   (12)
struct ring_buf rx_rb_2;
static unsigned char  uRxBuffer_2[DATA_SIZE];
struct ring_buf tx_rb_2;
static unsigned char uTxBuffer_2[DATA_SIZE];
struct device *uart2_dev = NULL;

static void uart2_fifo_callback(struct device *dev)
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
        if((ret = ring_buf_get(&tx_rb_2,read_buf,1)) > 0)
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
            ret = ring_buf_put(&rx_rb_2, read_buf, rx);
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

#define UART1_DEVICE_NAME    "UART_1"
struct k_sem rx_sem;
#define MAX_READ_SIZE   12
#define DATA_SIZE   (12)
struct ring_buf rx_rb_1;
static unsigned char  uRxBuffer_1[DATA_SIZE];
struct ring_buf tx_rb_1;
static unsigned char uTxBuffer_1[DATA_SIZE];
struct device *uart1_dev = NULL;

static void uart1_fifo_callback(struct device *dev)
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
        if((ret = ring_buf_get(&tx_rb_1,read_buf,1)) > 0)
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
            ret = ring_buf_put(&rx_rb_1, read_buf, rx);
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

	uart2_dev = device_get_binding(UART2_DEVICE_NAME);
    uart1_dev = device_get_binding(UART1_DEVICE_NAME);
    k_sem_init(&rx_sem, 0, 1);
    ring_buf_init(&rx_rb_2, DATA_SIZE, uRxBuffer_2);
    ring_buf_init(&tx_rb_2, DATA_SIZE, uTxBuffer_2);
    ring_buf_init(&rx_rb_1, DATA_SIZE, uRxBuffer_1);
    ring_buf_init(&tx_rb_1, DATA_SIZE, uTxBuffer_1);
    /* Verify uart_irq_callback_set() */
    uart_irq_callback_set(uart2_dev, uart2_fifo_callback);
    uart_irq_callback_set(uart1_dev, uart1_fifo_callback);
    /* Enable Tx/Rx interrupt before using fifo */
    /* Verify uart_irq_rx_enable() */
    uart_irq_rx_enable(uart2_dev);
    uart_irq_rx_enable(uart1_dev);

    static uint8_t state_next = 0;
    static uint16_t destense = 0;

    bool sensor1lastvalue = !unLockStatus.sensor1;
    bool sensor2lastvalue = !unLockStatus.sensor2;

	while(1)
	{
		k_sem_take(&rx_sem,K_FOREVER);
		while(ring_buf_get(&rx_rb_2, &byte, 1) > 0)
		{
			switch(byte)
			{
				case 'F':
				case 'f':
	    			motor_event.event = MOTOR_CMD_DOWN_EVENT;
	    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
					break;
				case 'B':
				case 'b':
					motor_event.event = MOTOR_CMD_UP_EVENT;
	    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
					break;
				case 'S':
				case 's':
					motor_event.event = MOTOR_CURRENT_ADC_BRAK_EVENT;
	    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
					break;
				default:
					motor_event.event = MOTOR_CURRENT_ADC_BRAK_EVENT;
	    			k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
			}
		}

        while(ring_buf_get(&rx_rb_1, &byte, 1) > 0)
        {
            //putchar(byte);
            switch(state_next)
            {
                case 0:
                    if(byte == 0xff)
                    {
                        state_next = 1;
                        destense = 0;
                    }
                    break;
                case 1:
                    destense = byte;
                    destense = destense << 8;
                    state_next = 2;
                    break;
                case 2:
                    destense |= byte;
                    state_next = 3;
                    break;
                case 3:
                    if(((((destense & 0xff00) >> 8) + (destense & 0x00ff) + (0xff)) & 0x00ff) == byte)
                    {
                        destensecopy[0] = destense;
                        if(destense < stTmpCfgParm.destencelevel)
                        {
                            unLockStatus.sensor1 = 1;
                            if(sensor1lastvalue != unLockStatus.sensor1)
                            {
                                sensor1lastvalue = unLockStatus.sensor1;
                                struct msg_up_event_type msg_up_event;
                                msg_up_event.event = MSG_UP_PARK_STATUS_CHANGE_EVENT;
                                k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                                printk("car detect destense = %d\r\n",destense);
                            }
                        }
                        else
                        {
                            unLockStatus.sensor1 = 0;
                            if(sensor1lastvalue != unLockStatus.sensor1)
                            {
                                sensor1lastvalue = unLockStatus.sensor1;
                                struct msg_up_event_type msg_up_event;
                                msg_up_event.event = MSG_UP_PARK_STATUS_CHANGE_EVENT;
                                k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                                printk("car not detect destense = %d\r\n",destense);
                            }
                        }
                    }
                    state_next = 0;
                    break;
            }
        }
	}
}

K_THREAD_DEFINE(motor_process_id, 512, motor_process, NULL, NULL, NULL,
		8, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_adc_id, 512, motor_adc_process, NULL, NULL, NULL,
		7, 0, K_NO_WAIT);
K_THREAD_DEFINE(motor_cmd_id, 300, motor_cmd_process, NULL, NULL, NULL,
		7, 0, K_NO_WAIT);