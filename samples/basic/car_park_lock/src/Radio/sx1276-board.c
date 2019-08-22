/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>

#include "radio.h"
#include "sx1276-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect( uint32_t channel );

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    NULL,//SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    NULL,//SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    NULL,//SX1276GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif
struct device *gpio_ant_sw = NULL;
struct device *gpiosx1276rst = NULL;
#define SX1276_RST_PORT     "GPIOA"
#define SX1276_RST_PIN      8

#define SX1276_DIO_PORT       "GPIOB"
#define SX1276_DIO_0_PORT       "GPIOB"
#define SX1276_DIO_0_PIN        10
#define SX1276_DIO_1_PORT       "GPIOB"
#define SX1276_DIO_1_PIN        2
#define SX1276_DIO_2_PORT       "GPIOB"
#define SX1276_DIO_2_PIN        1
#define SX1276_DIO_3_PORT       "GPIOB"
#define SX1276_DIO_3_PIN        11

void SX1276IoInit( void )
{

    gpiosx1276rst = device_get_binding(SX1276_RST_PORT);
    if (!gpiosx1276rst) {
        printk("%s error\n",__func__);
        return;
    }

    gpio_pin_configure(gpiosx1276rst, SX1276_RST_PIN, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
    gpio_pin_write(gpiosx1276rst, SX1276_RST_PIN, 0);

    gpio_ant_sw = device_get_binding(SX1276_DIO_2_PORT);
    if (!gpio_ant_sw) {
        printk("%s error\n",__func__);
        return;
    }

    gpio_pin_configure(gpio_ant_sw, SX1276_DIO_2_PIN,
               GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
}

#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)
extern struct k_msgq radio_msgq;

void sx1276_dio_irq(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
    struct radio_event_type radio_event;
    u32_t val1 = 0U;
    u32_t val2 = 0U;
    u32_t val3 = 0U;

    gpio_pin_read(gpiob, SX1276_DIO_0_PIN, &val1);
    gpio_pin_read(gpiob, SX1276_DIO_1_PIN, &val2);
    gpio_pin_read(gpiob, SX1276_DIO_3_PIN, &val3);
    if(val1)
    //if(BIT(SX1276_DIO_0_PIN) & pins)
    {
        radio_event.event = SX1276_DIO_0_IRQ_EVENT;
    }
    if(val2)
    //if(BIT(SX1276_DIO_1_PIN) & pins)
    {
        radio_event.event = SX1276_DIO_1_IRQ_EVENT;
    }
    /*if(BIT(SX1276_DIO_2_PIN) & pins)
    {
        radio_event.event = SX1276_DIO_2_IRQ_EVENT;
    }*/
    if(val3)
    //if(BIT(SX1276_DIO_3_PIN) & pins)
    {
        radio_event.event = SX1276_DIO_3_IRQ_EVENT;
    }
    /*switch(pins)
    {
        case SX1276_DIO_0_PIN:
        radio_event.event = SX1276_DIO_0_IRQ_EVENT;
        break;
        case SX1276_DIO_1_PIN:
        radio_event.event = SX1276_DIO_1_IRQ_EVENT;
        break;
        case SX1276_DIO_2_PIN:
        radio_event.event = SX1276_DIO_2_IRQ_EVENT;
        break;
        case SX1276_DIO_3_PIN:
        radio_event.event = SX1276_DIO_3_IRQ_EVENT;
        break;
    }*/
    k_msgq_put(&radio_msgq, &radio_event, K_NO_WAIT);
}

void sx1276_dio0_irq(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
    struct radio_event_type radio_event;
    radio_event.event = SX1276_DIO_0_IRQ_EVENT;
    k_msgq_put(&radio_msgq, &radio_event, K_NO_WAIT);
    printk("%s at %d\n", __func__, k_cycle_get_32());
}

void sx1276_dio1_irq(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
    struct radio_event_type radio_event;
    radio_event.event = SX1276_DIO_1_IRQ_EVENT;
    k_msgq_put(&radio_msgq, &radio_event, K_NO_WAIT);
    printk("%s at %d\n", __func__, k_cycle_get_32());
}

void sx1276_dio2_irq(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
    struct radio_event_type radio_event;
    radio_event.event = SX1276_DIO_2_IRQ_EVENT;
    k_msgq_put(&radio_msgq, &radio_event, K_NO_WAIT);
    printk("%s at %d\n", __func__, k_cycle_get_32());
}

void sx1276_dio3_irq(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
    struct radio_event_type radio_event;
    radio_event.event = SX1276_DIO_3_IRQ_EVENT;
    k_msgq_put(&radio_msgq, &radio_event, K_NO_WAIT);
    printk("%s at %d\n", __func__, k_cycle_get_32());
}

//static struct gpio_callback gpio_dio_0_cb;
//static struct gpio_callback gpio_dio_1_cb;
//static struct gpio_callback gpio_dio_2_cb;
//static struct gpio_callback gpio_dio_3_cb;
static struct gpio_callback gpio_dio_cb;

void SX1276IoIrqInit( void )
{
    struct device *gpiob;

    gpiob = device_get_binding(SX1276_DIO_PORT);
    if (!gpiob) {
        printk("%s error\n",__func__);
        return;
    }
    gpio_init_callback(&gpio_dio_cb, sx1276_dio_irq, BIT(SX1276_DIO_0_PIN) | BIT(SX1276_DIO_1_PIN) | BIT(SX1276_DIO_3_PIN));
    gpio_add_callback(gpiob, &gpio_dio_cb);

    gpio_pin_configure(gpiob, SX1276_DIO_0_PIN,
               GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE);
    gpio_pin_enable_callback(gpiob, SX1276_DIO_0_PIN);

    gpio_pin_configure(gpiob, SX1276_DIO_1_PIN,
               GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE);
    gpio_pin_enable_callback(gpiob, SX1276_DIO_1_PIN);

    /*gpio_ant_sw = device_get_binding(SX1276_DIO_2_PORT);
    if (!gpio_ant_sw) {
        printk("%s error\n",__func__);
        return;
    }

    gpio_pin_configure(gpio_ant_sw, SX1276_DIO_2_PIN,
               GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

    gpio_init_callback(&gpio_dio_2_cb, sx1276_dio2_irq, BIT(SX1276_DIO_2_PIN));

    gpio_add_callback(gpiob, &gpio_dio_2_cb);
    gpio_pin_enable_callback(gpiob, SX1276_DIO_2_PIN);
*/
    gpio_pin_configure(gpiob, SX1276_DIO_3_PIN,
               GPIO_DIR_IN | GPIO_INT |  GPIO_PUD_PULL_UP | EDGE);
    gpio_pin_enable_callback(gpiob, SX1276_DIO_3_PIN);
}

void SX1276IoDeInit( void )
{

}

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
static void SX1276SetBoardTcxo( uint8_t state )
{
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        DelayMs( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

void SX1276Reset( void )
{
    // Enables the TCXO if available on the board design
    SX1276SetBoardTcxo( true );
    gpio_pin_write(gpiosx1276rst, SX1276_RST_PIN, 0);
    k_busy_wait(USEC_PER_MSEC * 1U);
    gpio_pin_write(gpiosx1276rst, SX1276_RST_PIN, 1);
    k_busy_wait(USEC_PER_MSEC * 6U);
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return 1;//BOARD_TCXO_WAKEUP_TIME;
}


void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

static uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    // No antenna switch available.
    // Just control the TCXO if available.
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
        if( status == false )
        {
            SX1276SetBoardTcxo( true );
            SX1276AntSwInit( );
        }
        else
        {
            SX1276SetBoardTcxo( false );
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    #if 0
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO17,PIN_MODE_REPEATER);
    //Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 17);
    GPIO_ResetBits(SX1278_RF_SWITCH_PORT, SX1278_RF_SWITCH_PIN); /* rf switch pin low */
    #endif
    //gpio_pin_write(gpio_ant_sw, SX1276_DIO_2_PIN, 1);
}

void SX1276AntSwDeInit( void )
{
    #if 0
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO17,PIN_MODE_INACTIVE);
    //Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,17);
    GPIO_ResetBits(SX1278_RF_SWITCH_PORT, SX1278_RF_SWITCH_PIN); /* rf switch pin low */
    #endif
    //gpio_pin_write(gpio_ant_sw, SX1276_DIO_2_PIN, 0);
}

void SX1276SetAntSw( uint8_t opMode )
{
    // No antenna switch available
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        gpio_pin_write(gpio_ant_sw, SX1276_DIO_2_PIN, 0);
        //Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,17);
        //GPIO_ResetBits(SX1278_RF_SWITCH_PORT, SX1278_RF_SWITCH_PIN); /* rf switch pin low */
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        gpio_pin_write(gpio_ant_sw, SX1276_DIO_2_PIN, 1);
        //Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,17);
        //GPIO_SetBits(SX1278_RF_SWITCH_PORT, SX1278_RF_SWITCH_PIN); /* rf switch pin high */
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX1276DbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

void SX1276DbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif
