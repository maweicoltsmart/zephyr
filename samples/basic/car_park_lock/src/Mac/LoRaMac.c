/*!
 * \file      LoRaMac.c
 *
 * \brief     LoRa MAC layer implementation
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 */
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include "aes.h"
#include "cmac.h"
#include "utilities.h"
#include "LoRaMac.h"
#include "LoRaMacCrypto.h"
#include "cfg_parm.h"
#include "radio.h"
#include "sx1276-board.h"
#include "spi-board.h"
#include "motor.h"
/* size of stack area used by each thread */
#define STACKSIZE 512

/* scheduling priority used by each thread */
#define PRIORITY 7
/*!
 * Maximum PHY layer payload size
 */
#define LORAMAC_PHY_MAXPAYLOAD                      1
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
/*!
 * Maximum length of the fOpts field
 */
#define LORA_MAC_COMMAND_MAX_FOPTS_LENGTH           15

/*!
 * Device nonce is a random value extracted by issuing a sequence of RSSI
 * measurements
 */
static uint16_t LoRaMacDevNonce,LoRaMacDevNonceCopy;

/*!
 * Radio events function pointer
 */
static RadioEvents_t LoRaMacRadioEvents;

/*!
 * Acknowledge timeout timer. Used for packet retransmissions.
 */
//TimerEvent_t AckTimeoutTimer;
uint32_t TxDoneTimerTick;
uint8_t GlobalChannel;
uint8_t GlobalDR;
LoRaMacFrameCtrl_t RxfCtrl;
uint8_t RxDataLen = 0;
uint8_t *pRxDataBuf = NULL;
uint8_t RadioTxBuffer[256];
K_SEM_DEFINE(radio_can_tx_sem, 1, 1);
/*!
 * \brief Function to be executed on Radio Tx Done event
 */
static void LoRaMacOnRadioTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
static void LoRaMacOnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
static void LoRaMacOnRadioTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx error event
 */
static void LoRaMacOnRadioRxError( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
static void LoRaMacOnRadioRxTimeout( void );
static void SetWorkChannel(void)
{
    uint8_t channellist[24];
    uint8_t enablechannel = 0;
    static uint8_t channelupdate = 0;

    GlobalDR = 12;
    
    for(uint8_t loop1 = 0;loop1 < 3;loop1 ++)
    {
        for(uint8_t loop2 = 0;loop2 < 8;loop2 ++)
        {
            if(stTmpCfgParm.ChannelMask[loop1] & (1 << loop2))
            {
                channellist[enablechannel] = loop1 * 8 + loop2;
                enablechannel ++;
            }
        }
    }
    if(enablechannel < 1)
    {
        printk("No channel enabled\r\n");
        GlobalChannel = 0;
        return;
    }
    if(stTmpCfgParm.netState >= LORAMAC_JOINED)
    {
        GlobalChannel = channellist[stTmpCfgParm.LoRaMacDevEui[0] % 2];
    }
    else
    {
        do{
            if(channelupdate > 23)
            {
                channelupdate = 0;
            }
            GlobalChannel = channellist[channelupdate ++];//[loop3 % enablechannel];
        }while(!Radio.IsChannelFree ( MODEM_LORA, GlobalChannel * 200000 + 428200000, -90, 5 ));
    }
    printk("channel = %d\r\n",GlobalChannel);
}

static void RadioSetTx(void)
{
    Radio.Sleep( );
    Radio.SetChannel( GlobalChannel * 200000 + 428200000 );
    Radio.SetTxConfig( MODEM_LORA, stTmpCfgParm.TxPower, 0, 0,
                                   GlobalDR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, false, 0, LORA_IQ_INVERSION_ON, 4000 );
}

static void RadioSetRx(void)
{
    Radio.Sleep( );
    Radio.SetChannel( (GlobalChannel + 24)* 200000 + 428200000 );

    Radio.SetRxConfig( MODEM_LORA, 0, GlobalDR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, false, 0, LORA_IQ_INVERSION_ON, true );
}

static void LoRaMacOnRadioTxDone( void )
{
    printk("%s\r\n",__func__);
    RadioSetRx();
    Radio.Rx(0);
    if(stTmpCfgParm.netState >= LORAMAC_JOINED)
    {
        stTmpCfgParm.netState = LORAMAC_JOINED_IDLE;
    }
    k_sem_give(&radio_can_tx_sem);
}

static void LoRaMacOnRadioRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    LoRaMacHeader_t macHdr;
    //LoRaMacFrameCtrl_t fCtrl;

    uint8_t pktHeaderLen = 0;
    uint32_t address = 0;
    uint8_t appPayloadStartIndex = 0;
    uint8_t port = 0xFF;
    uint8_t frameLen = 0;
    uint32_t mic = 0;
    uint32_t micRx = 0;
    uint8_t *LoRaMacRxPayload = (uint8_t *)RadioTxBuffer;
    uint16_t sequenceCounter = 0;
    uint16_t sequenceCounterPrev = 0;
    uint16_t sequenceCounterDiff = 0;
    uint32_t downLinkCounter = 0;

    uint8_t *nwkSKey = stTmpCfgParm.LoRaMacNwkSKey;
    uint8_t *appSKey = stTmpCfgParm.LoRaMacAppSKey;

    bool isMicOk = false;
    Radio.Sleep( );
    macHdr.Value = payload[pktHeaderLen++];
    switch( macHdr.Bits.MType )
    {
        case FRAME_TYPE_JOIN_ACCEPT:
            if( stTmpCfgParm.netState >= LORAMAC_JOINED )
            {
                return;
            }
            LoRaMacJoinDecrypt( payload + 1, size - 1, stTmpCfgParm.LoRaMacAppKey, LoRaMacRxPayload + 1 );
            LoRaMacRxPayload[0] = macHdr.Value;
            LoRaMacJoinComputeMic( LoRaMacRxPayload, size - LORAMAC_MFR_LEN, stTmpCfgParm.LoRaMacAppKey, &mic );
            mic += LoRaMacDevNonce;
            micRx |= ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN];
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 1] << 8 );
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 2] << 16 );
            micRx |= ( ( uint32_t )LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 3] << 24 );
            if( micRx == mic )
            {
                LoRaMacJoinComputeSKeys( stTmpCfgParm.LoRaMacAppKey, LoRaMacRxPayload + 1, LoRaMacDevNonceCopy, stTmpCfgParm.LoRaMacNwkSKey, stTmpCfgParm.LoRaMacAppSKey );

                stTmpCfgParm.LoRaMacNetID = ( uint32_t )LoRaMacRxPayload[4];
                stTmpCfgParm.LoRaMacNetID |= ( ( uint32_t )LoRaMacRxPayload[5] << 8 );
                stTmpCfgParm.LoRaMacNetID |= ( ( uint32_t )LoRaMacRxPayload[6] << 16 );

                stTmpCfgParm.LoRaMacDevAddr = ( uint32_t )LoRaMacRxPayload[7];
                stTmpCfgParm.LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[8] << 8 );
                stTmpCfgParm.LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[9] << 16 );
                stTmpCfgParm.LoRaMacDevAddr |= ( ( uint32_t )LoRaMacRxPayload[10] << 24 );
                stTmpCfgParm.ChannelMask[0] = stTmpCfgParm.LoRaMacNetID & 0x000000ff;
                stTmpCfgParm.ChannelMask[1] = (stTmpCfgParm.LoRaMacNetID & 0x0000ff00) >> 8;
                stTmpCfgParm.ChannelMask[2] = (stTmpCfgParm.LoRaMacNetID & 0x00ff0000) >> 16;
                stTmpCfgParm.netState = LORAMAC_JOINED;
                cfg_parm_restore();
                
                //onEvent(EV_JOINED);
            }
            break;
        case FRAME_TYPE_DATA_CONFIRMED_DOWN:
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
            {
                if( stTmpCfgParm.netState < LORAMAC_JOINED )
                {
                    RadioSetRx();
                    Radio.Rx(0);
                    return;
                }
                // Check if the received payload size is valid

                address = payload[pktHeaderLen++];
                address |= ( (uint32_t)payload[pktHeaderLen++] << 8 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 16 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 24 );

                if( address != stTmpCfgParm.LoRaMacDevAddr )
                {
                    RadioSetRx();
                    Radio.Rx(0);
                    return;
                }
                else
                {
                    nwkSKey = stTmpCfgParm.LoRaMacNwkSKey;
                    appSKey = stTmpCfgParm.LoRaMacAppSKey;
                    downLinkCounter = stTmpCfgParm.DownLinkCounter;
                }

                RxfCtrl.Value = payload[pktHeaderLen++];

                sequenceCounter = ( uint16_t )payload[pktHeaderLen++];
                sequenceCounter |= ( uint16_t )payload[pktHeaderLen++] << 8;

                appPayloadStartIndex = 8 + RxfCtrl.Bits.FOptsLen;

                micRx |= ( uint32_t )payload[size - LORAMAC_MFR_LEN];
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 1] << 8 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 2] << 16 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 3] << 24 );

                sequenceCounterPrev = ( uint16_t )downLinkCounter;
                sequenceCounterDiff = ( sequenceCounter - sequenceCounterPrev );

                if( sequenceCounterDiff < ( 1 << 15 ) )
                {
                    downLinkCounter += sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                    }
                }
                else
                {
                    // check for sequence roll-over
                    uint32_t  downLinkCounterTmp = downLinkCounter + 0x10000 + ( int16_t )sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounterTmp, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                        downLinkCounter = downLinkCounterTmp;
                    }
                }

                if( isMicOk == true )
                {
                    if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
                    {
                        port = payload[appPayloadStartIndex++];
                        frameLen = ( size - 4 ) - appPayloadStartIndex;
                        LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
                                                       frameLen,
                                                       appSKey,
                                                       address,
                                                       DOWN_LINK,
                                                       downLinkCounter,
                                                       LoRaMacRxPayload );
                        if(LoRaMacRxPayload[0] == 0) // check
                        {
                            struct msg_up_event_type msg_up_event;
                            msg_up_event.event = MSG_UP_PARK_STATUS_REQ_EVENT;
                            k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                            RadioSetRx();
                            Radio.Rx(0);
                            return;
                        }
                        else if(LoRaMacRxPayload[0] == 1) // set destence param
                        {
                            stTmpCfgParm.destencelevel = LoRaMacRxPayload[1] | LoRaMacRxPayload[2] << 8;
                        }
                        else if(LoRaMacRxPayload[0] == 2) // motor cmd
                        {
                            if(LoRaMacRxPayload[1] == 0)
                            {
                                struct motor_event_type motor_event;
                                motor_event.event = MOTOR_CMD_DOWN_EVENT;
                                k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
                            }
                            else if(LoRaMacRxPayload[1] == 1)
                            {
                                struct motor_event_type motor_event;
                                motor_event.event = MOTOR_CMD_UP_EVENT;
                                k_msgq_put(&motor_msgq, &motor_event, K_NO_WAIT);
                            }
                        }

                        #if 0
                        //onEvent(EV_RXCOMPLETE);
                        if(RxfCtrl.Bits.Ack)
                        {
                            //if(stTmpCfgParm.netState == LORAMAC_TX_WAIT_RESP1)
                            {
                                ringbuf_put(&uart_tx_ring_buf,',');
                                ringbuf_put(&uart_tx_ring_buf,'A');
                                ringbuf_put(&uart_tx_ring_buf,'1');
                            }
                            /*if(stTmpCfgParm.netState == LORAMAC_TX_WAIT_RESP2)
                            {
                                putchar(',');
                                putchar('A');
                                putchar('2');
                            }*/
                        }
                        // RxDataLen = puthex (payload, (const uint8_t*) RadioTxBuffer, RxDataLen);
                        ringbuf_put(&uart_tx_ring_buf,',');
                        uint8_t temphexbyte[2];
                        RxDataLen = 0;
                        RxDataLen += puthex (temphexbyte, &port, 1);
                        ring_buffer_queue_arr(&uart_tx_ring_buf, temphexbyte, 2);
                        ringbuf_put(&uart_tx_ring_buf,',');
                        RxDataLen += 1;
                        for(uint8_t loop8 = 0;loop8 < frameLen;loop8++)
                        {
                            RxDataLen += puthex (temphexbyte, (const uint8_t*) &LoRaMacRxPayload[loop8], 1);
                            ring_buffer_queue_arr(&uart_tx_ring_buf, temphexbyte, 2);
                        }
                        ringbuf_put(&uart_tx_ring_buf,'\r');
                        ringbuf_put(&uart_tx_ring_buf,'\n');
#endif
                    }
                }
            }
            if(stTmpCfgParm.netState >= LORAMAC_JOINED)
            {
                stTmpCfgParm.netState = LORAMAC_JOINED_IDLE;
            }
            break;
        case FRAME_TYPE_PROPRIETARY:
            {
                break;
            }
        default:
                break;
    }
    RadioSetRx();
    Radio.Rx(0);
}

static void LoRaMacOnRadioTxTimeout( void )
{
    RadioSetRx();
    Radio.Rx(0);
    if(stTmpCfgParm.netState >= LORAMAC_JOINED)
    {
        stTmpCfgParm.netState = LORAMAC_JOINED_IDLE;
    }
    k_sem_give(&radio_can_tx_sem);
}

static void LoRaMacOnRadioRxError( void )
{
    RadioSetRx();
    Radio.Rx(0);
}

static void LoRaMacOnRadioRxTimeout( void )
{
    RadioSetRx();
    Radio.Rx(0);
}

void LoRaMacOnRadioCadDone( bool channelActivityDetected )
{
    Radio.Sleep( );
    //printf("cadone\r\n");
    if(channelActivityDetected)
    {
        //RTC_WakeUpCmd(DISABLE);
        //printf("rx\r\n");
        Radio.Rx( 0 );
    }
    else
    {
        //halt();
    }
}

LoRaMacStatus_t SendFrameOnChannel( uint8_t channel,uint8_t *data,uint8_t len,uint8_t confirm,uint8_t fPort)
{
    uint8_t *LoRaMacBuffer = (uint8_t *)RadioTxBuffer;
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    // uint8_t fPort = 1;
    uint8_t pktHeaderLen = 0;
    uint32_t mic = 0;
    
    macHdr.Bits.Major = 1;
    if(confirm == 1)
        macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
    else
        macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
    //if(GetRunModePin() == En_Normal_Mode)
    {
        macHdr.Bits.RFU = CLASS_C;
    }
    /*else if(GetRunModePin() == En_Low_Power_Mode)
    {
        macHdr.Bits.RFU = CLASS_B;
    }
    else
    {
        macHdr.Bits.RFU = CLASS_A;
    }*/
    //macHdr.Bits.RFU = CLASS_C;
    fCtrl.Value = 0;

    LoRaMacBuffer[pktHeaderLen++] = macHdr.Value;
    LoRaMacBuffer[pktHeaderLen++] = ( stTmpCfgParm.LoRaMacDevAddr ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( stTmpCfgParm.LoRaMacDevAddr >> 8 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( stTmpCfgParm.LoRaMacDevAddr >> 16 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( stTmpCfgParm.LoRaMacDevAddr >> 24 ) & 0xFF;

    LoRaMacBuffer[pktHeaderLen++] = fCtrl.Value;

    LoRaMacBuffer[pktHeaderLen++] = stTmpCfgParm.UpLinkCounter & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( stTmpCfgParm.UpLinkCounter >> 8 ) & 0xFF;

    // Store MAC commands which must be re-send in case the device does not receive a downlink anymore
    //MacCommandsBufferToRepeatIndex = ParseMacCommandsToRepeat( MacCommandsBuffer, MacCommandsBufferIndex, MacCommandsBufferToRepeat );
              
    LoRaMacBuffer[pktHeaderLen++] = fPort;

    LoRaMacPayloadEncrypt( (uint8_t* ) data, len, stTmpCfgParm.LoRaMacAppSKey, stTmpCfgParm.LoRaMacDevAddr, UP_LINK, stTmpCfgParm.UpLinkCounter, &LoRaMacBuffer[pktHeaderLen] );
              
    pktHeaderLen = pktHeaderLen + len;

    LoRaMacComputeMic( LoRaMacBuffer, pktHeaderLen, stTmpCfgParm.LoRaMacNwkSKey, stTmpCfgParm.LoRaMacDevAddr, UP_LINK, stTmpCfgParm.UpLinkCounter, &mic );

    LoRaMacBuffer[pktHeaderLen ++] = mic & 0xFF;
    LoRaMacBuffer[pktHeaderLen ++] = ( mic >> 8 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen ++] = ( mic >> 16 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen ++] = ( mic >> 24 ) & 0xFF;
    RadioSetTx();    // Send now
    Radio.Send( LoRaMacBuffer, pktHeaderLen );
    printk("%s, %d\r\n",__func__,GlobalChannel);
    return true;
}

LoRaMacStatus_t SendJoinRequest( void )
{
    uint8_t *LoRaMacBuffer = (uint8_t *)RadioTxBuffer;
    LoRaMacHeader_t macHdr;
    uint8_t pktHeaderLen = 0;
    uint32_t mic = 0;
    
    macHdr.Bits.Major = 1;
    macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;
    //if(GetRunModePin() == En_Normal_Mode)
    {
        macHdr.Bits.RFU = CLASS_C;
    }
    /*else if(GetRunModePin() == En_Low_Power_Mode)
    {
        macHdr.Bits.RFU = CLASS_B;
    }
    else
    {
        macHdr.Bits.RFU = CLASS_A;
    }*/
    LoRaMacBuffer[pktHeaderLen++] = macHdr.Value;
    memcpyr( LoRaMacBuffer + pktHeaderLen, stTmpCfgParm.LoRaMacAppEui, 8 );
    pktHeaderLen += 8;
    memcpyr( LoRaMacBuffer + pktHeaderLen, stTmpCfgParm.LoRaMacDevEui, 8 );
    pktHeaderLen += 8;

    LoRaMacDevNonce = rand1();//Radio.Random( );
    LoRaMacDevNonceCopy = LoRaMacDevNonce;//(LoRaMacDevNonce << 8) | (LoRaMacDevNonce >> 8);
    LoRaMacBuffer[pktHeaderLen++] = LoRaMacDevNonce & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

    LoRaMacJoinComputeMic( LoRaMacBuffer, pktHeaderLen & 0xFF, stTmpCfgParm.LoRaMacAppKey, &mic );

    LoRaMacBuffer[pktHeaderLen++] = mic & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( mic >> 8 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( mic >> 16 ) & 0xFF;
    LoRaMacBuffer[pktHeaderLen++] = ( mic >> 24 ) & 0xFF;
    // BoardDisableIrq();
    Radio.Sleep( );
    //RTC_WakeUpCmd(DISABLE);
    SetWorkChannel();
    RadioSetTx();
    // Send now
    Radio.Send( LoRaMacBuffer, pktHeaderLen );
    // BoardEnableIrq();
    //onEvent(EV_JOINING);
    return LORAMAC_STATUS_OK;
}
LoRaMacStatus_t LoRaMacInitialization( void )
{
    //modem_init ();
    //PWR_UltraLowPowerCmd(DISABLE); // TIM2Ê±ÖÓ»áÓÐÑÓ³Ù
    BoardDisableIrq();
    cfg_parm_dump_to_ram();
    
    SpiInit();
    SX1276IoInit();
    //TIM4_Config();
    LoRaMacRadioEvents.TxDone = LoRaMacOnRadioTxDone;
    LoRaMacRadioEvents.RxDone = LoRaMacOnRadioRxDone;
    LoRaMacRadioEvents.RxError = LoRaMacOnRadioRxError;
    LoRaMacRadioEvents.TxTimeout = LoRaMacOnRadioTxTimeout;
    LoRaMacRadioEvents.RxTimeout = LoRaMacOnRadioRxTimeout;
    LoRaMacRadioEvents.CadDone = LoRaMacOnRadioCadDone;
    Radio.Init( &LoRaMacRadioEvents );
    //factory = 433000000;
    // RadioSetTx();
    // RadioSetRx();    
    Radio.Sleep( );
    Radio.SetPublicNetwork( 0 );
    //printf("net mode\r\n");
    //GPIO_SetBits(SX1278_AUX_PORT, SX1278_AUX_PIN);
    // Random seed initialization
    srand1( Radio.Random() );
    //RTC_Config();
    ////RTC_WakeUpCmd(DISABLE);
    //ComportInit();
    BoardEnableIrq();
    return LORAMAC_STATUS_OK;
}
K_MSGQ_DEFINE(msgup_msgq, sizeof(struct motor_event_type), 20, 4);

void LoRaMacStateCheck( void )
{    
    struct msg_up_event_type msg_up_event;
    int ret;

    LoRaMacInitialization();
    while(true)
    {
        switch(stTmpCfgParm.netState)
        {
            case LORAMAC_IDLE:
                SendJoinRequest();
                printk("Joinning\r\n");
                k_sleep(K_MSEC(5000));
                break;
            case LORAMAC_JOINING:

                break;
            case LORAMAC_JOINED:
                msg_up_event.event = MSG_UP_LOCK_STATUS_CHANGE_EVENT;
                k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                k_msgq_put(&msgup_msgq, &msg_up_event, K_NO_WAIT);
                SetWorkChannel();
                printk("joined\r\n");
                stTmpCfgParm.netState = LORAMAC_JOINED_IDLE;
            case LORAMAC_JOINED_IDLE:
                k_sem_take(&radio_can_tx_sem, K_FOREVER);
                ret = k_msgq_get(&msgup_msgq, &msg_up_event, K_FOREVER);

                uint8_t temp[5];
                temp[0] = unLockStatus.value;
                temp[1] = destensecopy[0] & 0x00ff;
                temp[2] = (destensecopy[0] >> 8) & 0x00ff;
                temp[3] = destensecopy[1] & 0x00ff;
                temp[4] = (destensecopy[1] >> 8) & 0x00ff;
                SendFrameOnChannel( 0,temp,5,false,1);
                break;
            case LORAMAC_TX_ING:
                break;
            default:
                break;
        }
    }
}

K_MSGQ_DEFINE(radio_msgq, sizeof(struct radio_event_type), 20, 4);

extern void SX1276OnDio0Irq( void* context );
extern void SX1276OnDio1Irq( void* context );
extern void SX1276OnDio2Irq( void* context );
extern void SX1276OnDio3Irq( void* context );
extern void SX1276OnTimeoutIrqCallByThread( void );

void LoRaRadioEventCheck( void )
{
    struct radio_event_type radio_event;
    while(1)
    {
        k_msgq_get(&radio_msgq, &radio_event, K_FOREVER);
        
        switch(radio_event.event)
        {
            case SX1276_DIO_0_IRQ_EVENT:
                //printk("%s ,%d\r\n",__func__,__LINE__);
                SX1276OnDio0Irq(NULL);
                break;
            case SX1276_DIO_1_IRQ_EVENT:
                //printk("%s ,%d\r\n",__func__,__LINE__);
                SX1276OnDio1Irq(NULL);
                break;
            case SX1276_DIO_2_IRQ_EVENT:
                //printk("%s ,%d\r\n",__func__,__LINE__);
                SX1276OnDio2Irq(NULL);
                break;
            case SX1276_DIO_3_IRQ_EVENT:
                //printk("%s ,%d\r\n",__func__,__LINE__);
                SX1276OnDio3Irq(NULL);
                break;
            case SX1276_TIMEOUT_EVENT:
                //printk("%s ,%d\r\n",__func__,__LINE__);
                SX1276OnTimeoutIrqCallByThread();
                break;
        }
    }
}

K_THREAD_DEFINE(lora_mac_id, 512, LoRaMacStateCheck, NULL, NULL, NULL,
        PRIORITY, 0, K_NO_WAIT);
K_THREAD_DEFINE(lora_radio_id, 512, LoRaRadioEventCheck, NULL, NULL, NULL,
        PRIORITY, 0, K_NO_WAIT);
