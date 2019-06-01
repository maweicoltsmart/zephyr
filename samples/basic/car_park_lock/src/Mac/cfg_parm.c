#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include "cfg_parm.h"
#include <stdio.h>
#include "LoRaMac.h"

st_cfg_pkg stNvCfgParm;
st_cfg_pkg stTmpCfgParm;
uint8_t LoRaMacDevEuiInFlash[8] = {0x01,0x00,0x00,0x00,0x14,0x03,0x19,0x20};
uint8_t factorystring[250] = {0x00};
void cfg_parm_factory_reset(void)
{
    memset(&stTmpCfgParm,0,5);
    stTmpCfgParm.inNetMode = true;
    stTmpCfgParm.netState = LORAMAC_IDLE;
    //{ 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
    //stTmpCfgParm.LoRaMacDevEui = ;
    stTmpCfgParm.LoRaMacAppEui[0] = 'M';
    stTmpCfgParm.LoRaMacAppEui[1] = 'J';
    stTmpCfgParm.LoRaMacAppEui[2] = '-';
    stTmpCfgParm.LoRaMacAppEui[3] = 'M';
    stTmpCfgParm.LoRaMacAppEui[4] = 'o';
    stTmpCfgParm.LoRaMacAppEui[5] = 'd';
    stTmpCfgParm.LoRaMacAppEui[6] = 'e';
    stTmpCfgParm.LoRaMacAppEui[7] = 'm';
    stTmpCfgParm.LoRaMacAppKey[0] = 0x2B;
    stTmpCfgParm.LoRaMacAppKey[1] = 0x7E;
    stTmpCfgParm.LoRaMacAppKey[2] = 0x15;
    stTmpCfgParm.LoRaMacAppKey[3] = 0x16;
    stTmpCfgParm.LoRaMacAppKey[4] = 0x28;
    stTmpCfgParm.LoRaMacAppKey[5] = 0xAE;
    stTmpCfgParm.LoRaMacAppKey[6] = 0xD2;
    stTmpCfgParm.LoRaMacAppKey[7] = 0xA6;

    stTmpCfgParm.LoRaMacAppKey[8] = 0xAB;
    stTmpCfgParm.LoRaMacAppKey[9] = 0xF7;
    stTmpCfgParm.LoRaMacAppKey[10] = 0x15;
    stTmpCfgParm.LoRaMacAppKey[11] = 0x88;
    stTmpCfgParm.LoRaMacAppKey[12] = 0x09;
    stTmpCfgParm.LoRaMacAppKey[13] = 0xCF;
    stTmpCfgParm.LoRaMacAppKey[14] = 0x4F;
    stTmpCfgParm.LoRaMacAppKey[15] = 0x3C;
    //stTmpCfgParm.LoRaMacNwkSKey = ;
    //stTmpCfgParm.LoRaMacAppSKey = ;
    stTmpCfgParm.UpLinkCounter = 0;
    stTmpCfgParm.DownLinkCounter = 0;
    stTmpCfgParm.ChannelMask[0] = 0x41;
    stTmpCfgParm.ChannelMask[1] = 0x00;
    stTmpCfgParm.ChannelMask[2] = 0x00;
    stTmpCfgParm.TxPower = 20;

    stNvCfgParm.inNetMode = stTmpCfgParm.inNetMode;
    stNvCfgParm.netState = stTmpCfgParm.netState;
    stNvCfgParm.UpLinkCounter = stTmpCfgParm.UpLinkCounter;
    stNvCfgParm.DownLinkCounter = stTmpCfgParm.DownLinkCounter;
    for(uint8_t i = 0;i < 8;i ++)
    {
        stTmpCfgParm.LoRaMacDevEui[i] = LoRaMacDevEuiInFlash[i];
        stNvCfgParm.LoRaMacDevEui[i] = LoRaMacDevEuiInFlash[i];
        stNvCfgParm.LoRaMacAppEui[i] = stTmpCfgParm.LoRaMacAppEui[i];
    }
    for(uint8_t i = 0;i < 16;i ++)
    {
        stNvCfgParm.LoRaMacAppKey[i] = stTmpCfgParm.LoRaMacAppKey[i];
    }
    stNvCfgParm.ChannelMask[0] = stTmpCfgParm.ChannelMask[0];
    stNvCfgParm.ChannelMask[1] = stTmpCfgParm.ChannelMask[1];
    stNvCfgParm.ChannelMask[2] = stTmpCfgParm.ChannelMask[2];
    stNvCfgParm.TxPower = stTmpCfgParm.TxPower;
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stTmpCfgParm.addr_h,stTmpCfgParm.addr_l,stTmpCfgParm.speed.speed,stTmpCfgParm.channel.channel,stTmpCfgParm.option.option);
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stNvCfgParm.addr_h,stNvCfgParm.addr_l,stNvCfgParm.speed.speed,stNvCfgParm.channel.channel,stNvCfgParm.option.option);
}

void cfg_parm_restore(void)
{
    stNvCfgParm.inNetMode = stTmpCfgParm.inNetMode;
    stNvCfgParm.netState = stTmpCfgParm.netState;
    
    for(uint8_t i = 0;i < 8;i ++)
      stNvCfgParm.LoRaMacAppEui[i] = stTmpCfgParm.LoRaMacAppEui[i];
    for(uint8_t i = 0;i < 16;i ++)
    {
        stNvCfgParm.LoRaMacAppKey[i] = stTmpCfgParm.LoRaMacAppKey[i];
        stNvCfgParm.LoRaMacNwkSKey[i] = stTmpCfgParm.LoRaMacNwkSKey[i];
        stNvCfgParm.LoRaMacAppSKey[i] = stTmpCfgParm.LoRaMacAppSKey[i];
    }
      
    stNvCfgParm.LoRaMacNetID = stTmpCfgParm.LoRaMacNetID;
    stNvCfgParm.LoRaMacDevAddr = stTmpCfgParm.LoRaMacDevAddr;
    stNvCfgParm.UpLinkCounter = stTmpCfgParm.UpLinkCounter;
    stNvCfgParm.DownLinkCounter = stTmpCfgParm.DownLinkCounter;
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stTmpCfgParm.addr_h,stTmpCfgParm.addr_l,stTmpCfgParm.speed.speed,stTmpCfgParm.channel.channel,stTmpCfgParm.option.option);
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stNvCfgParm.addr_h,stNvCfgParm.addr_l,stNvCfgParm.speed.speed,stNvCfgParm.channel.channel,stNvCfgParm.option.option);
    stNvCfgParm.ChannelMask[0] = stTmpCfgParm.ChannelMask[0];
    stNvCfgParm.ChannelMask[1] = stTmpCfgParm.ChannelMask[1];
    stNvCfgParm.ChannelMask[2] = stTmpCfgParm.ChannelMask[2];
    stNvCfgParm.TxPower = stTmpCfgParm.TxPower;
}

void cfg_parm_dump_to_ram(void)
{
    stTmpCfgParm.inNetMode = stNvCfgParm.inNetMode;
    stTmpCfgParm.netState = stNvCfgParm.netState;
    
    for(uint8_t i = 0;i < 8;i ++)
    {
        stTmpCfgParm.LoRaMacDevEui[i] = stNvCfgParm.LoRaMacDevEui[i];
        stTmpCfgParm.LoRaMacAppEui[i] = stNvCfgParm.LoRaMacAppEui[i];
    }
    
    for(uint8_t i = 0;i < 16;i ++)
    {
        stTmpCfgParm.LoRaMacAppKey[i] = stNvCfgParm.LoRaMacAppKey[i];
        stTmpCfgParm.LoRaMacNwkSKey[i] = stNvCfgParm.LoRaMacNwkSKey[i];
        stTmpCfgParm.LoRaMacAppSKey[i] = stNvCfgParm.LoRaMacAppSKey[i];
    }
      
    stTmpCfgParm.LoRaMacNetID = stNvCfgParm.LoRaMacNetID;
    stTmpCfgParm.LoRaMacDevAddr = stNvCfgParm.LoRaMacDevAddr;
    stTmpCfgParm.UpLinkCounter = stNvCfgParm.UpLinkCounter;
    stTmpCfgParm.DownLinkCounter = stNvCfgParm.DownLinkCounter;
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stTmpCfgParm.addr_h,stTmpCfgParm.addr_l,stTmpCfgParm.speed.speed,stTmpCfgParm.channel.channel,stTmpCfgParm.option.option);
    //printf("%s : %02X %02X %02X %02X %02X \r\n",__func__,stNvCfgParm.addr_h,stNvCfgParm.addr_l,stNvCfgParm.speed.speed,stNvCfgParm.channel.channel,stNvCfgParm.option.option);
    stTmpCfgParm.ChannelMask[0] = stNvCfgParm.ChannelMask[0];
    stTmpCfgParm.ChannelMask[1] = stNvCfgParm.ChannelMask[1];
    stTmpCfgParm.ChannelMask[2] = stNvCfgParm.ChannelMask[2];
    stTmpCfgParm.TxPower = stNvCfgParm.TxPower;
}
