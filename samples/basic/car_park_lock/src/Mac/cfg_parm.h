#ifndef __CFG_PARM_H__
#define __CFG_PARM_H__
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>

/*struct cfg_parm{
    unsigned char *model_type;
    uint32_t radio_freq;
    uint8_t local_addr_h;
    uint8_t local_addr_l;
    uint8_t channel;
    uint32_t radio_rate;
    uint32_t radio_power;
    uint32_t uart_baud;
    USART_WordLength_TypeDef databit;
    USART_StopBits_TypeDef stopbit;
    USART_Parity_TypeDef parity;
};*/

#define IS_BROADCAST_ADDR(addr)     ((addr == 0xffff) || (addr == 0x0000))?1:0

typedef struct cfg_parm st_cfg_parm;
typedef struct cfg_parm *pst_cfg_parm;

struct cfg_pkg{
    unsigned char inNetMode;
    unsigned char netState;
    unsigned char LoRaMacDevEui[8];
    unsigned char LoRaMacAppEui[8];
    unsigned char LoRaMacAppKey[16];
    unsigned char LoRaMacNwkSKey[16];
    unsigned char LoRaMacAppSKey[16];
    uint32_t LoRaMacNetID;
    uint32_t LoRaMacDevAddr;
    uint32_t UpLinkCounter;
    uint32_t DownLinkCounter;
    unsigned char ChannelMask[3];
    unsigned char TxPower;
    uint16_t destencelevel;
};
typedef struct cfg_pkg st_cfg_pkg;
typedef struct cfg_pkg* pst_cfg_pkg;

extern st_cfg_pkg stTmpCfgParm;
extern const uint8_t LoRaMacDevEuiInFlash[];

void cfg_parm_factory_reset(void);
void cfg_parm_restore(void);
void cfg_parm_dump_to_ram(void);
uint8_t cfg_parm_get_tx_power(void);
uint32_t cfg_parm_get_uart_baud(void);
//USART_Parity_TypeDef cfg_parm_get_uart_parity(void);
float cfg_parm_get_air_baud(void);
uint16_t cfg_parm_get_wakeup_time(void);
uint8_t cfg_parm_get_air_bandwith(void);
uint8_t cfg_parm_get_air_sf(void);

#endif