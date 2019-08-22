/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
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
#include "spi-board.h"
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include <spi.h>

#define SPI_DRV_NAME    "SPI_2"
#define SPI_CS_CTRL_GPIO_DRV_NAME   "GPIOB"
#define SPI_CS_CTRL_GPIO_PIN    12

#define SPI_SLAVE   1
#define SLOW_FREQ   1000000
#define FAST_FREQ   1000000

struct spi_config spi_cfg_fast = {
    .frequency = FAST_FREQ,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB  | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .slave = SPI_SLAVE,
    //.cs = SPI_CS,
};

struct spi_cs_control spi_cs = {
    .gpio_pin = SPI_CS_CTRL_GPIO_PIN,
    .delay = 0,
};

struct device *spi_slow;
struct device *spi_fast;

void SpiInit( void )
{
    spi_fast = device_get_binding(SPI_DRV_NAME);
    if (!spi_fast) {
        printk("No aviable SPI Device\r\n");
        return;
    }
    spi_cs.gpio_dev = device_get_binding(SPI_CS_CTRL_GPIO_DRV_NAME);
    if (!spi_cs.gpio_dev) {
        printk("No aviable SPI cs pin\r\n");
        return;
    }
    gpio_pin_configure(spi_cs.gpio_dev, spi_cs.gpio_pin, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
    gpio_pin_write(spi_cs.gpio_dev, spi_cs.gpio_pin, 1);

}

void SpiDeInit( void )
{
    SpiInit();
}

uint16_t SpiInOut( uint16_t outData )
{
    #define BUF_SIZE 1
    u8_t buffer_tx[BUF_SIZE] = {0};
    u8_t buffer_rx[BUF_SIZE] = {0};
    buffer_tx[0] = outData;
    const struct spi_buf tx_bufs[] = {
        {
            .buf = buffer_tx,
            .len = BUF_SIZE,
        },
    };
    const struct spi_buf rx_bufs[] = {
        {
            .buf = buffer_rx,
            .len = BUF_SIZE,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs)
    };

    int ret;

    ret = spi_transceive(spi_fast, &spi_cfg_fast, &tx, &rx);
    if (ret) {
        printk("SPI transceive error\r\n");
    }
    /*if (memcmp(buffer_tx, buffer_rx, BUF_SIZE))
    {
        printk("tx rx different\r\n");
    }*/
    return buffer_rx[0];
}

void SpiCsSet( void )
{
    k_busy_wait(50); // 5usec
    gpio_pin_write(spi_cs.gpio_dev, spi_cs.gpio_pin, 1);
}

void SpiCsReSet( void )
{
    gpio_pin_write(spi_cs.gpio_dev, spi_cs.gpio_pin, 0);
    k_busy_wait(50); // 5usec
}