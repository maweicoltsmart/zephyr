/*!
 * \file      spi-board.h
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
 */
#ifndef __SPI_BOARD_H__
#define __SPI_BOARD_H__

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
//#include "spi.h"
void SpiInit( void );
void SpiDeInit( void );
uint16_t SpiInOut( uint16_t outData );
void SpiCsSet( void );
void SpiCsReSet( void );
// An Spi.c file has to be implmented under system directory.

#endif // __SPI_BOARD_H__
