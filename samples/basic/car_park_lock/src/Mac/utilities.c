/*!
 * \file      utilities.h
 *
 * \brief     Helper functions implementation
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
#include <stdlib.h>
#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "utilities.h"

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( uint32_t seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

int8_t Nibble2HexChar( uint8_t a )
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}

void BoardDisableIrq(void)
{

}
void BoardEnableIrq(void)
{
    
}

void TimerInit( TimerEvent_t *obj, void ( *callback )(struct k_timer *) )
{
    k_timer_init(obj,callback,NULL);
}


void TimerStart( TimerEvent_t *obj )
{

}

void TimerStop( TimerEvent_t *obj )
{
    k_timer_stop(obj);
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    k_timer_start(obj,K_MSEC(1),K_MSEC(0));
}

TimerTime_t TimerGetCurrentTime( void )
{
    return k_cycle_get_32();
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    volatile TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    /*if( savedTime == 0 )
    {
        return 0;
    }*/

    elapsedTime = TimerGetCurrentTime();

    if( elapsedTime < savedTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - savedTime ) );
    }
    else
    {
        return( elapsedTime - savedTime );
    }
}