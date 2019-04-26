/*
 * FreeModbus Libary: Linux Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

/* ----------------------- Standard includes --------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <zephyr.h>
#include <kernel.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <uart.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

/* ----------------------- Defines  -----------------------------------------*/
#define UART_DEVICE_NAME    "UART_1"

#if MB_ASCII_ENABLED == 1
#define BUF_SIZE    513         /* must hold a complete ASCII frame. */
#else
#define BUF_SIZE    256         /* must hold a complete RTU frame. */
#endif

/* ----------------------- Static variables ---------------------------------*/
static BOOL     bRxEnabled;
static BOOL     bTxEnabled;

static ULONG    ulTimeoutMs;
static UCHAR    ucBuffer[BUF_SIZE];
static int      uiRxBufferPos;
static int      uiTxBufferPos;

/* ----------------------- Function prototypes ------------------------------*/
static BOOL     prvbMBPortSerialWrite( UCHAR * pucBuffer, USHORT usNBytes );
struct device *uart_dev = NULL;
/* ----------------------- Begin implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL bEnableRx, BOOL bEnableTx )
{
    /* it is not allowed that both receiver and transmitter are enabled. */
    assert( !bEnableRx || !bEnableTx );

    if( bEnableRx )
    {
        //( void )tcflush( iSerialFd, TCIFLUSH );
        uiRxBufferPos = 0;
        bRxEnabled = TRUE;
    }
    else
    {
        bRxEnabled = FALSE;
    }
    if( bEnableTx )
    {
        bTxEnabled = TRUE;
        uiTxBufferPos = 0;
    }
    else
    {
        bTxEnabled = FALSE;
    }
}

static void uart_fifo_callback(struct device *dev)
{
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

        uart_irq_tx_disable(dev);
    }

    /* Verify uart_irq_rx_ready() */
    if (uart_irq_rx_ready(dev)) {
        //TC_PRINT("%c", recvData);
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    uart_dev = device_get_binding(UART_DEVICE_NAME);

    /* Verify uart_irq_callback_set() */
    uart_irq_callback_set(uart_dev, uart_fifo_callback);

    /* Enable Tx/Rx interrupt before using fifo */
    /* Verify uart_irq_rx_enable() */
    uart_irq_rx_enable(uart_dev);
    return TRUE;
}

BOOL
xMBPortSerialSetTimeout( ULONG ulNewTimeoutMs )
{
    if( ulNewTimeoutMs > 0 )
    {
        ulTimeoutMs = ulNewTimeoutMs;
    }
    else
    {
        ulTimeoutMs = 1;
    }
    return TRUE;
}

void
vMBPortClose( void )
{
    if( uart_dev != NULL )
    {
        uart_irq_rx_disable(uart_dev);
    }
}

BOOL
prvbMBPortSerialRead( UCHAR * pucBuffer, USHORT usNBytes, USHORT * usNBytesRead )
{
    BOOL            bResult = TRUE;

    *usNBytesRead = uart_fifo_read(uart_dev, pucBuffer, usNBytes);
    if(*usNBytesRead < 1)
    {
        bResult = FALSE;
    }
    return bResult;
}

BOOL
prvbMBPortSerialWrite( UCHAR * pucBuffer, USHORT usNBytes )
{
    for(USHORT i = 0;i < usNBytes;i ++)
    {
        uart_poll_out(uart_dev, pucBuffer[i]);
    }
    return true;
}

BOOL
xMBPortSerialPoll(  )
{
    BOOL            bStatus = TRUE;
    USHORT          usBytesRead;
    int             i;

    while( bRxEnabled )
    {
        if( prvbMBPortSerialRead( &ucBuffer[0], BUF_SIZE, &usBytesRead ) )
        {
            if( usBytesRead == 0 )
            {
                /* timeout with no bytes. */
                break;
            }
            else if( usBytesRead > 0 )
            {
                for( i = 0; i < usBytesRead; i++ )
                {
                    /* Call the modbus stack and let him fill the buffers. */
                    ( void )pxMBFrameCBByteReceived(  );
                }
                uiRxBufferPos = 0;
            }
        }
        else
        {
            vMBPortLog( MB_LOG_ERROR, "SER-POLL", "read failed on serial device: %s\n",
                        strerror( errno ) );
            bStatus = FALSE;
        }
    }
    if( bTxEnabled )
    {
        while( bTxEnabled )
        {
            ( void )pxMBFrameCBTransmitterEmpty(  );
            /* Call the modbus stack to let him fill the buffer. */
        }
        if( !prvbMBPortSerialWrite( &ucBuffer[0], uiTxBufferPos ) )
        {
            vMBPortLog( MB_LOG_ERROR, "SER-POLL", "write failed on serial device: %s\n",
                        strerror( errno ) );
            bStatus = FALSE;
        }
    }

    return bStatus;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    assert( uiTxBufferPos < BUF_SIZE );
    ucBuffer[uiTxBufferPos] = ucByte;
    uiTxBufferPos++;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    assert( uiRxBufferPos < BUF_SIZE );
    *pucByte = ucBuffer[uiRxBufferPos];
    uiRxBufferPos++;
    return TRUE;
}
