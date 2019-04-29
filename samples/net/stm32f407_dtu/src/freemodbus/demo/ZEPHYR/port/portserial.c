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
#include <ring_buffer.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbconfig.h"

/* ----------------------- Defines  -----------------------------------------*/
#define UART_DEVICE_NAME    "UART_6"

#if MB_ASCII_ENABLED == 1
#define BUF_SIZE    513         /* must hold a complete ASCII frame. */
#else
#define BUF_SIZE    256         /* must hold a complete RTU frame. */
#endif

/* ----------------------- Static variables ---------------------------------*/
static BOOL     bRxEnabled;
static BOOL     bTxEnabled;

static ULONG    ulTimeoutMs;
static UCHAR    ucRxBuffer[BUF_SIZE];
static UCHAR    ucTxBuffer[BUF_SIZE];
static int      uiRxBufferPos;
static int      uiTxBufferPos;
struct k_sem rx_sem;
#define MAX_READ_SIZE   256
#define DATA_SIZE   (256)
struct ring_buf rx_rb;
static UCHAR uRxBuffer[DATA_SIZE];
struct ring_buf tx_rb;
static UCHAR uTxBuffer[DATA_SIZE];

/* ----------------------- Function prototypes ------------------------------*/
static BOOL     prvbMBPortSerialWrite( UCHAR * pucBuffer, USHORT usNBytes );
struct device *uart_dev = NULL;
/* ----------------------- Begin implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL bEnableRx, BOOL bEnableTx )
{
    u8_t recvData;
    /* it is not allowed that both receiver and transmitter are enabled. */
    assert( !bEnableRx || !bEnableTx );

    if( bEnableRx )
    {
        //( void )tcflush( iSerialFd, TCIFLUSH );
        while (uart_fifo_read(uart_dev, &recvData, 1) > 0) {
             continue;
        }
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

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    uart_dev = device_get_binding(UART_DEVICE_NAME);
    k_sem_init(&rx_sem, 0, 1);
    ring_buf_init(&rx_rb, DATA_SIZE, uRxBuffer);
    ring_buf_init(&tx_rb, DATA_SIZE, uTxBuffer);

    /* Verify uart_irq_callback_set() */
    uart_irq_callback_set(uart_dev, uart_fifo_callback);

    /* Enable Tx/Rx interrupt before using fifo */
    /* Verify uart_irq_rx_enable() */
    uart_irq_rx_enable(uart_dev);
    
    prvbMBPortSerialWrite("uart3 working\r\n",15);
    prvbMBPortSerialWrite("uart3 working\r\n",15);
    usleep(500);
    prvbMBPortSerialWrite("uart3 working\r\n",15);
    
    printk("serial port init over\r\n");
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

    //usleep(500);
    k_sem_take(&rx_sem,50);
    *usNBytesRead = ring_buf_get(&rx_rb, pucBuffer, usNBytes);
    /*if(*usNBytesRead > 0)
    {
        printk("uart3 recv %d bytes\r\n",*usNBytesRead);
    }*/
    return bResult;
}

BOOL
prvbMBPortSerialWrite( UCHAR * pucBuffer, USHORT usNBytes )
{
    int ret;
    
    ret = ring_buf_put(&tx_rb, pucBuffer, usNBytes);
    if(ret != usNBytes)
    {
        printk("uart fifo write error\r\n");
    }
    uart_irq_tx_enable(uart_dev);
    /*for(USHORT i = 0;i < usNBytes;i ++)
    {
        uart_poll_out(uart_dev,pucBuffer[i]);
        //printk("%02X ",pucBuffer[i]);
    }*/
    //printk("\r\n");
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
        if( prvbMBPortSerialRead( &ucRxBuffer[0], BUF_SIZE, &usBytesRead ) )
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
        if( !prvbMBPortSerialWrite( &ucTxBuffer[0], uiTxBufferPos ) )
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
    /*int ret;

    ret = ring_buf_put(&tx_rb, &ucByte, 1);
    if(ret != 1)
    {
        printk("uart fifo write error\r\n");
    }*/
    assert( uiTxBufferPos < BUF_SIZE );
    ucTxBuffer[uiTxBufferPos] = ucByte;
    uiTxBufferPos++;
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    assert( uiRxBufferPos < BUF_SIZE );
    *pucByte = ucRxBuffer[uiRxBufferPos];
    uiRxBufferPos++;
    return TRUE;
}
