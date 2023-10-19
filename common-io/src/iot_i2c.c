/*
 * Copyright 2023 AWS
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h> // boolean
#include <stddef.h>  // size_t
#include <stdint.h>  // fixed-width types

#include "hal/include/hal_fc_event.h"
#include "hal/include/hal_udma_ctrl_reg_defs.h"
#include "hal/include/hal_udma_i2cm_reg_defs.h"
#include "iot_i2c.h"

#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "target/core-v-mcu/include/core-v-mcu-properties.h"

static volatile uint8_t i2c_bit_mask = 0;

typedef enum
{
    kI2cmCmdStart = 0x00,
    kI2cmCmdWaitEvt = 0x10,
    kI2cmCmdStop = 0x20,
    kI2cmCmdRdAck = 0x40,
    kI2cmCmdRdNack = 0x60,
    kI2cmCmdWr = 0x80,
    kI2cmCmdWait = 0xA0,
    kI2cmCmdRpt = 0xC0,
    kI2cmCmdCfg = 0xE0,
} i2cm_cmd_t;

#define IOT_I2C_ID( i2cm_ptr ) \
    ( ( ( intptr_t ) i2cm_ptr - UDMA_CH_ADDR_I2CM ) / UDMA_CH_SIZE )

typedef struct UDMAStatus
{
    uint16_t bytesCompleted;
    uint16_t bytesRequested;
} UDMAStatus_t;

typedef enum
{
    READ,
    WRITE
} OperationType;
struct IotI2CDescriptor
{
    bool initilized;
    bool is_config_set;
    bool onProgress;
    bool is_slave_addr_set;
    bool is_send_no_stop_flag_set;
    void * userParam;
    void * pvBuffer;
    UdmaI2cm_t * udma;
    UDMAStatus_t tx;
    UDMAStatus_t rx;
    size_t xBytes;
    uint8_t slave_addr;
    IotI2CConfig_t iot_i2c_config;
    IotI2CCallback_t callback;
    volatile OperationType operation;
};

static struct IotI2CDescriptor i2cContexts[ N_I2CM ] = { 0 };

static inline bool i2c_tx_in_progress( IotI2CHandle_t handle )
{
    return handle->udma->tx_size_b.size != 0U;
}

typedef struct IotI2CDescriptor * IotI2CHandle_t;

typedef void ( *IotI2CCallback_t )( IotI2COperationStatus_t xOpStatus,
                                    void * pvUserContext );
static uint8_t aucclkdiv[ 2 ];

static inline void i2cmRX_cb( void * ctx )
{
    // ISR critical section
    IotI2CHandle_t const handle = ( IotI2CHandle_t ) ctx;

    handle->rx.bytesCompleted = handle->rx.bytesRequested -
                                handle->udma->rx_size_b.size;

    if( handle->callback != NULL && handle->operation == READ )
    {
        if( handle->udma->rx_size != 0 )
        {
            handle->callback( eI2CNackFromSlave, handle->userParam );
        }
        else
        {
            handle->callback( eI2CCompleted, handle->userParam );
        }
    }
}

static inline void i2cmTX_cb( void * ctx )
{
    // ISR critical section
    IotI2CHandle_t const handle = ( IotI2CHandle_t ) ctx;

    handle->tx.bytesCompleted = handle->tx.bytesRequested -
                                handle->udma->tx_size_b.size;

    if( handle->callback != NULL && handle->operation == WRITE )
    {
        handle->onProgress = false;
        if( handle->udma->tx_size != 0 )
        {
            handle->callback( eI2CNackFromSlave, handle->userParam );
        }
        else
        {
            handle->callback( eI2CCompleted, handle->userParam );
        }
    }
}

IotI2CHandle_t iot_i2c_open( int32_t lI2CInstance )
{
    volatile UdmaCtrl_t * pudma_ctrl;
    IotI2CHandle_t handle = NULL;
    uint8_t i2cm_id;

    if( lI2CInstance >= 0 && lI2CInstance < N_I2CM )
    {
        if( i2cContexts[ lI2CInstance ].initilized == false )
        {
            handle = &i2cContexts[ lI2CInstance ];
        }
    }

    if( handle != NULL )
    {
        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;
        i2cm_id = ( uint8_t ) lI2CInstance;
        /* Enable reset and enable i2c clock */

        pudma_ctrl->reg_rst |= ( UDMA_CTRL_I2CM0_CLKEN << i2cm_id );
        pudma_ctrl->reg_rst &= ~( ( uint32_t ) UDMA_CTRL_I2CM0_CLKEN
                                  << i2cm_id );
        pudma_ctrl->reg_cg |= ( UDMA_CTRL_I2CM0_CLKEN << i2cm_id );

        handle->udma = ( UdmaI2cm_t * ) ( UDMA_CH_ADDR_I2CM +
                                          i2cm_id * UDMA_CH_SIZE );
        /* Set handlers. */
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2cm_id ),
                                 i2cmRX_cb,
                                 handle );
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2cm_id ),
                                 i2cmTX_cb,
                                 handle );
        /* Enable SOC events propagation to FC. */
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2cm_id ) );
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2cm_id ) );
        handle->initilized = true;
    }

    return handle;
}

void iot_i2c_set_callback( IotI2CHandle_t const pxI2CPeripheral,
                           IotI2CCallback_t xCallback,
                           void * pvUserContext )
{
    if( pxI2CPeripheral != NULL && xCallback != NULL )
    {
        pxI2CPeripheral->callback = xCallback;
        pxI2CPeripheral->userParam = pvUserContext;
    }
}

static uint8_t auccmd_read[ 64 ];
int32_t iot_i2c_read_sync( IotI2CHandle_t const pxI2CPeripheral,
                           uint8_t * const pucBuffer,
                           size_t xBytes )
{
    int32_t returnValue = IOT_I2C_SUCCESS;

    if( pxI2CPeripheral == NULL )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        returnValue = iot_i2c_read_async( pxI2CPeripheral, pucBuffer, xBytes );

        if( returnValue == IOT_I2C_SUCCESS )
        {
            int i2c_id = IOT_I2C_ID( pxI2CPeripheral->udma );
            // mask out event forwarding
            pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );
            pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );

            // Wait for both transfer to finish
            while( pxI2CPeripheral->udma->rx_size != 0 )
            {
            }

            while( pxI2CPeripheral->udma->tx_size != 0 )
            {
            }

            pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );
            pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        }

        pxI2CPeripheral->onProgress = false;
    }
    return returnValue;
}

static uint8_t auccmd_write[ 64 ];
int32_t iot_i2c_write_sync( IotI2CHandle_t const pxI2CPeripheral,
                            uint8_t * const pucBuffer,
                            size_t xBytes )
{
    int32_t returnValue = IOT_I2C_SUCCESS;

    if( pxI2CPeripheral == NULL )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        int i2c_id = IOT_I2C_ID( pxI2CPeripheral->udma );
        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        returnValue = iot_i2c_write_async( pxI2CPeripheral, pucBuffer, xBytes );

        if( returnValue == IOT_I2C_SUCCESS )
        {
            while( pxI2CPeripheral->udma->tx_size != 0 )
                ;
        }
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        pxI2CPeripheral->onProgress = false;
    }

    return returnValue;
}

int32_t iot_i2c_read_async( IotI2CHandle_t const pxI2CPeripheral,
                            uint8_t * const pucBuffer,
                            size_t xBytes )
{
    int32_t returnValue = IOT_I2C_SUCCESS;
    uint8_t * pcmd = auccmd_read;

    if( pxI2CPeripheral == NULL || pucBuffer == NULL || xBytes == 0 ||
        pxI2CPeripheral->initilized == false || xBytes > 255 )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        IotI2CHandle_t iot_i2c_handler = pxI2CPeripheral;

        uint8_t * pdata = ( uint8_t * ) pucBuffer;

        if( !iot_i2c_handler->is_slave_addr_set )
        {
            returnValue = IOT_I2C_SLAVE_ADDRESS_NOT_SET;
        }
        else if( iot_i2c_handler->onProgress )
        {
            returnValue = IOT_I2C_BUSY;
        }
        else
        {
            iot_i2c_handler->onProgress = true;
            iot_i2c_handler->udma->tx_cfg_b.en = 0; // Stop the transfer if any
            iot_i2c_handler->rx.bytesRequested = ( uint16_t ) xBytes;
            iot_i2c_handler->operation = READ;

            *pcmd++ = kI2cmCmdCfg;
            *pcmd++ = aucclkdiv[ 1 ];
            *pcmd++ = aucclkdiv[ 0 ];
            *pcmd++ = kI2cmCmdStart; // Put Start transaction on I2C bus
            *pcmd++ = kI2cmCmdWr;    // Command to repeat: I2C CMD_WR
            *pcmd++ = ( uint8_t ) ( iot_i2c_handler->slave_addr << 1 ) |
                      0x1; // Clear R/WRbar bit from i2c device's address to
                           // indicate write
            if( xBytes > 1 )
            { // Do len-1 reads with ACK, and follow by 1 read with NACK
                *pcmd++ = kI2cmCmdRpt; // Tell controller to repeat the
                                       // following command
                *pcmd++ = ( uint8_t ) ( xBytes - 1 ); // len-1 times
                *pcmd++ = kI2cmCmdRdAck; // command to repeat is read with ack
            }
            *pcmd++ = kI2cmCmdRdNack; // Read last byte with NACK to indicate
                                      // the end of the read
            *pcmd++ = kI2cmCmdStop;

            iot_i2c_handler->udma->rx_saddr = ( uintptr_t ) pdata;
            iot_i2c_handler->udma->rx_size = xBytes;
            iot_i2c_handler->udma->rx_cfg_b.en = 1;

            iot_i2c_handler->udma->tx_saddr = ( uintptr_t ) auccmd_read;
            iot_i2c_handler->udma->tx_size = ( uint32_t ) ( pcmd -
                                                            auccmd_read );
            iot_i2c_handler->udma->tx_cfg_b.en = 1; // initiate the transfer
        }
    }

    return returnValue;
}

int32_t iot_i2c_write_async( IotI2CHandle_t const pxI2CPeripheral,
                             uint8_t * const pucBuffer,
                             size_t xBytes )
{
    int32_t returnValue = IOT_I2C_SUCCESS;
    uint8_t * pcmd = auccmd_write;

    if( pxI2CPeripheral == NULL || pucBuffer == NULL || xBytes == 0 ||
        pxI2CPeripheral->initilized == false || xBytes > 255 )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        IotI2CHandle_t iot_i2c_handler = pxI2CPeripheral;
        uint8_t * pdata = ( uint8_t * ) pucBuffer;

        if( !iot_i2c_handler->is_slave_addr_set )
        {
            returnValue = IOT_I2C_SLAVE_ADDRESS_NOT_SET;
        }
        else if( iot_i2c_handler->onProgress )
        {
            returnValue = IOT_I2C_BUSY;
        }
        else
        {
            iot_i2c_handler->onProgress = true;
            iot_i2c_handler->udma->tx_cfg_b.en = 0;
            iot_i2c_handler->tx.bytesRequested = ( uint16_t ) xBytes;
            iot_i2c_handler->operation = WRITE;

            *pcmd++ = kI2cmCmdCfg;
            *pcmd++ = aucclkdiv[ 1 ];
            *pcmd++ = aucclkdiv[ 0 ];
            *pcmd++ = kI2cmCmdStart; // Put Start transaction on I2C bus
            *pcmd++ = kI2cmCmdWr;    // Command to repeat: I2C CMD_WR
            *pcmd++ = iot_i2c_handler->slave_addr
                      << 1; // Left shift for R/WRbar bit from i2c device's
                            // address to indicate write
            *pcmd++ = kI2cmCmdRpt;
            *pcmd++ = ( uint8_t ) xBytes;
            *pcmd++ = kI2cmCmdWr;
            for( int i = 0; i != ( int ) xBytes; i++ )
            {
                *pcmd++ = *pdata++;
            }

            if( !iot_i2c_handler->is_send_no_stop_flag_set )
            {
                *pcmd++ = kI2cmCmdStop;
                *pcmd++ = 0x0;
            }

            iot_i2c_handler->udma->tx_saddr = ( uintptr_t ) auccmd_write;
            iot_i2c_handler->udma->tx_size = ( uint32_t ) ( pcmd -
                                                            auccmd_write );
            iot_i2c_handler->udma->tx_cfg_b.en = 1;
        }
    }

    return returnValue;
}

int32_t iot_i2c_ioctl( IotI2CHandle_t const pxI2CPeripheral,
                       IotI2CIoctlRequest_t xI2CRequest,
                       void * const pvBuffer )
{
    int32_t returnValue = IOT_I2C_SUCCESS;

    if( xI2CRequest != eI2CSendNoStopFlag &&
        ( pxI2CPeripheral == NULL || pvBuffer == NULL ) )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        IotI2CHandle_t i2c_ctx = pxI2CPeripheral;

        switch( xI2CRequest )
        {
            case eI2CSetMasterConfig:
                // Mutex for busy state
                if( i2c_ctx->onProgress == true ||
                    i2c_ctx->is_config_set == true )
                {
                    returnValue = IOT_I2C_BUSY;
                }
                else
                {
                    i2c_ctx->onProgress = true;
                    uint32_t clk_divisor;
                    // Mutex end

                    IotI2CConfig_t * iot_i2c_handler = ( IotI2CConfig_t * )
                        pvBuffer;

                    i2c_ctx->iot_i2c_config = *iot_i2c_handler;

                    /* configure clock*/
                    clk_divisor = 5000000 / ( iot_i2c_handler->ulBusFreq );
                    aucclkdiv[ 0 ] = ( clk_divisor >> 0 ) & 0xFF;
                    aucclkdiv[ 1 ] = ( clk_divisor >> 8 ) & 0xFF;

                    i2c_ctx->is_config_set = true;
                    i2c_ctx->onProgress = false;
                }

                break;

            case eI2CGetMasterConfig:
                IotI2CConfig_t * iot_i2c_config = ( IotI2CConfig_t * ) pvBuffer;
                iot_i2c_config->ulBusFreq = i2c_ctx->iot_i2c_config.ulBusFreq;
                iot_i2c_config->ulMasterTimeout = i2c_ctx->iot_i2c_config
                                                      .ulMasterTimeout;
                break;

            case eI2CSetSlaveAddr:
                // Mutex for busy state
                if( i2c_ctx->onProgress == true )
                {
                    returnValue = IOT_I2C_BUSY;
                }
                else if( i2c_ctx->is_slave_addr_set == true )
                {
                    returnValue = IOT_I2C_FUNCTION_NOT_SUPPORTED;
                }
                else
                {
                    i2c_ctx->onProgress = true;
                    // Mutex end

                    i2c_ctx->slave_addr = ( *( uint8_t * ) pvBuffer );
                    i2c_ctx->is_slave_addr_set = true;

                    i2c_ctx->onProgress = false;
                }
                break;

            case eI2CGetBusState:
                IotI2CBusStatus_t * bus_state = ( IotI2CBusStatus_t * ) pvBuffer;

                if( i2c_ctx->onProgress == true ||
                    i2c_ctx->udma->tx_size_b.size != 0U )
                {
                    *bus_state = eI2cBusBusy;
                }
                else
                {
                    *bus_state = eI2CBusIdle;
                }
                break;

            case eI2CGetTxNoOfbytes:
                uint16_t * tx_bytes = ( uint16_t * ) pvBuffer;
                *tx_bytes = i2c_ctx->tx.bytesCompleted;
                break;

            case eI2CGetRxNoOfbytes:
                uint16_t * rx_bytes = ( uint16_t * ) pvBuffer;
                *rx_bytes = i2c_ctx->rx.bytesCompleted;
                break;

            case eI2CSendNoStopFlag:
                i2c_ctx->is_send_no_stop_flag_set = true;
                break;

            default:
                returnValue = IOT_I2C_INVALID_VALUE;
                break;
        }
    }

    return ( returnValue );
}

int32_t iot_i2c_close( IotI2CHandle_t const pxI2CPeripheral )
{
    int32_t returnValue = IOT_I2C_SUCCESS;
    UdmaCtrl_t * pudma_ctrl;
    int i2c_id;

    if( pxI2CPeripheral == NULL || pxI2CPeripheral->initilized == false )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;
        i2c_id = IOT_I2C_ID( pxI2CPeripheral->udma );

        ( void ) iot_i2c_cancel( pxI2CPeripheral );

        // disable I2C events
        pi_fc_event_handler_clear(
            ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );
        pi_fc_event_handler_clear(
            ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );

        // wait for tx to finish current frame
        while( pxI2CPeripheral->udma->tx_size_b.size != 0U )
        {
        }

        // disable I2C clock
        pudma_ctrl->reg_cg &= ~( uint32_t ) ( UDMA_CTRL_I2CM0_CLKEN << i2c_id );

        pxI2CPeripheral->callback = NULL;
        pxI2CPeripheral->userParam = NULL;
        pxI2CPeripheral->udma = NULL;

        pxI2CPeripheral->tx.bytesCompleted = 0U;
        pxI2CPeripheral->tx.bytesRequested = 0U;
    }

    return returnValue;
}

int32_t iot_i2c_cancel( IotI2CHandle_t const pxI2CPeripheral )
{
    int32_t returnValue = IOT_I2C_SUCCESS;

    if( pxI2CPeripheral == NULL || pxI2CPeripheral->initilized == false )
    {
        returnValue = IOT_I2C_INVALID_VALUE;
    }
    else
    {
        if( !pxI2CPeripheral->udma->tx_cfg_b.pending &&
            !pxI2CPeripheral->udma->rx_cfg_b.pending )
        {
            returnValue = IOT_I2C_NOTHING_TO_CANCEL;
        }

        int i2c_id = IOT_I2C_ID( pxI2CPeripheral->udma );
        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );

        pxI2CPeripheral->udma->rx_cfg_b.clr = 1;
        pxI2CPeripheral->udma->tx_cfg_b.clr = 1;

        // Busy wait until transaction is complete for rx
        while( pxI2CPeripheral->udma->rx_size_b.size != 0 )
        {
        }

        // Busy wait until transaction is complete for tx
        while( pxI2CPeripheral->udma->tx_size_b.size != 0 )
        {
        }

        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_TX( i2c_id ) );
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_I2C_RX( i2c_id ) );
        pxI2CPeripheral->initilized = false;
    }

    return returnValue;
}
