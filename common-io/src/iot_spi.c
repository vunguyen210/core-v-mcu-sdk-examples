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
#include "hal/include/hal_udma_qspi_reg_defs.h"
#include "iot_spi.h"

#include "target/core-v-mcu/include/core-v-mcu-config.h"
#include "target/core-v-mcu/include/core-v-mcu-properties.h"

#define WORD_PER_TRANSFER 4 // 1, 2 or 4 is only supported
#define WORD_SIZE         8

typedef enum
{
    kSPIm_Cfg = ( 0x0 << 28 ),
    kSPIm_SOT = ( 0x1 << 28 ),
    kSPIm_SendCmd = ( 0x2 << 28 ),
    kSPIm_Dummy = ( 0x4 << 28 ),
    kSPIm_Wait = ( 0x5 << 28 ),
    kSPIm_TxData = ( 0x6 << 28 ),
    kSPIm_RxData = ( 0x7 << 28 ),
    kSPIm_Repeat = ( 0x8 << 28 ),
    kSPIm_EOT = ( 0x9 << 28 ),
    kSPIm_RepeatEnd = ( 0xa << 28 ),
    kSPIm_RxCheck = ( 0xb << 28 ),
    kSPIm_FDX = ( 0xc << 28 ),
    kSPIm_UCA = ( 0xd << 28 ),
    kSPIm_UCS = ( 0xe << 28 )

} spim_cmd_t;

#define IOT_SPIM_ID( spim_ptr ) \
    ( ( ( intptr_t ) spim_ptr - UDMA_CH_ADDR_QSPIM ) / UDMA_CH_SIZE )
#define clockReference 5000000U

typedef struct BusStatusDescriptor
{
    uint16_t bytesCompleted;
    uint16_t bytesRequested;
    volatile uint8_t onProgress;
} BusStatus_t;

uint8_t chipSelect;

struct IotSPIDescriptor
{
    bool initilized;
    bool is_config_set;
    volatile bool onProgress;
    void * userParam;
    BusStatus_t tx;
    BusStatus_t rx;
    BusStatus_t cmd;
    UdmaQspi_t * udma;
    IotSPIMasterConfig_t iot_spi_config;
    IotSPICallback_t callback;
};

uint8_t clk_divisor;

static struct IotSPIDescriptor spiContexts[ N_QSPIM ] = { 0 };

/**
 * @brief IotSPIHandle_t is the handle type returned by calling iot_spi_open().
 *        This is initialized in open and returned to caller. The caller must
 * pass this pointer to the rest of APIs.
 */
typedef struct IotSPIDescriptor * IotSPIHandle_t;

/**
 * @brief The callback function for completion of SPI operation.
 */
typedef void ( *IotSPICallback_t )( IotSPITransactionStatus_t xStatus,
                                    void * pvSPIparam );

static inline void spiEOT_cb( void * ctx )
{
    // ISR critical section
    IotSPIHandle_t const handle = ( IotSPIHandle_t ) ctx;

    handle->rx.bytesCompleted = handle->rx.bytesRequested -
                                handle->udma->rx_size_b.size;
    handle->tx.bytesCompleted = handle->tx.bytesRequested -
                                handle->udma->tx_size_b.size;

    if( handle->callback != NULL )
    {
        handle->onProgress = false;
        if( handle->udma->tx_size != 0 )
        {
            handle->callback( eSPITransferError, handle->userParam );
        }
        else
        {
            handle->callback( eSPISuccess, handle->userParam );
        }
    }
}

IotSPIHandle_t iot_spi_open( int32_t lSPIInstance )
{
    volatile UdmaCtrl_t * pudma_ctrl;
    IotSPIHandle_t handle = NULL;
    int spim_id;

    if( lSPIInstance >= 0 && lSPIInstance < N_QSPIM )
    {
        if( spiContexts[ lSPIInstance ].initilized == false )
        {
            handle = &spiContexts[ lSPIInstance ];
        }
    }

    if( handle != NULL )
    {
        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;
        spim_id = ( uint8_t ) lSPIInstance;
        /* Enable reset and enable i2c clock */
        pudma_ctrl->reg_rst |= ( UDMA_CTRL_QSPIM0_CLKEN << spim_id );
        pudma_ctrl->reg_rst &= ~( ( uint32_t ) UDMA_CTRL_QSPIM0_CLKEN
                                  << spim_id );
        pudma_ctrl->reg_cg |= ( UDMA_CTRL_QSPIM0_CLKEN << spim_id );

        handle->udma = ( UdmaQspi_t * ) ( UDMA_CH_ADDR_QSPIM +
                                          spim_id * UDMA_CH_SIZE );
        /* Set handlers. */
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT(
                                     spim_id ),
                                 spiEOT_cb,
                                 handle );
        /* Enable SOC events propagation to FC. */
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spim_id ) );
        handle->initilized = true;
    }

    return handle;
}

void iot_spi_set_callback( IotSPIHandle_t const pxSPIPeripheral,
                           IotSPICallback_t xCallback,
                           void * pvUserContext )
{
    if( pxSPIPeripheral != NULL && xCallback != NULL )
    {
        pxSPIPeripheral->callback = xCallback;
        pxSPIPeripheral->userParam = pvUserContext;
    }
}

int32_t iot_spi_ioctl( IotSPIHandle_t const pxSPIPeripheral,
                       IotSPIIoctlRequest_t xSPIRequest,
                       void * const pvBuffer )
{
    int32_t returnValue = IOT_SPI_SUCCESS;

    if( pxSPIPeripheral == NULL || pvBuffer == NULL ||
        pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        IotSPIHandle_t spi_ctx = pxSPIPeripheral;

        switch( xSPIRequest )
        {
            case eSPISetMasterConfig:
                // Mutex for busy state
                if( spi_ctx->onProgress == true ||
                    spi_ctx->is_config_set == true )
                {
                    returnValue = IOT_SPI_BUS_BUSY;
                }
                else
                {
                    spi_ctx->onProgress = true;

                    IotSPIMasterConfig_t *
                        iot_spi_handler = ( IotSPIMasterConfig_t * ) pvBuffer;
                    spi_ctx->iot_spi_config = *iot_spi_handler;

                    /* configure clock*/
                    clk_divisor = ( uint8_t ) ( clockReference /
                                                ( iot_spi_handler->ulFreq ) );

                    spi_ctx->is_config_set = true;
                    spi_ctx->onProgress = false;
                }

                break;

            case eSPIGetMasterConfig:
                IotSPIMasterConfig_t *
                    iot_spi_config = ( IotSPIMasterConfig_t * ) pvBuffer;
                iot_spi_config->ulFreq = spi_ctx->iot_spi_config.ulFreq;
                iot_spi_config->eMode = spi_ctx->iot_spi_config.eMode;
                iot_spi_config->eSetBitOrder = spi_ctx->iot_spi_config
                                                   .eSetBitOrder;
                iot_spi_config->ucDummyValue = spi_ctx->iot_spi_config
                                                   .ucDummyValue;
                break;

            case eSPIGetTxNoOfbytes:
                uint16_t * tx_bytes = ( uint16_t * ) pvBuffer;
                *tx_bytes = spi_ctx->tx.bytesCompleted;
                break;

            case eSPIGetRxNoOfbytes:
                uint16_t * rx_bytes = ( uint16_t * ) pvBuffer;
                *rx_bytes = spi_ctx->rx.bytesCompleted;
                break;

            default:
                returnValue = IOT_SPI_INVALID_VALUE;
                break;
        }
    }

    return ( returnValue );
}

static uint32_t auccmd_rx_read[ 16 ] = { 0 };

int32_t iot_spi_read_sync( IotSPIHandle_t const pxSPIPeripheral,
                           uint8_t * const pvBuffer,
                           size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;

    if( pxSPIPeripheral == NULL || pvBuffer == NULL || xBytes == 0 ||
        pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        int spi_id = IOT_SPIM_ID( pxSPIPeripheral->udma );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        returnValue = iot_spi_read_async( pxSPIPeripheral, pvBuffer, xBytes );

        if( returnValue == IOT_SPI_SUCCESS )
        {
            while( pxSPIPeripheral->udma->rx_size != 0 )
                ;
            while( pxSPIPeripheral->udma->cmd_size != 0 )
                ;
        }

        // mask out event forwarding
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        pxSPIPeripheral->onProgress = false;
    }
    return returnValue;
}

int32_t iot_spi_read_async( IotSPIHandle_t const pxSPIPeripheral,
                            uint8_t * const pvBuffer,
                            size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    uint32_t * pcmd = auccmd_rx_read;

    if( pxSPIPeripheral == NULL || pvBuffer == NULL || xBytes == 0 ||
        pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        IotSPIHandle_t iot_spi_handler = pxSPIPeripheral;

        uint8_t * pdata = ( uint8_t * ) pvBuffer;

        if( iot_spi_handler->onProgress )
        {
            returnValue = IOT_SPI_BUS_BUSY;
        }
        else
        {
            iot_spi_handler->onProgress = true;

            // Stop the transfer if any
            iot_spi_handler->udma->rx_cfg_b.clr = 1;
            iot_spi_handler->udma->tx_cfg_b.clr = 1;
            iot_spi_handler->udma->cmd_cfg_b.clr = 1;
            iot_spi_handler->udma->rx_cfg_b.en = 0;
            iot_spi_handler->udma->tx_cfg_b.en = 0;
            iot_spi_handler->udma->cmd_cfg_b.en = 0;

            iot_spi_handler->rx.bytesRequested = ( uint16_t ) xBytes;

            *pcmd++ = kSPIm_Cfg | clk_divisor;
            *pcmd++ = ( uint32_t ) kSPIm_SOT | chipSelect; // cs 0
            *pcmd++ = kSPIm_RxData | ( ( WORD_PER_TRANSFER << 20 ) |
                                       ( ( WORD_SIZE - 1 ) << 16 ) |
                                       ( xBytes - 1 ) ); // user size recieved
            *pcmd++ = ( uint32_t ) kSPIm_EOT | 1;        // generate event

            iot_spi_handler->udma->rx_saddr = ( uintptr_t ) pdata;
            iot_spi_handler->udma->rx_size = xBytes - 1;
            iot_spi_handler->udma->rx_cfg_b.datasize = 2;
            iot_spi_handler->udma->rx_cfg_b.en = 1;

            iot_spi_handler->udma->cmd_saddr = ( uintptr_t ) auccmd_rx_read;
            iot_spi_handler->udma->cmd_size = ( uint32_t ) ( pcmd -
                                                             auccmd_rx_read ) *
                                              sizeof( *pcmd );
            iot_spi_handler->udma->cmd_cfg_b.en = 1; // initiate the transfer
        }
    }

    return returnValue;
}

static uint32_t auccmd_tx_write[ 16 ];

int32_t iot_spi_write_sync( IotSPIHandle_t const pxSPIPeripheral,
                            uint8_t * const pvBuffer,
                            size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;

    if( pxSPIPeripheral == NULL || pvBuffer == NULL || xBytes == 0 ||
        pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        int spi_id = IOT_SPIM_ID( pxSPIPeripheral->udma );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        returnValue = iot_spi_write_async( pxSPIPeripheral, pvBuffer, xBytes );

        if( returnValue == IOT_SPI_SUCCESS )
        {
            while( pxSPIPeripheral->udma->tx_size != 0 )
                ;
            while( pxSPIPeripheral->udma->cmd_size != 0 )
                ;
        }

        // mask out event forwarding
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );
        pxSPIPeripheral->onProgress = false;
    }
    return returnValue;
}

int32_t iot_spi_write_async( IotSPIHandle_t const pxSPIPeripheral,
                             uint8_t * const pvBuffer,
                             size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    uint32_t * pcmd = auccmd_tx_write;

    if( pxSPIPeripheral == NULL || pvBuffer == NULL || xBytes == 0 ||
        pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        IotSPIHandle_t iot_spi_handler = pxSPIPeripheral;

        uint8_t * pdata = ( uint8_t * ) pvBuffer;

        if( iot_spi_handler->onProgress )
        {
            returnValue = IOT_SPI_BUS_BUSY;
        }
        else
        {
            iot_spi_handler->onProgress = true;

            // Stop the transfer if any
            iot_spi_handler->udma->rx_cfg_b.clr = 1;
            iot_spi_handler->udma->tx_cfg_b.clr = 1;
            iot_spi_handler->udma->cmd_cfg_b.clr = 1;
            iot_spi_handler->tx.bytesRequested = ( uint16_t ) xBytes;

            *pcmd++ = kSPIm_Cfg | clk_divisor;
            *pcmd++ = ( uint32_t ) kSPIm_SOT | chipSelect; // cs 0
            *pcmd++ = kSPIm_TxData | ( ( WORD_PER_TRANSFER << 20 ) |
                                       ( ( WORD_SIZE - 1 ) << 16 ) |
                                       ( xBytes - 1 ) ); // user size recieved
            *pcmd++ = ( uint32_t ) kSPIm_EOT | 1;        // generate event

            iot_spi_handler->udma->tx_saddr = ( uintptr_t ) pdata;
            iot_spi_handler->udma->tx_size = xBytes - 1;
            iot_spi_handler->udma->tx_cfg_b.datasize = 2;
            iot_spi_handler->udma->tx_cfg_b.en = 1;

            iot_spi_handler->udma->cmd_saddr = ( uintptr_t ) auccmd_tx_write;
            iot_spi_handler->udma->cmd_size = ( uint32_t ) ( pcmd -
                                                             auccmd_tx_write ) *
                                              sizeof( *pcmd );
            iot_spi_handler->udma->cmd_cfg_b.en = 1; // initiate the transfer
        }
    }

    return returnValue;
}

int32_t iot_spi_transfer_sync( IotSPIHandle_t const pxSPIPeripheral,
                               uint8_t * const pvTxBuffer,
                               uint8_t * const pvRxBuffer,
                               size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;

    if( pxSPIPeripheral == NULL || pvTxBuffer == NULL || pvRxBuffer == NULL ||
        xBytes == 0 || pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        int spi_id = IOT_SPIM_ID( pxSPIPeripheral->udma );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        returnValue = iot_spi_transfer_async( pxSPIPeripheral,
                                              pvTxBuffer,
                                              pvRxBuffer,
                                              xBytes );

        if( returnValue == IOT_SPI_SUCCESS )
        {
            while( pxSPIPeripheral->udma->rx_size != 0 )
                ;
            while( pxSPIPeripheral->udma->tx_size != 0 )
                ;
            while( pxSPIPeripheral->udma->cmd_size != 0 )
                ;
        }

        // reenable event forwarding
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        pxSPIPeripheral->onProgress = false;
    }
    return returnValue;
}

int32_t iot_spi_transfer_async( IotSPIHandle_t const pxSPIPeripheral,
                                uint8_t * const pvTxBuffer,
                                uint8_t * const pvRxBuffer,
                                size_t xBytes )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    uint32_t * pcmd = auccmd_tx_write;

    if( pxSPIPeripheral == NULL || pvTxBuffer == NULL || pvRxBuffer == NULL ||
        xBytes == 0 || pxSPIPeripheral->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        IotSPIHandle_t iot_spi_handler = pxSPIPeripheral;

        uint8_t * pdataWrite = ( uint8_t * ) pvTxBuffer;
        uint8_t * pdataRead = ( uint8_t * ) pvRxBuffer;

        if( iot_spi_handler->onProgress )
        {
            returnValue = IOT_SPI_BUS_BUSY;
        }
        else
        {
            iot_spi_handler->onProgress = true;

            // Stop the transfer if any
            iot_spi_handler->udma->rx_cfg_b.clr = 1;
            iot_spi_handler->udma->tx_cfg_b.clr = 1;
            iot_spi_handler->udma->cmd_cfg_b.clr = 1;

            *pcmd++ = kSPIm_Cfg | clk_divisor;
            *pcmd++ = ( uint32_t ) kSPIm_SOT | chipSelect; // cs 0
            *pcmd++ = kSPIm_TxData | ( ( WORD_PER_TRANSFER << 20 ) |
                                       ( ( WORD_SIZE - 1 ) << 16 ) |
                                       ( xBytes - 1 ) ); // user size recieved
            *pcmd++ = kSPIm_RxData | ( ( WORD_PER_TRANSFER << 20 ) |
                                       ( ( WORD_SIZE - 1 ) << 16 ) |
                                       ( xBytes - 1 ) ); // user size recieved
            *pcmd++ = ( uint32_t ) kSPIm_FDX | ( xBytes - 1 );
            *pcmd++ = ( uint32_t ) kSPIm_EOT | 1; // generate event

            iot_spi_handler->udma->tx_saddr = ( uintptr_t ) pdataWrite;
            iot_spi_handler->udma->tx_size = xBytes - 1;
            iot_spi_handler->udma->tx_cfg_b.datasize = 2;
            iot_spi_handler->udma->tx_cfg_b.en = 1;

            iot_spi_handler->udma->rx_saddr = ( uintptr_t ) pdataRead;
            iot_spi_handler->udma->rx_size = xBytes;
            iot_spi_handler->udma->rx_cfg_b.en = 1;

            iot_spi_handler->udma->cmd_saddr = ( uintptr_t ) auccmd_tx_write;
            iot_spi_handler->udma->cmd_size = ( uint32_t ) ( pcmd -
                                                             auccmd_tx_write ) *
                                              sizeof( *pcmd );
            iot_spi_handler->udma->cmd_cfg_b.en = 1; // initiate the transfer
        }
    }

    return returnValue;
}

int32_t iot_spi_close( IotSPIHandle_t const pxSPIPeripheral )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    UdmaCtrl_t * pudma_ctrl;
    int spi_id;
    IotSPIHandle_t iot_spi_handler = pxSPIPeripheral;

    if( iot_spi_handler == NULL || iot_spi_handler->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;
        spi_id = IOT_SPIM_ID( pxSPIPeripheral->udma );

        ( void ) iot_spi_cancel( iot_spi_handler );

        // disable SPI events
        pi_fc_event_handler_clear(
            ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        // disable SPI clock
        pudma_ctrl->reg_cg &= ~( uint32_t ) ( UDMA_CTRL_QSPIM0_CLKEN
                                              << spi_id );

        iot_spi_handler->callback = NULL;
        iot_spi_handler->userParam = NULL;
        iot_spi_handler->udma = NULL;

        iot_spi_handler->tx.bytesCompleted = 0U;
        iot_spi_handler->tx.bytesRequested = 0U;
        iot_spi_handler->initilized = false;
    }

    return returnValue;
}

int32_t iot_spi_cancel( IotSPIHandle_t const pxSPIPeripheral )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    IotSPIHandle_t iot_spi_handler = pxSPIPeripheral;

    if( iot_spi_handler == NULL || iot_spi_handler->initilized == false )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        if( iot_spi_handler->udma->tx_size_b.size == 0 &&
            iot_spi_handler->udma->rx_size_b.size == 0 )
        {
            returnValue = IOT_SPI_NOTHING_TO_CANCEL;
        }
        int spi_id = IOT_SPIM_ID( pxSPIPeripheral->udma );

        // mask out event forwarding
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );

        iot_spi_handler->udma->rx_cfg_b.clr = 1;
        iot_spi_handler->udma->tx_cfg_b.clr = 1;
        iot_spi_handler->udma->cmd_cfg_b.clr = 1;

        // Busy wait until transaction is complete for rx
        while( iot_spi_handler->udma->rx_size_b.size != 0 )
        {
        }

        // Busy wait until transaction is complete for tx
        while( iot_spi_handler->udma->tx_size_b.size != 0 )
        {
        }

        // Busy wait until transaction is complete for cmd
        while( iot_spi_handler->udma->cmd_size_b.size != 0 )
        {
        }
        // reenable event forwarding
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_SPIM_EOT( spi_id ) );
    }

    return returnValue;
}

int32_t iot_spi_select_slave( int32_t lSPIInstance, int32_t lSPISlave )
{
    int32_t returnValue = IOT_SPI_SUCCESS;
    if( lSPIInstance < 0 && lSPIInstance >= N_QSPIM && lSPISlave >= 3 &&
        lSPISlave < 0 )
    {
        returnValue = IOT_SPI_INVALID_VALUE;
    }
    else
    {
        chipSelect = ( uint8_t ) lSPISlave;
    }
    return returnValue;
}
