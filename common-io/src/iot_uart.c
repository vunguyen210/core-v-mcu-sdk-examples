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
#include "iot_uart.h"

#include "hal/include/hal_fc_event.h"
#include "hal/include/hal_udma_ctrl_reg_defs.h"
#include "hal/include/hal_udma_uart_reg_defs.h"

#include "target/core-v-mcu/include/core-v-mcu-config.h"

#if !defined( __CDT_PARSER__ ) && \
    ( __STDC_VERSION__ >= 201112L && !defined( __STDC_NO_ATOMICS__ ) )
    #include <stdatomic.h>
typedef _Atomic uint16_t BufferCursor;
    #define INITIALIZE_CURSOR( cursor, value ) atomic_init( ( cursor ), value )
    #define LOAD_CURSOR_CONSUME( cursor ) \
        atomic_load_explicit( ( cursor ), memory_order_consume )

    #define LOAD_CURSOR_ACQUIRE( cursor ) \
        atomic_load_explicit( ( cursor ), memory_order_acquire )

    #define STORE_CURSOR_RELEASE( cursor, value ) \
        atomic_store_explicit( ( cursor ), ( value ), memory_order_release )
#else
typedef volatile uint16_t BufferCursor;
    #define INITIALIZE_CURSOR( cursor, value ) \
        do                                     \
        {                                      \
            *( cursor ) = ( value );           \
        } while( 0 )

    #define LOAD_CURSOR_CONSUME( cursor ) ( *( cursor ) )
    #define LOAD_CURSOR_ACQUIRE( cursor ) ( *( cursor ) )
    #define STORE_CURSOR_RELEASE( cursor, value ) \
        do                                        \
        {                                         \
            *( cursor ) = ( value );              \
        } while( 0 )
#endif

#define IOT_UART_PTR( uart_id ) \
    ( ( UdmaUart_t * ) ( UDMA_CH_ADDR_UART + ( uart_id ) *UDMA_CH_SIZE ) )
#define IOT_UART_ID( uart_ptr ) \
    ( ( ( intptr_t ) uart_ptr - UDMA_CH_ADDR_UART ) / UDMA_CH_SIZE )

#define IOT_RX_CIRCULAR_SIZE_LOG2 7
#define IOT_RX_CIRCULAR_SIZE      ( 1 << IOT_RX_CIRCULAR_SIZE_LOG2 )
#define IOT_RX_CIRCULAR_MASK      ( IOT_RX_CIRCULAR_SIZE - 1 )

// TODO: use FLL for clock lookup (actual hardware / newest bit file)
#define IOT_REF_CLK               ( 5000000 )

typedef struct UDMAStatus
{
    uint16_t bytesRequested;
} UDMAStatus_t;

struct IotUARTDescriptor
{
    UdmaUart_t * udma;

    IotUARTCallback_t callback;
    void * userParam;

    uint32_t baudrate;
    IotUARTParity_t parity;

    UDMAStatus_t tx;

    struct UartRxStatus
    {
        // circular buffer during idle
        uint16_t writer;
        BufferCursor reader;
        volatile uint8_t buffer[ IOT_RX_CIRCULAR_SIZE ];

        // async read request info
        uint16_t bytesRequested;
        bool sync;
    } rx;
};

static struct IotUARTDescriptor uartContexts[ N_UART ] = { 0 };

typedef struct IotUARTDescriptor * IotUARTHandle_t;

static IotUARTConfig_t retrieve_config( IotUARTHandle_t handle );

static int32_t set_config( IotUARTHandle_t handle,
                           const IotUARTConfig_t * config );

static void set_config_raw( UdmaUart_t * udma, const IotUARTConfig_t * config );

static int32_t cancel_raw( IotUARTHandle_t handle );

// RX may be replaced with DMA, which should instead check RX registers
static inline bool iot_uart_read_in_progress( IotUARTHandle_t handle )
{
    return handle->udma->rx_size_b.size != 0;
}

static inline bool iot_uart_write_in_progress( IotUARTHandle_t handle )
{
    return handle->udma->tx_size_b.size != 0;
}

static inline void uart_null_cb( IotUARTOperationStatus_t status, void * param )
{
    ( void ) status;
    ( void ) param;
}

static inline void uart_tx_udma_isr( void * ctx )
{
    IotUARTHandle_t const handle = ( IotUARTHandle_t ) ctx;
    handle->callback( eUartWriteCompleted, handle->userParam );
}

static inline void uart_rx_udma_isr( void * ctx )
{
    IotUARTHandle_t const handle = ( IotUARTHandle_t ) ctx;
    if( !handle->rx.sync )
    {
        handle->callback( eUartReadCompleted, handle->userParam );
    }
    handle->udma->irq_en_b.rx_irq_en = 1U;
}

static inline void uart_rx_irq_isr( void * ctx )
{
    IotUARTHandle_t const handle = ( IotUARTHandle_t ) ctx;
    uint16_t writer = handle->rx.writer;
    uint16_t reader = LOAD_CURSOR_CONSUME( &handle->rx.reader );

    do
    {
        handle->rx.buffer[ writer ] = handle->udma->data_b.rx_data;
        writer = ( writer + 1U ) & IOT_RX_CIRCULAR_MASK;
        // overflow handling
        if( reader == writer )
        {
            reader = ( writer + 1U ) & IOT_RX_CIRCULAR_MASK;
        }
    } while( handle->udma->valid_b.rx_data_valid == 1U );

    handle->rx.writer = writer;
    STORE_CURSOR_RELEASE( &handle->rx.reader, reader );
}

static inline bool uart_is_valid( IotUARTHandle_t handle )
{
    if( ( handle == NULL ) || ( handle->udma == NULL ) )
    {
        return false;
    }
    int id = IOT_UART_ID( handle->udma );
    return IOT_UART_PTR( id ) == handle->udma;
}

IotUARTHandle_t iot_uart_open( int32_t uartInstance )
{
    UdmaCtrl_t * pudma_ctrl;
    IotUARTHandle_t handle = NULL;
    uint8_t uart_id;

    if( ( uartInstance < 0 ) || ( uartInstance >= N_UART ) )
    {
        return NULL;
    }
    else if( uartContexts[ uartInstance ].udma != NULL )
    {
        return NULL;
    }

    handle = &uartContexts[ uartInstance ];

    if( handle != NULL )
    {
        IotUARTConfig_t config = { .ucFlowControl = 0U,
                                   .ucWordlength = 8U,
                                   .ulBaudrate = IOT_UART_BAUD_RATE_DEFAULT,
                                   .xParity = eUartParityNone,
                                   .xStopbits = eUartStopBitsOne };

        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;
        uart_id = ( uint8_t ) uartInstance;

        handle->udma = IOT_UART_PTR( uart_id );
        // reset and enable pUart clk
        pudma_ctrl->reg_rst |= ( UDMA_CTRL_UART0_CLKEN << uart_id );
        pudma_ctrl->reg_rst &= ~( ( uint32_t ) UDMA_CTRL_UART0_CLKEN
                                  << uart_id );
        pudma_ctrl->reg_cg |= ( UDMA_CTRL_UART0_CLKEN << uart_id );

        set_config_raw( handle->udma, &config );
        handle->baudrate = config.ulBaudrate;
        handle->parity = config.xParity;

        INITIALIZE_CURSOR( &handle->rx.reader, 0 );
        handle->rx.writer = 0;

        handle->udma->irq_en_b.rx_irq_en = 1U;
        handle->udma->uart_setup_b.en_tx = 1U;
        handle->udma->uart_setup_b.en_rx = 1U;

        handle->udma->rx_cfg_b.continuous = 0U;
        handle->udma->tx_cfg_b.continuous = 0U;

        while( handle->udma->status_b.rx_busy == 1U )
            ;
        while( handle->udma->status_b.tx_busy == 1U )
            ;

        // Set handlers
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UART_RX( uart_id ),
                                 uart_rx_irq_isr,
                                 handle );
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UDMA_UART_TX( uart_id ),
                                 uart_tx_udma_isr,
                                 handle );
        pi_fc_event_handler_set( ( uint32_t ) SOC_EVENT_UDMA_UART_RX( uart_id ),
                                 uart_rx_udma_isr,
                                 handle );

        handle->callback = uart_null_cb;

        // Enable SOC events and FC propagation
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UART_RX( uart_id ) );
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_UART_TX( uart_id ) );
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_UART_RX( uart_id ) );
    }

    return handle;
}

int32_t iot_uart_close( IotUARTHandle_t handle )
{
    int32_t result = IOT_UART_SUCCESS;
    UdmaCtrl_t * pudma_ctrl;
    int uart_id;

    if( !uart_is_valid( handle ) )
    {
        result = IOT_UART_INVALID_VALUE;
    }
    else
    {
        // cancel outstanding transactions
        ( void ) cancel_raw( handle );

        pudma_ctrl = ( UdmaCtrl_t * ) UDMA_CH_ADDR_CTRL;

        // disable UART clock
        uart_id = IOT_UART_ID( handle->udma );
        pudma_ctrl->reg_cg &= ~( uint32_t ) ( UDMA_CTRL_UART0_CLKEN
                                              << uart_id );
        handle->udma = NULL;

        // reset event callbacks
        pi_fc_event_handler_clear(
            ( uint32_t ) SOC_EVENT_UDMA_UART_TX( uart_id ) );
        pi_fc_event_handler_clear( ( uint32_t ) SOC_EVENT_UART_RX( uart_id ) );
        pi_fc_event_handler_clear(
            ( uint32_t ) SOC_EVENT_UDMA_UART_RX( uart_id ) );
    }

    return result;
}

void iot_uart_set_callback( IotUARTHandle_t uart,
                            IotUARTCallback_t callback,
                            void * userParam )
{
    if( uart_is_valid( uart ) && ( callback != NULL ) )
    {
        uart->callback = callback;
        uart->userParam = userParam;
    }
}

int32_t iot_uart_ioctl( IotUARTHandle_t handle,
                        IotUARTIoctlRequest_t request,
                        void * buffer )
{
    int32_t success = IOT_UART_SUCCESS;

    if( !uart_is_valid( handle ) )
    {
        success = IOT_UART_INVALID_VALUE;
    }
    else if( buffer == NULL )
    {
        success = IOT_UART_INVALID_VALUE;
    }

    else
    {
        switch( request )
        {
            case eUartSetConfig:
                success = set_config( handle,
                                      ( const IotUARTConfig_t * ) buffer );
                break;

            case eUartGetConfig:
                ( ( IotUARTConfig_t * ) buffer )[ 0 ] = retrieve_config(
                    handle );
                break;

            case eGetTxNoOfbytes:
                ( ( uint16_t * ) buffer )[ 0 ] = handle->tx.bytesRequested -
                                                 handle->udma->tx_size_b.size;

                break;

            case eGetRxNoOfbytes:
                ( ( uint16_t * ) buffer )[ 0 ] = handle->rx.bytesRequested -
                                                 handle->udma->rx_size_b.size;
                break;

            default:
                success = IOT_UART_INVALID_VALUE;
                break;
        }
    }

    return success;
}

int32_t iot_uart_write_async( IotUARTHandle_t handle,
                              uint8_t * const buffer,
                              size_t bytes )
{
    int32_t status = IOT_UART_SUCCESS;

    if( !uart_is_valid( handle ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( ( bytes == 0U ) || ( buffer == NULL ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( bytes > 0xFFFFU )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( iot_uart_write_in_progress( handle ) )
    {
        status = IOT_UART_BUSY;
    }
    else
    {
        handle->tx.bytesRequested = bytes & 0xFFFFU;
        handle->udma->tx_saddr = ( uint32_t ) ( uintptr_t ) buffer;
        handle->udma->tx_size_b.size = handle->tx.bytesRequested;
        handle->udma->tx_cfg_b.en = 1U;
    }

    return status;
}

int32_t iot_uart_read_async( IotUARTHandle_t handle,
                             uint8_t * buffer,
                             size_t bytes )
{
    int32_t status = IOT_UART_SUCCESS;

    if( !uart_is_valid( handle ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( ( bytes == 0U ) || ( buffer == NULL ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( bytes > 0xFFFFU )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else if( iot_uart_read_in_progress( handle ) )
    {
        status = IOT_UART_BUSY;
    }
    else
    {
        const uint32_t uartEventId = ( uint32_t ) SOC_EVENT_UDMA_UART_RX(
            IOT_UART_ID( handle->udma ) );
        handle->rx.bytesRequested = bytes & 0xFFFFU;

        // determine buffer fill strategy
        handle->udma->irq_en_b.rx_irq_en = 0U;

        // 2 cases:
        // - fewer bytes available than requested
        // - all bytes available

        uint16_t reader = LOAD_CURSOR_ACQUIRE( &handle->rx.reader );
        const size_t bytesAvailable = ( handle->rx.writer - reader ) &
                                      IOT_RX_CIRCULAR_MASK;

        size_t fillBytes;
        // copy out entire circular buffer, enable DMA
        if( bytesAvailable < bytes )
        {
            fillBytes = bytesAvailable;

            // set up a DMA for the remaining bytes
            pi_fc_event_disable( uartEventId );
            uint16_t dmaBytes = ( uint16_t ) ( bytes - bytesAvailable );
            handle->udma
                ->rx_saddr = ( uint32_t ) ( uintptr_t ) &buffer[ bytesAvailable ];
            handle->udma->rx_size_b.size = dmaBytes;
            handle->udma->rx_cfg_b.en = 1U;
        }
        // copy what bytes are needed, re-enable IRQ
        else // bytes <= bytesAvailable
        {
            fillBytes = bytes;
            handle->udma->irq_en_b.rx_irq_en = 1U;
        }

        // copy out bytes needed from circular buffer
        for( size_t i = 0; i != fillBytes; ++i )
        {
            buffer[ i ] = handle->rx.buffer[ reader ];

            // check for overflows
            uint16_t current = LOAD_CURSOR_CONSUME( &handle->rx.reader );
            if( reader != current )
            {
                reader = current;
            }
            else
            {
                reader = ( reader + 1U ) & IOT_RX_CIRCULAR_MASK;
                STORE_CURSOR_RELEASE( &handle->rx.reader, reader );
            }
        }

        // call UDMA ISR if transaction completed, otherwise enable it
        if( handle->udma->rx_size_b.size == 0 )
        {
            uart_rx_udma_isr( handle );
        }
        else
        {
            pi_fc_event_enable( uartEventId );
        }
    }

    return status;
}

int32_t iot_uart_write_sync( IotUARTHandle_t handle,
                             uint8_t * const buffer,
                             size_t bytes )
{
    int32_t status;

    if( !uart_is_valid( handle ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else
    {
        uint32_t uartTxEventId = ( uint32_t ) SOC_EVENT_UDMA_UART_TX(
            IOT_UART_ID( handle->udma ) );
        pi_fc_event_disable( uartTxEventId );

        status = iot_uart_write_async( handle, buffer, bytes );

        if( status == IOT_UART_SUCCESS )
        {
            while( iot_uart_write_in_progress( handle ) )
                ;
        }

        pi_fc_event_enable( uartTxEventId );
    }

    return status;
}

int32_t iot_uart_read_sync( IotUARTHandle_t handle,
                            uint8_t * buffer,
                            size_t bytes )
{
    int32_t status = IOT_UART_SUCCESS;

    if( !uart_is_valid( handle ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else
    {
        handle->rx.sync = true;

        status = iot_uart_read_async( handle, buffer, bytes );

        if( status == IOT_UART_SUCCESS )
        {
            while( iot_uart_read_in_progress( handle ) )
                ;
        }

        handle->rx.sync = false;
    }

    return status;
}

static IotUARTConfig_t retrieve_config( IotUARTHandle_t handle )
{
    UdmaUart_t * uart = handle->udma;
    uint32_t div = uart->uart_setup_b.div;
    IotUARTConfig_t config = {
        .ulBaudrate = ( div == IOT_REF_CLK / handle->baudrate )
                          ? handle->baudrate
                          : IOT_REF_CLK / div,
        .xParity = ( uart->uart_setup_b.parity_en == 0 ) ? eUartParityNone
                                                         : handle->parity,
        .xStopbits = uart->uart_setup_b.stop_bits == 1U ? eUartStopBitsTwo
                                                        : eUartStopBitsOne,
        .ucWordlength = uart->uart_setup_b.bits + 5U,
    };
    return config;
}

static int32_t set_config( IotUARTHandle_t handle,
                           const IotUARTConfig_t * config )
{
    int32_t result = IOT_UART_SUCCESS;

    if( !uart_is_valid( handle ) )
    {
        result = IOT_UART_INVALID_VALUE;
    }

    else if( ( config->ulBaudrate > IOT_REF_CLK ) )
    {
        result = IOT_UART_INVALID_VALUE;
    }

    else if( ( config->ucWordlength != 0 ) &&
             ( ( config->ucWordlength < 5U ) ||
               ( config->ucWordlength > 8U ) ) )
    {
        result = IOT_UART_INVALID_VALUE;
    }

    else if( config->ucFlowControl != 0U )
    {
        result = IOT_UART_INVALID_VALUE;
    }

    // TODO: check parity

    else if( iot_uart_read_in_progress( handle ) ||
             iot_uart_write_in_progress( handle ) )
    {
        result = IOT_UART_BUSY;
    }

    else
    {
        while( handle->udma->status_b.tx_busy == 1U )
            ;

        set_config_raw( handle->udma, config );
        handle->baudrate = config->ulBaudrate;
        handle->parity = config->xParity;
    }

    return result;
}

int32_t iot_uart_cancel( IotUARTHandle_t handle )
{
    int32_t status = IOT_UART_NOTHING_TO_CANCEL;
    int uart_id;

    if( !uart_is_valid( handle ) )
    {
        status = IOT_UART_INVALID_VALUE;
    }
    else
    {
        uart_id = IOT_UART_ID( handle->udma );

        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_UART_RX( uart_id ) );
        pi_fc_event_disable( ( uint32_t ) SOC_EVENT_UDMA_UART_TX( uart_id ) );
        // critical section

        status = cancel_raw( handle );

        // critical section
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_UART_RX( uart_id ) );
        pi_fc_event_enable( ( uint32_t ) SOC_EVENT_UDMA_UART_TX( uart_id ) );
    }

    return status;
}

static void set_config_raw( UdmaUart_t * udma, const IotUARTConfig_t * config )
{
    udma->uart_setup_b.div = ( uint16_t ) ( IOT_REF_CLK / config->ulBaudrate );

    udma->uart_setup_b.parity_en = ( config->xParity != eUartParityNone );

    if( config->ucWordlength != 0 )
    {
        udma->uart_setup_b.bits = ( config->ucWordlength - 5U ) & 0x03U;
    }

    udma->uart_setup_b.stop_bits = ( config->xStopbits == eUartStopBitsTwo );
}

static int32_t cancel_raw( IotUARTHandle_t handle )
{
    int32_t status = IOT_UART_NOTHING_TO_CANCEL;

    if( iot_uart_write_in_progress( handle ) )
    {
        handle->udma->tx_cfg_b.clr = 1U;

        // allow current UART baud to finish
        while( handle->udma->status_b.tx_busy == 1 )
            ;

        status = IOT_UART_SUCCESS;
    }

    if( iot_uart_read_in_progress( handle ) )
    {
        handle->rx.bytesRequested -= handle->udma->rx_size_b.size;
        handle->udma->rx_cfg_b.clr = 1U;
        status = IOT_UART_SUCCESS;
    }

    return status;
}
