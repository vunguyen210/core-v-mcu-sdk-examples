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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "iot_gpio.h"

#include "hal/include/hal_apb_gpio_reg_defs.h"
#include "hal/include/hal_apb_soc_ctrl_reg_defs.h"
#include "hal/include/hal_fc_event.h"
#include "target/core-v-mcu/include/core-v-mcu-config.h"

struct IotGpioDescriptor
{
    IotGpioCallback_t callback;
    void * userParam;
    IotGpioOutputMode_t mode;
    bool open;
    uint8_t gpioNum;
};

#define IOT_GPIO_PTR ( ( ApbGpio_t * ) GPIO_START_ADDR )
#define IOT_SOC_PTR  ( ( SocCtrl_t * ) SOC_CTRL_START_ADDR )

static struct IotGpioDescriptor descriptors[ N_GPIO ] = { 0 };

static inline void rdstat_select( uint8_t gpio_num )
{
    ApbGpio_t * gpio = IOT_GPIO_PTR;
    while( gpio->rdstat_b.gpio_sel != gpio_num )
    {
        gpio->setsel_b.gpio_num = gpio_num;
    }
}

static inline uint32_t rdstat_read( uint8_t gpio_num )
{
    ApbGpio_t * gpio = IOT_GPIO_PTR;
    uint32_t value;

    while( 1 )
    {
        value = gpio->rdstat;
        if( ( value & gpio_num ) == gpio_num )
        {
            return value;
        }
        gpio->setsel_b.gpio_num = gpio_num;
    }
}

static inline int32_t iot_gpio_set_dir( uint8_t gpio_num,
                                        IotGpioDirection_t dir,
                                        IotGpioOutputMode_t mode )
{
    ApbGpio_t * gpio = IOT_GPIO_PTR;
    int32_t status = IOT_GPIO_SUCCESS;
    switch( dir )
    {
        case eGpioDirectionInput:
            gpio->setmode = ( gpio_num << REG_SETMODE_gpio_num_LSB );
            break;

        case eGpioDirectionOutput:
            gpio->setmode = ( gpio_num << REG_SETMODE_gpio_num_LSB ) |
                            ( ( 0x2U - mode ) << REG_SETMODE_mode_LSB );
            break;

        default:
            status = IOT_GPIO_INVALID_VALUE;
    }

    return status;
}

static inline int32_t iot_gpio_set_mode( uint8_t gpio_num,
                                         IotGpioOutputMode_t mode )
{
    int32_t status = IOT_GPIO_SUCCESS;
    uint32_t rdstat;

    switch( mode )
    {
        case eGpioOpenDrain:
            rdstat = rdstat_read( gpio_num );
            rdstat = ( rdstat >> REG_RDSTAT_mode_LSB ) & REG_RDSTAT_mode_MASK;

            if( rdstat != 0x0 )
            {
                IOT_GPIO_PTR->setmode = ( gpio_num
                                          << REG_SETMODE_gpio_num_LSB ) |
                                        ( 0x2U << REG_SETMODE_mode_LSB );
            }
            break;

        case eGpioPushPull:
            rdstat = rdstat_read( gpio_num );
            rdstat = ( rdstat >> REG_RDSTAT_mode_LSB ) & REG_RDSTAT_mode_MASK;

            if( rdstat != 0x0 )
            {
                IOT_GPIO_PTR->setmode = ( gpio_num
                                          << REG_SETMODE_gpio_num_LSB ) |
                                        ( 0x1U << REG_SETMODE_mode_LSB );
            }
            break;

        default:
            status = IOT_GPIO_INVALID_VALUE;
    }

    return status;
}

static inline void iot_gpio_set_interrupt_impl( uint8_t gpio_num,
                                                uint8_t interrupt )
{
    IOT_GPIO_PTR->setint = ( uint32_t ) ( gpio_num
                                          << REG_SETINT_gpio_num_LSB ) |
                           ( ( interrupt & REG_SETINT_INTTYPE_MASK )
                             << REG_SETINT_INTTYPE_LSB ) |
                           ( 0x1U << REG_SETINT_INTENABLE_LSB );
}

static inline void iot_gpio_clr_interrupt( uint8_t gpio_num )
{
    IOT_GPIO_PTR->setint = ( gpio_num << REG_SETINT_gpio_num_LSB ) |
                           ( 0x7U << REG_SETINT_INTTYPE_LSB );
}

static inline int32_t iot_gpio_set_interrupt( uint8_t gpio_num,
                                              IotGpioInterrupt_t interrupt )
{
    int32_t status = IOT_GPIO_SUCCESS;

    switch( interrupt )
    {
        /*
         * 0x0: active low, level type interrupt
         *
         * 0x1: falling edge type interrupt
         *
         * 0x2: rising edge type interrupt
         *
         * 0x3: no interrupt
         *
         * 0x4: active high, level type interrupt
         *
         * 0x5 to 0x7: no interrupt
         */
        case eGpioInterruptLow:
            iot_gpio_set_interrupt_impl( gpio_num, 0x0 );
            break;

        case eGpioInterruptFalling:
            iot_gpio_set_interrupt_impl( gpio_num, 0x1 );
            break;

        case eGpioInterruptRising:
            iot_gpio_set_interrupt_impl( gpio_num, 0x2 );
            break;

        case eGpioInterruptHigh:
            iot_gpio_set_interrupt_impl( gpio_num, 0x4 );
            break;

        case eGpioInterruptNone:
            iot_gpio_clr_interrupt( gpio_num ); // 0x7
            break;

        case eGpioInterruptEdge:
            status = IOT_GPIO_FUNCTION_NOT_SUPPORTED;
            break;

        default:
            status = IOT_GPIO_INVALID_VALUE;
    }

    return status;
}
static inline int32_t iot_gpio_set_function( uint8_t gpio_num,
                                             int32_t function )
{
    int32_t status = IOT_GPIO_SUCCESS;

    if( ( function & REG_IO_CTRL_MUX_MASK ) == function )
    {
        IOT_SOC_PTR->io_ctrl_b[ gpio_num + 7U ].mux = ( uint32_t ) function &
                                                      REG_IO_CTRL_MUX_MASK;
    }
    else
    {
        status = IOT_GPIO_INVALID_VALUE;
    }

    return status;
}

static inline void gpio_null_callback( uint8_t val, void * ctx )
{
    ( void ) val;
    ( void ) ctx;
}

static void gpio_isr( void * ctx )
{
    IotGpioHandle_t handle = ( IotGpioHandle_t ) ctx;
    rdstat_select( handle->gpioNum );
    handle->callback( IOT_GPIO_PTR->rdstat_b.input, handle->userParam );
}

IotGpioHandle_t iot_gpio_open( int32_t lGpioNumber )
{
    IotGpioHandle_t handle = { 0 };
    struct IotGpioDescriptor config = { 0 };

    if( ( lGpioNumber >= 0 ) && ( lGpioNumber < N_GPIO ) )
    {
        handle = &descriptors[ lGpioNumber ];

        if( handle->open )
        {
            handle = NULL;
        }
        else
        {
            config.gpioNum = ( uint8_t ) lGpioNumber;

            config.open = true;

            *handle = config;

            iot_gpio_clr_interrupt( config.gpioNum );
            rdstat_select( handle->gpioNum );
            IOT_GPIO_PTR->rdstat_b.inten = 0U;

            pi_fc_event_handler_set( config.gpioNum + 128U, gpio_isr, handle );

            pi_fc_event_enable( config.gpioNum + 128 );

            handle->callback = gpio_null_callback;
        }
    }

    return handle;
}

int32_t iot_gpio_close( IotGpioHandle_t const handle )
{
    int32_t status = IOT_GPIO_SUCCESS;

    if( handle == NULL )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else if( !handle->open )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else
    {
        // disable interrupts
        iot_gpio_clr_interrupt( handle->gpioNum );
        rdstat_select( handle->gpioNum );
        IOT_GPIO_PTR->rdstat_b.inten = 0U;

        pi_fc_event_disable( handle->gpioNum + 128U );
        pi_fc_event_handler_clear( handle->gpioNum + 128U );
        handle->callback = gpio_null_callback;
        handle->userParam = NULL;
        handle->open = false;
    }

    return status;
}

void iot_gpio_set_callback( IotGpioHandle_t const handle,
                            IotGpioCallback_t callback,
                            void * userParam )
{
    if( ( handle != NULL ) && ( callback != NULL ) && ( handle->open ) )
    {
        handle->callback = callback;
        handle->userParam = userParam;
    }
}

int32_t iot_gpio_write_sync( IotGpioHandle_t const handle, uint8_t pinState )
{
    int32_t status = IOT_GPIO_SUCCESS;
    if( handle == NULL )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else if( !handle->open )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else
    {
        pi_fc_event_disable( handle->gpioNum + 128 );

        switch( pinState )
        {
            case 0U:
                IOT_GPIO_PTR->clrgpio_b.gpio_num = handle->gpioNum;
                break;
            case 1U:
                IOT_GPIO_PTR->setgpio_b.gpio_num = handle->gpioNum;
                break;
            default:
                status = IOT_GPIO_INVALID_VALUE;
        }

        pi_fc_event_enable( handle->gpioNum + 128 );
    }
    return status;
}

int32_t iot_gpio_read_sync( IotGpioHandle_t const handle, uint8_t * pinState )
{
    int32_t status = IOT_GPIO_SUCCESS;
    uint32_t rdstat;
    if( ( handle == NULL ) || ( pinState == NULL ) )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else if( !handle->open )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else
    {
        rdstat = rdstat_read( handle->gpioNum );
        if( ( ( rdstat >> REG_RDSTAT_mode_LSB ) & REG_RDSTAT_mode_MASK ) ==
            0x0 )
        {
            pinState[ 0 ] = ( rdstat >> REG_RDSTAT_INPUT_LSB ) &
                            REG_RDSTAT_INPUT_MASK;
        }
        else
        {
            pinState[ 0 ] = ( rdstat >> REG_RDSTAT_OUTPUT_LSB ) &
                            REG_RDSTAT_OUTPUT_MASK;
        }
    }
    return status;
}

int32_t iot_gpio_ioctl( IotGpioHandle_t const handle,
                        IotGpioIoctlRequest_t request,
                        void * const param )
{
    int32_t status = IOT_GPIO_SUCCESS;
    uint32_t rdstat;

    if( ( handle == NULL ) || ( param == NULL ) )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }
    else if( !handle->open )
    {
        status = IOT_GPIO_INVALID_VALUE;
    }

    if( status == IOT_GPIO_SUCCESS )
    {
        switch( request )
        {
            case eSetGpioDirection:
                status = iot_gpio_set_dir( handle->gpioNum,
                                           ( ( const IotGpioDirection_t * )
                                                 param )[ 0 ],
                                           handle->mode );
                break;

            case eSetGpioOutputMode:
                status = iot_gpio_set_mode( handle->gpioNum,
                                            ( ( const IotGpioOutputMode_t * )
                                                  param )[ 0 ] );
                if( status == IOT_GPIO_SUCCESS )
                {
                    handle->mode = ( (
                        const IotGpioOutputMode_t * ) param )[ 0 ];
                }
                break;

            case eSetGpioInterrupt:
                status = iot_gpio_set_interrupt(
                    handle->gpioNum,
                    ( ( const IotGpioInterrupt_t * ) param )[ 0 ] );
                break;

            case eSetGpioFunction:
                status = iot_gpio_set_function( handle->gpioNum,
                                                ( ( const int32_t * )
                                                      param )[ 0 ] );
                break;

            case eGetGpioDirection:
                rdstat = rdstat_read( handle->gpioNum );
                rdstat = ( rdstat >> REG_RDSTAT_mode_LSB ) &
                         REG_RDSTAT_mode_MASK;

                ( ( IotGpioDirection_t * )
                      param )[ 0 ] = ( rdstat == 0x0U ) ? eGpioDirectionInput
                                                        : eGpioDirectionOutput;

                break;

            case eGetGpioOutputType:
                ( ( IotGpioOutputMode_t * ) param )[ 0 ] = handle->mode;

                break;

            case eGetGpioInterrupt:
                rdstat = rdstat_read( handle->gpioNum );
                rdstat = ( rdstat >> REG_RDSTAT_INTTYPE_LSB ) &
                         REG_RDSTAT_INTTYPE_MASK;

                /*
                 * 0x0: active low, level type interrupt
                 *
                 * 0x1: falling edge type interrupt
                 *
                 * 0x2: rising edge type interrupt
                 *
                 * 0x3: no interrupt
                 *
                 * 0x4: active high, level type interrupt
                 *
                 * 0x5 to 0x7: no interrupt
                 */

                switch( rdstat )
                {
                    case 0x0:
                        ( ( IotGpioInterrupt_t * )
                              param )[ 0 ] = eGpioInterruptLow;
                        break;
                    case 0x1:
                        ( ( IotGpioInterrupt_t * )
                              param )[ 0 ] = eGpioInterruptFalling;
                        break;
                    case 0x2:
                        ( ( IotGpioInterrupt_t * )
                              param )[ 0 ] = eGpioInterruptRising;
                        break;
                    case 0x3:
                    case 0x5:
                    case 0x6:
                    case 0x7:
                        ( ( IotGpioInterrupt_t * )
                              param )[ 0 ] = eGpioInterruptNone;

                        break;
                    case 0x4:
                        ( ( IotGpioInterrupt_t * )
                              param )[ 0 ] = eGpioInterruptHigh;
                }
                break;

            case eGetGpioFunction:
                ( ( int32_t * )
                      param )[ 0 ] = IOT_SOC_PTR
                                         ->io_ctrl_b[ handle->gpioNum + 7U ]
                                         .mux;
                break;

            case eSetGpioPull:
            case eSetGpioSpeed:
            case eSetGpioDriveStrength:
            case eGetGpioPull:
            case eGetGpioSpeed:
            case eGetGpioDriveStrength:
                status = IOT_GPIO_FUNCTION_NOT_SUPPORTED;
                break;

            default:
                status = IOT_GPIO_INVALID_VALUE;
        }
    }

    return status;
}
