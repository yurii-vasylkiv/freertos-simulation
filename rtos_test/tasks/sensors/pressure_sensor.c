//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// pressure_sensor_bmp3.c
// UMSATS 2018-2020
//
// Repository:
//  UMSATS > Avionics 2019
//
// File Description:
//  Control and usage of BMP3 sensor inside of RTOS task.
//
// History
// 2019-04-06 Eric Kapilik
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

#include "pressure_sensor.h"
#include <math.h>
#include <stdbool.h>
#include <stm32/STM32.h>

#include "protocols/SPI.h"
#include "flight_system_configuration.h"
#include "protocols/UART.h"
#include "utilities/common.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "datafeeder.h"

#include "memory_management/memory_manager.h"
#include "events/event_detector.h"





#define PRES_TYPE           0x200000
#define TEMP_TYPE           0x100000

static float s_reference_pressure = 0.0f;
static float s_reference_altitude = 0.0f;

static char buf[128];

static QueueHandle_t s_queue;
static xTaskHandle handle;

static void delay_ms( uint32_t period_ms );



int pressure_sensor_init( FlightSystemConfiguration * parameters )
{
    int status = spi2_init( );
    if ( status != 0 )
    {
        return status;
    }

    s_queue = xQueueCreate( 10, sizeof( pressure_sensor_data ) );
    if ( s_queue == NULL )
    {
        return 2;
    }

    vQueueAddToRegistry( s_queue, "bmp3_queue" );

    return status;
}



float pressure_sensor_calculate_altitude( pressure_sensor_data * reading )
{
    if ( reading == NULL )
    {
        return 0.0;
    }

    float p_term = powf( ( s_reference_pressure / ( reading->pressure / 100 ) ), ( 1 / 5.257F ) ) - 1;
    float t_term = ( reading->temperature / 100 ) + 273.15F;
    return ( uint32_t ) ( p_term * t_term ) / 0.0065F + s_reference_altitude;
}



void prv_pressure_sensor_start( void * pvParameters )
{
    bool result_flag;
    press_data cxx_press_data;
    /* Variable used to store the compensated data */
    pressure_sensor_data dataStruct;

    uint32_t time_start = 1;

    while ( 1 )
    {
        result_flag = datafeeder_get_press ( &cxx_press_data );
        if ( ! result_flag )
        {
            continue;
        }

        dataStruct.pressure     = cxx_press_data.pressure;
        dataStruct.temperature  = cxx_press_data.temperature;
        dataStruct.time_ticks   = cxx_press_data.timestamp;

//        dataStruct.time_ticks   = xTaskGetTickCount() - time_start;

//        DISPLAY( "NEW PRESS data: %d\n", dataStruct.time_ticks);

        pressure_sensor_add_measurement( &dataStruct );
    }
}



void pressure_sensor_start( void * pvParameters )
{
    if ( pdFALSE == xTaskCreate( prv_pressure_sensor_start, "pressure_sensor-task", configMINIMAL_STACK_SIZE, pvParameters, 5, &handle ) )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    }
}



static void delay_ms ( uint32_t period_ms )
{
    vTaskDelay( ( TickType_t ) period_ms );
}



bool pressure_sensor_test ( void )
{
    return true;
}



bool pressure_sensor_read ( pressure_sensor_data * buffer )
{
    return pdPASS == xQueueReceive ( s_queue, buffer, 0 );
}



void pressure_sensor_data_to_bytes ( pressure_sensor_data bmp_reading, uint8_t * bytes )
{
    //Update the header bytes.
    uint32_t header = ( bytes[ 0 ] << 16 ) + ( bytes[ 1 ] << 8 ) + bytes[ 2 ];
    header |= PRES_TYPE | TEMP_TYPE;

    write_24( header, &bytes[ 0 ] );
    write_24( bmp_reading.pressure, &bytes[ 15 ] );
    write_24( bmp_reading.temperature, &bytes[ 18 ] );
    float altitude = pressure_sensor_calculate_altitude( &bmp_reading );
    float2bytes( altitude, &bytes[ 21 ] );
}



bool pressure_sensor_add_measurement ( pressure_sensor_data * _data )
{
    return pdTRUE == xQueueSend ( s_queue, _data, 0 );
}



#ifdef __cplusplus
}
#endif
