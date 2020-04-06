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
#include <board/board.h>

#include "protocols/SPI.h"
#include "core/system_configuration.h"
#include "protocols/UART.h"
#include "utilities/common.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "datafeeder.h"

#include "memory-management/memory_manager.h"
#include "event-detection/event_detector.h"



#define PRES_TYPE           0x200000
#define TEMP_TYPE           0x100000

static float s_reference_pressure = 0.0f;
static float s_reference_altitude = 0.0f;

static char buf[128];

static QueueHandle_t s_queue;
static xTaskHandle handle;
static uint8_t dataNeedsToBeConverted = 0;

static void delay_ms( uint32_t period_ms );



int pressure_sensor_init( FlightSystemConfiguration * parameters )
{
    int status = spi2_init( );
    if ( status != SPI_OK )
    {
        return PRESS_SENSOR_ERR;
    }

    s_queue = xQueueCreate( 10, sizeof( pressure_sensor_data ) );
    if ( s_queue == NULL )
    {
        return PRESS_SENSOR_ERR;
    }

    vQueueAddToRegistry( s_queue, "bmp3_queue" );

    return PRESS_SENSOR_OK;
}



void prv_pressure_sensor_start( void * pvParameters )
{
#if (userconf_FREE_RTOS_SIMULATOR_MODE_ON)
    #define MAKE_STR(x) _MAKE_STR(x)
    #define _MAKE_STR(x) #x
    #if (userconf_USE_COTS_DATA == 1)
        const char *CSV_FILE_PATH = MAKE_STR(COTS_CSV_FILE_PATH) ;
        if(!data_feeder_is_started()){
            data_feeder_start ( CSV_FILE_PATH );
        }
    #else
        const char *CSV_FILE_PATH = MAKE_STR(SRAD_CSV_FILE_PATH) ;
        if(!data_feeder_is_started()){
            data_feeder_start ( CSV_FILE_PATH );
        }
    #endif
#endif

    if(pvParameters != NULL)
    {
        FlightSystemConfiguration * systemConfiguration = ( FlightSystemConfiguration * ) pvParameters;
        dataNeedsToBeConverted = systemConfiguration->pressure_data_needs_to_converted;
    }

    (void) pvParameters;

    bool result_flag;
    press_data cxx_press_data;
    /* Variable used to store the compensated data */
    pressure_sensor_data dataStruct;
    memset(&dataStruct, 0, sizeof(pressure_sensor_data));

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
        dataStruct.timestamp    = cxx_press_data.timestamp;

//        dataStruct.time_ticks   = xTaskGetTickCount() - time_start;

//        DISPLAY( "NEW PRESS data: %d\n", dataStruct.time_ticks);

        if(dataNeedsToBeConverted)
        {
            dataStruct.pressure = (dataStruct.pressure / 100);
        }

        pressure_sensor_add_measurement( &dataStruct );
        memset(&dataStruct, 0, sizeof(pressure_sensor_data));
    }
}



void pressure_sensor_start( void * pvParameters )
{
    if ( pdFALSE == xTaskCreate( prv_pressure_sensor_start, "pressure_sensor-task", configMINIMAL_STACK_SIZE, pvParameters, 5, &handle ) )
    {
        board_error_handler( __FILE__, __LINE__ );
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


bool pressure_sensor_add_measurement ( pressure_sensor_data * _data )
{
    return pdTRUE == xQueueSend ( s_queue, _data, 0 );
}



#ifdef __cplusplus
}
#endif
