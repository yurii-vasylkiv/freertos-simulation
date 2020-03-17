#ifndef PRESSURE_SENSOR_BMP3_H
#define PRESSURE_SENSOR_BMP3_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// pressure_sensor_bmp3.h
// UMSATS 2018-2020
//
// Repository:
//  UMSATS > Avionics 2019
//
// File Description:
//  Control and usage of BMP3 sensor inside of RTOS task.
//
// History
// 2019-03-04 Eric Kapilik
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include "flight_system_configuration.h"
#include "protocols/UART.h"


#define PRES_LENGTH         3   //Length of a pressure measurement in bytes.
#define TEMP_LENGTH         3   //Length of a temperature measurement in bytes.
#define ALT_LENGTH          4
#define TIMEOUT             100 // milliseconds

//Groups a time stamp with the reading.
typedef struct pressure_sensor_data
{
    uint32_t time_ticks; //time of sensor reading in ticks.
    /*! Compensated temperature */
    int64_t temperature;
    /*! Compensated pressure */
    int64_t pressure;
} pressure_sensor_data;


int    pressure_sensor_init               (FlightSystemConfiguration * parameters);


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Task for testing the BMP3 sensor.
//    - initializes sensor
//    - read data & print to UART screen cycle
//
// Returns:
//  VOID
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void    pressure_sensor_start               ( void *pvParameters );
bool    pressure_sensor_test                ( void );
bool    pressure_sensor_read                ( pressure_sensor_data * buffer );
void    pressure_sensor_data_to_bytes       ( pressure_sensor_data reading, uint8_t * bytes );
float   pressure_sensor_calculate_altitude  ( pressure_sensor_data * reading );
bool    pressure_sensor_add_measurement     ( pressure_sensor_data * _data );

#ifdef __cplusplus
}
#endif


#endif // PRESSURE_SENSOR_BMP3_H
