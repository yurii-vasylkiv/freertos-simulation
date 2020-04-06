#ifndef PRESSURE_SENSOR_BMP3_H
#define PRESSURE_SENSOR_BMP3_H


#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include "core/system_configuration.h"
#include "protocols/UART.h"


#define PRES_LENGTH         3   //Length of a pressure measurement in bytes.
#define TEMP_LENGTH         3   //Length of a temperature measurement in bytes.
#define ALT_LENGTH          4
#define TIMEOUT             100 // milliseconds

//Groups a time stamp with the reading.
typedef struct pressure_sensor_data
{
    float timestamp; //time of sensor reading in ticks.
    /*! Compensated temperature */
    float temperature;
    /*! Compensated pressure */
    float pressure;
} pressure_sensor_data;

typedef enum { PRESS_SENSOR_ERR   = 0, PRESS_SENSOR_OK    = 1 } PressureSensorStatus;


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
bool    pressure_sensor_add_measurement     ( pressure_sensor_data * _data );

#ifdef __cplusplus
}
#endif


#endif // PRESSURE_SENSOR_BMP3_H
