#ifndef SENSOR_AG_H
#define SENSOR_AG_H

#include <inttypes.h>
#include <stdbool.h>
#include "flight_system_configuration.h"
#include "protocols/UART.h"


#define ACC_LENGTH  6 // Length of a accelerometer measurement in bytes.
#define GYRO_LENGTH 6 // Length of a gyroscope measurement in bytes.


//Groups both sensor readings and a time stamp.
typedef struct imu_sensor_data
{
    int16_t    acc_x;
    int16_t    acc_y;
    int16_t    acc_z;
    
    int16_t    gyro_x;
    int16_t    gyro_y;
    int16_t    gyro_z;
    
    uint32_t   time_ticks; // time of sensor reading in ticks.
}imu_sensor_data;


int imu_sensor_test            ();
int  imu_sensor_init           ( FlightSystemConfiguration * parameters );
void imu_sensor_start          ( void *param );
bool imu_read                  ( imu_sensor_data * buffer );
void imu_sensor_data_to_bytes  ( imu_sensor_data reading, uint8_t* bytes, uint32_t timestamp );

float imu_sensor_acc2g                  ( int16_t acc_value );
int16_t imu_sensor_g2acc                ( float g );

float imu_sensor_acc2g                  ( int16_t acc_value );
int16_t imu_sensor_deg_per_sec2rot      ( float deg_per_sec );

bool imu_add_measurement       ( imu_sensor_data *_data );


#endif // SENSOR_AG_H
