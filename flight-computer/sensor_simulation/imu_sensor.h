#ifndef SENSOR_AG_H
#define SENSOR_AG_H

#include <inttypes.h>
#include <stdbool.h>
#include "core/system_configuration.h"
#include "protocols/UART.h"


#define ACC_LENGTH  6 // Length of a accelerometer measurement in bytes.
#define GYRO_LENGTH 6 // Length of a gyroscope measurement in bytes.

typedef enum { IMU_ERR   = 0, IMU_OK    = 1 } IMUStatus;


//Groups both sensor readings and a time stamp.
typedef struct imu_sensor_data
{
    float    acc_x;
    float    acc_y;
    float    acc_z;

    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    
    float   timestamp; // time of sensor reading in ticks.
}imu_sensor_data;


int imu_sensor_test            ();
int  imu_sensor_init           ( FlightSystemConfiguration * parameters );
void imu_sensor_start          ( void *param );
bool imu_read                  ( imu_sensor_data * buffer );
bool imu_add_measurement       ( imu_sensor_data *_data );


#endif // SENSOR_AG_H
