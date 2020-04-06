//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS Google Drive: UMSATS/Guides and HowTos.../Command and Data Handling (CDH)/Coding Standards
//
// File Description:
//  Reads sensor data for accelerometer and gyroscope from the BMI088
//  On prototype flight computer:
//            +Z is out of the board (perpendicular to board surface when on a table).
//            +X is towards the recovery circuit (away from where the battery connects).
//            +Y is towards the crystal (away from the programming header).
// History
// 2019-03-29 by Benjamin Zacharias
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <board/board.h>
#include "memory-management/memory_manager.h"
#include "board/components/imu_sensor.h"
#include "protocols/SPI.h"
#include "core/system_configuration.h"
#include "utilities/common.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "datafeeder.h"
#include "math.h"

#define BMI088_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI088_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI088_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI088_ACCEL_RANGE_24G                      UINT8_C(0x03)

#define BMI08X_GYRO_RANGE_2000_DPS                  UINT8_C(0x00)
#define BMI08X_GYRO_RANGE_1000_DPS                  UINT8_C(0x01)
#define BMI08X_GYRO_RANGE_500_DPS                   UINT8_C(0x02)
#define BMI08X_GYRO_RANGE_250_DPS                   UINT8_C(0x03)
#define BMI08X_GYRO_RANGE_125_DPS                   UINT8_C(0x04)


#define ACC_BANDWIDTH			BMI08X_ACCEL_BW_NORMAL
#define ACC_ODR					BMI08X_ACCEL_ODR_100_HZ
#define ACC_RANGE				BMI088_ACCEL_RANGE_12G
#define ACC_PWR					BMI08X_ACCEL_PM_ACTIVE

#define GYRO_BANDWIDTH			BMI08X_GYRO_BW_23_ODR_200_HZ
#define GYRO_ODR				BMI08X_GYRO_BW_23_ODR_200_HZ
#define GYRO_RANGE				BMI08X_GYRO_RANGE_1000_DPS
#define GYRO_PWR				BMI08X_GYRO_PM_NORMAL

#define ACC_TYPE                         0x800000
#define GYRO_TYPE                        0x400000

static QueueHandle_t s_queue;
static xTaskHandle handle;
static uint8_t dataNeedsToBeConverted = 0;

//Wrapper functions for read and write
int8_t user_spi_read( uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t len );

int8_t user_spi_write( uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t len );

void delay_ms( uint32_t period );

static float imu_sensor_acc2g                ( int16_t acc_value );
static float imu_sensor_g2acc                ( float g );
static float imu_sensor_rot2deg_per_sec      ( int16_t gyro_value );
static float imu_sensor_deg_per_sec2rot      ( float deg_per_sec );


static float imu_sensor_acc2g ( int16_t acc_value )
{
    const int16_t range = pow (2, ( ACC_RANGE + 1 ) ) * 1.5;
    const float result = (float) acc_value / 32768  * range;
    return result;
}

static float imu_sensor_g2acc ( float g )
{
    const uint16_t  result = (int16_t ) ( g * 32768 / ( pow ( 2, ( ACC_RANGE + 1 ) ) * 1.5 ) );
    return result;
}

static float imu_sensor_rot2deg_per_sec ( int16_t gyro_value )
{
//    /**
//    * Registers containing the angular velocity sensor output. The sensor output is stored as signed 16-bit
//    * number in 2’s complement format in each 2 registers. From the registers, the gyro values can be
//    * calculated as follows:
//    * Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
//    * Rate_Y: RATE_Y_MSB * 256 +
//    * Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB
//    */
//
//    union
//    {
//        int16_t value;
//        uint8_t bytes[sizeof( int16_t )];
//    } container;
//
//    container.value = gyro_value;
//    int16_t actual_value = to_int16_t( container.bytes );

    const int16_t range = pow (2, ( GYRO_RANGE + 1 ) ) * 1.5;
    const float result = (float) gyro_value / 32768  * range;
    return result;
}

static float imu_sensor_deg_per_sec2rot     ( float deg_per_sec )
{
    return (int16_t ) ( deg_per_sec * 32768 ) / ( pow (2, (GYRO_RANGE + 1) ) * 1.5 );
}



int imu_sensor_init( FlightSystemConfiguration * parameters )
{

    int status = spi3_init( );
    if ( status != SPI_OK)
    {
        return IMU_ERR;
    }

    s_queue = xQueueCreate( 10, sizeof( imu_sensor_data ) );
    if ( s_queue == NULL )
    {
        return IMU_ERR;
    }

    vQueueAddToRegistry( s_queue, "bmi088_queue" );

    return IMU_OK;
}



void prv_imu_thread_start( void * param )
{
    if(param != NULL)
    {
        FlightSystemConfiguration * systemConfiguration = ( FlightSystemConfiguration * ) param;
        dataNeedsToBeConverted = systemConfiguration->imu_data_needs_to_converted;
    }

    //Get the parameters.
    TickType_t prevTime;
    imu_sensor_data dataStruct;

    // main loop: continuously read sensor data
    // vTaskDelay(pdMS_TO_TICKS(100)); //Wait so to make sure the other tasks have started.


    bool result_flag;
    xyz_data cxx_data;

    uint32_t time_start = 0;

    while ( 1 )
    {

        result_flag = datafeeder_get_acc( &cxx_data );
        if ( !result_flag )
        {
            continue;
        }

        dataStruct.acc_x = cxx_data.x;
        dataStruct.acc_y = cxx_data.y;
        dataStruct.acc_z = cxx_data.z;

        result_flag = datafeeder_get_gyro( &cxx_data );
        if ( !result_flag )
        {
            continue;
        }

        dataStruct.gyro_x = cxx_data.x;
        dataStruct.gyro_y = cxx_data.y;
        dataStruct.gyro_z = cxx_data.z;


        dataStruct.timestamp   = cxx_data.timestamp;

//        dataStruct.time_ticks   = time_start - xTaskGetTickCount();
//        DISPLAY( "NEW IMU data: %d\n", dataStruct.time_ticks);

        if(dataNeedsToBeConverted)
        {
            dataStruct.acc_x = imu_sensor_acc2g(dataStruct.acc_x);
            dataStruct.acc_y = imu_sensor_acc2g(dataStruct.acc_y);
            dataStruct.acc_z = imu_sensor_acc2g(dataStruct.acc_z);

            dataStruct.gyro_x = imu_sensor_rot2deg_per_sec(dataStruct.gyro_x);
            dataStruct.gyro_y = imu_sensor_rot2deg_per_sec(dataStruct.gyro_y);
            dataStruct.gyro_z = imu_sensor_rot2deg_per_sec(dataStruct.gyro_z);
        }

        imu_add_measurement( &dataStruct );
    }
}



void imu_sensor_start ( void * param )
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

    if ( pdFALSE == xTaskCreate ( prv_imu_thread_start, "imu-task", configMINIMAL_STACK_SIZE, param, 5, &handle ) )
    {
        board_error_handler( __FILE__, __LINE__ );
    }
}



bool imu_read ( imu_sensor_data * buffer )
{
    return pdPASS == xQueueReceive( s_queue, buffer, 0 );
}


void delay_ms( uint32_t period )
{
    vTaskDelay ( pdMS_TO_TICKS( period ) ); // wait for the given amount of milliseconds
}



int imu_sensor_test( )
{
    return IMU_ERR;
}



bool imu_add_measurement( imu_sensor_data * _data )
{
    return pdTRUE == xQueueSend ( s_queue, _data, 0 );
}

