#include"datafeeder.h"
#include <string>
#include <memory>
#include <cassert>
#include <pthread.h>
#include <deque>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <time.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>

#include "csv.h"

namespace
{
    const int MAX_ITEMS             = 100;

    pthread_t                       worker;

    std::deque < xyz_data >         gyro_queue;
    std::deque < xyz_data >         acc_queue;
    std::deque < press_data >       press_queue;
    std::string                     csv_file_name;

    xTaskHandle                     handle;

    int msleep( long msec )
    {
        struct timespec ts { };
        int res;

        if ( msec < 0 )
        {
            errno = EINVAL;
            return -1;
        }

        ts.tv_sec = msec / 1000;
        ts.tv_nsec = ( msec % 1000 ) * 1000000;

        do
        {
            res = nanosleep( &ts, &ts );
        } while ( res && errno == EINTR );

        return res;
    }

}

#ifdef __cplusplus
extern "C" {
#endif


uint32_t timestamp_uint;
void * worker_function( void * arg )
{
    double timestamp { };
    xyz_data gyro, acc;
    press_data press;
    double alt;
    uint32_t ev;

    io::CSVReader<9> reader (csv_file_name);

    // time,accx,accy,accz,rotx,roty,rotz,temp,pres,alt,Flags

    // ignore the first row
    reader.read_row(timestamp, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, press.temperature, press.pressure);


    while (reader.read_row(timestamp, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, press.temperature, press.pressure))
    {
        timestamp_uint += 50;
        acc.timestamp   = timestamp_uint;
        gyro.timestamp  = timestamp_uint;;
        press.timestamp = timestamp_uint;;

        taskENTER_CRITICAL( );

        if ( acc_queue.size( ) >= MAX_ITEMS )
            acc_queue.pop_front( );
        acc_queue.push_back( acc );

        if ( gyro_queue.size( ) >= MAX_ITEMS )
            gyro_queue.pop_front( );
        gyro_queue.push_back( gyro );

        if ( press_queue.size( ) >= MAX_ITEMS )
            press_queue.pop_front( );
        press_queue.push_back( press );

        taskEXIT_CRITICAL( );

        msleep( 3 );
    }


    return nullptr;
}

#ifdef __cplusplus
}
#endif



void prv_task_fnc( void * pvParams )
{
    try
    {
        if ( pthread_create( &worker, nullptr, worker_function, nullptr ) )
        {
            fprintf( stderr, "Error creating thread\n" );
            return;
        }
    }
    catch ( const char * s )
    {
        printf( "Error: %s", s );
        return;
    }
}



int data_feeder_start( const char * file )
{
    csv_file_name = std::string( file, strlen( file ) );
    if ( pdFALSE == xTaskCreate( prv_task_fnc, "fake-sensor-data", configMINIMAL_STACK_SIZE, nullptr, 5, &handle ) )
    {
        return 1;
    }

    return 0;
}



int datafeeder_get_gyro( xyz_data * data )
{
    if ( !gyro_queue.empty( ) )
    {
        taskENTER_CRITICAL( );

        auto _data = &gyro_queue.front( );
        data->timestamp = _data->timestamp;
        data->x = _data->x;
        data->y = _data->y;
        data->z = _data->z;
        gyro_queue.pop_front( );

        taskEXIT_CRITICAL( );

        return 1;
    }

    return 0;
}



int datafeeder_get_acc( xyz_data * data )
{
    if ( !acc_queue.empty( ) )
    {
        taskENTER_CRITICAL( );

        auto _data = &acc_queue.front( );
        data->timestamp = _data->timestamp;
        data->x = _data->x;
        data->y = _data->y;
        data->z = _data->z;
        acc_queue.pop_front( );

        taskEXIT_CRITICAL( );

        return 1;
    }

    return 0;
}



int datafeeder_get_press( press_data * data )
{
    if ( !press_queue.empty( ) )
    {
        taskENTER_CRITICAL( );

        auto _data = &press_queue.front( );
        data->timestamp = _data->timestamp;
        data->temperature = _data->temperature;
        data->pressure = _data->pressure;
        press_queue.pop_front( );

        taskEXIT_CRITICAL( );

        return 1;
    }

    return 0;
}



void data_feeder_join( )
{
    pthread_join( worker, nullptr );
}


