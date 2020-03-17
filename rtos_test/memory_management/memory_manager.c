#include "memory_manager.h"

#include <stdio.h>
#include <memory.h>
#include <string.h>
#include "queue.h"
#include <assert.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include <protocols/UART.h>
#include "flash.h"


static const char * MAGIC = "e10e720-6fb8-4fc0-8807-6e2201ac6e0d";
#define MAGIC_BUFFER_LENGTH                             36
#define CONFIGURATION_SECTOR_BASE_ADDRESS               0
#define CONFIGURATION_SECTOR_OFFSET                     CONFIGURATION_SECTOR_BASE_ADDRESS + sizeof(ConfigurationU)
#define DATA_SECTOR_SIZE                                (1024 /* KB */ * 1024 /* MB */ * 10)
#define CONFIGURATION_AUTOSAVE_INTERVAL                 200
#define PAGE_SIZE                                       256
#define IMU_ALIGNED_PAGE_SIZE                           trunc( (PAGE_SIZE / sizeof ( IMUDataU  ) ) *  sizeof ( IMUDataU ) )
#define PRESS_ALIGNED_PAGE_SIZE                         trunc( (PAGE_SIZE / sizeof ( PressureU ) ) *  sizeof ( IMUDataU ) )

#define IMU_MEASUREMENTS_PER_PAGE                       ( (int) (PAGE_SIZE / sizeof (IMUDataU) ) )
#define PRESS_MEASUREMENTS_PER_PAGE                     ( (int) (PAGE_SIZE / sizeof (PressureDataU) ) )


static ConfigurationU configurations                    = { };
static MemoryConfigurationSector confSector             = { };
static MemoryDataSector dataSectors[SectorsCount]       = { };
static buffer_queue s_page_cb                           = { };
static uint8_t is_queue_monitor_running                 = { 0 };
static uint8_t s_is_initialized                         = { 0 };
static size_t s_configurations_autosave_interval_tick   = { 0 };
static xTaskHandle handle                               = { 0 };



static int _add_data( Sector sector, uint8_t * buffer, size_t size );

static void _write_async_to_flash_if_anything_is_available( size_t size );

static void _queue_monitor_task( void * arg );

static int prv_update_configurations( );

static int _read_configurations_from_flash( ConfigurationU * conf_container );

static int _access_page( Sector sector, uint16_t pageIndex, uint8_t * dest );

static void _access_page_of_imu_measurements( uint32_t pageIndex );

static void _access_page_of_pressure_measurements( uint32_t pageIndex );

static void _access_single_pressure_measurement( uint32_t measurement_index );

static void _access_single_imu_measurement( uint32_t measurement_index );

static int _access_page( Sector sector, uint16_t pageIndex, uint8_t * dest );

static void _append_page( Sector sector, uint8_t * data );






int memory_manager_init( )
{
    for ( Sector sector = IMU ; sector < SectorsCount ; sector++ )
    {
        if ( dataSectors[ sector ].write == NULL )
        {
            dataSectors[ sector ].write = &dataSectors[ sector ].buffers[ 0 ];
        }

        if ( dataSectors[ sector ].read == NULL )
        {
            dataSectors[ sector ].read = &dataSectors[ sector ].buffers[ 1 ];
        }
    }

    buffer_queue_init( &s_page_cb);

    s_is_initialized = 1;

    return 0;
}



int memory_manager_configure( )
{
    if ( !s_is_initialized )
    {
        return 1;
    }

    if ( _read_configurations_from_flash( &configurations ) != 0 )
    {
        return 1;
    }

    if ( memcmp( configurations.magic, MAGIC, MAGIC_BUFFER_LENGTH ) == 0 ) // magic sequences match)
    {
        // Configuration is correct to read
        for ( size_t sector = IMU ; sector < SectorsCount ; sector++ )
        {
            dataSectors[ sector ].info = configurations.data_sectors[ sector ];
        }
    }
    else
    {
        // configuration magic-number is missing

        // fresh start
        for ( size_t sector = IMU ; sector < SectorsCount ; sector++ )
        {
            dataSectors[ sector ].info.size = DATA_SECTOR_SIZE;

            dataSectors[ sector ].info.startAddress = CONFIGURATION_SECTOR_OFFSET + DATA_SECTOR_SIZE * sector;

            dataSectors[ sector ].info.endAddress =
                    dataSectors[ sector ].info.startAddress + dataSectors[ sector ].info.size;

            dataSectors[ sector ].info.bytesWritten = 0;

            configurations.data_sectors[ sector ] = dataSectors[ sector ].info;
        }

        // storing the magic number
        memcpy( configurations.magic, MAGIC, MAGIC_BUFFER_LENGTH );
    }

    return 0;
}



int memory_manager_get_configurations( ConfigurationU * pConfigs )
{
    // validation
    if ( memcmp( configurations.magic, MAGIC, 36 ) == 0 ) // magic sequences match)
    {
        pConfigs = &configurations;
        return 0;
    }

    pConfigs = NULL;

    ( void ) pConfigs;

    return 1;
}



int memory_manager_update_sensors_ground_data( GroundDataU * _data )
{
    if ( _data == NULL )
    {
        return 1;
    }

    configurations.system.ground_pressure = _data->pressure;

    prv_update_configurations( );
    return 0;
}



int _read_configurations_from_flash( ConfigurationU * conf_container )
{
    assert( sizeof( ConfigurationU ) < PAGE_SIZE );

    if ( conf_container == NULL )
    {
        return 1;
    }

    int bytesLeftToWrite = sizeof( ConfigurationU );
    int configuration_pages = ( int ) trunc( ( double ) bytesLeftToWrite / PAGE_SIZE );

    if ( configuration_pages == 0 )
    {
        MemoryBuffer page = { };
        if ( _access_page( Conf, 0, page.data ) != 0 )
        {
            return 1;
        }

        memmove( conf_container->bytes, page.data, bytesLeftToWrite );
        return 0;
    }

    int bytesWritten = 0;
    MemoryBuffer page;
    while ( bytesLeftToWrite > PAGE_SIZE )
    {
        memset( &page, 0, sizeof( MemoryBuffer ) );
        if ( _access_page( Conf, 0, page.data ) != 0 )
        {
            return 1;
        }

        memmove( &conf_container->bytes[ bytesWritten ], page.data, PAGE_SIZE );
        bytesLeftToWrite -= PAGE_SIZE;
        bytesWritten += PAGE_SIZE;
    }


    memset( &page, 0, sizeof( MemoryBuffer ) );
    if ( _access_page( Conf, 0, page.data ) != 0 )
    {
        return 1;
    }

    memmove( conf_container->bytes, page.data, bytesLeftToWrite );
    return 0;
}



int prv_update_configurations( )
{
    assert( sizeof( ConfigurationU ) < PAGE_SIZE );

    buffer_item item = { };
    item.type = Conf;
    memmove( item.data, configurations.bytes, sizeof( ConfigurationU ) );
    buffer_queue_push_back( &s_page_cb, &item );
//    xTaskGenericNotify(handle, 0, eNoAction, NULL);
    return 0;
}



int memory_manager_update( Data * _container )
{
    if ( !s_is_initialized )
    {
        return 1;
    }

    if ( _container == NULL )
    {
        return 1;
    }

    if ( _container->inertial.updated )
    {
        memory_manager_add_imu_update( &_container->inertial.data );
        _container->inertial.updated = 0;
    }

    if ( _container->pressure.updated )
    {
        memory_manager_add_pressure_update( &_container->pressure.data );
        _container->pressure.updated = 0;
    }

    if ( _container->continuity.updated )
    {
        memory_manager_add_continuity_update( &_container->continuity.data );
        _container->continuity.updated = 0;
    }

    if ( _container->event.updated )
    {
        memory_manager_add_flight_event_update( &_container->event.data );
        _container->event.updated = 0;
    }

    return 0;
}



int memory_manager_add_imu_update( IMUDataU * _container )
{
    return _add_data( IMU, _container->bytes, sizeof( IMUDataU ) );
}



int memory_manager_add_pressure_update( PressureDataU * _container )
{
    return _add_data( Pressure, _container->bytes, sizeof( PressureDataU ) );
}



int memory_manager_add_continuity_update( ContinuityU * _container )
{

    return _add_data( Cont, _container->bytes, sizeof( ContinuityU ) );
}



int memory_manager_add_flight_event_update( FlightEventU * _container )
{

    return _add_data( Event, _container->bytes, sizeof( FlightEventU ) );
}



int memory_manager_start( )
{
    if ( !s_is_initialized )
    {
        return 1;
    }

    if ( pdFALSE == xTaskCreate( _queue_monitor_task, "memory-manager", configMINIMAL_STACK_SIZE, NULL, 5, &handle ) )
    {
        return 1;
    }

    return 0;
}



int memory_manager_stop( )
{
    is_queue_monitor_running = 0;
    return 0;
}

int memory_manager_get_system_configurations(FlightSystemConfiguration * systemConfiguration)
{
    if(systemConfiguration == NULL)
    {
        return 1;
    }

    memcpy(systemConfiguration, &configurations.system, sizeof( FlightSystemConfiguration ));

    return 0;
}

int memory_manager_get_memory_configurations(MemoryManagerConfiguration * memoryConfiguration)
{
    if(memoryConfiguration == NULL)
    {
        return 1;
    }

    memcpy(memoryConfiguration, &configurations.system, sizeof( FlightSystemConfiguration ));

    return 0;
}



int memory_manager_set_system_configurations(FlightSystemConfiguration * systemConfiguration)
{
    if(systemConfiguration == NULL)
    {
        return 1;
    }

    memset(&configurations.system, 0, sizeof( FlightSystemConfiguration ));
    configurations.system = *systemConfiguration;

    prv_update_configurations( );
    return 0;
}

int memory_manager_set_memory_configurations(MemoryManagerConfiguration * memoryConfiguration)
{
    if(memoryConfiguration == NULL)
    {
        return 1;
    }

    memset(&configurations.memory, 0, sizeof( MemoryManagerConfiguration ));
    configurations.memory = *memoryConfiguration;

    prv_update_configurations( );

    return 0;
}



int _add_data( Sector sector, uint8_t * buffer, size_t size )
{
    if ( !s_is_initialized )
    {
        return 1;
    }

    const size_t ALIGNED_PAGE_SIZE = trunc( (PAGE_SIZE /  ( size  ) ) *  ( size ) );

    if ( dataSectors[ sector ].write->info.bytesWritten >=  ALIGNED_PAGE_SIZE )
    {
        MemoryBuffer * temp_read = dataSectors[ sector ].read;
        dataSectors[ sector ].read = dataSectors[ sector ].write;
        dataSectors[ sector ].write = temp_read;
        _write_async_to_flash_if_anything_is_available( size );

        // safeguard check
        if ( dataSectors[ sector ].write->info.bytesWritten >= ALIGNED_PAGE_SIZE )
        {
            printf( "The page was not saved. Probably the monitor has stuck, or something went wrong. Overwriting the page..." );
            dataSectors[ sector ].write->info.bytesWritten = 0;
        }
    }

    MemoryBuffer * currentBuffer = dataSectors[ sector ].write;
    uint8_t * currWriteBufferPosition = &currentBuffer->data[ currentBuffer->info.bytesWritten ];
    memcpy( currWriteBufferPosition, buffer, size );
    currentBuffer->info.bytesWritten += size;


    s_configurations_autosave_interval_tick++;
    if ( s_configurations_autosave_interval_tick >= CONFIGURATION_AUTOSAVE_INTERVAL )
    {
        // it is time to update the configurations
        prv_update_configurations( );
        // reset the timer
        s_configurations_autosave_interval_tick = 0;
    }

    return 0;
}



void _write_async_to_flash_if_anything_is_available( size_t size )
{
    const size_t ALIGNED_PAGE_SIZE = trunc( (PAGE_SIZE /  ( size  ) ) *  ( size ) );

    // free the read page
    for ( size_t sector = IMU ; sector < SectorsCount ; sector++ )
    {
        if ( dataSectors[ sector ].read->info.bytesWritten >= ALIGNED_PAGE_SIZE )
        {
            buffer_item item = { };
            item.type = sector;
            memmove( item.data, dataSectors[ sector ].read->data, PAGE_SIZE );
            dataSectors[ sector ].read->info.bytesWritten = 0;
            buffer_queue_push_back( &s_page_cb, &item );
//            xTaskGenericNotify(handle, 0, eNoAction, NULL);
        }
    }
}



void _append_page( Sector sector, uint8_t * data )
{
    if ( sector == Conf )
    {
        const uint32_t OFFSET = confSector.info.startAddress + confSector.info.bytesWritten;

        uint32_t bytesWritten = flash_write( OFFSET, data, PAGE_SIZE );

        assert( bytesWritten == PAGE_SIZE );
    } else
    {
        const uint32_t OFFSET = dataSectors[ sector ].info.startAddress + dataSectors[ sector ].info.bytesWritten;

        uint32_t bytesWritten = flash_write( OFFSET, data, PAGE_SIZE );

        assert( bytesWritten == PAGE_SIZE );

        dataSectors[ sector ].info.bytesWritten += PAGE_SIZE;

        configurations.data_sectors[ sector ].bytesWritten = dataSectors[ sector ].info.bytesWritten;
    }
}



static int _access_page( Sector sector, uint16_t pageIndex, uint8_t * dest )
{
    uint32_t OFFSET;
    if ( sector != Conf )
    {
        assert( PAGE_SIZE * pageIndex <= dataSectors[ sector ].info.bytesWritten );

        OFFSET = dataSectors[ sector ].info.startAddress + pageIndex * PAGE_SIZE;
    } else
    {
        assert( PAGE_SIZE * pageIndex <= confSector.info.bytesWritten );

        OFFSET = confSector.info.startAddress + pageIndex * PAGE_SIZE;
    }

    int32_t bytesRead = flash_read( OFFSET, dest, PAGE_SIZE );

    assert( bytesRead == PAGE_SIZE );

    return 0;
}



void _access_single_imu_measurement( uint32_t measurement_index )
{
    assert( PAGE_SIZE * trunc( ( double ) measurement_index / IMU_MEASUREMENTS_PER_PAGE ) <=
            dataSectors[ IMU ].info.bytesWritten );

    MemoryBuffer buffer;

    uint32_t pageIndex = ceil( ( double ) measurement_index / IMU_MEASUREMENTS_PER_PAGE );

    _access_page( IMU, pageIndex, buffer.data );

    measurement_index -= pageIndex * IMU_MEASUREMENTS_PER_PAGE;

    IMUDataU data;

    memcpy( &data, &buffer.data[ measurement_index ], sizeof( IMUDataU ) );
}



void _access_single_pressure_measurement( uint32_t measurement_index )
{
    assert( PAGE_SIZE * trunc( ( double ) measurement_index / PRESS_MEASUREMENTS_PER_PAGE ) <=
            dataSectors[ Pressure ].info.bytesWritten );

    MemoryBuffer buffer;

    uint32_t pageIndex = ceil( ( double ) measurement_index / PRESS_MEASUREMENTS_PER_PAGE );

    _access_page( Pressure, pageIndex, buffer.data );

    measurement_index -= pageIndex * PRESS_MEASUREMENTS_PER_PAGE;

    PressureDataU data;

    memcpy( &data, &buffer.data[ measurement_index ], sizeof( PressureDataU ) );
}



void _access_page_of_pressure_measurements( uint32_t pageIndex )
{
    assert( PAGE_SIZE * pageIndex <= dataSectors[ Pressure ].info.bytesWritten );

    MemoryBuffer buffer = { };

    PressureDataU pressureMeasurements[PRESS_MEASUREMENTS_PER_PAGE];

    _access_page( Pressure, pageIndex, buffer.data );

    memcpy( &pressureMeasurements[ 0 ], &buffer.data[ 0 ], sizeof( PressureDataU ) );

    for ( uint32_t i = 1 ; i < PRESS_MEASUREMENTS_PER_PAGE ; i++ )
    {
        memcpy( &pressureMeasurements[ i ], &buffer.data[ i * sizeof( PressureDataU ) ], sizeof( PressureDataU ) );
    }
}



void _access_page_of_imu_measurements( uint32_t pageIndex )
{
    assert( PAGE_SIZE * pageIndex <= dataSectors[ IMU ].info.bytesWritten );

    MemoryBuffer buffer;

    IMUDataU imuMeasurements[IMU_MEASUREMENTS_PER_PAGE] = { };

    _access_page( IMU, pageIndex, buffer.data );

    memcpy( &imuMeasurements[ 0 ], &buffer.data[ 0 ], sizeof( IMUDataU ) );

    for ( uint32_t i = 1 ; i < IMU_MEASUREMENTS_PER_PAGE ; i++ )
    {
        memcpy( &imuMeasurements[ i ], &buffer.data[ i * sizeof( IMUDataU ) ], sizeof( IMUDataU ) );
    }
}



void _queue_monitor_task( void * arg )
{
    ( void ) arg;

    is_queue_monitor_running = 1;
    buffer_item item = { };

    while ( is_queue_monitor_running )
    {
//        xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
//                        ULONG_MAX,  /* Reset the notification value to 0 on exit.  */
//                        NULL,      /* Notified value pass out in ulNotifiedValue. */
//                         250);

//        DISPLAY_LINE("monitor woke up!", NULL);

        while ( buffer_queue_pop_front( &s_page_cb, &item ) )
        {
            _append_page( item.type, item.data );

            switch ( item.type )
            {
                case IMU:
//                    printf( "Monitor: IMU was flushed!\n" );
                    break;
                case Pressure:
//                    printf( "Monitor: Pressure was flushed!\n" );
                    break;
                case Cont:
//                    printf( "Monitor: Cont was flushed!\n" );
                    break;
                case Event:
//                    printf( "Monitor: Event was flushed!\n" );
                    break;
                case Conf:
//                    printf( "Monitor: Configuration was flushed!\n" );
                    break;
                default:
//                    printf( "Monitor: Wrong type! Wtf?\n" );
                    break;
            }
        }
    }

    DISPLAY_LINE("monitor EXITED!", NULL);

}

