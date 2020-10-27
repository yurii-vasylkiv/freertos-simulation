#include "flight_controller.h"
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>

#include "board/board.h"
#include "memory-management/memory_manager.h"
#include "event-detection/event_detector.h"
#include "configurations/UserConfig.h"

#include "board/components/imu_sensor.h"
#include "board/components/pressure_sensor.h"
#include "utilities/common.h"



#if (userconf_FREE_RTOS_SIMULATOR_MODE_ON == 1)
#include "sim-port/sensor-simulation/datafeeder.h"
#endif


typedef struct
{
    uint8_t isInitialized       ;
    uint8_t isRunning           ;

    xTaskHandle taskHandle      ;
    void * taskParameters       ;
}FlightControllerState;

static FlightControllerState controller     = {};


static void prv_flight_controller_task(void * pvParams);
static void get_sensor_data_update(Data * data);
int flight_controller_init(void * pvParams)
{
    controller.taskParameters = pvParams;
    controller.isInitialized = 1;

    return FLIGHT_CONTROLLER_OK;
}


int flight_controller_start()
{
    if ( !controller.isInitialized )
    {
        return FLIGHT_CONTROLLER_ERR;
    }

    if ( pdFALSE == xTaskCreate( prv_flight_controller_task, "fl--manager", configMINIMAL_STACK_SIZE, controller.taskParameters, 5, &controller.taskHandle ) )
    {
        return FLIGHT_CONTROLLER_ERR;
    }

    return FLIGHT_CONTROLLER_OK;
}

void flight_controller_stop()
{
    controller.isRunning = 0;
}



static void prv_flight_controller_task(void * pvParams)
{
    (void) pvParams;

    static FlightSystemConfiguration system_configurations;
    static FlightSystemConfigurationU systemConfigurationsU;
    static MemoryManagerConfiguration memoryConfigurations;
    static MemoryManagerConfigurationU memoryConfigurationsU;

    controller.isRunning = 1;


    uint32_t status = memory_manager_get_system_configurations( &system_configurations );
    if ( status != MEM_OK )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "System configurations have been extracted.");
    }

    systemConfigurationsU.values = system_configurations;
    if(common_is_mem_not_set(systemConfigurationsU.bytes, sizeof(FlightSystemConfiguration)))
    {
        // if memory read returned empty configurations
        system_configurations.imu_sensor_configuration      = imu_sensor_get_default_configuration();
        system_configurations.pressure_sensor_configuration = pressure_sensor_get_default_configuration();
    }

    status = memory_manager_get_memory_configurations( &memoryConfigurations );
    if ( status != MEM_OK)
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "Memory configurations have been extracted.");
    }

    memoryConfigurationsU.values = memoryConfigurations;
    if(common_is_mem_not_set(systemConfigurationsU.bytes, sizeof(MemoryManagerConfiguration)))
    {
        // if memory read returned empty configurations
        memoryConfigurations = memory_manager_get_default_memory_configurations();
    }

#if (userconf_FREE_RTOS_SIMULATOR_MODE_ON == 1)
#if (userconf_USE_COTS_DATA == 0)
    status = memory_manager_get_system_configurations( &system_configurations );
    if ( status != MEM_OK )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "System configurations have been extracted");
    }

    systemConfigurationsU.values = system_configurations;
    if(common_is_mem_not_set(systemConfigurationsU.bytes, sizeof(FlightSystemConfiguration)))
    {
        // if memory read returned empty configurations
        system_configurations.imu_sensor_configuration      = imu_sensor_get_default_configuration();
        system_configurations.pressure_sensor_configuration = pressure_sensor_get_default_configuration();
    }

    system_configurations.imu_data_needs_to_be_converted       = 1;
    system_configurations.pressure_data_needs_to_be_converted  = 1;

    status = memory_manager_set_system_configurations( &system_configurations );
    if ( status != MEM_OK )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "System configurations have been extracted");
    }

    memoryConfigurationsU.values = memoryConfigurations;
    if(common_is_mem_not_set(systemConfigurationsU.bytes, sizeof(MemoryManagerConfiguration)))
    {
        // if memory read returned empty configurations
        memoryConfigurations = memory_manager_get_default_memory_configurations();
    }

    imu_sensor_start      ( &system_configurations );
    pressure_sensor_start ( &system_configurations );

#else
    if ( 0 != imu_sensor_configure(&system_configurations.imu_sensor_configuration) )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "IMU sensor has been configured.");
    }

    if ( 0 != pressure_sensor_configure(&system_configurations.pressure_sensor_configuration) )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "Pressure sensor has been configured.");
    }

//     imu_sensor_start      ( NULL );
//     pressure_sensor_start ( NULL );
#endif
#else
    system_configurations.imu_data_needs_to_be_converted       = 1;
    system_configurations.pressure_data_needs_to_be_converted  = 1;

    if(0 != imu_sensor_configure(&system_configurations.imu_sensor_configuration))
    {
        board_error_handler( __FILE__, __LINE__ );
    }
    else
    {
        DEBUG_LINE( "IMU sensor has been configured.");
    }

    if(0 != pressure_sensor_configure(&system_configurations.pressure_sensor_configuration))
    {
        board_error_handler( __FILE__, __LINE__ );
    }
    else
    {
        DEBUG_LINE( "Pressure sensor has been configured.");
    }

    imu_sensor_start      ( &system_configurations );
    pressure_sensor_start ( &system_configurations );
#endif


    status = event_detector_init( &system_configurations );
    if ( status != 0 )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "Event Detector has been set & configured.");
    }

    // in case of reboot during the flight if memory is not corrupted event_detector_init should change this flag to the
    // appropriate stage that != FLIGHT_STATE_LAUNCHPAD
    if( ! event_detector_is_flight_started () )
    {
        // this will only be triggered in case if we are on the ground before the flight

        // # 1 set the ground pressure and temperature as references for the future calculations
        DEBUG_LINE( "Flight Controller: waiting for the ground pressure & temperature...");
        PressureSensorData initialGroundPressureData;
        while (!pressure_sensor_read(&initialGroundPressureData));
        DEBUG_LINE( "Flight Controller: ground pressure & temperature have been set!");

        system_configurations.ground_pressure = initialGroundPressureData.pressure;
        system_configurations.ground_temperature = initialGroundPressureData.temperature;

        // # 1 set the flight state to launchpad, grab the current tick count that is ~ 0;
        // set the starting power mode that is supposed to be low power mode
        system_configurations.flight_state = FLIGHT_STATE_LAUNCHPAD;
        system_configurations.current_system_time = xTaskGetTickCount();
        system_configurations.power_mode = 1;

        status = memory_manager_set_system_configurations(&system_configurations);
        if (status != MEM_OK)
        {
            board_error_handler(__FILE__, __LINE__);
        }
        else
        {
            DEBUG_LINE("Memory configurations have been updated.");
        }

        // write these configurations to the memory
        event_detector_update_configurations(&system_configurations);
    }


    status = flight_state_machine_init( FLIGHT_STATE_LAUNCHPAD );
    if ( status != 0 )
    {
        board_error_handler( __FILE__, __LINE__ );
    } else
    {
        DEBUG_LINE( "Flight State Machine has been set.");
    }

    static Data flightData;
    static FlightState flightState;
    static FlightStateMachineTickParameters smParams;

    controller.isRunning    = 1;

    int start_time  = 0;
    int last_time   = 0;
    int seconds     = 0;
    while ( controller.isRunning )
    {
//        flightData.timestamp = xTaskGetTickCount ( ) - start_time;

        get_sensor_data_update(&flightData);

        flightState = event_detector_feed(&flightData);

        flight_state_machine_tick(flightState, &smParams);

        memory_manager_update(&flightData);

        memset(&flightData, 0, sizeof(flightData));

        if ( ( xTaskGetTickCount() - last_time ) / configTICK_RATE_HZ >= 1 )
        {
            seconds++;
//            DEBUG_LINE( "Flight Time: %d sec", seconds);
            last_time = (xTaskGetTickCount() - start_time);
//            DEBUG_LINE("CURRENT_ALTITUDE = %f", event_detector_current_altitude())
        }
    }

}



static void get_sensor_data_update( Data * data )
{
    IMUSensorData imu_data;
    PressureSensorData pressure_data;

    if ( imu_read ( &imu_data ) )
    {
        data->inertial.data.timestamp = imu_data.timestamp;
        memcpy( &data->inertial.data.accelerometer, &imu_data.acc_x,  sizeof( float ) * 3 );
        memcpy( &data->inertial.data.gyroscope,     &imu_data.gyro_x, sizeof( float ) * 3 );
        data->inertial.updated = true;
    }

    if ( pressure_sensor_read ( &pressure_data ) )
    {
        data->pressure.data.timestamp   = pressure_data.timestamp;
        data->pressure.data.pressure    = pressure_data.pressure;
        data->pressure.data.temperature = pressure_data.temperature;
        data->pressure.updated          = true;
    }
}





//....................................................................................................................//
//......................................State Machine Implementation..................................................//
//....................................................................................................................//

typedef struct
{
    FlightState state;
    int (*function)(FlightStateMachineTickParameters*);
} state_machine_type;

uint8_t isInitialized = 0;

int sm_STATE_LAUNCHPAD   (FlightStateMachineTickParameters*);
int sm_STATE_PRE_APOGEE  (FlightStateMachineTickParameters*);
int sm_STATE_APOGEE      (FlightStateMachineTickParameters*);
int sm_STATE_POST_APOGEE (FlightStateMachineTickParameters*);
int sm_STATE_MAIN_CHUTE  (FlightStateMachineTickParameters*);
int sm_STATE_POST_MAIN   (FlightStateMachineTickParameters*);
int sm_STATE_LANDED      (FlightStateMachineTickParameters*);
int sm_STATE_EXIT        (FlightStateMachineTickParameters*);

state_machine_type state_machine[] =
{
        {FLIGHT_STATE_LAUNCHPAD,        sm_STATE_LAUNCHPAD     },
        {FLIGHT_STATE_PRE_APOGEE,       sm_STATE_PRE_APOGEE    },
        {FLIGHT_STATE_APOGEE,           sm_STATE_APOGEE        },
        {FLIGHT_STATE_POST_APOGEE,      sm_STATE_POST_APOGEE   },
        {FLIGHT_STATE_MAIN_CHUTE,       sm_STATE_MAIN_CHUTE    },
        {FLIGHT_STATE_POST_MAIN,        sm_STATE_POST_MAIN     },
        {FLIGHT_STATE_LANDED,           sm_STATE_LANDED        },
        {FLIGHT_STATE_EXIT,             sm_STATE_EXIT          }
};

static FlightState sm_state;

int flight_state_machine_init ( FlightState state )
{
    sm_state = state;

    isInitialized = 1;

    return 0;
}



int flight_state_machine_tick( FlightState state, FlightStateMachineTickParameters * parameters )
{
    if ( parameters == NULL )
    {
        DISPLAY_LINE("CANNOT return flight state. Flight Controller has not been initialized.");
        return 1;
    }

    if ( !isInitialized )
    {
        DISPLAY_LINE("Flight Controller has been already initialized.");
        return 1;
    }

    // Check to make sure that the state is being entered is valid
    if ( sm_state < FLIGHT_STATE_COUNT )
    {
        return state_machine [ state ].function ( parameters );
    }
    else
    {
        // Throw an exception
        return 1;
    }
}


int sm_STATE_LAUNCHPAD(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_PRE_APOGEE(FlightStateMachineTickParameters * parameters)
{
    return 0;
}

int sm_STATE_APOGEE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_POST_APOGEE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_MAIN_CHUTE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_POST_MAIN(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_LANDED(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_EXIT(FlightStateMachineTickParameters * parameters)
{

    return 0;
}




















