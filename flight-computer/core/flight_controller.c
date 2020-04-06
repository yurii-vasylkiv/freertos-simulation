#include "flight_controller.h"
#include <FreeRTOS.h>
#include <task.h>

#include "stm32/STM32.h"
#include "memory-management/memory_manager.h"
#include "event-detection/event_detector.h"
#include "UserConfig.h"


#if (userconf_FREE_RTOS_SIMULATOR_MODE_ON == 1)
#include "sensor-simulation/imu_sensor.h"
#include "sensor-simulation/pressure_sensor.h"
#include "sensor-simulation/datafeeder.h"
#else
#include "tasks/sensors/imu_sensor.h"
    #include "tasks/sensors/pressure_sensor.h"
    #include "tasks/sensors/datafeeder.h"
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
int flight_controller_initialize(void * pvParams)
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

    if ( pdFALSE == xTaskCreate( prv_flight_controller_task, "flight-controller", configMINIMAL_STACK_SIZE, controller.taskParameters, 5, &controller.taskHandle ) )
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
    uint32_t status = memory_manager_get_system_configurations( &system_configurations );
    if ( status != MEM_OK )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "System configurations have been extracted", NULL );
    }

    static MemoryManagerConfiguration memory_configurations;
    status = memory_manager_get_memory_configurations( &memory_configurations );
    if ( status != MEM_OK)
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "Memory configurations have been extracted", NULL );
    }

    pressure_sensor_data initialGroundPressureData;
    while ( ! pressure_sensor_read ( &initialGroundPressureData ) );

    system_configurations.ground_pressure = initialGroundPressureData.pressure;
    system_configurations.flight_state = FLIGHT_STATE_LAUNCHPAD;
    system_configurations.current_system_time = xTaskGetTickCount();
    system_configurations.power_mode = 1;

    status = memory_manager_set_system_configurations(&system_configurations);
    if ( status != MEM_OK )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "Memory configurations have been updated", NULL );
    }

    status = event_detector_init( &system_configurations );
    if ( status != 0 )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "Event Detector has been set & configured ", NULL );
    }

    status = flight_state_machine_init( FLIGHT_STATE_LAUNCHPAD );
    if ( status != 0 )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "Flight State Machine has been set", NULL );
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

        if ( ( xTaskGetTickCount() - last_time ) / configTICK_RATE_HZ == 1 )
        {
            seconds++;
            DISPLAY( "Flight Time: %d sec \n", seconds);
            last_time = (xTaskGetTickCount() - start_time);
        }
    }

}



static void get_sensor_data_update( Data * data )
{
    imu_sensor_data imu_data;
    pressure_sensor_data pressure_data;

    if ( imu_read ( &imu_data ) )
    {
//        data->inertial.data.timestamp   = xTaskGetTickCount();
        data->inertial.data.timestamp               = imu_data.timestamp;
        memcpy( &data->inertial.data.accelerometer, &imu_data.acc_x,  sizeof( float ) * 3 );
        memcpy( &data->inertial.data.gyroscope,     &imu_data.gyro_x, sizeof( float ) * 3 );
        data->inertial.updated = true;
    }

    if ( pressure_sensor_read ( &pressure_data ) )
    {
//        data->pressure.data.timestamp   = xTaskGetTickCount();
        data->pressure.data.timestamp   = pressure_data.timestamp;
        data->pressure.data.pressure    = pressure_data.pressure;
        data->pressure.data.temperature = pressure_data.temperature;
        data->pressure.updated          = true;

//        DISPLAY("Pressure measurement at %d ticks \n", data->pressure.data.timestamp);
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
        return 1;
    }

    if ( !isInitialized )
    {
        return 1;
    }

    // Check to make sure that the state is being entered is valid
    if ( sm_state < FLIGHT_STATE_COUNT )
    {
        return state_machine[ state ].function( parameters );
    } else
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




















