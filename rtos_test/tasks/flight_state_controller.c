#include "flight_state_controller.h"
#include <FreeRTOS.h>
#include <task.h>

#include "stm32/STM32.h"
#include "memory_management/memory_manager.h"
#include "tasks/sensors/pressure_sensor.h"
#include "tasks/sensors/imu_sensor.h"
#include "events/event_detector.h"
#include "flight_state_machine.h"





static uint8_t s_is_initialized                         = { 0 };
static uint8_t s_is_running                             = { 0 };

static xTaskHandle handle                               = { 0 };
static void * prv_xTaskParameters                       = { NULL };

static void prv_flight_controller_task(void * pvParams);
static void get_sensor_data_update(Data * data);
int flight_controller_initialize(void * pvParams)
{
    prv_xTaskParameters = pvParams;
    s_is_initialized = 1;

    return 0;
}


int flight_controller_start()
{
    if ( !s_is_initialized )
    {
        return 1;
    }

    if ( pdFALSE == xTaskCreate( prv_flight_controller_task, "flight-controller", configMINIMAL_STACK_SIZE, prv_xTaskParameters, 5, &handle ) )
    {
        return 1;
    }

    return 0;
}

void flight_controller_stop()
{
    s_is_running = 0;
}



static void prv_flight_controller_task(void * pvParams)
{
    (void) pvParams;

    static FlightSystemConfiguration system_configurations;
    uint32_t status = memory_manager_get_system_configurations( &system_configurations );
    if ( status != 0 )
    {
        stm32_error_handler( __FILE__, __LINE__ );
    } else
    {
        DISPLAY_LINE( "System configurations have been extracted", NULL );
    }

    static MemoryManagerConfiguration memory_configurations;
    status = memory_manager_get_memory_configurations( &memory_configurations );
    if ( status != 0 )
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
    if ( status != 0 )
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

    s_is_running = 1;

    int start_time = 0;
    int last_time = 0;
    int seconds = 0;
    while ( s_is_running )
    {
//        flightData.timestamp = xTaskGetTickCount ( ) - start_time;

        get_sensor_data_update(&flightData);

        flightState = event_detector_feed(&flightData);

        flight_state_machine_tick(flightState, &smParams);

        memory_manager_update(&flightData);

        memset(&flightData, 0, sizeof(flightData));

//        if ( ( xTaskGetTickCount() - last_time ) / configTICK_RATE_HZ == 1 )
//        {
//            seconds++;
//            DISPLAY( "Flight Time: %d sec \n", seconds);
//            last_time = (xTaskGetTickCount() - start_time);
//        }
    }

}



static void get_sensor_data_update( Data * data )
{
    imu_sensor_data imu_data;
    pressure_sensor_data pressure_data;

    if ( imu_read ( &imu_data ) )
    {
//        data->inertial.data.timestamp   = xTaskGetTickCount();
        data->inertial.data.timestamp               = imu_data.time_ticks;
        memcpy( &data->inertial.data.accelerometer, &imu_data.acc_x,  sizeof( int16_t ) * 3 );
        memcpy( &data->inertial.data.gyroscope,     &imu_data.gyro_x, sizeof( int16_t ) * 3 );
        data->inertial.updated = true;
    }

    if ( pressure_sensor_read ( &pressure_data ) )
    {
//        data->pressure.data.timestamp   = xTaskGetTickCount();
        data->pressure.data.timestamp   = pressure_data.time_ticks;
        data->pressure.data.pressure    = pressure_data.pressure;
        data->pressure.data.temperature = pressure_data.temperature;
        data->pressure.updated          = true;

//        DISPLAY("Pressure measurement at %d ticks \n", data->pressure.data.timestamp);
    }

}


























//#include <string.h>
//#include "flash.h"
//#include "sensors/pressure_sensor.h"
//#include "sensors/imu_sensor.h"
//#include "buzzer.h"
//#include "recovery.h"
//#include "configuration.h"
//#include "utilities/common.h"
//#include <FreeRTOS.h>
//#include <task.h>
//
//#define HEADER_SIZE 3
//
//typedef struct{
//    uint8_t data[HEADER_SIZE + ACC_LENGTH + GYRO_LENGTH + PRES_LENGTH + TEMP_LENGTH + ALT_LENGTH];
//}data_measurement;
//
//typedef struct
//{
//    uint8_t                 acc_z_filter_index0;
//    uint8_t                 running;
//    uint8_t                 data_bufferA[DATA_BUFFER_SIZE];   // This stores the data until we have enough to write to flash.
//    uint8_t                 data_bufferB[DATA_BUFFER_SIZE];   // This stores the data until we have enough to write to flash.
//    uint8_t                 launchpadBuffer[DATA_BUFFER_SIZE * 25];
//    uint8_t                 measurement_length;
//    uint16_t                buffer_index_curr;  //The current index in the buffer.
//    uint16_t                ring_buff_size;
//    uint32_t                flash_address;
//    int32_t                 acc_z_filtered;
//    configuration_data_t    *config_data;
//    TaskHandle_t            *timer_thread_handle;
//    data_measurement        measurement;
//    BufferSelection_t       buffer_selection;
//    imu_sensor_data         imu_reading;
//    pressure_sensor_data    bmp_reading;
//    float                   total_filtered_altitude;
//    uint8_t                 alt_filter_count;
//    float                   altitude;
//    float                   last_altitude;
//    uint8_t                 alt_count;
//    uint8_t                 alt_main_count;
//    uint16_t                apogee_holdout_count;
//    uint8_t                 landed_counter;
//    uint32_t                start_time;
//    flight_state_controller_thread_parameters *flight_state_controller_params;
//} necessary_parameters;
//
//typedef enum
//{
//    CONTROLLER_STATE_LAUNCHPAD                 = 1,
//    CONTROLLER_STATE_LAUNCHPAD_ARMED,
//    CONTROLLER_STATE_IN_FLIGHT_PRE_APOGEE,
//    CONTROLLER_STATE_IN_FLIGHT_POST_APOGEE,
//    CONTROLLER_STATE_IN_FLIGHT_POST_MAIN,
//    CONTROLLER_STATE_LANDED,
//    CONTROLLER_STATE_EXIT,
//    CONTROLLER_NUM_STATES
//} StateType;
//
//typedef struct
//{
//    StateType state;
//    void (*function)(necessary_parameters);
//} state_machine_type;
//
//void sm_STATE_LAUNCHPAD             (necessary_parameters);
//void sm_STATE_LAUNCHPAD_ARMED       (necessary_parameters);
//void sm_STATE_IN_FLIGHT_PRE_APOGEE  (necessary_parameters);
//void sm_STATE_IN_FLIGHT_POST_APOGEE (necessary_parameters);
//void sm_STATE_IN_FLIGHT_POST_MAIN   (necessary_parameters);
//void sm_STATE_LANDED                (necessary_parameters);
//void sm_STATE_EXIT                  (necessary_parameters);
//
//state_machine_type state_machine[] =
//{
//    {CONTROLLER_STATE_LAUNCHPAD,                sm_STATE_LAUNCHPAD              },
//    {CONTROLLER_STATE_LAUNCHPAD_ARMED,          sm_STATE_LAUNCHPAD_ARMED        },
//    {CONTROLLER_STATE_IN_FLIGHT_PRE_APOGEE,     sm_STATE_IN_FLIGHT_PRE_APOGEE   },
//    {CONTROLLER_STATE_IN_FLIGHT_POST_APOGEE,    sm_STATE_IN_FLIGHT_POST_APOGEE  },
//    {CONTROLLER_STATE_IN_FLIGHT_POST_MAIN,      sm_STATE_IN_FLIGHT_POST_MAIN    },
//    {CONTROLLER_STATE_LANDED,                   sm_STATE_LANDED                 },
//    {CONTROLLER_STATE_EXIT,                     sm_STATE_EXIT                   }
//};
//
//StateType sm_state = CONTROLLER_STATE_LAUNCHPAD;
///**
// * @brief Call this function to run the state machine
// */
//void state_machine_tick(necessary_parameters parameters)
//{
//    // Check to make sure that the state is being entered is valid
//    if(sm_state < CONTROLLER_NUM_STATES)
//    {
//        // Call the function for the state
//        (*state_machine[sm_state].function)(parameters);
//    }else
//    {
//        // Throw an exception
//    }
//}
//
//void sm_STATE_LAUNCHPAD(necessary_parameters parameters)
//{
//    /**
//     * @todo fill in this method
//     */
//     sm_state = CONTROLLER_STATE_LAUNCHPAD_ARMED;
//}
//void sm_STATE_LAUNCHPAD_ARMED(necessary_parameters parameters)
//{
//    if(parameters.imu_reading.acc_x < 10892)
//        return;
//
//    buzz(250);
//    vTaskResume(*parameters.timer_thread_handle); //start fixed timers.
//    parameters.config_data->values.flags = parameters.config_data->values.flags | 0x04;
//    //Record the launch event.
//    uint32_t header =
//        (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) + parameters.measurement.data[2];
//    header |= LAUNCH_DETECT;
//    parameters.config_data->values.flags = parameters.config_data->values.flags | 0x01;
//    write_config(parameters.config_data);
//    write_24(header, &parameters.measurement.data[0]);
//
//
//    uint16_t buff_end = (parameters.ring_buff_size);
//    for(uint8_t j = 0; j < 25; j++)
//    {
//        //need to copy last portion into the bufferA. Make sure to start from right place, which wont be the next spot.
//
//        if((parameters.buffer_index_curr + 256) < buff_end)
//        {
//            FlashStatus stat_f2 = flash_write(parameters.flash_address,
//											  &parameters.launchpadBuffer[parameters.buffer_index_curr],
//											  DATA_BUFFER_SIZE);
//            while(FLASH_IS_DEVICE_BUSY(stat_f2))
//            {
//                stat_f2 = flash_get_status_register();
//                vTaskDelay(1);
//            }
//            parameters.buffer_index_curr += 256;
//        }else
//        {
//
//            uint8_t buff_temp[256];
//            memcpy(&buff_temp, &parameters.launchpadBuffer[parameters.buffer_index_curr], buff_end - parameters.buffer_index_curr);
//            memcpy(&buff_temp[buff_end - parameters.buffer_index_curr], &parameters.launchpadBuffer, DATA_BUFFER_SIZE - (buff_end - parameters.buffer_index_curr));
//
//            FlashStatus stat_f2 = flash_write(parameters.flash_address, buff_temp,
//											  DATA_BUFFER_SIZE);
//            while(FLASH_IS_DEVICE_BUSY(stat_f2))
//            {
//                stat_f2 = flash_get_status_register();
//                vTaskDelay(1);
//            }
//            parameters.buffer_index_curr = DATA_BUFFER_SIZE - (buff_end - parameters.buffer_index_curr);
//        }
//
//        parameters.flash_address += DATA_BUFFER_SIZE;
//    }
//    parameters.buffer_index_curr = 0;
//
//
//    parameters.config_data->values.state = STATE_IN_FLIGHT_PRE_APOGEE;
//    sm_state = CONTROLLER_STATE_IN_FLIGHT_PRE_APOGEE;
//}
//void sm_STATE_IN_FLIGHT_PRE_APOGEE(necessary_parameters parameters)
//{
//    parameters.apogee_holdout_count++;
//    if(parameters.apogee_holdout_count > (20 * 15))
//    {
//
//        uint64_t acc_mag = pow(parameters.imu_reading.acc_x, 2) + pow(parameters.imu_reading.acc_y, 2) + pow(parameters.imu_reading.acc_z, 2);
//        uint32_t header;
//        if(acc_mag < 1 && parameters.total_filtered_altitude > 9000.0)
//        {
//            //5565132 = 3 * 1362^2 (aprox 0.5 g on all direction)
//            //2438m -> 8,000 ft
//            buzz(250);
//            RecoverySelect event = DROGUE;
//            recovery_enable_mosfet(event);
//            recovery_activate_mosfet(event);
//            RecoveryContinuityStatus cont = recovery_check_continuity(event);
//            header = (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) + parameters.measurement.data[2];
//            header |= DROGUE_DETECT;
//            parameters.measurement.data[0] = (header >> 16) & 0xFF;
//            parameters.measurement.data[1] = (header >> 8) & 0xFF;
//            parameters.measurement.data[2] = (header) & 0xFF;
//
//            if(cont == OPEN_CIRCUIT)
//            {
//                parameters.config_data->values.flags = parameters.config_data->values.flags | 0x08;
//                write_config(parameters.config_data);
//                header = (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) + parameters.measurement.data[2];
//                header |= DROGUE_DEPLOY;
//                parameters.measurement.data[0] = (header >> 16) & 0xFF;
//                parameters.measurement.data[1] = (header >> 8) & 0xFF;
//                parameters.measurement.data[2] = (header) & 0xFF;
//                parameters.config_data->values.state = STATE_IN_FLIGHT_POST_APOGEE;
//                sm_state = CONTROLLER_STATE_IN_FLIGHT_POST_APOGEE;
//            }
//        }
//    }
//}
//void sm_STATE_IN_FLIGHT_POST_APOGEE(necessary_parameters parameters)
//{
//    if(parameters.total_filtered_altitude < 375.0)
//    {
//        //375m ==  1230 ft
//        parameters.alt_main_count++;
//    }else
//    {
//        parameters.alt_main_count = 0;
//    }
//    if(parameters.alt_main_count > 5)
//    {
//        uint32_t header;
//        //deploy main
//        buzz(250);
//        RecoverySelect event = MAIN;
//        recovery_enable_mosfet(event);
//        recovery_activate_mosfet(event);
//        RecoveryContinuityStatus cont = recovery_check_continuity(event);
//        header = (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) + parameters.measurement.data[2];
//        header |= MAIN_DETECT;
//
//        parameters.measurement.data[0] = (header >> 16) & 0xFF;
//        parameters.measurement.data[1] = (header >> 8) & 0xFF;
//        parameters.measurement.data[2] = (header) & 0xFF;
//
//        if(cont == OPEN_CIRCUIT)
//        {
//            parameters.config_data->values.flags = parameters.config_data->values.flags | 0x10;
//            write_config(parameters.config_data);
//            header = (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) + parameters.measurement.data[2];
//            header |= MAIN_DEPLOY;
//            parameters.measurement.data[0] = (header >> 16) & 0xFF;
//            parameters.measurement.data[1] = (header >> 8) & 0xFF;
//            parameters.measurement.data[2] = (header) & 0xFF;
//
//            parameters.config_data->values.state = STATE_IN_FLIGHT_POST_MAIN;
//            sm_state = CONTROLLER_STATE_IN_FLIGHT_POST_MAIN;
//        }
//        else
//        {
//            // TODO: What if cont != OPEN_CIRCUIT? then what should we do here?
//        }
//    }
//}
//void sm_STATE_IN_FLIGHT_POST_MAIN(necessary_parameters parameters)
//{
//    if(parameters.alt_count > 0)
//    {
//        //If altitude is within a 1m range for 20 samples
//        if(parameters.altitude > (parameters.last_altitude - 1.0) && parameters.altitude < (parameters.last_altitude + 1.0))
//        {
//            parameters.alt_count++;
//            if(parameters.alt_count > 245)
//            {
//                parameters.alt_count = 201;
//            }
//        }else
//        {
//            parameters.alt_count = 0;
//        }
//
//    }else
//    {
//        parameters.last_altitude = parameters.altitude;
//        parameters.alt_count++;
//        if(parameters.alt_count > 245)
//        {
//            parameters.alt_count = 201;
//        }
//    }
//
//    if((pow(parameters.imu_reading.gyro_x, 2) +
//        pow(parameters.imu_reading.gyro_y, 2) +
//        pow(parameters.imu_reading.gyro_z, 2)) < 63075)
//    {
//        //If the gyro readings are all less than ~4.4 deg/sec and the altitude is not changing then the rocket has probably landed.
//        if(parameters.alt_count > 200){
//            parameters.config_data->values.state = STATE_LANDED;
//            sm_state = CONTROLLER_STATE_LANDED;
//        }
//        else
//        {
//            // TODO: What if cont parameters.alt_count <= 200? then what should we do here?
//        }
//    }
//    else
//    {
//        // TODO: what to do if we are here?
//    }
//}
//void sm_STATE_LANDED(necessary_parameters parameters)
//{
//    parameters.config_data->values.flags = parameters.config_data->values.flags & ~(0x01);
//    write_config(parameters.config_data);
//    uint32_t header = (parameters.measurement.data[0] << 16) + (parameters.measurement.data[1] << 8) +
//                      parameters.measurement.data[2];
//    header |= LAND_DETECT;
//
//    parameters.measurement.data[0] = (header >> 16) & 0xFF;
//    parameters.measurement.data[1] = (header >> 8)  & 0xFF;
//    parameters.measurement.data[2] = (header)       & 0xFF;
//
//    sm_state = CONTROLLER_STATE_EXIT;
//}
//void sm_STATE_EXIT(necessary_parameters parameters)
//{
//    // Put everything into low power mode.
//    parameters.running = 0;
//}
//
//
//void check_recovery_circuit(configuration_data_t *params)
//{
//    RecoverySelect event_d = DROGUE;
//    RecoveryContinuityStatus cont_d = recovery_check_continuity(event_d);
//    RecoverySelect event_m = MAIN;
//    RecoveryContinuityStatus cont_m = recovery_check_continuity(event_m);
//
//    while(cont_m == OPEN_CIRCUIT || cont_d == OPEN_CIRCUIT)
//    {
//
//        cont_m = recovery_check_continuity(event_m);
//        cont_d = recovery_check_continuity(event_d);
//    }
//
//    params->values.state = STATE_LAUNCHPAD_ARMED;
//    write_config(params);
//}
//
//void fill_buffer_and_or_write_to_flash(necessary_parameters parameters);
//bool try_to_get_data_from_imu(necessary_parameters parameters);
//bool try_to_get_data_from_pressure_sensor(necessary_parameters parameters);
//
//void thread_flight_state_controller_start(void *params)
//{
//    necessary_parameters parameters =
//    {
//        .flight_state_controller_params  = (flight_state_controller_thread_parameters *) params,
////        .config_data                     = parameters.flight_state_controller_params->configuration_data,
////        .timer_thread_handle             = parameters.flight_state_controller_params->timer_thread_handle,
//        .flash_address                   = FLASH_START_ADDRESS,
//        .measurement_length              = 0,
//        .buffer_index_curr               = 0,
//        .ring_buff_size                  = DATA_BUFFER_SIZE * 25,
//        .buffer_selection                = BUFFER_A,
//        .total_filtered_altitude         = 0,
//        .alt_filter_count                = 0,
//        .last_altitude                   = 0,
//        .alt_count                       = 0,
//        .alt_main_count                  = 0,
//        .apogee_holdout_count            = 0,
//        .landed_counter                  = 0,
//        .running                         = 1
//    };
//
////    if(CONFIGURATION_IS_IN_FLIGHT(parameters.config_data->values.flags)){
////        parameters.flash_address = parameters.config_data->values.end_data_address;
////    }
//
//    //Make sure the measurement starts empty.
//    clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//
////    if(!CONFIGURATION_IS_IN_FLIGHT(parameters.config_data->values.flags)){
////        check_recovery_circuit(parameters.config_data);
////    }
//
//    buzz(250); // CHANGE TO 2 SECONDS!!!!!!!
//    parameters.start_time = xTaskGetTickCount();
//    while(1)
//    {
//        if(!try_to_get_data_from_imu(parameters))
//            continue;
//
//        if(!try_to_get_data_from_pressure_sensor(parameters))
//            continue;
//
//        state_machine_tick(parameters);
//        fill_buffer_and_or_write_to_flash(parameters);
//
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//        parameters.measurement_length = 0;
//
//        if(!parameters.running){
//            vTaskSuspend(NULL);
//        }
//    };
//
//}
//
//bool try_to_get_data_from_imu(necessary_parameters parameters)
//{
//    //Try and get data from the IMU queue. Block for up to a quarter of the time between the fastest measurement.
//    if(imu_read(&parameters.imu_reading, parameters.config_data->values.data_rate / 4))
//    {
//        //Check if the current measurement has data.
//        if(!is_buffer_empty(parameters.measurement.data, sizeof(data_measurement)))
//        {
//            uint16_t timestamp = parameters.imu_reading.time_ticks - parameters.start_time;
//            imu_sensor_data_to_bytes(parameters.imu_reading, &parameters.measurement.data[0], timestamp);
//            parameters.measurement_length = ACC_LENGTH + GYRO_LENGTH;
//            parameters.start_time = parameters.imu_reading.time_ticks;
//        }
//
//        return true;
//    }else
//    {
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//        return false;
//    }
//}
//
//bool try_to_get_data_from_pressure_sensor(necessary_parameters parameters)
//{
//    //Try and get data from the BMP queue. Block for up to a quarter of the time between the fastest measurement.
//    if(pressure_sensor_read(&parameters.bmp_reading, parameters.config_data->values.data_rate / 4))
//    {
//        if(is_buffer_empty(parameters.measurement.data, sizeof(data_measurement)))
//        {
//            // TODO: this does not make sense, how can we have imu
//            //       readings and entering here if Measurement is Empty
//            //We already have a imu reading.
//            //Update the header bytes.
//            pressure_sensor_data_to_bytes(parameters.bmp_reading, &parameters.measurement.data[0]);
//
//            float approx_altitude = pressure_sensor_calculate_altitude(&parameters.bmp_reading);
//            parameters.total_filtered_altitude += (approx_altitude - parameters.total_filtered_altitude) * 0.2;
//            parameters.measurement_length += (PRES_LENGTH + TEMP_LENGTH + ALT_LENGTH);
//            // TODO: why we are not updating start_time here like in the IMU clause?
//        }
//
//        return true;
//    }
//    else
//    {
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//        return false;
//    }
//}
//
//
//void fill_buffer_and_or_write_to_flash(necessary_parameters parameters)
//{
//    if(is_buffer_empty(parameters.measurement.data, sizeof(data_measurement)) &&
//       parameters.config_data->values.state == STATE_LAUNCHPAD_ARMED &&
//       ((parameters.buffer_index_curr + parameters.measurement_length + HEADER_SIZE) <= parameters.ring_buff_size))
//    {
//        //check if room in launchpad buffer.
//
//        memcpy(&parameters.launchpadBuffer[parameters.buffer_index_curr], &(parameters.measurement.data),
//               parameters.measurement_length + HEADER_SIZE);
//
//        parameters.buffer_index_curr += (parameters.measurement_length + HEADER_SIZE);
//        parameters.buffer_index_curr = parameters.buffer_index_curr % (parameters.ring_buff_size);
//
//        //Reset the parameters.measurement.
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//
//    }else if(((parameters.buffer_index_curr + parameters.measurement_length + HEADER_SIZE) < DATA_BUFFER_SIZE) &&
//             (is_buffer_empty(parameters.measurement.data, sizeof(data_measurement))))
//    {
//
//        //There is room in the current buffer for the full parameters.measurement.
//
//        if(parameters.buffer_selection == BUFFER_A)
//        {
//            memcpy(&parameters.data_bufferA[parameters.buffer_index_curr], &(parameters.measurement.data),
//                   parameters.measurement_length + HEADER_SIZE);
//            //uart_transmit_bytes(uart,&data_bufferA[buffer_index_curr],parameters.measurement_length+2);
//        }else if(parameters.buffer_selection == BUFFER_B)
//        {
//
//            memcpy(&parameters.data_bufferB[parameters.buffer_index_curr], &(parameters.measurement.data),
//                   parameters.measurement_length + HEADER_SIZE);
//            //uart_transmit_bytes(uart,&data_bufferB[buffer_index_curr],parameters.measurement_length+2);
//        }
//
//        parameters.buffer_index_curr += (parameters.measurement_length + HEADER_SIZE);
//
//        //Reset the parameters.measurement.
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//
//
//    }
//    else if(is_buffer_empty(parameters.measurement.data, sizeof(data_measurement)))
//    {
//
//        //Split measurement across the buffers, and write to flash.
//        uint8_t bytesInPrevBuffer = DATA_BUFFER_SIZE - parameters.buffer_index_curr;
//        uint8_t bytesLeft = (parameters.measurement_length + HEADER_SIZE) - bytesInPrevBuffer;
//
//        //Put as much data as will fit into the almost full buffer.
//        switch(parameters.buffer_selection)
//        {
//            case BUFFER_A:
//            {
//                memcpy(&parameters.data_bufferA[parameters.buffer_index_curr], &(parameters.measurement.data),bytesInPrevBuffer);
//                parameters.buffer_selection  = BUFFER_B;
//                parameters.buffer_index_curr = 0;
//
//                memcpy(&parameters.data_bufferA[parameters.buffer_index_curr],&(parameters.measurement.data[bytesInPrevBuffer]), bytesLeft);
//                parameters.buffer_index_curr = bytesLeft;
//                break;
//            }
//            case BUFFER_B:
//            {
//                memcpy(&parameters.data_bufferB[parameters.buffer_index_curr], &(parameters.measurement.data),bytesInPrevBuffer);
//                parameters.buffer_selection  = BUFFER_A;
//                parameters.buffer_index_curr = 0;
//
//                //Put the rest of the measurement in the next buffer.
//                memcpy(&parameters.data_bufferB[parameters.buffer_index_curr],&(parameters.measurement.data[bytesInPrevBuffer]), bytesLeft);
//                parameters.buffer_index_curr = bytesLeft;
//                break;
//            }
//        }
//
//
//        // Put the rest of the measurement in the next buffer.
//        switch(parameters.buffer_selection)
//        {
//            case BUFFER_A:
//            {
//                memcpy(&parameters.data_bufferA[parameters.buffer_index_curr],&(parameters.measurement.data[bytesInPrevBuffer]), bytesLeft);
//                parameters.buffer_index_curr = bytesLeft;
//                break;
//            }
//            case BUFFER_B:
//            {
//                memcpy(&parameters.data_bufferB[parameters.buffer_index_curr],&(parameters.measurement.data[bytesInPrevBuffer]), bytesLeft);
//                parameters.buffer_index_curr = bytesLeft;
//                break;
//            }
//        }
//
//
//        //reset the measurement.
//        clear_buffer(parameters.measurement.data, sizeof(data_measurement));
//        parameters.measurement_length = 0;
//
//
//        //Flash write buffer not in use! then clear old buffer?
//
//        if(parameters.buffer_selection == 0)
//        {
//            //We just switched to A so uart_transmit B.
//            if(CONFIGURATION_IS_RECORDING(parameters.config_data->values.flags))
//            {
//                FlashStatus stat_f = flash_write(parameters.flash_address,
//												 parameters.data_bufferB, DATA_BUFFER_SIZE);
//                while(FLASH_IS_DEVICE_BUSY(stat_f))
//                {
//                    stat_f = flash_get_status_register();
//                    vTaskDelay(1);
//                }
//
//                parameters.flash_address += DATA_BUFFER_SIZE;
//                if(parameters.flash_address >= FLASH_SIZE_BYTES)
//                {
//                    while(1)
//                    {
//                        ;
//                    }
//                }
//
//            }else
//            {
//                uart6_transmit_bytes(parameters.data_bufferB, 256);
//            }
//        }else if(parameters.buffer_selection == 1)
//        {
//            //We just switched to B so uart_transmit A
//
//            if(CONFIGURATION_IS_RECORDING(parameters.config_data->values.flags))
//            {
//                FlashStatus stat_f2 = flash_write(parameters.flash_address,
//												  parameters.data_bufferA, DATA_BUFFER_SIZE);
//                while(FLASH_IS_DEVICE_BUSY(stat_f2))
//                {
//                    stat_f2 = flash_get_status_register();
//                    vTaskDelay(1);
//                }
//
//                parameters.flash_address += DATA_BUFFER_SIZE;
//
//                if(parameters.flash_address >= FLASH_SIZE_BYTES)
//                {
//                    while(1)
//                    {
//                        ;
//                    }
//                }
//            }else
//            {
//                uart6_transmit_bytes(parameters.data_bufferA, 256);
//            }
//        }
//
//
//    }
//}
