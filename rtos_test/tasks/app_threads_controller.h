//#ifndef STARTUP_TASK_H
//#define STARTUP_TASK_H
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// UMSATS 2018-2020
////
//// Repository:
////  UMSATS/Avionics-2019
////
//// File Description:
////  Header file for the startup task. This task is the first to run.
////
//// History
//// 2019-04-19 by Joseph Howarth
//// - Created.
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// INCLUDES
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//#include "flash.h"
//#include "protocols/UART.h"
//#include "configuration.h"
//
//typedef struct
//{
//    void                    *flight_state_controller_thread_handle ;
//    void                    *pressure_sensor_thread_handle ;
//    void                    *imu_thread_handle ;
//    void                    *cli_thread_params;
//    void                    *timer_thread_handle;
//    configuration_data_t    *configuration_data;
//}startup_thread_parameters;
//
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// Description:
////  This function will be the first task to run when the flight computer is powered on.
////
////As of right now, if S2 is not pressed the task will wait for an amount of time specified in the configuration
////        header file, and will then erase the flash memory and start the IMU, BMP and data logging
////        tasks.
////
////        If S2 is pressed then the xtract task will be started.
////
////        Should be passed a populated startup_thread_parameters struct as the parameter.
////
//// Returns:
////
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//void app_threads_controller_start(void const* pvParams);
//
//#endif // TEMPLATE_H
