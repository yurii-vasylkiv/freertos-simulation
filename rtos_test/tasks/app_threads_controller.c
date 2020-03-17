////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// UMSATS 2018-2020
////
//// Repository:
////  UMSATS/Avionics-2019
////
//// File Description:
////  Source file for the start up task.
////
//// History
//// 2019-04-19 by Joseph Howarth
//// - Created.
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// INCLUDES
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//#include "app_threads_controller.h"
//
//#include "../configuration.h"
//#include "flash.h"
//#include "../hardware_definitions.h"
//#include "../stm32/STM32.h"
//#include "command_line_interface/controller.h"
//#include <FreeRTOS.h>
//#include <task.h>
////-------------------------------------------------------------------------------------------------------------------------------------------------------------
//// Description:
////  This task will check if the memory is empty and erase it if it is not.
////
////    Right now the function assumes that if the first page of memory is empty,
////    then the whole memory is empty.
////
//// Returns:
////
//
//static void eraseFlash(startup_thread_parameters * params)
//{
//    HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN,GPIO_PIN_RESET);
//    FlashStatus stat;
//    configuration_data_t * config = params->configuration_data;
//
//    uint8_t dataRX[256];
//    uart6_transmit_line("Checking flash memory...");
//    // Read the first page of memory. If its empty, assume the whole memory is empty.
//    stat = flash_read(config->values.start_data_address, dataRX, 256);
//
//    uint16_t good= 0xFFFF;
//
//    int i;
//    for(i=0;i<256;i++)
//    {
//        if(dataRX[i] != 0xFF)
//        {
//            good --;
//        }
//    }
//
//    if(good == 0xFFFF)
//    {
//        uart6_transmit_line("flash empty.");
//        HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN,GPIO_PIN_RESET);
//    }
//    else
//    {
//        uart6_transmit_line("flash not empty.");
//        //Erase the whole flash. This could take up to 2 minutes.
//        //stat = erase_device(flash);
//        uint32_t address = FLASH_START_ADDRESS;
//        while(address < config->values.end_data_address)
//        {
//            if(address>FLASH_PARAM_END_ADDRESS)
//            {
//                stat = flash_erase_sector(address);
//                address += FLASH_SECTOR_SIZE;
//            }
//            else
//            {
//                stat = flash_erase_param_sector(address);
//                address += FLASH_PARAM_SECTOR_SIZE;
//            }
//
//            //Wait for erase to finish
//            while(FLASH_IS_DEVICE_BUSY(stat))
//            {
//                stat = flash_get_status_register();
//                vTaskDelay(pdMS_TO_TICKS(1));
//            }
//        }
//
//		flash_read(FLASH_START_ADDRESS, dataRX, 256);
//        uint16_t empty = 0xFFFF;
//
//        for(i=0;i<256;i++)
//        {
//            if(dataRX[i] != 0xFF)
//            {
//                empty --;
//            }
//        }
//
//        if(empty == 0xFFFF)
//        {
//            uart6_transmit_line("Flash Erased Success!");
//            HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN,GPIO_PIN_SET);
//            HAL_Delay(1000);
//            HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN,GPIO_PIN_RESET);
//        }
//
//    }
//}
//
//
//void app_threads_controller_start(void const* pvParams)
//{
//    startup_thread_parameters * sp = (startup_thread_parameters *) pvParams;
//
//    TaskHandle_t dataLoggingTask_h = sp->flight_state_controller_thread_handle;
//    TaskHandle_t bmpTask_h = sp->pressure_sensor_thread_handle;
//    TaskHandle_t imuTask_h = sp->imu_thread_handle;
//    TaskHandle_t cliTask_h = sp->cli_thread_params;
//    configuration_data_t * config = sp->configuration_data;
//
//    vTaskSuspend(cliTask_h);
//    vTaskSuspend(imuTask_h);
//    vTaskSuspend(bmpTask_h);
//    vTaskSuspend(dataLoggingTask_h);
//
//    while(1)
//    {
//
//        if((!HAL_GPIO_ReadPin(USR_PB_PORT,USR_PB_PIN))||(config->values.state == STATE_CLI))
//        {
//            HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN,GPIO_PIN_SET);
//            config->values.state = STATE_CLI;
//            vTaskResume(cliTask_h);
//            vTaskResume(NULL);
//        }
//        else
//        {
//            if(!CONFIGURATION_IS_IN_FLIGHT(config->values.flags))
//            {
//                uint32_t count = 0;
//                uint32_t delay_ms = 2000;
//                uint8_t state = 0;
//
//                uint32_t time = config->values.initial_time_to_wait;
//
//                while(count<time)
//                {
//                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
//                    HAL_GPIO_TogglePin(USR_LED_PORT,USR_LED_PIN);
//                    count += delay_ms;
//
//                    if(count > (time/2) && state ==0){
//
//                      delay_ms = 1000;
//                      state = 1;
//                    }
//                    else if( count > (3*(time/4)) && state == 1)
//                    {
//                      delay_ms = 500;
//                      state = 2;
//                    }
//                }
//
//                eraseFlash(sp);
//            }
//
//            vTaskResume(dataLoggingTask_h);
//            vTaskResume(imuTask_h);
//            vTaskResume(bmpTask_h);
//            vTaskSuspend(NULL);
//        }
//    }
//}
