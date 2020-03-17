#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS/Avionics-2019
//
// File Description:
//  Header file for the data logging module. This uses the flash memory interface to log data to memory.
//
// History
// 2019-04-09 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <math.h>
#include "flash.h"
#include "protocols/UART.h"
#include "flight_system_configuration.h"


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define DATA_BUFFER_SIZE  FLASH_PAGE_SIZE //Matches flash memory page size.

#define DROGUE_DETECT     0x080000
#define DROGUE_DEPLOY     0x040000

#define MAIN_DETECT       0x020000
#define MAIN_DEPLOY       0x010000

#define LAUNCH_DETECT     0x008000
#define LAND_DETECT       0x004000

#define POWER_FAIL        0x002000
#define OVERCURRENT_EVENT 0x001000



int flight_controller_initialize(void * pvParams);
int flight_controller_start();
void flight_controller_stop();

#endif // DATA_LOGGING_H
