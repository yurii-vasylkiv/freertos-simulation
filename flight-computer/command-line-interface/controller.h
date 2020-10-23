#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H

#include "board/components/flash.h"
#include "protocols/UART.h"
#include "core/system_configuration.h"


void command_line_interface_start         (void * const pvParameters);


#endif // COMMAND_CONTROLLER_H
