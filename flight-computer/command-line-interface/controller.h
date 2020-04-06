#ifndef COMMAND_CONTROLLER_H
#define COMMAND_CONTROLLER_H

#include "definitions.h"
#include "board/components/flash.h"
#include "protocols/UART.h"
#include "core/system_configuration.h"


DECL_NEW_USAGE(INTRO) =   "========== Welcome to Xtract ==========\r\n"
                          "This is a command line interface tool made by the Avionics subdivision of the Rockets team.\r\n\r\n"
                          "Here are some commands to get you started:";
                          
DECL_NEW_USAGE(GENERAL) = "Commands:\r\n"
                          "\t[help] - displays the help menu and more commands\r\n"
                          "\t[read] - Downloads flight data\r\n"
                          "\t[configure] - Setup flight computer\r\n"
                          "\t[ematch] - check and fire ematches\r\n"
                          "\t[mem] - Check on and erase the flash memory\r\n"
                          "\t[save] - Save all setting to the flight computer\r\n"
                          "\t[start] - Start the flight computer\r\n"
                          "\t[datafeeder] - datafeeder event loop\r\n"
						  "\t[test] - command line development debugging purpose\r\n";


OPTION_FUNCTION_DECL(general, help);
OPTION_FUNCTION_DECL(general, save);
OPTION_FUNCTION_DECL(general, start);
OPTION_FUNCTION_DECL(general, test);


void command_line_interface_start         (void                     *pvParameters);


#endif // COMMAND_CONTROLLER_H
