#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H

#include <math.h>
#include "board/components/flash.h"
#include "protocols/UART.h"
#include "core/system_configuration.h"
#include "event-detection/event_detector.h"

typedef struct
{

}FlightStateMachineTickParameters;


typedef enum { FLIGHT_CONTROLLER_ERR   = 0, FLIGHT_CONTROLLER_OK    = 1 } FlightControllerStatus;

int flight_controller_initialize(void * pvParams);
int flight_controller_start();
void flight_controller_stop();


int flight_state_machine_init ( FlightState state );
int flight_state_machine_tick ( FlightState state, FlightStateMachineTickParameters * parameters);

#endif // DATA_LOGGING_H
