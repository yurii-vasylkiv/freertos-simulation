#ifndef AVIONICS_FLIGHT_STATE_MACHINE_H
#define AVIONICS_FLIGHT_STATE_MACHINE_H


#include "events/event_detector.h"


typedef struct
{

}FlightStateMachineTickParameters;

int flight_state_machine_init ( FlightState state );

int flight_state_machine_tick ( FlightState state, FlightStateMachineTickParameters * parameters);

#endif //AVIONICS_FLIGHT_STATE_MACHINE_H
