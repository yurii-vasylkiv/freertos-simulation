#ifndef __EVENT_DETECTOR_H
#define __EVENT_DETECTOR_H

#include "memory-management/memory_manager.h"
#include "core/system_configuration.h"
#include <stdbool.h>

// Detection Parameters

typedef enum
{
    FLIGHT_STATE_LAUNCHPAD      =0,
    FLIGHT_STATE_PRE_APOGEE     =1,
    FLIGHT_STATE_APOGEE         =2,
    FLIGHT_STATE_POST_APOGEE    =3,
    FLIGHT_STATE_MAIN_CHUTE     =4,
    FLIGHT_STATE_POST_MAIN      =5,
    FLIGHT_STATE_LANDED         =6,
    FLIGHT_STATE_EXIT           =7,
    FLIGHT_STATE_COUNT
} FlightState;

typedef enum { EVENT_DETECTOR_ERR   = 0, EVENT_DETECTOR_OK    = 1 } EventDetectorStatus;


EventDetectorStatus event_detector_init( FlightSystemConfiguration * configurations);
EventDetectorStatus event_detector_update_configurations ( FlightSystemConfiguration * configurations);
EventDetectorStatus event_detector_feed ( DataContainer * data, FlightState * state );
float event_detector_current_altitude();
bool event_detector_is_flight_started();

#endif