#ifndef __EVENT_DETECTOR_H
#define __EVENT_DETECTOR_H

#include "memory-management/memory_manager.h"
#include "core/system_configuration.h"

// Detection Parameters

typedef enum
{
    FLIGHT_STATE_LAUNCHPAD=0,
    FLIGHT_STATE_PRE_APOGEE=1,
    FLIGHT_STATE_APOGEE=2,
    FLIGHT_STATE_POST_APOGEE=3,
    FLIGHT_STATE_MAIN_CHUTE=4,
    FLIGHT_STATE_POST_MAIN=5,
    FLIGHT_STATE_LANDED=6,
    FLIGHT_STATE_EXIT=7,
    FLIGHT_STATE_COUNT
} FlightState;


int event_detector_init( FlightSystemConfiguration * configurations);
int event_detector_update_configurations ( FlightSystemConfiguration * configurations);
FlightState event_detector_feed ( Data * data);

#endif