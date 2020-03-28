#include "flight_state_machine.h"


typedef struct
{
    FlightState state;
    int (*function)(FlightStateMachineTickParameters*);
} state_machine_type;


int sm_STATE_LAUNCHPAD   (FlightStateMachineTickParameters*);
int sm_STATE_PRE_APOGEE  (FlightStateMachineTickParameters*);
int sm_STATE_APOGEE      (FlightStateMachineTickParameters*);
int sm_STATE_POST_APOGEE (FlightStateMachineTickParameters*);
int sm_STATE_MAIN_CHUTE  (FlightStateMachineTickParameters*);
int sm_STATE_POST_MAIN   (FlightStateMachineTickParameters*);
int sm_STATE_LANDED      (FlightStateMachineTickParameters*);
int sm_STATE_EXIT        (FlightStateMachineTickParameters*);

state_machine_type state_machine[] =
{
    {FLIGHT_STATE_LAUNCHPAD,        sm_STATE_LAUNCHPAD     },
    {FLIGHT_STATE_PRE_APOGEE,       sm_STATE_PRE_APOGEE    },
    {FLIGHT_STATE_APOGEE,           sm_STATE_APOGEE        },
    {FLIGHT_STATE_POST_APOGEE,      sm_STATE_POST_APOGEE   },
    {FLIGHT_STATE_MAIN_CHUTE,       sm_STATE_MAIN_CHUTE    },
    {FLIGHT_STATE_POST_MAIN,        sm_STATE_POST_MAIN     },
    {FLIGHT_STATE_LANDED,           sm_STATE_LANDED        },
    {FLIGHT_STATE_EXIT,             sm_STATE_EXIT          }
};

static FlightState sm_state;
static uint8_t s_is_initialized                         = { 0 };

int flight_state_machine_init ( FlightState state )
{
    sm_state = state;

    s_is_initialized = 1;

    return 0;
}



int flight_state_machine_tick( FlightState state, FlightStateMachineTickParameters * parameters )
{
    if ( parameters == NULL )
    {
        return 1;
    }

    if ( !s_is_initialized )
    {
        return 1;
    }

    // Check to make sure that the state is being entered is valid
    if ( sm_state < FLIGHT_STATE_COUNT )
    {
        return state_machine[ state ].function( parameters );
    } else
    {
        // Throw an exception
        return 1;
    }
}


int sm_STATE_LAUNCHPAD(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_PRE_APOGEE(FlightStateMachineTickParameters * parameters)
{
    return 0;
}

int sm_STATE_APOGEE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_POST_APOGEE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_MAIN_CHUTE(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_POST_MAIN(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_LANDED(FlightStateMachineTickParameters * parameters)
{

    return 0;
}

int sm_STATE_EXIT(FlightStateMachineTickParameters * parameters)
{

    return 0;
}
