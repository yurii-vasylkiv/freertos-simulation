#include "event_detector.h"

#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include "configurations/UserConfig.h"

#include "protocols/UART.h"
#include "data_window.h"

#define CRITICAL_VERTICAL_ACCELERATION  6.9 // [g]
#define APOGEE_ACCELERATION             0.1 // [g]
#define MAIN_CHUTE_ALTITUDE             381 // [m] (converted from 1,250ft)
#define LANDING_ROTATION_SPEED          5   // [deg / s]

#define ALTITUDE_SENSITIVITY_THRESHOLD  5

static FlightState flightState;

static float CURRENT_ALTITUDE          = 0;
static uint32_t GROUND_PRESSURE        = 0;
static float GROUND_ALTITUDE           = 0;
static int   INITIALIZED               = 0;

static int   AVERAGE_PRESSURE_SAMPLING_RATE = 0;
static int   AVERAGE_IMU_SAMPLING_RATE      = 0;

typedef struct
{
    IMUDataU      inertial;
    PressureDataU pressure;
} FlightData;


#if (userconf_EVENT_DETECTION_AVERAGING_SUPPORT_ON == 1)
static moving_data_buffer altitude_data_window, vertical_acc_data_window;
static float mean( float * array, size_t length, size_t start, size_t end);
#endif

static float calculate_altitude ( float pressure );

static bool detectLaunch    ( float vertical_acceleration_in_g );
static bool detectApogee    ( float acceleration_x_in_g, float acceleration_y_in_g, float acceleration_z_in_g );
static bool detectAltitude  ( float target_altitude, uint32_t ground_pressure, uint32_t current_pressure );
static bool detectLanding   ( float gyro_x_in_deg_per_sec, float gyro_y_in_deg_per_sec, float gyro_z_in_deg_per_sec );

float event_detector_current_altitude()
{
    return CURRENT_ALTITUDE;
}

bool event_detector_is_flight_started()
{
    return flightState != FLIGHT_STATE_LAUNCHPAD;
}


int event_detector_init( FlightSystemConfiguration * configurations)
{
    if ( INITIALIZED == 1)
    {
        DISPLAY_LINE("Event Detector has been already initialized.", NULL);
        return 0;
    }

    if(configurations == NULL)
    {
        return 1;
    }

    GROUND_PRESSURE = configurations->ground_pressure;
    GROUND_ALTITUDE = calculate_altitude ( GROUND_PRESSURE );

    // in case of reboot during the flight if memory is not corrupted this flag should change to the
    // appropriate current flight stage that != FLIGHT_STATE_LAUNCHPAD
    flightState     = configurations->flight_state;

#if (userconf_EVENT_DETECTION_AVERAGING_SUPPORT_ON == 1)
    data_window_init ( &altitude_data_window );
    data_window_init ( &vertical_acc_data_window );
#endif


    INITIALIZED = 1;
    return 0;
}

int event_detector_update_configurations ( FlightSystemConfiguration * configurations)
{
    if (INITIALIZED == 0)
    {
        DISPLAY_LINE("CANNOT update configurations. Event Detector has not been initialized.", NULL);
        return 1;
    }

    if(configurations == NULL)
    {
        DISPLAY_LINE("CANNOT update configurations. NullPointer.", NULL);
        return 1;
    }

    GROUND_PRESSURE = configurations->ground_pressure;
    GROUND_ALTITUDE = calculate_altitude ( GROUND_PRESSURE );

    return 0;
}


FlightState event_detector_feed ( Data * data)
{
    if (INITIALIZED == 0)
    {
        DISPLAY_LINE("CANNOT feed. Event Detector has not been initialized.", NULL);
        return 1;
    }

    if(data->pressure.updated)
    {
        CURRENT_ALTITUDE = calculate_altitude( data->pressure.data.pressure) - GROUND_ALTITUDE;
#if (userconf_EVENT_DETECTION_AVERAGING_SUPPORT_ON == 1)
        data_window_insert( &altitude_data_window, &CURRENT_ALTITUDE );
#endif
    }

    switch ( flightState )
    {
        case FLIGHT_STATE_LAUNCHPAD:
        {
            if(data->inertial.updated)
            {
                if ( detectLaunch( data->inertial.data.accelerometer[ 0 ] ) )
                {
                    DEBUG_LINE( "FLIGHT_STATE_LAUNCHPAD: Detected Launch at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_PRE_APOGEE;
                }
            }

            return 0;
        }

        case FLIGHT_STATE_PRE_APOGEE:
        {
            if(data->inertial.updated)
            {
#if (userconf_EVENT_DETECTION_AVERAGING_SUPPORT_ON == 1)
                // Here we need to start looking at the average altitude and see the differences in the gradient sign
                // the idea is that if the gradient changes the sign then we reached the apogee since the altitude
                // is now decreasing instead of increasing. Also to avoid mechanical errors if the sign changes
                // dramatically then it cannot represent the real world scenario, as in the real world the inertia
                // makes it stop really slowly as well as falling down, therefore the difference needs to be small.

                float previous_average_altitude = mean( altitude_data_window.linear_repr, sizeof( altitude_data_window.linear_repr ), 0, MOVING_BUFFER_RANGE - 1 );
                float last_average_altitude     = mean( altitude_data_window.linear_repr, sizeof( altitude_data_window.linear_repr ), 1, MOVING_BUFFER_RANGE );

                float difference = last_average_altitude - previous_average_altitude;
                float absolute_difference = fabs(fabs(last_average_altitude) - fabs(previous_average_altitude));

                if((difference < 0) && absolute_difference < 5 && absolute_difference > 0.2)
                {
                    DEBUG_LINE( "FLIGHT_STATE_PRE_APOGEE: Detected APOGEE at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_APOGEE;
                }
#else
                if ( detectApogee( data->inertial.data.accelerometer[ 0 ], data->inertial.data.accelerometer[ 1 ], data->inertial.data.accelerometer[ 2 ] ) )
                {
                    DISPLAY_LINE( "Detected APOGEE at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_APOGEE;
                }
#endif

            }

            return 0;
        }
        case FLIGHT_STATE_APOGEE:
        {
            DEBUG_LINE( "FLIGHT_STATE_APOGEE: Igniting recovery circuit drogue", NULL );
            flightState = FLIGHT_STATE_POST_APOGEE;

            return 0;
        }

        case FLIGHT_STATE_POST_APOGEE:
        {
            if(data->pressure.updated)
            {
                if ( detectAltitude( MAIN_CHUTE_ALTITUDE, GROUND_ALTITUDE, data->pressure.data.pressure ) )
                {
                    DEBUG_LINE( "FLIGHT_STATE_POST_APOGEE: Detected Main Chute at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_MAIN_CHUTE;
                }
            }

            return 0;
        }

        case FLIGHT_STATE_MAIN_CHUTE:
        {
            DEBUG_LINE("FLIGHT_STATE_MAIN_CHUTE: Igniting recovery circuit for the main chute", NULL);
            flightState = FLIGHT_STATE_POST_MAIN;

            return 0;
        }

        case FLIGHT_STATE_POST_MAIN:
        {
            if(data->inertial.updated)
            {
                if ( detectLanding( data->inertial.data.gyroscope[ 0 ], data->inertial.data.gyroscope[ 1 ],
                                    data->inertial.data.gyroscope[ 2 ] ) )
                {
                    DEBUG_LINE( "FLIGHT_STATE_POST_MAIN: Detected landing at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_LANDED;
                    return 0;
                }
            }

            // OR

            if(data->pressure.updated)
            {
                if ( detectAltitude(0, GROUND_ALTITUDE, data->pressure.data.pressure ) )
                {
                    DEBUG_LINE( "FLIGHT_STATE_POST_MAIN: Detected landing at %fm", CURRENT_ALTITUDE);
                    flightState = FLIGHT_STATE_LANDED;
                    return 0;
                }
            }

            return 0;
        }
        case FLIGHT_STATE_LANDED:
        {
            DEBUG_LINE("FLIGHT_STATE_LANDED: Rocket landed!", NULL);
            flightState = FLIGHT_STATE_EXIT;

            return 0;
        }

        case FLIGHT_STATE_EXIT:
        {
            DEBUG_LINE( "FLIGHT_STATE_EXIT: Exit!", NULL );
            flightState = FLIGHT_STATE_COUNT;
            return 0;
        }
        case FLIGHT_STATE_COUNT:
            break;
    }

    return 1;
}


static bool detectLaunch    ( float vertical_acceleration_in_g )
{
    return vertical_acceleration_in_g > CRITICAL_VERTICAL_ACCELERATION;
}


static bool detectApogee    ( float acceleration_x_in_g, float acceleration_y_in_g, float acceleration_z_in_g )
{
    const float ACCELERATION_VECTOR = sqrt(acceleration_x_in_g*acceleration_x_in_g + acceleration_y_in_g*acceleration_y_in_g + acceleration_z_in_g*acceleration_z_in_g);
    return ACCELERATION_VECTOR < APOGEE_ACCELERATION;
}


static bool detectAltitude  ( float target_altitude, uint32_t ground_altitude, uint32_t current_pressure )
{
    float current_altitude = calculate_altitude( current_pressure ) - ground_altitude;
    return fabsf ( ( current_altitude ) - ( target_altitude ) ) < ALTITUDE_SENSITIVITY_THRESHOLD;
}

static bool detectLanding   ( float gyro_x_in_deg_per_sec, float gyro_y_in_deg_per_sec, float gyro_z_in_deg_per_sec )
{
    float gyroscope_orientation_vector = sqrt( gyro_x_in_deg_per_sec * gyro_x_in_deg_per_sec + gyro_y_in_deg_per_sec * gyro_y_in_deg_per_sec + gyro_z_in_deg_per_sec * gyro_z_in_deg_per_sec);
    return gyroscope_orientation_vector < LANDING_ROTATION_SPEED;
}


static float calculate_altitude( float pressure )
{
    /*
     Pb = static pressure (pressure at sea level) [Pa]
     Tb = standard temperature (temperature at sea level) [K]
     Lb = standard temperature lapse rate [K/m] = -0.0065 [K/m]
     h  = height about sea level [m]
     hb = height at the bottom of atmospheric layer [m]
     R  = universal gas constant = 8.31432
     q0 = gravitational acceleration constant = 9.80665
     M  = molar mass of Earth’s air = 0.0289644 [kg/mol]
    */

    static const float  Pb  = 101325.00f;
    static const float  Tb  = 15.00 + 273.15;
    static const float  Lb  = -0.0065;
    static const int    hb  = 0;
    static const float  R   = 8.31432;
    static const float  g0  = 9.80665;
    static const float  M   = 0.0289644;

    return hb + ( Tb / Lb ) * ( pow ( ( pressure / Pb ), ( -R * Lb ) / ( g0 * M ) ) - 1 );
}


#if (userconf_EVENT_DETECTION_AVERAGING_SUPPORT_ON == 1)

static float mean( float * array, size_t length, size_t start, size_t end)
{
    assert(end < length);
    assert(start < end);

    float sum = 0;
    for (size_t i = start; i <= end; i++)
    {
        sum += array[i];
    }

    return sum / (end - start);
}

#endif





