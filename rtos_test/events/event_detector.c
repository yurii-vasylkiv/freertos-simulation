#include "event_detector.h"

#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "memory_management/queue.h"
#include "protocols/UART.h"
#include "tasks/sensors/imu_sensor.h"
#include "tasks/sensors/pressure_sensor.h"

#define CRITICAL_VERTICAL_ACCELERATION  6.9 // [g]
#define APOGEE_ACCELERATION             0.1 // [g]
#define MAIN_CHUTE_ALTITUDE             381 // [m] (converted from 1,250ft)
#define LANDING_ROTATION_SPEED          5   // [deg / s]
#define BACKUP_TIME_LAUNCH_TO_APOGEE    30  // [s]
#define BACKUP_TIME_APOGEE_TO_MAIN      144 // [s]
#define BACKUP_TIME_MAIN_TO_GROUND      22  // [s]


static FlightState flightState;


static uint32_t GROUND_PRESSURE = 0;
static float GROUND_ALTITUDE = 0;
static int   INITIALIZED     = 0;

typedef struct
{
    IMUDataU      inertial;
    PressureDataU pressure;
} FlightData;


static int_averaging_queue temperature_values;
static int_averaging_queue pressure_values;
static int_averaging_queue accel_values [ 3 ];
static int_averaging_queue gyro_values  [ 3 ];

static float calculate_altitude ( float pressure );

static bool detectLaunch    ( float vertical_acceleration );
static bool detectApogee    ( float acceleration_x, float acceleration_y, float acceleration_z );
static bool detectAltitude  ( float target_altitude, uint32_t ground_pressure, uint32_t current_pressure );
static bool detectLanding   ( float gyro_x, float gyro_y, float gyro_z );
static bool averageData     ( FlightData * container );


static bool averageData (FlightData * container)
{
//    if(!int_averaging_queue_get( &accel_values[ 0 ], &container->inertial.accelerometer [ 0 ] ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &accel_values[ 1 ], &container->inertial.accelerometer [ 1 ]  ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &accel_values[ 2 ], &container->inertial.accelerometer [ 2 ]  ))
//    {
//        return false;
//    }
//
//
//    if(!int_averaging_queue_get( &gyro_values[ 0 ], &container->inertial.gyroscope [ 0 ]  ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &gyro_values[ 1 ], &container->inertial.gyroscope [ 1 ] ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &gyro_values[ 2 ], &container->inertial.gyroscope [ 2 ] ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &pressure_values, &container->pressure.pressure ))
//    {
//        return false;
//    }
//
//    if(!int_averaging_queue_get( &temperature_values, &container->pressure.temperature ))
//    {
//        return false;
//    }

    return true;
}

int event_detector_init( FlightSystemConfiguration * configurations)
{
    if(configurations == NULL)
    {
        return 1;
    }

    GROUND_PRESSURE = configurations->ground_pressure;
    GROUND_ALTITUDE = calculate_altitude ( GROUND_PRESSURE );

    flightState     = FLIGHT_STATE_LAUNCHPAD;

//    float_averaging_queue_init(&temperature_values, 5);
//    float_averaging_queue_init(&pressure_values, 5);
//    float_averaging_queue_init(&accel_values [ 0 ], 50);
//    float_averaging_queue_init(&accel_values [ 1 ] , 50);
//    float_averaging_queue_init(&accel_values [ 2 ] , 50);
//
//    float_averaging_queue_init(&gyro_values [ 0 ] , 50);
//    float_averaging_queue_init(&gyro_values [ 1 ] , 50);
//    float_averaging_queue_init(&gyro_values [ 2 ] , 50);

    INITIALIZED = 1;
    return 0;
}

int event_detector_update_configurations ( FlightSystemConfiguration * configurations)
{
    if (INITIALIZED == 0)
    {
        return 1;
    }

    if(configurations == NULL)
    {
        return 1;
    }

    GROUND_PRESSURE = configurations->ground_pressure;
    GROUND_ALTITUDE = calculate_altitude ( GROUND_PRESSURE );

    return 0;
}



float BACKUP_TIMER_LAUNCH_TIME = 0; //% [s]
float BACKUP_TIMER_APOGEE_TIME = 0; //% [s]
float BACKUP_TIMER_MAIN_TIME   = 0; //% [s]
float MIN_PRESSURE             = 0;

FlightState event_detector_feed ( Data * data)
{
    if (INITIALIZED == 0)
    {
        return 1;
    }

//    static FlightData averages;
//
    if(data->pressure.updated)
    {
//        uint32_t current_altitude = calculate_altitude(data->pressure.data.pressure);

//        DISPLAY_LINE("Current Altitude: %f", current_altitude - GROUND_ALTITUDE);
//        float_averaging_queue_add( &pressure_values,    data->pressure.data.pressure    );
//        float_averaging_queue_add( &temperature_values, data->pressure.data.temperature );
    }
//
//    if(data->inertial.updated)
//    {
//        float_averaging_queue_add( &accel_values[ 0 ], data->inertial.data.accelerometer[ 0 ] );
//        float_averaging_queue_add( &accel_values[ 1 ], data->inertial.data.accelerometer[ 1 ] );
//        float_averaging_queue_add( &accel_values[ 2 ], data->inertial.data.accelerometer[ 2 ] );
//
//        float_averaging_queue_add( &gyro_values[ 0 ], data->inertial.data.gyroscope[ 0 ] );
//        float_averaging_queue_add( &gyro_values[ 1 ], data->inertial.data.gyroscope[ 1 ] );
//        float_averaging_queue_add( &gyro_values[ 2 ], data->inertial.data.gyroscope[ 2 ] );
//    }
//
//    memset(&averages, 0, sizeof( FlightData ));
//
//    if( ! averageData(&averages) )
//    {
//        return flightState;
//    }

    switch ( flightState )
    {
        case FLIGHT_STATE_LAUNCHPAD:
        {
            if(data->inertial.updated)
            {
                if ( detectLaunch( data->inertial.data.accelerometer[ 0 ] ) )
                {
                    DISPLAY_LINE( "Detected Launch at %f; vertical acceleration = %f", data->inertial.data.timestamp, data->inertial.data.accelerometer[ 0 ] );
                    flightState = FLIGHT_STATE_PRE_APOGEE;
                }
            }

            return 0;
        }

        case FLIGHT_STATE_PRE_APOGEE:
        {
            if(data->inertial.updated)
            {
                if ( detectApogee( data->inertial.data.accelerometer[ 0 ], data->inertial.data.accelerometer[ 1 ],
                                   data->inertial.data.accelerometer[ 2 ] ) )
                {
                    DISPLAY_LINE( "Detected APOGEE at %f; vertical acceleration = %f", data->inertial.data.timestamp, data->inertial.data.accelerometer[ 0 ]);
                    flightState = FLIGHT_STATE_APOGEE;

                    // time,acceleration,pres,altMSL,temp,latxacc,latyacc,gyrox,gyroy,gyroz,magx,magy,magz,launch_detect,apogee_detect,Aon,Bon
//                        time         acc x     pres    altmsl  temp    acc_y        acc_z
//                    SRAD 3400.32	-0.183594	56235	4698.07	44.29	0.125	    -0.984375   -14.84	-30.17	1335.81	0.000386	-0.002148	0.000415	1	0	0	0 // 1.00912g
//                    COTS 3411.31	0.183594	53173	5118.01	44.37	0.296875	-0.519531	-9.87	-3.15	469.91	0.000562	-0.001695	0.000153	1	1	1	0 // 0.62590g
                }
            }

            return 0;
        }
        case FLIGHT_STATE_APOGEE:
        {
            DISPLAY_LINE( "Igniting recovery circuit drogue", NULL );
            flightState = FLIGHT_STATE_POST_APOGEE;

            return 0;
        }

        case FLIGHT_STATE_POST_APOGEE:
        {
            if(data->pressure.updated)
            {
                if ( detectAltitude( MAIN_CHUTE_ALTITUDE, GROUND_ALTITUDE, data->pressure.data.pressure ) )
                {
                    DISPLAY_LINE( "Detected Main Chute at %f; vertical acceleration = %I64d", data->inertial.data.timestamp, data->pressure.data.pressure);
                    flightState = FLIGHT_STATE_MAIN_CHUTE;
                }
            }

            return 0;
        }

        case FLIGHT_STATE_MAIN_CHUTE:
        {
            DISPLAY_LINE("Igniting recovery circuit main", NULL);
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
                    DISPLAY_LINE( "Detected landing at %f.", data->inertial.data.timestamp);
                    flightState = FLIGHT_STATE_LANDED;
                }
            }

            return 0;
        }
        case FLIGHT_STATE_LANDED:
        {
            DISPLAY_LINE("Rocket landed!", NULL);
            flightState = FLIGHT_STATE_EXIT;

            return 0;
        }

        case FLIGHT_STATE_EXIT:
        {
            DISPLAY_LINE( "Exit!", NULL );

            return 0;
        }
        case FLIGHT_STATE_COUNT:
            break;
    }

    return 1;
}


static bool detectLaunch    ( float vertical_acceleration )
{
    float vertical_acceleration_in_g = vertical_acceleration; // imu_sensor_acc2g(vertical_acceleration);

    return vertical_acceleration_in_g > CRITICAL_VERTICAL_ACCELERATION;
}


static bool detectApogee    ( float acceleration_x, float acceleration_y, float acceleration_z )
{
    float acceleration_x_in_g = acceleration_x; // imu_sensor_acc2g(acceleration_x);
    float acceleration_y_in_g = acceleration_y; // imu_sensor_acc2g(acceleration_y);
    float acceleration_z_in_g = acceleration_z; // imu_sensor_acc2g(acceleration_z);
    const float ACCELERATION_VECTOR = sqrt(acceleration_x_in_g*acceleration_x_in_g + acceleration_y_in_g*acceleration_y_in_g + acceleration_z_in_g*acceleration_z_in_g);

    return ACCELERATION_VECTOR < APOGEE_ACCELERATION;
}


static bool detectAltitude  ( float target_altitude, uint32_t ground_altitude, uint32_t current_pressure )
{
    float current_altitude = calculate_altitude( current_pressure ) - ground_altitude;
//    DISPLAY_LINE("Current Altitude: %d", current_altitude);

    return fabsf ( ( current_altitude ) - ( target_altitude ) ) < 5; /* 5 metres difference threshold */
}

static bool detectLanding   ( float gyro_x, float gyro_y, float gyro_z )
{
    float gyroscope_orientation_vector = sqrt( gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
    return gyroscope_orientation_vector < LANDING_ROTATION_SPEED; // imu_sensor_deg_per_sec2rot ( LANDING_ROTATION_SPEED );
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
