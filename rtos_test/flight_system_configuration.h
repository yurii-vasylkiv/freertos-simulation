#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS Google Drive: UMSATS/Guides and HowTos.../Command and Data Handling (CDH)/Coding Standards
//
// File Description:
//  Template header file for C / C++ projects. Unused sections can be deleted.
//
// History
// 2019-05-27 by Joseph Howarth
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <inttypes.h>

//Defaults for the configuration options.
#define ID                              0x5A

#define DATA_RATE                       50
#define INITIAL_WAIT_TIME               10000       // in milliseconds
#define FLAGS                           0x00        // default not in flight, not recording.
#define DATA_START_ADDRESS              0x00001000  // Start writing to second page of memory.
#define DATA_END_ADDRESS                0x00001000  // Assume no saved data.



#define GND_ALT                         0
#define GND_PRES                        101325

//Macros to get flags.
#define CONFIGURATION_IS_IN_FLIGHT(x)   ((x>>0)&0x01)
#define CONFIGURATION_IS_RECORDING(x)   ((x>>1)&0x01)
#define CONFIGURATION_IS_PRE_DROGUE(x)  ((x>>2)&0x01)
#define CONFIGURATION_IS_POST_DROGUE(x) ((x>>3)&0x01)
#define CONFIGURATION_IS_POST_MAIN(x)   ((x>>4)&0x01)


typedef struct
{
    // 4 bytes
    uint32_t landing_rotation_speed_deg_per_sec;      // -
    uint32_t backup_time_launch_to_apogee_sec;        // -
    uint32_t backup_time_apogee_to_main_sec;          // -
    uint32_t backup_time_main_to_ground_sec;          // -

    uint32_t ground_pressure;                         // -
    uint32_t current_system_time;                     // -
    uint16_t altitude_main_recovery_m;                // -

    uint8_t flight_state;                             // -
    uint8_t power_mode;                               // -
    uint8_t launch_acceleration_critical_value_m_s2;  // -
    uint8_t e_match_line_keep_active_for;             // -
} FlightSystemConfiguration;




#endif // CONFIGURATION_H
