//
// Created by vasil on 07/02/2021.
//

#include "system_configuration.h"

const FlightSystemConfiguration DEFAULT_FLIGHT_SYSTEM_CONFIGURATION =
{
    // TODO: to be edited from GUI
    .landing_rotation_speed_deg_per_sec       = 0,
    .backup_time_launch_to_apogee_sec         = 27,
    .backup_time_apogee_to_main_sec           = 116,
    .backup_time_main_to_ground_sec           = 191,
    .e_match_line_keep_active_for             = 5,
    .launch_acceleration_critical_value_m_s2  = 7, /*6.9*/
    .altitude_main_recovery_m                 = 381,
    .ground_pressure                          = 0,
    .ground_temperature                       = 0,
    .imu_data_needs_to_be_converted           = 0,
    .pressure_data_needs_to_be_converted      = 0
};
