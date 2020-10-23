/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "mem.h"

#include <stdio.h>
#include <stdint.h>


#include "protocols/UART.h"
#include "board/components/flash.h"
#include "core/system_configuration.h"
#include "memory-management/memory_manager.h"


static bool cli_tools_mem_read                           (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_scan                           (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_erase_data_section             (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_erase_config_section           (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_erase_all                      (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_read_imu_index                 (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_read_press_index               (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_read_cont_index                (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_read_flight_event_index        (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);
static bool cli_tools_mem_read_configuration_index       (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg);


bool cli_tools_mem (char *pcWriteBuffer, size_t xWriteBufferLen, const char * cmd_option, const char * str_option_arg)
{
    if (strcmp(cmd_option, "mem") == 0)
    {
        sprintf(pcWriteBuffer, "%s", str_option_arg);
        return true;
    }

    if (strcmp(cmd_option, "read_imu_index") == 0)
        return cli_tools_mem_read_imu_index(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "read_press_index") == 0)
        return cli_tools_mem_read_press_index(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "read_cont_index") == 0)
        return cli_tools_mem_read_cont_index(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "read_flight_event_index") == 0)
        return cli_tools_mem_read_flight_event_index(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "read_configuration") == 0)
        return cli_tools_mem_read_configuration_index(pcWriteBuffer, xWriteBufferLen, "0");

    if (strcmp(cmd_option, "read") == 0)
        return cli_tools_mem_read(pcWriteBuffer, xWriteBufferLen, NULL);

    if (strcmp(cmd_option, "scan") == 0)
        return cli_tools_mem_scan(pcWriteBuffer, xWriteBufferLen, NULL);

    if (strcmp(cmd_option, "erase_data_section") == 0)
        return cli_tools_mem_erase_data_section(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "erase_config_section") == 0)
        return cli_tools_mem_erase_config_section(pcWriteBuffer, xWriteBufferLen, str_option_arg);

    if (strcmp(cmd_option, "erase_all") == 0)
        return cli_tools_mem_erase_all(pcWriteBuffer, xWriteBufferLen, NULL);

    sprintf(pcWriteBuffer, "Command [%s] not recognized\n", cmd_option);
    return false;
}

bool cli_tools_mem_read (char *pcWriteBuffer, size_t xWriteBufferLen, const char * str_option_arg)
{
    const char * cmd_option = "read";
    uint32_t value = atoi(str_option_arg);

    if(value > 0 && value <= FLASH_END_ADDRESS)
    {
        sprintf(pcWriteBuffer, "Reading 256 bytes starting at address %i ...\n", value);

        uint8_t data_rx[FLASH_PAGE_SIZE];
        FlashStatus stat;
        stat = flash_read(value, data_rx, FLASH_PAGE_SIZE);

        uint8_t busy = stat;

        while(FLASH_IS_DEVICE_BUSY(busy))
        {
            busy = flash_get_status_register();
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if(stat == FLASH_OK)
        {
            sprintf(pcWriteBuffer, "Success!\n");
        }
        else
        {
            sprintf(pcWriteBuffer, "Failed!\n");
            return false;
        }

        for(size_t i = 0; i < FLASH_PAGE_SIZE; i++)
        {
            if((i + 1) % 16 == 0)
            {
                sprintf(pcWriteBuffer, "0x%02X\n", data_rx[i]);
            }else
            {
                sprintf(pcWriteBuffer, "0x%02X\n", data_rx[i]);
            }
        }

        return true;
    }

    sprintf(pcWriteBuffer, "[%s]: Argument [%s] is in invalid range\n", cmd_option, str_option_arg);
    return false;
}


bool cli_tools_mem_scan (char *pcWriteBuffer, size_t xWriteBufferLen, const char * str_option_arg)
{
    const char * cmd_option = "scan";
    uint32_t value = atoi(str_option_arg);

    uint32_t end_Address = flash_scan();
    sprintf(pcWriteBuffer, "End address :%i \n", end_Address);

    return true;
}


bool cli_tools_mem_erase_data_section (char *pcWriteBuffer, size_t xWriteBufferLen, const char * str_option_arg)
{
    uint8_t dataRX[FLASH_PAGE_SIZE];

    sprintf(pcWriteBuffer, "Erasing data section ...\n");

    uint32_t address = FLASH_START_ADDRESS;
    FlashStatus stat = FLASH_ERR;

    while(address <= FLASH_END_ADDRESS)
    {

        if(address > FLASH_PARAM_END_ADDRESS)
        {
            stat = flash_erase_sector(address);
            address += FLASH_SECTOR_SIZE;
        }else
        {
            stat = flash_erase_param_sector(address);
            address += FLASH_PARAM_SECTOR_SIZE;
        }
        //Wait for erase to finish
        while(FLASH_IS_DEVICE_BUSY(stat))
        {

            stat = flash_get_status_register();

            vTaskDelay(pdMS_TO_TICKS(1));
        }

//        HAL_GPIO_TogglePin(USR_LED_PORT,USR_LED_PIN);
        sprintf(pcWriteBuffer, "Erasing sector %i ...\n", address);
    }

    flash_read(FLASH_START_ADDRESS, dataRX, 256);
    uint16_t empty = 0xFFFF;
    int i;
    for(i = 0; i < 256; i++)
    {

        if(dataRX[i] != 0xFF)
        {
            empty--;
        }
    }

    if(empty == 0xFFFF)
    {

        sprintf(pcWriteBuffer, "Flash Erased Success!\n");
    }

    if(stat == FLASH_OK)
    {
        sprintf(pcWriteBuffer, "Success!\n");
    }else
    {
        sprintf(pcWriteBuffer, "Failed!\n");
        return false;
    }

    return true;
}

bool cli_tools_mem_erase_config_section (char *pcWriteBuffer, size_t xWriteBufferLen, const char * str_option_arg)
{
    const char * cmd_option = "erase_config_section";
    sprintf(pcWriteBuffer, "Success!\n");
    return true;
}

bool cli_tools_mem_erase_all (char *pcWriteBuffer, size_t xWriteBufferLen, const char * str_option_arg)
{
    const char * cmd_option = "erase_all";
    sprintf(pcWriteBuffer, "Success!\n");
    return true;
}


static bool cli_tools_mem_read_imu_index (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg)
{
    const char * cmd_option = "read_imu_index";
    uint32_t value = atoi(str_option_arg);

    IMUDataU dst = {};
    if (MEM_OK == memory_manager_get_single_imu_entry(&dst, value) )
    {
        sprintf(pcWriteBuffer, "[%s]: timestamp=%i, accel=[%f, %f, %f], gyro=[%f, %f, %f]\n",
                cmd_option, dst.timestamp, dst.accelerometer[0], dst.accelerometer[1], dst.accelerometer[2],
                dst.gyroscope[0], dst.gyroscope[1], dst.gyroscope[2]);

        return true;
    }

    sprintf(pcWriteBuffer, "Failure!\n");
    return false;

}

static bool cli_tools_mem_read_press_index (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg)
{
    const char * cmd_option = "read_press_index";
    uint32_t value = atoi(str_option_arg);

    PressureDataU dst = {};
    if (MEM_OK == memory_manager_get_single_pressure_entry(&dst, value) )
    {
        sprintf(pcWriteBuffer, "[%s]: timestamp=%i, pressure=%f, temperature=%f\n", cmd_option, dst.timestamp, dst.pressure, dst.temperature);
        return true;
    }

    sprintf(pcWriteBuffer, "Failure!\n");
    return false;
}


static bool cli_tools_mem_read_cont_index (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg)
{
    const char * cmd_option = "read_cont_index";
    uint32_t value = atoi(str_option_arg);

    ContinuityU dst = {};
    if (MEM_OK == memory_manager_get_single_cont_entry(&dst, value) )
    {
        sprintf(pcWriteBuffer, "[%s]: timestamp=%i, status=%i\n", cmd_option, dst.timestamp, dst.status);
        return true;
    }

    sprintf(pcWriteBuffer, "Failure!\n");
    return false;
}

static bool cli_tools_mem_read_flight_event_index (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg)
{
    const char * cmd_option = "read_flight_event_index";
    uint32_t value = atoi(str_option_arg);

    FlightEventU dst = {};
    if (MEM_OK == memory_manager_get_single_flight_event_entry(&dst, value) )
    {
        sprintf(pcWriteBuffer, "[%s]: timestamp=%i, status=%i\n", cmd_option, dst.timestamp, dst.status);
        return true;
    }

    sprintf(pcWriteBuffer, "Failure!\n");
    return false;
}

static bool cli_tools_mem_read_configuration_index (char* pcWriteBuffer, size_t xWriteBufferLen, const char* str_option_arg)
{
    const char * cmd_option = "read_configuration_index";
    uint32_t value = atoi(str_option_arg);

    ConfigurationU dst = {};
    if (MEM_OK == memory_manager_get_single_configuration_entry(&dst, value) )
    {
        sprintf(pcWriteBuffer,
        "[%s]:\n"
        "magic = %s\n"
        "\n"
        "Memory:\n"
        " write_drogue_continuity_ms      = %i\n"
        " write_pre_launch_multiplier     = %i\n"
        " write_pre_apogee_multiplier     = %i\n"
        " write_post_apogee_multiplier    = %i\n"
        " write_ground_multiplier         = %i\n"
        " write_interval_accelerometer_ms = %i\n"
        " write_interval_gyroscope_ms     = %i\n"
        " write_interval_magnetometer_ms  = %i\n"
        " write_interval_pressure_ms      = %i\n"
        " write_interval_altitude_ms      = %i\n"
        " write_interval_temperature_ms   = %i\n"
        " write_interval_flight_state_ms  = %i\n"
        " write_drogue_continuity_ms      = %i\n"
        " write_main_continuity_ms        = %i\n"
        "\n"
        "System:\n"
        " landing_rotation_speed_deg_per_sec      = %i\n"
        " backup_time_launch_to_apogee_sec        = %i\n"
        " backup_time_apogee_to_main_sec          = %i\n"
        " backup_time_main_to_ground_sec          = %i\n"
        " ground_pressure                         = %i\n"
        " ground_temperature                      = %i\n"
        " current_system_time                     = %i\n"
        " altitude_main_recovery_m                = %i\n"
        " flight_state                            = %i\n"
        " power_mode                              = %i\n"
        " launch_acceleration_critical_value_m_s2 = %i\n"
        " e_match_line_keep_active_for            = %i\n"
        " imu_data_needs_to_be_converted          = %i\n"
        " pressure_data_needs_to_be_converted     = %i\n"
        "\n"
        "IMU:\n"
        " accel_bandwidth        = %i\n"
        " accel_output_data_rate = %i\n"
        " accel_range            = %i\n"
        " accel_power            = %i\n"
        " gyro_bandwidth         = %i\n"
        " gyro_output_data_rate  = %i\n"
        " gyro_range             = %i\n"
        " gyro_power             = %i\n"
        "\n"
        "Pressure:\n"
        " output_data_rate                             = %i\n"
        " temperature_oversampling                     = %i\n"
        " pressure_oversampling                        = %i\n"
        " infinite_impulse_response_filter_coefficient = %i\n",

        cmd_option,

        (const char*) dst.magic,

        dst.memory.write_drogue_continuity_ms,
        dst.memory.write_pre_launch_multiplier,
        dst.memory.write_pre_apogee_multiplier,
        dst.memory.write_post_apogee_multiplier,
        dst.memory.write_ground_multiplier,
        dst.memory.write_interval_accelerometer_ms,
        dst.memory.write_interval_gyroscope_ms,
        dst.memory.write_interval_magnetometer_ms,
        dst.memory.write_interval_pressure_ms,
        dst.memory.write_interval_altitude_ms,
        dst.memory.write_interval_temperature_ms,
        dst.memory.write_interval_flight_state_ms,
        dst.memory.write_drogue_continuity_ms,
        dst.memory.write_main_continuity_ms,

        dst.system.landing_rotation_speed_deg_per_sec,
        dst.system.backup_time_launch_to_apogee_sec,
        dst.system.backup_time_apogee_to_main_sec,
        dst.system.backup_time_main_to_ground_sec,
        dst.system.ground_pressure,
        dst.system.ground_temperature,
        dst.system.current_system_time,
        dst.system.altitude_main_recovery_m,
        dst.system.flight_state,
        dst.system.power_mode,
        dst.system.launch_acceleration_critical_value_m_s2,
        dst.system.e_match_line_keep_active_for,
        dst.system.imu_data_needs_to_be_converted,
        dst.system.pressure_data_needs_to_be_converted,

        dst.system.imu_sensor_configuration.accel_bandwidth,
        dst.system.imu_sensor_configuration.accel_output_data_rate,
        dst.system.imu_sensor_configuration.accel_range,
        dst.system.imu_sensor_configuration.accel_power,
        dst.system.imu_sensor_configuration.gyro_bandwidth,
        dst.system.imu_sensor_configuration.gyro_output_data_rate,
        dst.system.imu_sensor_configuration.gyro_range,
        dst.system.imu_sensor_configuration.gyro_power,

        dst.system.pressure_sensor_configuration.output_data_rate,
        dst.system.pressure_sensor_configuration.temperature_oversampling,
        dst.system.pressure_sensor_configuration.pressure_oversampling,
        dst.system.pressure_sensor_configuration.infinite_impulse_response_filter_coefficient
        );

        return true;
    }

    sprintf(pcWriteBuffer, "Failure!\n");
    return false;
}



