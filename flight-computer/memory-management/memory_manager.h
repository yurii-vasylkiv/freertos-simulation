#ifndef MEMORY_MANAGER_MEMORY_MANAGER_H
#define MEMORY_MANAGER_MEMORY_MANAGER_H

#include <inttypes.h>
#include "core/system_configuration.h"

typedef union
{
    struct{
        uint32_t timestamp;
        float accelerometer [ 3 ];
        float gyroscope     [ 3 ];
    };
    uint8_t bytes[sizeof(float) * 1   +
                  sizeof(float ) * 3   +
                  sizeof(float ) * 3
    ];
} IMUDataU;

typedef union
{
    struct{
        int64_t pressure;
    };
    uint8_t bytes[sizeof(int64_t)  * 1];
} GroundDataU;

typedef struct
{
    uint8_t updated;
    IMUDataU data;
} IMUData;


typedef union
{
    struct {
        uint32_t timestamp;
        float pressure;
        float temperature;
    };
    uint8_t bytes[sizeof(float)    * 1 +
                  sizeof(float)    * 1 +
                  sizeof(float)    * 1
    ];
} PressureDataU;

typedef struct
{
    uint8_t updated;
    PressureDataU data;
} PressureData;

typedef enum { Open      = 0, Short     = 1 } ContinuityStatus;
typedef enum { Launchpad = 0, PreApogee = 1, Apogee = 2, PostApogee = 3, MainChute = 4, PostMain = 5, Landed = 6, Exited = 7    } FlightEventStatus;
typedef enum { IMU       = 0, Pressure  = 1,  Cont   = 2, Event      = 3, SectorsCount } Sector;
typedef enum { MEM_ERR   = 0, MEM_OK    = 1 } MemoryStatus;

typedef union
{
    uint8_t updated;
    struct {
        uint32_t timestamp;
        ContinuityStatus status;
    };
    uint8_t bytes[sizeof(float) * 1 +
                  sizeof(uint8_t ) * 1
    ];
} ContinuityU;

typedef struct
{
    uint8_t updated;
    ContinuityU data;
} Continuity;

typedef union
{
    struct {
        uint32_t timestamp;
        FlightEventStatus status;
    };
    uint8_t bytes[sizeof(float) * 1 +
                  sizeof(uint8_t ) * 1
    ];
} FlightEventU;


typedef struct
{
    uint8_t updated;
    FlightEventU data;
} FlightEvent;

typedef struct
{
    uint32_t timestamp;
    IMUData inertial;
    PressureData pressure;
    Continuity continuity;
    FlightEvent event;
} Data;

typedef struct
{
    uint32_t startAddress;
    uint32_t endAddress;
    uint32_t bytesWritten;
} BufferInfo;

typedef struct
{
    uint32_t size;
    uint32_t startAddress;
    uint32_t endAddress;
    uint32_t bytesWritten;
} MemorySectorInfo;


typedef struct
{
    BufferInfo info;
    uint8_t data[256];
} MemoryBuffer;

typedef struct
{
    MemorySectorInfo info;
    MemoryBuffer buffers[2];
    MemoryBuffer *read;
    MemoryBuffer *write;
} MemoryDataSector;



enum {CONFIGURATION_DATA_SECTOR_BUFFER_COUNT = 1};
typedef struct
{
    MemorySectorInfo info;
    MemoryBuffer buffers[CONFIGURATION_DATA_SECTOR_BUFFER_COUNT];
    uint8_t current_buffer_index;
} MemoryConfigurationSector;

enum { Conf = 100 };

typedef struct
{
    // to keep memory alignment right using 4 of 8-bits integers
    // counted as one 32-bit integer
    // 4 bytes
    uint8_t  write_pre_launch_multiplier;             // +
    uint8_t  write_pre_apogee_multiplier;             // +
    uint8_t  write_post_apogee_multiplier;            // +
    uint8_t  write_ground_multiplier;                 // +
    // 4 bytes
    uint16_t write_interval_accelerometer_ms;         // +
    // 4 bytes
    uint16_t write_interval_gyroscope_ms;             // +
    uint16_t write_interval_magnetometer_ms;          // +
    // 4 bytes
    uint16_t write_interval_pressure_ms;              // +
    uint16_t write_interval_altitude_ms;              // +
    // 4 bytes
    uint16_t write_interval_temperature_ms;           // +
    uint16_t write_interval_flight_state_ms;          // +
    // 4 bytes
    uint16_t write_drogue_continuity_ms;              // +
    uint16_t write_main_continuity_ms;                // +
} MemoryManagerConfiguration;



typedef union
{
    struct{
        uint8_t magic[36];
        MemorySectorInfo data_sectors[SectorsCount]; // 4 * 3 * 4 = 48 bytes

        MemoryManagerConfiguration memory;
        FlightSystemConfiguration system;
    };

    uint8_t bytes[sizeof(uint8_t)          * 36             +
                  sizeof(MemorySectorInfo) * SectorsCount   +
                  sizeof(MemoryManagerConfiguration)        +
                  sizeof(FlightSystemConfiguration)
    ];

    // as long as this union is less then 256 bytes it is easy to handle, but then we will have to use multiple pages
    // to store this union at.
} ConfigurationU;


MemoryStatus memory_manager_init();
MemoryStatus memory_manager_configure();
MemoryStatus memory_manager_update(Data * _container);
MemoryStatus memory_manager_add_imu_update(IMUDataU *_container);
MemoryStatus memory_manager_add_pressure_update(PressureDataU *_container);
MemoryStatus memory_manager_add_continuity_update(ContinuityU *_container);
MemoryStatus memory_manager_add_flight_event_update(FlightEventU *_container);
MemoryStatus memory_manager_start();
MemoryStatus memory_manager_stop();
MemoryStatus memory_manager_get_system_configurations(FlightSystemConfiguration  * systemConfiguration);
MemoryStatus memory_manager_get_memory_configurations(MemoryManagerConfiguration * memoryConfiguration);
MemoryStatus memory_manager_set_system_configurations(FlightSystemConfiguration  * systemConfiguration);
MemoryStatus memory_manager_set_memory_configurations(MemoryManagerConfiguration * memoryConfiguration);

MemoryStatus memory_manager_get_single_pressure_entry(PressureDataU * dst, uint32_t entry_index );
MemoryStatus memory_manager_get_single_imu_entry(IMUDataU * dst, uint32_t entry_index );
MemoryStatus memory_manager_get_single_cont_entry(ContinuityU * dst, uint32_t entry_index );
MemoryStatus memory_manager_get_single_flight_event_entry(FlightEventU * dst, uint32_t entry_index );
MemoryStatus memory_manager_get_single_configuration_entry(ConfigurationU * dst, uint32_t entry_index );



#endif //MEMORY_MANAGER_MEMORY_MANAGER_H
