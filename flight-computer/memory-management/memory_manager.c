#include "memory_manager.h"

#include <stdio.h>
#include <memory.h>
#include <stdbool.h>
#include <string.h>
#include "queue.h"
#include <assert.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>
#include "protocols/UART.h"
#include "utilities/common.h"
#include "board/components/flash.h"


/* ------------------- This memory manager ic designed for Flash Memory Cypress S25FL064P0XMFA000 ------------------- */
/* The layout of the memory is designed to support recording the real-time IMU (Bosch Sensortec BMI088) and Pressure
 * (Bosch Sensortec BMP388) sensor measurements as well as the flags such as continuity circuit and flight events (e.g.
 * launchpad, pre-apogee, apogee, post-apogee, etc.)
 *
 * The following NOR flash memory chip consists of 128 uniform 64kB
 * sectors with the two (Top or Bottom) 64 kB sectors further split up into thirty-two 4 kB sub sectors, that is in total
 * 8,388,608b --- 8,192Kb -- 8Mb of memory.
 *
 * In order to fit all the data into the memory the memory manager splits the entire memory into chunks called data
 * sectors that store:
 * 1. Global configurations (system-wise and memory-manager-wise),
 * 2. Measurements of the two sensors: IMU and Pressure
 * 3. Two system state flags: continuity circuit state and flight event state.
 *
 * NOTE:
 * One limitation of flash memory is that, although it can be read or programmed a byte or a word at a time in a random
 * access fashion, it can be erased only a block at a time. This generally sets all bits in the block to 1. Starting with
 * freshly erased block, any location within that block can be programmed. However, once a bit has been set to 0, only by
 * erasing the entire block can it be changed back to 1.
 *
 * [1. Global configurations data sector]
 * Because we can only erase in blocks we should avoid erasing for as long as possible. Therefore, since the first data
 * sector stores global configurations that is meant to be mutable (where a user can change the configuration values
 * dynamically in run-time through a command line interface), it was decided to allocate as much as 8,192b -- 8Kb of
 * memory for this section. Given that 8Kb contains 32 of 256-byte pages and configuration update takes about 70 bytes,
 * this then should fit roughly 95 configuration updates that are sequentially placed in memory.
 * For example:
 * [ [original_config] [config_update1] [config_update_2] [config_update3] [config_update4] ... [config_update225] ]
 *
 * Only the last update is to be used in the system while all the previous updates become unusable, but cannot be deleted
 * until we manually erase the entire 8Kb block. The system must not reach the end of the sector, otherwise undefined
 * behaviour is expected.
 *
 * [2, 3. Measurements of the two sensors (IMU and Pressure), Two system state flags (continuity circuit and flight event)]
 * The following four data sectors each takes 16 uniform 64 Kb sectors, that is 1,048,576b -- 1,024Kb -- 1Mb of memory.
 * These are the primary data sectors and they are meant to store the most of the information generated during the flight
 *     ____________________________________________________________________________________________________________
 *     |             |           |           |           | approximate size  | est. updates |       estimated     |
 *     | Data Sector | size (b)  |  offset   |  # pages  | of an update (b)  |   per page   |  updates per sector |
 *     |_____________|___________|___________|___________|___________________|______________|_____________________|
 *     |     IMU     | 1,048,576 |    4096   |    4096   |        28         |      9       |       36,864        |
 *     |_____________|___________|___________|___________|___________________|______________|_____________________|
 *     | Pressure    | 1,048,576 |    4096   |    4096   |        12         |      21      |       86,016        |
 *     |_____________|___________|___________|___________|___________________|______________|_____________________|
 *     | Continuity  | 1,048,576 |    4096   |    4096   |         5         |      51      |       204,800       |
 *     |_____________|___________|___________|___________|___________________|______________|_____________________|
 *     | FlightEvent | 1,048,576 |    4096   |    4096   |         5         |      51      |       204,800       |
 *     |_____________|___________|___________|___________|___________________|______________|_____________________|
 *
 * The first data sector is IMU:
 * */


// define the basic information about the metadata sector
#define RESERVED_SECTORS_BASE_ADDRESS                   0
#define RESERVED_SECTORS_COUNT                          2
#define RESERVED_SECTOR_SUB_SIZE                        FLASH_4KB_SUBSECTOR_SIZE // or 16 pages

// define the basic information about the global configuration sector
#define GLOBAL_CONFIGURATION_SECTOR_BASE                RESERVED_SECTORS_BASE_ADDRESS
#define GLOBAL_CONFIGURATION_SECTOR_SUB_COUNT           1 // 4KB

#define GLOBAL_CONFIGURATION_SECTOR_SIZE                RESERVED_SECTORS_BASE_ADDRESS + RESERVED_SECTOR_SUB_SIZE * GLOBAL_CONFIGURATION_SECTOR_SUB_COUNT // 4KB
#define GLOBAL_CONFIGURATION_SECTOR_OFFSET              RESERVED_SECTORS_BASE_ADDRESS + GLOBAL_CONFIGURATION_SECTOR_SIZE

#define MEMORY_METADATA_SECTOR_BASE                     GLOBAL_CONFIGURATION_SECTOR_OFFSET
#define MEMORY_METADATA_SECTOR_SUB_COUNT                512 // 2 MB

#define MEMORY_METADATA_SECTOR_SIZE                     RESERVED_SECTORS_BASE_ADDRESS + RESERVED_SECTOR_SUB_SIZE * MEMORY_METADATA_SECTOR_SUB_COUNT // 2 MB
#define MEMORY_METADATA_SECTOR_OFFSET                   GLOBAL_CONFIGURATION_SECTOR_OFFSET + MEMORY_METADATA_SECTOR_SIZE

// define the basic information about the data sectors
#define DATA_SECTORS_BASE                               MEMORY_METADATA_SECTOR_OFFSET
#define PAGE_SIZE                                       FLASH_PAGE_SIZE

#define IMU_ENTRIES_PER_PAGE                            ( (int) ( PAGE_SIZE / sizeof ( IMUDataU  ) ) )
#define PRESSURE_ENTRIES_PER_PAGE                       ( (int) ( PAGE_SIZE / sizeof ( PressureDataU ) ) )
#define TEMPERATURE_ENTRIES_PER_PAGE                    ( (int) ( PAGE_SIZE / sizeof ( TemperatureDataU ) ) )

#define CONTINUITY_ENTRIES_PER_PAGE                     ( (int) ( PAGE_SIZE / sizeof ( ContinuityU ) ) )
#define FLIGHT_EVENT_ENTRIES_PER_PAGE                   ( (int) ( PAGE_SIZE / sizeof ( FlightEventU ) ) )

#define GLOBAL_CONFIGURATION_ENTRIES_PER_PAGE           ( (int) ( PAGE_SIZE / sizeof ( GlobalConfigurationU ) ) )
#define MEMORY_METADATA_ENTRIES_PER_PAGE                ( (int) ( PAGE_SIZE / sizeof ( MemoryLayoutMetaDataU ) ) )

#define toUserDataSector( memory_sector ) ( UserDataSector ) memory_sector - 2
#define   toSystemSector( memory_sector ) ( SystemSector   ) memory_sector
#define   toMemorySector( user_sector ) ( MemorySector   ) user_sector + 2

// signature sequence used as an identification of the flash memory data validity
// the sequence is written at the beginning of the first or the second 4KB subsector of the flash memory
const char * MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE = "6e2201ac6e0d";
#define MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE_BUFFER_LENGTH                           12

// Variable used to control the initialization process of the memory manager to prevent any actions if this flag is not set
static bool prvIsInitialized = { 0 };
static bool prvIsConfigured  = { 0 };

// a circular buffer queue is a processing queue that is used to temporarily store the pages of information to be fetched by a flash write monitor
static buffer_queue prvPageBuffer = { 0 };

#define METADATA_AUTOSAVE_DATA_BASED_INTERVAL                                           200
#define METADATA_AUTOSAVE_TIME_BASED_INTERVAL                                           pdMS_TO_TICKS(250) // milliseconds to ticks

// used as an up-to-date representation of the meta data sector to be written to flash
// the state of this structure is written to flash for every X any sensor (IMU, Pressure) data updates or every N time
// where X equals to whatever number macro METADATA_AUTOSAVE_DATA_BASED_INTERVAL is.
// where N equals to whatever number macro METADATA_AUTOSAVE_TIME_BASED_INTERVAL is.
// The check is performed in prvMemoryAddNewUserDataSectorEntry()
static MemoryLayoutMetaDataContainer prvMemoryMetaDataFlashSnapshot = { 0 };

// used as a counter that once it reaches CONFIGURATION_AUTOSAVE_INTERVAL, prvGlobalConfigurationDiskSnapshot is then sent to the
// flash write monitor processing queue (prvPageBuffer)
static size_t prvMetadataAutosaveDataBasedCounter = { 0 };
static size_t prvMetadataAutosaveTimeBasedCounter = { 0 };

static MetaDataUpdateFrequencyMode prvMetaDataUpdateMode = MetaDataUpdateTimeBasedFrequencyMode;

// used as an up-to-date representation of the global configuration data sector to be written to flash
// the state of this structure must be written to flash before the flight ONLY
static GlobalConfigurationContainer prvGlobalConfigurationDiskSnapshot = { 0 };


// used to hold the writing position addresses updated after each write to flash operation
// so that with each subsequent call of prvMemoryAddNewUserDataSectorEntry() the memory manager logic could place the data into the right
// position in the temporary buffer, and thus, into the correct position on flash memory later
static MemorySectorBuffer prvCurrentUserDataMemorySectorRAMBuffers[UserDataSectorCount]     = { 0 };
static BufferInfo         prvCurrentSystemMemorySectorDataPointersOnDisk[SystemSectorCount] = { 0 };

static uint8_t     is_queue_monitor_running  = { 0 };
static xTaskHandle prvQueueMonitorTaskHandle = { 0 };


static int prvMemorySectorFindLastWrittenPageIndexResults[MemorySectorCount] = { 0 };


static const MemoryManagerConfiguration prvDefaultMemoryManagerConfiguration = {
        // TODO: to be edited from GUI
        .write_pre_launch_multiplier         = 0,
        .write_pre_apogee_multiplier         = 0,
        .write_post_apogee_multiplier        = 0,
        .write_ground_multiplier             = 0,
        .write_interval_accelerometer_ms     = 0,
        .write_interval_gyroscope_ms         = 0,
        .write_interval_magnetometer_ms      = 0,
        .write_interval_pressure_ms          = 0,
        .write_interval_altitude_ms          = 0,
        .write_interval_temperature_ms       = 0,
        .write_interval_flight_state_ms      = 0,
        .write_drogue_continuity_ms          = 0,
        .write_main_continuity_ms            = 0,

        .user_data_sector_sizes              = {
                0xE00000  /* Gyro  - 14MB  */,
                0xE00000  /* Accel - 14MB  */,
                0x04EC00  /* Mag   - 315KB */,
                0x07E400  /* Press - 505KB */,
                0x07E400  /* Temp  - 505KB */,
                0x028000  /* Cont  - 160KB */,
                0x028000  /* Temp  - 160KB */
        }

};

MemoryManagerConfiguration memory_manager_get_default_memory_configurations ( )
{
    return prvDefaultMemoryManagerConfiguration;
}

void memory_manager_set_metadata_update_mode ( MetaDataUpdateFrequencyMode mode )
{
    prvMetaDataUpdateMode = mode;
}

static uint32_t prvMemorySectorGetSize ( MemorySector sector );

static uint32_t prvMemorySectorGetDataEntriesPerPage ( MemorySector sector );

static uint32_t prvMemorySectorGetDataStructSize ( MemorySector sector );

static uint32_t prvMemorySectorGetAlignedDataStructSize ( MemorySector sector );

static MemoryManagerStatus prvMemorySectorFindLastWrittenPageIndex ( MemorySector sector, uint32_t * result );

static MemoryManagerStatus prvMemoryAddNewUserDataSectorEntry ( UserDataSector sector, uint8_t * buffer );

static void prvMemoryAsyncWriteAnyMemorySectorIfAvailable ( );

static void prvQueueMonitorTask ( void * arg );

static MemoryManagerStatus prvMemoryWriteAsyncMetaDataSector ( );

static MemoryManagerStatus prvMemoryWriteAsyncGlobalConfigurationSector ( );


static MemoryManagerStatus prvVerifySystemSectorIntegrity ( SystemSector sector, uint8_t * data, bool * status );

static MemoryManagerStatus prvMemoryAccessPage ( MemorySector sector, int64_t pageIndex, uint8_t * dest );

static MemoryManagerStatus prvMemoryWritePageNow ( MemorySector sector, uint8_t * data );

static MemoryManagerStatus prvMemoryAccessSectorSingleDataEntry ( MemorySector sector, uint32_t index, void * dst );

static MemoryManagerStatus prvMemoryAccessLastDataEntry ( MemorySector sector, void * dst );


MemoryManagerStatus memory_manager_init ( ) /* noexcept */
{

    // can be initialized only once
    if ( prvIsInitialized == true )
    {
        return MEM_ERR;
    }

    // Initialize the memory sector *read and *write pointers for their safe & correct functioning
    for ( UserDataSector sector = UserDataSectorGyro; sector < UserDataSectorCount; sector++ )
    {
        if ( prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write == NULL )
        {
            prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write = &prvCurrentUserDataMemorySectorRAMBuffers[ sector ].buffers[ 0 ];
        }

        if ( prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read == NULL )
        {
            prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read = &prvCurrentUserDataMemorySectorRAMBuffers[ sector ].buffers[ 1 ];
        }
    }

    // important part of initialization is the buffer queue that will hold a page of information to be written
    // to the flash memory
    buffer_queue_init ( &prvPageBuffer );

    // initialization flag
    prvIsInitialized = true;

    // returns OK status
    return MEM_OK;
}


MemoryManagerStatus memory_manager_configure ( )
{
    // only allow if memory_manager_init() was called before
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    bool isIntegrityOK = false;

    // then find out whether the fetched meta configuration is a valid meta configuration subsector
    if ( MEM_OK != prvVerifySystemSectorIntegrity ( SystemSectorGlobalConfigurationData, prvGlobalConfigurationDiskSnapshot.data.bytes, &isIntegrityOK ) )
    {
        return MEM_ERR;
    }

    if ( isIntegrityOK == true )
    {
        // then prvGlobalConfigurationDiskSnapshot already holds correct data and we must not modify it at this point
        // Thus, we only update the pointer to the data on disk
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorGlobalConfigurationData ].bytesWritten = prvMemorySectorFindLastWrittenPageIndexResults[ MemorySystemSectorGlobalConfigurationData ] * PAGE_SIZE;
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ].startAddress  = GLOBAL_CONFIGURATION_SECTOR_BASE;
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ].endAddress    = GLOBAL_CONFIGURATION_SECTOR_OFFSET;
    }
    else
    {
        // in case if memory was erased, flash memory sets all bits high, and we cannot use such data with ASCII table.
        // Thus, we need to set everything we read to zeros
        memset ( &prvGlobalConfigurationDiskSnapshot, 0, sizeof ( GlobalConfigurationContainer ) );

        // meta configuration sector is invalid and thus either the data was invalidated or the memory was erased
        // ---- initiate a fresh start ------
        prvGlobalConfigurationDiskSnapshot.updated            = true;
        prvGlobalConfigurationDiskSnapshot.data.values.memory = prvDefaultMemoryManagerConfiguration;
        prvGlobalConfigurationDiskSnapshot.data.values.system = get_default_system_configuration ( );
        memcpy ( prvGlobalConfigurationDiskSnapshot.data.values.signature, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE_BUFFER_LENGTH );
        prvGlobalConfigurationDiskSnapshot.updated = 1;
    }

    // then find out whether the fetched meta configuration is a valid meta configuration subsector
    if ( MEM_OK != prvVerifySystemSectorIntegrity ( SystemSectorUserDataSectorMetaData, prvMemoryMetaDataFlashSnapshot.data.bytes, &isIntegrityOK ) )
    {
        return MEM_ERR;
    }

    if ( isIntegrityOK == true )
    {
        // then prvMemoryMetaDataFlashSnapshot already holds correct data and we must not modify it at this point
        // Thus, we only update the pointer to the data on disk
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ].bytesWritten = prvMemorySectorFindLastWrittenPageIndexResults[ MemorySystemSectorUserDataSectorMetaData ] * PAGE_SIZE;
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ].startAddress = MEMORY_METADATA_SECTOR_BASE;
        prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ].endAddress   = MEMORY_METADATA_SECTOR_OFFSET;
    }
    else
    {
        // meta configuration sector is invalid and thus either the data was invalidated or the memory was erased
        // ---- initiate a fresh start ------

        // in case if memory was erased, flash memory sets all bits high, and we cannot use such data with ASCII table.
        // Thus, we need to set everything we read to zeros
        memset ( &prvMemoryMetaDataFlashSnapshot, 0, sizeof ( MemoryLayoutMetaDataContainer ) );

        // for user data sectors
        int32_t              globalOffset = DATA_SECTORS_BASE;
        for ( UserDataSector sectorIndex  = UserDataSectorGyro; sectorIndex < UserDataSectorCount; sectorIndex++ )
        {

            MemorySectorInfo * sectorInfo = &prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ sectorIndex ];
            sectorInfo->startAddress = globalOffset;
            sectorInfo->size         = prvDefaultMemoryManagerConfiguration.user_data_sector_sizes[ sectorIndex ];

            globalOffset += sectorInfo->size;

            // it is a data sector, so we initialize it properly using the right offsets and sizes
            sectorInfo->endAddress   = sectorInfo->startAddress + sectorInfo->size;
            sectorInfo->bytesWritten = 0; /* nothing has been written to this memory sector, hence — 0 */
        }

        // for reserved sectors
        // we need to fill in the information about the meta configuration sector from scratch
        BufferInfo * blockInfo = &prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorGlobalConfigurationData ];
        blockInfo->startAddress = GLOBAL_CONFIGURATION_SECTOR_BASE;
        blockInfo->endAddress   = GLOBAL_CONFIGURATION_SECTOR_OFFSET;
        blockInfo->bytesWritten = 0; /* nothing has been written to this memory sector, hence — 0 */

        blockInfo = &prvCurrentSystemMemorySectorDataPointersOnDisk[ SystemSectorUserDataSectorMetaData ];
        blockInfo->startAddress = MEMORY_METADATA_SECTOR_BASE;
        blockInfo->endAddress   = MEMORY_METADATA_SECTOR_OFFSET;
        blockInfo->bytesWritten = 0; /* nothing has been written to this memory sector, hence — 0 */

        // storing the signature number
        memcpy ( prvMemoryMetaDataFlashSnapshot.data.values.signature, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE_BUFFER_LENGTH );

        // so that this state is put into the processing queue of the flash write monitor right away
        prvMemoryMetaDataFlashSnapshot.updated = 1;
    }

    prvIsConfigured = true;

    return MEM_OK;
}

MemoryManagerStatus memory_manager_start ( )
{
    if ( !prvIsInitialized )
    {
        return MEM_ERR;
    }

    if ( pdFALSE == xTaskCreate ( prvQueueMonitorTask, "mem-manager", configMINIMAL_STACK_SIZE, NULL, 5, &prvQueueMonitorTaskHandle ) )
    {
        return MEM_ERR;
    }

    return MEM_OK;
}

MemoryManagerStatus memory_manager_stop ( )
{
    is_queue_monitor_running = 0;
    return MEM_OK;
}

MemoryManagerStatus memory_manager_user_data_update ( DataContainer * _container )
{
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    if ( _container == NULL )
    {
        return MEM_ERR;
    }

    if ( _container->gyro.updated )
    {
        memory_manager_add_gyro_update ( &_container->gyro.data );
        _container->gyro.updated = 0;
    }

    if ( _container->acc.updated )
    {
        memory_manager_add_accel_update ( &_container->acc.data );
        _container->acc.updated = 0;
    }

    if ( _container->mag.updated )
    {
        memory_manager_add_mag_update ( &_container->mag.data );
        _container->mag.updated = 0;
    }

    if ( _container->press.updated )
    {
        memory_manager_add_pressure_update ( &_container->press.data );
        _container->press.updated = 0;
    }

    if ( _container->temp.updated )
    {
        memory_manager_add_temp_update ( &_container->temp.data );
        _container->temp.updated = 0;
    }

    if ( _container->cont.updated )
    {
        memory_manager_add_continuity_update ( &_container->cont.data );
        _container->cont.updated = 0;
    }

    if ( _container->event.updated )
    {
        memory_manager_add_flight_event_update ( &_container->event.data );
        _container->event.updated = 0;
    }

    // of course we are not forgetting to check on the metadata entry whether it is the time to flush it onto the disk
    if ( prvMetaDataUpdateMode == MetaDataUpdateDataBasedFrequencyMode )
    {
        prvMetadataAutosaveDataBasedCounter++;
        if ( prvMetadataAutosaveDataBasedCounter >= METADATA_AUTOSAVE_DATA_BASED_INTERVAL )
        {
            // it is time to update the configurations: signaling the monitor to process the metadata buffer
            prvMemoryWriteAsyncMetaDataSector ( );
            // reset the timer
            prvMetadataAutosaveDataBasedCounter = 0;
        }
    }

    return MEM_OK;
}

MemoryManagerStatus memory_manager_get_system_configurations ( FlightSystemConfiguration * systemConfiguration )
{
    if ( systemConfiguration == NULL )
    {
        return MEM_ERR;
    }

    memcpy ( systemConfiguration, &prvGlobalConfigurationDiskSnapshot.data.values.system, sizeof ( FlightSystemConfiguration ) );

    return MEM_OK;
}

MemoryManagerStatus memory_manager_set_system_configurations ( FlightSystemConfiguration * systemConfiguration )
{
    if ( systemConfiguration == NULL )
    {
        return MEM_ERR;
    }

    memset ( &prvGlobalConfigurationDiskSnapshot.data.values.system, 0, sizeof ( FlightSystemConfiguration ) );
    prvGlobalConfigurationDiskSnapshot.data.values.system = *systemConfiguration;

    prvMemoryWriteAsyncGlobalConfigurationSector ( );
    return MEM_OK;
}

MemoryManagerStatus memory_manager_get_memory_configurations ( MemoryManagerConfiguration * memoryConfiguration )
{
    if ( memoryConfiguration == NULL )
    {
        return MEM_ERR;
    }

    memcpy ( memoryConfiguration, &prvGlobalConfigurationDiskSnapshot.data.values.memory, sizeof ( MemoryManagerConfiguration ) );

    return MEM_OK;
}

MemoryManagerStatus memory_manager_set_memory_configurations ( MemoryManagerConfiguration * memoryConfiguration )
{
    if ( memoryConfiguration == NULL )
    {
        return MEM_ERR;
    }

    memset ( &prvGlobalConfigurationDiskSnapshot.data.values.memory, 0, sizeof ( MemoryManagerConfiguration ) );
    prvGlobalConfigurationDiskSnapshot.data.values.memory = *memoryConfiguration;

    prvMemoryWriteAsyncGlobalConfigurationSector ( );

    return MEM_OK;
}

MemoryManagerStatus memory_manager_erase_configuration_section ( )
{
    if ( FLASH_OK != flash_erase_4Kb_subsector ( GLOBAL_CONFIGURATION_SECTOR_BASE ) )
    {
        return MEM_ERR;
    }

    return MEM_OK;
}

MemoryManagerStatus memory_manager_erase_everything ( )
{
    if ( FLASH_OK != flash_erase_device ( ) )
    {
        return MEM_ERR;
    }

    return MEM_OK;
}


MemoryManagerStatus prvVerifySystemSectorIntegrity ( SystemSector sector, uint8_t * data, bool * status )
{
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    if ( data == NULL )
    {
        return MEM_ERR;
    }

    *status = false;

    MemoryBuffer buffer = { };
    // try reading the metadata from flash before all (we first try the very first subsector)
    if ( MEM_OK != prvMemoryAccessPage ( ( MemorySector ) sector, -1, buffer.data ) )
    {
        return MEM_ERR;
    }

    memcpy ( data, buffer.data, prvMemorySectorGetDataStructSize ( ( MemorySector ) sector ) );


    // and then check whether that subsector is valid
    if ( memcmp ( data, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE_BUFFER_LENGTH ) == 0 ) /* if signature sequences match */
    {
        *status = true;
    }
    else
    {
        *status = false;
    }

    return MEM_OK;
}


MemoryManagerStatus prvMemoryAddNewUserDataSectorEntry ( UserDataSector sector, uint8_t * buffer )
{
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    if ( buffer == NULL )
    {
        return MEM_ERR;
    }

    uint32_t size = prvMemorySectorGetDataStructSize ( toMemorySector ( sector ) );

    int page_aligned_boundary = prvMemorySectorGetAlignedDataStructSize ( toMemorySector ( sector ) );
    // check whether the number of data entries filled up the page size, such that no more entries of this data type
    // will fit into the page size without over-fitting.
    if ( prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write->info.bytesWritten >= page_aligned_boundary )
    {
        // if there is no more room for another data entry then we signal to the monitor that it is time to take out this
        // page and flush it ont ot the memory. To do that we swap the read and write buffer, so that the read buffer now
        // contains that full page of data (write buffer), while the write buffer should be come empty and ready for new data
        MemoryBuffer * temp_read = prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read;
        prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read  = prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write;
        prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write = temp_read;

        // here's the signal to the monitor to process that page.
        prvMemoryAsyncWriteAnyMemorySectorIfAvailable ( );

        // safeguard check: if the read buffer that was supposed to be processed by the monitor has actually been already processed or not yet
        if ( temp_read->info.bytesWritten >= size )
        {
            // if not then the monitor did not manage to do it before we landed here. Maybe the producing data is faster than its consuming?
            printf ( "The page was not saved. Probably the monitor has stuck, or it ia just too slow for the data producer. Overwriting the page...\n" );
            // resetting the current data pointer
            prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write->info.bytesWritten = 0;
        }
    }

    // now that we emptied the write buffer and it is newly fresh and ready, we are copying the incoming data entry to it
    MemoryBuffer * currentBuffer           = prvCurrentUserDataMemorySectorRAMBuffers[ sector ].write;
    uint8_t      * currWriteBufferPosition = &currentBuffer->data[ currentBuffer->info.bytesWritten ];
    memcpy ( currWriteBufferPosition, buffer, size );
    // and updating its current data pointer
    currentBuffer->info.bytesWritten += size;

    return MEM_OK;
}

MemoryManagerStatus prvMemoryWriteAsyncMetaDataSector ( )
{
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    buffer_item item = { };
    item.type = MemorySystemSectorUserDataSectorMetaData;
    memmove ( item.data, prvMemoryMetaDataFlashSnapshot.data.bytes, prvMemorySectorGetDataStructSize ( MemorySystemSectorUserDataSectorMetaData ) );
    buffer_queue_push_back ( &prvPageBuffer, &item );
//    xTaskGenericNotify(handle, 0, eNoAction, NULL);
    return MEM_OK;
}

static MemoryManagerStatus prvMemoryWriteAsyncGlobalConfigurationSector ( )
{
    if ( prvIsInitialized == false )
    {
        return MEM_ERR;
    }

    buffer_item item = { };
    item.type = MemorySystemSectorGlobalConfigurationData;
    memmove ( item.data, prvGlobalConfigurationDiskSnapshot.data.bytes, prvMemorySectorGetDataStructSize ( MemorySystemSectorGlobalConfigurationData ) );
    buffer_queue_push_back ( &prvPageBuffer, &item );
//    xTaskGenericNotify(handle, 0, eNoAction, NULL);
    return MEM_OK;
}

void prvMemoryAsyncWriteAnyMemorySectorIfAvailable ( )
{
    // free the read page
    for ( UserDataSector sector = UserDataSectorGyro; sector < UserDataSectorCount; sector++ )
    {
        int page_aligned_boundary = prvMemorySectorGetAlignedDataStructSize ( toMemorySector( sector ) );
        if ( prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read->info.bytesWritten >= page_aligned_boundary )
        {
            buffer_item item = { };
            item.type = toMemorySector ( sector );
            memmove ( item.data, prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read->data, PAGE_SIZE );
            prvCurrentUserDataMemorySectorRAMBuffers[ sector ].read->info.bytesWritten = 0;
            buffer_queue_push_back ( &prvPageBuffer, &item );
            // xTaskGenericNotify(handle, 0, eNoAction, NULL);
        }
    }
}

MemoryManagerStatus prvMemoryWritePageNow ( MemorySector sector, uint8_t * data )
{
    if ( sector == MemorySystemSectorGlobalConfigurationData )
    {

#if ( userconf_FREE_RTOS_SIMULATOR_MODE_ON == 0 )

        memory_manager_erase_configuration_section ( );
        FlashStatus status = flash_write ( GLOBAL_CONFIGURATION_SECTOR_BASE, data, PAGE_SIZE );
        if ( status != FLASH_OK )
        {
            return MEM_ERR;
        }

#else
        uint32_t bytesWritten = flash_write ( GLOBAL_CONFIGURATION_SECTOR_BASE, data, PAGE_SIZE );
        if ( bytesWritten != PAGE_SIZE)
        {
            return MEM_ERR;
        }
#endif

    }

    else if ( sector == MemorySystemSectorUserDataSectorMetaData )
    {
        SystemSector systemSector = toSystemSector ( sector );

        const uint32_t OFFSET = MEMORY_METADATA_SECTOR_BASE + prvCurrentSystemMemorySectorDataPointersOnDisk[ systemSector ].bytesWritten;

#if ( userconf_FREE_RTOS_SIMULATOR_MODE_ON == 0 )

        FlashStatus status = flash_write ( OFFSET, data, PAGE_SIZE );
        if ( status != FLASH_OK )
        {
            return MEM_ERR;
        }

//        ConfigurationU dst = {};
//        if (MEM_OK != memory_manager_get_single_configuration_entry (&dst, 0) )
//        {
//            return MEM_ERR;
//        }

#else
        uint32_t bytesWritten = flash_write( OFFSET, data, PAGE_SIZE );
        if ( bytesWritten != PAGE_SIZE)
        {
            return MEM_ERR;
        }
#endif

        prvCurrentSystemMemorySectorDataPointersOnDisk[ systemSector ].bytesWritten += PAGE_SIZE;
    }
    else
    {
        UserDataSector   userDataSector = toUserDataSector ( sector );
        MemorySectorInfo sectorInfo     = prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ userDataSector ];
        const uint32_t   OFFSET         = sectorInfo.startAddress + sectorInfo.bytesWritten;

#if ( userconf_FREE_RTOS_SIMULATOR_MODE_ON == 0 )

        FlashStatus status = flash_write ( OFFSET, data, PAGE_SIZE );
        if ( status != FLASH_OK )
        {
            return MEM_ERR;
        }
#else
        uint32_t bytesWritten = flash_write( OFFSET, data, PAGE_SIZE );
        if ( bytesWritten != PAGE_SIZE)
        {
            return MEM_ERR;
        }
#endif
        // now start address should point to a page size away from the previous one
        prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ userDataSector ].bytesWritten += PAGE_SIZE;

        // reset the read RAM buffer cursor
        // flagging to the other components that this piece of data has been already processed and emptied
        memset ( prvCurrentUserDataMemorySectorRAMBuffers[ userDataSector ].read, 0, PAGE_SIZE );
    }

    return MEM_OK;
}

/**
 * @param sector: desired memory sector
 * @param pageIndex: -1 for the last available page in the given sector, 0-N to indicate the page index to be retrieved
 * @param dest: destination buffer (Note: MUST be at least 255 bytes of size)
 * @return MemoryManagerStatus: OK or ERR
 */
static MemoryManagerStatus prvMemoryAccessPage ( MemorySector sector, int64_t pageIndex, uint8_t * dest )
{
    uint32_t OFFSET     = 0;
    uint32_t PAGE_INDEX = 0;

    if ( pageIndex == -1 )
    {
        prvMemorySectorFindLastWrittenPageIndex ( sector, &PAGE_INDEX );
        prvMemorySectorFindLastWrittenPageIndexResults[ sector ] = PAGE_INDEX;
    }
    else
    {
        PAGE_INDEX = pageIndex;
    }


    // meta configuration sector has only META_CONFIGURATION_SUBSECTOR_COUNT subsectors, thus indexes greater than that
    // will cause reading invalid data and thus must be avoided
    if ( sector == MemorySystemSectorGlobalConfigurationData )
    {
        if ( PAGE_INDEX >= ( GLOBAL_CONFIGURATION_SECTOR_SIZE / PAGE_SIZE ) )
        {
            return MEM_ERR;
        }

        OFFSET = GLOBAL_CONFIGURATION_SECTOR_BASE + PAGE_INDEX * PAGE_SIZE;
    }
    else if ( sector == MemorySystemSectorUserDataSectorMetaData )
    {
        if ( PAGE_INDEX >= ( MEMORY_METADATA_SECTOR_SIZE / PAGE_SIZE ) )
        {
            return MEM_ERR;
        }

        OFFSET = MEMORY_METADATA_SECTOR_BASE + PAGE_INDEX * PAGE_SIZE;
    }
    else
    {
        if ( PAGE_SIZE * PAGE_INDEX > prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ toUserDataSector ( sector ) ].bytesWritten )
        {
            return MEM_ERR;
        }

        OFFSET = prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ toUserDataSector ( sector ) ].startAddress + ( PAGE_INDEX * PAGE_SIZE );
    }


    if ( FLASH_OK != flash_read ( OFFSET, dest, PAGE_SIZE ) )
    {
        return MEM_ERR;
    }

    return MEM_OK;
}


MemoryManagerStatus prvMemoryAccessSectorSingleDataEntry ( MemorySector sector, uint32_t index, void * dst )
{
    uint8_t entries_per_page = prvMemorySectorGetDataEntriesPerPage ( sector );
    uint8_t struct_size      = prvMemorySectorGetDataStructSize ( sector );

    if ( PAGE_SIZE * trunc ( ( double ) index / prvMemorySectorGetDataEntriesPerPage ( sector ) ) > prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ sector ].bytesWritten )
    {
        return MEM_ERR;
    }

    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    MemoryBuffer buffer = { };

    uint32_t pageIndex = ceil ( ( double ) index / entries_per_page );

    if ( MEM_OK != prvMemoryAccessPage ( sector, pageIndex, buffer.data ) )
    {
        return MEM_ERR;
    }

    index = ( pageIndex * entries_per_page ) - index;

    memcpy ( dst, &buffer.data[ index ], struct_size );

    return MEM_OK;
}

static MemoryManagerStatus prvMemoryAccessLastDataEntry ( MemorySector sector, void * dst )
{
    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    if ( toSystemSector ( sector ) < SystemSectorCount )
    {
        uint32_t lastPageIndex = 0;
        if ( MEM_OK != prvMemorySectorFindLastWrittenPageIndex ( sector, &lastPageIndex ) )
        {
            return MEM_ERR;
        }

        if ( MEM_OK != prvMemoryAccessPage ( sector, lastPageIndex, dst ) )
        {
            return MEM_ERR;
        }
    }

    else
    {
        uint32_t lastAddress = prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ toUserDataSector ( sector ) ].startAddress +
                               prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ toUserDataSector ( sector ) ].bytesWritten - PAGE_SIZE +
                               ( ( prvMemorySectorGetDataEntriesPerPage ( sector ) - 1 ) * prvMemorySectorGetDataStructSize ( sector ) );

        if ( FLASH_OK != flash_read ( lastAddress, dst, prvMemorySectorGetDataStructSize ( sector ) ) )
        {
            return MEM_ERR;
        }
    }

    return MEM_OK;
}


void prvQueueMonitorTask ( void * arg )
{
    ( void ) arg;

    is_queue_monitor_running = 1;
    buffer_item item = { };

    while ( is_queue_monitor_running )
    {

        if ( prvMetaDataUpdateMode == MetaDataUpdateTimeBasedFrequencyMode )
        {
            if ( ( xTaskGetTickCount ( ) - prvMetadataAutosaveTimeBasedCounter ) >= METADATA_AUTOSAVE_TIME_BASED_INTERVAL )
            {
                // it is time to update the configurations
                prvMemoryWriteAsyncMetaDataSector ( );

                // reset the timer
                prvMetadataAutosaveTimeBasedCounter = xTaskGetTickCount ( );
            }
        }

//        xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
//                        ULONG_MAX,  /* Reset the notification value to 0 on exit.  */
//                        NULL,      /* Notified value pass out in ulNotifiedValue. */
//                         250);
//        DISPLAY_LINE("monitor woke up!");

        while ( buffer_queue_pop_front ( &prvPageBuffer, &item ) )
        {
            prvMemoryWritePageNow ( item.type, item.data );

            switch ( ( MemorySector ) item.type )
            {
                case MemorySystemSectorGlobalConfigurationData:
//                    uart6_transmit_line ( "Monitor: Configuration was flushed!" );
                    break;
                case MemorySystemSectorUserDataSectorMetaData:
//                    uart6_transmit_line ( "Monitor: MetaData was flushed!" );
                    break;
                case MemoryUserDataSectorGyro:
//                    uart6_transmit_line ( "Monitor: Gyro was flushed!" );
                case MemoryUserDataSectorAccel:
//                    uart6_transmit_line ( "Monitor: Accel was flushed!" );
                case MemoryUserDataSectorMag:
//                    uart6_transmit_line ( "Monitor: Mag was flushed!" );
                    break;
                case MemoryUserDataSectorPressure:
//                    uart6_transmit_line ( "Monitor: Pressure was flushed!" );
                    break;
                case MemoryUserDataSectorTemperature:
//                    uart6_transmit_line ( "Monitor: Temperature was flushed!" );
                    break;
                case MemoryUserDataSectorContinuity:
//                    uart6_transmit_line ( "Monitor: Cont was flushed!" );
                    break;
                case MemoryUserDataSectorFlightEvent:
//                    uart6_transmit_line ( "Monitor: Event was flushed!" );
                    break;
                case MemorySectorCount:
                default:
                    break;
            }
        }
    }

    DISPLAY_LINE( "monitor EXITED!" );
}


static uint32_t prvMemorySectorGetDataEntriesPerPage ( MemorySector sector )
{
    switch ( sector )
    {
        case MemorySystemSectorGlobalConfigurationData:
            return GLOBAL_CONFIGURATION_ENTRIES_PER_PAGE;
        case MemorySystemSectorUserDataSectorMetaData:
            return MEMORY_METADATA_ENTRIES_PER_PAGE;
        case MemoryUserDataSectorGyro:
        case MemoryUserDataSectorAccel:
        case MemoryUserDataSectorMag:
            return IMU_ENTRIES_PER_PAGE;
        case MemoryUserDataSectorPressure:
            return PRESSURE_ENTRIES_PER_PAGE;
        case MemoryUserDataSectorTemperature:
            return TEMPERATURE_ENTRIES_PER_PAGE;
        case MemoryUserDataSectorContinuity:
            return CONTINUITY_ENTRIES_PER_PAGE;
        case MemoryUserDataSectorFlightEvent:
            return FLIGHT_EVENT_ENTRIES_PER_PAGE;
        case MemorySectorCount:
        default:
            return 0;
    }
}

static uint32_t prvMemorySectorGetDataStructSize ( MemorySector sector )
{
    switch ( sector )
    {
        case MemorySystemSectorGlobalConfigurationData:
            return sizeof ( GlobalConfigurationU );
        case MemorySystemSectorUserDataSectorMetaData:
            return sizeof ( MemoryLayoutMetaDataU );
        case MemoryUserDataSectorGyro:
        case MemoryUserDataSectorAccel:
        case MemoryUserDataSectorMag:
            return sizeof ( IMUDataU );
        case MemoryUserDataSectorPressure:
            return sizeof ( PressureDataU );
        case MemoryUserDataSectorTemperature:
            return sizeof ( TemperatureDataU );
        case MemoryUserDataSectorContinuity:
            return sizeof ( ContinuityU );
        case MemoryUserDataSectorFlightEvent:
            return sizeof ( FlightEventU );
        case MemorySectorCount:
        default:
            return 0;
    }
}

static uint32_t prvMemorySectorGetAlignedDataStructSize ( MemorySector sector )
{
    switch ( sector )
    {
        case MemorySystemSectorGlobalConfigurationData:
            return trunc ( ( PAGE_SIZE / sizeof ( GlobalConfigurationU ) ) ) * sizeof ( GlobalConfigurationU );
        case MemorySystemSectorUserDataSectorMetaData:
            return trunc ( ( PAGE_SIZE / sizeof ( MemoryLayoutMetaDataU ) ) ) * sizeof ( MemoryLayoutMetaDataU );
        case MemoryUserDataSectorGyro:
        case MemoryUserDataSectorAccel:
        case MemoryUserDataSectorMag:
            return trunc ( ( PAGE_SIZE / sizeof ( IMUDataU ) ) ) * sizeof ( IMUDataU );
        case MemoryUserDataSectorPressure:
            return trunc ( ( PAGE_SIZE / sizeof ( PressureDataU ) ) ) * sizeof ( PressureDataU );
        case MemoryUserDataSectorTemperature:
            return trunc ( ( PAGE_SIZE / sizeof ( TemperatureDataU ) ) ) * sizeof ( TemperatureDataU );
        case MemoryUserDataSectorContinuity:
            return trunc ( ( PAGE_SIZE / sizeof ( ContinuityU ) ) ) * sizeof ( ContinuityU );
        case MemoryUserDataSectorFlightEvent:
            return trunc ( ( PAGE_SIZE / sizeof ( FlightEventU ) ) ) * sizeof ( FlightEventU );
        case MemorySectorCount:
        default:
            return 0;
    }
}

int prvMemorySectorGetPageCount ( MemorySector sector )
{
    if ( toSystemSector ( sector ) < SystemSectorCount )
    {
        // we are accessing the system sector where their sizes are predetermined
        switch ( sector )
        {
            case SystemSectorGlobalConfigurationData:
                return ( GLOBAL_CONFIGURATION_SECTOR_SIZE / PAGE_SIZE );
            case SystemSectorUserDataSectorMetaData:
                return ( MEMORY_METADATA_SECTOR_SIZE / PAGE_SIZE );
            case SystemSectorCount:
            default:
                return 0;
        }
    }
    else
    {
        // we are accessing the user data sector where the sizes are changed by user and not known before the configuration sector is read
        // thus this function will not work for user data sections without prior call to memory_manager_configure () that was successful
        // the indication of that can be checked with prvIsConfigured variable (true if successful, false, if not successful, or has not been called)
        if ( prvIsConfigured == false )
        {
            // returning 0 makes sense because if everything goes right, there is no way that sector would have 0 pages
            return 0;
        }

        // if prvIsConfigured == true, then we can freely use prvGlobalConfigurationDiskSnapshot variable and can safely use user_data_sector_sizes to
        // find out the user sector sizes.
        return ( prvGlobalConfigurationDiskSnapshot.data.values.memory.user_data_sector_sizes[ toUserDataSector ( sector ) ] / PAGE_SIZE );
    }
}

static uint32_t prvMemorySectorGetSize ( MemorySector sector )
{
    if ( toSystemSector ( sector ) < SystemSectorCount )
    {
        // we are accessing the system sector where their sizes are predetermined
        switch ( sector )
        {
            case SystemSectorGlobalConfigurationData:
                return ( GLOBAL_CONFIGURATION_SECTOR_SIZE );
            case SystemSectorUserDataSectorMetaData:
                return ( MEMORY_METADATA_SECTOR_SIZE );
            case SystemSectorCount:
            default:
                return 0;
        }
    }
    else
    {
        // we are accessing the user data sector where the sizes are changed by user and not known before the configuration sector is read
        // thus this function will not work for user data sections without prior call to memory_manager_configure () that was successful
        // the indication of that can be checked with prvIsConfigured variable (true if successful, false, if not successful, or has not been called)
        if ( prvIsConfigured == false )
        {
            // returning 0 makes sense because if everything goes right, there is no way that sector would have 0 pages
            return 0;
        }

        // if prvIsConfigured == true, then we can freely use prvGlobalConfigurationDiskSnapshot variable and can safely use user_data_sector_sizes to
        // find out the user sector sizes.
        return prvGlobalConfigurationDiskSnapshot.data.values.memory.user_data_sector_sizes[ toUserDataSector ( sector ) ];
    }
}


static MemoryManagerStatus prvMemorySectorFindLastWrittenPageIndex ( MemorySector sector, uint32_t * result )
{
    if ( result == NULL )
    {
        return MEM_ERR;
    }

    int pageIndex = 0;

    int sectorPageCount = prvMemorySectorGetPageCount ( sector );
    // need to search the sector and find the latest record

    uint8_t dataRX[256];

    int j;

    while ( pageIndex < sectorPageCount )
    {
        memset ( dataRX, 0, PAGE_SIZE );

        if ( prvMemoryAccessPage ( sector, pageIndex, dataRX ) == MEM_ERR )
        {
            return MEM_ERR;
        }

        if ( common_is_mem_empty ( dataRX, PAGE_SIZE ) )
        {
            if ( pageIndex != 0 )
            {
                pageIndex--;
            }

            break;
        }

        pageIndex++;
    }

    ( *result ) = pageIndex;

    return MEM_OK;
}


MemoryManagerStatus memory_manager_add_gyro_update ( IMUDataU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorGyro, _container->bytes );
}

MemoryManagerStatus memory_manager_add_accel_update ( IMUDataU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorAccel, _container->bytes );
}

MemoryManagerStatus memory_manager_add_mag_update ( IMUDataU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorMag, _container->bytes );
}

MemoryManagerStatus memory_manager_add_pressure_update ( PressureDataU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorPressure, _container->bytes );
}

MemoryManagerStatus memory_manager_add_temp_update ( TemperatureDataU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorTemperature, _container->bytes );
}

MemoryManagerStatus memory_manager_add_continuity_update ( ContinuityU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorContinuity, _container->bytes );
}

MemoryManagerStatus memory_manager_add_flight_event_update ( FlightEventU * _container )
{
    return prvMemoryAddNewUserDataSectorEntry ( UserDataSectorFlightEvent, _container->bytes );
}


MemoryManagerStatus memory_manager_get_single_press_entry ( PressureDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorPressure, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_temp_entry ( TemperatureDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorTemperature, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_gyro_entry ( IMUDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorGyro, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_acc_entry ( IMUDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorAccel, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_mag_entry ( IMUDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorMag, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_cont_entry ( ContinuityU * dst, uint32_t measurement_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorContinuity, measurement_index, dst );
}

MemoryManagerStatus memory_manager_get_single_flight_event_entry ( FlightEventU * dst, uint32_t measurement_index )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemoryUserDataSectorFlightEvent, measurement_index, dst );
}

MemoryManagerStatus memory_manager_get_single_configuration_entry ( GlobalConfigurationU * dst, uint32_t entry_index )
{
    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemorySystemSectorGlobalConfigurationData, entry_index, dst );
}

MemoryManagerStatus memory_manager_get_single_metadata_entry ( MemoryLayoutMetaDataU * dst, uint32_t entry_index )
{
    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessSectorSingleDataEntry ( MemorySystemSectorUserDataSectorMetaData, entry_index, dst );
}


MemoryManagerStatus memory_manager_get_last_press_entry ( PressureDataU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorPressure, dst );
}

MemoryManagerStatus memory_manager_get_last_temp_entry ( TemperatureDataU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorTemperature, dst );
}

MemoryManagerStatus memory_manager_get_last_gyro_entry ( IMUDataU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorGyro, dst );
}

MemoryManagerStatus memory_manager_get_last_acc_entry ( IMUDataU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorAccel, dst );
}

MemoryManagerStatus memory_manager_get_last_mag_entry ( IMUDataU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorMag, dst );
}

MemoryManagerStatus memory_manager_get_last_cont_entry ( ContinuityU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorContinuity, dst );
}

MemoryManagerStatus memory_manager_get_last_flight_event_entry ( FlightEventU * dst )
{
    if ( dst == NULL || !prvIsConfigured || !prvIsInitialized )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemoryUserDataSectorFlightEvent, dst );
}

MemoryManagerStatus memory_manager_get_last_configuration_entry ( GlobalConfigurationU * dst )
{
    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemorySystemSectorGlobalConfigurationData, dst );
}

MemoryManagerStatus memory_manager_get_last_metadata_entry ( MemoryLayoutMetaDataU * dst )
{
    if ( dst == NULL )
    {
        return MEM_ERR;
    }

    return prvMemoryAccessLastDataEntry ( MemorySystemSectorUserDataSectorMetaData, dst );
}


MemoryManagerStatus memory_manager_get_stats ( char * buffer, size_t xBufferLen )
{
    int length = 0;
    length += snprintf ( buffer + length, xBufferLen, "\n----- Memory Statistics -----\r\n" );
    length += snprintf ( buffer + length, xBufferLen, "signature: %s\r\n", prvGlobalConfigurationDiskSnapshot.data.values.signature );
    length += snprintf ( buffer + length, xBufferLen, "Data Sectors: " );

    for ( UserDataSector sector = UserDataSectorGyro; sector < UserDataSectorCount; sector++ )
    {
        length += snprintf ( buffer + length, xBufferLen, "%i,", sector );
    }

    length += snprintf ( buffer + length, xBufferLen, "\r\n" );

    MemorySectorInfo     dataSector;
    for ( UserDataSector sector = UserDataSectorGyro; sector < UserDataSectorCount; sector++ )
    {
        dataSector = prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ sector ];
        length += snprintf ( buffer + length, xBufferLen, "Sector #%i:\r\n", sector );
        length += snprintf ( buffer + length, xBufferLen, "--------------------\r\n" );
        length += snprintf ( buffer + length, xBufferLen, " size:            %lu\r\n", dataSector.size );
        length += snprintf ( buffer + length, xBufferLen, " begin:           %lu\r\n", dataSector.startAddress );
        length += snprintf ( buffer + length, xBufferLen, " end:             %lu\r\n", dataSector.endAddress );
        length += snprintf ( buffer + length, xBufferLen, " size on disk:    %lu\r\n", dataSector.bytesWritten );
        length += snprintf ( buffer + length, xBufferLen, " pages on disk:   %lu\r\n", dataSector.bytesWritten / PAGE_SIZE );
        length += snprintf ( buffer + length, xBufferLen, " entries on disk: %lu\r\n", dataSector.bytesWritten / PAGE_SIZE * prvMemorySectorGetDataEntriesPerPage ( toMemorySector( sector ) ) );

//        uint8_t dst [prvMemorySectorGetDataStructSize(sector)];
//        memset(dst, 0, prvMemorySectorGetDataStructSize(sector));
//
//        if( MEM_OK != prvMemoryAccessSectorSingleDataEntry(&dst, sector, 1) )
//        {
//            return MEM_ERR;
//        }
//        length += snprintf (buffer+length, xBufferLen, " first timestamp: %i\n", common_read_32(&dst[0]));
//
//        memset(dst, 0, prvMemorySectorGetDataStructSize(sector));
//        uint32_t last_index = ( prvImmutableMemorySectorOrdinals[ sector ].info.bytesWritten / PAGE_SIZE * prvMemorySectorGetDataEntriesPerPage(sector) ) - 1;
//        if( MEM_OK != prvMemoryAccessSectorSingleDataEntry(&dst, sector, last_index) )
//        {
//            return MEM_ERR;
//        }
//
//        length += snprintf (buffer+length, xBufferLen, " last timestamp:  %i\n", common_read_32(&dst[0]));
        length += snprintf ( buffer + length, xBufferLen, "--------------------\r\n" );
    }

    return MEM_OK;
}


MemoryManagerStatus memory_manager_get_configurations ( GlobalConfigurationU * pConfigs )
{
    if ( pConfigs == NULL )
    {
        return MEM_ERR;
    }

    // validation
    if ( memcmp ( prvGlobalConfigurationDiskSnapshot.data.values.signature, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE, MEMORY_MANAGER_DATA_INTEGRITY_SIGNATURE_BUFFER_LENGTH ) == 0 ) // signature sequences match)
    {
        pConfigs = &prvGlobalConfigurationDiskSnapshot.data;
        return MEM_OK;
    }

    ( void ) pConfigs;
    return MEM_ERR;
}

MemoryManagerStatus memory_manager_update_sensors_ground_data ( GroundDataU * _data )
{
    if ( _data == NULL )
    {
        return MEM_ERR;
    }

    prvGlobalConfigurationDiskSnapshot.data.values.system.ground_pressure = _data->pressure;

    prvMemoryWriteAsyncMetaDataSector ( );
    return MEM_OK;
}


// debug function
MemoryManagerStatus _access_single_page_in_user_data_sector ( MemorySector sector, uint32_t pageIndex )
{
    assert ( PAGE_SIZE * pageIndex <= prvMemoryMetaDataFlashSnapshot.data.values.user_sectors[ UserDataSectorPressure ].bytesWritten );

    MemoryBuffer buffer = { };

    int16_t sectorDataStructSize = prvMemorySectorGetDataStructSize ( sector );

    uint8_t entries[PRESSURE_ENTRIES_PER_PAGE][sectorDataStructSize];

    if ( prvMemoryAccessPage ( sector, pageIndex, buffer.data ) != MEM_OK )
    {
        return MEM_ERR;
    }

    memcpy ( &entries[ 0 ], &buffer.data[ 0 ], sectorDataStructSize );

    for ( uint32_t i = 1; i < PRESSURE_ENTRIES_PER_PAGE; i++ )
    {
        memcpy ( &entries[ i ], &buffer.data[ i * sectorDataStructSize ], sectorDataStructSize );
    }

    return MEM_OK;
}















































































