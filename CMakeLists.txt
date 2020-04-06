cmake_minimum_required(VERSION 3.5.1)
project(avionics ASM C CXX RC)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_VERBOSE_MAKEFILE ON)


SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wmissing-prototypes -Wmissing-declarations -Wl,-gc-sections,--print-memory-usage")

# Add -O0 to remove optimizations when using gcc
IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

add_definitions(-DSTM32F401xE)

add_definitions(-DUSE_HAL_DRIVER)
add_definitions("-D__weak=__attribute__((weak))")
add_definitions("-D__packed=__attribute__((__packed__))")

#CMAKE_FORCE_C_COMPILER  (gcc)   # absolute path needs to be set accordingly
#CMAKE_FORCE_CXX_COMPILER(g++)   # absolute path needs to be set accordingly

set(CMAKE_C_FLAGS "-g -std=gnu11 -fstack-usage -fdata-sections -fdata-sections -mthreads "    )
set(CMAKE_CXX_FLAGS "-g -std=c++17 -fstack-usage -fdata-sections -fdata-sections -mthreads "  )

add_library(RTOS_LIB
        FreeRTOS/Source/portable/MSVC-MingW/port.c
        FreeRTOS/Source/portable/MemMang/heap_1.c
        FreeRTOS/Source/croutine.c
        FreeRTOS/Source/event_groups.c
        FreeRTOS/Source/list.c
        FreeRTOS/Source/queue.c
        FreeRTOS/Source/tasks.c
        FreeRTOS/Source/timers.c
        FreeRTOS/Source/event_groups.c)

file(GLOB_RECURSE USER_SRC "flight-computer/*.c" "flight-computer/*.c" "flight-computer/*.cpp" "flight-computer/memory-management/*.c" "flight-computer/event-detection/*.c")
file(GLOB_RECURSE CLI_SRC "FreeRTOS-Plus-CLI/*.c")


file(GLOB_RECURSE SRAD_CSV_FILE_PATH "flight-computer/sensor-simulation/srad_flight.data.csv")
file(GLOB_RECURSE COTS_CSV_FILE_PATH "flight-computer/sensor-simulation/cots_flight.data.csv")

add_definitions(-DSRAD_CSV_FILE_PATH=${SRAD_CSV_FILE_PATH})
add_definitions(-DCOTS_CSV_FILE_PATH=${COTS_CSV_FILE_PATH})

set(CXX_SRC flight-computer/sensor-simulation/csv.h flight-computer/sensor-simulation/datafeeder.cpp)

include_directories(cxx)
include_directories(Configs)
include_directories(FreeRTOS/Source/include)
include_directories(FreeRTOS/Source/portable/MSVC-MingW)
include_directories(FreeRTOS-Plus-CLI)
include_directories(flight-computer)
include_directories(.)

add_executable(${PROJECT_NAME}.elf ${USER_SRC} ${CLI_SRC} ${CXX_SRC})

target_link_libraries(${PROJECT_NAME}.elf RTOS_LIB pthread winmm ws2_32)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.bin)

set_target_properties(${CMAKE_PROJECT_NAME}.elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
