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

option(COMPILE_IN_SIMULATION_MODE "Indicates whether the build needs to be built against WinOS or RTOS" ON)

add_definitions(-DSTM32F401xE)
add_definitions(-DUSE_HAL_DRIVER)
add_definitions("-D__weak=__attribute__((weak))")
add_definitions("-D__packed=__attribute__((__packed__))")

IF(COMPILE_IN_SIMULATION_MODE)
    message(STATUS "Selected WinOS BUILD with MINGW build tools")

    set(CMAKE_C_FLAGS "-g -std=gnu11 -fstack-usage -fdata-sections -fdata-sections -mthreads "    )
    set(CMAKE_CXX_FLAGS "-g -std=c++17 -fstack-usage -fdata-sections -fdata-sections -mthreads "  )

    file(GLOB_RECURSE SRAD_CSV_FILE_PATH "flight-computer/sim-port/sensor-simulation/srad_flight.data.csv" )
    file(GLOB_RECURSE COTS_CSV_FILE_PATH "flight-computer/sim-port/sensor-simulation/cots_flight.data.csv" )

    add_definitions(-DSRAD_CSV_FILE_PATH=${SRAD_CSV_FILE_PATH})
    add_definitions(-DCOTS_CSV_FILE_PATH=${COTS_CSV_FILE_PATH})

    include_directories         (flight-computer/sim-port/os-kernel/FreeRTOS/Source/include)
    include_directories         (flight-computer/sim-port/os-kernel/FreeRTOS/Source/portable/MSVC-MingW)
    include_directories         (os-kernel/FreeRTOS-Plus-CLI)

    set(SIM_PORT_SRC        # sensor simulation
                            flight-computer/sim-port/sensor-simulation/imu_sensor.c
                            flight-computer/sim-port/sensor-simulation/pressure_sensor.c
                            flight-computer/sim-port/sensor-simulation/datafeeder.cpp

                            # Communication/Transmission protocols
                            flight-computer/sim-port/transmission-protocols/SPI.c
                            flight-computer/sim-port/transmission-protocols/UART.c
        )

    add_library(RTOS_LIB    flight-computer/sim-port/os-kernel/FreeRTOS/Source/portable/MSVC-MingW/port.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/portable/MemMang/heap_1.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/croutine.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/event_groups.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/list.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/queue.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/tasks.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/timers.c
                            flight-computer/sim-port/os-kernel/FreeRTOS/Source/event_groups.c
                            os-kernel/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c
                            os-kernel/FreeRTOS-Plus-CLI/Sample-CLI-commands.c
            )

ELSE()
    message(STATUS "Selected ARM BUILD with ARM-Build tools")
    find_package (Python 3.0 REQUIRED)
    execute_process(COMMAND python generator.py WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

    if(EXISTS "${CMAKE_SOURCE_DIR}/generated/CMakeConfigFile.cmake")
        include(${CMAKE_SOURCE_DIR}/generated/CMakeConfigFile.cmake)
    else()
        message(FATAL_ERROR "Cannot generate project: please, first run \"python process-user-configurations.py\"")
    endif()


    set(CMAKE_C_COMPILER     ${EABI_PREFIX}-gcc                     )   # absolute path needs to be set accordingly
    set(CMAKE_CXX_COMPILER   ${EABI_PREFIX}-g++                     )   # absolute path needs to be set accordingly

    include(CMakeForceCompiler)
    set(CMAKE_C_COMPILER_WORKS                                     1)
    CMAKE_FORCE_C_COMPILER  (${ARM_TOOLS_DIR}/${CMAKE_C_COMPILER}   GNU)   # absolute path needs to be set accordingly
    CMAKE_FORCE_CXX_COMPILER(${ARM_TOOLS_DIR}/${CMAKE_CXX_COMPILER} GNU)   # absolute path needs to be set accordingly

    set(AS                   ${EABI_PREFIX}-gcc-as                  )   # absolute path needs to be set accordingly
    set(AR                   ${EABI_PREFIX}-gcc-ar                  )   # absolute path needs to be set accordingly
    set(OBJCOPY              ${EABI_PREFIX}-objcopy                 )   # absolute path needs to be set accordingly
    set(OBJDUMP              ${EABI_PREFIX}-objdump                 )   # absolute path needs to be set accordingly
    set(SIZE                 ${EABI_PREFIX}-size                    )   # absolute path needs to be set accordingly
    set(GDB                  ${EABI_PREFIX}-gdb                     )   # absolute path needs to be set accordingly


    SET(CMAKE_C_FLAGS "-g -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 \
                   -specs=nano.specs -fstack-usage -fdata-sections -fdata-sections")

    SET(CMAKE_CXX_FLAGS "-g -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c++11 \
                    -specs=nano.specs -fstack-usage -fdata-sections -fdata-sections")

    set(SYS_CALLS_SRC sys/syscalls.c)


    SET(LINKER_SCRIPT       linker-script/STM32F401RE_FLASH.ld)
    SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")
    SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wmissing-prototypes -Wmissing-declarations -Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

    include_directories     (board-hardware-drivers/BMI08x-Sensor-API)
    include_directories     (board-hardware-drivers/BMI08x-Sensor-API/Inc)
    include_directories     (board-hardware-drivers/CMSIS/Include)
    include_directories     (board-hardware-drivers/STM32F4xx_HAL_Driver/Inc)
    include_directories     (board-hardware-drivers/STM32F4xx_HAL_Driver/Inc/Custom)
    include_directories     (board-hardware-drivers/CMSIS/Device/ST/STM32F4xx/Include)
    include_directories     (os-kernel/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    include_directories     (os-kernel/FreeRTOS/Source/CMSIS_RTOS)
    include_directories     (os-kernel/FreeRTOS/Source/include)

    add_library(CMSIS_LIB
                            board-hardware-drivers/CMSIS/Device/ST/STM32F4xx/Source/system_stm32f4xx.c
                            board-hardware-drivers/CMSIS/Device/ST/STM32F4xx/Source/stm32f4xx_hal_msp.c
                            board-hardware-drivers/CMSIS/Device/ST/STM32F4xx/Source/stm32f4xx_hal_timebase_tim.c
                            board-hardware-drivers/CMSIS/Device/ST/STM32F4xx/Source/stm32f4xx_it.c
                            Startup/startup_stm32f401xe.s
            )

    file(GLOB_RECURSE HAL_SRC "board-hardware-drivers/STM32F4xx_HAL_Driver/Src/*.c")
    file(GLOB_RECURSE BOSCH_API_SRC "board-hardware-drivers/BMI08x-Sensor-API/Src/*.c")

    add_library(RTOS_LIB
                            os-kernel/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
                            os-kernel/FreeRTOS/Source/portable/MemMang/heap_1.c
                            os-kernel/FreeRTOS/Source/croutine.c
                            os-kernel/FreeRTOS/Source/event_groups.c
                            os-kernel/FreeRTOS/Source/list.c
                            os-kernel/FreeRTOS/Source/queue.c
                            os-kernel/FreeRTOS/Source/tasks.c
                            os-kernel/FreeRTOS/Source/timers.c
                            os-kernel/FreeRTOS/Source/event_groups.c
                            os-kernel/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c
                            os-kernel/FreeRTOS-Plus-CLI/Sample-CLI-commands.c
            )

    set(IMPL_FOLDERS_SRC        # Communication/Transmission protocols
                                flight-computer/protocols/impl/UART.c
                                flight-computer/protocols/impl/SPI.c

                                # sensors
                                flight-computer/board/components/impl/imu_sensor.c
                                flight-computer/board/components/impl/pressure_sensor.c
        )

ENDIF()

include_directories         (flight-computer)
include_directories         (configurations)
include_directories         (${CMAKE_SOURCE_DIR})

set(USER_SRC                # Board
                            flight-computer/board/board.c
                            flight-computer/board/components/buzzer.c
                            flight-computer/board/components/flash.c
                            flight-computer/board/components/recovery.c

                            # Core
                            flight-computer/core/flight_controller.c

                            # Memory Management
                            flight-computer/memory-management/memory_manager.c
                            flight-computer/memory-management/queue.c

                            # Event Detection
                            flight-computer/event-detection/event_detector.c

                            # CLI
                            flight-computer/command-line-interface/controller.c
                            )




IF(COMPILE_IN_SIMULATION_MODE)
    add_executable(${PROJECT_NAME}.elf flight-computer/main.c ${USER_SRC} ${CXX_SRC} ${SIM_PORT_SRC})
    target_link_libraries(RTOS_LIB pthread winmm ws2_32)
ELSE()
    add_executable(${PROJECT_NAME}.elf flight-computer/main.c ${USER_SRC} ${HAL_SRC} ${BOSCH_API_SRC} ${SYS_CALLS_SRC} ${IMPL_FOLDERS_SRC} ${LINKER_SCRIPT})
    target_link_libraries(${PROJECT_NAME}.elf CMSIS_LIB -lm)
ENDIF()

target_link_libraries(${PROJECT_NAME}.elf RTOS_LIB)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.bin)

set_target_properties(${CMAKE_PROJECT_NAME}.elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")


IF(COMPILE_IN_SIMULATION_MODE)
ELSE()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/cmake-build-debug/${PROJECT_NAME}.map")

    add_custom_target(upload
            ${GDB} -iex "target remote tcp:localhost:3333"
            -iex "monitor program $<TARGET_FILE:${PROJECT_NAME}.elf>"
            -iex "monitor reset init"
            -iex "disconnect" -iex "quit")
ENDIF()