#ifndef STM32F4XX_HAL_UART_CLI_H
#define STM32F4XX_HAL_UART_CLI_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS>Avionics-2019
//
// File Description:
//  Header file for communicating with STM32 microchip via UART Serial Connection. Handles initialization and transmission/reception.
//
// History
// 2019-02-13 Eric Kapilik
// - Created.

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define TIMEOUT_MAX 0xFFFF
#define BUFFER_SIZE 2048

char PRINT_BUFFER[256];
#define DISPLAY(format, args...)                                                              \
            sprintf(PRINT_BUFFER, format, args);                                              \
            uart6_transmit(PRINT_BUFFER);                                                     \

#define DISPLAY_LINE(format, args...)                                                         \
            sprintf(PRINT_BUFFER, format, args);                                              \
            uart6_transmit_line(PRINT_BUFFER)                                                 \

#define INPUT(x) const char * t = uart6_receive_command(); memcpy(x, t, strlen(t))

int UART_Port2_Init(void);
int UART_Port6_Init(void);

int uart2_transmit(const char * message);
int uart6_transmit(const char * message);

int uart2_transmit_line(const char * message);
int uart6_transmit_line(const char * message);

int uart2_transmit_bytes(uint8_t * bytes, uint16_t numBytes);
int uart6_transmit_bytes(uint8_t * bytes, uint16_t numBytes);

char* uart2_receive_command();
char* uart6_receive_command();

int uart2_receive(uint8_t * buf, size_t size);
int uart6_receive(uint8_t * buf, size_t size);

#endif //STM32F4XX_HAL_UART_CLI_H
