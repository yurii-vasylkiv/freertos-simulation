//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS>Avionics-2019
//
// File Description:
//  Source file for communicating with STM32 microchip via UART Serial Connection. Handles initialization and transmission/reception.
//
// History
// 2019-02-13 Eric Kapilik
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "UART.h"
#include <string.h>
#include <stdlib.h>
#include "../main.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "../hardware_definitions.h"

static uint8_t bufftx[BUFFER_SIZE] = ""; // uart_transmit buffer
static uint8_t buffrx[BUFFER_SIZE] = ""; // receive buffer

static UART_HandleTypeDef uart2;
static UART_HandleTypeDef uart6;



static void Error_Handler_UART(void);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int UART_Port2_Init(void)
{
    HAL_StatusTypeDef status;
    HAL_Init();
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // GPIO uses pins 2 & 3
    
    /* Setup UART2 TX Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_2; //USART_TX_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Setup UART2 RX Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_3; //USART_RX_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    uart2.Instance = USART2;
    uart2.Init.BaudRate = 9600;
    uart2.Init.WordLength = UART_WORDLENGTH_8B;
    uart2.Init.StopBits = UART_STOPBITS_1;
    uart2.Init.Parity = UART_PARITY_NONE;
    uart2.Init.Mode = UART_MODE_TX_RX;
    uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart2.Init.OverSampling = UART_OVERSAMPLING_16;

    status = HAL_UART_Init(&uart6);

    return status;
}
int UART_Port6_Init(void)
{
    HAL_StatusTypeDef status;
    __HAL_RCC_USART6_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* Setup UART6 TX Pin */
    GPIO_InitStruct.Pin = UART_TX_PIN; //USART_TX_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(UART_TX_PORT, &GPIO_InitStruct);
    
    /* Setup UART6 RX Pin */
    GPIO_InitStruct.Pin = UART_RX_PIN; //USART_RX_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(UART_RX_PORT, &GPIO_InitStruct);

    uart6.Instance = USART6;
    uart6.Init.BaudRate = 115200;
    uart6.Init.WordLength = UART_WORDLENGTH_8B;
    uart6.Init.StopBits = UART_STOPBITS_1;
    uart6.Init.Parity = UART_PARITY_NONE;
    uart6.Init.Mode = UART_MODE_TX_RX;
    uart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart6.Init.OverSampling = UART_OVERSAMPLING_16;
    status = HAL_UART_Init(&uart6);

    return status;
}

static int uart_transmit(UART_HandleTypeDef *huart, const char * message)
{
    memcpy (&bufftx, message, strlen(message));

    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(huart, (uint8_t*)bufftx, sizeof(uint8_t) * (strlen(message)), TIMEOUT_MAX);
    return status;
}

static int uart_transmit_line(UART_HandleTypeDef *huart, const char * message)
{
    size_t i = strlen(message);
    memcpy (&bufftx, message, i);
    bufftx[i++] = '\r';
    bufftx[i++] = '\n';
    bufftx[i++] = '\0';

    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(huart, (uint8_t*)bufftx, sizeof(uint8_t) * (i), TIMEOUT_MAX);
    return status;
}

static int uart_transmit_bytes(UART_HandleTypeDef *huart, uint8_t * bytes, uint16_t numBytes)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(huart,  bytes, numBytes, TIMEOUT_MAX);
    return status;
}

static char* uart_receive_command(UART_HandleTypeDef *huart)
{
    uint8_t c; //key pressed character
    size_t i;
    
    c = '\0'; //clear out character received
    buffrx[0] = '\0'; //clear out receive buffer
    i = 0; //start at beginning of index
    
    while(i < BUFFER_SIZE){
        //get character (BLOCKING COMMAND)
        if (HAL_UART_Receive(huart, &c, 1, 0xFFFF) != HAL_OK){
            //did not receive character for some reason.
        }
        
        //print the character back.
        if(c != '\0'){
            
            if(HAL_UART_Transmit(huart, &c, sizeof(c), TIMEOUT_MAX) != HAL_OK){
                //Do something meaningful here...
            }
            
            //adjust our buffer
            if(c == '\r'){ //return entered, command is complete
                break;
            }
            else if(c == 127){ //User hits backspace, clear from buffer and display (backspace is \177 or 127)
                if(i > 0){ i--; } //don't let i become negative
                buffrx[i] = '\0';
            }
            else{ //add character to end of receive buffer
                buffrx[i++] = c;
            }
            
        }
    }
    
    //put a new line for user display
    c = '\n';
    if(HAL_UART_Transmit(huart, &c, sizeof(c), TIMEOUT_MAX) != HAL_OK){
        //handle transmission error
    }

    buffrx[i] = '\0'; //string terminator added to the end of the message
    
    return (char*)buffrx;
}
static int uart_receive(UART_HandleTypeDef *huart, uint8_t * buf, size_t size){
    uint8_t c; //key pressed character
    size_t i = 0; //start at beginning of index

    while(i < size){
        //get character (BLOCKING COMMAND)
        if (HAL_UART_Receive(huart, &buf[i++], 1, 0xFFFF) != HAL_OK){
            return i == size;
        }
    }

    return true;
}


int uart2_transmit(const char * message)
{
    return uart_transmit(&uart2, message);
}

int uart6_transmit(const char * message)
{
    return uart_transmit(&uart6, message);
}

int uart2_transmit_line(const char * message)
{
    return uart_transmit_line(&uart2, message);
}

int uart6_transmit_line(const char * message)
{
    return uart_transmit_line(&uart2, message);
}

int uart2_transmit_bytes(uint8_t * bytes, uint16_t numBytes)
{
    return uart_transmit_bytes(&uart2, bytes, numBytes);
}

int uart6_transmit_bytes(uint8_t * bytes, uint16_t numBytes)
{
    return uart_transmit_bytes(&uart6, bytes, numBytes);
}

char* uart2_receive_command()
{
    return uart_receive_command(&uart2);
}

char* uart6_receive_command()
{
    return uart_receive_command(&uart6);
}

int uart2_receive(uint8_t * buf, size_t size)
{
    return uart_receive(&uart2, buf, size);
}


int uart6_receive(uint8_t * buf, size_t size)
{
    return uart_receive(&uart6, buf, size);
}