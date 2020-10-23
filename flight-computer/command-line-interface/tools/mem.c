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


bool cli_tools_mem (char *pcWriteBuffer, size_t xWriteBufferLen, const char * cmd_option, const char * str_option_arg)
{
    if (strcmp(cmd_option, "mem") == 0)
    {
        sprintf(pcWriteBuffer, "%s", str_option_arg);
        return true;
    }

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