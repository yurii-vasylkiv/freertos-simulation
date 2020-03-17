#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>





#define PAGE_SIZE   256 // 256 bytes per page
#define FLASH_SIZE 1024*1024*65 // 65 MB

const char FILE_NAME[] = "myFlash.bin"; // The file that will represent the flash

typedef struct
{
    bool something;
} FlashStruct_t;



void initialize_flash( void )
{
    if ( access( FILE_NAME, F_OK ) != -1 )
    {
        FILE * writePrt = fopen( FILE_NAME, "ab+" ); // Open the file new, all old contents are erased
        if ( !writePrt )
        {
            perror( "fopen" );
        }
        fseek( writePrt, 0, SEEK_CUR );
        fclose( writePrt );

    } else
    {
        FILE * writePrt = fopen( FILE_NAME, "wb+" ); // Open the file new, all old contents are erased
        if ( !writePrt )
        {
            perror( "fopen" );
        }

        // Create or over write the file with all 0xff for the specified size
        fseek( writePrt, FLASH_SIZE, SEEK_SET );
        fputc( '\0', writePrt );
        fclose( writePrt );
    }
}



uint32_t program_page( uint32_t address, uint8_t * data_buffer, uint16_t num_bytes )
{
    FILE * writePrt = fopen( FILE_NAME, "rb+" ); // File must exist
    int32_t bytesWritten = 0;

    if ( address > FLASH_SIZE )
    {
        return 0;
    }


    // Go to te given position of memory address (inside he file), fseek return zero if able to seek or non-zero if failed
    fseek( writePrt, address, SEEK_SET );

    // Overwrite the block of memory with given contents
    bytesWritten = fwrite( data_buffer, sizeof( uint8_t ), PAGE_SIZE, writePrt );

    fclose( writePrt );

    // Return operation flag
    return bytesWritten;
}



uint32_t read_page( uint32_t address, uint8_t * data_buffer, uint16_t num_bytes )
{
    FILE * readPrt = fopen( FILE_NAME, "rb" );

    if ( address > FLASH_SIZE )
    {
        return 0;
    }
    int32_t bytesRead = 0;

    // Go to the given position of memory address
    fseek( readPrt, address,
           SEEK_SET ); // Go to the position that is number of bytes which is address from the beginning of file

    // Read the given address and number of bytes to read
    bytesRead = fread( data_buffer, sizeof( uint8_t ), num_bytes, readPrt );

    fclose( readPrt );

    // Return operation flag
    return bytesRead;
}


/**
 * @file flash.c
 * @author UMSATS Rocketry Division. Joseph Howarth
 * @date 2019-03-29
 * @brief Source file for the flash memory interface.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see https://github.com/UMSATS/Avionics-2019
 */

#include <stdint.h>
#include "stm32/STM32.h"

#include "FreeRTOS.h"
#include "portable.h"

#include "flash.h"
#include "hardware_definitions.h"
#include "protocols/SPI.h"





/**
 * @brief
 * This function sets the write enable. This is needed before a
 * write status register, program or erase command.
 * @param p_flash Pointer to @c Flash structure
 * @return Will be FLASH_BUSY if there is another operation in progress, FLASH_OK otherwise.
 * @see https://github.com/UMSATS/Avionics-2019/
 */
FlashStatus enable_write( )
{
    uint8_t status_reg = flash_get_status_register( );
    if ( FLASH_IS_DEVICE_BUSY( status_reg ) )
    {
        return FLASH_BUSY;
    } else
    {
        uint8_t command = FLASH_ENABLE_WRITE_COMMAND;
        spi1_transmit( &command, NULL, 1, 10 );
        return FLASH_OK;
    }
}



/**
 * @brief
 * This function is responsible to work like a generic interface to send any command to the flash driver
 * @param p_flash Pointer to @c Flash structure
 * @param address pointer to where in the flash memory you want to apply the operation to
 * @return Will be FLASH_BUSY if there is another operation in progress, FLASH_OK otherwise.
 * @note Any additional commands should always call this function. If this function does not satisfy the
 * needs later on when the interface is extended to potentially support more operations
 * a developer should modify this function to his needs to keep this function as a generic interface forever
 * @see https://github.com/UMSATS/Avionics-2019/
 */

FlashStatus execute_command( uint32_t address, uint8_t command, uint8_t * data_buffer, uint16_t num_bytes )
{
    uint8_t status_reg = flash_get_status_register( );
    if ( FLASH_IS_DEVICE_BUSY( status_reg ) )
    {
        return FLASH_BUSY;
    } else
    {
        enable_write( );
        if ( command == FLASH_BULK_ERASE_COMMAND )
        {
            enable_write( );
            spi1_send( &command, 1, data_buffer, num_bytes, 10 );
            return FLASH_OK;
        } else
        {
            uint8_t command_address[] =
                    {
                            ( command ),
                            ( address & ( FLASH_HIGH_BYTE_MASK_24B ) ) >> 16,
                            ( address & ( FLASH_MID_BYTE_MASK_24B ) ) >> 8,
                            ( address & ( FLASH_LOW_BYTE_MASK_24B ) )
                    };

            spi1_send( command_address, 4, data_buffer, num_bytes, 10 );
            return FLASH_OK;
        };
    }
}



uint8_t flash_get_status_register( )
{
    uint8_t command = FLASH_GET_STATUS_REG_COMMAND;
    uint8_t status_reg;
    spi1_receive( &command, 1, &status_reg, 1, 10 );
    return status_reg;
}



FlashStatus flash_erase_sector( uint32_t address )
{
    return execute_command( address, FLASH_ERASE_SEC_COMMAND, NULL, 0 );
}



FlashStatus flash_erase_param_sector( uint32_t address )
{
    return execute_command( address, FLASH_ERASE_PARAM_SEC_COMMAND, NULL, 0 );
}



uint32_t flash_write( uint32_t address, uint8_t * data_buffer, uint16_t num_bytes )
{
//    return execute_command(address, FLASH_PP_COMMAND, data_buffer, num_bytes);
    return program_page( address, data_buffer, num_bytes );
}



uint32_t flash_read( uint32_t address, uint8_t * data_buffer, uint16_t num_bytes )
{
    //return execute_command(address, FLASH_READ_COMMAND, data_buffer, num_bytes);
    return read_page( address, data_buffer, num_bytes );
}



FlashStatus flash_erase_device( )
{
    return execute_command( 0, FLASH_BULK_ERASE_COMMAND, NULL, 0 );
}



FlashStatus flash_check_id( )
{
    uint8_t command = FLASH_READ_ID_COMMAND;
    uint8_t id[3] = { 0, 0, 0 };

    spi1_receive( ( uint8_t * ) &command, 1, id, 3, 10 );
    if ( ( id[ 0 ] == FLASH_MANUFACTURER_ID ) && ( id[ 1 ] == FLASH_DEVICE_ID_MSB ) &&
         ( id[ 2 ] == FLASH_DEVICE_ID_LSB ) )
    {
        return FLASH_OK;
    }

    return FLASH_OK;
}



int flash_initialize( )
{
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    //This configures the write protect pin(Active Low).
    GPIO_InitStruct.Pin = FLASH_WP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = 0;

    HAL_GPIO_Init( FLASH_WP_PORT, &GPIO_InitStruct );

    GPIO_InitTypeDef GPIO_InitStruct2 = { 0 };
    //This configures the hold pin(Active Low).
    GPIO_InitStruct2.Pin = FLASH_HOLD_PIN;
    GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct2.Pull = GPIO_NOPULL;
    GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct2.Alternate = 0;

    HAL_GPIO_Init( FLASH_HOLD_PORT, &GPIO_InitStruct2 );

    HAL_GPIO_WritePin( FLASH_WP_PORT, FLASH_WP_PIN, GPIO_PIN_SET );
    HAL_GPIO_WritePin( FLASH_HOLD_PORT, FLASH_HOLD_PIN, GPIO_PIN_SET );
    //Set up the SPI interface
    int status = spi1_init( );
    if ( status != 0 )
    {
        return 1;
    }

    HAL_GPIO_WritePin( FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_SET );

    if ( FLASH_ERROR == flash_check_id( ) )
    {
        return 0;
    }

    initialize_flash( );

    return 0;
}



size_t flash_scan( )
{
    size_t result = 0;
    uint8_t dataRX[256];
    size_t i;
    int j;
    i = FLASH_START_ADDRESS;
    while ( i < FLASH_SIZE_BYTES )
    {
        FlashStatus status;
        for ( j = 0 ; j < 256 ; j++ )
        {
            dataRX[ j ] = 0;
        }

        status = flash_read( i, dataRX, 256 );
        uint16_t empty = 0xFFFF;
        for ( j = 0 ; j < 256 ; j++ )
        {
            if ( dataRX[ j ] != 0xFF )
            {
                empty--;
            }
        }

        if ( empty == 0xFFFF )
        {
            result = i;
            break;
        }

        i = i + 256;
    }

    if ( result == 0 ) result = FLASH_SIZE_BYTES;
    return result;
}






