#ifndef CLI_CONTROLLER_SRC
#define CLI_CONTROLLER_SRC

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <FreeRTOS_CLI.h>
#include "command-line-interface/tools/read.h"
#include "command-line-interface/tools/configure.h"
#include "command-line-interface/tools/mem.h"

#include "protocols/UART.h"
#include "board/board.h"
#include "core/system_configuration.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE	256

/* Dimensions the buffer into which string outputs can be placed. */
#define cmdMAX_OUTPUT_SIZE	2048

/* Dimensions the buffer passed to the recvfrom() call. */
#define cmdSOCKET_INPUT_BUFFER_SIZE 256

/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )

static xTaskHandle handle;

static bool isOptArgSyntaxOk (const char *pcParameter, BaseType_t xParameterStringLength, char *pcWriteBuffer, char * cmd_option, char * str_option_arg);
// ---------------------------------------------------------------------------------------------------------------------
static BaseType_t prvUsageCommand       ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvReadCommand        ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvConfigureCommand   ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvEMatchCommand      ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvMemoryCommand      ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvSaveCommand        ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvStartCommand       ( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvParameterEchoCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static const char *INTRO_MESSAGE =   "========== Welcome to Xtract ==========\r\n"
                                     "This is a command line interface tool made by the Avionics subdivision of the Rockets team.\r\n\r\n"
                                     "Here are some commands to get you started:";

static const char * GENERAL_USAGE = "Commands:\r\n"
                                    "\t[help] - displays the help menu and more commands\r\n"
                                    "\t[read] - Downloads flight data\r\n"
                                    "\t[configure] - Setup flight computer\r\n"
                                    "\t[e-match] - check and fire ematches\r\n"
                                    "\t[mem] - Check on and erase the flash memory\r\n"
                                    "\t[save] - Save all setting to the flight computer\r\n"
                                    "\t[start] - Start the flight computer\r\n";

static const CLI_Command_Definition_t xUsageCommand =
{
        "usage", /* The command string to type. */
        "\r\nusage:\r\n Displays a list of available commands, their description and usage.\r\n"
        "Usage:\r\n "
        "NO ARGUMENTS ARE REQUIRED\n\n",
        prvUsageCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xReadCommand =
{
        "read", /* The command string to type. */
        "\r\nread:\r\n Downloads the flight data from the on-board flash memory.\r\n"
        "Usage:\r\n "
        "NO ARGUMENTS ARE REQUIRED\n\n",
        prvReadCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xConfigureCommand =
{
        "configure", /* The command string to type. */
        "\r\nconfigure:\r\n "
        "Sets up the flight computer configurations.\r\n"
        "Usage:\r\n "
        "[set_data_rate]            - Set data rate Hz(0-100)\r\n "
        "[set_initial_time]         - Set the initial time to wait (0-10000000)\r\n "
        "[record_to_flash]          - Set if recording to flash (1/0)\r\n "
        "[set_accel_bw]             - Set accelerometer bandwidth (0,2,4)\r\n "
        "[set_accel_range]          - Set accelerometer range (3,6,12,24)\r\n "
        "[set_accel_odr]            - Set accelerometer odr (12,25,50,100,200,400,800,1600)\r\n "
        "[set_gyro_bw]              - Set gyro bandwidth and odr (32_100,64_200,12_100,23_200,47_400,116_1000,230_2000,532_2000)\t\t\t      [Enter the number (1-8) for the option to select]\r\n "
        "[set_gyro_range]           - Set gyro range (125,250,500,1000,2000)\r\n "
        "[set_press_odr]            - Set BMP388 odr (1,12,25,50,100,200) \r\n "
        "[set_press_os]             - Set pressure oversampling (0,2,4,8,16,32) \r\n "
        "[set_temp_os]              - Set temperature oversampling (0,2,4,8,16,32) \r\n "
        "[set_press_iir_coeff]      - Set BMP388 IIR filter coefficient (0,1,3,7,15,31,63,127) \r\n "
        "[show]                     - Read the current settings\r\n\n",
        prvConfigureCommand, /* The function to run.  */
        -1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xEMatchCommand =
{
        "e-match", /* The command string to type. */

        "\r\ne-match:\r\n "
        "Checks and fires the e-matches.\r\n"
        "Usage:\r\n "
        "[check_continuity_drogue]  - Check continuity Drogue\r\n "
        "[check_continuity_main]    - Check continuity Main\r\n "
        "[check_overcurrent_drogue] - Check overcurrent Drogue\r\n "
        "[check_overcurrent_main]   - Check overcurrent Main\r\n "
        "[enable_drogue]            - Enable Drogue\r\n "
        "[enable_main]              - Enable Main\r\n "
        "[fire_drogue]              - Fire Drogue (delayed)\r\n "
        "[fire_main]                - Fire Main   (delayed)\r\n "
        "[set_delay]                - Set delay (5-60)\r\n\n",

        prvUsageCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xMemoryCommand =
{
        "mem", /* The command string to type. */
        "\r\nmem:\r\n "
        "Checks on and erases the on-board flash memory.\r\n"
        "Usage:\r\n "
        "[read]                     - Read 256 bytes (hex address 0-7FFFFF).\r\n "
        "[scan]                     - Scan Memory\r\n "
        "[erase_data_section]       - Erase data section\r\n "
        "[erase_config_section]     - Erase config section.\r\n "
        "[erase_all]                - Erase all flash memory.\r\n\n",
        prvMemoryCommand, /* The function to run. */
        -1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xSaveCommand =
{
        "save", /* The command string to type. */
        "\r\nsave:\r\n Saves all the configurations to the flight computer flash memory.\r\n"
        "Usage:\r\n "
        "NO ARGUMENTS ARE REQUIRED\n",
        prvUsageCommand, /* The function to run. */
        0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xStartCommand =
{
        "start", /* The command string to type. */
        "\r\nstart:\r\n Starts the flight computer controller.\r\n"
        "Usage:\r\n "
        "NO ARGUMENTS ARE REQUIRED\n\n",
        prvUsageCommand, /* The function to run. */
        0 /* No parameters are expected. */
};


void vRegisterSampleCLICommands( void )
{
    /* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand( &xUsageCommand );
    FreeRTOS_CLIRegisterCommand( &xReadCommand );
    FreeRTOS_CLIRegisterCommand( &xConfigureCommand );
    FreeRTOS_CLIRegisterCommand( &xEMatchCommand );
    FreeRTOS_CLIRegisterCommand( &xMemoryCommand );
    FreeRTOS_CLIRegisterCommand( &xSaveCommand );
    FreeRTOS_CLIRegisterCommand( &xStartCommand );
}
// ---------------------------------------------------------------------------------------------------------------------


void prv_cli_function(void * pvParams)
{
    uart6_transmit_line(INTRO_MESSAGE);
    uart6_transmit_line(GENERAL_USAGE);

    long lBytes, lByte;
    signed char cInChar, cInputIndex = 0;
    static char cInputString[ cmdMAX_INPUT_SIZE ], cOutputString[ cmdMAX_OUTPUT_SIZE ], cLocalBuffer[ cmdSOCKET_INPUT_BUFFER_SIZE ];
    BaseType_t xMoreDataToFollow;
    volatile int iErrorCode = 0;

    /* As per most FreeRTOS tasks, this task is implemented in an infinite loop. */
    vRegisterSampleCLICommands();

//    int comm_index = 0;
//    char commands[10][256] = {
//            "help",
//             "configure set_data_rate=100 set_accel_range=6 set_gyro_bw=1 set_temp_os=32" ,
//             "configure set_data_rate=100" ,
//             "configure set_accel_range=6 set_gyro_bw=1 set_temp_os=32" ,
//             "configure set_temp_os:=32" ,
//             "configure set_data_rate=100 set_accel_range:=6 set_gyro_bw=1 set_temp_os=32" ,
//             "configure set_data_rate=100 set_accel_range=6 set_gyro_bw=1 set_temp_os=32" ,
//             "help",
//             "help"
//    };

    while(1)
    {
        /* Process the input string received prior to the newline. */
        DISPLAY(">>", NULL);
        INPUT(cInputString);
//        memcpy(cInputString, commands[comm_index], strlen(commands[comm_index]));
//        comm_index++;

//        if(comm_index ==8)
//            comm_index = 0;

        do
        {
            /* Pass the string to FreeRTOS+CLI. */
            xMoreDataToFollow = FreeRTOS_CLIProcessCommand( cInputString, cOutputString, cmdMAX_OUTPUT_SIZE );

            /* Send the output generated by the command's implementation. */
            const char * cOutputStringP = cOutputString;
            DISPLAY("%s", cOutputStringP);
            memset(cOutputString, 0x00, cmdMAX_OUTPUT_SIZE);

        } while( xMoreDataToFollow != pdFALSE ); /* Until the command does not generate any more output. */

        /* All the strings generated by the command processing
        have been sent.  Clear the input string ready to receive
        the next command. */
        cInputIndex = 0;
        memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
    }
}

void command_line_interface_start(void *pvParameters)
{
    if(pdFALSE == xTaskCreate(prv_cli_function, "cli-task", configMINIMAL_STACK_SIZE, NULL, 5, NULL))
    {
        board_error_handler( __FILE__, __LINE__ );
    }
}



static BaseType_t prvUsageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) xWriteBufferLen;
    ( void ) pcCommandString;
    configASSERT( pcWriteBuffer );

    /* generate response */
    strcpy (pcWriteBuffer, GENERAL_USAGE );

    return pdFALSE;
}

BaseType_t prvHelpCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) xWriteBufferLen;
    ( void ) pcCommandString;
    configASSERT( pcWriteBuffer );

    /* generate response */
    strcpy (pcWriteBuffer, GENERAL_USAGE );

    return pdFALSE;
}


static BaseType_t prvReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
   write buffer is not NULL.  NOTE - for simplicity, this example assumes the
   write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) xWriteBufferLen;
    ( void ) pcCommandString;
    configASSERT( pcWriteBuffer );

    /* generate response */
    cli_tools_read(pcWriteBuffer, xWriteBufferLen);

    return pdFALSE;
}



static BaseType_t prvConfigureCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static BaseType_t lParameterNumber = 0;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( lParameterNumber == 0 )
    {
        /* Next time the function is called the first parameter will be echoed
        back. */
        lParameterNumber = 1L;

        /* There is more data to be returned as no parameters have been echoed
        back yet. */
        xReturn = pdPASS;
    }
    else
    {
        /* Obtain the parameter string. */
        pcParameter = FreeRTOS_CLIGetParameter
        (
                pcCommandString,		/* The command string itself. */
                lParameterNumber,		/* Return the next parameter. */
                &xParameterStringLength	/* Store the parameter string length. */
        );

        if( pcParameter != NULL )
        {
            char cmd_option [xParameterStringLength];
            char str_option_arg [xParameterStringLength];
            memset(cmd_option, 0, xParameterStringLength);
            memset(str_option_arg, 0, xParameterStringLength);

            if(isOptArgSyntaxOk(pcParameter, xParameterStringLength, pcWriteBuffer, cmd_option, str_option_arg) ){
                cli_tools_configure(pcWriteBuffer, xWriteBufferLen, cmd_option, str_option_arg);
            }

            /* There might be more parameters to return after this one. */
            xReturn = pdTRUE;
            lParameterNumber++;
        }
        else
        {
            if(lParameterNumber == 1L) /* the command was executed with no parameters */
            {
                cli_tools_configure(pcWriteBuffer, xWriteBufferLen, pcCommandString, xConfigureCommand.pcHelpString);
                DISPLAY("%s", pcWriteBuffer);
            }

            /* No more parameters were found.  Make sure the write buffer does
            not contain a valid string. */
            pcWriteBuffer[ 0 ] = 0x00;

            /* No more data to return. */
            xReturn = pdFALSE;

            /* Start over the next time this command is executed. */
            lParameterNumber = 0;
        }
    }

    return xReturn;
}


static BaseType_t prvMemoryCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static BaseType_t lParameterNumber = 0;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( lParameterNumber == 0 )
    {
        /* Next time the function is called the first parameter will be echoed
        back. */
        lParameterNumber = 1L;

        /* There is more data to be returned as no parameters have been echoed
        back yet. */
        xReturn = pdPASS;
    }
    else
    {
        /* Obtain the parameter string. */
        pcParameter = FreeRTOS_CLIGetParameter
                (
                        pcCommandString,		/* The command string itself. */
                        lParameterNumber,		/* Return the next parameter. */
                        &xParameterStringLength	/* Store the parameter string length. */
                );

        if( pcParameter != NULL )
        {
            char cmd_option [xParameterStringLength];
            char str_option_arg [xParameterStringLength];
            memset(cmd_option, 0, xParameterStringLength);
            memset(str_option_arg, 0, xParameterStringLength);

            if(isOptArgSyntaxOk(pcParameter, xParameterStringLength, pcWriteBuffer, cmd_option, str_option_arg) ){
                cli_tools_mem(pcWriteBuffer, xWriteBufferLen, cmd_option, str_option_arg);
            }

            /* There might be more parameters to return after this one. */
            xReturn = pdTRUE;
            lParameterNumber++;
        }
        else
        {
            if(lParameterNumber == 1L) /* the command was executed with no parameters */
            {
                cli_tools_mem(pcWriteBuffer, xWriteBufferLen, pcCommandString, xMemoryCommand.pcHelpString);
                DISPLAY("%s", pcWriteBuffer);
            }

            /* No more parameters were found.  Make sure the write buffer does
            not contain a valid string. */
            pcWriteBuffer[ 0 ] = 0x00;

            /* No more data to return. */
            xReturn = pdFALSE;

            /* Start over the next time this command is executed. */
            lParameterNumber = 0;
        }
    }

    return xReturn;
}
















static bool isOptArgSyntaxOk (const char *pcParameter, BaseType_t xParameterStringLength, char *pcWriteBuffer, char * cmd_option, char * str_option_arg)
{
    static const char * ILLEGAL_OPT_ARG_CHARACTERS = "!@#$%^&*(),`~/\\|?><'\";:[]{}+";
    bool parse_failure = false;
    uint8_t symbol = ' ';

    /* Return the parameter string. */
    char pcParameterMutable [xParameterStringLength+1];
    strncpy (pcParameterMutable, pcParameter, xParameterStringLength);
    pcParameterMutable[xParameterStringLength] = '\0';

    // Extract the first token
    strcpy(cmd_option, strtok(pcParameterMutable, "="));
    if(cmd_option == NULL)
    {
        sprintf( pcWriteBuffer, "Invalid parameter format: %s\n, correct format is option=argument\n", pcParameter);
        return false;
    }

    for (uint8_t index = 0; index < strlen(ILLEGAL_OPT_ARG_CHARACTERS); index++) {
        symbol = ILLEGAL_OPT_ARG_CHARACTERS[index];
        if (strchr(cmd_option, symbol) != NULL) {
            sprintf(pcWriteBuffer, "Option [%s] contains invalid character: [%c]\n", cmd_option, symbol);
            return false;
        }
    }

    const char * tempStr_option_arg = strtok(NULL, "=");
    if(tempStr_option_arg != NULL) {
        strcpy(str_option_arg, tempStr_option_arg);
        if (str_option_arg == NULL) {
            sprintf(pcWriteBuffer, "Invalid parameter format: %s\n, correct format is option=argument\n", pcParameter);
            return false;
        }

        symbol = ' ';
        for (uint8_t index = 0; index < strlen(ILLEGAL_OPT_ARG_CHARACTERS); index++) {
            symbol = ILLEGAL_OPT_ARG_CHARACTERS[index];
            if (strchr(str_option_arg, symbol) != NULL) {
                sprintf(pcWriteBuffer, "Argument [%s] contains invalid character: [%c]\n", str_option_arg, symbol);
                return false;
            }
        }
    }

    return true;
}




#endif // CLI_CONTROLLER_SRC

