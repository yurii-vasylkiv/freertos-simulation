//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2018-2020
//
// Repository:
//  UMSATS Google Drive: UMSATS/Guides and HowTos.../Command and Data Handling (CDH)/Coding Standards
//
// File Description:
//  Reads sensor data for accelerometer and gyroscope from the BMI088
//  On prototype flight computer:
//            +Z is out of the board (perpendicular to board surface when on a table).
//            +X is towards the recovery circuit (away from where the battery connects).
//            +Y is towards the crystal (away from the programming header).
// History
// 2019-03-29 by Benjamin Zacharias
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "imu_sensor.h"
#include "external/sensors/bmi088.h"
#include "protocols/SPI.h"
#include "external/sensors/bmi08x.h"
#include "configuration.h"
#include "cmsis_os.h"
#include "utilities/common.h"

#define INTERNAL_ERROR -127
#define ACC_TYPE             0x800000
#define GYRO_TYPE            0x400000


static QueueHandle_t s_queue;

//Wrapper functions for read and write
int8_t user_spi_read (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

void delay_ms(uint32_t period);

//configuration functions for accelerometer and gyroscope
int8_t accel_config(configuration_data_t * configParams);
int8_t gyro_config (configuration_data_t * configParams);


static struct bmi08x_dev s_device = {
        .accel_id = 0,
        .gyro_id = 1,
        .intf = BMI08X_SPI_INTF, // determines if we use SPI or I2C
        .read = user_spi_read,   //a function pointer to our spi read function
        .write = user_spi_write, //a function pointer to our spi write function
        .delay_ms = delay_ms//user_delay_milli_sec
};



int imu_sensor_init(configuration_data_t * parameters){

    int status = spi3_init();
    if(status != 0)
    {
        return status;
    }

    status = bmi088_init(&s_device); // bosch API initialization method
    if(status != BMI08X_OK)
        return status;

    s_queue = xQueueCreate(10, sizeof(imu_sensor_data));
    if (s_queue == NULL) {
        return 2;
    }

    vQueueAddToRegistry(s_queue, "bmi088_queue");

    status = accel_config(parameters);
    if(status != BMI08X_OK)
    {
        return status;
    }

    status = gyro_config(parameters);
    if(status != BMI08X_OK)
    {
        return status;
    }

    return BMI08X_OK;
}



void imu_thread_start(void const *param)
{
    //Get the parameters.
    imu_sensor_thread_parameters * params = (imu_sensor_thread_parameters *)param;
    configuration_data_t * configParams = params->configuration_data;
    TickType_t prevTime;
    imu_sensor_data dataStruct;

    //main loop: continuously read sensor data
    //vTaskDelay(pdMS_TO_TICKS(100));//Wait so to make sure the other tasks have started.

    prevTime=xTaskGetTickCount();
    
    int8_t result_flag;
    struct bmi08x_sensor_data container;
    while(1){
        
        result_flag = bmi08a_get_data(&container, &s_device);
        if(BMI08X_E_NULL_PTR == result_flag)
        {
            continue;
        }
        dataStruct.acc_x = container.x,
        dataStruct.acc_y = container.y,
        dataStruct.acc_z = container.z;
        
        result_flag = bmi08g_get_data(&container, &s_device);
        if(BMI08X_E_NULL_PTR == result_flag)
        {
            continue;
        }
        
        dataStruct.gyro_x = container.x,
        dataStruct.gyro_y = container.y,
        dataStruct.gyro_z = container.z;
        
        dataStruct.time_ticks = xTaskGetTickCount();

        imu_add_measurement(&dataStruct);
        vTaskDelayUntil(&prevTime,configParams->values.data_rate);
    }
}

bool imu_read(imu_sensor_data * buffer, uint8_t data_rate)
{
    return pdPASS == xQueueReceive(s_queue, buffer, data_rate);
}

void imu_sensor_data_to_bytes(imu_sensor_data reading, uint8_t* buffer, uint32_t timestamp)
{
    // Make sure time doesn't overwrite type and event bits.
    uint32_t header = (ACC_TYPE | GYRO_TYPE) + (timestamp & 0x0FFF);
    write_24(header, &buffer[0]);
    write_16(reading.acc_x, &buffer[3]);
    write_16(reading.acc_y, &buffer[5]);
    write_16(reading.acc_z, &buffer[7]);
    write_16(reading.gyro_x, &buffer[9]);
    write_16(reading.gyro_y, &buffer[11]);
    write_16(reading.gyro_z, &buffer[13]);
}

//set the accelerometer starting configurations
int8_t accel_config(configuration_data_t * configParams){
    uint8_t data = 0;
    int8_t rslt;

    /* Read accel chip id */
    rslt = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &s_device);
    if(rslt != BMI08X_OK)
        return rslt;
    
    /* Assign the desired configurations */
    //Not sure yet what configurations we want
    s_device.accel_cfg.bw = configParams->values.ac_bw;
    s_device.accel_cfg.odr = configParams->values.ac_odr;
    s_device.accel_cfg.range = configParams->values.ac_range;
    s_device.accel_cfg.power = configParams->values.ac_pwr;

    rslt = bmi08a_set_power_mode(&s_device);
    if(rslt != BMI08X_OK)
        return rslt;
    /* Wait for 10ms to switch between the power modes - delay_ms taken care inside the function*/

    rslt = bmi08a_set_meas_conf(&s_device);
    if(rslt != BMI08X_OK)
        return rslt;

    return rslt;
}

//set the accelerometer starting configurations
int8_t gyro_config(configuration_data_t * configParams)
{
    uint8_t data = 0;
    int8_t rslt;

    /* Read gyro chip id */
    rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &s_device);
    if(rslt != BMI08X_OK)
        return rslt;
    
    
    //set power mode
    s_device.gyro_cfg.power = configParams->values.gy_pwr;
    rslt = bmi08g_set_power_mode(&s_device);
    if(rslt != BMI08X_OK)
        return rslt;
    
    /* Wait for 30ms to switch between the power modes - delay_ms taken care inside the function*/
    /* Assign the desired configurations */
   s_device.gyro_cfg.odr = configParams->values.gy_odr;
   s_device.gyro_cfg.range = configParams->values.gy_range;
   s_device.gyro_cfg.bw = configParams->values.gy_bw;
   s_device.gyro_id = data;

    rslt = bmi08g_set_meas_conf(&s_device);
    if(rslt != BMI08X_OK)
        return rslt;
    
    return rslt;
}

int8_t user_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int status;
    //debug removeHAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN, GPIO_PIN_SET);
    if(dev_addr == 0x00|| dev_addr == 0x1E){
        //Accelerometer.
        if((status = spi3_receive(&reg_addr,1, data, len, 10)) != 0) // The register address will always be 1.
        {
            return status;
        }
    }
    else if(dev_addr == 0x01|| dev_addr == 0x0F){
//        //Gyroscope.
        if((status = spi3_receive(&reg_addr,1, data, len, 11)) != 0) // The register address will always be 1.
        {
            return status;
        }
    }

    //delay_ms(500);
    //HAL_GPIO_WritePin(USR_LED_PORT,USR_LED_PIN, GPIO_PIN_RESET);
    //delay_ms(500);
    return BMI08X_OK;
}

int8_t user_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){

    if(dev_addr == 0x00 || dev_addr == 0x1E){
    spi3_send(&reg_addr,1, data, len, 10);
    }
    else if(dev_addr == 0x01 || dev_addr == 0x0F){
        spi3_send(&reg_addr,1, data, len, 11);
    }
    return BMI08X_OK;
}

void delay_ms(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period)); // wait for the given amount of milliseconds
}


static int8_t __imu_config(configuration_data_t * parameters)
{
    int8_t result = accel_config(parameters);
    if(result != BMI08X_OK)
    {
        return false;
    }

    result = gyro_config(parameters);
    if(result != BMI08X_OK)
    {
        return false;
    }
    
    return result == BMI08X_OK;
}

static uint8_t __imu_init()
{

}


int imu_sensor_test()
{
//    int8_t result;
//    result = bmi08a_perform_selftest(s_bmp3_sensor->bmi088_ptr);
//    if(result != BMI08X_OK)
//        return false;
//
//    result = bmi08g_perform_selftest(s_bmp3_sensor->bmi088_ptr);
//    if(result != BMI08X_OK)
//        return false;
    
    char res = 0;
    uint8_t id = 0x1E;
    
    uint8_t command[] = {0x80};
    uint8_t id_read[] = {0x00,0x00,0x00,0x00};
    uint8_t id_dummy[] = {0x00,0x00};
    
    
    spi3_receive(command,1,id_dummy,2,10);
    spi3_receive(command,1,id_read,2,10);
    
    if(id_read[1] == id)
    {
        res += 1;
    }
    
    spi3_receive(command,1,id_read,2,11);
    
    if(id_read[0] == 0x0F)
    {
        res += 1;
    }
    
    if(res == 2)
    {
        return 1;
    }
    
    return 0;
}

bool imu_add_measurement (imu_sensor_data * _data)
{
    return pdTRUE == xQueueSend(s_queue, _data,1);
}

