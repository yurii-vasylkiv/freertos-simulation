#include "STM32.h"
#include "hardware_definitions.h"


static STM32Status system_clock_config(void);
static void GPIO_init(void);

STM32Status stm32_init(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_StatusTypeDef result = HAL_Init();
    if(result != HAL_OK)
    {
        return (STM32Status)result;
    }
    
    /* Configure the system clock */
    if(system_clock_config() != STM32_OK)
    {
        return STM32_SYS_CLOCK_CONFIG_ERROR;
    }
    
    /* Initialize all configured peripherals */
    GPIO_init();
    
    return STM32_OK;
}

void stm32_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void stm32_led_blink(uint32_t ms)
{
    HAL_GPIO_TogglePin(USR_LED_PORT,USR_LED_PIN);
    stm32_delay(ms);
}


STM32Status system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /**Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState              = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource         = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM              = 8;
    RCC_OscInitStruct.PLL.PLLN              = 84;
    RCC_OscInitStruct.PLL.PLLP              = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ              = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        return STM32_SYS_CLOCK_CONFIG_ERROR;
    }
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType             = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource          = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider         = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider        = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider        = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        return STM32_SYS_CLOCK_CONFIG_ERROR;
    }
    
    return STM32_OK;
}


void GPIO_init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    //set up PA5 as output.
    GPIO_InitStruct.Pin                     = USR_LED_PIN;
    GPIO_InitStruct.Mode                    = GPIO_MODE_OUTPUT_PP;
    
    HAL_GPIO_Init(USR_LED_PORT,&GPIO_InitStruct);
    
    GPIO_InitTypeDef GPIO_InitStruct2;
    
    GPIO_InitStruct2.Pin                    = USR_PB_PIN;
    GPIO_InitStruct2.Mode                   = GPIO_MODE_INPUT;
    GPIO_InitStruct2.Pull                   = GPIO_PULLUP;
    
    HAL_GPIO_Init(USR_PB_PORT,&GPIO_InitStruct2);
}

void stm32_error_handler(const char* file, uint32_t line)
{
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
        printf("%s:%i:ERROR!\n", file, line);
        // stm32_led_blink(50);
    }
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }

}
