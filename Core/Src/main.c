/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct gpio{
    GPIO_TypeDef *port;
    uint16_t pin;
}gpio;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KLIPPER_PWM_CYCLE_TIME 0.002
#define KLIPPER_PWM_FREQUENCY 1/KLIPPER_PWM_CYCLE_TIME

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//overwrite printf
int _write(int file, char *ptr, int len)
{
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
gpio pwm1 = {INPUT_1_GPIO_Port, INPUT_1_Pin};
gpio pwm2 = {INPUT_2_GPIO_Port, INPUT_2_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void ESC_init_sequence(TIM_HandleTypeDef *handle, uint8_t channel){
    __HAL_TIM_SET_COMPARE(handle, channel, 0);
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(handle, channel, 50);
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(handle, channel, 0);

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t PWM_IN_1 = 0;
uint8_t PWM_IN_2 = 0;


uint32_t Frequency = 0;
uint8_t interrupt_number_flag = 0;
uint8_t StartStopFlag = 0;
uint32_t main_frequency=0;

void TIM3_IRQ(void){
    switch(interrupt_number_flag) {
        case 0:
//            measure_PWM(pwm2, &PWM_IN_2);
            interrupt_number_flag = 1;


//            if(StartStopFlag==0 && HAL_GPIO_ReadPin(pwm1.port, pwm1.pin) == 1){
//                PWM_IN_1=100;
//            } else if(StartStopFlag==0 && HAL_GPIO_ReadPin(pwm1.port, pwm1.pin) == 0){
//                PWM_IN_1=0;
//            }
            StartStopFlag = 0;

            break;
        case 1:
//            measure_PWM(pwm2, &PWM_IN_1);
            interrupt_number_flag = 0;

//            if(StartStopFlag==0 && HAL_GPIO_ReadPin(pwm2.port, pwm2.pin) == 1){
//                PWM_IN_2=100;
//            } else if(StartStopFlag==0 && HAL_GPIO_ReadPin(pwm2.port, pwm2.pin) == 0){
//                PWM_IN_2=0;
//            }
            StartStopFlag = 0;

            break;
        default:
            break;
    }


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(interrupt_number_flag) {
        case 0:
            if (GPIO_Pin == pwm1.pin && StartStopFlag ==0 && HAL_GPIO_ReadPin(pwm1.port, pwm1.pin) == 1){
                __HAL_TIM_SET_COUNTER(&htim2, 0);
//                HAL_TIM_Base_Start(&htim2);
                StartStopFlag = 1;
            }
            else if (GPIO_Pin == pwm1.pin && StartStopFlag ==1&& HAL_GPIO_ReadPin(pwm1.port, pwm1.pin) == 0){
//                HAL_TIM_Base_Stop(&htim2);
                StartStopFlag = 2;
//                uint32_t IC_Value1 = TIM2->CNT;
                PWM_IN_1 = TIM2->CNT*(htim2.Init.Prescaler+1)*100/( main_frequency *KLIPPER_PWM_CYCLE_TIME);
            }
            break;
        case 1:
            if (GPIO_Pin == pwm2.pin && StartStopFlag ==0 && HAL_GPIO_ReadPin(pwm2.port, pwm2.pin) == 1){
                __HAL_TIM_SET_COUNTER(&htim2, 0);
//                HAL_TIM_Base_Start(&htim2);
                StartStopFlag = 1;
            }
            else if (GPIO_Pin == pwm2.pin && StartStopFlag ==1&& HAL_GPIO_ReadPin(pwm2.port, pwm2.pin) == 0){
//                HAL_TIM_Base_Stop(&htim2);
                StartStopFlag = 2;
                uint32_t IC_Value1 = TIM2->CNT;
                PWM_IN_2 = IC_Value1*(htim2.Init.Prescaler+1)*100/( main_frequency *KLIPPER_PWM_CYCLE_TIME);
            }
            break;
        default:
            break;
    }

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    main_frequency = HAL_RCC_GetSysClockFreq();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    ESC_init_sequence(&htim1, TIM_CHANNEL_1);
    ESC_init_sequence(&htim1, TIM_CHANNEL_3);


    HAL_TIM_Base_Init(&htim3);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
       if (htim1.Instance->CCR1 != PWM_IN_1 &&htim1.Instance->CCR1 !=0){
          htim1.Instance->CCR1 = PWM_IN_1;
      }else if(htim1.Instance->CCR1 ==0 && PWM_IN_1 !=0){
//          ESC_init_sequence(&htim1, TIM_CHANNEL_1);
            htim1.Instance->CCR1 = PWM_IN_1;
      }

      if (htim1.Instance->CCR3 != PWM_IN_2 &&htim1.Instance->CCR3 !=0){
          htim1.Instance->CCR3 = PWM_IN_2;
      }else if(htim1.Instance->CCR3 ==0 && PWM_IN_2 !=0) {
          ESC_init_sequence(&htim1, TIM_CHANNEL_3);
          htim1.Instance->CCR3 = PWM_IN_2;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
