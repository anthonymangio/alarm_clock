/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_InitTypeDef myGPIO_InitStruct = {0};
uint32_t previousTimeInms = 0;
uint32_t currentTimeInms = 0;
uint8_t keyPressed = 0;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm = {0};
int state = 0b00000000;
int i = 0;
int alarmFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void send_to_lcd (char data, int rs);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_put_cur(int row, int col);
void lcd_init (void);
void lcd_send_string (char *str);
void lcd_clear (void);
void delay (uint16_t us);
int convertASCIItoInt(char one, char two);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char msg[32];
  char input[4];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

  lcd_init();
  HAL_Delay(1); 
  
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //Enter Current Time State
    while(state == 0b00000000){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        sprintf(msg, "Enter Current");
        lcd_send_string(msg);  
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "24hr:  Hr  Min");
        lcd_send_string(msg); 
        HAL_Delay(1);
        i++;     
      }

      if (keyPressed){
        if(i<3)
          lcd_put_cur(1, 4+i);
        else
          lcd_put_cur(1, 6+i);
        HAL_Delay(1);
        sprintf(msg, "%c", keyPressed);
        lcd_send_string(msg);
        HAL_Delay(1);
        input[i-1] = keyPressed;
        keyPressed = 0;
        i++;
      }
      if(i == 5){
        state = 0b00000001; // Next State is Check Current Time State
        i = 0;
      }
    }
    // Check Current Time State
    while(state == 0b00000001){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        sprintf(msg, "Time: %c%cHr %c%cMin",input[0],input[1],input[2],input[3]);
        lcd_send_string(msg);  
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "Correct? Y-1 N-2");
        lcd_send_string(msg); 
        i++;     
      }  
      if (keyPressed){
        if (keyPressed == 49){
          sTime.Hours = convertASCIItoInt(input[0], input[1]);
          sTime.Minutes = convertASCIItoInt(input[2], input[3]);
          sTime.Seconds = 0;
          HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
          state = 0b00000010; // Next State is Display Current Time State
          i = 0;
        }
        else if(keyPressed == 50){
          state = 0b00000000; // Next State is Enter Current Time State
          i = 0;
        }

        keyPressed = 0;
      }
    }
    // Display Current Time State
    while(state == 0b00000010){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "Set Alm-1 Time-2" );
        lcd_send_string(msg);  
        i++;     
      }  
      // Refresh Time Periodically for the Display
      lcd_put_cur(0, 0);
      HAL_Delay(1);
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
      HAL_RTC_GetAlarm(&hrtc,&sAlarm,RTC_ALARM_A,RTC_FORMAT_BIN);
      if(alarmFlag == 1)
        sprintf(msg,"Time:%02d.%02d.%02d %s",sTime.Hours,sTime.Minutes,sTime.Seconds, "ON");
      else
        sprintf(msg,"Time:%02d.%02d.%02d %s",sTime.Hours,sTime.Minutes,sTime.Seconds, "  ");
      lcd_send_string(msg); 
      HAL_Delay(1);

      if (keyPressed){
        if (keyPressed == 49){
          state = 0b00000100; // Next State is Alarm Set State
          i = 0;
        }
        else if(keyPressed == 50){
          state = 0b00000000; // Next State is Enter Current Time State
          i = 0;
        }
        keyPressed = 0;
      }
    }    
    // Alarm Set State
    while(state == 0b00000100){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        sprintf(msg, "Enter Alarm Time");
        lcd_send_string(msg);  
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "24hr:  Hr  Min");
        lcd_send_string(msg); 
        i++;     
      }

      if (keyPressed){
        if(i<3)
          lcd_put_cur(1, 4+i);
        else
          lcd_put_cur(1, 6+i);
        HAL_Delay(1);
        sprintf(msg, "%c", keyPressed);
        lcd_send_string(msg);
        HAL_Delay(1);
        input[i-1] = keyPressed;
        keyPressed = 0;
        i++;
      }
      if(i == 5){
        state = 0b00001000; // Next State is Check Alarm Check State
        i = 0;
      }
    }
    // Alarm Check State
    while(state == 0b00001000){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        sprintf(msg, "Time: %c%cHr %c%cMin",input[0],input[1],input[2],input[3]);
        lcd_send_string(msg);  
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "Correct? Y-1 N-2");
        lcd_send_string(msg); 
        i++;     
      }  
      if (keyPressed){
        if (keyPressed == 49){
          sAlarm.AlarmTime.Hours = convertASCIItoInt(input[0], input[1]);
          sAlarm.AlarmTime.Minutes = convertASCIItoInt(input[2], input[3]);
          sAlarm.Alarm = RTC_ALARM_A;
          HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
          HAL_RTC_SetAlarm_IT(&hrtc,&sAlarm,RTC_FORMAT_BIN);
          //HAL_RTC_
          state = 0b00000010; // Next State is Display Current Time State
          i = 0;
          alarmFlag = 1;
        }
        else if(keyPressed == 50){
          state = 0b00000100; // Next State is Enter Alarm Time State
          i = 0;
        }

        keyPressed = 0;
      }
    }
    // Alarm State
    while(state == 0b00010000){
      if(i == 0){
        lcd_clear();
        HAL_Delay(1); 
        sprintf(msg, "Press Any Button" );
        lcd_send_string(msg);  
        HAL_Delay(1); 
        lcd_put_cur(1, 0);
        HAL_Delay(1); 
        sprintf(msg, "to Disable" );
        lcd_send_string(msg);  
        HAL_Delay(1); 
        i++;     
      } 
      HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin, 1); // Turn on Buzzer
      
      if (keyPressed){
        state = 0b00000010; // Next State is Display Current Time State
        keyPressed = 0;
        HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin, 0); // Turn off Buzzer
        i = 0;
      }
    }
    
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentTimeInms = HAL_GetTick();
  if (currentTimeInms - previousTimeInms > 100) {
    myGPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
    myGPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    myGPIO_InitStruct.Pull = GPIO_PULLDOWN;
    myGPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &myGPIO_InitStruct);

    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0);

    if(GPIO_Pin == C1_Pin && HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))
    {
      keyPressed = 49; //ASCII value of 1
    }
    else if(GPIO_Pin == C2_Pin && HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))
    {
      keyPressed = 50; //ASCII value of 2
    }
    else if(GPIO_Pin == C3_Pin && HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))
    {
      keyPressed = 51; //ASCII value of 3
    }
    else if(GPIO_Pin == C4_Pin && HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))
    {
      keyPressed = 65; //ASCII value of A
    }

    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0);

    if(GPIO_Pin == C1_Pin && HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))
    {
      keyPressed = 52; //ASCII value of 4
    }
    else if(GPIO_Pin == C2_Pin && HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))
    {
      keyPressed = 53; //ASCII value of 5
    }
    else if(GPIO_Pin == C3_Pin && HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))
    {
      keyPressed = 54; //ASCII value of 6
    }
    else if(GPIO_Pin == C4_Pin && HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))
    {
      keyPressed = 66; //ASCII value of B
    }

    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0);

    if(GPIO_Pin == C1_Pin && HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))
    {
      keyPressed = 55; //ASCII value of 7
    }
    else if(GPIO_Pin == C2_Pin && HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))
    {
      keyPressed = 56; //ASCII value of 8
    }
    else if(GPIO_Pin == C3_Pin && HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))
    {
      keyPressed = 57; //ASCII value of 9
    }
    else if(GPIO_Pin == C4_Pin && HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))
    {
      keyPressed = 67; //ASCII value of C
    }

    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

    if(GPIO_Pin == C1_Pin && HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))
    {
      keyPressed = 42; //ASCII value of *
    }
    else if(GPIO_Pin == C2_Pin && HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))
    {
      keyPressed = 48; //ASCII value of 0
    }
    else if(GPIO_Pin == C3_Pin && HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))
    {
      keyPressed = 35; //ASCII value of #
    }
    else if(GPIO_Pin == C4_Pin && HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))
    {
      keyPressed = 68; //ASCII value of D
    }

    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);

    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
    myGPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    myGPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(C1_GPIO_Port, &myGPIO_InitStruct);

    previousTimeInms = currentTimeInms;
  }
}

void send_to_lcd (char data, int rs)
{
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);  // rs = 1 for data, rs=0 for command
    /* write the data to the respective pin */
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data>>3)&0x01));
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data>>2)&0x01));
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data>>1)&0x01));
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data>>0)&0x01));
    /* Toggle EN PIN to send the data
     * if the HCLK > 100 MHz, use the  20 us delay
     * if the LCD still doesn't work, increase the delay to 50, 80 or 100..
     */
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 1);
  //delay (100);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, 0);
}

void lcd_send_cmd (char cmd)
{
    char datatosend;
    /* send upper nibble first */
    datatosend = ((cmd>>4)&0x0f);
    send_to_lcd(datatosend,0);  // RS must be while sending command
    /* send Lower Nibble */
    datatosend = ((cmd)&0x0f);
    send_to_lcd(datatosend, 0);
}

void lcd_send_data (char data)
{
    char datatosend;
   
    /* send higher nibble */
    datatosend = ((data>>4)&0x0f);
    send_to_lcd(datatosend, 1);  // rs =1 for sending data
    /* send Lower nibble */
    datatosend = ((data)&0x0f);
    send_to_lcd(datatosend, 1);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}

void lcd_init (void)
{
    //4 bit initialisation
    HAL_Delay(500);  // wait for >40ms
    lcd_send_cmd (0x30);
    HAL_Delay(5);  // wait for >4.1ms
    lcd_send_cmd (0x30);
    HAL_Delay(1);  // wait for >100us
    lcd_send_cmd (0x30);
    HAL_Delay(10);
    lcd_send_cmd (0x20);  // 4bit mode
    HAL_Delay(10);

  // display initialisation
    lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd (0x01);  // clear display
    HAL_Delay(1);
    lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
    HAL_Delay(1);
}

void lcd_send_string (char *str)
{
  int i = 0;

	while (i < strlen(str))
  {
    lcd_send_data (str[i]);
    HAL_Delay(1);
    i++;
  }
}

void lcd_clear (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void delay (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

int convertASCIItoInt(char one, char two)
{
  int digit1 = one - '0';
  int digit2 = two - '0';
  int result = digit1 * 10 + digit2;
  return result;
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) 
{
    state = 0b00010000; // enter alarm state
    i = 0;
}

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
