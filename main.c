/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
//#define X_AXIS
//#define XYZ_AXIS
#define ALL_EVENTS
//#define TEMP


#define LIS2DH12_ADDR      (0x18 << 1)   // SA0 = 0 → 0x18 (7-bit), shifted for HAL
#define MOTION_THRESHOLD   40   // mg deviation for motion
#define NOMOTION_THRESHOLD 1     // mg tolerance for stationary

#define TEMP_CFG_REG        0x1F
#define OUT_TEMP_L          0x0C
#define OUT_TEMP_H          0x0D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char msg[100];
int8_t temp_offset = 0; // calibration offset
int16_t x_offset = 0, y_offset = 0, z_offset = 0;  		 // Calibration offsets
int32_t x_filtered = 0, y_filtered = 0, z_filtered = 0;  // Simple filtered values
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LIS2DH12_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

uint8_t LIS2DH12_ReadReg(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

void LIS2DH12_Init(void)
{
    // CTRL_REG1 (0x20): enable X,Y,Z axes, 100Hz data rate
	LIS2DH12_WriteReg(0x20, 0x57);
    // CTRL_REG4 (0x23): ±2g full scale, high resolution
	LIS2DH12_WriteReg(0x23, 0x08);
}

void LIS2DH12_TEMP_Init(void)
{
	uint8_t data;
// Enable temperature sensor
    data = 0xC0; // TEMP_EN bits
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, TEMP_CFG_REG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}
void LIS2DH12_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);  //// Read 6 consecutive registers: OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H

    *x = (int16_t)(buffer[1] << 8 | buffer[0]) >> 4;  //// Combine low+high for each axis & Align 12-bit data
    *y = (int16_t)(buffer[3] << 8 | buffer[2]) >> 4;
    *z = (int16_t)(buffer[5] << 8 | buffer[4]) >> 4;
}


void LIS2DH12_Calibrate(void)   // Calibration routine
{
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int16_t x, y, z;

    for (int i = 0; i < 100; i++)
    {
        LIS2DH12_ReadXYZ(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        HAL_Delay(10);
    }

    x_offset = x_sum / 100;
    y_offset = y_sum / 100;
//    z_offset = z_sum / 100;
    z_offset = (z_sum / 100) - 1000; // subtract gravity (~1g)
}


void UART_SendXYZ(int16_t x, int16_t y, int16_t z) // Send over UART
{
    sprintf(msg, "X=%d mg, Y=%d mg, Z=%d mg\r\n", x, y, z);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

}

void Check_Motion(int x_mg, int y_mg, int z_mg)
{
    int z_baseline = 1000 + 100;  // gravity magnitude + motion

    if ( abs(x_mg) > MOTION_THRESHOLD ||
         abs(y_mg) > MOTION_THRESHOLD ||
		 z_mg > z_baseline )
     //   abs(abs(z_mg) - z_baseline) > 1000)
    	{
        	sprintf(msg, " EVENT: Motion detected!\r\n");
        	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    	}

    else if ( (abs(x_mg) > NOMOTION_THRESHOLD && abs(x_mg) < MOTION_THRESHOLD) ||
              (abs(y_mg) > NOMOTION_THRESHOLD && abs(y_mg) < MOTION_THRESHOLD) ||
			  ((z_mg > 1000) && z_mg > z_baseline ) )
             //abs(abs(z_mg) - z_baseline) < 1000)
    			{
        			sprintf(msg, " EVENT: Device is Tilted !\r\n");
        			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    			}
}

// ---------- FREE-FALL INITIALIZATION ----------
void LIS2DH12_Init_FreeFall(void)
{
    uint8_t int1_cfg = 0x95; // XLIE, YLIE, ZLIE (low events on all axes)
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x30, I2C_MEMADD_SIZE_8BIT, &int1_cfg, 1, HAL_MAX_DELAY);

    uint8_t int1_ths = 0x10; // Threshold (16 mg) ±2 g full-scale, high-resolution mode:
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x32, I2C_MEMADD_SIZE_8BIT, &int1_ths, 1, HAL_MAX_DELAY);

    uint8_t int1_dur = 0x02; // Duration = 2 samples
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x33, I2C_MEMADD_SIZE_8BIT, &int1_dur, 1, HAL_MAX_DELAY);
}

void LIS2DH12_Poll_FreeFall(void)
{
    uint8_t src;
//    while (1)
//    {
        HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, 0x31, I2C_MEMADD_SIZE_8BIT, &src, 1, HAL_MAX_DELAY);

        if (src & 0x40) // IA bit set → interrupt active
        {
            sprintf(msg, "Free-fall detected! SRC=0x%02X\r\n", src);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        HAL_Delay(100);
//   }
}

// ---------- TAP INITIALIZATION ----------
void LIS2DH12_Init_Tap(void)
{
    //uint8_t click_cfg = 0x15; // Enable single-click detection on X, Y, Z
    //uint8_t click_cfg = 0x2A; // Enable single-click detection on X, Y, Z
    uint8_t click_cfg = 0x3F;
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x38, I2C_MEMADD_SIZE_8BIT, &click_cfg, 1, HAL_MAX_DELAY);

    uint8_t click_ths = 0x08; // Threshold ~125 mg
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x3A, I2C_MEMADD_SIZE_8BIT, &click_ths, 1, HAL_MAX_DELAY);

    uint8_t time_limit = 0x05; // Max time for tap recognition
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, &time_limit, 1, HAL_MAX_DELAY);

    uint8_t time_latency = 0x05; // Shorter latency
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x3C, I2C_MEMADD_SIZE_8BIT, &time_latency, 1, HAL_MAX_DELAY);

    uint8_t time_window = 0x30; // Longer window for second tap
    HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &time_window, 1, HAL_MAX_DELAY);
}

void LIS2DH12_Poll_Tap(void)
{
    uint8_t src;
//    while (1)
//    {
        HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, 0x39, I2C_MEMADD_SIZE_8BIT, &src, 1, HAL_MAX_DELAY);

        if (src & 0x10) // Single-click detected
        {
            sprintf(msg, "Single Tap detected! SRC=0x%02X\r\n", src);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
        else if (src & 0x20) // Double-click detected
        {
            sprintf(msg, "Double Tap detected! SRC=0x%02X\r\n", src);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        HAL_Delay(100);
//    }
}

int8_t LIS2DH12_ReadTempRaw(void)
{
    uint8_t temp_l, temp_h;

    // Read low byte
    HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, OUT_TEMP_L,
                     I2C_MEMADD_SIZE_8BIT, &temp_l, 1, HAL_MAX_DELAY);

    // Read high byte
    HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, OUT_TEMP_H,
                     I2C_MEMADD_SIZE_8BIT, &temp_h, 1, HAL_MAX_DELAY);

    // Temperature is 8-bit resolution, high byte contains main value
    return (int8_t)temp_h;
}

void LIS2DH12_CalibrateTemp(int8_t reference_temp)
{
    int8_t raw = LIS2DH12_ReadTempRaw();
    temp_offset = reference_temp - raw;
}


void LIS2DH12_PrintTemp(void)
{
	/*
    int8_t temp = LIS2DH12_ReadTemp();
    sprintf(msg, "Temperature Output: %d °C\r\n", temp);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    */
		int8_t raw = LIS2DH12_ReadTempRaw();
	    int8_t calibrated = raw + temp_offset;

	    sprintf(msg, "Raw: %d, Calibrated: %d °C\r\n", raw, calibrated);
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/*
void LIS2DH12_InitMotionINT1(void)
{
    uint8_t cfg;

    // CTRL_REG3 (0x22): enable IA1 interrupt on INT1 pin
    cfg = 0x40;
    LIS2DH12_WriteReg(0x22, cfg);

    // INT1_CFG (0x30): XH, YH, ZH events ORed
    cfg = 0x2A;
    LIS2DH12_WriteReg(0x30, cfg);

    // INT1_THS (0x32): threshold ~16 mg for 2g
    cfg = 0x10;
    LIS2DH12_WriteReg(0x32, cfg);

    // INT1_DURATION (0x33): duration = 5 samples
    cfg = 0x05;
    LIS2DH12_WriteReg(0x33, cfg);

    // CTRL_REG5 (0x24): latch interrupt until cleared
    cfg = 0x08;
    LIS2DH12_WriteReg(0x24, cfg);
}
*/

/*
// --- EXTI Callback for INT1 Pin (PB0 → EXTI0) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {  // PB0 is EXTI0
        uint8_t src = LIS2DH12_ReadReg(0x31); // INT1_SRC register

        if (src & 0x40)
        { // IA bit set
            sprintf(msg, ".....EVENT: Motion detected (INT1)!\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    else
    {
    	 sprintf(msg, "....ELSE-CONDITION (INT1)!\r\n");
    	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
*/

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  LIS2DH12_Init();
  LIS2DH12_Calibrate();

  LIS2DH12_Init_FreeFall();     //FREE-FALL INIT
  LIS2DH12_Init_Tap();			//TAP INIT

  LIS2DH12_TEMP_Init();
  LIS2DH12_CalibrateTemp(25);	//Calibrate once at known ambient (e.g., 25 °C)
  int16_t x, y, z;			    //Local Variables For X,Y,Z-Axis Reading

  /* USER CODE BEGIN 2 */

							// Check registers
							uint8_t ctrl1 = LIS2DH12_ReadReg(0x20);
							uint8_t ctrl4 = LIS2DH12_ReadReg(0x23);
							uint8_t status = LIS2DH12_ReadReg(0x27);

							sprintf(msg, "CTRL1=0x%02X, CTRL4=0x%02X, STATUS=0x%02X \r\n", ctrl1, ctrl4, status);
							HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

// LIS2DH12_Poll_FreeFall();    	// FREE-FALL EVENT
// LIS2DH12_Poll_Tap();				// SINGLE & DOUBLE TAP EVENT

  	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef X_AXIS
	  uint8_t data[2];
	  HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

	  int16_t x_raw = (int16_t)(data[1] << 8 | data[0]);
	  int x_mg = (int)(x_raw * 1); // convert to milli-g (0.98(1) for 2g)

	  sprintf(msg, "X_L=0x%02X, X_H=0x%02X, Raw=%d, X=%d mg\r\n",data[0], data[1], x_raw, x_mg);

	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  HAL_Delay(500);
#endif

#ifdef XYZ_AXIS
	  LIS2DH12_ReadXYZ(&x, &y, &z);


	          x -= x_offset;    // Apply offsets
	          y -= y_offset;
	          z -= z_offset;


	          x_filtered = (x_filtered * 7 + x) / 8;  // Simple moving average filter
	          y_filtered = (y_filtered * 7 + y) / 8;
	          z_filtered = (z_filtered * 7 + z) / 8;

	      UART_SendXYZ(x_filtered, y_filtered, z_filtered);
/*
	      int x_mg = (int)(x_raw * 1);  // For ±2g high-resolution mode: 0.98 mg/LSB ≈ 1 mg/LSB
	      int y_mg = (int)(y_raw * 1);
	      int z_mg = (int)(z_raw * 1);
	      // Print results
	       	      sprintf(msg, " \nSTATUS=0x%02X  X=%d mg, Y=%d mg, Z=%d mg ", status, x_mg, y_mg, z_mg);
	       	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
*/

	      Check_Motion( x_filtered, y_filtered, z_filtered);
	      HAL_Delay(1000);

#endif

#ifdef ALL_EVENTS
	      LIS2DH12_ReadXYZ(&x, &y, &z);

	      	          x -= x_offset;    // Apply offsets
	      	          y -= y_offset;
	      	          z -= z_offset;

	      	          x_filtered = (x_filtered * 7 + x) / 8;  // Simple moving average filter
	      	          y_filtered = (y_filtered * 7 + y) / 8;
	      	          z_filtered = (z_filtered * 7 + z) / 8;
	      	          HAL_Delay(50);
	      	          UART_SendXYZ(x_filtered, y_filtered, z_filtered);

	       Check_Motion( x_filtered, y_filtered, z_filtered);
	       LIS2DH12_Poll_Tap();				// SINGLE & DOUBLE TAP EVENT
	       LIS2DH12_Poll_FreeFall();    	// FREE-FALL EVENT
	       HAL_Delay(100);
#endif

#ifdef TEMP
	      LIS2DH12_PrintTemp();
	      HAL_Delay(1000); // Print every second
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
