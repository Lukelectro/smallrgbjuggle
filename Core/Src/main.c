/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adxl345.h"
#include "config.h"
#include <stdbool.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
#define I2C_ADR 0xA6 // adxl 345 alternate adres/sdo low (0x53+rw dus 0xA6/7)
unsigned char adxl_read_byte(unsigned char);
void adxl_read_n_bytes(unsigned char, unsigned char, unsigned char* );
void adxl_write_byte(unsigned char, unsigned char);
void adxl_init();
void blink (int num);
void go_sleep();
void setcolor_rgb(unsigned int, unsigned int, unsigned int);
void rainbow();

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
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(NSHDN_GPIO_Port, NSHDN_Pin, 1);	/* Enable 5V supply */
  //HAL_GPIO_WritePin(NSHDN_GPIO_Port, NSHDN_Pin, 0);	/* Disable 5V supply */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); /* BLUE */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); /* RED*/
  HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1); /* GREEN */

  	  /* Flash R,G,B to verify color order is correct */
	setcolor_rgb(PWM_MAX,0,0);
	HAL_Delay(200);
	setcolor_rgb(0,PWM_MAX,0);
	HAL_Delay(200);
	setcolor_rgb(0,0,PWM_MAX);
	HAL_Delay(200);
	setcolor_rgb(0,0,0);
	HAL_Delay(1000);

   unsigned char data=0x00;
   data = adxl_read_byte(ADXL345_DEVID);

   if(data == 0b11100101) /* If ADXL345/343 device ID succesfully read, green on. Else red. */
   {
	   setcolor_rgb(0,PWM_MAX,0);
   }
   else
   {
	   setcolor_rgb(PWM_MAX,0,0);
   }

   HAL_Delay(2500);

	//setup array of nice collors
	int colors[LEN_COLOR];
	for(int i = 0;i<LEN_COLOR;i++){
	colors[i]=colorset_percentage[i]*PWM_MAX/100;
	}

   setcolor_rgb(0,0,0);

   adxl_init();
   blink(2);

   //go_sleep(); /* Start by sleeping, so lights are not blinding during assembly TODO:first configure wake-up interrupts... */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
	   static unsigned char buffer[6], intjes, rgb;
	   static int x,y,z, tap, cc=0;
	   static uint32_t prevtaptick=0, prevfftick=0; // timestamps for tap and freefall
	   static bool Juggle=false, Catch=false, Flying=false; // Juggle in progress? Just catched?
	   static enum modes{direct, catchchange, freefall, blinkcolorwheel, fadecolorwheel, pureRGB} mode;

	   intjes = adxl_read_byte(ADXL345_INT_SOURCE); // read adxl interrupt flags (to sense taps/freefall etc.)
	   // reading resets them, so only read once a cycle

	   // switch mode triple tap
		if((intjes&ADXL345_SINGLE_TAP)&&(!Juggle)){ // on tap but not while juggling
			prevtaptick=HAL_GetTick();
			tap++;
		}

		if(tap>2)
		{ // TRIPLE tap. 3 and up > 2 :)
			tap=0; // reset counter
			mode++; // resets due to default case, so no fuss here.
			setcolor_rgb(0,0,0);
			blink(mode+1); /* +1 because 0 indexed */
		}
		else if( (HAL_GetTick() - 1000) > prevtaptick )
		{ /* assuming tick at 1 Khz, 1 second */
			tap=0;
		}


		if(intjes&ADXL345_FREE_FALL)
		{ // detect freefall to keep time since last freefal to prevent modeswitch during juggle
			prevfftick=HAL_GetTick();
			Juggle = true;
			Flying=true;
		}
		else
		{
		if( (HAL_GetTick() -5000) > prevfftick )
		{ // If a freefall is about 5 seconds ago (TODO: is 10s not a little too long?)
		Juggle = false;
		}

		if( (HAL_GetTick() - 180) > prevfftick)
		{ // If a freefall just ended, assume ball is catched
		//(increase wait time when falsely assumed, decrease when lagging too much)
		// (Uses tick to prevent false catch detection while still falling)
			if(Flying){ // Only detect catches when previously falling (flying). Otherwise lying still counts as a catch...
				Flying = false;
				Catch = true;
			}
		}
	}


		switch(mode){
				case direct: // colour change based on orientation to gravity / test mode

					adxl_read_n_bytes(0x32, 6, buffer); // read xyz in one go
					x=buffer[0]|(buffer[1]<<8);
					y=buffer[2]|(buffer[3]<<8);
					z=buffer[4]|(buffer[5]<<8);

					// because it is 2's complement, if bit 9 is set then bits 31 to 9 should also be set (To convert from 10 bit signed int to 32 bit signed int)
					if(x&1<<9) x|=0xFFFFFC00;
					if(y&1<<9) y|=0xFFFFFC00;
					if(z&1<<9) z|=0xFFFFFC00;

					//scale XYZ to SETPOINT as max
					float g,r,b;
					g=x*PWM_MAX/(1<<8); // adxl is 10 bit, signed.
					r=y*PWM_MAX/(1<<8);
					b=z*PWM_MAX/(1<<8);

					if(g>PWM_MAX) g=PWM_MAX; // crowbar (force safe value)
					if(r>PWM_MAX) r=PWM_MAX; // crowbar (force safe value)
					if(b>PWM_MAX) b=PWM_MAX; // crowbar (force safe value)
					if(g<0) g=0; // crowbar (force safe value / discard if below zero)
					if(r<0) r=0; // crowbar (force safe value / discard if below zero)
					if(b<0) b=0; // crowbar (force safe value / discard if below zero)

					__HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,g); /* GREEN */
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,b); 	/* BLUE */
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,r); 	/* RED */

					break;

				case catchchange:
				// clourchange on catch / juggle modue, End-Of-FreeFall based

				setcolor_rgb(colors[0+cc],colors[1+cc],colors[2+cc]);

				if(Catch){ // on catch:
				//Change colour to the next one from the predefined list for catch
					if ((3+cc)>=LEN_COLOR) cc=0; else cc+=3;
					Catch=false;
				}

				break;


				case freefall: // on freefall, red, on catch: flash green, else: be blue
					if(intjes&ADXL345_FREE_FALL){ // on freefall:
					setcolor_rgb(PWM_MAX,0,0);
					}else if(Catch){ // on catch:
						setcolor_rgb(0,PWM_MAX,0);
						Catch=false;
					}else{
						setcolor_rgb(0,0,PWM_MAX);
						}
					HAL_Delay(200);
					break;
				// a case that keeps changing colour while juggle is true. (juggleball misbehaved as such while testing colorchange on catch and it kind of seems like a nice idea too)
				case blinkcolorwheel:
					if(Juggle){
							if ((3+cc)>=LEN_COLOR) cc=0; else cc+=3;
						setcolor_rgb(colors[0+cc],colors[1+cc],colors[2+cc]);
						HAL_Delay(300);
					}else{
						setcolor_rgb(0,PWM_MAX,0);
					} // So even when not juggling / on modechange, do show something other than "black" on the RGB leds
					break;

				case fadecolorwheel:
					if(Juggle) rainbow(); else setcolor_rgb(PWM_MAX,0,0);
					break;

				case pureRGB:
					if(Catch){ // on catch:
					//Change colour to the next one, only RGB
						setcolor_rgb(0,0,0);
						if (rgb<2) rgb++; else rgb=0;
						Catch=false;
					}
					switch(rgb){ // instead show RGB
						case 0: setcolor_rgb(PWM_MAX,0,0); break;
						case 1: setcolor_rgb(0,PWM_MAX,0); break;
						case 2: setcolor_rgb(0,0,PWM_MAX); break;
					}
					break;

				default:
					mode = 0;
					break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0000020B;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4096;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4096;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSHDN_GPIO_Port, NSHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INPUT_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INPUT_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSHDN_Pin */
  GPIO_InitStruct.Pin = NSHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSHDN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
unsigned char adxl_read_byte(unsigned char addr) {
	unsigned char data;
	HAL_I2C_Master_Transmit(&hi2c1, 0xA6, &addr,1,1000); /* transmit to ADXL345/343 the register adress to be read */
	HAL_I2C_Master_Receive(&hi2c1, 0xA6, &data, 1, 1000); /* read the register */
	return data;
}

void adxl_read_n_bytes(unsigned char addr, unsigned char n, unsigned char* buff) {
	HAL_I2C_Master_Transmit(&hi2c1, 0xA6, &addr,1,1000); /* transmit to ADXL345/343 the first register adress to be read */
	HAL_I2C_Master_Receive(&hi2c1, 0xA6, buff, n, 1000); /* read the register(s) */
}

void adxl_write_byte(unsigned char addr, unsigned char data) {
	unsigned char buffer[2] = {addr,data};
	HAL_I2C_Master_Transmit(&hi2c1, 0xA6, buffer,2,1000); /* transmit to ADXL345/343  */
}

void adxl_init()
{
/*
Datasheet excerpt:
"
Therefore, some experimentation with values for the
DUR, latent, window, and THRESH_TAP registers is required.
In general, a good starting point is to set the DUR register to a
value greater than 0x10 (10 ms), the latent register to a value greater
than 0x10 (20 ms), the window register to a value greater than
0x40 (80 ms), and the THRESH_TAP register to a value greater
than 0x30 (3 g).
"
*/

// Determine treshhold values in actual application (Maybe even at runtime? Meh, no. just calibrate them once. Manually)
	adxl_write_byte(ADXL345_THRESH_TAP, 0x2D); // 62.5mg per increment
	adxl_write_byte(ADXL345_DUR, 0x10);	  // 625us per increment
	adxl_write_byte(ADXL345_LATENT, 0x80);

//tuned these thressholds so it does not wake up during transport but wakes easily when needed
	adxl_write_byte(ADXL345_THRESH_INACT, 20); //20 is OK  <<-- Below this, it goes to sleep
	adxl_write_byte(ADXL345_THRESH_ACT, 120);   //was 20, now 120 <<-- above this, it wakes (So set higher, so it does not wake in transport)

	adxl_write_byte(ADXL345_TIME_INACT, 30); // seconds (30, 2 for test)
	adxl_write_byte(ADXL345_ACT_INACT_CTL, 0xFF); // look for activity/inactivity on all axes, AC coupled
	adxl_write_byte(ADXL345_THRESH_FF, 0x06); // Freefall thresshold. Reccomended between 0x05 and 0x09 (300/600m g)
	adxl_write_byte(ADXL345_TIME_FF, 0x14); //100ms (0x14 - 0x46) 350ms recommended.
	adxl_write_byte(ADXL345_TAP_AXES, 0x07); // detect taps on all axes.
	adxl_write_byte(ADXL345_BW_RATE, 0x1A); // Low power 100Hz (50uA active, lower yet in sleep).
	adxl_write_byte(ADXL345_POWER_CTL,0x08); // 8Hz in sleep, be active now, do not link inactivity/activity, do not autosleep on inactivity
	adxl_write_byte(ADXL345_INT_ENABLE,0x5C);// enable single tap, activity, inactivity & freefall interrupts
	adxl_write_byte(ADXL345_INT_MAP,0x10); // Only activity to INT2 pin
}

void blink (int num){
 setcolor_rgb(0,0,0);
 HAL_Delay(200);

while(num--){
 setcolor_rgb(0,PWM_MAX,0);
 HAL_Delay(100);
 setcolor_rgb(0,0,0);
 HAL_Delay(200);
 }

}


void setcolor_rgb(unsigned int red, unsigned int green, unsigned int blue)
{
   __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,green);
   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,blue);
   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,red);
}

void go_sleep()
{
	/* Use HAL_PWR_EnterSLEEPMode(regulator,PWR_SLEEPENTRY_WFI) to go to sleep. */
	/* Disable tick interrupt first if not wanting to wake-up on tick */
	/* re-enable after wake-up.  */
	HAL_SuspendTick(); /* Stop systick before going to sleep, so it does not wake every 1 ms */

	adxl_read_byte(ADXL345_INT_SOURCE); // read interrupts (and clear them) from adxl
 	// because if it detects activity now, it's too soon to react too an thus will never be reacted too.

	adxl_write_byte(ADXL345_POWER_CTL,0x0C); // put ADXL to sleep at 8 Hz sample rate.
	// lower sample rates do not save more power, so let's update at 8Hz.

	HAL_PWR_EnterSLEEPMode(0,PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();

	adxl_write_byte(ADXL345_POWER_CTL,0x00); // wake ADXL -> standbye
	adxl_write_byte(ADXL345_POWER_CTL,0x08); // standbye -> measure (For lower noise)
	adxl_read_byte(ADXL345_INT_SOURCE); // read interrupts (and clear them) from adxl, so MCU does not get sent back to sleep again by inactivity.

}

void rainbow(){
	/* TODO: maybe try to copy visual effect from fastled, as this is currently not a rainbow. (Though still a nice colorfade) */
	for(int i=0;i<PWM_MAX;i++){
		setcolor_rgb(0,PWM_MAX-i,i);
		HAL_Delay(2);
	}
	for(int i=0;i<PWM_MAX;i++){
		setcolor_rgb(i,0,PWM_MAX-i);
		HAL_Delay(2);
	}
	for(int i=0;i<PWM_MAX;i++){
		setcolor_rgb(PWM_MAX-i,i,0);
		HAL_Delay(2);
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
