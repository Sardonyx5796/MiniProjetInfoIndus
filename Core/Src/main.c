/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "HorombeRGB565.h"
#include "fond1.h"
#include <string.h>
#include <math.h>
#include "angele_wav.h"
#include "boum.h"
#include "clack.h"
#include "kk.h"
#include "pa.h"
#include "po.h"
#include "tou1.h"
#include "tou2.h"
#include "tou3.h"
#include "tss.h"
# define SIZE_BUFFER 2048

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
uint16_t sinus12bit[360];
uint32_t index_fichier = 0;
uint16_t buffer[2048];
uint16_t sound2play = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	// uint32_t bp1, bp1_old;

	char text[50]={};
	uint32_t periode;
	static TS_StateTypeDef  TS_State;
	// uint32_t potl,potr,joystick_h, joystick_v;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	//RTC_TimeTypeDef stime;
	//RTC_DateTypeDef sdate;
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
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_ADC3_Init();
  MX_UART7_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+ BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_RED);
  BSP_LCD_DrawBitmap(0,0,(uint8_t*)fond1_bmp);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(00);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_SetBackColor(00);

  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
/*
  for(int i=0; i<360;i++)
  {
  sinus12bit[i] = 2048 + 1024*sin(i*3.14145/180);
  }
*/
  //Démarrage du timer 7 avec interruptions
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
  Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sound2play = 0;

	  int last_x = 0;
	  int last_y = 0;
	  BSP_TS_GetState(&TS_State);

	  if(TS_State.touchDetected){
		  BSP_LCD_Clear(0);
	  	  BSP_LCD_FillCircle(TS_State.touchX[0],TS_State.touchY[0],20);
	  	  last_x = TS_State.touchX[0];
	  	  last_y = TS_State.touchY[0];
	  }


	  if (last_x>40 && last_x<120 && last_y>40 && last_y<80){
		  sound2play = 1;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (tou1_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>140 && last_x<220 && last_y>40 && last_y<80){
		  sound2play = 2;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (tou2_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>240 && last_x<320 && last_y>40 && last_y<80){
		  sound2play = 3;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (tou3_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>40 && last_x<120 && last_y>100 && last_y<140){
		  sound2play = 4;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (boum_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>140 && last_x<220 && last_y>100 && last_y<140){
		  sound2play = 5;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (clack_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>240 && last_x<320 && last_y>100 && last_y<140){
		  sound2play = 6;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (kk_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>40 && last_x<120 && last_y>160 && last_y<200){
		  sound2play = 7;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (pa_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>140 && last_x<220 && last_y>160 && last_y<200){
		  sound2play = 8;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (po_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>240 && last_x<320 && last_y>160 && last_y<200){
		  sound2play = 9;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (tss_sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }


	  if (last_x>400 && last_x<440 && last_y>40 && last_y<80){
		  // Angele base
		  sound2play = 0;
		  // Initialisation du buffer avec le début du son
	      for (int i= 0;i<SIZE_BUFFER;i++)
	    	  buffer[i] = (sounddata_data[index_fichier++] << 4);
	      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) buffer, SIZE_BUFFER, DAC_ALIGN_12B_R);
	  }
	  if (last_x>400 && last_x<440 && last_y>100 && last_y<140){
		  // Stop
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	  }
	  if (last_x>400 && last_x<440 && last_y>160 && last_y<200){
		  // Stop
		  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }sounddata_data
}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	/* Fonction d'interruption se déclenchant lorsque
	 * la moitié du buffer a été transmise au DAC par le DMA
	 * Pendant que le DMA s'occupe de transmettre la deuxième moitié du buffer,
	 * on actualise les valeurs de la première moitié du buffer avec la suite de notre son.
	 * Le son est lu en boucle
	 */

	if (sound2play ==0) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==1) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (tou1_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (tou1_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==2) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (tou2__sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (tou2_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==3) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (tou3__sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (tou3_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==4) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (boum_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==5) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (clack_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (clack_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==6) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (kk_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (kk_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==7) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (pa_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (pa_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==8) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (po_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (po_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==9) {

		HAL_GPIO_TogglePin(LED18_GPIO_Port, LED18_Pin);
		if (index_fichier < (tss_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = 0; i<(SIZE_BUFFER/2);i++)
				buffer[i] = (tss_sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
}
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	/* Fonction d'interruption se déclenchant lorsque
	 * la totalité du buffer a été transmise au DAC par le DMA
	 * Pendant que le DMA s'occupe de transmettre la première moitié du buffer,
	 * on actualise les valeurs de la deuxième moitié du buffer avec la suite de notre son.
	*/

	if (sound2play == 0){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 1){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 2){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play ==3){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 4){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 5){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 6){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 7){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 8){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}
	if (sound2play == 9){

		if (index_fichier < (boum_sounddata_length - SIZE_BUFFER/2)) {
			for (int i = SIZE_BUFFER/2;i<SIZE_BUFFER;i++)
				buffer[i] = (sounddata_data[index_fichier++]<<4);
		}
		else
			index_fichier = 0;
	}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
	  HAL_GPIO_TogglePin(LED16_GPIO_Port, LED16_Pin);
  }
  /* USER CODE END Callback 1 */
}

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
