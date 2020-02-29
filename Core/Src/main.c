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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NEX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char nexByte;
char nexBuf[NEX_BUFFER_SIZE];
size_t nexBufLen = 0;
size_t ffCount = 0;
char CardFlag = '0';
char codeAvtSer[4] = {0};
char codeAvtNom[10] = {0};
uint8_t flagAvt = 0;
uint8_t codeSer;
uint16_t codeNom;
uint8_t codeCardSer;
uint16_t codeCardNom;
int codeAvtSerSave = 0;
int codeAvtNomSave = 0;
uint8_t i = 0;
uint8_t chet = 0;
uint8_t chet1 = 0;
uint8_t pagenom = 20;
int j=0;
uint8_t flagRightCard = 0;
char text[20] = " Card's code";
char str[50]= {0};
uint8_t shlag = 0;
uint8_t preshlag = 0;
char codeCardHEXtext[12];
int codesCardsSave[10][2] = {0};
uint8_t kolcards = 0;
uint8_t plusUs = 0;
uint8_t nomminusUs = 20;
uint8_t per3 = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //ѕСаРСЮвЪР ЯаХалТРЭШЩ Юв‚ вРЩЬХаЮТ
{
	if (htim->Instance==TIM1){                 //їаХалТРЭШХ ЯаШиЫЮ Юв TIM1”
		shlag = 0;
	}
}
void delay_ms(uint16_t value)      //ЧРФХаЦЪР б ЯаХалТРЭШХЬ ЭР value (10^(-4))
{
  TIM1->ARR = value;                  //ФЮ ЪРЪЮУЮ ЧЭРзХЭШп СгФХв бзШвРвм вРЩЬХа
  TIM1->CNT = 0;                    //ѕСЭгЫХЭШХ вРЩЬХаР
  TIM1->CR1 = TIM_CR1_CEN;	      //ТЪЫозХЭШХ вРЩЬХаРЂ
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  TIM4->CCR4=87;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  //·ХЫХЭлЩ бТХвЮФШЮФђ
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  //єаРбЭлЩ бТХвЮФШЮФ”
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  //·ТгЪ”
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&nexByte, 1);

  while (1)
  {
	  if(nomminusUs < kolcards)   //ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп
	  {
		  for(uint8_t nomc = nomminusUs; nomc < kolcards; nomc++)     //БФТШУ ТбХе ЪРав ТТХае
		  {
			  codesCardsSave[nomc][0] = codesCardsSave[nomc + 1][0];
			  codesCardsSave[nomc][1] = codesCardsSave[nomc + 1][1];
    		  sprintf(str,"users.t%d.txt=\"%d %d\"\xff\xff\xff", nomc, codesCardsSave[nomc][0], codesCardsSave[nomc][1]);
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
		  }
		  sprintf(str,"users.t%d.txt=\"\"\xff\xff\xff", (kolcards - 1));
		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
		  kolcards--;
		  nomminusUs = 20;
	  }
	  nomminusUs = 20;
	  if(shlag != preshlag)  //їХаХЪЫозХЭШХ иЫРУСРШЬРђ
	  {
		  if(shlag == 0)
		  {
			  TIM4->CCR4=87;    //БХаТЮЯаШТЮФ Т 0
			  sprintf(str,"shlag.bt0.val=0\xff\xff\xff");     //БЬХЭР ЯЮЫЮЦХЭШп ЪЭЮЯЪШ ЭР ФШбЯЫХХ
			  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
		  } else if(shlag == 1)
		  {
			  TIM4->CCR4=380;  //БХаТЮЯаШТЮФ Т 180
		  }
		  preshlag = shlag;
	  }
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) //їаЮТХаЪР Бјєљ
	  {
		  shlag = 0;

	  }

	  if(flagAvt == 1)  //їаШиХЫ ЪЮФ ЪРавл б ЪЫРТШРвгал‹
	  {
		  sprintf(text, "%s\r\n", text);
		  CDC_Transmit_FS(text, strlen(text));
		  codeAvtSerSave = atoi(codeAvtSer);
		  codeAvtNomSave = atoi(codeAvtNom);
		  memset(codeAvtSer, 0, 4);
		  memset(codeAvtNom, 0, 10);
		  HAL_Delay(100);
		  flagAvt = 0;
		  codesCardsSave[kolcards][0] = codeAvtSerSave;
		  codesCardsSave[kolcards][1] = codeAvtNomSave;
		  kolcards++;
	  }
	  if(CardFlag == '1')  //їаШиХЫ ЪЮФ ЪРавл бЮ бзШвлТРвХЫпЏ
	  {
		  if(plusUs == 1){     //ЅРЦРвР ЪЭЮЯЪР ФЮСРТЫХЭШп ЯЮЫмЧЮТРвХЫпЏ
			  codesCardsSave[kolcards][0] = codeCardSer;    //ґЮСРТЫХЭШХ ЭЮЬХаР ЪРавл Т ЬРббШТ
			  codesCardsSave[kolcards][1] = codeCardNom;
    		  sprintf(str,"users.t%d.txt=\"%d %d\"\xff\xff\xff", kolcards, codeCardSer, codeCardNom);  //ѕвЮСаРЦХЭШХ ЭЮЬХаР ЭР ФШбЯЫХХ
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
    		  sprintf(str,"vis b%d,1\xff\xff\xff", kolcards);
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
			  sprintf(str, "ґЮСРТЫХЭ ЯЮЫмЧЮТРвХЫм %d", kolcards);
			  CDC_Transmit_FS(str, strlen(str));
    		  kolcards++;
			  plusUs = 0;
		  } else{
	      for(uint8_t p = 0; p < kolcards; p++)   //БаРТЭХЭШХ бЮ ТбХЬШ ЪРавРЬШ ШЧ бЯШбЪР
	      {
	    	  if(codesCardsSave[p][0] == codeCardSer && codesCardsSave[p][1] == codeCardNom)  //їаЮзШвРЭР ЯаРТШЫмЭРп ЪРавРђ
	    	  {
	    		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  //ІЪЫозХЭШХ ЧХЫХЭЮУЮ бТХвЮФШЮФР”
	    		  HAL_Delay(500);
	    		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	    		  sprintf(str,"page 1\xff\xff\xff");                     //їХаХеЮФ ЭР 1 бваРЭШжг
	    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
	    		  flagRightCard = 1;
				  sprintf(str, "їаЮзШвРЭР ЪРавР ШЧ бЯШбЪР");
				  CDC_Transmit_FS(str, strlen(str));

	    	  }
	      }
	      if(flagRightCard == 0)  //їаЮзШвРЭР ЭХЯаРТШЫмЭРп ЪРавРђ
	      {
    		  sprintf(str,"page 0\xff\xff\xff");                   //їХаХеЮФ ЭР ЭгЫХТго бваРЭШжг
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
    		  sprintf(str,"t1.txt=\"%d %d\"\xff\xff\xff", codeCardSer, codeCardNom);   //ѕвЮСаРЦХЭШХ ЭЮЬХаР ЪРавл Т вХЪбвЮТЮЬ ЯЮЫХ
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
    		  sprintf(str,"t1.pco=63488\xff\xff\xff");                 //БЬХЭР жТХвР вХЪбвР ЭР ЪаРбЭлЩ
    		  HAL_UART_Transmit(&huart1, str, strlen(str), 10000);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  //ІЪЫозХЭШХ ЪаРбЭЮУЮ бТХвЮФШЮФР”
	    	  HAL_Delay(500);
	    	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			  sprintf(str, "їаЮзШвРЭР ЪРавР ЭХ ШЧ бЯШбЪР");
			  CDC_Transmit_FS(str, strlen(str));
	      }
	      flagRightCard = 0;
		  memset(codeCardHEXtext, 0, 12);
		  }
		  CardFlag = '0';
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3600;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if(huart == &huart1) {
	  if (nexBufLen < NEX_BUFFER_SIZE){
		  nexBuf[nexBufLen++] = nexByte;
		  if (nexByte == 0xff)
		  {
			  ffCount++;
		  } else
		  {
			  ffCount = 0;
		  }
		  if (ffCount == 3){   //їЮЫгзХЭЮ бЮЮСйХЭШХ Юв ФШбЯЫХп
			  	  ffCount = 0;
			  	  nexBufLen = 0;
			  	  if(nexBuf[0]==0x66 && nexBuf[1]==0x00){
			  		pagenom = 0;
			  	  }
			  	  if(nexBuf[0]==0x66 && nexBuf[1]==0x03){
			  		pagenom = 3;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x02){    //ЅРЦРвР ЪЭЮЯЪР ФЫп иЫРУСРгЬР
			  		if(shlag == 0)
			  		{
			  			shlag = 1;
			  			delay_ms(50000);
			  		} else if(shlag == 1)
			  		{
			  			shlag = 0;
			  		}
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x0E){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР ФЮСРТЫХЭШп ЯЮЫмЧЮТРвХЫп");
					  CDC_Transmit_FS(str, strlen(str));
			  		  plusUs = 1;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x08){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 0");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 0;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x09){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 1");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 1;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x0a){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 2");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 2;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x0b){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 3");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 3;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x0c){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 4");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 4;
			  	  }
			  	  if(nexBuf[0]==0x65 && nexBuf[1]==0x03 && nexBuf[2]==0x0d){
					  sprintf(str, "ЅРЦРвР ЪЭЮЯЪР гФРЫХЭШп ЯЮЫмЧЮТРвХЫп 5");
					  CDC_Transmit_FS(str, strlen(str));
			  		  nomminusUs = 5;
			  	  }
		  }
	  }

	  HAL_UART_Receive_IT(&huart1, (uint8_t*)&nexByte, 1);    //їЮЫгзШвм бЫХФгойШЩ СРЩв
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //ѕСаРСЮвзШЪ ЯаХалТРЭШЩ Юв GPIO
{
	if(GPIO_Pin == GPIO_PIN_10) {    //їаШиХЫ 0
		if(i == 0){
			chet = 0;
		} else if(i == 25){
			chet = 0;
		}
	} else if (GPIO_Pin == GPIO_PIN_11) {   //їаШиЫР 1
		if(i == 0){
			chet = 1;
		} else if(i == 25){
			chet = 1;
		} else if(i < 9) {
			chet1++;
		uint8_t kk = 7 - (i-1);
		uint8_t jj = (1<<kk);
		codeSer = codeSer | jj;
		} else{
			chet1++;
		uint8_t kk = 15 - (i-9);
		uint16_t jj = (1<<kk);
		codeNom = codeNom | jj;
		}
	}
if(i == 12){                            //їаЮТХаЪР ЭР зХвЭЮбвм
	if(chet1%2 == 0 && chet == 1){
		//ѕиШСЪРђ
	}
	if(chet1%2 == 1 && chet == 0){
		//ѕиШСЪРђ
	}
	chet1 = 0;
}
if(i == 25){
	if(chet1%2 == 0 && chet == 1){
		//ѕиШСЪРђ
	}
	if(chet1%2 == 1 && chet == 0){
		//ѕиШСЪРђ
	}
	chet1 = 0;
}
	i++;
	if (i > 25)      //їаШиХЫ ТХбм ЪЮФ
	{
		i = 0;
		codeCardSer = codeSer;
		codeSer = 0;
		codeCardNom = codeNom;
		codeNom = 0;
		CardFlag = '1';

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
