/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define	Versiyon 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define GsmModulStart HAL_Delay(2000);HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_SET);HAL_Delay(800);HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_RESET);HAL_Delay(1000)

#define IPR "AT+IPR=115200"
#define CIPCLOSE "AT+CIPCLOSE\r\n"
#define CIPMODE "AT+CIPMODE=0\r\n"
#define CIPMUX "AT+CIPMUX=0\r\n"
#define CSTT "AT+CSTT="%s", "", """
#define CIICR "AT+CIICR\r\n"
#define CIPQSEND "AT+CIPQSEND=1\r\n"
#define ATE0  "ATE0\r\n"
#define AT  "AT\r\n"
#define APNSet "AT+CSTT="soracom.io"\r\n"
#define UPCont  "AT+CIICR\r\n"
#define CIFSR "AT+CIFSR\r\n" // ip sor

#define Led1Set			I2C_IO(1, 1)
#define Led1Reset		I2C_IO(1, 0)
#define Led2Set			I2C_IO(2, 1)
#define Led2Reset		I2C_IO(2, 0)
#define Led3Set			I2C_IO(3, 1)
#define Led3Reset		I2C_IO(3, 0)
#define Led4Set			I2C_IO(4, 1)
#define Led4Reset		I2C_IO(4, 0) 

#define bin(a) ((( (a/10000000*128) + \
(((a/1000000)&1)*64) + \
(((a/100000)&1)*32) + \
(((a/10000)&1)*16) + \
(((a/1000)&1)*8) + \
(((a/100)&1)*4) + \
(((a/10)&1)*2) + \
(a&1)) * (a/10000000)) + \
(( ((a/262144)*64) + \
(((a/32768)&1)*32) + \
(((a/4096)&1)*16) + \
(((a/512)&1)*8) + \
(((a/64)&1)*4) + \
(((a/8)&1)*2) + \
(a&1)) * (1-(a/10000000))))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void Connect_Tcp_port(char *Ip ,char *Port);
float readTemp() ;
int MS_Delay(int Countt);
void TimerConutMs(void);
void TimerConutMs10(void);
void TimerConutMs100(void);
void TimerConutLoopSn(void);
void TimerConutLoop10Ms(void);
void TimerConutLoop100Ms(void);


void I2C_IO(uint8_t IO, uint8_t HighLow);
uint8_t DataComparator(uint8_t FirstByte  , char Array1[] , char Array2[] );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_InitTypeDef GPIO_InitStruct = {0};

uint8_t		RecordData[100] __attribute__((at(0x20001000))); // datayi kayidet 

uint8_t PinDegistir=0;
uint8_t PinDegistir_t=0;
uint8_t LEDDurum=0;
uint8_t ButonDurum=0;

uint8_t CountUs=0;
uint16_t CountMs=0;
uint8_t Count10Ms=0;
uint8_t Count100Ms=0;
uint16_t EncoderCount=0;

uint8_t LoopSnFlag=0;
uint8_t Loop10MsFlag=0;
uint8_t Loop100MsFlag=0;

uint8_t LoopWatch=0;
float TimeMs=0;
uint32_t TimerCount=0;
#define AdcReadPin 4
uint32_t	ADCValue[AdcReadPin];
float AnalogValue1=0,AnalogValue=0,TempSensVoltmv=0,Volt=0;
uint8_t GsmErrorCount=0;
	
	
int			Humidity;
int			TempKart;
uint8_t 	status = 0;
uint8_t 	measure_RH_command = 0xE5;
uint8_t 	measure_Temp_command = 0xE0;
uint8_t 	measured_RH[5] = {0,0,0,0,0};
uint8_t 	measured_Temp[5] = {0,0,0,0,0};

char Dizi1[]={ 'a','z','1','g','h','j','k','w','t','y','u','o'};
	
char Dizi2[]={ 'a','z','1','g','h','j','k','w','t','y','u','o'};
	
uint16_t CycleCout = 0;
	
	
uint32_t Deger =0;
	
	
uint8_t  GsmGelenByte=0;
uint8_t  GsmGelenByteAry[100];
uint8_t GsmByteGonder[20]={'1','2','3','A','B','a','b','2','2','2','2','2','2','2','2','2','2','2','2'};
char TcpIp[40];
	char Msjlar[30];
	char tx_buffer[30];
		char Data_buffer[100];

	char APN[20];
	
uint8_t GsmByteGonderIndex=1;
uint8_t			GsmGelenByteStart=0;
uint8_t			GsmGelenByteKaydet=0;
uint8_t			GsmGelenByteIndex=0;
uint8_t   	GsmGelenByteStop=0;
uint8_t 	  GsmGelenByteKayitta=0;
uint8_t     GsmTcpStart=0;
uint8_t	Dongudencik=0;
uint8_t PortKApat=0;
uint32_t Count1 =450;
uint8_t	SicaklikSor=0;
uint16_t length=0;
uint8_t sistemKapat=0;
uint8_t 	YanitGeldi=0;
float sicaklikDerce=0;
uint32_t systemCont=0;
float sicaklikInt=0;
float sicaklikFloat=0;
uint8_t I2cOutPin=0;

uint8_t I2cOut=0;
uint8_t DataDogrumu=2;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

//		HAL_Delay(3000);
//		HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_SET);
//		HAL_Delay(850);
//		HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_RESET);
//		HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
//	GsmModulStart;
		HAL_UART_Receive_IT(&huart1, (uint8_t *) &GsmGelenByte, 1);

//HAL_Delay(5000);
//HAL_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)APN,strlen(APN));HAL_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)IPR,strlen(IPR));HAL_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)CIPCLOSE,strlen(CIPCLOSE));HAL_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)CIPMODE,strlen(CIPMODE));HAL_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)CIPMUX,strlen(CIPMUX));HAL_Delay(2000);

//  	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPMODE=0\r\n",strlen("AT+CIPMODE=0\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPMUX=0\r\n",strlen("AT+CIPMUX=0\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CSTT=\"api.thingspeak.com/update?api_key=KIJWHEAZY1FKUQPS&field1=40\",\"\",\"\"\r\n",strlen("AT+CSTT=\"api.thingspeak.com/update?api_key=KIJWHEAZY1FKUQPS&field1=40\",\"\",\"\"\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CSTT=\"internet\",\"\",\"\"\r\n",strlen("AT+CSTT=\"internet\",\"\",\"\"\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIICR\r\n",strlen("AT+CIICR\r\n"));
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIFSR\r\n",strlen("AT+CIFSR\r\n"));
//		memset(GsmGelenByteAry,0,sizeof(GsmGelenByteAry));
//		HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n",Ip,Port)
//	 	HAL_Delay(1000);HAL_UART_Transmit_IT(&huart1,(uint8_t *)TcpIp,strlen(TcpIp));
// Connect_Tcp_port("78.179.100.206","7272");


//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)CIICR,strlen(CIICR));HAL_Delay(500);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)CIFSR,strlen(CIFSR));HAL_Delay(500);
//   Connect_Tcp_port("95.8.172.55","7272");

	__HAL_TIM_CLEAR_IT(&htim17, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim17);	//100us  85.103.18.91
	 HAL_TIM_Base_Start(&htim3);
	 
	while(1)
	{
		if(Loop10MsFlag==1)
		{
		Loop10MsFlag=0;
		TimerConutLoop10Ms();
		}
		if(Loop100MsFlag==1)
		{
		Loop100MsFlag=0;
    TimerConutLoop100Ms();
		}
		if(LoopSnFlag==1)
		{
		LoopSnFlag=0;
		TimerConutLoopSn();
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 18;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 479;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RxLed_Pin|TxLed_Pin|GsmStart_Pin|Led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RxLed_Pin TxLed_Pin GsmStart_Pin Led2_Pin */
  GPIO_InitStruct.Pin = RxLed_Pin|TxLed_Pin|GsmStart_Pin|Led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)	//GsmSim800
	{
		if(GsmGelenByteIndex==98)
		{
			GsmGelenByteIndex=0;
		}
		if(GsmGelenByteIndex<99)
		{
			GsmGelenByteKayitta=1;
			GsmGelenByteStart=0;
			GsmGelenByteAry[GsmGelenByteIndex] =	GsmGelenByte;
				if((GsmGelenByte == 10 || GsmGelenByte == '!') &&(GsmGelenByteAry[GsmGelenByteIndex-1] == 13 || GsmGelenByteAry[GsmGelenByteIndex-1] == '*') )
				{
					GsmGelenByteIndex=0;
				}
				GsmGelenByteIndex++;
		}
		HAL_GPIO_TogglePin(GPIOA, RxLed_Pin);
		HAL_UART_Receive_IT(&huart1, (uint8_t *) &GsmGelenByte, 1);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_CLEAR_IT(&htim17, TIM_IT_UPDATE);//10us
	CountUs++;
	if(CountUs>=100)
	{//1ms
		CountUs=0;
		TimerConutMs();
		CountMs++;
		if(CountMs>=1000)
		{ //1sn
		 LoopSnFlag=1;
			HAL_GPIO_TogglePin(Led2_GPIO_Port,Led2_Pin);
			CountMs=0;
		}
		Count10Ms++;
		if(Count10Ms>=10)
		{//10ms
			Loop10MsFlag=1;
			Count10Ms=0;
			TimerConutMs10();
		}
		Count100Ms++;
		if(Count100Ms>=100)
		{//100ms
			Loop100MsFlag=1;
			Count100Ms=0;
			TimerConutMs100();
		}
		
	}
}


void TimerConutMs(void)
{

//		if(DataComparator(1,(char *)GsmGelenByteAry,"ERROR")==1)
//		{
//     GsmErrorCount++;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out1-Open")==1)
//		{
//			Led1Set;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out2-Open")==1)
//		{
//			Led2Set;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out3-Open")==1)
//		{
//			Led3Set;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out4-Open")==1)
//		{
//			Led4Set;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out1-Close")==1)
//		{
//			Led1Reset;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out2-Close")==1)
//		{
//			Led2Reset;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out3-Close")==1)
//		{
//			Led3Reset;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Out4-Close")==1)
//		{
//			Led4Reset;
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Humidity")==1)
//		{
//      
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Temperature1")==1)
//		{
//    
// 		}
//		if(DataComparator(3,(char *)GsmGelenByteAry,"Temperature2")==1)
//		{
//    
// 		}
//		
}
void TimerConutMs10(void)
{
	
////////----------------------PinCongif-------------------------/////////////
	if(PinDegistir!=PinDegistir_t)	
	{
		PinDegistir_t=PinDegistir;				
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		if(LEDDurum==1)HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		if(LEDDurum==0)HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	else
	{
		PinDegistir_t=PinDegistir+1;
		if(PinDegistir_t>200)PinDegistir_t=0;
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)ButonDurum=1;
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1)ButonDurum=0;
	}
/////////-----------------------------------------------/////////////
}
void TimerConutMs100(void)
{
	
}
void TimerConutLoop10Ms(void)
{
	if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='1'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='n')
		{		
			HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_SET);
		  YanitGeldi=1;
		}
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='1'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='f'&&GsmGelenByteAry[8]=='f')
		{
			HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_RESET);
			YanitGeldi=1;
		}
		
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='2'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='n')
		{		
			HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_SET);YanitGeldi=1;
		
		}
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='2'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='f'&&GsmGelenByteAry[8]=='f')
		{
			HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_RESET);YanitGeldi=1;
		}
		
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='3'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='n')
		{		
//			HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,GPIO_PIN_SET);YanitGeldi=1;
		
		}
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='3'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='f'&&GsmGelenByteAry[8]=='f')
		{
//			HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,GPIO_PIN_RESET);YanitGeldi=1;
		}
			if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='4'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='n')
		{		
//			HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,GPIO_PIN_SET);YanitGeldi=1;
		
		}
		if(GsmGelenByteAry[1]=='l'&&GsmGelenByteAry[2]=='e'&&GsmGelenByteAry[3]=='d'&&GsmGelenByteAry[4]=='4'&&GsmGelenByteAry[5]==' '&&GsmGelenByteAry[6]=='o'&&GsmGelenByteAry[7]=='f'&&GsmGelenByteAry[8]=='f')
		{
//			HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,GPIO_PIN_RESET);YanitGeldi=1;
		}
		
	 if(GsmGelenByteAry[1]=='k'&&GsmGelenByteAry[2]=='a'&&GsmGelenByteAry[3]=='c'&&GsmGelenByteAry[4]==' '&&GsmGelenByteAry[5]=='d'&&GsmGelenByteAry[6]=='e'&&GsmGelenByteAry[7]=='r'&&GsmGelenByteAry[8]=='e'&&GsmGelenByteAry[9]=='c'&&GsmGelenByteAry[10]=='e')
	 {
	  SicaklikSor=1;
	 
	 }
	  if(GsmGelenByteAry[1]=='k'&&GsmGelenByteAry[2]=='a'&&GsmGelenByteAry[3]=='p'&&GsmGelenByteAry[4]=='a'&&GsmGelenByteAry[5]=='t')
		{
		  sistemKapat=1;
			SicaklikSor=2;
		}
		
		  if(GsmGelenByteAry[1]=='i'&&GsmGelenByteAry[2]=='s'&&GsmGelenByteAry[3]=='l'&&GsmGelenByteAry[4]=='e'&&GsmGelenByteAry[5]=='m'&&GsmGelenByteAry[22]=='.')
		{
			SicaklikSor=1;
			HAL_Delay(300);
			sicaklikFloat=(GsmGelenByteAry[23]-0x30)*100+(GsmGelenByteAry[24]-0x30)*10+(GsmGelenByteAry[25]-0x30);
			sicaklikInt=(((GsmGelenByteAry[20]-0x30)*10)+((GsmGelenByteAry[21]-0x30)*1));
			sicaklikFloat=sicaklikInt+sicaklikFloat/1000.f;
			
		}
		

}
void TimerConutLoop100Ms(void)
{
	
	if(ButonDurum==1)
	{
		HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_SET);
	}
	if(ButonDurum==0)
	{
		HAL_GPIO_WritePin(TxLed_GPIO_Port,TxLed_Pin,GPIO_PIN_RESET);
	}
	
	
		if((GsmTcpStart==0 )&&(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0 ||GsmByteGonderIndex > 0||SicaklikSor==1||YanitGeldi==1||(	GsmGelenByteAry[1]=='s'&&GsmGelenByteAry[2]=='i'&&GsmGelenByteAry[3]=='c'&&GsmGelenByteAry[4]=='a'&&GsmGelenByteAry[5]=='k'&&GsmGelenByteAry[6]=='l'&&GsmGelenByteAry[7]=='i'&&GsmGelenByteAry[8]=='k')) )
		{
		
			
		  if(SicaklikSor==1 && YanitGeldi==0 )
				{
					length= sprintf(Data_buffer,"Sicaklik  %d C' \n" , (uint8_t)sicaklikDerce);
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length));
					HAL_Delay(200);
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)Data_buffer, strlen(Data_buffer));
					HAL_Delay(200);
				}
			if(SicaklikSor==0 && YanitGeldi==0 )
				{
					length = sprintf(Data_buffer,"islemci sicakligi\n");
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length));
					HAL_Delay(200);
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)Data_buffer, strlen(Data_buffer));
					HAL_Delay(200);
				}
				if(YanitGeldi==1 )
				{
					
					length = sprintf(Data_buffer," Uygulandi \n");
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length));
					HAL_Delay(200);
					HAL_UART_Transmit_IT(&huart1,(uint8_t *)Data_buffer, strlen(Data_buffer));
					HAL_Delay(200);
					YanitGeldi=0;
				}
			GsmByteGonderIndex = 0;
			if(sistemKapat==1){HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));	MS_Delay(500);}
			if( YanitGeldi==0)	SicaklikSor=0;
			if( YanitGeldi==0)sistemKapat=0;
      
		}
  sicaklikDerce  = (uint8_t)readTemp() ;
}
void TimerConutLoopSn(void)
{
	EncoderCount = __HAL_TIM_GetCounter(&htim3); 
	
	if(LoopWatch!=LEDDurum)
	{
		LoopWatch=LEDDurum;
		LEDDurum=1;
		I2C_IO(I2cOut, 1);//Led
	}
	else
	{
		LEDDurum=0;
		I2C_IO(I2cOut, 0);//Led

	}
	LoopWatch++;	
}
int MS_Delay(int Countt)
{
	uint32_t CountT =(Countt*10000);
while(CountT--);

}

uint8_t DataComparator(uint8_t FirstByte , char Array1[] , char Array2[] )
{
	uint8_t DataDogru=0;
	uint8_t DataYalis=0;
 if(sizeof(Array2)-1==0 || FirstByte>200 || sizeof(Array2)-1>200)
 {
  DataYalis=1;
 }

 for(uint8_t x=0 ;x < sizeof(Array2)-1 ; x++)
 {

//   dizim[10+x]=(uint8_t)Array1[FirstByte+x];	
  if((uint8_t)Array1[FirstByte+x] == Array2[x] )
	{
	 
	  DataDogru++;
	}
	else
	{
//	dizim[x]=Array2[x];	
		 DataYalis++;
	}
 }
//  dizim[22]=sizeof(Array2);
//  dizim[21]=DataYalis;
 if(DataYalis > 0) return 0;
 if(DataYalis == 0) return 1;
 if(DataDogru == 0) return 0;
 return 0;
}

void Connect_Tcp_port(char *Ip ,char *Port)
{ 
	
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));			// Close TCP or UDP connection. (acik bir tcp varsa kapatir )
		HAL_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CFUN=1\r\n",strlen("AT+CFUN=1\r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPSHUT\r\n",strlen("AT+CIPSHUT\r\n")); 
		MS_Delay(1000);	
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CGATT=0\r\n",strlen("AT+CGATT=0 \r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"\"IP\",airtelgprs.com\"\r\n",strlen("'IP\",airtelgprs.com\"\r\n")); 
		MS_Delay(100);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPMODE=0\r\n",strlen("AT+CIPMODE=0\r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CGACT=1,1\r\n",strlen("AT+CGACT=1,1\r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CSTT=\"airtelgprs.com\"\r\n",strlen("AT+CSTT=\"airtelgprs.com\"\r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIICR\r\n",strlen("AT+CIICR\r\n")); 
		MS_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIFSR\r\n",strlen("AT+CIFSR\r\n")); 
		MS_Delay(2000);
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+IPR=115200\r\n",strlen("AT+IPR=115200\r\n")); // set sim800 uart baudrate to 9600 (baudrate 9600 olarak ayarlar )
//	MS_Delay(100);
//	 /// DAOKDA
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));			// Close TCP or UDP connection. (acik bir tcp varsa kapatir )
//	HAL_Delay(100);
//	  ///DACLOSE OKDA  
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPMODE=0\r\n",strlen("AT+CIPMODE=0\r\n"));		// TCP Application Mode : 0->Normal Mode ,1->Transparent Mode ( tcp calisma modunu ayarlanir)
//	MS_Delay(100);
//	  //DA+CME ERROR: 3DA
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPMUX=0\r\n",strlen("AT+CIPMUX=0\r\n"));			// Single IP connection (ip adresine baglati olacagini ayarlar)
//	MS_Delay(100);
//	 //DA+CME ERROR: 3DA
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CSTT=\"'airtelgprs.com\",\"\",\"\"\r\n",strlen("AT+CSTT=\"internet\",\"\",\"\"\r\n"));	// APN settings (simkartini apn ayarlarini internet olarak yapapar)
//	MS_Delay(100);
//	 //DA+CME ERROR: 3DA
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIICR\r\n",strlen("AT+CIICR\r\n")); // Bring Up Wireless Connection with GPRS or CSD ( kablosuz baglanti istegi)
//	MS_Delay(1000);
//	 
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIFSR\r\n",strlen("AT+CIFSR\r\n")); // Get Local IP Address ( sim800 kendi global ip 'si)
//	MS_Delay(100);
//	
//	
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n",Ip,Port)); // Start up TCP connection(tpc baslat ip adresi ve port)
	GsmTcpStart=1;
	HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_RESET);
	
	Count1 = (Count1*100000);
	PortKApat=1;
	while(GsmTcpStart && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==1 )
	{
		Count1--;
	if(Count1==0)NVIC_SystemReset();
	HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_SET);
	if(GsmGelenByteAry[1]=='C'&&GsmGelenByteAry[2]=='O'&&GsmGelenByteAry[3]=='N'&&GsmGelenByteAry[4]=='N'&&GsmGelenByteAry[5]=='E'&&GsmGelenByteAry[6]=='C'&&GsmGelenByteAry[7]=='T'&&GsmGelenByteAry[8]==' '&&GsmGelenByteAry[9]=='O'&&GsmGelenByteAry[10]=='K')
	{
	 GsmTcpStart=0;
//			HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,GPIO_PIN_SET);
	}
	if(GsmGelenByteAry[1]=='A'&&GsmGelenByteAry[2]=='L'&&GsmGelenByteAry[3]=='R'&&GsmGelenByteAry[4]=='E'&&GsmGelenByteAry[5]=='A'&&GsmGelenByteAry[6]=='D'&&GsmGelenByteAry[7]=='Y'&&GsmGelenByteAry[8]==' '&&GsmGelenByteAry[9]=='C'&&GsmGelenByteAry[10]=='O'&&GsmGelenByteAry[11]=='N'&&GsmGelenByteAry[12]=='N'&&GsmGelenByteAry[13]=='E'&&GsmGelenByteAry[14]=='C'&&GsmGelenByteAry[15]=='T')
	{
		 GsmTcpStart=0;
//			HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,GPIO_PIN_SET);

	}
	if(GsmGelenByteAry[1]=='E'&&GsmGelenByteAry[2]=='R'&&GsmGelenByteAry[3]=='R'&&GsmGelenByteAry[4]=='O'&&GsmGelenByteAry[5]=='R')
	{
//		if(PortKApat==1)
//			{
//			HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));	
//				MS_Delay(500);
//			PortKApat=0;
//			}
//		HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_RESET);
		if(Count1<3500000)HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_SET);

	}
	
		if(GsmGelenByteAry[1]=='C'&&GsmGelenByteAry[2]=='O'&&GsmGelenByteAry[3]=='N'&&GsmGelenByteAry[4]=='N'&&GsmGelenByteAry[5]=='E'&&GsmGelenByteAry[6]=='C'&&GsmGelenByteAry[7]=='T'&&GsmGelenByteAry[8]==' '&&GsmGelenByteAry[9]=='F'&&GsmGelenByteAry[10]=='A'&&GsmGelenByteAry[11]=='I'&&GsmGelenByteAry[12]=='L')
	{
		if(PortKApat==1)
				{
					
				HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"));	
					MS_Delay(500);
						HAL_UART_Transmit_IT(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n",Ip,"123")); // Start up TCP connection(tpc baslat ip adresi ve port)
         MS_Delay(500);
				PortKApat=0;
				}
//		HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,GPIO_PIN_SET);
		if(Count1<3500000)	HAL_GPIO_WritePin(GsmStart_GPIO_Port,GsmStart_Pin,GPIO_PIN_SET);
    

	}
	}
		HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,GPIO_PIN_RESET);
	MS_Delay(1000);
  
	 // DAOKDACONNET OKDA 
	              
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"AT+CIPQSEND=1\r\n",strlen("AT+CIPQSEND=1\r\n")); // Select Data Transmitting Mode. 0->Normal mode ,1->Quick send mode (AT+CIPQSEND=1 giden datayi acar )
//	MS_Delay(100);

//	 
//	HAL_UART_Transmit_IT(&huart1,(uint8_t *)"ATE0\r\n",strlen("ATE0\r\n"));									// Set Command Echo Mode. 0->Echo mode off,1->Echo mode on (komut konfigure)
//	MS_Delay(100);
}





uint8_t eepromOku(uint8_t adres)
{
	uint8_t data;
	
	uint8_t status=HAL_I2C_Mem_Read(&hi2c1, 0xA0,adres,I2C_MEMADD_SIZE_8BIT,&data,1,1);
	HAL_Delay(2);
	return(data);
}
void eepromYaz(uint8_t adres, 		uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, 0xA0,adres,I2C_MEMADD_SIZE_8BIT,&data,1,10);
	HAL_Delay(2);

}

float readTemp() {
	float temperature;
	uint8_t temp[2];
	float f_temp[2];

	if(HAL_I2C_Mem_Read(&hi2c1, (0x18 << 1) | 0x01, 0x05, 1, temp, 2, 10) == HAL_OK) {
		temp[0] = temp[0] & 0x1F;
		if((temp[0] & 0x10) == 0x10) {
		  temp[0] = temp[0] & 0x0F;
		  f_temp[0] = temp[0];
		  f_temp[1] = temp[1];
		  temperature = 256 - (f_temp[0] * 16 + f_temp[1] / 16);
		} else {
		  f_temp[0] = temp[0];
		  f_temp[1] = temp[1];
		  temperature = (f_temp[0]* 16 + f_temp[1] / 16);
		}
	} else
	{
	
  	HAL_I2C_Master_Transmit(&hi2c1, 0x80, &measure_RH_command, 1, 0x0F);
		status = HAL_I2C_Master_Receive(&hi2c1, 0x80, &measured_RH[0], 2, 0xFF);
		HAL_I2C_Master_Transmit(&hi2c1, 0x80, &measure_Temp_command, 1, 0x0F);
		status = HAL_I2C_Master_Receive(&hi2c1, 0x80, &measured_Temp[0], 2, 0xFF);

		Humidity = ((measured_RH[0] << 8) | measured_RH[1]) * 0.0019073486328125f - 6;
		TempKart = ((measured_Temp[0] << 8) | measured_Temp[1]) * 0.0026812744140625f - 46.85f;
		
		return TempKart;
	
	  } 

	return temperature;
	
}
void I2C_IO(uint8_t IO, uint8_t HighLow)
{   
	switch(IO)
	{//i2c bagli gpio cogaltici bit aktif etmek icin 
		case 0:
		if(HighLow==1)
		{
			I2cOutPin = 255;
		}
		if(HighLow==0)
		{
			I2cOutPin =0;
		}
		break;

		case 1:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(00000001); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(11111110); 
		}
		break;

		case 2:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(00000010); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(11111101); 
		}
		break;

		case 3:
		if(HighLow==1)
		{    
			I2cOutPin = I2cOutPin | bin(00000100); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(11111011); 
		}
		break;

		case 4:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(00001000); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(11110111); 
		}
		break;

		case 5:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(00010000); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(11101111); 
		}
		break;

		case 6:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(01000000); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(10111111); 
		}
		break;

		case 7:
		if(HighLow==1)
		{
			I2cOutPin = I2cOutPin | bin(10000000); 
		}
		if(HighLow==0)
		{
			I2cOutPin =	I2cOutPin & bin(01111111); 
		}
		break;			
	}
	if( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_BUSY_TX  && HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_BUSY ) 
	{ //I2c hatindan veri tarnferi varsa gonderilecek komutlar bekliyor hat bosalana kadar saglikli bir haberlesme i?in 
		HAL_I2C_Master_Transmit(&hi2c1, 0x40,&I2cOutPin, 1, 50);// kullandigimiz 8bit 0x40 adresli gpio cogaltici
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
