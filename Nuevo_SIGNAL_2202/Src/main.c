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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	int Derecha;
	int Izquierda;
	int Vel;
}Accion;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Nbanderas 10

void ModoManual( int *EtapasManual, int Flags[],Accion*accion);
void ModoAutomatico( int *EtapasAutomatico, int Flags[],Accion*accion, int valor);
void GMM(int*GMM_Etapas, int*EtapasManual,int*EtapasAutomatico, int Flags[], int MaxBanderas, uint32_t* T_Alarm);
void Marcha(int GMM_Etapas, Accion Automatico, Accion Manual);
//void Marcha(int EtapasManual, int EtapasAutomático
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

//ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
void MX_USB_HOST_Process(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t EntradaAnalogica;

int EtapasAuto=0;
int EtapasMan=0;
int EtapasGMM=0;
int Bandera[10]; 
Accion ManualAct,AutomaticAct={0,0,150};
uint32_t Alarma;

/*
Banderas: 
0-> Cambio Auto/Man
1-> Pulsador 1
2-> Pulsador 2
3-> Pulsador 3
4-> Pulsador 4 
5-> Pulsador 5
*/

//////// CALLBACKS ////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin==GPIO_PIN_1){
		//selector auto/manual-> PA1

		Bandera[0]=1; //Hay cambio auto/man
				
	}else if(GPIO_Pin==GPIO_PIN_2){
		//selector Pulsador1-> PA2
		Bandera[1]=1;
		
	}else if(GPIO_Pin==GPIO_PIN_3){
		//selector Pulsador2-> PC3
		Bandera[2]=1;
		
	}else if(GPIO_Pin==GPIO_PIN_4){
		//selector Pulsador3-> PC4
		Bandera[3]=1;
		
	}else if(GPIO_Pin==GPIO_PIN_5){
		//selector Pulsador4-> PC5
		Bandera[4]=1;
		
	}else if(GPIO_Pin==GPIO_PIN_6){
		//selector Pulsador5-> PE6
		Bandera[5]=1;
	}else{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==ADC1){
		EntradaAnalogica= HAL_ADC_GetValue(&hadc1);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

////////    MAIN    ////////////
int main(void)
{
 
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
	
  /* USER CODE BEGIN 2 */
	
	/* Inicializamos la lectura de la entrada analógica */
	HAL_ADC_Start_IT(&hadc1);
	/* Inicializamos y damos el valor por defecto a la salida PWM */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,159);

	/*Establecemos la Etapa del GMM actual en función de la posición del selector de dos posiciones */
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)){ 
		// MODO AUTOMATICO
			EtapasGMM=1;
	}else{
		// MODO MANUAL
			EtapasGMM=0;
	}
	
  /* USER CODE END 2 */
  
///////      WHILE(1)       ////////
 
  while (1)
  {
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	
	// Llamada a la función encargada de la gestión de Modos de Marcha
	GMM(&EtapasGMM,&EtapasMan,&EtapasAuto,Bandera,Nbanderas,&Alarma);
  
	//Llamada a las funciones de gestión de modo Automático y Manual en función del GMM
	if(EtapasGMM==1){
		ModoAutomatico(&EtapasAuto,Bandera,&AutomaticAct,EntradaAnalogica);

	}else if(EtapasGMM==0){
		ModoManual(&EtapasMan, Bandera, &ManualAct);
	}
	
	//Llamada a la Funcion encargada de mover el motor
	Marcha(EtapasGMM,AutomaticAct,ManualAct);
			
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_10|LD4_Pin|LD3_Pin 
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD10 LD4_Pin LD3_Pin 
                           LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|LD4_Pin|LD3_Pin 
                          |LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}
/////////////////////////    FUNCIONES   //////////////////////////////////

/* USER CODE BEGIN 4 */

///////    GMMM      /////////

void GMM(int*GMM_Etapas, int*EtapasManual,int*EtapasAutomatico, int Band[], int MaxBanderas, uint32_t* T_Alarm){

if (Band[0]==1){
		HAL_Delay(120);    //Antirrebotes
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
			//ACTIVACION MODO AUTOATICO Y LECTURA DE ENTRADA ANALOGICA
			*GMM_Etapas=1;
			HAL_ADC_Start_IT(&hadc1);
		}else{
			//ACTIVACION MODO MANUAL Y DESACTIVACION LECTURA ANALOGICA
			*GMM_Etapas=0;
			HAL_ADC_Stop_IT(&hadc1);
		}
		/*Cuando hay cambio de modo se reinicializan todas las banderas
		y se ponen a cero todas las etapas*/
		for(int i=0;i < MaxBanderas; i++){
				Band[i]=0;
		}
		*EtapasAutomatico=0;
		*EtapasManual=0;
  }
		//Activación MODO ALARMA en caso de que el carro no pueda llegar a su destino
			if((*EtapasManual==0&&*GMM_Etapas==0)||(*EtapasAutomatico==0&&*GMM_Etapas==1)){
			//Siempre que la estamos fuera de la etapa 0 en alguno de los dos modos, estas realizando una accion
			*T_Alarm=HAL_GetTick();
		}
			//Si no volvemos a la etapa 0 en menos de x tiempo, se activa el estado de alarma
		if((HAL_GetTick()- *T_Alarm)>4000){
			
			*EtapasAutomatico=0;
			*EtapasManual=0;
			*GMM_Etapas=3;
		}

//ILUMINACIÖN LEDS
	if (*GMM_Etapas==1){
		//En Auto encendemos luz Verde
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET);

	}else if(*GMM_Etapas==0){
		//En Manual encendemos luz Azul
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else{
		//En alarma Parpadeamos luz Roja
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		HAL_Delay(50);
	}

}

void ModoAutomatico( int *EtapasAutomatico, int Flags[],Accion*accion, int valor){
if(*EtapasAutomatico==0){
		if(valor <75&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)){
			*EtapasAutomatico = 1;
		}else if(valor>95&&valor<175&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)){
				HAL_Delay(500);
			if(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)){
			*EtapasAutomatico=5;
			}
		}else if(valor>195&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)){
			*EtapasAutomatico=10;
		}
		if(Flags[4]==1){
		accion->Vel=accion->Vel-15;
			HAL_Delay(100);
			Flags[4]=0;
		}else if(Flags[5]==1){
		accion->Vel=accion->Vel+15;
			HAL_Delay(100);
			Flags[4]=0;
		}
//X1
	}else if(*EtapasAutomatico==1){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)){
			*EtapasAutomatico=0;
			}	
//X5
	}else if(*EtapasAutomatico==5){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
				HAL_Delay(20);  //evitar fluctuaciones de la señal->provocaria etapa fugaz
				*EtapasAutomatico=6;
		}else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
				*EtapasAutomatico=8;
		}
//X6
	}else if(*EtapasAutomatico==6){
		 if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
					*EtapasAutomatico=8;
	 }
//X8
	}else if(*EtapasAutomatico==8){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
					*EtapasAutomatico=0;
		}
	}else if(*EtapasAutomatico==10){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
						*EtapasAutomatico=0;
		}
	}
	if(*EtapasAutomatico==1||*EtapasAutomatico==8){
			accion->Izquierda=1;
		  accion->Derecha=0;
			accion->Vel=140;
	}
	/*else if(*EtapasAutomatico==0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)&&!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)){
			accion->Izquierda=1;
		  accion->Derecha=0;
			accion->Vel=99;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		}*/
	else if(*EtapasAutomatico==5||*EtapasAutomatico==6||*EtapasAutomatico==10){
			accion->Izquierda=0;
		  accion->Derecha=1;
			accion->Vel=140;		
		}
	/*else if(*EtapasAutomatico==0 && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)&&HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)){
			accion->Izquierda=0;
		  accion->Derecha=1;
			accion->Vel=99;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
		}*/
		else{
			accion->Izquierda=0;
		  accion->Derecha=0;
		
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	
	return;

}


void ModoManual( int *EtapasManual, int Flags[], Accion* accion){

//X0
	if(*EtapasManual==0){
		if(Flags[1] == 1){
			*EtapasManual = 1;
			HAL_Delay(120);
			Flags[1]=0;
		}else if(Flags[2] == 1){
			*EtapasManual=5;
			HAL_Delay(200);
			Flags[2]=0;
		}else if(Flags[3] == 1){
			*EtapasManual=10;
			HAL_Delay(120);
			Flags[3]=0;
		}
//X1
	}else if(*EtapasManual==1){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)){
			*EtapasManual=0;
			}	
//X5
	}else if(*EtapasManual==5){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
				HAL_Delay(20);  //evitar fluctuaciones de la señal->provocaria etapa fugaz
				*EtapasManual=6;
		}else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
				*EtapasManual=8;
		}
//X6
	}else if(*EtapasManual==6){
		 if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
					*EtapasManual=8;
	 }
//X8
	}else if(*EtapasManual==8){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
					*EtapasManual=0;
		}
	}else if(*EtapasManual==10){
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
						*EtapasManual=0;
		}
	}
	if(*EtapasManual==1||*EtapasManual==8){
			accion->Izquierda=1;
		  accion->Derecha=0;
			accion->Vel=140;
	}else if(*EtapasManual==0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)&&!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)){
			accion->Izquierda=1;
		  accion->Derecha=0;
			accion->Vel=99;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		}else if(*EtapasManual==5||*EtapasManual==6||*EtapasManual==10){
			accion->Izquierda=0;
		  accion->Derecha=1;
			accion->Vel=140;		
		}else if(*EtapasManual==0 && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5)&&HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)&&!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)){
			accion->Izquierda=0;
		  accion->Derecha=1;
			accion->Vel=99;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET);
		}else{
			accion->Izquierda=0;
		  accion->Derecha=0;
		
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	
	return;
}

void Marcha(int GMM_Etapas, Accion Automatico, Accion Manual){
	if(GMM_Etapas==0){ //MANUAL
		if(Manual.Izquierda){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Manual.Vel);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			}else if(Manual.Derecha){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Manual.Vel);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			}
	}else if(GMM_Etapas==1){ //AUTOMATICO
		if(Automatico.Izquierda){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Automatico.Vel);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			}else if(Automatico.Derecha){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Automatico.Vel);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
			}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
			}
	}else{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
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
