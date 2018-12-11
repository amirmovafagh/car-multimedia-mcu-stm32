
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_PinState acciPinState, rcaiPinState;

	
/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef readAdc;


uint8_t buffer[8]={0x00, 0x00, 0x00, 0x63, 0x04, 0x23, 0x12, 0x15}; 

uint8_t aaa[4];
uint8_t bt[5];                                               //buffer for tea5767



uint8_t senddata[5];
HAL_StatusTypeDef init;


int rx_index=0;

uint8_t rx_data;
uint8_t rx_buffer[32];



uint32_t frq2;
uint16_t soundModuleI2CAddress = 0x88;
uint16_t RadioModuleI2CAddress = 0xC0;


uint8_t mode[3] = {'m','o','d'};
uint8_t audio[3] = {'a','u','d'};
uint8_t radio[3] = {'r','a','d'};
uint8_t bluetooth[3] = {'b','l','t'};
int ADC_buffer[2];
uint32_t temp_val, temperature;
float vsense = 3.3/1023;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */


bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n);
void checkMode(void);
void checkAudio(void);
void checkRadio(void);
void bluetoothSettings(void);
void bluetoothCall(void);
void bluetoothMusic(void);
void checkBluetooth(void);
int arrayToInt(uint8_t mArr[]);
int arrayToInt_withIndex(uint8_t mArr[],int index);
bool checkDeviceI2cConnection(uint16_t DevAddress);
void tea5767Setfrequency( uint32_t frequency );


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){//recive from uart1
	HAL_UART_Receive (&huart1, &rx_data, 1,100);
  if( huart->Instance == USART1 )
  {    
    
    if(rx_index==0 || rx_index==-1)
    {
      for(int i=0; i<32; i++)
      {
        rx_buffer[i]=0;
        //type[i] = 0;
      }
    }
    
      //if the charcter received is other than'?' ascii 0x3f, save the data in buffer
    if (rx_data != 0x3f)
    {
      rx_buffer[rx_index++] = rx_data;
      
      
    }
    else
    {
      //HAL_UART_Transmit (&huart1, rx_buffer, rx_index,100); //transmit the data via usart      
      
      //VOICE CHANNEL SWITCHING
      if(areEqual(mode, rx_buffer,0,3)){
        //HAL_UART_Transmit (&huart1, rx_buffer, rx_index,100);
        checkMode();
				rx_index = 0;
				return;
        
      }
      
      //AUDIO SECTION
      if(areEqual(audio, rx_buffer,0 , 3)){
        checkAudio();
				rx_index = 0;
				return;
      }else if(areEqual(radio, rx_buffer,0 , 3)){  //Radio SECTION
				checkRadio();
				rx_index = 0;
				return;
      }
      
      //BLUeTOOTH SECTION
			
      if(areEqual(bluetooth, rx_buffer,0 , 3)){
				checkBluetooth();
				rx_index = 0;
				return;
      }
      
      
      
      
      //RTD SECTION
      /*if(areEqual(rtd, rx_buffer,0 , 3)){
        
      }*/

      rx_index = 0;

    }
    
    
    
  }
	
}






bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n){//compare two array
    // Linearly compare elements
    for (i; i<n; i++){
         if (arr1[i] != arr2[i]){
						//HAL_IWDG_Refresh(&hiwdg);
						return false;
				 }
			 }         
    // If all elements were same.
    return true;
}
int arrayToInt(uint8_t mArr[]){ //forr radio frq
  int b,o,n,m,k;
	
  if(mArr[1]==0x00){
    m = (mArr[0]-'0');
    k = m;
  }else if(mArr[2]==0x00){
		n = (mArr[0]-'0')*10;
    m = mArr[1]-'0';
    k = n+m;
	}else if(mArr[3]==0x00){
		o = (mArr[0]-'0')*100;
		n = (mArr[1]-'0')*10;
    m = mArr[2]-'0';
    k = o+n+m;	
	}else {
		b = (mArr[0]-'0')*1000;
		o = (mArr[1]-'0')*100;
		n = (mArr[2]-'0')*10;
    m = mArr[3]-'0';
    k = b+o+n+m;	
	}
  return k;
}

int arrayToInt_withIndex(uint8_t mArr[],int index){//converting recived string data from android to INT for sound values
  int b,o,n,m,k;
	if(index == 0){
		b= (mArr[index]-'0')*10;
		o= (mArr[index+1]-'0');
		k=b+o;
		return k;
	}else {
		b= (mArr[index]-'0')*100;
		o= (mArr[index+1]-'0')*10;
		n= (mArr[index+2]-'0');
		k=b+o+n;
		return k;
	}
	
}

bool checkDeviceI2cConnection(uint16_t DevAddress){
	//check I2c connection situation with pt2313 module
	init=HAL_I2C_IsDeviceReady(&hi2c1,DevAddress,1,10);
	if(init == HAL_OK){
		return true;
	}else return false;
}



/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
HAL_GPIO_WritePin( GPIOB, GPIO_PIN_14, 1);
	
//	HAL_GPIO_WritePin(ldro_GPIO_Port,ldro_Pin, 1);


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 2);
	__HAL_IWDG_START(&hiwdg);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //start timer2 channel_1 as pwm
	TIM2->CCR1 = 130;//pwm value from 0 to 65535

//__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
 __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);//ssssssssssssssssssssssssssssssssssssssssssssssssssss
 __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
	
	
		/*HAL_UART_Transmit(&huart1,"usart1 ok",11,2);
		HAL_UART_Transmit(&huart2,"usart2 ok",11,2);
		HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, 1);
		HAL_Delay(2000);*/
		
		
//HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);		
HAL_GPIO_WritePin(stbo_GPIO_Port,stbo_Pin, 0);

		
		
HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);		
HAL_GPIO_WritePin(stbo_GPIO_Port,stbo_Pin, 0);

HAL_GPIO_WritePin(enpo_GPIO_Port,enpo_Pin, 0);
HAL_GPIO_WritePin(enho_GPIO_Port,enho_Pin, 0);

				buffer[0]=30;
	buffer[1]=208;//192 223
	buffer[2]=240;//224 255
	buffer[3]=144;//128 159
	buffer[4]=176;//160 191
	buffer[5]=65;//64aux  65raspbery  66radio
	buffer[6]=96;
	buffer[7]=112;
	//if(checkDeviceI2cConnection(0x44<<1)){
		
		HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,buffer,7,2);
//	}
 	
    
		HAL_UART_Transmit (&huart1, "RUN", 3,10);


  while (1)
  {
		
		HAL_IWDG_Refresh(&hiwdg);
		//HAL_Delay(5000); //check watchDog
		
		
		/*acciPinState = HAL_GPIO_ReadPin(GPIOA,acci_Pin);
	if(acciPinState == GPIO_PIN_RESET)//GPIO_PIN_1
		{
				HAL_GPIO_WritePin(enpo_GPIO_Port,enpo_Pin, 0);
	 HAL_GPIO_WritePin(enho_GPIO_Port,enho_Pin, 0);		
	HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);		
HAL_GPIO_WritePin(stbo_GPIO_Port,stbo_Pin, 0);
    
			
  }else{
		
    HAL_GPIO_WritePin(enpo_GPIO_Port,enpo_Pin, 1);
	 HAL_GPIO_WritePin(enho_GPIO_Port,enho_Pin, 1);		
	HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);		
	HAL_GPIO_WritePin(stbo_GPIO_Port,stbo_Pin, 1);	
	}
	
	rcaiPinState = HAL_GPIO_ReadPin(GPIOA,rcai_Pin);
	if(rcaiPinState == GPIO_PIN_RESET)
	{
		
	}*/

	



//
	//HAL_GPIO_TogglePin(ldgo_GPIO_Port,ldgo_Pin);
//		HAL_GPIO_TogglePin(ldro_GPIO_Port,ldro_Pin);	
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 69;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 110;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(anto_GPIO_Port, anto_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, stbo_Pin|muto_Pin|swao_Pin|fan_Pin 
                          |enpo_Pin|enho_Pin|ldro_Pin|ldgo_Pin 
                          |ampo_Pin|acro_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : anto_Pin */
  GPIO_InitStruct.Pin = anto_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(anto_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : brdi_Pin ladi_Pin rcai_Pin acci_Pin */
  GPIO_InitStruct.Pin = brdi_Pin|ladi_Pin|rcai_Pin|acci_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : stbo_Pin muto_Pin swao_Pin fan_Pin 
                           enpo_Pin enho_Pin ldro_Pin ldgo_Pin 
                           ampo_Pin acro_Pin */
  GPIO_InitStruct.Pin = stbo_Pin|muto_Pin|swao_Pin|fan_Pin 
                          |enpo_Pin|enho_Pin|ldro_Pin|ldgo_Pin 
                          |ampo_Pin|acro_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void tea5767Setfrequency( uint32_t frequency )
{
  uint32_t pllValue;
 // uint8_t buffer[5] = { 0, 0, 0, 0, 0 };
  
  // Make sure I2C is initialised
 // if (!_tea5767Initialised) tea5767Init();

  // Calculate PLL word for high side injection mode
  // NDEC = (4*(FRF + FIF)) / FREFS
  // where:
  // FRF = Desired tuning frequency in Hz
  // FIF = Intermediate frequency in Hz (225kHz)
  // FREFS = Reference frequency in Hz (32.768kHz)
  pllValue = (4 * (frequency + 225000)) / 32768;
  
 // buffer[0] = (pllValue >> 8) & 0x3F;              // Upper 6 PLL bits (also turns mute and search mode off!)
 // buffer[1] = (pllValue & 0xFF);                   // Lower 8 PLL bits
  //buffer[2] = TEA5767_WBYTE3_HLSI;                 // High side injection mode
 // buffer[3] = TEA5767_WBYTE4_XTAL;                 // XTAL bit = 1 for 32.768kHz crystal
//  buffer[4] = 0;                                   // PLLREF bit = 0 for 32.768kHz crystal

  // Send data over I2C
  bt[0]=(pllValue >> 8) & 0x3F;
   bt[1]=(pllValue & 0xFF);
		// bt[0]=0x2c;
   // bt[1]=0x37;
    bt[2]=176;
    bt[3]=16;
    bt[4]=0;
		
		senddata[0]= bt[0];
		senddata[1]= bt[1];
		senddata[2]= bt[2];
		senddata[3]= bt[3];
		senddata[4]= bt[4];
		

		
		
	HAL_I2C_Master_Transmit(&hi2c1,RadioModuleI2CAddress,bt,5,2);	
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
