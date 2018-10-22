
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_PinState acciPinState, rcaiPinState;

	
/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef readAdc;


char ch[20];//usart
char ch2[20];//usart
char str[20];//uint8_t str[20];
uint8_t buffer[8]={0x00, 0x00, 0x00, 0x63, 0x04, 0x23, 0x12, 0x15}; 

uint8_t bt[5];                                               //buffer for tea5767
unsigned int pllfrq;
float pll;
uint32_t frq2;
uint8_t senddata[5];
HAL_StatusTypeDef init;

int d;
int rx_index=0;
int rx2_index=0;
uint8_t rx_data;
uint8_t rx_buffer[22];
uint8_t rx2_data;
uint8_t rx2_buffer[22];
uint8_t type[3];


uint8_t mode[3]={'m','o','d'};
uint8_t rspnsReadyStatus[3]= {'M','G','1'};
uint8_t rspnsConnectingStatus[3]= {'M','G','2'};
uint8_t rspnsConnectedStatus[3]= {'M','G','3'};
uint8_t rspnsOutCallStatus[3]= {'M','G','4'};
uint8_t rspnsInCallStatus[3]= {'M','G','5'};
uint8_t rspnsOnCallStatus[3]= {'M','G','6'};
uint8_t rspnsNum[3]= {'N','U','M'};
uint8_t audio[3]={'a','u','d'};
uint8_t radio[3]={'r','a','d'};
uint8_t bluetooth[3]={'b','l','t'};
uint8_t rspnsEnterPairing[2]= {'I','I'};
uint8_t rspnsConnectToDevice[2]= {'I','V'};
uint8_t rspnsOutgoingCall[2]= {'I','R'};

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
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);


bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n);
void checkMode(void);
void checkAudio(void);
void checkRadio(void);
void bluetoothSettings(void);
void bluetoothCall(void);
void checkBluetooth(void);
int arrayToInt(uint8_t mArr[]);
bool checkDeviceI2cConnection(uint16_t DevAddress);
void tea5767Setfrequency( uint32_t frequency );


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive (&huart1, &rx_data, 1,100);
  if( huart->Instance == USART1 )
  {    
    
    if(rx_index==0 || rx_index==-1)
    {
      for(int i=0; i<20; i++)
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
        
      }
      
      //AUDIO SECTION
      if(areEqual(audio, rx_buffer,0 , 3)){
        checkAudio();
      }else if(areEqual(radio, rx_buffer,0 , 3)){  //Radio SECTION
				checkRadio();
      }
      
      //BLUeTOOTH SECTION
			
      if(areEqual(bluetooth, rx_buffer,0 , 3)){
				
				checkBluetooth();
				
      }
      
      
      
      
      //RTD SECTION
      /*if(areEqual(rtd, rx_buffer,0 , 3)){
        
      }*/

      rx_index = 0;

    }
    //HAL_UART_Receive_IT (&huart1, &rx_data, 1); // Receive data (one character only)
    
    
  }
	
//	HAL_UART_Receive(&huart1,(uint8_t*) ch,1,1000);
	
//	str[0]=ch[0];
//	str[1]=ch[1];
	//if(buffercmp(Receive_buffer,"led"==0xff){
	//if(buffercmp(Receive_buffer,"led"==0xff){
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);

		

//	HAL_UART_Transmit(&huart1,(uint8_t*) ch,1,10);
	//ch[0]=0;
	//}
	//HAL_UART_Receive_IT(&huart1,(unsigned char*)Receive_buffer,3);
	
  /* Prevent unused argument(s) compilation warning */
 // UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxHalfCpltCallback can be implemented in the user file
   */
	 
	 
//	 	HAL_UART_Receive(&huart2,(uint8_t*) ch2,1,10);
	

//	HAL_UART_Transmit(&huart2,(uint8_t*) ch2,10,10);
	
}

//recive from uart2
void HAL_UART_RxCpltCallback2(UART_HandleTypeDef *huart)
	{
		HAL_UART_Receive (&huart2, &rx2_data, 1,10);
		if(huart->Instance == USART2)
		{    
			
			if(rx2_index==0)
			{
				for(int i=0; i<20; i++)
				{
					rx2_buffer[i]=0;
					
				}
				
			}
    
				//if the charcter received is other than'enter' ascii 0x3f, save the data in buffer
			if (rx2_data != 0x0A )
			{
				//HAL_UART_Transmit(&huart1, &rx2_buffer[0,1,2], 20, 100);
				rx2_buffer[rx2_index++] = rx2_data;

			}
			else
			{
				//check 0-9 and + in 0 index of rx2 buffer for read incoming call
				if(rx2_buffer[0] == 0x2B || rx2_buffer[0] == 0x30 || rx2_buffer[0] == 0x31 || rx2_buffer[0] == 0x32
						|| rx2_buffer[0] == 0x33  || rx2_buffer[0] == 0x34  || rx2_buffer[0] == 0x35
						|| rx2_buffer[0] == 0x36  || rx2_buffer[0] == 0x37  || rx2_buffer[0] == 0x38
						|| rx2_buffer[0] == 0x39){
						
						HAL_UART_Transmit(&huart1, rx2_buffer, 22,10);
						
					}
				if(rx2_index>=1){
					if(areEqual(rx2_buffer, rspnsReadyStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsReadyStatus, 3,10);
					}
					
					if(areEqual(rx2_buffer, rspnsConnectingStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsConnectingStatus, 3,10);
					}
					
					if(areEqual(rx2_buffer, rspnsConnectedStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsConnectedStatus, 3,10);
					}
					
					if(areEqual(rx2_buffer, rspnsInCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsInCallStatus, 3,10);
						
						
					}
					
					
					
					if(areEqual(rx2_buffer, rspnsOnCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsOnCallStatus, 3,10);
					}
					
					
					if(areEqual(rx2_buffer, rspnsOutCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsOutCallStatus, 3,10);
					}
					
					if(areEqual(rx2_buffer, rspnsOutgoingCall, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsOutgoingCall, 2,10);
					}
					
					if(areEqual(rx2_buffer, rspnsEnterPairing, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsEnterPairing, 2,10);
					}
					
					if(areEqual(rx2_buffer, rspnsConnectToDevice, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsConnectToDevice, 2,10);
					}
				}
				
				rx2_index = 0;
			}
    
    
    //return;
		}

	
}

void checkBluetooth(){

	static uint8_t settings[3]= {'s','t','n'};
	static uint8_t call[3]= {'c','l','l'};
	
	uint8_t secondType[3];
	
	for (int i=4; i<7; i++){
		secondType[i-4] = rx_buffer[i];
	}
	
	if (areEqual(secondType, settings, 0,3)){
		bluetoothSettings();
		
	}else if (areEqual(secondType, call, 0, 3)){
		bluetoothCall();
	}else{
		//bluetoothMusic();
	}
	
	
}

void bluetoothCall(){
	
	static uint8_t answer[3]= {'a','n','s'};
	static uint8_t reject[3]= {'r','j','t'};
	static uint8_t endCall[3]= {'e','n','d'};
	static uint8_t redial[3]= {'r','d','l'};
	static uint8_t checkStatus[3]= {'c','h','k'};
	uint8_t secondType[14];
	
	for (int i=8; i<12; i++){
		secondType[i-8] = rx_buffer[i];
	}
	
	if(areEqual(answer, secondType, 0, 3)){						//answer incoming call
		HAL_UART_Transmit(&huart2, "AT#CE\r", 6,100);
		return;
		
	}else if(areEqual(reject, secondType, 0, 3)){			//reject
		HAL_UART_Transmit(&huart2, "AT#CF\r", 6,100);
		return;
		
	}else if(areEqual(endCall, secondType, 0, 3)){		//end call
		HAL_UART_Transmit(&huart2, "AT#CG\r", 6,100);
		return;
		
	}else if(areEqual(checkStatus, secondType, 0, 3)){		//end call
		HAL_UART_Transmit(&huart2, "AT#CY\r", 6,100);
		return;
		
	}else if(areEqual(redial, secondType, 0, 3)){			//redial
		HAL_UART_Transmit(&huart2, "AT#CH\r", 6,100);
		return;
		
	}else{//outgoing call
		
		for (int i=8; i<23; i++){
			secondType[i-8] = rx_buffer[i];
			
		}
		HAL_UART_Transmit(&huart2, "AT#CW", 5,100);
		HAL_UART_Transmit(&huart2, secondType, 14,100);
		HAL_UART_Transmit(&huart2, "\r", 1,100);
		
		return;
	}
	
	
}


void bluetoothSettings(){
	static uint8_t enterPairing[3]= {'p','o','n'}; 
	static uint8_t cancelPairing[3]= {'p','o','f'}; 
	static uint8_t enableAutoConn[3]= {'e','a','c'}; 
	static uint8_t disableAutoConn[3]= {'d','a','c'}; 
	static uint8_t enableAutoAnswer[3]= {'e','a','n'}; 
	static uint8_t disableAutoAnswer[3]= {'d','a','n'}; 
	uint8_t secondType[3];
	
	for (int i=8; i<12; i++){
		secondType[i-8] = rx_buffer[i];
	}
	
	//start and stop pairing mode
	if(areEqual(secondType, enterPairing, 0,3)){
		HAL_UART_Transmit(&huart2, "AT#CA\r", 6,100);
		
		return;
	}else if(areEqual(secondType, cancelPairing, 0,3)){
		HAL_UART_Transmit(&huart2, "AT#CB\r", 6,100);
		HAL_UART_Transmit(&huart1, "Cancel pairing mode", 19,100);
		return;
	}
	
	//enable and disable Auto connect mode
	if(areEqual(secondType, enableAutoConn, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MG\r", 6,100);
		HAL_UART_Transmit(&huart1, "Enable Auto connect", 19,100);
		return;
	}else if(areEqual(secondType, disableAutoConn, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MH\r", 6,100);
		HAL_UART_Transmit(&huart1, "Disable Auto connect", 20,100);
		return;
	}
	
	//enable and disable Auto Answer mode
	if(areEqual(secondType, enableAutoAnswer, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MP\r", 6,100);
		HAL_UART_Transmit(&huart1, "Enable Auto Answer", 18,100);
		return;
	}else if(areEqual(secondType, disableAutoAnswer, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MQ\r", 6,100);
		HAL_UART_Transmit(&huart1, "Disable Auto Answer", 19,100);
		return;
	}
	
	
}

bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n)
{
    // Linearly compare elements
    for (i; i<n; i++){
         if (arr1[i] != arr2[i]){
					return false;
				 }
			 }         
    // If all elements were same.
    return true;
}

void checkMode(){
	if(checkDeviceI2cConnection(0x44<<1)){
		
		
		static uint8_t aux[3]={'a','u','x'};
		uint8_t secondType[3];
	
		for(int i=4; i<7; i++){
			secondType[i-4]=rx_buffer[i];
		}
		HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);  // mute befor change mode
		if(areEqual(secondType, radio, 0, 3)){
		
			HAL_UART_Transmit (&huart1, secondType, 3,100);
		
			frq2=94000000;
			tea5767Setfrequency(frq2);
			buffer[5]=66;	
			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
		}else if(areEqual(secondType, aux, 0, 3)){
			buffer[5]=64;	
			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			HAL_UART_Transmit (&huart1, "AUX", 3,100);
		}else{
			buffer[5]=65;	
			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			HAL_UART_Transmit (&huart1, "pin", 3,100);
		}
		HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);
		}else{ HAL_UART_Transmit (&huart1, "modProb", 7,10);
			
		}
		
}

void checkAudio(){                         //Audio Module settings
	if(checkDeviceI2cConnection(0x44<<1)){
		
		static uint8_t volume[3] ={'v','o','l'};
		static uint8_t vol_lf[3] ={'v','l','f'};
		static uint8_t vol_rf[3] ={'v','r','f'};
		static uint8_t vol_lr[3] ={'v','l','r'};
		static uint8_t vol_rr[3] ={'v','r','r'};
		static uint8_t vol_bas[3] ={'b','a','s'};
		static uint8_t vol_treble[3] ={'t','b','l'};
		static uint8_t vol_loud[3] ={'l','o','d'};
		uint8_t secondType[4];
		
		for(int i=4; i<7; i++){
			secondType[i-4]=rx_buffer[i];
		}
		if(areEqual(secondType, volume, 0, 3)){ //change volume
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
			
			buffer[0]= arrayToInt(secondType);
			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);		//+++++++++++++++++++++++++++++++++++++++++++++++
			HAL_UART_Transmit (&huart1, "changed vol", 11,100);
			if(buffer[0]==63){
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);
				//HAL_GPIO_WritePin(muto_GPIO_Port,stbo_Pin, 1);	 
			}else{
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);
				//HAL_GPIO_WritePin(muto_GPIO_Port,stbo_Pin, 0);	 
			 
			}
		}else if(areEqual(secondType, vol_lf, 0, 3)){ //speaker left front
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
					buffer[1]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}else if(areEqual(secondType, vol_rf, 0, 3)){ //speaker right front
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
					buffer[2]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);	
			
		}else if(areEqual(secondType, vol_lr, 0, 3)){ //speaker left rear
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
					buffer[3]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}else if (areEqual(secondType, vol_rr, 0, 3)){ //speaker right rear
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
					buffer[4]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}else if (areEqual(secondType, vol_bas, 0, 3)){ //change bas
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
					buffer[6]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}else if (areEqual(secondType, vol_loud, 0, 3)){ //change Loud
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
			//		buffer[0]= arrayToInt(secondType);
    		//	HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}else { 																			 //change treble
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
							buffer[7]= arrayToInt(secondType);
    			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
			
		}
	}else HAL_UART_Transmit (&huart1, "audProb", 7,10);
  
 
}

void checkRadio(){									//RADIO Module settings
	if(checkDeviceI2cConnection(0x60<<1)){
		uint8_t frequency[3]={'f','r','q'};
		uint8_t secondType[4];
			
		for(int i=4; i<7; i++){
			secondType[i-4]=rx_buffer[i];
		}
		
		if(areEqual(secondType, frequency, 0, 3)){ //change frequency
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
			HAL_UART_Transmit (&huart1, "radiFrq", 7,10);
			frq2=arrayToInt(secondType);
			frq2=frq2*100000;
			
			tea5767Setfrequency(frq2);
			HAL_UART_Transmit (&huart1, "okk frq", 3,100);

		}
	}
	
}

int arrayToInt(uint8_t mArr[]){
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
  d=k;
  return k;
}

bool checkDeviceI2cConnection(uint16_t DevAddress){
	//check I2c connection situation with pt2313 module
	init=HAL_I2C_IsDeviceReady(&hi2c1,DevAddress,1,100);
	if(init == HAL_OK){
		return true;
	}else return false;
}



/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int ADC_result[2],ADC_buffer[2];
int ADC_average=0;
		int ADC_sum=0;
		int ADC_counter=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	if(hadc->Instance == ADC1){
		
		ADC_result[0] = ADC_buffer[0];
		ADC_result[1] = ADC_buffer[1];
		if(ADC_result[0] < 3350){
			
				ADC_sum += ADC_result[0];
				ADC_counter = ADC_counter++;
				ADC_result[0] =ADC_sum / ADC_counter;
			if(ADC_counter > 200){
				uint8_t ADC_buffer_send[4];
				for(int i=0; i<3000000 ; i++){}
				ADC_buffer_send[3] = (( ADC_result[0] / 1 ) % 10) + '0';
				ADC_buffer_send[2] = (( ADC_result[0] / 10 ) % 10) + '0';
				ADC_buffer_send[1] = (( ADC_result[0] / 100 ) % 10) + '0';
				ADC_buffer_send[0] = (( ADC_result[0] / 1000 ) % 10) + '0';
				HAL_UART_Transmit (&huart1, ADC_buffer_send, 4,100);
			}
			
			
			HAL_IWDG_Refresh(&hiwdg);
		}else {
		ADC_sum = 0;
		ADC_counter = 0;
		ADC_result[0] = 0;
		}
	}
}
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 2);
	__HAL_IWDG_START(&hiwdg);

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

		
		
HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);		
HAL_GPIO_WritePin(stbo_GPIO_Port,stbo_Pin, 0);

HAL_GPIO_WritePin(enpo_GPIO_Port,enpo_Pin, 0);
	 HAL_GPIO_WritePin(enho_GPIO_Port,enho_Pin, 0);

				buffer[0]=31;
	buffer[1]=208;//192 223
	buffer[2]=240;//224 255
	buffer[3]=144;//128 159
	buffer[4]=176;//160 191
	buffer[5]=66;//64aux  65raspbery  66radio
	buffer[6]=96;
	buffer[7]=112;
 HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);	
    

//	frq2=88000000;
  while (1)
  {
		
		//ADCresult = 4000;
		HAL_IWDG_Refresh(&hiwdg);
		//HAL_Delay(5000); //check watchDog
		/*=== start adc ===*/
		//HAL_ADC_Start(&hadc1);         
		
		
		

		//readAdc = HAL_ADC_PollForConversion(&hadc1,100);
		 
		/*if(readAdc == HAL_OK){ 
			
		  temp_val = HAL_ADC_GetValue(&hadc1);
		}*/
		
		//HAL_ADC_PollForConversion(&hadc2, 100);
		//ADCresult = HAL_ADC_GetValue(&hadc2);
		
		
		
		//HAL_ADC_Stop(&hadc1); 
		//HAL_ADC_Stop(&hadc2);
		//temperature = ((temp_val-1.43)/4.3)+25;
		
		acciPinState = HAL_GPIO_ReadPin(GPIOA,acci_Pin);
	/*if(acciPinState == GPIO_PIN_RESET)//GPIO_PIN_1
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
	}*/
	
	rcaiPinState = HAL_GPIO_ReadPin(GPIOA,rcai_Pin);
	if(rcaiPinState == GPIO_PIN_RESET)
	{
		
	}

	
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
    bt[2]=16;
    bt[3]=18;
    bt[4]=0;
		
		senddata[0]= bt[0];
		senddata[1]= bt[1];
		senddata[2]= bt[2];
		senddata[3]= bt[3];
		senddata[4]= bt[4];
		

		
		
	HAL_I2C_Master_Transmit(&hi2c1,0x60<<1,bt,5,10);	
	
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
