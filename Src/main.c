/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"



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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
GPIO_PinState acciPinState, reverseGearPinState, carLightLampPinState;

	
/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef readAdc;
bool debugState = true;
bool  avTVinputState = false;
bool  carLightState = false;
bool  radioAntenaState = false;
bool  avCameraInputState = false;
bool  accState = false;
bool  firstRun = true;
bool  firstOFF = true;
bool  headUnitCPU_HighTemp = false;
bool delaySwChannelOne = false;
bool delaySwChannelTwo = false;


//uint8_t buffer[8]={0x00, 0x00, 0x00, 0x63, 0x04, 0x23, 0x12, 0x15}; 
uint8_t pt2313_buffer[8]=
{15,//mainVolume
192,//LeftFront 223-192
224,//RightFront 255-224
128,//LeftRear 159-128
160,//RightRear 191-160
93,//switch mode 64aux  93android & BT  90radio
96,//base 111-96
112//treble 127-112
}; 

int  i2c_timeout = 2;
int uart_timeout = 100;

int bluetoothCMDlength = 6;
uint32_t TimCall = 0;
HAL_StatusTypeDef init = HAL_BUSY;
int rx_index=0;
int MCU_SHUTDOWN_TIMER = 0;
bool fanPermition = true;
bool fanState = true;
uint8_t rx_data = 0;
uint8_t rx_buffer[64];

uint8_t rx2_data=0;
uint8_t rx2_buffer[64];
int rx2_index=0;

double VroomTemp = 1.43;
double avg_slope = 4.3;
double Vsense = 0;
uint32_t temp = 0;



uint16_t soundModuleI2CAddress = 0x89;
uint16_t RadioModuleI2CAddress = 0xC0;


uint8_t mode[3] = {'m','o','d'};			//MODE androidBT , Radio , AUX
uint8_t audio[3] = {'a','u','d'};			//Audio
uint8_t radio[3] = {'r','a','d'};			//Radio
uint8_t bluetooth[3] = {'b','l','t'};	//Bluetooth
uint8_t other[3] = {'o','t','h'};     //other commads such as brightness , cpu temp , switch av channel
uint8_t debug[3] = {'d','b','g'};			//Debug

uint8_t rspnsReadyStatus[3]= {'M','G','1'};
uint8_t rspnsConnectingStatus[3]= {'M','G','2'};
uint8_t rspnsConnectedStatus[3]= {'M','G','3'};
uint8_t rspnsOutCallStatus[3]= {'M','G','4'};
uint8_t rspnsInCallStatus[3]= {'M','G','5'};
uint8_t rspnsOnCallStatus[3]= {'M','G','6'};
uint8_t rspnsNum[3]= {'N','U','M'};
uint8_t rspnsEnterPairing[2]= {'I','I'};
uint8_t rspnsMusicStart[2]= {'M','B'};
uint8_t rspnsMusicPause[2]= {'M','P'};
uint8_t rspnsMusicStop[2]= {'M','A'};
uint8_t rspnsMusicResume[2]= {'M','R'};
uint8_t rspnsConnectToDevice[2]= {'I','V'};
uint8_t rspnsOutgoingCall[2]= {'I','R'};

uint32_t ADC_buffer[2];

float vsense = 3.3/1023;
int pwmValue = 130 ; //brightness default value 
int pwmValueLightOn = 95 ;
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
/* USER CODE BEGIN PFP */

void custom_delay(uint32_t milliseconds);
bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n);
void checkMode(void);
void checkAudio(void);
void checkRadio(void);
void bluetoothSettings(void);
void bluetoothCall(void);
void bluetoothMusic(void);
void checkBluetooth(void);
void checkOtherCommands(void);
int arrayToInt(uint8_t mArr[]);
int arrayToInt_withIndex_soundValues(uint8_t mArr[],int index);
int arrayToInt_withIndex_radioValues(uint8_t mArr[],int index);
bool checkDeviceI2cConnection(uint16_t DevAddress);
bool reverseGearPinStateFunc (void);
bool acciPinStateFunc (void);
void getARMTemp_SetFanState(void);
void changeChannelDelay (int channel);


void custom_delay(uint32_t milliseconds) {

   /* Initially clear flag */

   (void) SysTick->CTRL;

   while (milliseconds != 0) {

      /* COUNTFLAG returns 1 if timer counted to 0 since the last flag read */

      milliseconds -= (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> SysTick_CTRL_COUNTFLAG_Pos;

   }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){//recive from uart1
	
  if(huart->Instance == USART1  )
  {    
    HAL_UART_Receive_DMA (&huart1, &rx_data, 1);
    if(rx_index==0 || rx_index==-1|| rx_index==0xFF)
    {
      for(int i=0; i<64; i++)
      {
        rx_buffer[i]=0;
				
      }
			
    }
      //if the charcter received is other than'?' ascii 0x3f, save the data in buffer
    if (rx_data != 0x3f)
    {
      rx_buffer[rx_index++] = rx_data;
    }
    else
    {
     if(accState){     
      //VOICE CHANNEL SWITCHING
      if(areEqual(mode, rx_buffer,0,3) ){//sound module is on checked with acc state
        checkMode();
				rx_index = 0;
				return;
        
      }
      
      
      if(areEqual(audio, rx_buffer,0 , 3) ){  //AUDIO SECTION
        checkAudio();
				rx_index = 0;
				return;
      }
			if(areEqual(radio, rx_buffer,0 , 3) ){  //Radio SECTION
				checkRadio();
				rx_index = 0;
				return;
      }
      
      
			
      if(areEqual(bluetooth, rx_buffer,0 , 3) ){	//BT SECTION
				checkBluetooth();
				rx_index = 0;
				return;
      }
			//enable and disable debugmode
      if(areEqual(debug, rx_buffer,0 , 3) ){
				if(debugState){
					debugState = false;
					HAL_UART_Transmit (&huart1, (uint8_t*)"dOff", 4, uart_timeout);
				}else{
					debugState = true;
					HAL_UART_Transmit (&huart1, (uint8_t*)"dOn", 3, uart_timeout);
				}
				rx_index = 0;
				return;
      }
		}
			
			if(areEqual(other, rx_buffer,0 , 3)){	//Other SECTION
				checkOtherCommands();
				rx_index = 0;
				return;
      }

      rx_index = 0;

    }
  }

	
		if(huart->Instance == USART2 )
		{
			HAL_UART_Receive_DMA (&huart2, &rx2_data, 1);
			
			if(rx2_index > 50 ||rx2_buffer[60]!=0x00 || rx2_index==0 ||rx2_data == 0x7E||rx2_data == 0x7B||rx2_data == 0xFF||rx2_data == 0xC7||rx2_data == 0x7C||rx2_data == 0x7E|| rx2_data == 0xFF || rx2_data == 0x3E || rx2_data == 0xEF|| rx2_data == 0xFB||rx2_data == 0xFE||rx2_data == 0x2E||rx2_data == 0xFC || rx2_data == 0x7F|| rx2_data == 0xFC||rx2_data == 0x1F)
			{
				for(int i=0; i<64; i++)
				{
					rx2_buffer[i]=0;
					rx2_index=0;
					
				}
				
			}
    
				//if the charcter received is other than'enter' ascii 0x3f, save the data in buffer
			if (rx2_data != 0x0A && accState)
			{
				
				rx2_buffer[rx2_index++] = rx2_data;

			}
			else
			{
				//check 0-9 and + in 0 index of rx2 buffer for read incoming call
				if(rx2_buffer[0] == 0x2B || rx2_buffer[0] == 0x30 || rx2_buffer[0] == 0x31 || rx2_buffer[0] == 0x32
						|| rx2_buffer[0] == 0x33  || rx2_buffer[0] == 0x34  || rx2_buffer[0] == 0x35
						|| rx2_buffer[0] == 0x36  || rx2_buffer[0] == 0x37  || rx2_buffer[0] == 0x38
						|| rx2_buffer[0] == 0x39){
						
						HAL_UART_Transmit_IT(&huart1, rx2_buffer, 22);
						
					}
				if(rx2_index>=1){
					if(areEqual(rx2_buffer, rspnsReadyStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsReadyStatus, 3);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectingStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsConnectingStatus, 3);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectedStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsConnectedStatus, 3);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsInCallStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsInCallStatus, 3);
						rx2_index = 0;
						return;
						
						
					}
					
					
					
					if(areEqual(rx2_buffer, rspnsOnCallStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsOnCallStatus, 3);
						rx2_index = 0;
						return;
					}
					
					
					if(areEqual(rx2_buffer, rspnsOutCallStatus, 0, 3)){
						HAL_UART_Transmit_IT(&huart1, rspnsOutCallStatus, 3);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsOutgoingCall, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsOutgoingCall, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicStart, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsMusicStart, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicResume, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsMusicResume, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicPause, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsMusicPause, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicStop, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsMusicStop, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsEnterPairing, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsEnterPairing, 2);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectToDevice, 0, 2)){
						HAL_UART_Transmit_IT(&huart1, rspnsConnectToDevice, 2);
						rx2_index = 0;
						return;
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
		bluetoothMusic();
	}
	
	
}
	
void bluetoothCall(){
	
	static uint8_t answer[3]= {'a','n','s'};
	static uint8_t reject[3]= {'r','j','t'};
	static uint8_t endCall[3]= {'e','n','d'};
	static uint8_t redial[3]= {'r','d','l'};
	static uint8_t checkStatus[3]= {'c','h','k'};
	static uint8_t releaseHeldCall[3] = {'r', 'h', 'c'};
	static uint8_t releaseActiveCall[3] = {'r', 'a', 'c'};
	static uint8_t holdActiveCall[3] = {'h', 'a', 'c'};
	static uint8_t audioTransfer[3] = {'a', 't', 'r'};
	uint8_t secondType[14];
	
	for (int i=8; i<12; i++){
		secondType[i-8] = rx_buffer[i];
	}
	
	if(areEqual(checkStatus, secondType, 0, 3)){						//check Status call
		
		//custom_delay(700);
		HAL_UART_Transmit(&huart2, "AT#CY\r", bluetoothCMDlength,uart_timeout);
		
		return;
		
	}else if(areEqual(reject, secondType, 0, 3)){			//reject
		HAL_UART_Transmit(&huart2, "AT#CF\r", bluetoothCMDlength,uart_timeout);
		return;
		
	}else if(areEqual(endCall, secondType, 0, 3)){		//end call
		HAL_UART_Transmit(&huart2, "AT#CG\r", bluetoothCMDlength,uart_timeout);
		return;
		
	}else if(areEqual(answer, secondType, 0, 3)){		//answer incoming call
		HAL_UART_Transmit(&huart2, "AT#CE\r", bluetoothCMDlength,uart_timeout);
		return;
		
	}else if(areEqual(redial, secondType, 0, 3)){			//redial
		HAL_UART_Transmit(&huart2, "AT#CH\r", bluetoothCMDlength,uart_timeout);
		return;
		
	}
	else if(areEqual(audioTransfer, secondType, 0, 3)){			//Audio transfer
		HAL_UART_Transmit(&huart2, "AT#CO\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(releaseHeldCall, secondType, 0,3)){  //release held call, reject waiting call
		HAL_UART_Transmit(&huart2, "AT#CQ\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(releaseActiveCall, secondType, 0,3)){  //release active call, accept other call
		HAL_UART_Transmit(&huart2, "AT#CR\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(holdActiveCall, secondType, 0,3)){  //hold active call, accept other call
		HAL_UART_Transmit(&huart2, "AT#CS\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else{//outgoing call
		
		for (int i=8; i<23; i++){
			secondType[i-8] = rx_buffer[i];
			
		}
		HAL_UART_Transmit(&huart2, "AT#CW", 5,uart_timeout);
		HAL_UART_Transmit(&huart2, secondType, 14,uart_timeout);
		HAL_UART_Transmit(&huart2, "\r", 1,uart_timeout);
		
		return;
	}
	
	
}

void bluetoothMusic(){
	{
	static uint8_t decreaseVolume[3]= {'v','d','n'};
	static uint8_t increaseVolume[3]= {'v','u','p'};
	static uint8_t muteMic[3]= {'m','u','t'};
	static uint8_t playPause[3] = {'p', 'p', 'p'};
	static uint8_t stop[3] = {'s', 't', 'p'};
	static uint8_t forward[3]= {'f','w','d'};
	static uint8_t backward[3]= {'b','w','d'};
	uint8_t secondType[3];
	
	for (int i=8; i<12; i++){
		secondType[i-8] = rx_buffer[i];
	}
	
	if(areEqual(decreaseVolume, secondType, 0, 3)){ //volume down
		HAL_UART_Transmit(&huart2, "AT#VD\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(increaseVolume, secondType, 0, 3)){  //volume Up
		HAL_UART_Transmit(&huart2, "AT#VU\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(muteMic, secondType, 0, 3)){	//TOGGLE MIC
		HAL_UART_Transmit(&huart2,"AT#CM\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(playPause, secondType, 0, 3)){  //play , pause
		HAL_UART_Transmit(&huart2, "AT#MA\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(stop, secondType, 0, 3)){  //stop
		HAL_UART_Transmit(&huart2, "AT#MC\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(forward, secondType, 0, 3)){  //forward
		HAL_UART_Transmit(&huart2, "AT#MD\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	else if(areEqual(backward, secondType, 0, 3)){ //backward
		HAL_UART_Transmit(&huart2, "AT#ME\r", bluetoothCMDlength,uart_timeout);
		return;
	}
	
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
		HAL_UART_Transmit(&huart2, "AT#CA\r", bluetoothCMDlength,uart_timeout);
		
		return;
	}else if(areEqual(secondType, cancelPairing, 0,3)){
		HAL_UART_Transmit(&huart2, "AT#CB\r", bluetoothCMDlength,uart_timeout);
		HAL_UART_Transmit_IT(&huart1, "Cancel pairing mode", 19);
		return;
	}
	
	//enable and disable Auto connect mode
	if(areEqual(secondType, enableAutoConn, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MG\r", bluetoothCMDlength,uart_timeout);
		HAL_UART_Transmit_IT(&huart1, "Enable Auto connect", 19);
		return;
	}else if(areEqual(secondType, disableAutoConn, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MH\r", bluetoothCMDlength,uart_timeout);
		HAL_UART_Transmit_IT(&huart1, "Disable Auto connect", 20);
		return;
	}
	
	//enable and disable Auto Answer mode
	if(areEqual(secondType, enableAutoAnswer, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MP\r", bluetoothCMDlength,uart_timeout);
		HAL_UART_Transmit_IT(&huart1, "Enable Auto Answer", 18);
		return;
	}else if(areEqual(secondType, disableAutoAnswer, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MQ\r", bluetoothCMDlength,uart_timeout);
		HAL_UART_Transmit_IT(&huart1, "Disable Auto Answer", 19);
		return;
	}
	
	
}



bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n){//compare two array
    // Linearly compare elements
    for (int a=i; a<n; a++){
         if (arr1[a] != arr2[a]){
						return false;
				 }
			 }         
    // If all elements were same.
    return true;
}

int arrayToInt(uint8_t mArr[]){ //for radio frq
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

int arrayToInt_withIndex_soundValues(uint8_t mArr[],int index){//converting recived string data from android to INT for sound values
  int b,o,n,k;
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

int arrayToInt_withIndex_radioValues(uint8_t mArr[],int index){//converting recived string data from android to INT for radio values
  int b,o,n,k,j;
	if(index == 0 || index == 7){
		k= (mArr[index]-'0');
		return k;
	}else {
		b= (mArr[index]-'0')*1000;
		o= (mArr[index+1]-'0')*100;
		n= (mArr[index+2]-'0')*10;
		j= (mArr[index+3]-'0');
		k=b+o+n+j;
		return k;
	}
	
}

bool checkDeviceI2cConnection(uint16_t DevAddress){//check I2c connection situation with i2c modules
	
	
	/* Checks if target device is ready for communication. */
	/* 64 is number of trials, 1000ms is timeout */
	  
	
	if(HAL_I2C_IsDeviceReady(&hi2c1,DevAddress,100,500) == HAL_OK){
		return true;
	}else return false;
}


bool reverseGearPinStateFunc (){
	reverseGearPinState = HAL_GPIO_ReadPin(GPIOA,rearCameraInput_Pin); //check reverse gear state
	if(reverseGearPinState == GPIO_PIN_RESET){
		return true; //set gear on reverse state
	}else
		return false; //not on reverse
}

bool acciPinStateFunc (){
	acciPinState = HAL_GPIO_ReadPin(GPIOA,accInput_Pin); //check Switch On state
	if(acciPinState == GPIO_PIN_RESET){
		accState = true;
		return true; //switch on
		}else{
		accState = false;
		return false; //off
		}
}

void getARMTemp_SetFanState(){
	//temp = ((VroomTemp - Vsense) / avg_slope) + 25;
	temp= (147.5 - ((75.0*3.5 *Vsense)) / 4096.0);
	if(temp > 60 || headUnitCPU_HighTemp){
		HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_SET);
	}else if(temp < 56 && !headUnitCPU_HighTemp){
		HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_RESET);
	}
}

void changeChannelDelay(int channel){
	if(channel == 1){
		delaySwChannelTwo = false;
		if(!delaySwChannelOne){
			delaySwChannelOne = true;
			TIM2->CCR1 = 0;
			HAL_Delay(3000);
		}
	}else{
		delaySwChannelOne = false;
		if(!delaySwChannelTwo){
			delaySwChannelTwo = true;
			TIM2->CCR1 = 0;
			HAL_Delay(2000);
		}
	}
}


/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		if(shutDownTimer < 60){
			shutDownTimer++;
		}else
			shutDownTimer = 0;
	}
	htim1.Init.Prescaler = 1099;     init for 1 second counting
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65514;
}*/


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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	__HAL_IWDG_START(&hiwdg);
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) ADC_buffer, 2);
	
	//HAL_TIM_Base_Start_IT(&htim1); //statrt timer for counting turn off system
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //start timer2 channel_1 as pwm
	TIM2->CCR1 = pwmValue;//pwm value from 0 to 65535

//__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);//ssssssssssssssssssssssssssssssssssssssssssssssssssss
	//__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	HAL_UART_Receive_DMA (&huart1, &rx_data, 1);
	HAL_UART_Receive_DMA (&huart2, &rx2_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	HAL_GPIO_WritePin(headunitOutput_GPIO_Port, headunitOutput_Pin, GPIO_PIN_RESET); //set headUnit Off for first time installing and on with first acc power on
	HAL_GPIO_WritePin(muteOutput_GPIO_Port, muteOutput_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(power12V_GPIO_Port, power12V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_SET);
	TIM2->CCR1 = 0;
	HAL_Delay(500);
	HAL_GPIO_WritePin(headunitOutput_GPIO_Port, headunitOutput_Pin, GPIO_PIN_SET);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_Delay(20000);
	
	/* check i2c Addresses 
	for(int i = 0 ; i < 255 ;i++){
						if(HAL_I2C_IsDeviceReady(&hi2c1,i,10	,1000) == HAL_OK){
						custom_delay(1000);
					}
					}
	*/
	//HAL_UART_Transmit (&huart1, (uint8_t*)"RUN", 3, uart_timeout);
  while (1)
  {
		
		
			HAL_IWDG_Refresh(&hiwdg);
		
		
				//HAL_Delay(30000); //check watchDog
			 //watch dog with down init 30 seconds need to reset

			carLightLampPinState = HAL_GPIO_ReadPin(GPIOA,lampDetectInput_Pin); //check car light_lamp state9
			reverseGearPinState = HAL_GPIO_ReadPin(GPIOA,rearCameraInput_Pin); //check reverse gear state
			acciPinState = HAL_GPIO_ReadPin(GPIOA,accInput_Pin); //check Switch On state
			
		
			if(acciPinStateFunc())//GPIO_PIN_1
			{
				__HAL_UART_ENABLE(&huart2);
				fanPermition = true;
				MCU_SHUTDOWN_TIMER = 0;
				
				HAL_GPIO_WritePin(power12V_GPIO_Port, power12V_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(powerUSBHub_GPIO_Port, powerUSBHub_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(headunitOutput_GPIO_Port, headunitOutput_Pin, GPIO_PIN_SET);
				
				accState = true;				
				if(firstRun){ //if the acc swith off to on and the pt2313 will turn on and must set values	
					
					TIM2->CCR1 = 0;
					HAL_Delay(3000);
					TIM2->CCR1 = pwmValue;
					HAL_GPIO_WritePin(standbySoundModuleAmpOutput_GPIO_Port, standbySoundModuleAmpOutput_Pin, GPIO_PIN_RESET);// must wait for noise gone
					HAL_Delay(2500);// must wait for noise gone
					//if after delay acc state changed to off dont set sound values 
					if(acciPinStateFunc()){
						if(HAL_I2C_IsDeviceReady(&hi2c1,soundModuleI2CAddress,10	,1000) == HAL_OK){
						HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,pt2313_buffer,8, i2c_timeout);
						}
					}
					HAL_Delay(500);
					HAL_GPIO_WritePin(muteOutput_GPIO_Port, muteOutput_Pin, GPIO_PIN_RESET); //unmute output
					if(acciPinStateFunc()){
						HAL_UART_Transmit_IT (&huart1, (uint8_t*)"RUN", 3);
						HAL_Delay(500);
						HAL_UART_Transmit_IT (&huart1, (uint8_t*)"RUN", 3);
					}
					
					firstRun = false;
				}
				
			}else
			{
				
				accState = false;
				firstRun = true;
				HAL_GPIO_WritePin(muteOutput_GPIO_Port, muteOutput_Pin, GPIO_PIN_SET);//mute output
				HAL_Delay(1000);
				MCU_SHUTDOWN_TIMER++;
				if(MCU_SHUTDOWN_TIMER > 3){
					__HAL_UART_DISABLE(&huart2);
					
					if(avCameraInputState || avTVinputState){
					avCameraInputState = false;
					avTVinputState = false;
					HAL_GPIO_WritePin(accRTDoutput_GPIO_Port, accRTDoutput_Pin, GPIO_PIN_SET);
					HAL_Delay(4000); //waiting for switch mode av to hdmi
					}
					HAL_GPIO_WritePin(standbySoundModuleAmpOutput_GPIO_Port, standbySoundModuleAmpOutput_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(antennaOutput_GPIO_Port, antennaOutput_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(amplifireOutput_GPIO_Port, amplifireOutput_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(powerUSBHub_GPIO_Port, powerUSBHub_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(power12V_GPIO_Port, power12V_Pin, GPIO_PIN_RESET);
					if(firstOFF){
						firstOFF = false;
						//HAL_UART_Transmit_IT (&huart1, (uint8_t*)"OFF", 3);    //<ijad tadakhol darsurate hamzamani daryaft az Rx>
					}
				
					if(MCU_SHUTDOWN_TIMER > 6 && MCU_SHUTDOWN_TIMER < 10){
						
						HAL_UART_Transmit_IT (&huart1, (uint8_t*)"OFF", 3);
					}
				
				
					if(MCU_SHUTDOWN_TIMER > 129600 ){ //after 36 hour headunit will off
						fanPermition = false;
						HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(headunitOutput_GPIO_Port, headunitOutput_Pin, GPIO_PIN_RESET);
						MCU_SHUTDOWN_TIMER = 0;
					}
				}	
			}

			if(accState){
				firstOFF = true;
				
				if( !reverseGearPinStateFunc() && !avTVinputState)
				{
					avCameraInputState = false;
					HAL_GPIO_WritePin(accRTDoutput_GPIO_Port, accRTDoutput_Pin, GPIO_PIN_SET);
					changeChannelDelay(1);
				}else{
					avCameraInputState = true;
					HAL_GPIO_WritePin(accRTDoutput_GPIO_Port, accRTDoutput_Pin, GPIO_PIN_RESET); // with 0 switch will be on and set on AV camera input
					changeChannelDelay(2);
				}
				
				if(avTVinputState){
					if(!reverseGearPinStateFunc() )
					{
						avCameraInputState = false;
						
						HAL_GPIO_WritePin(switchRTDoutput_GPIO_Port, switchRTDoutput_Pin, GPIO_PIN_SET);//Switch from camera to tv 
					}else{
						avCameraInputState = true;
						HAL_GPIO_WritePin(switchRTDoutput_GPIO_Port, switchRTDoutput_Pin, GPIO_PIN_RESET); //Switch from tv to camera	
					}
				}
				
				
				if(carLightLampPinState == GPIO_PIN_RESET){ //check car light state and change the brightness
					carLightState = true;
					//TIM2->CCR1 = pwmValueLightOn;
					TIM2->CCR1 = pwmValue;
				}else {
					carLightState = false;
					TIM2->CCR1 = pwmValue;
				}
				
			}
		
		
	

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(fanPermition){getARMTemp_SetFanState();}
			
		
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
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
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
	__HAL_RCC_I2C1_CLK_ENABLE();
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 110;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
	

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
  huart1.Init.BaudRate = 19200;
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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(antennaOutput_GPIO_Port, antennaOutput_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, standbySoundModuleAmpOutput_Pin|muteOutput_Pin|switchRTDoutput_Pin|fan_Pin 
                          |power12V_Pin|powerUSBHub_Pin|ledRedOutput_Pin|ledGreenOutput_Pin 
                          |amplifireOutput_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(headunitOutput_GPIO_Port, headunitOutput_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(accRTDoutput_GPIO_Port, accRTDoutput_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : antennaOutput_Pin */
  GPIO_InitStruct.Pin = antennaOutput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(antennaOutput_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : breakDetectInput_Pin lampDetectInput_Pin rearCameraInput_Pin accInput_Pin */
  GPIO_InitStruct.Pin = breakDetectInput_Pin|lampDetectInput_Pin|rearCameraInput_Pin|accInput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : standbySoundModuleAmpOutput_Pin muteOutput_Pin switchRTDoutput_Pin fan_Pin 
                           power12V_Pin powerUSBHub_Pin ledRedOutput_Pin ledGreenOutput_Pin 
                           amplifireOutput_Pin accRTDoutput_Pin */
  GPIO_InitStruct.Pin = standbySoundModuleAmpOutput_Pin|muteOutput_Pin|switchRTDoutput_Pin|fan_Pin 
                          |power12V_Pin|powerUSBHub_Pin|ledRedOutput_Pin|ledGreenOutput_Pin 
                          |amplifireOutput_Pin|accRTDoutput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : headunitOutput_Pin */
  GPIO_InitStruct.Pin = headunitOutput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(headunitOutput_GPIO_Port, &GPIO_InitStruct);

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
  while(1){}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
