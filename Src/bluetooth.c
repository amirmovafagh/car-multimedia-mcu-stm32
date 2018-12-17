#include "config.h"

uint8_t rx2_data;
uint8_t rx2_buffer[22];
int rx2_index=0;
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

extern uint8_t rx_buffer[32];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback2(UART_HandleTypeDef *huart){
		HAL_UART_Receive (&huart2, &rx2_data, 1, uart_timeout);
		if(huart->Instance == USART2)
		{    
			
			if(rx2_index==0)
			{
				for(int i=0; i<22; i++)
				{
					rx2_buffer[i]=0;
					
				}
				
			}
    
				//if the charcter received is other than'enter' ascii 0x3f, save the data in buffer
			if (rx2_data != 0x0A )
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
						
						HAL_UART_Transmit(&huart1, rx2_buffer, 22, uart_timeout);
						
					}
				if(rx2_index>=1){
					if(areEqual(rx2_buffer, rspnsReadyStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsReadyStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectingStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsConnectingStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectedStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsConnectedStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsInCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsInCallStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
						
						
					}
					
					
					
					if(areEqual(rx2_buffer, rspnsOnCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsOnCallStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					
					if(areEqual(rx2_buffer, rspnsOutCallStatus, 0, 3)){
						HAL_UART_Transmit(&huart1, rspnsOutCallStatus, 3, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsOutgoingCall, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsOutgoingCall, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicStart, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsMusicStart, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicResume, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsMusicResume, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicPause, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsMusicPause, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsMusicStop, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsMusicStop, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsEnterPairing, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsEnterPairing, 2, uart_timeout);
						rx2_index = 0;
						return;
					}
					
					if(areEqual(rx2_buffer, rspnsConnectToDevice, 0, 2)){
						HAL_UART_Transmit(&huart1, rspnsConnectToDevice, 2, uart_timeout);
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
		HAL_UART_Transmit(&huart2, "AT#CY\r", 6, uart_timeout);
		return;
		
	}else if(areEqual(reject, secondType, 0, 3)){			//reject
		HAL_UART_Transmit(&huart2, "AT#CF\r", 6, uart_timeout);
		return;
		
	}else if(areEqual(endCall, secondType, 0, 3)){		//end call
		HAL_UART_Transmit(&huart2, "AT#CG\r", 6, uart_timeout);
		return;
		
	}else if(areEqual(answer, secondType, 0, 3)){		//answer incoming call
		HAL_UART_Transmit(&huart2, "AT#CE\r", 6, uart_timeout);
		return;
		
	}else if(areEqual(redial, secondType, 0, 3)){			//redial
		HAL_UART_Transmit(&huart2, "AT#CH\r", 6, uart_timeout);
		return;
		
	}
	else if(areEqual(audioTransfer, secondType, 0, 3)){			//Audio transfer
		HAL_UART_Transmit(&huart2, "AT#CO\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(releaseHeldCall, secondType, 0,3)){  //release held call, reject waiting call
		HAL_UART_Transmit(&huart2, "AT#CQ\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(releaseActiveCall, secondType, 0,3)){  //release active call, accept other call
		HAL_UART_Transmit(&huart2, "AT#CR\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(holdActiveCall, secondType, 0,3)){  //hold active call, accept other call
		HAL_UART_Transmit(&huart2, "AT#CS\r", 6, uart_timeout);
		return;
	}
	else{//outgoing call
		
		for (int i=8; i<23; i++){
			secondType[i-8] = rx_buffer[i];
			
		}
		HAL_UART_Transmit(&huart2, "AT#CW", 5, uart_timeout);
		HAL_UART_Transmit(&huart2, secondType, 14, uart_timeout );
		HAL_UART_Transmit(&huart2, "\r", 1, uart_timeout);
		
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
		HAL_UART_Transmit(&huart2, "AT#VD\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(increaseVolume, secondType, 0, 3)){  //volume Up
		HAL_UART_Transmit(&huart2, "AT#VU\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(muteMic, secondType, 0, 3)){	//TOGGLE MIC
		HAL_UART_Transmit(&huart2, "AT#CM\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(playPause, secondType, 0, 3)){  //play , pause
		HAL_UART_Transmit(&huart2, "AT#MA\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(stop, secondType, 0, 3)){  //stop
		HAL_UART_Transmit(&huart2, "AT#MC\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(forward, secondType, 0, 3)){  //forward
		HAL_UART_Transmit(&huart2, "AT#MD\r", 6, uart_timeout);
		return;
	}
	else if(areEqual(backward, secondType, 0, 3)){ //backward
		HAL_UART_Transmit(&huart2, "AT#ME\r", 6, uart_timeout);
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
		HAL_UART_Transmit(&huart2, "AT#CA\r", 6, uart_timeout);
		
		return;
	}else if(areEqual(secondType, cancelPairing, 0,3)){
		HAL_UART_Transmit(&huart2, "AT#CB\r", 6, uart_timeout);
		HAL_UART_Transmit(&huart1, "Cancel pairing mode", 19, uart_timeout);
		return;
	}
	
	//enable and disable Auto connect mode
	if(areEqual(secondType, enableAutoConn, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MG\r", 6, uart_timeout);
		HAL_UART_Transmit(&huart1, "Enable Auto connect", 19, uart_timeout);
		return;
	}else if(areEqual(secondType, disableAutoConn, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MH\r", 6, uart_timeout);
		HAL_UART_Transmit(&huart1, "Disable Auto connect", 20, uart_timeout);
		return;
	}
	
	//enable and disable Auto Answer mode
	if(areEqual(secondType, enableAutoAnswer, 0,3)){
		
		HAL_UART_Transmit(&huart2, "AT#MP\r", 6, uart_timeout);
		HAL_UART_Transmit(&huart1, "Enable Auto Answer", 18, uart_timeout);
		return;
	}else if(areEqual(secondType, disableAutoAnswer, 0 , 3)){
		HAL_UART_Transmit(&huart2, "AT#MQ\r", 6, uart_timeout);
		HAL_UART_Transmit(&huart1, "Disable Auto Answer", 19, uart_timeout);
		return;
	}
	
	
}
