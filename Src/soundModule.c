#include "config.h"

void checkMode(){
	if(checkDeviceI2cConnection(soundModuleI2CAddress)){
		HAL_GPIO_WritePin(muteOutput_GPIO_Port, muteOutput_Pin, GPIO_PIN_RESET);
//		static uint8_t aux[3]={'a','u','x'};
		static uint8_t brightness[3] ={'b','r','g'};
		uint8_t secondType[3];
	
		for(int i=4; i<7; i++){
			secondType[i-4]=rx_buffer[i];
		}
		//HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);  // mute befor change mode
		if(areEqual(secondType, brightness, 0, 3)){
			for(int i=8; i<12; i++){
					secondType[i-8]=rx_buffer[i];
				}
			int i= arrayToInt(secondType);
			if(carLightState)
				i = pwmValueLightOn = 70+i;
			else
				i = pwmValue = 70 + i;
			TIM2->CCR1 = i ;
			return;
			
		}else{
			uint8_t secondTypeAUDIO_mode[2];
		
		for(int i=4; i<6 ; i++){
			secondTypeAUDIO_mode[i-4]=rx_buffer[i];
		}
			pt2313_buffer[5]=arrayToInt_withIndex_soundValues(secondTypeAUDIO_mode, 0);	
		if(radioAntenaState){
			if(pt2313_buffer[5]!=90 && pt2313_buffer[5]!=94 && pt2313_buffer[5]!=86 && pt2313_buffer[5]!=78 && pt2313_buffer[5]!=70 && pt2313_buffer[5]!=66 &&
				pt2313_buffer[5]!=74 && pt2313_buffer[5]!=82){// check radio state for disable radio Antenna
				HAL_GPIO_WritePin(antennaOutput_GPIO_Port, antennaOutput_Pin, GPIO_PIN_RESET);
				radioAntenaState = false;
			}
		}
			
			HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,pt2313_buffer,8, i2c_timeout);
			if(debugState){
				HAL_UART_Transmit (&huart1, (uint8_t*)"mod", 3, uart_timeout);
			}
			return;
		}
		
		}else{ 
			if(debugState){
				HAL_UART_Transmit (&huart1, (uint8_t*)"modProb", 7, uart_timeout);
			}
		}
		
}

void checkAudio(){                         //Audio Module settings
	if(checkDeviceI2cConnection(soundModuleI2CAddress)){
		
		uint8_t secondTypeAUDIO[30];
		
		for(int i=4; i<30 ; i++){
			secondTypeAUDIO[i-4]=rx_buffer[i];
		}
		
		if(secondTypeAUDIO[10] == 0x00 && secondTypeAUDIO[7] == 0x00){
			pt2313_buffer[0] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 0); //volume
			for(int i = 0 ; i<30000 ; i++){}
			HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,pt2313_buffer,8, i2c_timeout);
				return;
		}
		pt2313_buffer[0] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 0); //volume
		pt2313_buffer[1] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 3); //speaker left front
		pt2313_buffer[2] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 7); //speaker right front
		pt2313_buffer[3] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 11); //speaker left rear
		pt2313_buffer[4] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 15); //speaker right rear
		pt2313_buffer[6] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 19); //change bas
		pt2313_buffer[7] = arrayToInt_withIndex_soundValues(secondTypeAUDIO, 23); //change treble
		for(int i = 0 ; i<30000 ; i++){}
		HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,pt2313_buffer,8, i2c_timeout);
		/*if(buffer[0]==63){
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);
			}else{
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);
			}*/
			
	}else HAL_UART_Transmit (&huart1, (uint8_t*)"audProb", 7, uart_timeout);
}
