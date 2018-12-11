#include "config.h"

void checkMode(){
	if(checkDeviceI2cConnection(soundModuleI2CAddress)){
		HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);
		static uint8_t aux[3]={'a','u','x'};
		static uint8_t brightness[3] ={'b','r','g'};
		uint8_t secondType[3];
	
		for(int i=4; i<7; i++){
			secondType[i-4]=rx_buffer[i];
		}
		//HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);  // mute befor change mode
		if(areEqual(secondType, radio, 0, 3)){
		
			HAL_UART_Transmit (&huart1, secondType, 3,100);
		
			frq2=94000000;
			tea5767Setfrequency(frq2);
			buffer[5]=66;	
			HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,buffer,7,2);
			return;
		}else if(areEqual(secondType, aux, 0, 3)){
			buffer[5]=64;	
			HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,buffer,7,2);
			HAL_UART_Transmit (&huart1, "AUX", 3,100);
			return;
		}
		else if (areEqual(secondType, brightness, 0, 3)){ //Brightness
			for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
			int i= arrayToInt(secondType);
    	TIM2->CCR1 = 70 + i;
			return;
		}
		else{
			buffer[5]=65;	
			HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,buffer,7,2);
			HAL_UART_Transmit (&huart1, "pin", 3,100);
			return;
		}
		
		}else{ HAL_UART_Transmit (&huart1, "modProb", 7,10);
			
		}
		
}

void checkAudio(){                         //Audio Module settings
	if(checkDeviceI2cConnection(0x44<<1)){
		
		uint8_t secondTypeAUDIO[30];
		
		for(int i=4; i<30 ; i++){
			secondTypeAUDIO[i-4]=rx_buffer[i];
		}
		
		if(secondTypeAUDIO[10] == 0x00 && secondTypeAUDIO[7] == 0x00){
			buffer[0] = arrayToInt_withIndex(secondTypeAUDIO, 0); //volume
			for(int i = 0 ; i<30000 ; i++){}
			HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,buffer,7,2);
				return;
		}
		buffer[0] = arrayToInt_withIndex(secondTypeAUDIO, 0); //volume
		buffer[1] = arrayToInt_withIndex(secondTypeAUDIO, 3); //speaker left front
		buffer[2] = arrayToInt_withIndex(secondTypeAUDIO, 7); //speaker right front
		buffer[3] = arrayToInt_withIndex(secondTypeAUDIO, 11); //speaker left rear
		buffer[4] = arrayToInt_withIndex(secondTypeAUDIO, 15); //speaker right rear
		buffer[6] = arrayToInt_withIndex(secondTypeAUDIO, 19); //change bas
		buffer[7] = arrayToInt_withIndex(secondTypeAUDIO, 23); //change treble
		for(int i = 0 ; i<30000 ; i++){}
		HAL_I2C_Master_Transmit(&hi2c1,soundModuleI2CAddress,buffer,7,2);
		/*if(buffer[0]==63){
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 1);
			}else{
				HAL_GPIO_WritePin(muto_GPIO_Port,muto_Pin, 0);
			}*/
			
	}else HAL_UART_Transmit (&huart1, "audProb", 7,10);
  
 
}