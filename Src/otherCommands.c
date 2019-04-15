#include "config.h"

extern bool avTVinputState;
extern bool headUnitCPU_HighTemp;

void checkOtherCommands(){
	static uint8_t temperature[3] ={'t','m','p'};
	static uint8_t brightness[3] ={'b','r','g'};
	static uint8_t channelSwitch[3] ={'c','n','l'};
	static uint8_t amplifire[3] ={'a','m','p'};
	uint8_t secondType[3];

	for(int i=4; i<7; i++){
		secondType[i-4]=rx_buffer[i];
	}
	
	if(areEqual(secondType, brightness, 0, 3)){//change brightness
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
	}
	
	if(areEqual(secondType, temperature, 0, 3)){ //headUnit temp fan state
		for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
		int j = arrayToInt(secondType);
		if (j == 1){
			headUnitCPU_HighTemp = true;
			//HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_SET);
			return;
		}else{
			headUnitCPU_HighTemp = false;
			//HAL_GPIO_WritePin(fan_GPIO_Port,fan_Pin, GPIO_PIN_RESET);
			return;
		}
	}
	
	if(areEqual(secondType, amplifire, 0, 3)){ //amplifire on & off
		for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
		int b = arrayToInt(secondType);
		if (b == 1){
			
			HAL_GPIO_WritePin(amplifireOutput_GPIO_Port, amplifireOutput_Pin, GPIO_PIN_SET);
			return;
		}else{
			
			HAL_GPIO_WritePin(amplifireOutput_GPIO_Port, amplifireOutput_Pin, GPIO_PIN_RESET);
			return;
		}
	}
	
	if(areEqual(secondType, channelSwitch, 0, 3)){// change hdmi to the TV_AV
		for(int i=8; i<12; i++){
				secondType[i-8]=rx_buffer[i];
			}
		int c = arrayToInt(secondType);
		if (c == 1){
			avTVinputState = true;
			HAL_GPIO_WritePin(accRTDoutput_GPIO_Port, accRTDoutput_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(switchRTDoutput_GPIO_Port, switchRTDoutput_Pin, GPIO_PIN_SET);// set On Tv
			return;
		}else{
			HAL_GPIO_WritePin(switchRTDoutput_GPIO_Port, switchRTDoutput_Pin, GPIO_PIN_RESET);// back to hdmi
			avTVinputState = false;
			return;
			
		}
	}
	
}
