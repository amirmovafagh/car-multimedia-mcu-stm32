#include "config.h"

void checkRadio(){									//RADIO Module settings
	if(checkDeviceI2cConnection(RadioModuleI2CAddress)){
		uint8_t frequency[3]={'f','r','q'};
		uint8_t secondTypeRadio[4];
			
		for(int i=4; i<7; i++){
			secondTypeRadio[i-4]=rx_buffer[i];
		}
		
		if(areEqual(secondTypeRadio, frequency, 0, 3)){ //change frequency
			for(int i=8; i<12; i++){
				secondTypeRadio[i-8]=rx_buffer[i];
			}
			HAL_UART_Transmit (&huart1, "radiFrq", 7,10);
			frq2=arrayToInt(secondTypeRadio);
			frq2=frq2*100000;
			
			tea5767Setfrequency(frq2);
			HAL_UART_Transmit (&huart1, "okk frq", 3,100);

		}
	}else{ HAL_UART_Transmit (&huart1, "radProb", 7,10);
			
		}
	
}