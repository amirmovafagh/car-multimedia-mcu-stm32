#include "config.h"
uint8_t senddata[5];
uint8_t bt[5];      

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
			frq2=arrayToInt(secondTypeRadio);
			frq2=frq2*100000;
			
			tea5767Setfrequency(frq2);
			if(debugState){
				HAL_UART_Transmit (&huart1, (uint8_t*)"set frq", 7, uart_timeout);
			}
			

		}
	}else{ 
			if(debugState){
				HAL_UART_Transmit (&huart1, (uint8_t*)"radProb", 7, uart_timeout);
			}
		}
	
}

void tea5767Setfrequency( uint32_t frequency )
{
  uint32_t pllValue;

  pllValue = (4 * (frequency + 225000)) / 32768;


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
		
	HAL_I2C_Master_Transmit(&hi2c1,RadioModuleI2CAddress,bt,5, i2c_timeout);	
	
}
