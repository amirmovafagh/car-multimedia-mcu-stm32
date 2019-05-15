#include "config.h"

#define MVM_X33 1300
#define otherCars 3500

extern int ADC_buffer[2];
int ADC_result[1]; //ADC_result[0]=> controllWheel data ,ADC_result[1]=>internal Temp sensor
int ADC_average=0;
int ADC_sum=0;
int ADC_counter=0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	if(hadc->Instance == ADC1){
		
		ADC_result[0] = ADC_buffer[0];
		Vsense = ADC_buffer[1];
		
		//controll Wheel get and send data
		if(ADC_result[0] < otherCars && accState){
				ADC_sum += ADC_result[0];
				ADC_counter = ADC_counter+1;
				ADC_result[0] =ADC_sum / ADC_counter;
			if(ADC_counter > 150){
				uint8_t ADC_buffer_send[4];
				
				ADC_buffer_send[3] = (( ADC_result[0] / 1 ) % 10) + '0';
				ADC_buffer_send[2] = (( ADC_result[0] / 10 ) % 10) + '0';
				ADC_buffer_send[1] = (( ADC_result[0] / 100 ) % 10) + '0';
				ADC_buffer_send[0] = (( ADC_result[0] / 1000 ) % 10) + '0';
				HAL_IWDG_Refresh(&hiwdg);
					
				HAL_UART_Transmit (&huart1, ADC_buffer_send, 4,uart_timeout);
				custom_delay(300);
			}
			
			
			HAL_IWDG_Refresh(&hiwdg);
		}else {
			ADC_sum = 0;
			ADC_counter = 0;
			//ADC_result[0] = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
		
		
		
		
	}
}
