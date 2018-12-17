#include "config.h"

extern int ADC_buffer[2];
int ADC_result[2];
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
				ADC_counter = ADC_counter+1;
				ADC_result[0] =ADC_sum / ADC_counter;
			if(ADC_counter > 200){
				uint8_t ADC_buffer_send[4];
				for(int i=0; i<3000000 ; i++){}
				ADC_buffer_send[3] = (( ADC_result[0] / 1 ) % 10) + '0';
				ADC_buffer_send[2] = (( ADC_result[0] / 10 ) % 10) + '0';
				ADC_buffer_send[1] = (( ADC_result[0] / 100 ) % 10) + '0';
				ADC_buffer_send[0] = (( ADC_result[0] / 1000 ) % 10) + '0';
					HAL_IWDG_Refresh(&hiwdg);
					
				HAL_UART_Transmit (&huart1, ADC_buffer_send, 4, uart_timeout);
			}
			
			
			HAL_IWDG_Refresh(&hiwdg);
		}else {
			ADC_sum = 0;
			ADC_counter = 0;
			ADC_result[0] = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
}
