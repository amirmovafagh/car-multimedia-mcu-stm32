#include "config.h"

  	  

#define FM_LEVEL    0x60	//status	
#define FM_USN      3
#define	FM_WAM      4
#define	FM_IFCOUNT     3

#define AM_LEVEL    0xEF	//status	
#define	AM_WAM      	1
#define	AM_IFCOUNT     3

void RadioInit(void);
void Write_String(uint8_t subaddr, int length);
void setFrequency (uint8_t mode ,uint16_t freq);
uint8_t  Search(void);
void PlayList(uint8_t i);
uint8_t CheckFreq(uint16_t freq);
uint8_t getIFCounter(void);
uint8_t getUSN(void);
uint8_t getWAM(void);
void softmute(void);
void SendList(uint8_t i);


uint16_t MAXFREQ = 0X0870;		//108MHz    108*20=2160 hex equal with 0x0870
uint16_t MINFREQ = 0X06D6;		//87.5MHz
uint8_t displayCheck;
uint8_t subaddress = 0x20; //i2c subaddress
uint8_t testBuffer[16];
uint8_t readBuffer[8];
uint8_t   DATA_BUF[16]=
{   
	0x27,	// frequency Band at 5th bit 0AM , 1FM -bit 0 to 3 for frequence
	0x1A,	// continue of frequence bit Address with data byte 0h
	0x00,	//def
	0x80,	//noise, highpass filter, monoStereo, deEmphasis setting/ out gain
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x40, 0x04 
};
uint8_t   DATA_BUF_FM[16]=
{
//	0x27,0x08,0x00,0x00,0x09,0x4D,0x98,0x2E,	/* 0  - 7  */
//	0xCD,0x66,0x15,0xCD,0xEE,0x14,0x40,0x14,	/* 8  - 15 */
	0x26, 0xD8, 0x00, 0x80, 0x00, 0xCB, 0x99, 0x00,/* 0  - 7  */
	0x95, 0x66, 0x00, 0x94, 0xEE, 0x14, 0x40, 0x14 /* 8  - 15 */
//	0x26, 0xD6, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,/* 0  - 7  */
//	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x40, 0x04 /* 8  - 15 */
};

uint8_t   DATA_BUF_AM[16]=
{
	0x03, 0x21, 0x80, 0x24, 0x0E, 0x54, 0x04, 0x12,/* 0  - 7  */
	0x55, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x14 /* 8  - 15 */
};

uint32_t 	 CurrentFrequency =1520;
uint8_t MAXNO=1;
uint8_t getLevel(void);
uint8_t SDATA_BUF[256][2];
uint8_t displayCheck;
uint8_t devWriteRegister=0xC0;
uint8_t secondTypeRadio[8];
uint8_t radioMode = 1;  //0 AM , 1 FM
int outputGain = 0; //default off
int frq;
				uint8_t CurrentFindedFrequency;
		 char buffer[16];

void checkRadio(){									//RADIO Module settings
	if(checkDeviceI2cConnection(RadioModuleI2CAddress)){
			HAL_GPIO_WritePin(antennaOutput_GPIO_Port, antennaOutput_Pin, GPIO_PIN_SET);
			radioAntenaState = true;
		//int mode = 0; //default FM
		//default frequence

		for(int i = 4; i < 12; i++){
			secondTypeRadio[i - 4] = rx_buffer[i];
		}
		radioMode = arrayToInt_withIndex_radioValues(secondTypeRadio, 0);
		frq = arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
		outputGain = arrayToInt_withIndex_radioValues(secondTypeRadio, 7);
		if(outputGain==0){
			DATA_BUF[3]=0x80;
			DATA_BUF_FM[3]=0x80;
			DATA_BUF_AM[3]=0x80;
		}else	{
			DATA_BUF[3]=0x81;
			DATA_BUF_FM[3]=0x81;
			DATA_BUF_AM[3]=0x81;}

		
		if(frq == 0 && radioMode == 1){//Auto Search FM
			for(int i=0; i<256 ;i++){
				SDATA_BUF[i][0]=0;
				SDATA_BUF[i][1]=0;
			}
			for(int i=0;i<16;i++)
			{
				DATA_BUF[i] = DATA_BUF_FM[i];
			}
			DATA_BUF[0]=0x20; // data buffer mode section FM
			MAXFREQ = 0X0870;//108MHz
			MINFREQ = 0X06D6;//87.5MHz
			RadioInit();
			MAXNO = Search();
			PlayList(MAXNO);
			
		}else if(frq == 0 && radioMode == 0){//Auto Search AM
			// AM
			for(int i=0; i<256 ;i++){
				SDATA_BUF[i][0]=0;
				SDATA_BUF[i][1]=0;
			}
		for(int i=0;i<16;i++)
		{
			DATA_BUF[i] = DATA_BUF_AM[i];
		}
			//DATA_BUF[0]=0x00; // data buffer mode section AM
			MAXFREQ = 0X06B8;//1720MHz
			MINFREQ = 0X021C;//540MHz
			RadioInit();
			MAXNO = Search();
			PlayList(MAXNO);
		}
		else{
			if(radioMode == 1){//FM
				// FM
		for(int i=0;i<16;i++)
		{
			DATA_BUF[i] = DATA_BUF_FM[i];
		}	
				//DATA_BUF[0]=0x20; // data buffer mode section FM
				MAXFREQ = 0X0870;//108MHz
				MINFREQ = 0X06D6;//87.5MHz
				frq = 2 * arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
				setFrequency(0x20, frq);
				HAL_I2C_Master_Receive(&hi2c1,0xC1,readBuffer,4,10);
			}else{//mode 0 AM
				DATA_BUF[0]=0x00; // data buffer mode section AM
				DATA_BUF[1]=0x00;
				MAXFREQ = 0X06B8;//1720MHz
				MINFREQ = 0X021C;//540MHz
				frq = arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
				setFrequency(0x00, frq);
				HAL_I2C_Master_Receive(&hi2c1,0xC1,readBuffer,4,10);
			}
			
		}

	}else{ 
			if(1){
				HAL_UART_Transmit_IT (&huart1, (uint8_t*)"radProb", 7);
			}
		}
	
	
}

void RadioInit(){
	Write_String(subaddress,16) ;	 //,DATA_BUF
}
	
	void softmute(void)
{
  Write_String(0x40,1) ; 
}

void Write_String(uint8_t subaddr, int length)	   
{
	
	HAL_I2C_Master_Transmit(&hi2c1,devWriteRegister,&subaddr,1,10);
	
	HAL_I2C_Mem_Write(&hi2c1, devWriteRegister,subaddress,1, DATA_BUF,length,10);
}

void setFrequency (uint8_t mode ,uint16_t freq) 
{	  
    freq &=0x0fff;
    if(freq<MINFREQ)
	  freq = MINFREQ;
	if(freq>MAXFREQ)
	  freq = MAXFREQ;
	DATA_BUF[1] =(uint8_t)  freq & 0x00ff;
		if(radioMode == 0){
			DATA_BUF[0] = (uint8_t) ((freq>>8) | (0<< 5));
		}else DATA_BUF[0] = (uint8_t) ((freq>>8) | (1<< 5));
	
HAL_IWDG_Refresh(&hiwdg);	
	Write_String(mode,2) ; 
}

uint8_t getLevel(void)
{
  return readBuffer[1];	
}

uint8_t getIFCounter(void) 
{
  return readBuffer[3]&0X1F;
}
uint8_t getUSN(void)
{
 return (readBuffer[2]>>4);	
}
uint8_t getWAM(void)
{
  return readBuffer[2]&0X0F;
}

uint8_t CheckFreq(uint16_t freq)
{
	if(radioMode == 0){
			setFrequency (0x00,freq);
		}else setFrequency (0x20,freq) ;
    
	HAL_I2C_Master_Receive(&hi2c1,0xC1,readBuffer,4,10);
	if(radioMode == 0){
		if(getLevel() < AM_LEVEL)
			return 0;
		else
			{
				if(getWAM()> AM_WAM)
					return 0;
				else
					{
						if(getIFCounter()> AM_IFCOUNT)
							return 0;
						else
						return 1;
					}
			}
	}else{
		if(getLevel() < FM_LEVEL)
			return 0;
		else
			{
				if(getUSN() > FM_USN)
					return 0; 
				else
					{
						if(getWAM()> FM_WAM)
							return 0;
						else
							{
								if(getIFCounter()> FM_IFCOUNT)
									return 0;
								else
								return 1;
							}
					}
			}
	}	
  
}

void SendList(uint8_t i)
{
	CurrentFrequency = SDATA_BUF[i][0];
	CurrentFrequency = CurrentFrequency << 8;
	CurrentFrequency |= SDATA_BUF[i][1];
	
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer,sprintf(buffer, "rad-%d", CurrentFrequency),uart_timeout);
	custom_delay(200);
	
}

void PlayList(uint8_t i)
{  
	   /*if(i>MAXNO)
	   i=MAXNO;
	   else if(i<1)
	    i=1;*/
    CurrentFrequency = SDATA_BUF[i][0];
	
    CurrentFrequency = CurrentFrequency << 8;
    CurrentFrequency |= SDATA_BUF[i][1];
		 
		 //HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer,sprintf(buffer, "rad-%d", CurrentFrequency));
		setFrequency (subaddress,CurrentFrequency);
}
uint8_t  Search(void)
{
	uint8_t i=0;
	uint16_t freq = MINFREQ;
  while(freq <=MAXFREQ)
		{
			displayCheck = freq;		 
			//for(int v = 0 ; v<200000; v++){}
			custom_delay(25);
			//HAL_Delay(30);
			HAL_IWDG_Refresh(&hiwdg);
			if(CheckFreq(freq))
				{
					i++;
					if ( i < 255 ){
						SDATA_BUF[i][0]= freq >> 8;
						SDATA_BUF[i][1]= freq &0x00ff;
						SendList(i);
					}
					//HAL_UART_Transmit(&huart1, (uint8_t*)buffer,sprintf(buffer, "rad-%d", CurrentFrequency),10);
				}
		  freq +=1;
		}
		return i;
}

