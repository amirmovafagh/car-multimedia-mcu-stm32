#include "config.h"

#define	  MAXFREQ   0X0870 //108mh    108*20=2160 hex equal with 0x0870
#define	  MINFREQ	  0X06D6		//87.5mh

#define FM_LEVEL    0x60	//status	//有效频点设置值
#define FM_USN      3
#define	FM_WAM      4
#define	IFCOUNT     3

void RadioInit(void);
void Write_String(uint8_t subaddr, int length);
void setFrequency (uint8_t mode ,uint16_t freq);
uint8_t  Search(void);
void PlayList(uint8_t i);
uint8_t CheckFreq(uint16_t freq);
uint8_t getIFCounter(void) ;
uint8_t getUSN(void);
uint8_t getWAM(void);
void softmute(void);

uint8_t displayCheck;
uint8_t subaddress = 0x20;
uint8_t testBuffer[16];
uint8_t readBuffer[8];
uint8_t   DATA_BUF[16]=
{   
	0x27,	// frequency Band at 5h bit 0AM , 1FM -bit 0 to 3 for frequence
	0x1A,	// continue of frequence bit Address with data byte 0h
	0x00,	//def
	0x80,	//noise, highpass filter, monoStereo, deEmphasis setting/ out gain
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x40, 0x04 
};
uint8_t   DATA_BUF2[16]=
{   
	0x27, 0xF8, 0x00, 0x80, 0x09, 0x4d, 0x99, 0x0e,
	0x00, 0x66, 0x15, 0xcd, 0xEE, 0x14, 0x30, 0x14 
};

uint32_t 	 CurrentFrequency =1520;
uint8_t MAXNO;
uint8_t getLevel(void);
uint8_t SDATA_BUF[64][2];
uint8_t displayCheck;
uint8_t devWriteRegister=0xC0;
		uint8_t secondTypeRadio[8];
uint8_t radioMode;
int frq;
				uint8_t CurrentFindedFrequency;
		 char buffer[16];



void checkRadio(){									//RADIO Module settings
	if(checkDeviceI2cConnection(RadioModuleI2CAddress)){
		//int mode = 0; //default FM
			//default frequence
		int outputGain = 0; //default off
		RadioInit();

		
		
		for(int i = 4; i < 12; i++){
			secondTypeRadio[i - 4] = rx_buffer[i];
		}
		radioMode = arrayToInt_withIndex_radioValues(secondTypeRadio, 0);
		frq = arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
		outputGain = arrayToInt_withIndex_radioValues(secondTypeRadio, 7);
		
		if(frq == 0){
			
			MAXNO = Search();
			PlayList(1);
			
		}else{
			if(radioMode == 0){
				frq = 2 * arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
				setFrequency(0x20, frq);
			}else{//mode 1 AM
				frq = arrayToInt_withIndex_radioValues(secondTypeRadio, 2);
				setFrequency(0x00, frq);
			}
			
		}

	}else{ 
			if(debugState){
				HAL_UART_Transmit (&huart1, (uint8_t*)"radProb", 7, uart_timeout);
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
	
	HAL_I2C_Mem_Write(&hi2c1, devWriteRegister,subaddr,1, DATA_BUF,length,10);
}

void setFrequency (uint8_t mode ,uint16_t freq) 
{	  
    freq &=0x0fff;
    if(freq<MINFREQ)
	  freq = MINFREQ;
	if(freq>MAXFREQ)
	  freq = MAXFREQ;
	DATA_BUF[1] =(uint8_t)  freq & 0x00ff; 
	DATA_BUF[0] = (uint8_t) ((freq>>8) | (1<< 5));
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
  setFrequency (0x20,freq) ;  
	
  	

	HAL_I2C_Master_Receive(&hi2c1,0xC1,readBuffer,5,10);
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
		  if(getIFCounter()> IFCOUNT)
	       return 0;
	      else
	       return 1;
		}

	  }
	  
   }
}



void PlayList(uint8_t i)
{  
	   if(i>MAXNO)
	   i=MAXNO;
	   else if(i<1)
	    i=1;
    CurrentFrequency = SDATA_BUF[i][0];
	
    CurrentFrequency = CurrentFrequency << 8;
    CurrentFrequency |= SDATA_BUF[i][1];
		 
		 
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
		 custom_delay(10);
		 //HAL_Delay(30);
			 HAL_IWDG_Refresh(&hiwdg);
		 if(CheckFreq(freq))
		  {
				char buffer[16];
				i++;
				SDATA_BUF[i][0]= freq >> 8;
				SDATA_BUF[i][1]= freq &0x00ff;
				
				PlayList(i);
				HAL_UART_Transmit(&huart1, (uint8_t*)buffer,sprintf(buffer, "rad-%d", CurrentFrequency),100);

		  }
		  freq +=1;
	}
  
  return i;
}

