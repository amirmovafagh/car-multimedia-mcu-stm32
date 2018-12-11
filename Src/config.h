#include "main.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"

extern uint8_t rx_buffer[32];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;
extern uint16_t soundModuleI2CAddress;
extern uint16_t RadioModuleI2CAddress;
extern uint8_t buffer[8];
extern uint32_t frq2;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t mode[3];
extern uint8_t audio[3];
extern uint8_t radio[3];
extern uint8_t bluetooth[3];


extern bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n);
extern void checkMode(void);
extern void checkAudio(void);
extern void checkRadio(void);
extern void bluetoothSettings(void);
extern void bluetoothCall(void);
extern void bluetoothMusic(void);
extern void checkBluetooth(void);
extern int arrayToInt(uint8_t mArr[]);
int arrayToInt_withIndex(uint8_t mArr[],int index);
extern bool checkDeviceI2cConnection(uint16_t DevAddress);
extern void tea5767Setfrequency( uint32_t frequency );
