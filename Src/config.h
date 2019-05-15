#include "main.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"

extern int uart_timeout;
extern int i2c_timeout;

extern bool radioAntenaState;
extern uint8_t rx_buffer[128];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;
extern uint16_t soundModuleI2CAddress;
extern uint16_t RadioModuleI2CAddress;
extern uint8_t pt2313_buffer[8];
extern uint32_t frq2;
extern int pwmValue;
extern int pwmValueLightOn;
extern bool carLightState;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t mode[3];
extern uint8_t audio[3];
extern uint8_t radio[3];
extern uint8_t bluetooth[3];
extern uint8_t debug[3];
extern bool accState;
extern bool debugState;
extern double Vsense;

extern void custom_delay(uint32_t milliseconds);
extern bool areEqual(uint8_t arr1[], uint8_t arr2[], int i, int n);
extern void checkMode(void);
extern void checkAudio(void);
extern void checkRadio(void);
extern void bluetoothSettings(void);
extern void bluetoothCall(void);
extern void bluetoothMusic(void);
extern void checkBluetooth(void);
extern void checkOtherCommands(void);
extern int arrayToInt(uint8_t mArr[]);
extern int arrayToInt_withIndex_soundValues(uint8_t mArr[],int index);
extern int arrayToInt_withIndex_radioValues(uint8_t mArr[],int index);
extern bool checkDeviceI2cConnection(uint16_t DevAddress);
