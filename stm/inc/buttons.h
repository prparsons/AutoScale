#ifndef buttons_h_
#define buttons_h_

#include "main.h"

void buttonsInit(I2C_HandleTypeDef* _pHandle);
uint8_t getButton();
uint8_t convertButton(uint8_t uIndex);
char convertButtonChar(uint8_t uIndex);

#endif
