#include "buttons.h"

const uint8_t uButton_Numbers[] = { 1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 255, 0, 255, 255 };
const char cButton_Char[] = { '1', '2', '3', 'a', '4', '5', '6', 'b', '7', '8', '9', 'c', 'g', '0', 'r', 'd' };
const uint8_t I2C_ADDRESS_BUTTONS = 0x46 << 1;
I2C_HandleTypeDef* pHandle;
uint8_t RxBuffer;

void buttonsInit(I2C_HandleTypeDef* _pHandle)
{
   pHandle = _pHandle;
}

uint8_t getButton()
{
   RxBuffer = 255;
   HAL_I2C_Master_Transmit(pHandle, I2C_ADDRESS_BUTTONS, &RxBuffer, 1, 10000);
   HAL_I2C_Master_Receive(pHandle, I2C_ADDRESS_BUTTONS, &RxBuffer, 1, 10000);
   return RxBuffer;
   /*
   while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
     {
       / * Error_Handler() function is called when Timeout error occurs.
          When Acknowledge failure occurs (Slave don't acknowledge it's address)
          Master restarts communication * /
       if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
       {
         Error_Handler();
       }
     }*/
}

uint8_t convertButton(uint8_t uIndex)
{
   if (uIndex > 15) return 255;
   return uButton_Numbers[uIndex];
}

char convertButtonChar(uint8_t uIndex)
{
   if (uIndex > 15) return 255;
   return cButton_Char[uIndex];
}
