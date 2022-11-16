#include "loadCell.h"


// Grams is in milligrams
// Grains is in decigrains
float fCalibrationGrams = 0.07698;
float fCalibrationGrains = 0.01185;
int32_t iTareGrams = 9430;
int32_t iTareGrains = 1449;
int32_t iAverage = 0;
uint32_t uData;

const char arcHex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
char csMg[9];
char csGrn[8];
char csHex[7];

bool readCell()
{
   uint8_t uBit;
   if (!HAL_GPIO_ReadPin(CELL_DAT_GPIO_Port, CELL_DAT_Pin)) { // Wait until it's ready pin is high when not ready)
      uData = 0;
      for (int8_t i = 23; i >= 0; i--) {
         HAL_GPIO_WritePin(CELL_CLK_GPIO_Port, CELL_CLK_Pin, GPIO_PIN_SET);
         timerDelay(2);//16);
         uBit = HAL_GPIO_ReadPin(CELL_DAT_GPIO_Port, CELL_DAT_Pin);
         HAL_GPIO_WritePin(CELL_CLK_GPIO_Port, CELL_CLK_Pin, GPIO_PIN_RESET);
         if (uBit) {
            uData |= 1 << i;
         }
      }
      // 25th pulse finishes
      HAL_GPIO_WritePin(CELL_CLK_GPIO_Port, CELL_CLK_Pin, GPIO_PIN_SET);
      timerDelay(2);//16);
      HAL_GPIO_WritePin(CELL_CLK_GPIO_Port, CELL_CLK_Pin, GPIO_PIN_RESET);

      return TRUE;
   }
   return FALSE;
}

void setTare()
{
    float fTemp = iAverage * fCalibrationGrams;
    iTareGrams = fTemp;
    fTemp = iAverage * fCalibrationGrains;
    iTareGrains = fTemp;
}

void calibrateGrams(int iGrams)
{
    // We're measuring milligrams, so:
    // iGrams * 1000 = (iAverage * fCalibrationGrams) - iTareGrams
    // fCalibration = (iTare + iGrams(1000)) / iAverage
    fCalibrationGrams = (iTareGrams + (iGrams * 1000.0f)) / ((float)iAverage);
}

void calibrateGrains(int iGrains)
{
    // We're measuring in...deci-grains (display in tenths of a grain)
    // iGrains * 10 = (iAverage * fCalibrationGrains) - iTareGrains
    fCalibrationGrains = (iTareGrains + (iGrains * 10.0f)) / ((float)iAverage);
}

void calculateAverage()
{
   //iAverage = (uData * .2) + (iAverage * 0.8);
   filterInsert(uData);
   iAverage = _filterGet(20);
}

char* createStringHex()
{
   csHex[6] = 0;
   int iTemp = uData;
   int i = 5;
   while (iTemp > 0) {
      int iRemainder = iTemp % 16;
      csHex[i--] = arcHex[iRemainder];
      iTemp /= 16;
   }

   while (i >= 0) csHex[i--] = '0';

   return csHex;
}

char* createString(Scale_Mode mode)
{
   if (mode == SCALE_MODE_GRAM) {
      return createStringGrams();
   }// else if (mode == SCALE_MODE_GRAIN)
   return createStringGrains();
}

char* createStringGrams()
{
   // 6 digits, 3 before and 3 after decimal
   float fTemp = ((float)iAverage) * fCalibrationGrams;
   uint32_t uTemp;
   if (fTemp - iTareGrams < 0) {
      csMg[0] = '-';
      int iTempSigned = fTemp;
      iTempSigned -= iTareGrams;
      iTempSigned *= -1;
      uTemp = iTempSigned;
   } else {
      csMg[0] = '+';
      uTemp = fTemp;
      uTemp -= iTareGrams; // Rough Tare (need signed numbers)
   }

   int i = 7;
   for (;uTemp > 0; i--) {
      if (i == 4) {
         // decimal
         continue;
      }
      int iRemainder = uTemp % 10;
      csMg[i] = arcHex[iRemainder];
      if (csMg[i] == 0) {
         csMg[i] = '0';
      }

      uTemp /= 10;
   }
   while (i >= 1) csMg[i--] = '0';
   csMg[4] = '.';
   return csMg;
}

char* createStringGrains()
{
   float fTemp = ((float)iAverage) * fCalibrationGrains;
   uint32_t uTemp;
   if (fTemp - ((float)iTareGrains) < 0) {
      csGrn[0] = '-';
      int iTempSigned = fTemp;
      iTempSigned -= iTareGrains;
      iTempSigned *= -1;
      uTemp = iTempSigned;
   } else {
      csGrn[0] = '+';
      uTemp = fTemp;
      uTemp -= iTareGrains;
   }

   int i = 6;
   for (;uTemp > 0; i--) {
      if (i == 5) {
         // decimal point
         continue;
      }
      int iRemainder = uTemp % 10;
      csGrn[i] = arcHex[iRemainder];
      if (csGrn[i] == 0) {
         csGrn[i] = '0';
      }

      uTemp /= 10;
   }
   while (i >= 1) csGrn[i--] = '0';
   csGrn[5] = '.';
   return csGrn;
}

float getGrains()
{
   return ((((float)iAverage) * fCalibrationGrains) - ((float)iTareGrains)) * .1;
}

float getGrams()
{
   return ((((float)iAverage) * fCalibrationGrams) - ((float)iTareGrams)) * .001;
}

struct filterList {
    uint32_t u[256];
    unsigned char index;

    uint32_t uFastAvg;
} list;

void filterInit()
{
    list.index = 0;
    for (int i = 0; i < 256; i++) {
        list.u[i] = 0;
    }
    list.uFastAvg = 0;
}

void filterInsert(uint32_t u)
{
   list.u[++list.index] = u;
   list.uFastAvg = (list.uFastAvg * .8) + (u * .2);
}

float _filterGet(unsigned char uSamples)
{
    unsigned char u = list.index;
    unsigned char uFinish = u - uSamples;
    uint32_t uAvg = 0;
    do {
       uAvg += list.u[u--];
    } while (u != uFinish);
    return uAvg / uSamples;
}

float filterGet()
{
   return _filterGet(10);
}

float filterGetFast()
{
   return list.uFastAvg;
}
