#include "display.h"

#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "DEV_Config.h"
#include "LCD_2inch4.h"
#include "GUI_Paint.h"
#include "fonts.h"

DisplayLine arDisplayList[LCD_LINES];
int iMenuIndexPrev = 0;

void displayInit_LCD(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim)
{
   for (int i = 0; i < LCD_LINES; i++) {
      arDisplayList[i].type = LINE_TYPE_TEXT;
      arDisplayList[i].scText = 0;
      arDisplayList[i].indent = 0;
   }

   DEV_Module_Init(hspi, htim);
   LCD_2IN4_Init();
   LCD_2IN4_Clear(WHITE);

   Paint_NewImage(LCD_2IN4_WIDTH,LCD_2IN4_HEIGHT, ROTATE_0, WHITE);
   Paint_SetClearFuntion(LCD_2IN4_Clear);
   Paint_SetDisplayFuntion(LCD_2IN4_DrawPaint);
   Paint_Clear(WHITE);

   DEV_Delay_ms(1000);
   Paint_SetRotate(ROTATE_0);
   //Paint_DrawString_EN (5, 10, "DEMO:",        &Font12,    YELLOW,  RED);
   //Paint_DrawString_EN (5, 34, "Hello World",  &Font16,    BLUE,    CYAN);
}

void displaySetLine_LCD(DisplayLine* pLine, int iLineNum)
{
   bool bOldText = arDisplayList[iLineNum].scText != nullptr;
   bool bNewText = pLine->scText != nullptr;

   if (bOldText && bNewText) {
      int iLenOld = strlen(arDisplayList[iLineNum].scText);
      int iLenNew = strlen(pLine->scText);
      int iMin = iLenOld < iLenNew ? iLenOld : iLenNew;
      for (int i = 0; i < iMin; i++) {
         if (arDisplayList[iLineNum].scText[i] == pLine->scText[i]) {
            arDisplayList[iLineNum].scText[i] = ' ';
         }
      }
   }

   if (bOldText) {
      displayEraseLine_LCD(iLineNum);
      free(arDisplayList[iLineNum].scText);
   }

   if (pLine->scText != nullptr) {
      int length = strlen(pLine->scText);
      arDisplayList[iLineNum].scText = malloc(length + 1);
      strcpy(arDisplayList[iLineNum].scText, pLine->scText);
      arDisplayList[iLineNum].scText[length] = 0;
   } else {
      arDisplayList[iLineNum].scText = nullptr;
   }

   arDisplayList[iLineNum].type = pLine->type;
   arDisplayList[iLineNum].indent = pLine->indent;

   displayDrawLine_LCD(iLineNum);
}

void displayEraseLine_LCD(int iLineNum)
{
   sFONT* pFont = &Font16;
   switch (arDisplayList[iLineNum].type) {
      case LINE_TYPE_TEXT:
      case LINE_TYPE_HLTEXT:
      case LINE_TYPE_SELTEXT:
      case LINE_TYPE_HLSELTEXT: return;
      case LINE_TYPE_STATIC:
      case LINE_TYPE_BUTTON: break;
      case LINE_TYPE_HEADER: pFont = &Font24; break;
   }
   int iY = iLineNum * 30 + 10;
   int iX = (arDisplayList[iLineNum].indent * 10) + 5;
   int colorHighlight = WHITE;
   Paint_DrawString_EN(iX, iY, arDisplayList[iLineNum].scText, pFont, colorHighlight, colorHighlight);
}

void displayDrawLine_LCD(int iLineNum)
{
   int iY = iLineNum * 30 + 10;
   int iX = (arDisplayList[iLineNum].indent * 10) + 5;
   int colorText = BLACK;
   int colorHighlight = WHITE;
   sFONT* pFont = &Font16;
   switch (arDisplayList[iLineNum].type) {
      case LINE_TYPE_TEXT: colorHighlight = LGRAY; break;
      case LINE_TYPE_HLTEXT: colorHighlight = LIGHTBLUE; break;
      case LINE_TYPE_SELTEXT: colorHighlight = GRAY; break;
      case LINE_TYPE_HLSELTEXT: colorHighlight = BLUE; break;
      case LINE_TYPE_STATIC:
      case LINE_TYPE_BUTTON: break;
      case LINE_TYPE_HEADER: pFont = &Font24; break;
   }
   Paint_DrawString_EN(iX, iY, arDisplayList[iLineNum].scText, pFont, colorHighlight, colorText);
}

void displaySwitchMode(MachineState* pState, Display_Mode modeFrom)
{
   //**right now we're just drawing things, not handling from

   iMenuIndexPrev = 0;
   DisplayLine line;
   int i = pState->displayMode;//avoiding warning
   switch (i) {
      case MAIN_MENU: {
         line.indent = 0;
         line.scText = "";
         line.type = LINE_TYPE_TEXT;
         displaySetLine_LCD(&line, 1);
         displaySetLine_LCD(&line, 2);

         displayPrintMain(pState, 0, LINE_TYPE_SELTEXT);
         displayPrintMain(pState, 1, LINE_TYPE_TEXT);
         displayPrintMain(pState, 2, LINE_TYPE_TEXT);
         displayPrintMain(pState, 3, LINE_TYPE_TEXT);
      } break;

      case SETTINGS: {
         pState->iMenuSelection = 0;
         displayPrintSettings(pState, 0, LINE_TYPE_SELTEXT);
         displayPrintSettings(pState, 1, LINE_TYPE_TEXT);
         displayPrintSettings(pState, 2, LINE_TYPE_TEXT);
         displayPrintSettings(pState, 3, LINE_TYPE_TEXT);
      } break;

      case LOAD_PROGRAM: {
         pState->iMenuSelection = 0;
         displayNewProgram(pState, 0, LINE_TYPE_SELTEXT);
         displayNewProgram(pState, 1, LINE_TYPE_TEXT);
         displayNewProgram(pState, 2, LINE_TYPE_TEXT);
         displayNewProgram(pState, 3, LINE_TYPE_TEXT);
         displayNewProgram(pState, 4, LINE_TYPE_TEXT);
         displayNewProgram(pState, 5, LINE_TYPE_TEXT);
      } break;
   }
}

void displayUpdate_(MachineState* pState)
{
   DisplayLine line;
   switch (pState->displayMode) {
      case MAIN_MENU: {
         // First update the weight display
         line.indent = 0;
         line.scText = malloc(32);
         if (pState->scaleMode == SCALE_MODE_GRAM) sprintf(line.scText, "%3.3f grams", pState->fGramsCurrent);
         else sprintf(line.scText, "%3.1f grains", pState->fGrainsCurrent);
         line.type = LINE_TYPE_HEADER;
         displaySetLine_LCD(&line, 0);
         // Now look to see if there's been any change in menu selection
         if (pState->iMenuSelection != iMenuIndexPrev) {
            displayPrintMain(pState, iMenuIndexPrev, LINE_TYPE_TEXT);
            iMenuIndexPrev = pState->iMenuSelection;
            displayPrintMain(pState, iMenuIndexPrev, LINE_TYPE_SELTEXT);
         }
         free(line.scText);
      } break;

      case LOAD_PROGRAM: {
         if (pState->iMenuSelection != iMenuIndexPrev) {
            // First, change the prev selection to normal
            displayNewProgram(pState, iMenuIndexPrev, LINE_TYPE_TEXT);
            iMenuIndexPrev = pState->iMenuSelection;
            displayNewProgram(pState, iMenuIndexPrev, LINE_TYPE_SELTEXT);
         }
      } break;

      case SETTINGS: {
         if (pState->iMenuSelection != iMenuIndexPrev) {
            // First, change the prev selection to normal
            displayPrintSettings(pState, iMenuIndexPrev, LINE_TYPE_TEXT);
            iMenuIndexPrev = pState->iMenuSelection;
            displayPrintSettings(pState, iMenuIndexPrev, LINE_TYPE_SELTEXT);
         }
      } break;

      default: break;
   }
}

void displayHandleMenu(MachineState* pState, uint8_t uButton, char cButton)
{
   switch (pState->displayMode) {
      case MAIN_MENU: {
         switch (uButton) {
            case 2:
               if (--(pState->iMenuSelection) < 0) pState->iMenuSelection = 0;
               break;
            case 8:
               if (++(pState->iMenuSelection) > 3) pState->iMenuSelection = 3;
               break;
            case 4: {
               int8_t* pi = &(CommandQueue.iStep[0]);
               (*pi)++;
               CommandQueue.Q_Step[0][*pi].uCommand = STEP_CHAMBER_ENABLE;
               (*pi)++;
               CommandQueue.Q_Step[0][*pi].uCommand = STEP_CHAMBER_DISABLE;
               pi = &(CommandQueue.iStep[1]);
               (*pi)++;
               CommandQueue.Q_Step[1][*pi].uCommand = STEP_CHAMBER_DECREASE;
            } break;
            case 6: {
               int8_t* pi = &(CommandQueue.iStep[0]);
               (*pi)++;
               CommandQueue.Q_Step[0][*pi].uCommand = STEP_CHAMBER_ENABLE;
               (*pi)++;
               CommandQueue.Q_Step[0][*pi].uCommand = STEP_CHAMBER_DISABLE;
               pi = &(CommandQueue.iStep[1]);
               (*pi)++;
               CommandQueue.Q_Step[1][*pi].uCommand = STEP_CHAMBER_INCREASE;
            } break;
            case 5:
               if (pState->iMenuSelection == 0) {
                  pState->displayMode = LOAD_PROGRAM;
                  displaySwitchMode(pState, MAIN_MENU);
               } else if (pState->iMenuSelection == 3) {
                  pState->displayMode = SETTINGS;
                  displaySwitchMode(pState, MAIN_MENU);
               }
               break;
         }
      } break;

      case SETTINGS: {
         // First get what thing is selected and set an increment for it
         int16_t* pSetting = &(pState->iMeasureDumpDwell);
         int16_t iIncrement = 100;
         int16_t iMin = 100;
         int16_t iMax = 2000;
         switch (uButton) {
            case 2:
               if (--(pState->iMenuSelection) < 0) pState->iMenuSelection = 0;
               break;
            case 8:
               if (++(pState->iMenuSelection) > 3) pState->iMenuSelection = 3;
               break;
            case 4:
               *pSetting -= iIncrement;
               if (*pSetting < iMin) *pSetting = iMin;
               displayPrintSettings(pState, pState->iMenuSelection, LINE_TYPE_SELTEXT);
               break;
            case 6:
               *pSetting += iIncrement;
               if (*pSetting > iMax) *pSetting = iMax;
               displayPrintSettings(pState, pState->iMenuSelection, LINE_TYPE_SELTEXT);
               break;
         }
      } break;

      case LOAD_PROGRAM: {
         if (pState->bMenuHighlight) {
            if (cButton == 'g') {
               pState->bMenuHighlight = FALSE;
               displayNewProgram(pState, pState->iMenuSelection, LINE_TYPE_SELTEXT);
               break;
            }
            switch (pState->iMenuSelection) {
               case 0:
               case 3: {// 3.2f
                  int iValue;
                  if (pState->iMenuSelection == 0) iValue = pState->fTargetWeight * 100;
                  else iValue = pState->fGramPerCC * 100;

                  iValue = iValue % 10000;
                  iValue *= 10;
                  iValue += uButton;

                  if (pState->iMenuSelection == 0) pState->fTargetWeight = iValue / 100.0;
                  else pState->fGramPerCC = iValue / 100.0;
                  displayNewProgram(pState, pState->iMenuSelection, LINE_TYPE_HLTEXT);
               } break;
               case 2: // 2d
                  pState->iTricklePercent = pState->iTricklePercent % 10;
                  pState->iTricklePercent *= 10;
                  pState->iTricklePercent += uButton;
                  displayNewProgram(pState, pState->iMenuSelection, LINE_TYPE_HLTEXT);
                  break;
               case 1:
               case 4:
               case 5: {// 4d
                  int16_t iValue;
                  if (pState->iMenuSelection == 1) iValue = pState->iMeasureDumpDwell;
                  else if (pState->iMenuSelection == 4) iValue = pState->iTrickle1RPMfast;
                  else iValue = pState->iTrcikle2RPMslow;

                  iValue = iValue % 1000;
                  iValue *= 10;
                  iValue += uButton;

                  if (pState->iMenuSelection == 1) pState->iMeasureDumpDwell = iValue;
                  else if (pState->iMenuSelection == 4) pState->iTrickle1RPMfast = iValue;
                  else pState->iTrcikle2RPMslow = iValue;
                  displayNewProgram(pState, pState->iMenuSelection, LINE_TYPE_HLTEXT);
               } break;
            }
         } else {
            switch (uButton) {
               case 2:
                  if (--(pState->iMenuSelection) < 0) pState->iMenuSelection = 0;
                  break;
               case 8:
                  if (++(pState->iMenuSelection) > 5) pState->iMenuSelection = 5;
                  break;
               case 5:
                  pState->bMenuHighlight = TRUE;
                  displayNewProgram(pState, pState->iMenuSelection, LINE_TYPE_HLTEXT);
                  break;
            }
         }
      } break;
      default:
         break;
   }
}

void displayPrintMain(MachineState* pState, int8_t iIndex, LineType type)
{
   DisplayLine line;
   line.scText = malloc(32);
   line.indent = 0;
   line.type = type;

   switch (iIndex) {
      case 0: strcpy(line.scText, "New Program"); break;
      case 1: strcpy(line.scText, "Run Program"); break;
      case 2: strcpy(line.scText, "Calibrate"); break;
      case 3: strcpy(line.scText, "Settings"); break;
   }
   displaySetLine_LCD(&line, iIndex + 3);
   free(line.scText);
}

void displayPrintSettings(MachineState* pState, int8_t iIndex, LineType type)
{
   DisplayLine line;
   line.scText = malloc(32);
   line.indent = 0;
   line.type = type;

   switch (iIndex) {
      case 0: sprintf(line.scText, "Dump Dwell ms %4d", pState->iMeasureDumpDwell); break;
      case 1: sprintf(line.scText, "Trickle1 rpm %4d", pState->iTrickle1RPMfast); break;
      case 2: sprintf(line.scText, "Trickle2 rpm %4d", pState->iTrcikle2RPMslow); break;
      case 3: sprintf(line.scText, "Trickle %% %2d", pState->iTricklePercent); break;
   }
   displaySetLine_LCD(&line, iIndex + 1);
   free(line.scText);
}

void displayNewProgram(MachineState* pState, int8_t iIndex, LineType type)
{
   DisplayLine line;
   line.scText = malloc(32);
   line.indent = 0;
   line.type = type;

   switch (iIndex) {
      case 0: sprintf(line.scText, "Target Weight %3.2f", pState->fTargetWeight); break;
      case 1: sprintf(line.scText, "Dump dwell ms %4d", pState->iMeasureDumpDwell); break;
      case 2: sprintf(line.scText, "Trickle %% %2d", pState->iTricklePercent); break;
      case 3: sprintf(line.scText, "Gram / CC %3.2f", pState->fGramPerCC); break;
      case 4: sprintf(line.scText, "Trickle1 rpm %4d", pState->iTrickle1RPMfast); break;
      case 5: sprintf(line.scText, "Trickle2 rpm %4d", pState->iTrcikle2RPMslow); break;
   }
   displaySetLine_LCD(&line, iIndex + 1);
   free(line.scText);
}
