#ifndef display_h_
#define display_h_

#include "main.h"
#include "system.h"

#define LCD_LINES 10

typedef enum _lineType {
   LINE_TYPE_TEXT = 0,
   LINE_TYPE_HLTEXT,
   LINE_TYPE_SELTEXT,
   LINE_TYPE_HLSELTEXT,
   LINE_TYPE_STATIC,
   LINE_TYPE_BUTTON,
   LINE_TYPE_HEADER
} LineType;

typedef struct _displayLine {
   LineType type;
   int indent;
   char* scText;
} DisplayLine;

void displayInit_LCD(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim);
void displaySetLine_LCD(DisplayLine* pLine, int iLineNum);
void displayEraseLine_LCD(int iLineNum);
void displayDrawLine_LCD(int iLineNum);

void displaySwitchMode(MachineState* pState, Display_Mode modeFrom);
void displayUpdate_(MachineState* pState);
void displayHandleMenu(MachineState* pState, uint8_t uButton, char cButton);

void displayPrintMain(MachineState* pState, int8_t iIndex, LineType type);
void displayPrintSettings(MachineState* pState, int8_t iIndex, LineType type); // Helper function to contain the printing of settings menu in single place
void displayNewProgram(MachineState* pState, int8_t iIndex, LineType type);

#endif
