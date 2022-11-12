#ifndef loadCell_h_
#define loadCell_h_
/*
 * Filter ideas:
 *    These processors contain instructions made for this kind of thing. IIR and FIR at a glance look
 *    like they'd be pretty good fits. Some of these links also touch on the simd available.
 *
 *    https://www.groupfourtransducers.com/news/digital-signal-conditioners-explained/
 *    https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIRLPF.html
 *    https://www.st.com/resource/en/application_note/an4841-digital-signal-processing-for-stm32-microcontrollers-using-cmsis-stmicroelectronics.pdf
 *
 *
 */

#include "main.h"
#include "system.h"

bool readCell();
void setTare();
void calibrateGrams(int iGrams);
void calibrateGrains(int iGrains);
void calculateAverage();
char* createStringHex();
char* createString(Scale_Mode mode);
char* createStringGrams();
char* createStringGrains();

float getGrains();
float getGrams();

void filterInit();
void filterInsert(uint32_t u);
float _filterGet(unsigned char uSamples);
float filterGet();
float filterGetFast();

#endif
