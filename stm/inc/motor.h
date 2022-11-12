#ifndef motor_h_
#define motor_h_

#include "main.h"

enum MOTOR_PWM {
   PWM0 = 0,
   PWM1
};

// Config defines (pick one of these)
#define Step_Full    0x00
#define Step_2    0x01
#define Step_4    0x02
#define Step_8    0x03
#define Step_16      0x04
#define Step_32      0x05
#define Step_64      0x06
#define Step_128  0x07
// If you want variable stepper rates, OR this with the step setting
#define Step_Var  0x08

typedef struct _stepperTimerTable {
   int32_t iNumSteps;
   uint32_t uNewDelay;
   bool bClockwise;
   bool bStep;
} stepperTimerTable; //**this isn't a table, table entry

typedef struct _stepperMotor {
   TIM_HandleTypeDef* htim;
   GPIO_TypeDef* gpio_EnStby;
   GPIO_TypeDef* gpio_M;
   uint16_t en;
   uint16_t stby;
   uint16_t m0; // updw
   uint16_t m1; // seten
   uint16_t m2; // clk
   uint16_t m3; // cwccw

   bool bStepOnTimer;
   int timerIndexCurrent;
   int timerIndexStop;
   int tableIndexCurrent;
   stepperTimerTable* arStepper_Table;
} StepperMotor;

void servoInit(TIM_HandleTypeDef* _pHandle, int iChannel, enum MOTOR_PWM motor);
void servoStart(enum MOTOR_PWM motor);
void servoStop(enum MOTOR_PWM motor);
void servoSetSpeed(enum MOTOR_PWM motor, int iSpeed, bool bRunning);

void stepTableInit();
void stepRunTwo();
void stepBumpSpeed();

void stepperInit(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2);
void stepperConfig(StepperMotor* pMotor, uint8_t uConfig);
void stepperEnable(StepperMotor* pMotor, bool bEnable);
void stepperSetCW(StepperMotor* pMotor);
void stepperSetCCW(StepperMotor* pMotor);

void stepper_30CW(StepperMotor* pMotor);
void stepper_30CCW(StepperMotor* pMotor);
void stepper_180(StepperMotor* pMotor, int iDwellMS); // -1 for don't change
void stepper_2stage(StepperMotor* pMotor);
void stepper_bump(StepperMotor* pMotor);
#endif
