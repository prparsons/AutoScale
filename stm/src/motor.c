#include "motor.h"

// Servo (PWM)
TIM_HandleTypeDef* arServoHandles[] = { nullptr, nullptr };
int arServoChannels[] = { 0, 0 };

// Indefinite number of steps in these tables (set the numsteps to -1, manually adjust stepper index)
stepperTimerTable stepper_TableSimple[7]; // Simple speed up, slow down
stepperTimerTable stepper_TableTwoStage[16]; // Multi stage slow down
stepperTimerTable stTable_32_180_simple[11];
stepperTimerTable stTable_32_30_moveCW[5];
stepperTimerTable stTable_32_30_moveCCW[5];

StepperMotor motorList[2];

void servoInit(TIM_HandleTypeDef* _pHandle, int iChannel, enum MOTOR_PWM motor)
{
   arServoHandles[motor] = _pHandle;
   arServoChannels[motor] = iChannel;
}

void servoStart(enum MOTOR_PWM motor)
{
   HAL_TIM_PWM_Start(arServoHandles[motor], arServoChannels[motor]);
}

void servoStop(enum MOTOR_PWM motor)
{
   HAL_TIM_PWM_Stop(arServoHandles[motor], arServoChannels[motor]);
}

void servoSetSpeed(enum MOTOR_PWM motor, int iSpeed, bool bRunning)
{
   if (bRunning) HAL_TIM_PWM_Stop(arServoHandles[motor], arServoChannels[motor]); // stop generation of pwm

   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = iSpeed;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   if (HAL_TIM_PWM_ConfigChannel(arServoHandles[motor], &sConfigOC, arServoChannels[motor]) != HAL_OK) Error_Handler();

   if (bRunning) HAL_TIM_PWM_Start(arServoHandles[motor], arServoChannels[motor]); // start pwm generation

}

void stepTableInit()
{
   stTable_32_30_moveCW[4].iNumSteps = 57;
   stTable_32_30_moveCW[3].iNumSteps = 256;
   stTable_32_30_moveCW[2].iNumSteps = 334;
   stTable_32_30_moveCW[1].iNumSteps = 256;
   stTable_32_30_moveCW[0].iNumSteps = 57;
   stTable_32_30_moveCW[4].uNewDelay = 14000;
   stTable_32_30_moveCW[3].uNewDelay = 3125;
   stTable_32_30_moveCW[2].uNewDelay = 2500;
   stTable_32_30_moveCW[1].uNewDelay = 3125;
   stTable_32_30_moveCW[0].uNewDelay = 14000;
   stTable_32_30_moveCW[4].bClockwise = TRUE;
   stTable_32_30_moveCW[3].bClockwise = TRUE;
   stTable_32_30_moveCW[2].bClockwise = TRUE;
   stTable_32_30_moveCW[1].bClockwise = TRUE;
   stTable_32_30_moveCW[0].bClockwise = TRUE;
   stTable_32_30_moveCW[4].bStep = TRUE;
   stTable_32_30_moveCW[3].bStep = TRUE;
   stTable_32_30_moveCW[2].bStep = TRUE;
   stTable_32_30_moveCW[1].bStep = TRUE;
   stTable_32_30_moveCW[0].bStep = TRUE;

   stTable_32_30_moveCCW[4].iNumSteps = 57;
   stTable_32_30_moveCCW[3].iNumSteps = 256;
   stTable_32_30_moveCCW[2].iNumSteps = 334;
   stTable_32_30_moveCCW[1].iNumSteps = 256;
   stTable_32_30_moveCCW[0].iNumSteps = 57;
   stTable_32_30_moveCCW[4].uNewDelay = 14000;
   stTable_32_30_moveCCW[3].uNewDelay = 3125;
   stTable_32_30_moveCCW[2].uNewDelay = 2500;
   stTable_32_30_moveCCW[1].uNewDelay = 3125;
   stTable_32_30_moveCCW[0].uNewDelay = 14000;
   stTable_32_30_moveCCW[4].bClockwise = FALSE;
   stTable_32_30_moveCCW[3].bClockwise = FALSE;
   stTable_32_30_moveCCW[2].bClockwise = FALSE;
   stTable_32_30_moveCCW[1].bClockwise = FALSE;
   stTable_32_30_moveCCW[0].bClockwise = FALSE;
   stTable_32_30_moveCCW[4].bStep = TRUE;
   stTable_32_30_moveCCW[3].bStep = TRUE;
   stTable_32_30_moveCCW[2].bStep = TRUE;
   stTable_32_30_moveCCW[1].bStep = TRUE;
   stTable_32_30_moveCCW[0].bStep = TRUE;

   stTable_32_180_simple[10].iNumSteps = 57;
   stTable_32_180_simple[9].iNumSteps = 256;
   stTable_32_180_simple[8].iNumSteps = 2574;
   stTable_32_180_simple[7].iNumSteps = 256;
   stTable_32_180_simple[6].iNumSteps = 57;
   stTable_32_180_simple[5].iNumSteps = 99;
   stTable_32_180_simple[4].iNumSteps = 57;
   stTable_32_180_simple[3].iNumSteps = 256;
   stTable_32_180_simple[2].iNumSteps = 2574;
   stTable_32_180_simple[1].iNumSteps = 256;
   stTable_32_180_simple[0].iNumSteps = 57;

   stTable_32_180_simple[10].uNewDelay = 14000;
   stTable_32_180_simple[9].uNewDelay = 3125;
   stTable_32_180_simple[8].uNewDelay = 2500;
   stTable_32_180_simple[7].uNewDelay = 3125;
   stTable_32_180_simple[6].uNewDelay = 14000;
   stTable_32_180_simple[5].uNewDelay = 16000;
   stTable_32_180_simple[4].uNewDelay = 14000;
   stTable_32_180_simple[3].uNewDelay = 3125;
   stTable_32_180_simple[2].uNewDelay = 2500;
   stTable_32_180_simple[1].uNewDelay = 3125;
   stTable_32_180_simple[0].uNewDelay = 14000;

   stTable_32_180_simple[10].bClockwise = TRUE;
   stTable_32_180_simple[9].bClockwise = TRUE;
   stTable_32_180_simple[8].bClockwise = TRUE;
   stTable_32_180_simple[7].bClockwise = TRUE;
   stTable_32_180_simple[6].bClockwise = TRUE;
   stTable_32_180_simple[5].bClockwise = TRUE;
   stTable_32_180_simple[4].bClockwise = FALSE;
   stTable_32_180_simple[3].bClockwise = FALSE;
   stTable_32_180_simple[2].bClockwise = FALSE;
   stTable_32_180_simple[1].bClockwise = FALSE;
   stTable_32_180_simple[0].bClockwise = FALSE;

   stTable_32_180_simple[10].bStep = TRUE;
   stTable_32_180_simple[9].bStep = TRUE;
   stTable_32_180_simple[8].bStep = TRUE;
   stTable_32_180_simple[7].bStep = TRUE;
   stTable_32_180_simple[6].bStep = TRUE;
   stTable_32_180_simple[5].bStep = FALSE;
   stTable_32_180_simple[4].bStep = TRUE;
   stTable_32_180_simple[3].bStep = TRUE;
   stTable_32_180_simple[2].bStep = TRUE;
   stTable_32_180_simple[1].bStep = TRUE;
   stTable_32_180_simple[0].bStep = TRUE;

   stepper_TableSimple[4].iNumSteps = 16;
   stepper_TableSimple[3].iNumSteps = 16;
   stepper_TableSimple[2].iNumSteps = 6400;
   stepper_TableSimple[1].iNumSteps = 3200;
   stepper_TableSimple[0].iNumSteps = 1600;
   stepper_TableSimple[4].uNewDelay = 32768;
   stepper_TableSimple[3].uNewDelay = 16384;
   stepper_TableSimple[2].uNewDelay = 8192;
   stepper_TableSimple[1].uNewDelay = 16384;
   stepper_TableSimple[0].uNewDelay = 32768;
   stepper_TableSimple[4].bClockwise = FALSE;
   stepper_TableSimple[3].bClockwise = FALSE;
   stepper_TableSimple[2].bClockwise = FALSE;
   stepper_TableSimple[1].bClockwise = FALSE;
   stepper_TableSimple[0].bClockwise = FALSE;
   stepper_TableSimple[4].bStep = TRUE;
   stepper_TableSimple[3].bStep = TRUE;
   stepper_TableSimple[2].bStep = TRUE;
   stepper_TableSimple[1].bStep = TRUE;
   stepper_TableSimple[0].bStep = TRUE;

   stepper_TableTwoStage[5].iNumSteps = 16;
   stepper_TableTwoStage[4].iNumSteps = 16;
   stepper_TableTwoStage[3].iNumSteps = -1;
   stepper_TableTwoStage[2].iNumSteps = -1;
   stepper_TableTwoStage[1].iNumSteps = 8;
   stepper_TableTwoStage[0].iNumSteps = 4;
   stepper_TableTwoStage[5].uNewDelay = 8192;
   stepper_TableTwoStage[4].uNewDelay = 2048;
   stepper_TableTwoStage[3].uNewDelay = 512;
   stepper_TableTwoStage[2].uNewDelay = 8192;
   stepper_TableTwoStage[1].uNewDelay = 16384;
   stepper_TableTwoStage[0].uNewDelay = 32768;
   stepper_TableTwoStage[5].bClockwise = FALSE;
   stepper_TableTwoStage[4].bClockwise = FALSE;
   stepper_TableTwoStage[3].bClockwise = FALSE;
   stepper_TableTwoStage[2].bClockwise = FALSE;
   stepper_TableTwoStage[1].bClockwise = FALSE;
   stepper_TableTwoStage[0].bClockwise = FALSE;
}

void stepRunTwo()
{
   //arStepper_Table = stepper_TableTwoStage;
   //uStepper_IndexCurrent = 0;
   //iStepper_IndexStop = 1; //***will zero work?
   //uStepper_IndexTable = uStepper_IndexMaxTwo;

   //stepEnable(TRUE);
}

void stepBumpSpeed()
{
   //iStepper_IndexStop = 0;
   //uStepper_IndexCurrent = 0;
}

void stepperInit(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2)
{
   motorList[0].htim = htim1;
   motorList[0].gpio_EnStby = ST1_EN_GPIO_Port;
   motorList[0].gpio_M = ST1_EN_GPIO_Port;
   motorList[0].en = ST1_EN_Pin;
   motorList[0].stby = ST1_STBY_Pin;
   motorList[0].m0 = ST1_M0_Pin;
   motorList[0].m1 = ST1_M1_Pin;
   motorList[0].m2 = ST1_M2_Pin;
   motorList[0].m3 = ST1_M3_Pin;
   motorList[0].bStepOnTimer = TRUE;
   motorList[0].timerIndexCurrent = 0;
   motorList[0].timerIndexStop = 1;
   motorList[0].tableIndexCurrent = 11;
   motorList[0].arStepper_Table = stTable_32_180_simple;


   motorList[1].htim = htim2;
   motorList[1].gpio_EnStby = ST0_EN_GPIO_Port;
   motorList[1].gpio_M = ST0_M0_GPIO_Port;
   motorList[1].en = ST0_EN_Pin;
   motorList[1].stby = ST0_STBY_Pin;
   motorList[1].m0 = ST0_M0_Pin;
   motorList[1].m1 = ST0_M1_Pin;
   motorList[1].m2 = ST0_M2_Pin;
   motorList[1].m3 = ST0_M3_Pin;
   motorList[1].bStepOnTimer = TRUE;
   motorList[1].timerIndexCurrent = 0;
   motorList[1].timerIndexStop = 1;
   motorList[1].tableIndexCurrent = 11;
   motorList[1].arStepper_Table = stTable_32_180_simple;
}

void stepperConfig(StepperMotor* pMotor, uint8_t uConfig)
{
   GPIO_TypeDef* gpio_EnStby = pMotor->gpio_EnStby;
   GPIO_TypeDef* gpio_M = pMotor->gpio_M;
   // turn off everything
   gpio_EnStby -> ODR &= ~(pMotor->en + pMotor->stby);
   gpio_M -> ODR &= ~(pMotor->m0 + pMotor->m1 + pMotor->m2 + pMotor->m3);

   HAL_Delay(1000);//timerDelay(16); // should be 1sec

     if (uConfig & Step_Var) {
        switch (uConfig & Step_128) {
           case Step_2:
              gpio_M -> ODR |= pMotor->m0;
              break;
           case Step_4:
              gpio_M -> ODR |= pMotor->m1;
              break;
           case Step_8:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m1;
              break;
           case Step_16:
              gpio_M -> ODR |= pMotor->m2;
              break;
           case Step_32:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m2;
              break;
           case Step_64:
              gpio_M -> ODR |= pMotor->m1 + pMotor->m2;
              break;
           case Step_128:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m1 + pMotor->m2;
              break;
        }
     } else {
        switch (uConfig & Step_128) {
           case Step_Full:
              gpio_M -> ODR |= pMotor->m3;
              break;
           case Step_2:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m3;
              break;
           case Step_4:
              gpio_M -> ODR |= pMotor->m1 + pMotor->m3;
              break;
           case Step_8:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m1 + pMotor->m3;
              break;
           case Step_16:
              gpio_M -> ODR |= pMotor->m2 + pMotor->m3;
              break;
           case Step_32:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m2 + pMotor->m3;
              break;
           case Step_64:
              gpio_M -> ODR |= pMotor->m1 + pMotor->m2 + pMotor->m3;
              break;
           case Step_128:
              gpio_M -> ODR |= pMotor->m0 + pMotor->m1 + pMotor->m2 + pMotor->m3;
              break;
        }
     }

     timerDelay(16);// Delay 1us
     gpio_EnStby -> ODR |= pMotor->stby;
     timerDelay(16*200);//delay 200us
     gpio_M -> ODR &= ~(pMotor->m0 + pMotor->m1 + pMotor->m2 + pMotor->m3);
}

void stepperEnable(StepperMotor* pMotor, bool bEnable)
{
   if (bEnable) HAL_GPIO_WritePin(pMotor->gpio_EnStby, pMotor->en, GPIO_PIN_SET);
   else HAL_GPIO_WritePin(pMotor->gpio_EnStby, pMotor->en, GPIO_PIN_RESET);
}

void stepperSetCW(StepperMotor* pMotor)
{
   HAL_GPIO_WritePin(pMotor->gpio_M, pMotor->m3, GPIO_PIN_SET);
}

void stepperSetCCW(StepperMotor* pMotor)
{
   HAL_GPIO_WritePin(pMotor->gpio_M, pMotor->m3, GPIO_PIN_RESET);
}

void stepper_30CW(StepperMotor* pMotor)
{
   HAL_TIM_Base_Stop_IT(pMotor->htim);
   pMotor->arStepper_Table = stTable_32_30_moveCW;
   pMotor->tableIndexCurrent = 5;
   pMotor->timerIndexCurrent = 0;
   pMotor->timerIndexStop = 1;
   stepperEnable(pMotor, TRUE);
   HAL_TIM_Base_Start_IT(pMotor->htim);
}

void stepper_30CCW(StepperMotor* pMotor)
{
   HAL_TIM_Base_Stop_IT(pMotor->htim);
   pMotor->arStepper_Table = stTable_32_30_moveCCW;
   pMotor->tableIndexCurrent = 5;
   pMotor->timerIndexCurrent = 0;
   pMotor->timerIndexStop = 1;
   stepperEnable(pMotor, TRUE);
   HAL_TIM_Base_Start_IT(pMotor->htim);
}

void stepper_180(StepperMotor* pMotor, int iDwellMS)
{
   // 16mhz, delay X # of steps of 16,000 cycles ea
   // so it's a ms based delay
   HAL_TIM_Base_Stop_IT(pMotor->htim);
   pMotor->arStepper_Table = stTable_32_180_simple;
   if (iDwellMS > 0) pMotor->arStepper_Table[5].iNumSteps = iDwellMS;
   pMotor->tableIndexCurrent = 11;
   pMotor->timerIndexCurrent = 0;
   pMotor->timerIndexStop = 1;
   stepperEnable(pMotor, TRUE);
   HAL_TIM_Base_Start_IT(pMotor->htim);
}

void stepper_2stage(StepperMotor* pMotor)
{
   HAL_TIM_Base_Stop_IT(pMotor->htim);
   pMotor->arStepper_Table = stepper_TableTwoStage;
   pMotor->timerIndexCurrent = 0;
   pMotor->timerIndexStop = 1;
   pMotor->tableIndexCurrent = 5;
   stepperEnable(pMotor, TRUE);
   HAL_TIM_Base_Start_IT(pMotor->htim);
}

void stepper_bump(StepperMotor* pMotor)
{
   HAL_TIM_Base_Stop_IT(pMotor->htim);
   pMotor->timerIndexCurrent = 0;
   pMotor->timerIndexStop = 0;
   stepperEnable(pMotor, TRUE);
   HAL_TIM_Base_Start_IT(pMotor->htim);
}
