#include "system.h"

#include <string.h>
#include "buttons.h"
#include "loadcell.h"
#include "motor.h"


void createCalibrate(MachineState* pState, struct _CommandQueue* pQueue)
{
   strcpy(pState->strDisplay1, "000.0");
   pState->displayMode = SCALE_CALIBRATE_I;

   pQueue->iHi = 2;
   Command* pQ = &(pQueue->Q_High[2]);

   pQ->uCommand = CALIBRATE_I;
   pQ->pButtonHandler = &button_Calibrate;
   pQ->uButton[0] = 0; // Entering weight
   pQ--;

   pQ->uCommand = CALIBRATE_II;
   // Q will set target weight from above, once it's reached it will continue
   pQ--;

   pQ->uCommand = CALIBRATE_III;
   // Change message, wait short delay, then
   // just call the calibrate function
}
void createLoad(MachineState* pState, struct _CommandQueue* pQueue)
{
   strcpy(pState->strDisplay1, "000.0");
   pState->displayMode = LOAD_PROGRAM_I;

   pQueue->iHi = 2;
   Command* pQ = &(pQueue->Q_High[2]);

   pQ->uCommand = LOAD_I;
   pQ->pButtonHandler = &button_Load_Wt;
   pQ->uButton[0] = 0;
   pQ--;

   pQ->uCommand = LOAD_II;
   pQ->pButtonHandler = &button_Load_Rds;
   pQ->uButton[0] = 0;
   pQ--;

   pQ->uCommand = LOAD_III;
   pQ->uButton[0] = 0;
   pQ->uButton[1] = 0;
}

void createStep2Stage(MachineState* pState, struct _CommandQueue* pQueue, float fTargetWeight)
{
   Command* pQ = &(pQueue->Q_High[pQueue->iHi + 3]);

   pQ->uCommand = STEP_0_START_TWO;
   pQ--;

   pQ->uCommand = STEP_0_SPEED_BUMP_COND;
   pQ->fTargetWeight = fTargetWeight * 0.9;
   pQ--;

   pQ->uCommand = STEP_0_SPEED_BUMP_COND;
   pQ->fTargetWeight = fTargetWeight;

   if (pQueue->iHi >= 0) pQueue->iHi += 3;
   else pQueue->iHi = 2;
}


void create_trickle(Command* Q_Step, int8_t* indexStep, float fTargetWeight)
{
   (*indexStep)++;
   Q_Step[*indexStep].uCommand = STEP_0_SPEED_BUMP_COND;
   Q_Step[*indexStep].fTargetWeight = fTargetWeight;
   (*indexStep)++;
   Q_Step[*indexStep].uCommand = STEP_0_SPEED_BUMP_COND;
   Q_Step[*indexStep].fTargetWeight = fTargetWeight * 0.9;
   (*indexStep)++;
   Q_Step[*indexStep].uCommand = STEP_0_START_TWO;
}

void create_dump(Command* Q_Step0, Command* Q_Step1, int8_t* indexStep0, int8_t* indexStep1, int16_t iDwell)
{
   (*indexStep0)++;
   Q_Step0[*indexStep0].uCommand = STEP_CHAMBER_DUMP;
   Q_Step0[*indexStep0].uButton[0] = iDwell;
   (*indexStep1)++;
   Q_Step0[*indexStep1].uCommand = STEP_CHAMBER_DUMP;
   Q_Step0[*indexStep1].uButton[0] = iDwell;
}

void button_Calibrate(MachineState* pState, uint32_t* pWeight, uint8_t uButton)
{
   *pWeight = ((*pWeight) * 10) + uButton;
   uint32_t uTemp = *pWeight / 10000;
   uTemp *= 10000;
   *pWeight = *pWeight - uTemp;

   char* pDigit = &(pState->strDisplay1[5]);
   *pDigit-- = 0; // 5

   uint16_t uWeight = *pWeight;
   uint16_t uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 4
   uWeight *= .1;

   *pDigit-- = '.'; // 3

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 2
   uWeight *= .1;

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 1
   uWeight *= .1;

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 0
   uWeight *= .1;
}

void button_Load_Wt(MachineState* pState, uint32_t* pWeight, uint8_t uButton)
{
   button_Calibrate(pState, pWeight, uButton);
}

void button_Load_Rds(MachineState* pState, uint32_t* pWeight, uint8_t uButton)
{
   *pWeight = ((*pWeight) * 10) + uButton;
   uint32_t uTemp = *pWeight / 10000;
   uTemp *= 10000;
   *pWeight = *pWeight - uTemp;

   char* pDigit = &(pState->strDisplay1[4]);
   *pDigit-- = 0; // 4

   uint16_t uWeight = *pWeight;
   uint16_t uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 3
   uWeight *= .1;

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 2
   uWeight *= .1;

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 1
   uWeight *= .1;

   uDigit = uWeight % 10;
   *pDigit-- = uDigit + '0'; // 0
   uWeight *= .1;
}

void uintToString(uint32_t uNumber, uint8_t uDesiredNumDigits, char* strNum)
{
   char* pDigit = &(strNum[uDesiredNumDigits]);
   *pDigit = 0;

   do {
      pDigit--;
      *pDigit = (uNumber % 10) + '0';
      uNumber *= .1;
   } while (pDigit != strNum);
}

void runQueue(MachineState* pCurrentState, uint8_t uButton)
{
   if (CommandQueue.iHi >= 0) {
      int i = CommandQueue.iHi;
      switch (CommandQueue.Q_High[i].uCommand) {
         case PWM_0_START:
            pCurrentState->bPWM0_Running = TRUE;
            pCurrentState->iPWM0_Speed = CommandQueue.Q_High[i].uSpeed;
            servoSetSpeed(PWM0, pCurrentState->iPWM0_Speed, FALSE);
            servoStart(PWM0);
            CommandQueue.iHi--;
            break;

         case PWM_0_STOP:
            pCurrentState->bPWM0_Running = FALSE;
            servoStop(PWM0);
            CommandQueue.iHi--;
            break;

         case PWM_0_SPEED:
            pCurrentState->iPWM0_Speed = CommandQueue.Q_High[i].uSpeed;
            servoSetSpeed(PWM0, pCurrentState->iPWM0_Speed, TRUE);
            CommandQueue.iHi--;
            break;

         case PWM_0_STOP_COND: {
            bool bMadeWeight = FALSE;
            if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            else bMadeWeight = pCurrentState->fGramsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            if (bMadeWeight) {
               pCurrentState->bPWM0_Running = FALSE;
               servoStop(PWM0);
               CommandQueue.iHi--;
            }
         } break;

         case PWM_0_SPEED_COND: {
            bool bMadeWeight = FALSE;
            if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            else bMadeWeight = pCurrentState->fGramsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            if (bMadeWeight) {
               pCurrentState->iPWM0_Speed = CommandQueue.Q_High[i].uSpeed;
               servoSetSpeed(PWM0, pCurrentState->iPWM0_Speed, TRUE);
               CommandQueue.iHi--;
            }
         } break;

         case STEP_0_START_TWO:
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            stepRunTwo();
            CommandQueue.iHi--;
            break;

         case STEP_0_SPEED_BUMP_COND: {
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            bool bMadeWeight = FALSE;
            if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            else bMadeWeight = pCurrentState->fGramsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            if (bMadeWeight) {
               stepBumpSpeed();
               CommandQueue.iHi--;
            }
            // else update str1 with weight
         } break;

         case STEP_0_STOP_COND: {
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            bool bMadeWeight = FALSE;
            if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            else bMadeWeight = pCurrentState->fGramsCurrent >= CommandQueue.Q_High[i].fTargetWeight;
            if (bMadeWeight) {
               stepBumpSpeed();
               CommandQueue.iHi--;
            }
         } break;

         case CALIBRATE_I:
            if (uButton != 255) {
               if (convertButtonChar(uButton) == 'g') {
                  if ((--CommandQueue.iHi < 0) || // Make sure we have a next command
                        (CommandQueue.Q_High[CommandQueue.iHi].uCommand != CALIBRATE_II) || // Make sure it's the right command
                        (CommandQueue.Q_High[i].uButton[0] == 0)) { // Make sure we aren't trying to calibrate nothing
                     Error_Handler();
                     CommandQueue.iHi = -1;
                     pCurrentState->displayMode = SCALE_GRAIN;
                     return;
                  }

                  // Pass weight on
                  CommandQueue.Q_High[CommandQueue.iHi].fTargetWeight = ((float)CommandQueue.Q_High[i].uButton[0]) * 0.1;
                  // Set display mode
                  pCurrentState->displayMode = SCALE_CALIBRATE_II;
               } else {
                  // Run the button handler
                  CommandQueue.Q_High[i].pButtonHandler(pCurrentState, &(CommandQueue.Q_High[i].uButton[0]), convertButton(uButton));
               }
            }
            break;

         case CALIBRATE_II: {
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            bool bMadeWeight = FALSE;
            if (pCurrentState->scaleMode == SCALE_MODE_GRAM) bMadeWeight = pCurrentState->fGramsCurrent >= CommandQueue.Q_High[i].fTargetWeight * 0.95;
            else bMadeWeight = pCurrentState->fGrainsCurrent >= CommandQueue.Q_High[i].fTargetWeight * 0.95;
            if (bMadeWeight) { // Once we're close (calibration...might not be accurate right now)
               if ((--CommandQueue.iHi < 0) || // Make sure we have a next command
                     (CommandQueue.Q_High[CommandQueue.iHi].uCommand != CALIBRATE_III)) { // Make sure it's the right command
                  Error_Handler();
                  CommandQueue.iHi = -1;
                  pCurrentState->displayMode = SCALE_GRAIN;
                  return;
               }

               // Pass weight on
               CommandQueue.Q_High[CommandQueue.iHi].fTargetWeight = CommandQueue.Q_High[i].fTargetWeight;
               // Set display mode
               pCurrentState->displayMode = SCALE_CALIBRATE_III;
            }
         } break;

         case CALIBRATE_III:
            //*** Need to do both grams and grains (display should have specified when entered)
            //*** need to delay/wait a bit without blocking...(make sure display was updated)
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            if (pCurrentState->scaleMode == SCALE_MODE_GRAM) {
               calibrateGrams(CommandQueue.Q_High[i].fTargetWeight);
               pCurrentState->displayMode = SCALE_GRAM;
            } else {
               calibrateGrains(CommandQueue.Q_High[i].fTargetWeight);
               pCurrentState->displayMode = SCALE_GRAIN;
            }
            CommandQueue.iHi = -1;
            break;

         case LOAD_I:
            if (uButton != 255) {
               if (convertButtonChar(uButton) == 'g') {
                  if ((--CommandQueue.iHi < 0) || // Make sure we have a next command
                        (CommandQueue.Q_High[CommandQueue.iHi].uCommand != LOAD_II) || // Make sure it's the right command
                        (CommandQueue.Q_High[i].uButton[0] == 0)) { // Make sure we aren't trying to load nothing
                     Error_Handler();
                     CommandQueue.iHi = -1;
                     if (pCurrentState->scaleMode == SCALE_MODE_GRAM) pCurrentState->displayMode = SCALE_GRAM;
                     else pCurrentState->displayMode = SCALE_GRAIN;
                     return;
                  }

                  // Pass weight on
                  CommandQueue.Q_High[CommandQueue.iHi].fTargetWeight = ((float)CommandQueue.Q_High[i].uButton[0]) * 0.1;
                  // Set display mode
                  pCurrentState->displayMode = LOAD_PROGRAM_II;
                  strcpy(pCurrentState->strDisplay1, "0000");
               } else {
                  // Run the button handler
                  CommandQueue.Q_High[i].pButtonHandler(pCurrentState, &(CommandQueue.Q_High[i].uButton[0]), convertButton(uButton));
               }
            }
            break;

         case LOAD_II:
            if (uButton != 255) {
               if (convertButtonChar(uButton) == 'g') {
                  if ((--CommandQueue.iHi < 0) || // Make sure we have a next command
                        (CommandQueue.Q_High[CommandQueue.iHi].uCommand != LOAD_III) || // Make sure it's the right command
                        (CommandQueue.Q_High[i].uButton[0] == 0)) { // Make sure we aren't trying to load nothing
                     Error_Handler();
                     CommandQueue.iHi = -1;
                     if (pCurrentState->scaleMode == SCALE_MODE_GRAM) pCurrentState->displayMode = SCALE_GRAM;
                     else pCurrentState->displayMode = SCALE_GRAIN;
                     return;
                  }

                  // Pass weight on
                  CommandQueue.Q_High[CommandQueue.iHi].fTargetWeight = CommandQueue.Q_High[i].fTargetWeight;
                  CommandQueue.Q_High[CommandQueue.iHi].uButton[0] = 0;
                  CommandQueue.Q_High[CommandQueue.iHi].uButton[1] = CommandQueue.Q_High[i].uButton[0];
                  CommandQueue.Q_High[CommandQueue.iHi].uCommandIndex = 0;
                  uintToString(0, 4, pCurrentState->strDisplay2);
                  uintToString(CommandQueue.Q_High[CommandQueue.iHi].uButton[1], 4, pCurrentState->strDisplay3);
                  // Set display mode
                  pCurrentState->displayMode = LOAD_PROGRAM_III;
               } else {
                  // Run the button handler
                  CommandQueue.Q_High[i].pButtonHandler(pCurrentState, &(CommandQueue.Q_High[i].uButton[0]), convertButton(uButton));
               }
            }
            break;

         case LOAD_III:
            // str1 weight (step commands will load it too)
            // str2 current
            // str3 total
            // uButton[0]: current round index
            // uButton[1]: total num rounds
            strcpy(pCurrentState->strDisplay1, createString(pCurrentState->scaleMode));
            float fWeight = pCurrentState->scaleMode == SCALE_MODE_GRAM ? pCurrentState->fGramsCurrent : pCurrentState->fGrainsCurrent;
            switch (CommandQueue.Q_High[i].uCommandIndex) {
               case 0: {
                  // Dump the chamber
                  create_dump(CommandQueue.Q_Step[0], CommandQueue.Q_Step[1], &(CommandQueue.iStep[0]), &(CommandQueue.iStep[1]), pCurrentState->iMeasureDumpDwell);
                  CommandQueue.Q_High[i].uCommandIndex++;
               } break;
               case 1:
                  // dispense program
                  // need a trickle dwell as the first step
                  //create_trickle(CommandQueue.Q_Step[2], CommandQueue.iStep[2], CommandQueue.Q_High[i].fTargetWeight);
                  //createStep2Stage(pCurrentState, &CommandQueue, CommandQueue.Q_High[i].fTargetWeight);
                  CommandQueue.Q_High[i].uCommandIndex++;
                  break;
               case 2:
                  if (fWeight <= CommandQueue.Q_High[i].fTargetWeight * 0.9) {
                     // They picked it up
                     CommandQueue.Q_High[i].uCommandIndex++;
                  }
                  break;
               case 3:
                  CommandQueue.Q_High[i].uButton[0] += 1;
                  if (CommandQueue.Q_High[i].uButton[0] == CommandQueue.Q_High[i].uButton[1]) {
                     // Done loading
                     CommandQueue.iHi = -1;
                     pCurrentState->displayMode = SCALE_GRAIN;
                     return;
                  }

                  CommandQueue.Q_High[i].uCommandIndex++;
                  uintToString(CommandQueue.Q_High[i].uButton[0], 4, pCurrentState->strDisplay2);
                  break;
               case 4:
                  // Wait for them to replace the tray before continuing
                  //**could detect button or external trigger
                  if (fWeight < -10.0) { //*** tray weight needs to be a setting
                     CommandQueue.Q_High[i].uCommandIndex++;
                  }
                  break;
               case 5:
                  // Wait for them to replace the tray before continuing
                  //**could detect button or external trigger
                  // this still isn't right...they could then

                  if (fWeight >= -0.1) { // should some sort of close float equals
                     CommandQueue.Q_High[i].uCommandIndex = 0;
                  }
                  break;
            }
            break;
      }
   }
}

void runStepperQueue(MachineState* pCurrentState, StepperMotor* stepper, Command* Q_Step, int8_t* indexStep)
{
   int i = *indexStep;
   if (i < 0) return;

   switch (Q_Step[i].uCommand) {
      case STEP_CHAMBER_DUMP:
         stepper_180(stepper, Q_Step[i].uButton[0]);
         (*indexStep)--;
         break;

      case STEP_CHAMBER_INCREASE:
         stepper_30CCW(stepper);
         HAL_Delay(500);
         (*indexStep)--;
         break;

      case STEP_CHAMBER_DECREASE:
         stepper_30CW(stepper);
         HAL_Delay(500);
         (*indexStep)--;
         break;

      case STEP_CHAMBER_ENABLE:
         stepperEnable(stepper, TRUE);
         (*indexStep)--;
         break;

      case STEP_CHAMBER_DISABLE:
         stepperEnable(stepper, FALSE);
         (*indexStep)--;
         break;

      case STEP_0_START_TWO:
         stepper_2stage(stepper);
         (*indexStep)--;
         break;

      case STEP_0_SPEED_BUMP_COND: {
         bool bMadeWeight = FALSE;
         if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= Q_Step[i].fTargetWeight;
         else bMadeWeight = pCurrentState->fGramsCurrent >= Q_Step[i].fTargetWeight;
         if (bMadeWeight) {
            stepper_bump(stepper);
            (*indexStep)--;
         }
         // else update str1 with weight
      } break;

      case STEP_0_STOP_COND: {
         bool bMadeWeight = FALSE;
         if (pCurrentState->scaleMode == SCALE_MODE_GRAIN) bMadeWeight = pCurrentState->fGrainsCurrent >= Q_Step[i].fTargetWeight;
         else bMadeWeight = pCurrentState->fGramsCurrent >= Q_Step[i].fTargetWeight;
         if (bMadeWeight) {
            stepper_bump(stepper);
            (*indexStep)--;
         }
      } break;
   }
}
