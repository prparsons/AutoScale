#include "system.h"

#include <string.h>
#include "buttons.h"
#include "loadcell.h"
#include "motor.h"


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

         case RUN_PROGRAM: {
            float fWeight = pCurrentState->scaleMode == SCALE_MODE_GRAM ? pCurrentState->fGramsCurrent : pCurrentState->fGrainsCurrent;
            switch (CommandQueue.Q_High[i].uCommandIndex) {
               case 0: {
                  // Dump the chamber
                  create_dump(CommandQueue.Q_Step[0], CommandQueue.Q_Step[1], &(CommandQueue.iStep[0]), &(CommandQueue.iStep[1]), pCurrentState->iMeasureDumpDwell);
                  CommandQueue.Q_High[i].uCommandIndex++;
               } break;

               case 1:
                  // trickle the last little bit
                  CommandQueue.Q_High[i].uCommandIndex++;
                  break;

               case 2:
                  if (fWeight <= CommandQueue.Q_High[i].fTargetWeight * 0.9) {
                     // They picked it up
                     CommandQueue.Q_High[i].uCommandIndex++;
                  }
                  break;

               case 3:
                  // Wait for them to replace the tray before continuing
                  //**could detect button or external trigger...
                  if (fWeight < -pCurrentState->fTrayWeight) {
                     CommandQueue.Q_High[i].uCommandIndex++;
                  }
                  break;

               case 4:
                  // Need a better way to wait than delay, course would it be necessary if
                  //    tray/cell were consistent?
                  if (fWeight >= -0.1) { // should some sort of close float equals
                     CommandQueue.Q_High[i].uCommandIndex = 0;
                     // until i figure out tray/load cell so it always sets down in the same place,
                     // just wait a bit then tare again.
                     HAL_Delay(500);
                     setTare();
                  }
                  break;
            }
         } break;
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
