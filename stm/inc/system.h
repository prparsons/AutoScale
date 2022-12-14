#ifndef system_h_
#define system_h_

#include "main.h"
#include "motor.h"


#define PWM_COND_GREAT 0x01

typedef enum _Display_Mode {
   SCALE_GRAIN,         // Grains:\n+-000.0
   SCALE_GRAM,          // Grams:\n+-00.000
   MAIN_MENU,
   SETTINGS,
   CALIBRATE,
   LOAD_PROGRAM,
   LOAD_RUN
} Display_Mode;

typedef enum _Stepper_Status {
   STEPPER_OFF,
   STEPPER_RUN_SIMPLE,
   STEPPER_RUN,
   STEPPER_RUN_TWO_STAGE
} Stepper_Status;

typedef enum _Scale_Mode {
   SCALE_MODE_GRAM,
   SCALE_MODE_GRAIN
} Scale_Mode;

enum Commands {
   PWM_0_START =                 0x0001,
   PWM_0_STOP =                  0x0002,
   PWM_0_SPEED =                 0x0003,
   PWM_0_SPEED_COND =            0x0004,
   PWM_0_STOP_COND =             0x0005,
   STEP_0_START_TWO =            0x0008,
   STEP_0_SPEED_BUMP_COND =      0x0009,
   STEP_0_STOP_COND =            0x000a,
   STEP_CHAMBER_DUMP =           0x0030,
   STEP_CHAMBER_INCREASE =       0x0031,
   STEP_CHAMBER_DECREASE =       0x0032,
   STEP_CHAMBER_ENABLE =         0x0033,
   STEP_CHAMBER_DISABLE =        0x0034,
   RUN_PROGRAM
};

typedef struct _MachineState {
   bool bPWM0_Running;
   int iPWM0_Speed;

   Stepper_Status stepTrickle;
   int16_t iMeasureDumpDwell;
   int8_t iTricklePercent; // if we had the dump chamber dump 90% of the target weight, then trickle percent would be 10%
   float fGramPerCC;
   int16_t iTrickle1RPMfast;
   int16_t iTrcikle2RPMslow;

   Scale_Mode scaleMode;
   float fGramsCurrent;
   float fGrainsCurrent;
   float fTargetWeight;
   float fTrayWeight;

   Display_Mode displayMode;
   char strDisplay1[16];
   char strDisplay2[16];
   char strDisplay3[16];
   int iMenuSelection;
   bool bMenuHighlight;
} MachineState;

// Unions maybe? (reduce size if they get big)
typedef struct _Command {
   uint16_t uCommand;

   // Motor based stuff
   uint16_t uSpeed; // PWM, servo
   // Stepper program pointer?
   float fTargetWeight;
   uint8_t uCondition;

   // Display/menu based stuff
   void (*pButtonHandler)(MachineState*, uint32_t*, uint8_t);
   uint32_t uButton[2]; // rename this to data?
   uint8_t uCommandIndex;
} Command;

// Initial idea was high priority, low priority, I suppose that would need a timer to check
//    and see whether we're close to sample rate (or something else?) interruption.
/*
 * What if instead we have motor queues and a UI queue?
 * should we worry about time?
 *
 * After implementing a queue for the stepper and then the menus in the lcd...this command queue idea
 * seems kind of cumbersome
 *
 * main.c loop?
 */
struct _CommandQueue {
   Command Q_High[16];
   int8_t iHi;

   Command Q_Step[2][16];
   int8_t iStep[2];
} CommandQueue;

// Create Command functions
void create_trickle(Command* Q_Step, int8_t* indexStep, float fTargetWeight);
void create_dump(Command* Q_Step0, Command* Q_Step1, int8_t* indexStep0, int8_t* indexStep1, int16_t iDwell);

void runQueue(MachineState* pCurrentState, uint8_t uButton);
void runStepperQueue(MachineState* pCurrentState, StepperMotor* stepper, Command* Q_Step, int8_t* indexStep);

#endif
