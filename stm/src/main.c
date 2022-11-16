/* USER CODE BEGIN Header */
//       [STM32F405 Pin, ThingPlus Pin]
// PC0   [8, 8 D0]
//    Input to read the load cell
// PC1   [9, 7 D1]
//    Output to write clk to drive load cell "serial"
//
//
// GPIO
//    - https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2001%20-%20GPIO%20Operations%20using%20HAL%20(and%20FreeRTOS).pdf
//
// Timer idea for delay
//    - https://controllerstech.com/create-1-microsecond-delay-stm32/
//    - https://www.digikey.com/en/maker/projects/getting-started-with-stm32-timers-and-timer-interrupts/d08e6493cefa486fb1e79c43c0b08cc6
//
/*
 * Explanation
 *  Uses an hx711, basically a 24bit ADC. Interface is it's own kind of serial setup.
 *  -Wait until DAT is low (indicates it's ready). Sample is read MSB to LSB.
 *      Wait at least 0.1 microsecond until starting
 *  -We set CLK high, on it's rising edge (0.1microsecond max delay) DAT will have the next bit ready.
 *      CLK high time 0.2 - 50 microsecond
 *      CLK low time min 0.2 microsecond (1 typical)
 *  -Gain is set by how many pulses we send (DAT is pulled high by 25th pulse
 *      Pulse   Gain
 *      25      128
 *      26      32
 *      27      64
 *
 *
 * Pinout
 *  ThingPlus  ->  Load Cell Amplifier
 *  D0         ->   DAT
 *  D1         ->   CLK
 *
 *  ThingPlus  ->  Display MSP430G2553 (I2C)
 *  SCL        ->   P1.6 SCL
 *  SDA        ->   P1.7 SDA
 *
 *  Load Cell Amplifier
 *  VDD   ->   ThingPlus +3.3V
 *  VCC   ->   VREF +4.096V
 *
 *  Precision Voltage Ref (LM4040)
 *  VCC   ->   ThingPlus +3.3V
 *
 *
 * To add second stepper (and more), moved to ATP carrier board, didn't have to re-map anything,
 * though some of the board labels have changed. Labels below are all correct for ATP board.
 *
 *
 *  stepper driver 1
 *    en   pa8   g1
 *    stby pa0   g2
 *    0    pc8   g3
 *    1    pc9   g4
 *    2    pc13  g5
 *    3    pc2   g6
 *
 *  stepper driver 2
 *    en
 *    stby
 *    0
 *    1
 *    2
 *    3
 *
 *  LCD
 *    dc          pb0   a1
 *    rst         pc5   a0
 *    cs          pa4   i2s_ws
 *    pwm(timer)  pa2   tx1      (timer 5, channel 3)
 *  spi1
 *    mosi        pa7
 *    miso        pa6
 *    sck         pa5
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "buttons.h"
#include "display.h"
#include "loadCell.h"
#include "motor.h"
#include "system.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern StepperMotor motorList[2];

MachineState currentState;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timerDelay(uint16_t u16ths)
{
   __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
   while (__HAL_TIM_GET_COUNTER(&htim1) < u16ths);  // wait for the counter to reach the input parameter
}

void statusBlink(enum LED_STATUS code)
{
   int iTimes;
   switch (code) {
      case LED_INIT_SUCCESS: {
         for (int i = 0; i < 2; i++) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
            HAL_Delay(250);
         }
         HAL_Delay(1000);
         iTimes = 2;
      } break;
      case LED_SAMPLE_SUCCESS:   iTimes = 4; break;
      case LED_SAMPLE_FAIL:      iTimes = 6; break;
      case LED_I2C_FAIL:         iTimes = 8; break;

      default: return;
   }
   for (int i = 0; i < iTimes; i++) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
      HAL_Delay(250);
   }
   HAL_Delay(1000);
   for (int i = 0; i < iTimes; i++) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
      HAL_Delay(250);
   }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
   if(HAL_I2C_Init(&hi2c2) != HAL_OK) {
      statusBlink(LED_I2C_FAIL);
   }
   buttonsInit(&hi2c2);
   servoInit(&htim8, TIM_CHANNEL_1, PWM0);

   displayInit_LCD(&hspi1, &htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   HAL_TIM_Base_Start(&htim1);
   //__HAL_TIM_SET_COUNTER, HAL_TIM_Base_Start, HAL_TIM_Base_Start_IT, __HAL_TIM_SET_COMPARE
   // __HAL_TIM_GET_AUTORELOAD/__HAL_TIM_SET_AUTORELOAD
   // Compare is pulse width, counter is the actual counter
   /*
    * PWM Prescaler and Counter Period (PSC and ARR)
    * Frequency PWM = Freq Clock / ((ARR + 1)*(PSC + 1))
    *
    * desired frequency * ARR = timer frequency
    * PSC = (clock / timer frequency) - 1
    *
    * 50Hz = 64MHz / ((4999 + 1)*(255 + 1))
    *
    * Ideally want to maximize ARR (thats your max pulse width)
    * Close enough to 12bit with these values
    */
   //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

   statusBlink(LED_INIT_SUCCESS);
   HAL_Delay(3000);

   uint8_t u = 0;
   uint8_t uButton = 255;
   char cButton = 'x';
   currentState.bPWM0_Running = FALSE;
   currentState.iPWM0_Speed = 375; // 1.5ms (max value 4999), 500 == 2ms, 250 == 1ms
   currentState.displayMode = MAIN_MENU;
   currentState.scaleMode = SCALE_MODE_GRAM;
   currentState.iMeasureDumpDwell = 500;
   currentState.iMenuSelection = 0;
   currentState.fTargetWeight = 0;
   currentState.fTrayWeight = 2;
   currentState.iTricklePercent = 90;
   currentState.fGramPerCC = 10;
   currentState.iTrickle1RPMfast = 2;
   currentState.iTrcikle2RPMslow = 1;
   currentState.bMenuHighlight = FALSE;
   displaySwitchMode(&currentState, 0);

   stepTableInit();
   stepperInit(&htim3, &htim2);
   stepperConfig(&motorList[1], Step_32);
   stepperConfig(&motorList[0], Step_32);

   filterInit();

   CommandQueue.iHi = -1;
   CommandQueue.iStep[0] = -1;
   CommandQueue.iStep[1] = -1;

   while (1) {
      if (readCell()) {
         calculateAverage();
         currentState.fGrainsCurrent = getGrains();
         currentState.fGramsCurrent = getGrams();
         if (++u % 5 == 0) {
            displayUpdate_(&currentState);
         }
      }
      HAL_Delay(10);

      // Read Buttons
      uButton = getButton();
      if (uButton < 16) {
         cButton = convertButtonChar(uButton);
         // Green:      Run/Go/Enter
         // Red:        Stop/kill/done
         // F1:         Tare
         // F2:         Calibrate
         // F3:         Run Load
         if (cButton == 'r') {
            // Stop everything, quit whatever program...red is kill button
            if (currentState.bPWM0_Running) {
               servoStop(PWM0);
               currentState.bPWM0_Running = FALSE;
            }
            CommandQueue.iHi = -1;
            stepperEnable(&motorList[0], FALSE);
            stepperEnable(&motorList[1], FALSE);
            currentState.stepTrickle = STEPPER_OFF;
            // Go back to main menu
            if (currentState.scaleMode == SCALE_MODE_GRAIN) currentState.displayMode = SCALE_GRAIN;
            else currentState.displayMode = SCALE_GRAM;
            if (currentState.displayMode != MAIN_MENU) {
               Display_Mode prev = currentState.displayMode;
               currentState.displayMode = MAIN_MENU;
               displaySwitchMode(&currentState, prev);
            }
         } else if (cButton == 'a') {
            setTare();
         } else if (cButton == 'b') {
         } else if (cButton == 'c') {
         } else if (cButton == 'd') {
            int i = ++CommandQueue.iStep[0];
            CommandQueue.Q_Step[0][i].uCommand = STEP_CHAMBER_DUMP;
            CommandQueue.Q_Step[0][i].uButton[0] = currentState.iMeasureDumpDwell;
            i = ++CommandQueue.iStep[1];
            CommandQueue.Q_Step[1][i].uCommand = STEP_CHAMBER_DUMP;
            CommandQueue.Q_Step[1][i].uButton[0] = currentState.iMeasureDumpDwell;
         } else {
            runQueue(&currentState, uButton);
            uButton = convertButton(uButton);
            displayHandleMenu(&currentState, uButton, cButton);
         }
      } else {
         // Run current program
         runQueue(&currentState, 255);
      }
      runStepperQueue(&currentState, &motorList[0], CommandQueue.Q_Step[0], &CommandQueue.iStep[0]);
      runStepperQueue(&currentState, &motorList[1], CommandQueue.Q_Step[1], &CommandQueue.iStep[1]);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8192;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8192;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 212;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 255;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 375;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ST0_M2_Pin|CELL_CLK_Pin|ST0_M3_Pin|ST2_M3_Pin
                          |ST0_M0_Pin|ST0_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST0_STBY_Pin|ST2_EN_Pin|ST0_EN_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST2_STBY_Pin|ST2_M1_Pin|ST2_M2_Pin|ST2_M0_Pin
                          |ST1_EN_Pin|ST1_STBY_Pin|ST1_M0_Pin|ST1_M1_Pin
                          |ST1_M2_Pin|ST1_M3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST0_M2_Pin ST0_M3_Pin ST2_M3_Pin RST_Pin
                           ST0_M0_Pin ST0_M1_Pin */
  GPIO_InitStruct.Pin = ST0_M2_Pin|ST0_M3_Pin|ST2_M3_Pin|RST_Pin
                          |ST0_M0_Pin|ST0_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CELL_DAT_Pin */
  GPIO_InitStruct.Pin = CELL_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CELL_DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CELL_CLK_Pin */
  GPIO_InitStruct.Pin = CELL_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CELL_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST0_STBY_Pin ST2_EN_Pin CS_Pin ST0_EN_Pin
                           PA15 */
  GPIO_InitStruct.Pin = ST0_STBY_Pin|ST2_EN_Pin|CS_Pin|ST0_EN_Pin
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin ST2_STBY_Pin ST2_M1_Pin ST2_M2_Pin
                           ST2_M0_Pin ST1_EN_Pin ST1_STBY_Pin ST1_M0_Pin
                           ST1_M1_Pin ST1_M2_Pin ST1_M3_Pin */
  GPIO_InitStruct.Pin = DC_Pin|ST2_STBY_Pin|ST2_M1_Pin|ST2_M2_Pin
                          |ST2_M0_Pin|ST1_EN_Pin|ST1_STBY_Pin|ST1_M0_Pin
                          |ST1_M1_Pin|ST1_M2_Pin|ST1_M3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   for (int i = 0; i < 2; i++) {
      if (htim == motorList[i].htim) {
         if (motorList[i].bStepOnTimer) {
            HAL_GPIO_WritePin(motorList[i].gpio_M2, motorList[i].m2, GPIO_PIN_RESET); //m2 == m_clk
            timerDelay(16);//that updates timer stuff without the config...

            if (motorList[i].timerIndexCurrent++ == motorList[i].timerIndexStop) {
               if (motorList[i].tableIndexCurrent == 0) {
                  motorList[i].bStepOnTimer = FALSE;
                  stepperEnable(&motorList[i], FALSE);
                  HAL_TIM_Base_Stop_IT(motorList[i].htim);
                  return;
               }

               motorList[i].timerIndexCurrent = 0;
               motorList[i].timerIndexStop = motorList[i].arStepper_Table[--motorList[i].tableIndexCurrent].iNumSteps;
               motorList[i].bStepOnTimer = motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].bStep;
               if (motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].bClockwise) {
                  stepperSetCW(&motorList[i]);
               } else {
                  stepperSetCCW(&motorList[i]);
               }

               __HAL_TIM_SET_AUTORELOAD(htim, motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].uNewDelay);
               __HAL_TIM_SET_COUNTER(htim, 0xffffffff);
            }
            HAL_GPIO_WritePin(motorList[i].gpio_M2, motorList[i].m2, GPIO_PIN_SET);
         } else {
            if (motorList[i].timerIndexCurrent++ == motorList[i].timerIndexStop) {
               if (motorList[i].tableIndexCurrent == 0) {
                  motorList[i].bStepOnTimer = FALSE;
                  stepperEnable(&motorList[i], FALSE);
                  HAL_TIM_Base_Stop_IT(motorList[i].htim);
                  return;
               }

               motorList[i].timerIndexCurrent = 0;
               motorList[i].timerIndexStop = motorList[i].arStepper_Table[--motorList[i].tableIndexCurrent].iNumSteps;
               motorList[i].bStepOnTimer = motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].bStep;
               if (motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].bClockwise) {
                  stepperSetCW(&motorList[i]);
               } else {
                  stepperSetCCW(&motorList[i]);
               }

               __HAL_TIM_SET_AUTORELOAD(htim, motorList[i].arStepper_Table[motorList[i].tableIndexCurrent].uNewDelay);
               __HAL_TIM_SET_COUNTER(htim, 0xffffffff);
            }
         }
      }
   }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

