/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OneWire.h"
#include "i2c_slave.h"
#include "reg_map.h"
#include "sw_timers.h"
#include "switches.h"
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CONTROL_BOARD_REGS_t regInstPrev;
CONTROL_BOARD_REGS_t *pRegPrev = &regInstPrev;
CONTROL_BOARD_REGS_t *pReg;
extern map_t map;

CAPTURE_CHANNELS_t IC_ch[4] = { 0 };

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} PortPin;

static const PortPin portPinMap[] = { { GPIOA, GPIO_PIN_2 },  { GPIOA, GPIO_PIN_3 },
                                      { GPIOB, GPIO_PIN_3 },  { GPIOB, GPIO_PIN_5 },
                                      { GPIOB, GPIO_PIN_6 },  { GPIOB, GPIO_PIN_7 },
                                      { GPIOB, GPIO_PIN_13 }, { GPIOB, GPIO_PIN_15 },
                                      { GPIOD, GPIO_PIN_2 } };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t ADC_raw[10];
float ADC_Volt[10];
float ADC_data[10];
uint8_t adcChAmount = 5;
uint8_t off_request = 0;

uint8_t BoxOVT_LIM = 50; //*C
uint8_t BoxOVT_OFF = 60; //*C

uint8_t OVC_Lim = 12; // A
uint8_t OVC_Off = 15; // A

uint8_t OV_Lim = 33;  // V
uint8_t OVT_Off = 35; // V

uint8_t UV_Lim = 23;  // V
uint8_t UVT_Off = 21; // V

uint32_t fanThPeriod = 10;
uint32_t i2cTimeout = 100;

uint16_t pressTimeToOff = 5000; // ms
uint16_t BtsInResetHoldTime = 10000;
uint16_t BtsPowerOnDelayTime = 1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void PaStatusGet();
void PaSet();
void PAControl(uint8_t addres);

void FanSet();
void FanGet();

void ADCproces();

void calculatePeriod(uint8_t in);
void Btn_Hndl(void);

SoftTimer FanSpeedUpdTim;

SoftTimer PaStatusUpdTim;
void PaStatusGet(void);

SoftTimer fanControlTim;
void fanControl(void);

SoftTimer FanOffTim;
void FanOff_Hndl(void);

SoftTimer PowerOffDelay_Tim;
void PowerOffDelay_Hndl(void);

SoftTimer BlinkTim;
void Blink_Hndl(void);

SoftTimer AdcTim;

SoftTimer ProtectionTim;
void Protection_Hndl(void);

SoftTimer blinkUpdTim;
void blinkUpd_Hndl(void);

SoftTimer captureWait;
void CaptureRdy_Hndl(void);

SoftTimer gpioControlTim;
void GpioControl(void);

SoftTimer antSwithTim;
void antSwith_Hndl(void);

SoftTimer antSwithUpdateTim;
void antSwithUpdate_Hndl(void);

void FanSet(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define REG_CHANGED(REG) ON_CHANGE(&pRegPrev->REG, &pReg->REG)

uint8_t ON_CHANGE(uint8_t *regPrev, uint8_t *reg)
{
    uint8_t changeF = 0;
    if (*regPrev != *reg)
    {
        *regPrev = *reg;
        changeF = 1;
    }
    return changeF;
}

volatile uint8_t rx_data[16];
uint8_t Status = 0;
float Pavg_pad = 0;
double swr = 0;
uint16_t T = 0;

// Timer handlers ===========================================
void Blink_Hndl(void)
{ /*led_Toggle; */
}

void PowerOffDelay_Hndl(void)
{
    BIT_SET(pReg->CB.POW_OFF_FLAG, POW_OFF);
    CreateTimer(&BlinkTim, 1, 0, 1000, DONE, Blink_Hndl); // Stop blinking
}

void FanOff_Hndl(void)
{
    pReg->CB.FAN_PWM = 0;
    BIT_SET(pReg->CB.MAIN_CONTROL, FAN_AUTO); // If temperature is ok - switch off
}

void antSwith_Hndl(void)
{
    static int pos = 0;
    static int switchIdx = 0;
    static const SwitchEntity *sw = NULL;
    if (!sw) // First init
        sw = getSwitchEntityByIndex(switchIdx);

    if (REG_CHANGED(CB.SWITCH_ENTITY_IDX))
    {
        switchIdx = pReg->CB.SWITCH_ENTITY_IDX;
        sw = getSwitchEntityByIndex(switchIdx);
        pos = 0;
    }

    if (BIT_CHECK(pReg->CB.SWITCH_CONTROL, SWITCH_CONTROL_RUN))
    {
        SetAntennaSwitch(sw, pos);
        pos = (pos < sw->numConfigs - 1) ? pos + 1 : 0;
    }
    else if (BIT_CHECK(pReg->CB.SWITCH_CONTROL, SWITCH_CONTROL_MANUAL))
    {
        uint8_t manualPos = pReg->CB.SWITCH_MANUAL_POS;
        SetAntennaSwitch(sw, manualPos);
    }

    led_Toggle;
}

void antSwithUpdate_Hndl(void)
{
    if (REG_CHANGED(CB.SWITCH_DELAY_MS_L) || REG_CHANGED(CB.SWITCH_DELAY_MS_H))
    {
        uint8_t periodH = pReg->CB.SWITCH_DELAY_MS_H;
        uint8_t periodL = pReg->CB.SWITCH_DELAY_MS_L;
        uint16_t period = (periodH << 8) | periodL;
        CreateTimer(&antSwithTim, 1, 0, period, ACTIVE, antSwith_Hndl);
    }
}

//TODO:
// Add pause, run, stop, step pins (step active in pause mode). Stop is more prioritive then pause
// Add uart interface
// Add LCD with statuses
// Add control by buttons
// Add control by FPV joystic
// Add USB CDC interface

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    SoftTimersInit();

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick.
     */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    hadc1.Instance->CR2 |= ADC_CR2_CAL; // adc calibrate
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_raw, adcChAmount);

    I2C1_Slave_init(); // Also clears the register map
    /////////////////////////////////////////////////

    // init reg map
    pReg = &map.Reg;

    pReg->CB.TEMPERATURE_TH = 36;
    pReg->CB.MIN_PWM = 60;
    pReg->CB.FAN_PWM = 250;

    // BIT_CLEAR(pReg->CB.MAIN_CONTROL, FAN_AUTO);
    // FanSet();
    // fan_on;

    ADCproces();
    HAL_Delay(500);

    pReg->CB.SWITCH_DELAY_MS_L = 0;
    pReg->CB.SWITCH_DELAY_MS_H = 2; // 512ms default switch time
    BIT_SET(pReg->CB.SWITCH_CONTROL, SWITCH_CONTROL_RUN);

    // Pow off in case of invalid ADC values, or normal ON
    // Protection_Hndl();

    //	Timer,	Rewrite, time, period, 	state, handler
    //	CreateTimer(&FanSpeedUpdTim, 0, 0, 		2000, 	ACTIVE, FanGet);
    // CreateTimer(&PaStatusUpdTim, 0, 100, 100, ACTIVE, PaStatusGet);
    // CreateTimer(&BlinkTim, 1, 150, 1000, ACTIVE, Blink_Hndl);
    // CreateTimer(&blinkUpdTim, 0, 0, 100, ACTIVE, blinkUpd_Hndl);
    //  CreateTimer(&AdcTim, 1, 0, 500, ACTIVE, ADCproces);
    //  CreateTimer(&ProtectionTim, 0, 0, 50, ACTIVE, Protection_Hndl);

    // If temperature is ok - switch off
    // CreateTimer(&fanControlTim, 0, 200, 200, ACTIVE, fanControl);

    // After start clean the dust and stop till temperature rise!
    // CreateTimer(&FanOffTim, 0, 2000, 0, ACTIVE, FanOff_Hndl);

    // Update output val of GPIO
    CreateTimer(&gpioControlTim, 0, 0, 10, ACTIVE, GpioControl);
    CreateTimer(&antSwithTim, 1, 150, 1000, ACTIVE, antSwith_Hndl);
    CreateTimer(&antSwithUpdateTim, 0, 150, 1, ACTIVE, antSwithUpdate_Hndl);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // int scanRes = 1;
    // for (int i = 0; i < 127; i++)
    // {
    //     scanRes = HAL_I2C_Master_Receive(&hi2c2, i << 1, (uint8_t
    //     *)&rx_data[0], 1, i2cTimeout); if (scanRes == 0)
    //         validAddr = i;
    // }

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        ProcessTimer();

        // PaSet();
        // Btn_Hndl();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_AnalogWDGConfTypeDef AnalogWDGConfig = { 0 };
    ADC_ChannelConfTypeDef sConfig = { 0 };

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 5;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analog WatchDog 1
     */
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    AnalogWDGConfig.HighThreshold = 0;
    AnalogWDGConfig.LowThreshold = 0;
    AnalogWDGConfig.Channel = ADC_CHANNEL_6;
    AnalogWDGConfig.ITMode = ENABLE;
    if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    /* USER CODE BEGIN TIM1_Init 1 */

    // TIM_IC_InitTypeDef sIcConfig = {0};

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 2000; // 2000 = 10kHz
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
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC,
                      GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 |
                          GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                          GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_12 |
                          GPIO_PIN_15,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC13 PC14 PC15 PC0
                             PC1 PC2 PC3 PC4
                             PC5 PC6 PC7 PC8
                             PC9 PC10 PC11 PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 |
                          GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                          GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 PA2 PA3
                             PA7 PA12 PA15 */
    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB12 PB13 PB15
                             PB3 PB4 PB5 PB6
                             PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void Protection_Hndl()
{
    // 1-st stage - Set alarm flags
    (pReg->CB.CURRENT_I > OVC_Lim) ? BIT_SET(pReg->CB.ALARM_1_FLAG, OVC)
                                   : BIT_CLEAR(pReg->CB.ALARM_1_FLAG, OVC);
    (pReg->CB.VOLTAGE_SUPPLY_I > OV_Lim) ? BIT_SET(pReg->CB.ALARM_1_FLAG, OVT)
                                         : BIT_CLEAR(pReg->CB.ALARM_1_FLAG, OVT);
    (pReg->CB.VOLTAGE_SUPPLY_I < UV_Lim) ? BIT_SET(pReg->CB.ALARM_1_FLAG, UVT)
                                         : BIT_CLEAR(pReg->CB.ALARM_1_FLAG, UVT);
    (pReg->CB.PEAK_TEMPERATURE > BoxOVT_LIM)
        ? BIT_SET(pReg->CB.OVER_TEMP_FLAG, BOX_OVT)
        : BIT_CLEAR(pReg->CB.OVER_TEMP_FLAG, BOX_OVT); // 0-bit = boxTemp

    // 2-nd stage - Emergency off (If Enabled)
    if (pReg->CB.PROTECTION == 0xFF)
    {
        if ((pReg->CB.CURRENT_I > OVC_Off) || (pReg->CB.VOLTAGE_SUPPLY_I > OVT_Off) ||
            (pReg->CB.VOLTAGE_SUPPLY_I < UVT_Off) || (pReg->CB.PEAK_TEMPERATURE > BoxOVT_OFF))
        {
            // Sequentialy off all systems
            if (BIT_CHECK(pReg->CB.MAIN_CONTROL, MAIN_POW_28_EN))
            {
                BIT_CLEAR(pReg->CB.MAIN_CONTROL, MAIN_POW_28_EN);
            }
            else if (BIT_CHECK(pReg->CB.MAIN_CONTROL, BTS_3_EN))
            {
                BIT_CLEAR(pReg->CB.MAIN_CONTROL, BTS_3_EN);
            }
            else if (BIT_CHECK(pReg->CB.MAIN_CONTROL, BTS_2_EN))
            {
                BIT_CLEAR(pReg->CB.MAIN_CONTROL, BTS_2_EN);
            }
            else if (BIT_CHECK(pReg->CB.MAIN_CONTROL, BTS_1_EN)) // Total off
            {
                BIT_CLEAR(pReg->CB.MAIN_CONTROL, BTS_1_EN);
                BIT_CLEAR(pReg->CB.MAIN_CONTROL, SUPPLY_EN);
            }
        }
    }
}

void blinkUpd_Hndl(void)
{
    if (((pReg->CB.TEMP_BTS_1 == 0) || (pReg->CB.TEMP_BTS_2 == 0) || (pReg->CB.TEMP_BTS_3 == 0)) &&
        (BIT_CHECK(pReg->CB.POW_OFF_FLAG, POW_OFF) == 0))
    {
        if (GetTimerState(&BlinkTim) != ACTIVE)
        {
            CreateTimer(&BlinkTim, 0, 0, 1000, ACTIVE, Blink_Hndl); // Set if DONE
        }
    }
    else // All BTS ready and send their temperatures
    {
        if (BIT_CHECK(pReg->CB.POW_OFF_FLAG, POW_OFF) == 0) // Not in powerOff proc
        {
            CreateTimer(&BlinkTim, 1, 0, 1000, DONE, Blink_Hndl); // Stop blinking
            led_on;
        }
        else // Pow off proc, but min 1 of bts is not active and Timer is active?
             // so stop timer and blink from PowOffControl
        {
            CreateTimer(&BlinkTim, 1, 0, 1000, DONE, Blink_Hndl); // Stop blinking
        }
    }
}

/*
void PaStatusGet() {
  uint8_t packSz = 16; // Get 16 regs from PA
  uint8_t paFailCnt = 0;

  foreach_pa() {
    uint8_t res = 1;
    memset(rx_data, 0, packSz);
    res = HAL_I2C_Master_Transmit(&hi2c2, (pReg->PA[i].INST_I2C_ADR) << 1,
                                  (uint8_t *)&rx_data[0], 1,
                                  i2cTimeout); // Write 0
    res += HAL_I2C_Master_Receive(&hi2c2, (pReg->PA[i].INST_I2C_ADR) << 1,
                                  (uint8_t *)&rx_data[0], packSz, i2cTimeout);
    if (res == 0) {
      memcpy((void *)&pReg->PA[i], rx_data, packSz - 1); // Cpy 15 regs of PA

      if (pReg->PA[i].TEMPERATURE >
          pReg->PA[i].OVER_TEMP_SET) // TODO convert temp
        BIT_SET(pReg->CB.OVER_TEMP_FLAG,
                (i + 1)); // 0-bit = boxTemp, others - PA
      else
        BIT_CLEAR(pReg->CB.OVER_TEMP_FLAG,
                  (i + 1)); // 0-bit = boxTemp, others - PA

      pReg->PA[i].POWER_I = (uint8_t)Pavg_pad;
      pReg->PA[i].POWER_F =
          (uint8_t)((Pavg_pad - (float)pReg->PA[i].POWER_I) * 100);

      pReg->PA[i].SWR_I = (uint8_t)swr;
      pReg->PA[i].SWR_F = (uint8_t)((swr - (float)pReg->PA[i].SWR_I) * 100);
    } else // Slave not responding
    {
      pReg->PA[i].STATUS = 0;
    }
  }

  // Crutch to make PAs work
  foreach_pa() {
    if (pReg->PA[i].STATUS == 0)
      paFailCnt++;
  }

  if (paFailCnt == PA_AMOUNT) // All failed, so this is i2c bus problem
  {
    HAL_I2C_DeInit(&hi2c2);
    MX_I2C2_Init();
  }
}
*/
void PaSet()
{
    uint8_t reg;
    foreach_pa()
    {
        if (REG_CHANGED(PA[i].SET_TDD))
        {
            reg = pReg->PA[i].SET_TDD;
            switch (i)
            {
            case 0:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD1_on : TDD1_off;
            }
            break;
            case 1:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD2_on : TDD2_off;
            }
            break;
            case 2:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD3_on : TDD3_off;
            }
            break;
            case 3:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD4_on : TDD4_off;
            }
            break;
            case 4:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD5_on : TDD5_off;
            }
            break;
            case 5:
            {
                BIT_CHECK(reg, TDD_EN) ? TDD6_on : TDD6_off;
            }
            break;

            default:
                break;
            }
        } // SET_TDD

        if (REG_CHANGED(PA[i].SHUTDOWN))
        {
            reg = pReg->PA[i].SHUTDOWN;
            switch (i)
            {
            case 0:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA1_Shutdown_on : PA1_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA1_power_on : PA1_power_off;
            }
            break;
            case 1:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA2_Shutdown_on : PA2_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA2_power_on : PA2_power_off;
            }
            break;
            case 2:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA3_Shutdown_on : PA3_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA3_power_on : PA3_power_off;
            }
            break;
            case 3:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA4_Shutdown_on : PA4_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA4_power_on : PA4_power_off;
            }
            break;
            case 4:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA5_Shutdown_on : PA5_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA5_power_on : PA5_power_off;
            }
            break;
            case 5:
            {
                BIT_CHECK(reg, PA_POW_5_EN) ? PA6_Shutdown_on : PA6_Shutdown_off;
                BIT_CHECK(reg, PA_POW_28_EN) ? PA6_power_on : PA6_power_off;
            }
            break;

            default:
                break;
            }
        } // SHUTDOWN
    }
}

void GpioControl()
{
    uint8_t reg;
    foreach_io()
    {
        if (REG_CHANGED(CB.IO.Idx[i]))
        {
            reg = pReg->CB.IO.Idx[i];
            if (i < sizeof(portPinMap) / sizeof(portPinMap[0]))
            {
                PortPin pp = portPinMap[i];
                BIT_CHECK(reg, GPIO_VAL)
                ? HAL_GPIO_WritePin(pp.port, pp.pin, GPIO_PIN_SET)
                : HAL_GPIO_WritePin(pp.port, pp.pin, GPIO_PIN_RESET);
            }
        }
    }
}

void ADCproces()
{
    for (int i = 0; i <= adcChAmount; i++)
        ADC_Volt[i] = ADC_raw[i] * (3.3 / 4095);

    ADC_data[0] = ADC_Volt[0] / 0.085; // 28V
    ADC_data[1] = ADC_Volt[1] * 11.9;  /// 0.12;		//Vin
    ADC_data[2] = ADC_Volt[2] * 2.35;  /// 0.05;		//Iin
    ADC_data[3] = ADC_Volt[3] / 0.085; // 12v

    ADC_data[4] = ((float)ADC_raw[4]) / 4095 * 3300; // MCU Temp
    ADC_data[4] = (((ADC_data[4] - 760.0) / 2.5) + 25) / 10;

    pReg->CB.VOLTAGE_28V_I = (uint8_t)ADC_data[0];
    pReg->CB.VOLTAGE_28V_F = (ADC_data[0] - pReg->CB.VOLTAGE_28V_I) * 100;

    pReg->CB.VOLTAGE_SUPPLY_I = (uint8_t)ADC_data[1];
    pReg->CB.VOLTAGE_SUPPLY_F = (ADC_data[1] - pReg->CB.VOLTAGE_SUPPLY_I) * 100;

    pReg->CB.CURRENT_I = (uint8_t)ADC_data[2];
    pReg->CB.CURRENT_F = (ADC_data[2] - pReg->CB.CURRENT_I) * 100;

    pReg->CB.VOLTAGE_12V_I = (uint8_t)ADC_data[3];
    pReg->CB.VOLTAGE_12V_F = (ADC_data[3] - pReg->CB.VOLTAGE_12V_I) * 100;

    // Not used, change to MAX_TEMPERATURE
    //	pReg->CB.PEAK_TEMPERATURE = (uint8_t) ADC_data[4];
    //	pReg->CB.TEMPERATURE_F = (ADC_data[4] - pReg->CB.PEAK_TEMPERATURE)*100;
}

uint8_t GetMaxTemp()
{
    int16_t maxTemp = -100;
    foreach_pa()
    {
        if ((pReg->PA[i].STATUS != 0) && (pReg->PA[i].POWER_mW > 0) && (pReg->PA[i].SWR > 0) &&
            (BIT_CHECK(pReg->PA[i].SHUTDOWN, PA_POW_5_EN)))
        {
            maxTemp = MAX(maxTemp, pReg->PA[i].TEMPERATURE); // TODO convert
        }
    }
    maxTemp = MAX(maxTemp, pReg->CB.TEMP_BTS_1);
    maxTemp = MAX(maxTemp, pReg->CB.TEMP_BTS_2);
    maxTemp = MAX(maxTemp, pReg->CB.TEMP_BTS_3);

    pReg->CB.PEAK_TEMPERATURE = (int8_t)maxTemp;
    return maxTemp;
}

void FanSet()
{
    uint16_t pwmVal;
    uint8_t reg;
    uint8_t scaleCoef = 0;
    uint8_t maxPwm = 255;
    uint8_t temp = GetMaxTemp();

    if (pReg->CB.TEMPERATURE_TH > BoxOVT_LIM)
    {
        pReg->CB.TEMPERATURE_TH = BoxOVT_LIM - 1;
    } // Protection
    scaleCoef = (maxPwm - pReg->CB.MIN_PWM) / (BoxOVT_LIM - pReg->CB.TEMPERATURE_TH);

    if (temp > pReg->CB.TEMPERATURE_TH)
    {
        pwmVal = pReg->CB.MIN_PWM + (temp - pReg->CB.TEMPERATURE_TH) * scaleCoef;
        if (pwmVal > maxPwm)
        {
            pwmVal = maxPwm;
        }
    }
    else
    {
        pwmVal = 0;
    }

    if (BIT_CHECK(pReg->CB.MAIN_CONTROL,
                  FAN_AUTO)) // Control board controlls fan speed depending of
                             // temperature
    {
        pReg->CB.FAN_PWM = (uint8_t)(pwmVal & 0xFF);
    }

    if (REG_CHANGED(CB.FAN_PWM))
    {
        reg = pRegPrev->CB.FAN_PWM;

        if (reg == 0xFF)
        {
            htim1.Instance->CCR2 = htim1.Init.Period - 1;
        }
        else
        {
            // htim1.Instance->CCR2 = (htim1.Init.Period-1)/255*(reg+1);
            htim1.Instance->CCR2 = reg * 7; // Scale to htim1.Init.Period = 2000; //2000 = 10kHz
        }
        htim1.Instance->EGR |= TIM_EGR_UG;
    }
}

void FanGet()
{
    uint32_t fanSpeed = 0;
    fanSpeed = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
    fanSpeed = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
    fanSpeed = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
    fanSpeed = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);

    pReg->CB.FAN_ALARM_FLAG = 0; // TODO need debug
}

void captureStart(void)
{
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    for (int i = 0; i < 4; i++)
    {
        memset(&IC_ch[i], 0, sizeof(CAPTURE_CHANNELS_t));
    }
}

void captureInput(uint8_t in)
{
    if (IC_ch[in].Cnt < UINT16_MAX)
    {
        IC_ch[in].Cnt++;
    }

    if (IC_ch[in].State_f == IDLE_CAPTURE)
    {
        IC_ch[in].CCValPrev = 0;
        IC_ch[in].CCVal = htim1.Instance->CNT;
        IC_ch[in].State_f = FIRST_CAPTURE;
    }
    else if (IC_ch[in].State_f == FIRST_CAPTURE)
    {
        IC_ch[in].CCValPrev = IC_ch[in].CCVal;
        IC_ch[in].CCVal = htim1.Instance->CNT;
        IC_ch[in].State_f = SECOND_CAPTURE;
    }
}

uint32_t uint32_time_diff(uint32_t now, uint32_t before)
{
    return (now >= before) ? (now - before) : (UINT32_MAX - before + now);
}

void calculatePeriod(uint8_t in)
{
    if (IC_ch[in].State_f == SECOND_CAPTURE)
    {
        IC_ch[in].Period = uint32_time_diff(IC_ch[in].CCVal, IC_ch[in].CCValPrev);
        IC_ch[in].CCValPrev = 0;
        IC_ch[in].CCVal = 0;
        IC_ch[in].State_f = IDLE_CAPTURE;
    }
}

void captureStop(void)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void fanControl(void)
{
    FanSet(); // Update val

    captureStart();
    CreateTimer(&captureWait, 0, 100, 0, ACTIVE, CaptureRdy_Hndl);
}

void CaptureRdy_Hndl(void)
{
    captureStop();
    for (int i = 0; i < 4; i++)
    {
        calculatePeriod(i);
        if (IC_ch[i].State_f == IDLE_CAPTURE)
        {
            if (IC_ch[i].Period < fanThPeriod)
            {
                BIT_SET(pReg->CB.FAN_ALARM_FLAG, i);
            }
            else
            {
                BIT_CLEAR(pReg->CB.FAN_ALARM_FLAG, i);
            }
        }
        else
        {
            memset(&IC_ch[i], 0, sizeof(CAPTURE_CHANNELS_t));
            BIT_SET(pReg->CB.FAN_ALARM_FLAG, i);
        }
    }
}

void Btn_Hndl(void)
{
    if (REG_CHANGED(CB.POW_OFF_FLAG))
    {
        if (BIT_CHECK(pReg->CB.POW_OFF_FLAG, BTN_PRESSED)) // Pressed
        {
            // To prevent 2-nd start on POW_OFF bit set by PowerOffDelay_Hndl
            if (BIT_CHECK(pReg->CB.POW_OFF_FLAG, POW_OFF) == 0)
            {
                // Start off timer
                CreateTimer(&PowerOffDelay_Tim, 0, pressTimeToOff, 0, ACTIVE, PowerOffDelay_Hndl);
            }
        }
        else // Button was hold less than pressTimeToOff time - clear timer,cancel
             // off procedure
        {
            if (GetTimerState(&PowerOffDelay_Tim) != DONE)
            {
                // RESET timer on release btn
                CreateTimer(&PowerOffDelay_Tim, 1, pressTimeToOff, 0, DONE, PowerOffDelay_Hndl);
                BIT_CLEAR(pReg->CB.POW_OFF_FLAG, POW_OFF);
                off_request = 0;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2) //
    {
        if (HAL_GPIO_ReadPin(Button_pin) == 1) // Pressed
        {
            BIT_SET(pReg->CB.POW_OFF_FLAG, BTN_PRESSED);
        }
        else
        {
            BIT_CLEAR(pReg->CB.POW_OFF_FLAG, BTN_PRESSED);
        }
    }

    // Input capture
    switch (GPIO_Pin)
    {
    case GPIO_PIN_8:
    {
        captureInput(0);
    }
    break;
    case GPIO_PIN_9:
    {
        captureInput(1);
    }
    break;
    case GPIO_PIN_10:
    {
        captureInput(2);
    }
    break;
    case GPIO_PIN_11:
    {
        captureInput(3);
    }
    break;
    default:
        break;
    }
    //__NOP();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim4.Instance)
    {
        // supply_off
        //		i2c1_ram[101]= 1;
        //		off_request=1;
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

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
