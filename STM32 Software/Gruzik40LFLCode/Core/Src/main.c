/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "app_fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Eeprom.h"
#include "ICM-42688p.h"
#include "Line_Follower.h"
#include "LowPassFilter.h"
#include "RingBuffer.h"
#include "SimpleParser.h"
#include "map.h"
#include "motor.h"
#include "odometry.h"
#include "robot_config.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_SPEED_FILTER_ALPHA 0.35f
#define ROBOT_TELEMETRY_PERIOD_MS 100u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SDReadingReady = 0u;
Eeprom_t eeprom;
FRESULT FatFsResult;
FATFS SdFatFs;
FIL SdCardFile;

LineFollower_t GRUZIK;
motor_t Motor_L;
motor_t Motor_R;
ICM_t ICM;
Map_t map;
Odometry_t RobotOdom;

uint8_t RxData;
RingBuffer_t ReceiveBuffer;
uint8_t ReceivedData[PARSER_LINE_BUFFER_SIZE];
volatile uint8_t ReceivedLines;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Robot_LoadDefaultSettings(void);
static void Robot_StartPeripherals(void);
static void Robot_ResetEncoderCounters(void);
static void Robot_SampleSensorsWhenStopped(void);
static void Robot_UpdateOdometryTick(void);
static void Robot_UpdateSpeedStats(void);
static void Robot_StopOutputs(void);
static void Robot_ServiceTelemetry(void);
static void Robot_SendOdomTelemetry(void);
static void Robot_SendDebugTelemetry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM20_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  Robot_LoadDefaultSettings();
  Robot_StartPeripherals();

  FatFsResult = f_mount(&SdFatFs, "", 1);
  SDReadingReady = (FatFsResult == FR_OK) ? 1u : 0u;

  GRUZIK.PowerMode = Stop;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t led_timer = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() > (led_timer + 500u))
    {
        led_timer = HAL_GetTick();
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    if (ReceivedLines > 0u)
    {
        Parser_TakeLine(&ReceiveBuffer, ReceivedData);
        Parser_Parse(ReceivedData, &GRUZIK);
        ReceivedLines--;
    }

    Robot_ServiceTelemetry();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void Robot_LoadDefaultSettings(void)
{
    GRUZIK.PowerMode = Initialization;
    GRUZIK.state = PidFollowing;
    GRUZIK.Kp = 0.015f;
    GRUZIK.Kd = 0.55f;
    GRUZIK.Base_speed_R = 125.0f;
    GRUZIK.Base_speed_L = 125.0f;
    GRUZIK.Max_speed_R = 200.0f;
    GRUZIK.Max_speed_L = 200.0f;
    GRUZIK.Sharp_bend_speed_right = -75.0f;
    GRUZIK.Sharp_bend_speed_left = 120.0f;
    GRUZIK.Bend_speed_right = -75.0f;
    GRUZIK.Bend_speed_left = 120.0f;
    GRUZIK.Speed_offset = 0.014f;
    GRUZIK.Speed_level = 1.0f;
    GRUZIK.treshold = 3300u;
    GRUZIK.LastEndTimer = HAL_GetTick();

    map.p = 90.0f;
    map.i = 0.0f;
    map.d = 10.0f;
    map.DefaultPlaybackSpeed = 80.0f;
    Map_Reset(&map);

    Motor_Init(&Motor_R, 0.0f, 0.0f);
    Motor_Init(&Motor_L, 0.0f, 0.0f);
    Motor_SetEncoderDirection(&Motor_L, ROBOT_LEFT_ENCODER_DIRECTION);
    Motor_SetEncoderDirection(&Motor_R, ROBOT_RIGHT_ENCODER_DIRECTION);

    LowPassFilter_Init(&Motor_R.EncoderRpmFilter, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_L.EncoderRpmFilter, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_L.MetersPerSecondLPF, ENCODER_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_R.MetersPerSecondLPF, ENCODER_SPEED_FILTER_ALPHA);

    Odometry_Init(&RobotOdom, ROBOT_ODOMETRY_GYRO_WEIGHT);
}

static void Robot_StartPeripherals(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

    (void)ICM_Init(&ICM, &hspi3);
    (void)ICM_CalibrateGyroZ(&ICM, 300u, 1u);

    HAL_TIM_Base_Start_IT(&htim20);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&GRUZIK.Adc1_Values, 9);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&GRUZIK.Adc2_Values, 4);
    HAL_UART_Receive_IT(&huart1, &RxData, 1);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    Robot_ResetEncoderCounters();
}

static void Robot_ResetEncoderCounters(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, ROBOT_ENCODER_COUNTER_CENTER);
    __HAL_TIM_SET_COUNTER(&htim1, ROBOT_ENCODER_COUNTER_CENTER);
}

static void Robot_SampleSensorsWhenStopped(void)
{
    Motor_L.EncoderValue = __HAL_TIM_GET_COUNTER(&htim4);
    Motor_R.EncoderValue = __HAL_TIM_GET_COUNTER(&htim1);
    Robot_ResetEncoderCounters();

    Motor_CalculateSpeed(&Motor_L);
    Motor_CalculateSpeed(&Motor_R);
    (void)ICM_Read_GyroZ(&ICM);

    if ((fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M) &&
        (fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M) &&
        (fabsf(ICM.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS))
    {
        ICM_UpdateGyroZBias(&ICM, ROBOT_GYRO_BIAS_LEARN_RATE);
    }
}

static void Robot_UpdateOdometryTick(void)
{
    Motor_L.EncoderValue = __HAL_TIM_GET_COUNTER(&htim4);
    Motor_R.EncoderValue = __HAL_TIM_GET_COUNTER(&htim1);
    Robot_ResetEncoderCounters();

    Motor_CalculateSpeed(&Motor_L);
    Motor_CalculateSpeed(&Motor_R);

    uint8_t imu_ok = ICM_Read_GyroZ(&ICM);
    float gyro_z_dps = ICM.Gyro_Z;

    if (imu_ok == 0u)
    {
        float encoder_yaw_rate = (Motor_R.DistanceInMeasurement - Motor_L.DistanceInMeasurement) / ROBOT_WHEEL_BASE_M;
        gyro_z_dps = (encoder_yaw_rate / ROBOT_CONTROL_DT_S) * RAD_TO_DEG;
    }

    if ((fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M) &&
        (fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M) &&
        (fabsf(ICM.Gyro_Z) < ROBOT_GYRO_STATIONARY_DPS))
    {
        ICM_UpdateGyroZBias(&ICM, ROBOT_GYRO_BIAS_LEARN_RATE);
    }

    Odometry_Update(&RobotOdom, Motor_L.DistanceInMeasurement, Motor_R.DistanceInMeasurement, gyro_z_dps, ROBOT_CONTROL_DT_S);
    GRUZIK.Yaw = RobotOdom.pose.yaw_rad;
    GRUZIK.Orientation = RobotOdom.pose.yaw_rad * RAD_TO_DEG;
}

static void Robot_UpdateSpeedStats(void)
{
    float current_speed = 0.5f * (Motor_R.MetersPerSecondLPF.output + Motor_L.MetersPerSecondLPF.output);
    if (current_speed < 0.0f)
    {
        current_speed = -current_speed;
    }

    GRUZIK.AverageSpeedSum += current_speed;
    GRUZIK.AverageSpeedNum += 1.0f;
    GRUZIK.AverageSpeed = GRUZIK.AverageSpeedSum / GRUZIK.AverageSpeedNum;

    if (current_speed > GRUZIK.MaxSpeed)
    {
        GRUZIK.MaxSpeed = current_speed;
    }
}

static void Robot_StopOutputs(void)
{
    motor_control(&GRUZIK, 0.0f, 0.0f);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
}

static void Robot_ServiceTelemetry(void)
{
    static uint32_t last_telemetry_ms = 0u;

    if (TelemetryMode == TELEMETRY_OFF)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_telemetry_ms) < ROBOT_TELEMETRY_PERIOD_MS)
    {
        return;
    }
    last_telemetry_ms = now;

    if (TelemetryMode == TELEMETRY_DEBUG)
    {
        (void)LineFollower_UpdateSensors(&GRUZIK);
        Robot_SendOdomTelemetry();
        Robot_SendDebugTelemetry();
    }
    else
    {
        Robot_SendOdomTelemetry();
    }
}

static void Robot_SendOdomTelemetry(void)
{
    char tx[192];
    int len = snprintf(tx, sizeof(tx),
                       "ODOM,%.4f,%.4f,%.2f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
                       RobotOdom.pose.x_m,
                       RobotOdom.pose.y_m,
                       RobotOdom.pose.yaw_rad * RAD_TO_DEG,
                       RobotOdom.total_distance_m,
                       Motor_L.MetersPerSecondLPF.output,
                       Motor_R.MetersPerSecondLPF.output,
                       ICM.Gyro_Z,
                       RobotOdom.encoder_yaw_rad * RAD_TO_DEG,
                       RobotOdom.gyro_yaw_rad * RAD_TO_DEG);

    if ((len > 0) && (len < (int)sizeof(tx)))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 100);
    }
}

static void Robot_SendDebugTelemetry(void)
{
    char tx[320];
    int len = snprintf(tx, sizeof(tx),
                       "DBG,%d,%d,%d,%ld,%ld,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                       GRUZIK.SensorPosition,
                       GRUZIK.actives,
                       GRUZIK.Last_end,
                       (long)Motor_L.EncoderDeltaCounts,
                       (long)Motor_R.EncoderDeltaCounts,
                       Motor_L.EncoderRpmFilter.output,
                       Motor_R.EncoderRpmFilter.output,
                       Motor_L.DistanceTraveled,
                       Motor_R.DistanceTraveled,
                       ICM.Gyro_Z_RawDps,
                       ICM.Gyro_Z_BiasDps,
                       ICM.Gyro_Z,
                       GRUZIK.Adc1_Values[0],
                       GRUZIK.Adc1_Values[1],
                       GRUZIK.Adc1_Values[2],
                       GRUZIK.Adc1_Values[3],
                       GRUZIK.Adc1_Values[5],
                       GRUZIK.Adc1_Values[6],
                       GRUZIK.Adc1_Values[7],
                       GRUZIK.Adc1_Values[8],
                       GRUZIK.Adc2_Values[0],
                       GRUZIK.Adc2_Values[1],
                       GRUZIK.Adc2_Values[2],
                       GRUZIK.Adc2_Values[3]);

    if ((len > 0) && (len < (int)sizeof(tx)))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)len, 100);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (RB_Write(&ReceiveBuffer, RxData) == RB_OK)
        {
            if (RxData == ENDLINE)
            {
                ReceivedLines++;
            }
        }
        HAL_UART_Receive_IT(&huart1, &RxData, 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM20)
    {
        if (GRUZIK.PowerMode != Start)
        {
            if (TelemetryMode != TELEMETRY_OFF)
            {
                Robot_SampleSensorsWhenStopped();
            }
            else
            {
                Robot_ResetEncoderCounters();
            }
            return;
        }

        Robot_UpdateOdometryTick();

        if (GRUZIK.state == Mapping)
        {
            float speed_mps = 0.5f * (Motor_R.MetersPerSecondLPF.output + Motor_L.MetersPerSecondLPF.output);
            MapUpdate(&map, &RobotOdom.pose, speed_mps, GRUZIK.Error_P, GRUZIK.Error_D);
        }
        else if (GRUZIK.state == UnMapping)
        {
            if (DriveOnMap(&map, &RobotOdom.pose) == 0u)
            {
                GRUZIK.UnMappingDone = 1u;
                GRUZIK.DoneUnMappingTimer = HAL_GetTick();
                GRUZIK.state = PidFollowing;
            }
        }
    }

    if (htim->Instance == TIM5)
    {
        if (GRUZIK.PowerMode == Start)
        {
            HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

            if (GRUZIK.state != UnMapping)
            {
                PID_control(&GRUZIK);
            }

            Robot_UpdateSpeedStats();

            if ((GRUZIK.UnMappingDone == 1u) && (HAL_GetTick() >= (GRUZIK.DoneUnMappingTimer + 1750u)))
            {
                GRUZIK.PowerMode = Stop;
                Robot_StopOutputs();
            }
        }
        else
        {
            Robot_StopOutputs();
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
