/*
 * telemetry.c
 *
 * UART output for GRUZIK4.0.
 *
 * Streams go out with HAL_UART_Transmit_IT and a frame is skipped while the previous one
 * is still on the wire, so the main loop (SD card, parser) never waits for 9600 baud.
 *
 * Frames (one line each, \r\n terminated):
 *   S,<pos>,<actives>,<last_end>,<12 x 3 hex ADC, robot left to right>,<battery_mV>
 *   L,<s_mm>,<x_mm>,<y_mm>,<yaw_cdeg>,<v_mm_s>,<v_target_mm_s>,<state>,<lap>,<last_sync_mm>
 *   ODOM,... and DBG,... as before (old app)
 *   INFO,GRUZIK4.0,<fw>,<sensors>,<caps>,<battery_mV>
 */

#include "telemetry.h"
#include "ICM-42688p.h"
#include "Line_Follower.h"
#include "SimpleParser.h"
#include "map.h"
#include "motor.h"
#include "odometry.h"
#include "robot_config.h"
#include "speed_map.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern LineFollower_t GRUZIK;
extern Odometry_t RobotOdom;
extern motor_t Motor_L;
extern motor_t Motor_R;
extern ICM_t ICM;
extern Map_t map;
extern SpeedMap_t SpeedMap;

static char AsyncTxBuffer[384];

uint8_t Uart_WaitTxIdle(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return 0u;
        }
    }
    return 1u;
}

void Uart_SendLen(const char *data, uint16_t len)
{
    if (len == 0u)
    {
        return;
    }
    (void)Uart_WaitTxIdle(200u);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 500u);
}

void Uart_Send(const char *text)
{
    Uart_SendLen(text, (uint16_t)strlen(text));
}

uint8_t Uart_TrySendAsync(const char *data, uint16_t len)
{
    if ((len == 0u) || (len > sizeof(AsyncTxBuffer)))
    {
        return 0u;
    }
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return 0u;
    }

    memcpy(AsyncTxBuffer, data, len);
    return (HAL_UART_Transmit_IT(&huart1, (uint8_t *)AsyncTxBuffer, len) == HAL_OK) ? 1u : 0u;
}

static uint32_t BatteryMillivolts(void)
{
    float volts = ((float)GRUZIK.Adc1_Values[4] * 8.15f) / 3322.0f;
    return (uint32_t)(volts * 1000.0f);
}

static uint16_t SensorRaw(uint8_t index)
{
    static const uint8_t adc1_indices[8] = {0u, 1u, 2u, 3u, 5u, 6u, 7u, 8u};
    if (index < 8u)
    {
        return GRUZIK.Adc1_Values[adc1_indices[index]];
    }
    return GRUZIK.Adc2_Values[index - 8u];
}

static uint8_t RobotStateCode(void)
{
    if (GRUZIK.PowerMode != Start)
    {
        return 0u;
    }
    if (GRUZIK.state == Mapping)
    {
        return 2u;
    }
    if (GRUZIK.state == UnMapping)
    {
        return 3u;
    }
    if (GRUZIK.state == SpeedRun)
    {
        return 4u;
    }
    return 1u;
}

static void SendSensorFrame(void)
{
    static const char hex[] = "0123456789ABCDEF";
    char tx[96];

    if (GRUZIK.PowerMode != Start)
    {
        (void)LineFollower_UpdateSensors(&GRUZIK);
    }

    int len = snprintf(tx, sizeof(tx), "S,%d,%d,%d,", GRUZIK.SensorPosition, GRUZIK.actives, GRUZIK.Last_end);
    if ((len <= 0) || (len > (int)(sizeof(tx) - 48u)))
    {
        return;
    }

    for (uint8_t i = 0u; i < 12u; i++)
    {
        uint16_t value = SensorRaw(i) & 0x0FFFu;
        tx[len++] = hex[(value >> 8) & 0x0Fu];
        tx[len++] = hex[(value >> 4) & 0x0Fu];
        tx[len++] = hex[value & 0x0Fu];
    }

    len += snprintf(&tx[len], sizeof(tx) - (size_t)len, ",%lu\r\n", (unsigned long)BatteryMillivolts());
    if (len < (int)sizeof(tx))
    {
        (void)Uart_TrySendAsync(tx, (uint16_t)len);
    }
}

static void SendLiveFrame(void)
{
    char tx[112];
    float v = 0.5f * (Motor_L.MetersPerSecondLPF.output + Motor_R.MetersPerSecondLPF.output);
    float s = RobotOdom.total_distance_m;
    float v_target = 0.0f;
    uint16_t lap = 0u;
    int32_t sync_mm = 0;

    if ((GRUZIK.state == SpeedRun) && (SpeedMap.running != 0u))
    {
        s = SpeedMap.s_track;
        v_target = SpeedMap.v_target;
        lap = SpeedMap.lap;
        sync_mm = SpeedMap.last_sync_mm;
    }

    int len = snprintf(tx, sizeof(tx), "L,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%ld\r\n",
                       (long)lroundf(s * 1000.0f),
                       (long)lroundf(RobotOdom.pose.x_m * 1000.0f),
                       (long)lroundf(RobotOdom.pose.y_m * 1000.0f),
                       (long)lroundf(RobotOdom.pose.yaw_rad * RAD_TO_DEG * 100.0f),
                       (long)lroundf(v * 1000.0f),
                       (long)lroundf(v_target * 1000.0f),
                       (unsigned int)RobotStateCode(),
                       (unsigned int)lap,
                       (long)sync_mm);
    if ((len > 0) && (len < (int)sizeof(tx)))
    {
        (void)Uart_TrySendAsync(tx, (uint16_t)len);
    }
}

static int FormatOdom(char *tx, size_t size)
{
    return snprintf(tx, size,
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
}

static void SendOdomFrame(uint8_t with_debug)
{
    char tx[384];
    int len = FormatOdom(tx, sizeof(tx));
    if ((len <= 0) || (len >= (int)sizeof(tx)))
    {
        return;
    }

    if (with_debug != 0u)
    {
        if (GRUZIK.PowerMode != Start)
        {
            (void)LineFollower_UpdateSensors(&GRUZIK);
        }
        int extra = snprintf(&tx[len], sizeof(tx) - (size_t)len,
                             "DBG,%d,%d,%d,%ld,%ld,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f",
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
                             ICM.Gyro_Z);
        if ((extra <= 0) || ((len + extra) >= (int)sizeof(tx)))
        {
            return;
        }
        len += extra;
        for (uint8_t i = 0u; i < 12u; i++)
        {
            extra = snprintf(&tx[len], sizeof(tx) - (size_t)len, ",%u", (unsigned int)SensorRaw(i));
            if ((extra <= 0) || ((len + extra) >= (int)sizeof(tx) - 2))
            {
                return;
            }
            len += extra;
        }
        tx[len++] = '\r';
        tx[len++] = '\n';
    }

    (void)Uart_TrySendAsync(tx, (uint16_t)len);
}

void Telemetry_Service(void)
{
    static uint32_t last_ms = 0u;
    uint32_t period;

    switch (TelemetryMode)
    {
    case TELEMETRY_SENSORS:
        period = ROBOT_TELEMETRY_SENSOR_PERIOD_MS;
        break;
    case TELEMETRY_LIVE:
        period = ROBOT_TELEMETRY_LIVE_PERIOD_MS;
        break;
    case TELEMETRY_ODOM:
    case TELEMETRY_DEBUG:
        period = ROBOT_TELEMETRY_ODOM_PERIOD_MS;
        break;
    default:
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_ms) < period)
    {
        return;
    }
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }
    last_ms = now;

    if (TelemetryMode == TELEMETRY_SENSORS)
    {
        SendSensorFrame();
    }
    else if (TelemetryMode == TELEMETRY_LIVE)
    {
        SendLiveFrame();
    }
    else
    {
        SendOdomFrame((TelemetryMode == TELEMETRY_DEBUG) ? 1u : 0u);
    }
}

void Telemetry_SendInfo(void)
{
    char tx[128];
    snprintf(tx, sizeof(tx), "INFO,%s,%u,12,odom|sd|map|speedmap|live|sensors|corner,%lu,%u\r\n",
             MY_NAME, (unsigned int)ROBOT_FIRMWARE_VERSION, (unsigned long)BatteryMillivolts(),
             (unsigned int)SpeedMap.count);
    Uart_Send(tx);
}
