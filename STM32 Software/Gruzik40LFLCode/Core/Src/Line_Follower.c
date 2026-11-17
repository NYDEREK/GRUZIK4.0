/*
 * Line_Follower.c
 *
 * Line sensor and PD motor control for GRUZIK4.0.
 */

#include "Line_Follower.h"
#include "main.h"
#include "robot_config.h"
#include "tim.h"
#include <math.h>

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static uint16_t SensorValueAt(const LineFollower_t *LF, uint8_t index)
{
    static const uint8_t adc1_indices[8] = {0u, 1u, 2u, 3u, 5u, 6u, 7u, 8u};

    if (index < 8u)
    {
        return LF->Adc1_Values[adc1_indices[index]];
    }

    return LF->Adc2_Values[index - 8u];
}

static int SensorRead(LineFollower_t *LF)
{
    static const int sensor_weights[12] = {
        12000, 11000, 10000, 9000, 8000, 7000,
        6000, 5000, 4000, 3000, 2000, 1000
    };

    int position_sum = 0;
    int active = 0;

    for (uint8_t i = 0u; i < 12u; i++)
    {
        uint8_t is_active = (SensorValueAt(LF, i) > LF->treshold) ? 1u : 0u;
        LF->SensorValues[i] = is_active;

        if (is_active)
        {
            position_sum += sensor_weights[i];
            active++;
        }
    }

    if (LF->SensorValues[0] != 0u && HAL_GetTick() > (LF->LastEndTimer + 1u))
    {
        LF->LastEndTimer = HAL_GetTick();
        LF->Last_end = 0;
    }

    if (LF->SensorValues[11] != 0u && HAL_GetTick() > (LF->LastEndTimer + 1u))
    {
        LF->LastEndTimer = HAL_GetTick();
        LF->Last_end = 1;
    }

    LF->actives = active;
    if (active == 0)
    {
        LF->Last_idle++;
        LF->SensorPosition = (LF->Last_end == 1) ? 12000 : 1000;
    }
    else
    {
        LF->Last_idle = 0;
        LF->SensorPosition = position_sum / active;
    }

    return LF->SensorPosition;
}

int LineFollower_UpdateSensors(LineFollower_t *LF)
{
    return SensorRead(LF);
}

void motor_control(LineFollower_t *LF, float pos_right, float pos_left)
{
    float speed_scale = (LF->Speed_level > 0.05f) ? LF->Speed_level : 1.0f;
    float left_pwm = clampf(pos_left * speed_scale, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
    float right_pwm = clampf(pos_right * speed_scale, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);

    if (left_pwm >= 0.0f)
    {
        HAL_GPIO_WritePin(Direction_L_A_GPIO_Port, Direction_L_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Direction_L_B_GPIO_Port, Direction_L_B_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)left_pwm);
    }
    else
    {
        HAL_GPIO_WritePin(Direction_L_A_GPIO_Port, Direction_L_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Direction_L_B_GPIO_Port, Direction_L_B_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(-left_pwm));
    }

    if (right_pwm >= 0.0f)
    {
        HAL_GPIO_WritePin(Direction_R_A_GPIO_Port, Direction_R_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Direction_R_B_GPIO_Port, Direction_R_B_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)right_pwm);
    }
    else
    {
        HAL_GPIO_WritePin(Direction_R_A_GPIO_Port, Direction_R_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Direction_R_B_GPIO_Port, Direction_R_B_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)(-right_pwm));
    }
}

static void sharp_turn(LineFollower_t *LF)
{
    if (LF->Last_idle < 25)
    {
        if (LF->Last_end == 1)
        {
            motor_control(LF, LF->Sharp_bend_speed_right, LF->Sharp_bend_speed_left);
        }
        else
        {
            motor_control(LF, LF->Sharp_bend_speed_left, LF->Sharp_bend_speed_right);
        }
    }
    else
    {
        if (LF->Last_end == 1)
        {
            motor_control(LF, LF->Bend_speed_right, LF->Bend_speed_left);
        }
        else
        {
            motor_control(LF, LF->Bend_speed_left, LF->Bend_speed_right);
        }
    }
}

static uint8_t EdgeSensorsActive(const LineFollower_t *LF)
{
    for (uint8_t i = 0u; i < ROBOT_GAP_BRIDGE_EDGE_GUARD_COUNT; i++)
    {
        if (LF->SensorValues[i] != 0u)
        {
            return 1u;
        }
        if (LF->SensorValues[11u - i] != 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

static uint8_t GapBridgeCandidate(const LineFollower_t *LF, float error)
{
    if (LF->actives <= 0)
    {
        return 0u;
    }

    if (fabsf(error) > ROBOT_GAP_BRIDGE_CENTER_ERROR)
    {
        return 0u;
    }

    return (EdgeSensorsActive(LF) == 0u) ? 1u : 0u;
}

static uint8_t TryGapBridge(LineFollower_t *LF)
{
    uint32_t now = HAL_GetTick();

    if (LF->GapBridgeActive != 0u)
    {
        if ((now - LF->GapBridgeStartMs) <= ROBOT_GAP_BRIDGE_TIMEOUT_MS)
        {
            motor_control(LF, LF->LastLineMotorRight, LF->LastLineMotorLeft);
            return 1u;
        }

        LF->GapBridgeActive = 0u;
        LF->GapBridgeArmed = 0u;
        return 0u;
    }

    if (LF->GapBridgeArmed != 0u)
    {
        LF->GapBridgeActive = 1u;
        LF->GapBridgeStartMs = now;
        motor_control(LF, LF->LastLineMotorRight, LF->LastLineMotorLeft);
        return 1u;
    }

    return 0u;
}

static void forward_brake(LineFollower_t *LF, float pos_right, float pos_left)
{
    if (LF->actives == 0)
    {
        if (TryGapBridge(LF) == 0u)
        {
            sharp_turn(LF);
        }
    }
    else
    {
        LF->GapBridgeActive = 0u;
        motor_control(LF, pos_right, pos_left);
    }
}

static void past_errors(LineFollower_t *LF, int error)
{
    for (int i = 9; i > 0; i--)
    {
        LF->Errors[i] = LF->Errors[i - 1];
    }
    LF->Errors[0] = error;
}

/* Pivot with the Sharp_bend speeds towards side (+1 = line on the right) */
static void corner_pivot(LineFollower_t *LF, int8_t side)
{
    if (side > 0)
    {
        motor_control(LF, LF->Sharp_bend_speed_right, LF->Sharp_bend_speed_left);
    }
    else
    {
        motor_control(LF, LF->Sharp_bend_speed_left, LF->Sharp_bend_speed_right);
    }
}

static uint8_t CornerLatchUpdate(LineFollower_t *LF, float error)
{
    if (LF->CornerLatch == 0u)
    {
        LF->CornerLatched = 0u;
        return 0u;
    }

    int8_t side = (error > 0.0f) ? 1 : -1;
    if (LF->CornerLatched == 0u)
    {
        if (fabsf(error) >= LF->CornerIn)
        {
            LF->CornerLatched = 1u;
            LF->CornerSide = side;
        }
    }
    else if ((fabsf(error) <= LF->CornerOut) || ((side != LF->CornerSide) && (fabsf(error) > LF->CornerOut)))
    {
        LF->CornerLatched = 0u;
    }

    return LF->CornerLatched;
}

void PID_control(LineFollower_t *LF)
{
    uint16_t position = (uint16_t)LineFollower_UpdateSensors(LF);

    if (LF->actives == 0)
    {
        /* line lost: the latch side is a better guess than Last_end */
        if ((LF->CornerLatched != 0u) && (LF->GapBridgeActive == 0u))
        {
            LF->Last_end = (LF->CornerSide > 0) ? 1 : 0;
        }
        forward_brake(LF, 0.0f, 0.0f);
        return;
    }

    float error = 6500.0f - (float)position;

    past_errors(LF, (int)error);
    uint8_t window = LF->DWin;
    if (window < 1u)
    {
        window = 1u;
    }
    if (window > 9u)
    {
        window = 9u;
    }

    LF->P = error;
    LF->Error_P = error;
    LF->D = (window == 1u) ? (error - LF->Last_error) : ((error - (float)LF->Errors[window]) / (float)window);
    LF->Error_D = LF->D;
    LF->Last_error = error;

    float base_left = (LF->DynamicBaseActive != 0u) ? LF->DynamicBase : LF->Base_speed_L;
    float base_right = (LF->DynamicBaseActive != 0u) ? LF->DynamicBase : LF->Base_speed_R;
    if (LF->ErrSlow > 0.0f)
    {
        float ratio = clampf(fabsf(error) / 5500.0f, 0.0f, 1.0f);
        float scale = 1.0f - (clampf(LF->ErrSlow, 0.0f, 1.0f) * ratio * ratio);
        base_left *= scale;
        base_right *= scale;
    }

    float motor_speed = (LF->P * LF->Kp) + (LF->D * LF->Kd);
    float left_speed = base_left + motor_speed;
    float right_speed = base_right - motor_speed;

    left_speed = clampf(left_speed, -ROBOT_PWM_MAX, LF->Max_speed_L);
    right_speed = clampf(right_speed, -ROBOT_PWM_MAX, LF->Max_speed_R);

    LF->LastLineMotorRight = right_speed;
    LF->LastLineMotorLeft = left_speed;
    LF->GapBridgeArmed = GapBridgeCandidate(LF, error);

    if (CornerLatchUpdate(LF, error) != 0u)
    {
        LF->GapBridgeActive = 0u;
        LF->GapBridgeArmed = 0u;
        corner_pivot(LF, LF->CornerSide);
        return;
    }

    forward_brake(LF, right_speed, left_speed);
}

float GetAverageSpeed(LineFollower_t *LF)
{
    return LF->AverageSpeed;
}
