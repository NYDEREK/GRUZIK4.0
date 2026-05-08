/*
 * motor.c
 *
 * Encoder and wheel-speed calculations for GRUZIK4.0.
 */

#include "main.h"
#include "motor.h"
#include "robot_config.h"
#include <string.h>

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

void Motor_Init(motor_t *motor, float Kp, float Ki)
{
    memset(motor, 0, sizeof(*motor));
    motor->kp = Kp;
    motor->ki = Ki;
    motor->EncoderDirection = 1;
    motor->PWM_to_MS_Scaler_value = 1.0f;
}

void Motor_SetEncoderDirection(motor_t *motor, int8_t direction)
{
    motor->EncoderDirection = (direction < 0) ? -1 : 1;
}

void Motor_CalculateSpeed(motor_t *motor)
{
    int32_t delta_counts = (int32_t)motor->EncoderValue - (int32_t)ROBOT_ENCODER_COUNTER_CENTER;
    delta_counts *= motor->EncoderDirection;

    motor->EncoderDeltaCounts = delta_counts;
    motor->NumberOfRotations = (float)delta_counts / ROBOT_ENCODER_COUNTS_PER_WHEEL_REV;
    motor->DistanceInMeasurement = motor->NumberOfRotations * ROBOT_WHEEL_CIRCUMFERENCE_M;
    motor->DistanceTraveled += motor->DistanceInMeasurement;

    motor->MetersPerSecond = motor->DistanceInMeasurement / ROBOT_CONTROL_DT_S;
    if ((motor->MetersPerSecond > 10.0f) || (motor->MetersPerSecond < -10.0f))
    {
        motor->MetersPerSecond = motor->PreviousMetersSecond;
    }

    LowPassFilter_Update(&motor->MetersPerSecondLPF, motor->MetersPerSecond);
    motor->PreviousMetersSecond = motor->MetersPerSecondLPF.output;
    motor->LpfDistanceInMeasurement = motor->MetersPerSecondLPF.output * ROBOT_CONTROL_DT_S;

    motor->RPM = motor->NumberOfRotations * (60.0f / ROBOT_CONTROL_DT_S);
    if ((motor->RPM > 30000.0f) || (motor->RPM < -30000.0f))
    {
        motor->RPM = motor->PreviousRPM;
    }

    LowPassFilter_Update(&motor->EncoderRpmFilter, motor->RPM);
    motor->PreviousRPM = motor->EncoderRpmFilter.output;
}

void PI_Loop(motor_t *motor)
{
    motor->current_speed = motor->MetersPerSecondLPF.output * motor->PWM_to_MS_Scaler_value;
    if (motor->current_speed < 0.0f)
    {
        motor->current_speed *= -1.0f;
    }

    motor->error = motor->set_speed - motor->current_speed;
    motor->Error_sum = clampf(motor->Error_sum + motor->error, -10.0f, 10.0f);
    motor->speed = motor->set_speed + (motor->error * motor->kp) + (motor->Error_sum * motor->ki);
}
