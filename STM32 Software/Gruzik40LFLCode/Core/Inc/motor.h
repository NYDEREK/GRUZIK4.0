/*
 * motor.h
 *
 * Encoder and speed estimation for GRUZIK4.0.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "LowPassFilter.h"

typedef struct
{
    uint32_t EncoderValue;
    int32_t EncoderDeltaCounts;
    int8_t EncoderDirection;

    float RPM;
    float PreviousRPM;
    float NumberOfRotations;
    float MetersPerSecond;
    float PreviousMetersSecond;

    LowPassFilter_t EncoderRpmFilter;
    LowPassFilter_t MetersPerSecondLPF;

    float DistanceInMeasurement;
    float LpfDistanceInMeasurement;
    float DistanceTraveled;

    float PWM_to_MS_Scaler_value;

    float set_speed;
    float current_speed;
    float speed;
    float error;
    float Error_sum;

    float ki;
    float kp;
    float P;
    float I;
} motor_t;

void Motor_Init(motor_t *motor, float Kp, float Ki);
void Motor_SetEncoderDirection(motor_t *motor, int8_t direction);
void Motor_CalculateSpeed(motor_t *motor);
void PI_Loop(motor_t *motor);

#endif /* INC_MOTOR_H_ */
