/*
 * Line_Follower.h
 *
 * Line sensor and PD motor control for GRUZIK4.0.
 */

#ifndef INC_LINE_FOLLOWER_H_
#define INC_LINE_FOLLOWER_H_

#include "main.h"

typedef struct
{
    uint8_t PowerMode;
    float Kp;
    float Kd;
    float Error_P;
    float Error_D;

    float Turbine_Speed;
    uint32_t Turbine_Prep_Time;

    float Base_speed_R;
    float Base_speed_L;
    float Max_speed_R;
    float Max_speed_L;

    float Sharp_bend_speed_right;
    float Sharp_bend_speed_left;
    float Bend_speed_right;
    float Bend_speed_left;

    float battery_voltage;
    uint16_t Adc1_Values[9];
    uint16_t Adc2_Values[4];
    float Speed_level;
    float Speed_offset;

    float MaxSpeed;
    float AverageSpeed;
    float AverageSpeedSum;
    float AverageSpeedNum;
    float SmallestValueR;
    float SmallestValueL;

    float ChangePoint[32][2];
    int ChangePointsCount;
    uint8_t SimpleMapState;

    uint8_t SensorValues[12];
    uint16_t treshold;
    uint32_t LastEndTimer;
    int Last_end;
    int Last_idle;
    int SensorPosition;
    int actives;

    float P;
    float D;
    int Errors[10];
    float Last_error;

    uint8_t LineFollowing;

    uint32_t DSHOTTimer;
    uint32_t PreviousTimeDshot;
    uint32_t HighestDelayDshot;

    uint8_t state;
    float Yaw;
    float Orientation;
    uint32_t DoneUnMappingTimer;
    uint8_t UnMappingDone;
} LineFollower_t;

enum state
{
    PidFollowing,
    Mapping,
    UnMapping,
    SpeedUnMapping,
};

enum PowerState
{
    Initialization,
    Stop,
    Start,
};

void PID_control(LineFollower_t *LF);
float GetAverageSpeed(LineFollower_t *LF);
void motor_control(LineFollower_t *LF, float pos_right, float pos_left);

#endif /* INC_LINE_FOLLOWER_H_ */
