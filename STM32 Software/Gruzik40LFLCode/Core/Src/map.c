/*
 * map.c
 *
 * Route mapping and playback for GRUZIK4.0.
 */

#include "map.h"
#include "Line_Follower.h"
#include "app_fatfs.h"
#include "robot_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern LineFollower_t GRUZIK;
extern FRESULT FatFsResult;
extern FIL SdCardFile;

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

static uint8_t ReadMapLine(Map_t *map)
{
    char line[96];

    while (1)
    {
        UINT len = 0u;
        uint8_t i = 0u;
        uint8_t sample = 0u;

        while (i < (uint8_t)(sizeof(line) - 1u))
        {
            if ((f_read(&SdCardFile, &sample, 1, &len) != FR_OK) || (len == 0u))
            {
                break;
            }

            if (sample == '\r')
            {
                continue;
            }

            if (sample == '\n')
            {
                break;
            }

            line[i++] = (char)sample;
        }
        line[i] = '\0';

        if ((i == 0u) && (len == 0u))
        {
            return 0u;
        }

        char *token = strtok(line, " \t");
        if (token == NULL)
        {
            continue;
        }
        map->SetX = strtof(token, NULL);

        token = strtok(NULL, " \t");
        if (token == NULL)
        {
            continue;
        }
        map->SetY = strtof(token, NULL);

        token = strtok(NULL, " \t");
        if (token != NULL)
        {
            map->SetSpeed = clampf(strtof(token, NULL), -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
        }
        else
        {
            map->SetSpeed = map->DefaultPlaybackSpeed;
        }

        return 1u;
    }
}

void Map_Reset(Map_t *map)
{
    float p = map->p;
    float i = map->i;
    float d = map->d;
    float default_speed = map->DefaultPlaybackSpeed;

    memset(map, 0, sizeof(*map));
    map->p = p;
    map->i = i;
    map->d = d;
    map->DefaultPlaybackSpeed = (default_speed > 0.0f) ? default_speed : 80.0f;
}

uint8_t Map_BeginPlayback(Map_t *map)
{
    map->PlaybackActive = 0u;
    map->ErrorSum = 0.0f;
    map->LastError = 0.0f;

    if (ReadMapLine(map) == 0u)
    {
        return 0u;
    }

    map->PlaybackActive = 1u;
    return 1u;
}

void MapUpdate(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d)
{
    if (map->Mapping == 0u)
    {
        return;
    }

    map->PreviousXri = map->Xri;
    map->PreviousYri = map->Yri;
    map->PreviousOri = map->Ori;
    map->PreviousPci[0] = map->Pci[0];
    map->PreviousPci[1] = map->Pci[1];

    map->Xri = pose->x_m;
    map->Yri = pose->y_m;
    map->Ori = pose->yaw_rad;
    map->Pci[0] = map->Xri + (ROBOT_SENSOR_OFFSET_M * cosf(map->Ori));
    map->Pci[1] = map->Yri + (ROBOT_SENSOR_OFFSET_M * sinf(map->Ori));

    float dx = map->Pci[0] - map->PreviousPci[0];
    float dy = map->Pci[1] - map->PreviousPci[1];
    map->Ti = sqrtf((dx * dx) + (dy * dy));
    map->Si += map->Ti;

    if (map->Ti > 0.000001f)
    {
        map->PreviousAi = map->Ai;
        map->Ai = atan2f(dy, dx);
        map->Ki = Odometry_NormalizeAngle(map->Ai - map->PreviousAi) / map->Ti;
    }

    if (map->loopNumber >= ROBOT_MAP_RECORD_EVERY_TICKS)
    {
        uint8_t buffer[128];
        map->loopNumber = 0u;
        snprintf((char *)buffer, sizeof(buffer), "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
                 map->Xri, map->Yri, map->Pci[0], map->Pci[1], speed_mps, error_p, error_d);
        f_puts((TCHAR *)buffer, &SdCardFile);
    }

    map->loopNumber++;
}

uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose)
{
    if (map->PlaybackActive == 0u)
    {
        return 0u;
    }

    map->Xri = pose->x_m;
    map->Yri = pose->y_m;
    map->Ori = pose->yaw_rad;

    float dx = map->SetX - map->Xri;
    float dy = map->SetY - map->Yri;
    float distance_to_target = sqrtf((dx * dx) + (dy * dy));

    while (distance_to_target < ROBOT_MAP_TARGET_RADIUS_M)
    {
        if (ReadMapLine(map) == 0u)
        {
            map->PlaybackActive = 0u;
            return 0u;
        }

        dx = map->SetX - map->Xri;
        dy = map->SetY - map->Yri;
        distance_to_target = sqrtf((dx * dx) + (dy * dy));
    }

    map->SetRotation = atan2f(dy, dx);
    float error = Odometry_NormalizeAngle(map->SetRotation - map->Ori);
    map->ErrorSum = clampf(map->ErrorSum + error, -3.0f, 3.0f);

    float error_derivative = error - map->LastError;
    map->LastError = error;

    float correction = (map->p * error) + (map->i * map->ErrorSum) + (map->d * error_derivative);
    float right_speed = clampf(map->SetSpeed + correction, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
    float left_speed = clampf(map->SetSpeed - correction, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);

    motor_control(&GRUZIK, right_speed, left_speed);
    return 1u;
}
