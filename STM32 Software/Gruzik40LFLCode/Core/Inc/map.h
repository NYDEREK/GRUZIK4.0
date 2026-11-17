/*
 * map.h
 *
 * Route mapping and playback for GRUZIK4.0.
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

#include "main.h"
#include "odometry.h"
#include "robot_config.h"

typedef struct
{
    float x;
    float y;
    float speed;
} MapPoint_t;

typedef struct
{
    float x_ri;
    float y_ri;
    float x_pc;
    float y_pc;
    float speed_mps;
    float error_p;
    float error_d;
} MapRecord_t;

typedef struct
{
    uint8_t Mapping;
    uint8_t PlaybackActive;

    /* playback route, loaded from map.txt before the run */
    MapPoint_t Points[ROBOT_MAP_MAX_POINTS];
    uint16_t PointCount;
    uint16_t PlayIndex;

    /* GRUZIK.txt records, written by the main loop */
    MapRecord_t Queue[ROBOT_MAP_SD_QUEUE_LEN];
    volatile uint16_t QueueHead;
    volatile uint16_t QueueTail;
    volatile uint16_t QueueDropped;

    float Ti;
    float Xri;
    float PreviousXri;
    float Yri;
    float PreviousYri;
    float Ori;
    float PreviousOri;
    float Pci[2];
    float PreviousPci[2];
    float Si;
    float Ai;
    float PreviousAi;
    float Ki;

    float StartXri;
    float StartYri;
    float StartOri;
    float StartPci[2];
    float StartDistance;
    float StartHeading;
    uint16_t RecordedPoints;

    /* lap closure */
    uint8_t CloseCandidate;
    float CloseMinDistance;
    float CloseMinS;
    float CloseMinHeading;
    float CloseLength;
    float CloseError;
    float CloseTurn;

    float SetX;
    float SetY;
    float PreviousSetX;
    float PreviousSetY;
    float SetRotation;
    float SetSpeed;
    float DefaultPlaybackSpeed;

    float p;
    float i;
    float d;
    float ErrorSum;
    float LastError;
    float FilteredErrorDerivative;

    uint8_t loopNumber;
    uint8_t TargetValid;
    uint8_t StartValid;
    uint8_t AutoClosed;
} Map_t;

void Map_Reset(Map_t *map);
uint8_t Map_LoadPlayback(Map_t *map);
uint8_t Map_BeginPlayback(Map_t *map);
uint8_t MapUpdate(Map_t *map, const Odometry_t *odom, float speed_mps, float error_p, float error_d);
uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose);

/* main loop: move queued records to GRUZIK.txt */
void Map_ServiceRecording(Map_t *map);
void Map_FlushRecording(Map_t *map);
uint8_t Map_GyroScaleSuggestion(const Map_t *map, float current_scale, float *suggested, int32_t *turns,
                                float *error_rad);

#endif /* INC_MAP_H_ */
