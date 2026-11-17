/*
 * speed_map.h
 *
 * Speed map for GRUZIK4.0.
 *
 * The robot always follows the physical line with the normal PID, so corners are never cut.
 * Only the base speed comes from the map: target speed as a function of driven distance,
 * built from the track curvature recorded during the mapping lap.
 */

#ifndef INC_SPEED_MAP_H_
#define INC_SPEED_MAP_H_

#include "main.h"
#include "robot_config.h"

#define SPEEDMAP_DETECT_SPAN 8u
#define SPEEDMAP_DETECT_RING 16u

typedef struct
{
    float v_max;     /* m/s, straights */
    float v_min;     /* m/s, floor */
    float accel;     /* m/s^2, speeding up on straights */
    float brake;     /* m/s^2, slowing down before a corner */
    float lat_accel; /* m/s^2, corner speed = sqrt(lat_accel / curvature) */
    float lag_s;     /* s, look ahead by speed * lag to cover the motor response */
    float window_m;  /* m, curvature smoothing window */
    float speed_kp;  /* PWM per m/s of speed error */
    float speed_ff;  /* PWM per m/s, 0 = learned during mapping */
    uint16_t laps;   /* 0 = keep going */
} SpeedMapParams_t;

typedef struct
{
    float s;    /* map distance where the corner exit is detected */
    float turn; /* signed heading change of the corner */
} SpeedLandmark_t;

typedef struct
{
    float ring[SPEEDMAP_DETECT_RING];
    float corner_start_h;
    uint16_t filled;
    uint8_t head;
    uint8_t in_corner;
    int8_t sign;
} CornerDetector_t;

typedef struct
{
    SpeedMapParams_t params;

    /* recorded track, heading every ROBOT_SPEEDMAP_DS_M */
    uint16_t count;
    uint8_t closed;
    uint8_t dirty;
    float total_turn;
    float ff_auto;

    /* recording */
    volatile uint8_t recording;
    float rec_next_s;
    float rec_prev_s;
    float rec_prev_h;
    float ff_cmd_sum;
    float ff_speed_sum;
    uint32_t ff_samples;

    /* profile */
    uint8_t built;
    float length_m;
    float lap_time_s;
    float v_peak;
    uint16_t landmark_count;
    SpeedLandmark_t landmarks[ROBOT_SPEEDMAP_MAX_LANDMARKS];

    /* run */
    volatile uint8_t running;
    float anchor_map_s;   /* map distance (laps included) at the last sync */
    float anchor_driven;  /* driven distance at the last sync */
    float dist_scale;     /* map metres per driven metre, learned from syncs (wheel slip) */
    float s_track;
    float v_target;
    float v_ramp;
    float base_cmd;
    float ff_used;
    uint16_t lap;
    int32_t last_sync_mm;
    uint16_t sync_count;
    float det_next_s;
    float det_prev_s;
    float det_prev_h;
    CornerDetector_t det;
} SpeedMap_t;

extern float SpeedMapHeading[ROBOT_SPEEDMAP_MAX_SAMPLES];
extern int16_t SpeedMapKappa[ROBOT_SPEEDMAP_MAX_SAMPLES];     /* 1/m * 100 */
extern uint16_t SpeedMapVelocity[ROBOT_SPEEDMAP_MAX_SAMPLES]; /* mm/s */

void SpeedMap_Init(SpeedMap_t *sm);

/* ISR side */
void SpeedMap_BeginRecording(SpeedMap_t *sm);
void SpeedMap_RecordTick(SpeedMap_t *sm, float s, float heading, float v_meas, float cmd_mean, float yaw_rate);
float SpeedMap_RunTick(SpeedMap_t *sm, float s_driven, float heading, float v_meas, float cmd_max, float dt_s,
                       uint8_t *finished);

/* main loop side */
void SpeedMap_FinishRecording(SpeedMap_t *sm, uint8_t closed, float s_close, float heading_close);
uint8_t SpeedMap_Build(SpeedMap_t *sm);
uint8_t SpeedMap_BeginRun(SpeedMap_t *sm);
uint8_t SpeedMap_Save(SpeedMap_t *sm);
uint8_t SpeedMap_Load(SpeedMap_t *sm);
void SpeedMap_Clear(SpeedMap_t *sm);
void SpeedMap_SendReady(const SpeedMap_t *sm);
void SpeedMap_Dump(const SpeedMap_t *sm);

#endif /* INC_SPEED_MAP_H_ */
