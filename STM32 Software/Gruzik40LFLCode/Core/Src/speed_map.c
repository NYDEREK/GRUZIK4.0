/*
 * speed_map.c
 *
 * Speed map for GRUZIK4.0.
 *
 * Mapping lap: every 1 cm of driven distance the fused heading is stored. Nothing touches
 * the SD card from the interrupt.
 *
 * Profile (built in the main loop before a run):
 *   curvature k(s)     = heading change over a window / window length
 *   corner speed       = sqrt(lat_accel / |k|), clamped to v_min..v_max
 *   backward pass      = braking before corners, v(s) <= sqrt(v(s+ds)^2 + 2 * brake * ds)
 *   forward pass       = acceleration on straights, v(s) <= sqrt(v(s-ds)^2 + 2 * accel * ds)
 *
 * Run: the line PID steers as always, the base PWM comes from a speed loop
 *   base = ff * v_target + kp * (v_target - v_measured)
 * with v_target read speed * lag ahead (minimum over that stretch).
 *
 * Distance drift: wheel slip makes the driven distance drift away from the map. Corner exits
 * are detected with the same causal detector during mapping and during the run, every exit in
 * the run snaps the distance to the matching exit of the map.
 */

#include "speed_map.h"
#include "app_fatfs.h"
#include "telemetry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define SPEEDMAP_FILE_MAGIC   0x314D5347u /* "GSM1" */
#define SPEEDMAP_FILE_VERSION 1u
#define SPEEDMAP_DEFAULT_FF   100.0f

typedef struct
{
    uint32_t magic;
    uint16_t count;
    uint8_t closed;
    uint8_t version;
    float ds;
    float total_turn;
    float ff_auto;
} SpeedMapFileHeader_t;

float SpeedMapHeading[ROBOT_SPEEDMAP_MAX_SAMPLES];
int16_t SpeedMapKappa[ROBOT_SPEEDMAP_MAX_SAMPLES];
uint16_t SpeedMapVelocity[ROBOT_SPEEDMAP_MAX_SAMPLES];

extern uint8_t SDReadingReady;
extern FATFS SdFatFs;

static FIL SpeedMapFile;

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

static void SanitizeParams(SpeedMapParams_t *p)
{
    p->v_max = clampf(p->v_max, 0.2f, 8.0f);
    p->v_min = clampf(p->v_min, 0.1f, p->v_max);
    p->accel = clampf(p->accel, 0.2f, 60.0f);
    p->brake = clampf(p->brake, 0.2f, 60.0f);
    p->lat_accel = clampf(p->lat_accel, 0.2f, 60.0f);
    p->lag_s = clampf(p->lag_s, 0.0f, 0.5f);
    p->window_m = clampf(p->window_m, 0.02f, 0.40f);
    p->speed_kp = clampf(p->speed_kp, 0.0f, 2000.0f);
    p->speed_ff = clampf(p->speed_ff, 0.0f, 2000.0f);
}

void SpeedMap_Init(SpeedMap_t *sm)
{
    memset(sm, 0, sizeof(*sm));
    sm->params.v_max = 2.5f;
    sm->params.v_min = 0.8f;
    sm->params.accel = 6.0f;
    sm->params.brake = 8.0f;
    sm->params.lat_accel = 7.0f;
    sm->params.lag_s = 0.06f;
    sm->params.window_m = 0.10f;
    sm->params.speed_kp = 120.0f;
    sm->params.speed_ff = 0.0f;
    sm->params.laps = 0u;
}

void SpeedMap_Clear(SpeedMap_t *sm)
{
    SpeedMapParams_t params = sm->params;
    memset(sm, 0, sizeof(*sm));
    sm->params = params;
}

/* ------------------------------------------------------------------ heading access */

static float HeadingAt(const SpeedMap_t *sm, int32_t index)
{
    int32_t n = (int32_t)sm->count;
    if (n <= 0)
    {
        return 0.0f;
    }

    if (sm->closed != 0u)
    {
        float turns = 0.0f;
        while (index < 0)
        {
            index += n;
            turns -= 1.0f;
        }
        while (index >= n)
        {
            index -= n;
            turns += 1.0f;
        }
        return SpeedMapHeading[index] + (turns * sm->total_turn);
    }

    if (index < 0)
    {
        index = 0;
    }
    if (index >= n)
    {
        index = n - 1;
    }
    return SpeedMapHeading[index];
}

/* ------------------------------------------------------------------ corner detector */

static void Detector_Reset(CornerDetector_t *d)
{
    memset(d, 0, sizeof(*d));
}

/* Returns 1 on a corner exit, turn_out = heading change of that corner */
static uint8_t Detector_Push(CornerDetector_t *d, float heading, float *turn_out)
{
    d->ring[d->head] = heading;
    d->head = (uint8_t)((d->head + 1u) % SPEEDMAP_DETECT_RING);
    if (d->filled < SPEEDMAP_DETECT_RING)
    {
        d->filled++;
    }
    if (d->filled <= SPEEDMAP_DETECT_SPAN)
    {
        return 0u;
    }

    uint8_t old_index = (uint8_t)((d->head + SPEEDMAP_DETECT_RING - 1u - SPEEDMAP_DETECT_SPAN) % SPEEDMAP_DETECT_RING);
    float old_heading = d->ring[old_index];
    float kappa = (heading - old_heading) / ((float)SPEEDMAP_DETECT_SPAN * ROBOT_SPEEDMAP_DS_M);
    int8_t sign = (kappa >= 0.0f) ? 1 : -1;

    if (d->in_corner == 0u)
    {
        if (fabsf(kappa) > ROBOT_SPEEDMAP_CORNER_ON_KAPPA)
        {
            d->in_corner = 1u;
            d->sign = sign;
            d->corner_start_h = old_heading;
        }
        return 0u;
    }

    if ((fabsf(kappa) < ROBOT_SPEEDMAP_CORNER_OFF_KAPPA) || (sign != d->sign))
    {
        float turn = heading - d->corner_start_h;
        d->in_corner = 0u;
        if (fabsf(turn) > ROBOT_SPEEDMAP_CORNER_MIN_RAD)
        {
            *turn_out = turn;
            return 1u;
        }
    }
    return 0u;
}

/* ------------------------------------------------------------------ recording (ISR) */

void SpeedMap_BeginRecording(SpeedMap_t *sm)
{
    sm->recording = 0u;
    sm->running = 0u;
    sm->count = 0u;
    sm->closed = 0u;
    sm->built = 0u;
    sm->dirty = 0u;
    sm->total_turn = 0.0f;
    sm->rec_next_s = 0.0f;
    sm->rec_prev_s = 0.0f;
    sm->rec_prev_h = 0.0f;
    sm->ff_cmd_sum = 0.0f;
    sm->ff_speed_sum = 0.0f;
    sm->ff_samples = 0u;
    sm->landmark_count = 0u;
    sm->recording = 1u;
}

void SpeedMap_RecordTick(SpeedMap_t *sm, float s, float heading, float v_meas, float cmd_mean, float yaw_rate)
{
    if (sm->recording == 0u)
    {
        return;
    }

    while ((s >= sm->rec_next_s) && (sm->count < ROBOT_SPEEDMAP_MAX_SAMPLES))
    {
        float span = s - sm->rec_prev_s;
        float frac = (span > 1e-6f) ? ((sm->rec_next_s - sm->rec_prev_s) / span) : 1.0f;
        frac = clampf(frac, 0.0f, 1.0f);
        SpeedMapHeading[sm->count] = sm->rec_prev_h + ((heading - sm->rec_prev_h) * frac);
        sm->count++;
        sm->rec_next_s += ROBOT_SPEEDMAP_DS_M;
    }
    sm->rec_prev_s = s;
    sm->rec_prev_h = heading;

    /* PWM per m/s on straights, used as speed loop feed-forward */
    if ((fabsf(yaw_rate) < 0.6f) && (v_meas > 0.15f) && (cmd_mean > 5.0f))
    {
        sm->ff_cmd_sum += cmd_mean;
        sm->ff_speed_sum += v_meas;
        sm->ff_samples++;
    }
}

void SpeedMap_FinishRecording(SpeedMap_t *sm, uint8_t closed, float s_close, float heading_close)
{
    sm->recording = 0u;
    if (sm->count == 0u)
    {
        return;
    }

    if (closed != 0u)
    {
        uint32_t close_count = (uint32_t)lroundf(s_close / ROBOT_SPEEDMAP_DS_M);
        if ((close_count >= 50u) && (close_count < sm->count))
        {
            sm->count = (uint16_t)close_count;
        }
        sm->closed = 1u;
        sm->total_turn = heading_close - SpeedMapHeading[0];
    }
    else
    {
        sm->closed = 0u;
        sm->total_turn = SpeedMapHeading[sm->count - 1u] - SpeedMapHeading[0];
    }

    if ((sm->ff_samples > 200u) && (sm->ff_speed_sum > 1.0f))
    {
        sm->ff_auto = sm->ff_cmd_sum / sm->ff_speed_sum;
    }
    sm->built = 0u;
    sm->dirty = 1u;
}

/* ------------------------------------------------------------------ profile */

uint8_t SpeedMap_Build(SpeedMap_t *sm)
{
    if (sm->count < 50u)
    {
        sm->built = 0u;
        return 0u;
    }

    SanitizeParams(&sm->params);
    const SpeedMapParams_t *p = &sm->params;
    const int32_t n = (int32_t)sm->count;
    const float ds = ROBOT_SPEEDMAP_DS_M;
    int32_t span = (int32_t)lroundf(p->window_m / ds);
    if (span < 2)
    {
        span = 2;
    }

    /* curvature and corner speed */
    for (int32_t i = 0; i < n; i++)
    {
        int32_t a = i - (span / 2);
        int32_t b = a + span;
        if (sm->closed == 0u)
        {
            if (a < 0)
            {
                a = 0;
            }
            if (b > (n - 1))
            {
                b = n - 1;
            }
        }
        float span = (float)(b - a) * ds;
        float kappa = (span > 0.0f) ? ((HeadingAt(sm, b) - HeadingAt(sm, a)) / span) : 0.0f;
        SpeedMapKappa[i] = (int16_t)clampf(kappa * 100.0f, -32767.0f, 32767.0f);

        float v = p->v_max;
        if (fabsf(kappa) > 1e-4f)
        {
            v = sqrtf(p->lat_accel / fabsf(kappa));
        }
        v = clampf(v, p->v_min, p->v_max);
        SpeedMapVelocity[i] = (uint16_t)lroundf(v * 1000.0f);
    }

    uint8_t rounds = (sm->closed != 0u) ? 2u : 1u;

    /* braking before corners */
    float next_v = (sm->closed != 0u) ? ((float)SpeedMapVelocity[0] * 0.001f) : p->v_min;
    for (uint8_t r = 0u; r < rounds; r++)
    {
        for (int32_t i = n - 1; i >= 0; i--)
        {
            float v = (float)SpeedMapVelocity[i] * 0.001f;
            float limit = sqrtf((next_v * next_v) + (2.0f * p->brake * ds));
            if (v > limit)
            {
                v = limit;
                SpeedMapVelocity[i] = (uint16_t)lroundf(v * 1000.0f);
            }
            next_v = v;
        }
    }

    /* acceleration on straights */
    float prev_v = (sm->closed != 0u) ? ((float)SpeedMapVelocity[n - 1] * 0.001f) : p->v_min;
    for (uint8_t r = 0u; r < rounds; r++)
    {
        for (int32_t i = 0; i < n; i++)
        {
            float v = (float)SpeedMapVelocity[i] * 0.001f;
            float limit = sqrtf((prev_v * prev_v) + (2.0f * p->accel * ds));
            if (v > limit)
            {
                v = limit;
                SpeedMapVelocity[i] = (uint16_t)lroundf(v * 1000.0f);
            }
            prev_v = v;
        }
    }

    float lap_time = 0.0f;
    float peak = 0.0f;
    for (int32_t i = 0; i < n; i++)
    {
        float v = (float)SpeedMapVelocity[i] * 0.001f;
        lap_time += ds / ((v > 0.05f) ? v : 0.05f);
        if (v > peak)
        {
            peak = v;
        }
    }

    /* corner exits, same detector as in the run */
    CornerDetector_t det;
    Detector_Reset(&det);
    sm->landmark_count = 0u;
    int32_t start = (sm->closed != 0u) ? -(n / 2) : 0;
    for (int32_t j = start; j < n; j++)
    {
        float turn = 0.0f;
        if ((Detector_Push(&det, HeadingAt(sm, j), &turn) != 0u) && (j >= 0) &&
            (sm->landmark_count < ROBOT_SPEEDMAP_MAX_LANDMARKS))
        {
            sm->landmarks[sm->landmark_count].s = (float)j * ds;
            sm->landmarks[sm->landmark_count].turn = turn;
            sm->landmark_count++;
        }
    }

    sm->length_m = (float)n * ds;
    sm->lap_time_s = lap_time;
    sm->v_peak = peak;
    sm->built = 1u;
    return 1u;
}

/* ------------------------------------------------------------------ run */

uint8_t SpeedMap_BeginRun(SpeedMap_t *sm)
{
    sm->running = 0u;
    if ((sm->built == 0u) && (SpeedMap_Build(sm) == 0u))
    {
        return 0u;
    }

    if (sm->params.speed_ff > 0.0f)
    {
        sm->ff_used = sm->params.speed_ff;
    }
    else if (sm->ff_auto > 0.0f)
    {
        sm->ff_used = sm->ff_auto;
    }
    else
    {
        sm->ff_used = SPEEDMAP_DEFAULT_FF;
    }

    sm->anchor_map_s = 0.0f;
    sm->anchor_driven = 0.0f;
    sm->dist_scale = 1.0f;
    sm->s_track = 0.0f;
    sm->v_target = 0.0f;
    sm->v_ramp = sm->params.v_min;
    sm->base_cmd = 0.0f;
    sm->lap = 0u;
    sm->last_sync_mm = 0;
    sm->sync_count = 0u;
    sm->det_next_s = 0.0f;
    sm->det_prev_s = 0.0f;
    sm->det_prev_h = 0.0f;
    Detector_Reset(&sm->det);
    sm->running = 1u;
    return 1u;
}

static float WrapDistance(const SpeedMap_t *sm, float d)
{
    if (sm->closed == 0u)
    {
        return d;
    }
    float half = 0.5f * sm->length_m;
    while (d > half)
    {
        d -= sm->length_m;
    }
    while (d <= -half)
    {
        d += sm->length_m;
    }
    return d;
}

static float MapPositionAbs(const SpeedMap_t *sm, float s_driven)
{
    return sm->anchor_map_s + (sm->dist_scale * (s_driven - sm->anchor_driven));
}

static void TrySync(SpeedMap_t *sm, float turn, float s_driven)
{
    float since_anchor = s_driven - sm->anchor_driven;
    float window = ROBOT_SPEEDMAP_SYNC_WINDOW_RATIO * since_anchor;
    if (window < ROBOT_SPEEDMAP_SYNC_WINDOW_M)
    {
        window = ROBOT_SPEEDMAP_SYNC_WINDOW_M;
    }

    float s_abs = MapPositionAbs(sm, s_driven);
    float s_now = s_abs;
    if ((sm->closed != 0u) && (sm->length_m > 0.0f))
    {
        s_now = fmodf(s_now, sm->length_m);
        if (s_now < 0.0f)
        {
            s_now += sm->length_m;
        }
    }

    int32_t best = -1;
    float best_d = 0.0f;
    for (uint16_t i = 0u; i < sm->landmark_count; i++)
    {
        const SpeedLandmark_t *lm = &sm->landmarks[i];
        if ((lm->turn * turn) <= 0.0f)
        {
            continue;
        }
        float tolerance = 0.4f * fabsf(lm->turn);
        if (tolerance < 0.35f)
        {
            tolerance = 0.35f;
        }
        if (fabsf(lm->turn - turn) > tolerance)
        {
            continue;
        }
        float d = WrapDistance(sm, lm->s - s_now);
        if ((fabsf(d) < window) && ((best < 0) || (fabsf(d) < fabsf(best_d))))
        {
            best = (int32_t)i;
            best_d = d;
        }
    }

    if (best < 0)
    {
        return;
    }

    float new_abs = s_abs + best_d;

    /* average wheel slip between two anchors (the start counts as one) */
    if (since_anchor > 0.5f)
    {
        float ratio = (new_abs - sm->anchor_map_s) / since_anchor;
        ratio = clampf(ratio, 0.90f, 1.10f);
        sm->dist_scale = clampf(sm->dist_scale + (0.5f * (ratio - sm->dist_scale)), 0.90f, 1.10f);
    }

    sm->anchor_map_s = new_abs;
    sm->anchor_driven = s_driven;
    sm->last_sync_mm = (int32_t)lroundf(best_d * 1000.0f);
    sm->sync_count++;
}

static uint16_t ProfileIndex(const SpeedMap_t *sm, float s)
{
    int32_t n = (int32_t)sm->count;
    int32_t i = (int32_t)(s / ROBOT_SPEEDMAP_DS_M);
    if (sm->closed != 0u)
    {
        i %= n;
        if (i < 0)
        {
            i += n;
        }
    }
    else
    {
        if (i < 0)
        {
            i = 0;
        }
        if (i >= n)
        {
            i = n - 1;
        }
    }
    return (uint16_t)i;
}

float SpeedMap_RunTick(SpeedMap_t *sm, float s_driven, float heading, float v_meas, float cmd_max, float dt_s,
                       uint8_t *finished)
{
    *finished = 0u;
    if ((sm->running == 0u) || (sm->count < 2u))
    {
        *finished = 1u;
        return 0.0f;
    }

    /* corner detector on a 1 cm grid of driven distance */
    while (s_driven >= sm->det_next_s)
    {
        float span = s_driven - sm->det_prev_s;
        float frac = (span > 1e-6f) ? ((sm->det_next_s - sm->det_prev_s) / span) : 1.0f;
        float h = sm->det_prev_h + ((heading - sm->det_prev_h) * clampf(frac, 0.0f, 1.0f));
        float turn = 0.0f;
        if (Detector_Push(&sm->det, h, &turn) != 0u)
        {
            TrySync(sm, turn, s_driven);
        }
        sm->det_next_s += ROBOT_SPEEDMAP_DS_M;
    }
    sm->det_prev_s = s_driven;
    sm->det_prev_h = heading;

    /* position on the map */
    float s_abs = MapPositionAbs(sm, s_driven);
    if (s_abs < 0.0f)
    {
        s_abs = 0.0f;
    }
    if (sm->closed != 0u)
    {
        uint16_t lap = (uint16_t)(s_abs / sm->length_m);
        sm->lap = lap;
        sm->s_track = s_abs - ((float)lap * sm->length_m);
        if ((sm->params.laps > 0u) && (lap >= sm->params.laps))
        {
            *finished = 1u;
        }
    }
    else
    {
        sm->lap = 0u;
        sm->s_track = s_abs;
        if (s_abs >= sm->length_m)
        {
            *finished = 1u;
        }
    }

    /* slowest point between here and speed * lag ahead */
    float lookahead = clampf(v_meas * sm->params.lag_s, 0.0f, 0.6f);
    uint16_t steps = (uint16_t)(lookahead / ROBOT_SPEEDMAP_DS_M);
    uint16_t v_mm = 0xFFFFu;
    for (uint16_t k = 0u; k <= steps; k++)
    {
        uint16_t v = SpeedMapVelocity[ProfileIndex(sm, sm->s_track + ((float)k * ROBOT_SPEEDMAP_DS_M))];
        if (v < v_mm)
        {
            v_mm = v;
        }
    }
    float profile_v = (float)v_mm * 0.001f;

    /* launch: the target never rises faster than the acceleration setting */
    sm->v_ramp += sm->params.accel * dt_s;
    if (sm->v_ramp > profile_v)
    {
        sm->v_ramp = profile_v;
    }
    sm->v_target = sm->v_ramp;

    float base = (sm->ff_used * sm->v_target) + (sm->params.speed_kp * (sm->v_target - v_meas));
    sm->base_cmd = clampf(base, -ROBOT_SPEEDMAP_MAX_REVERSE_PWM, cmd_max);
    return sm->base_cmd;
}

/* ------------------------------------------------------------------ SD card */

static uint8_t OpenWithMount(BYTE mode)
{
    if (SDReadingReady == 0u)
    {
        if (f_mount(&SdFatFs, "", 1) != FR_OK)
        {
            return 0u;
        }
        SDReadingReady = 1u;
    }

    FRESULT result = f_open(&SpeedMapFile, ROBOT_SPEEDMAP_FILE, mode);
    if (result == FR_OK)
    {
        return 1u;
    }
    if (result == FR_NO_FILE)
    {
        return 0u;
    }

    (void)f_mount(NULL, "", 0);
    HAL_Delay(20u);
    if (f_mount(&SdFatFs, "", 1) != FR_OK)
    {
        SDReadingReady = 0u;
        return 0u;
    }
    return (f_open(&SpeedMapFile, ROBOT_SPEEDMAP_FILE, mode) == FR_OK) ? 1u : 0u;
}

uint8_t SpeedMap_Save(SpeedMap_t *sm)
{
    if (sm->count == 0u)
    {
        return 0u;
    }
    if (OpenWithMount(FA_WRITE | FA_CREATE_ALWAYS) == 0u)
    {
        return 0u;
    }

    SpeedMapFileHeader_t header;
    memset(&header, 0, sizeof(header));
    header.magic = SPEEDMAP_FILE_MAGIC;
    header.version = SPEEDMAP_FILE_VERSION;
    header.count = sm->count;
    header.closed = sm->closed;
    header.ds = ROBOT_SPEEDMAP_DS_M;
    header.total_turn = sm->total_turn;
    header.ff_auto = sm->ff_auto;

    UINT written = 0u;
    FRESULT result = f_write(&SpeedMapFile, &header, sizeof(header), &written);
    if ((result == FR_OK) && (written == sizeof(header)))
    {
        UINT bytes = (UINT)sm->count * (UINT)sizeof(float);
        result = f_write(&SpeedMapFile, SpeedMapHeading, bytes, &written);
        if (written != bytes)
        {
            result = FR_DISK_ERR;
        }
    }
    f_close(&SpeedMapFile);

    if (result == FR_OK)
    {
        sm->dirty = 0u;
        return 1u;
    }
    return 0u;
}

uint8_t SpeedMap_Load(SpeedMap_t *sm)
{
    if (OpenWithMount(FA_READ) == 0u)
    {
        return 0u;
    }

    SpeedMapFileHeader_t header;
    UINT read = 0u;
    uint8_t ok = 0u;
    if ((f_read(&SpeedMapFile, &header, sizeof(header), &read) == FR_OK) && (read == sizeof(header)) &&
        (header.magic == SPEEDMAP_FILE_MAGIC) && (header.version == SPEEDMAP_FILE_VERSION) &&
        (fabsf(header.ds - ROBOT_SPEEDMAP_DS_M) < 1e-6f) && (header.count >= 50u) &&
        (header.count <= ROBOT_SPEEDMAP_MAX_SAMPLES))
    {
        UINT bytes = (UINT)header.count * (UINT)sizeof(float);
        if ((f_read(&SpeedMapFile, SpeedMapHeading, bytes, &read) == FR_OK) && (read == bytes))
        {
            sm->count = header.count;
            sm->closed = header.closed;
            sm->total_turn = header.total_turn;
            sm->ff_auto = header.ff_auto;
            sm->built = 0u;
            sm->dirty = 0u;
            ok = 1u;
        }
    }
    f_close(&SpeedMapFile);

    if (ok == 0u)
    {
        sm->count = 0u;
    }
    return ok;
}

/* ------------------------------------------------------------------ app transfer */

void SpeedMap_SendReady(const SpeedMap_t *sm)
{
    char tx[128];
    snprintf(tx, sizeof(tx), "SPEEDMAP_READY,%u,%ld,%ld,%ld,%ld,%u,%u\r\n",
             (unsigned int)sm->count,
             (long)lroundf(sm->length_m * 1000.0f),
             (long)lroundf(sm->lap_time_s * 1000.0f),
             (long)lroundf(sm->v_peak * 1000.0f),
             (long)lroundf(((sm->params.speed_ff > 0.0f) ? sm->params.speed_ff : sm->ff_auto) * 10.0f),
             (unsigned int)sm->landmark_count,
             (unsigned int)sm->closed);
    Uart_Send(tx);
}

/*
 * SM_BEGIN,<count>,<ds_0.1mm>,<closed>,<step>,<ff_x10>,<total_turn_mrad>
 * SMH,<index>,<heading_mrad>,<d1>,...   d = change of the rounded heading, every <step> samples
 * SM_END,<lines>,<count>
 */
void SpeedMap_Dump(const SpeedMap_t *sm)
{
    char tx[200];
    const uint16_t step = ROBOT_SPEEDMAP_DUMP_STEP;
    uint16_t lines = 0u;

    snprintf(tx, sizeof(tx), "SM_BEGIN,%u,%ld,%u,%u,%ld,%ld\r\n",
             (unsigned int)sm->count,
             (long)lroundf(ROBOT_SPEEDMAP_DS_M * 10000.0f),
             (unsigned int)sm->closed,
             (unsigned int)step,
             (long)lroundf(sm->ff_auto * 10.0f),
             (long)lroundf(sm->total_turn * 1000.0f));
    Uart_Send(tx);

    uint16_t index = 0u;
    while (index < sm->count)
    {
        long previous = lroundf(SpeedMapHeading[index] * 1000.0f);
        int len = snprintf(tx, sizeof(tx), "SMH,%u,%ld", (unsigned int)index, previous);
        uint16_t next = (uint16_t)(index + step);
        for (uint16_t k = 0u; (k < ROBOT_SPEEDMAP_DUMP_PER_LINE) && (next < sm->count); k++)
        {
            long value = lroundf(SpeedMapHeading[next] * 1000.0f);
            int extra = snprintf(&tx[len], sizeof(tx) - (size_t)len, ",%ld", value - previous);
            if ((extra <= 0) || ((len + extra) >= (int)(sizeof(tx) - 3u)))
            {
                break;
            }
            len += extra;
            previous = value;
            next = (uint16_t)(next + step);
        }
        tx[len++] = '\r';
        tx[len++] = '\n';
        Uart_SendLen(tx, (uint16_t)len);
        lines++;
        index = next;
    }

    snprintf(tx, sizeof(tx), "SM_END,%u,%u\r\n", (unsigned int)lines, (unsigned int)sm->count);
    Uart_Send(tx);
}
