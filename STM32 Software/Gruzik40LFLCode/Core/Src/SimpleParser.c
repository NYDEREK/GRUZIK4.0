/*
 * SimpleParser.c
 *
 * UART command parser for GRUZIK4.0.
 */

#include "SimpleParser.h"
#include "Line_Follower.h"
#include "app_fatfs.h"
#include "gpio.h"
#include "map.h"
#include "odometry.h"
#include "robot_config.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern FRESULT FatFsResult;
extern FATFS SdFatFs;
extern FIL SdCardFile;
extern uint8_t SDReadingReady;
extern Map_t map;
extern Odometry_t RobotOdom;
volatile uint8_t TelemetryMode = TELEMETRY_OFF;
volatile uint8_t TireCleaningActive = 0u;
volatile uint8_t ManualDriveActive = 0u;

static uint8_t SdFileOpen = 0u;
static uint16_t MapUploadPointCount = 0u;
static uint16_t MapUploadExpectedCount = 0u;
static float TireCleanSpeed = 170.0f;
static uint32_t ManualDriveLastTick = 0u;

static void UartSend(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 500);
}

static void SendFatFsError(const char *prefix, const char *operation, const char *file_name, FRESULT result)
{
    char tx[96];
    snprintf(tx, sizeof(tx), "%s,%s,%s,%u\r\n", prefix, operation, file_name, (unsigned int)result);
    UartSend(tx);
}

static uint8_t SdEnsureMounted(const char *context)
{
    FatFsResult = f_mount(&SdFatFs, "", 1);
    if (FatFsResult == FR_OK)
    {
        SDReadingReady = 1u;
        return 1u;
    }

    (void)f_mount(NULL, "", 0);
    HAL_Delay(20);
    FatFsResult = f_mount(&SdFatFs, "", 1);
    if (FatFsResult == FR_OK)
    {
        SDReadingReady = 1u;
        return 1u;
    }

    SDReadingReady = 0u;
    SendFatFsError("SD_ERROR", "mount", context, FatFsResult);
    return 0u;
}

static uint8_t SdOpenFile(const char *error_prefix, const char *operation, const char *file_name, BYTE mode)
{
    if (SdEnsureMounted(file_name) == 0u)
    {
        return 0u;
    }

    FatFsResult = f_open(&SdCardFile, file_name, mode);
    if (FatFsResult == FR_OK)
    {
        return 1u;
    }

    (void)f_mount(NULL, "", 0);
    HAL_Delay(20);
    if (SdEnsureMounted(file_name) == 0u)
    {
        return 0u;
    }

    FatFsResult = f_open(&SdCardFile, file_name, mode);
    if (FatFsResult == FR_OK)
    {
        return 1u;
    }

    SendFatFsError(error_prefix, operation, file_name, FatFsResult);
    return 0u;
}

static char *NextValue(void)
{
    return strtok(NULL, ",");
}

static uint8_t ParseFloatValue(float *out)
{
    char *parse_pointer = NextValue();
    if ((parse_pointer == NULL) || (strlen(parse_pointer) == 0u) || (strlen(parse_pointer) >= 32u))
    {
        return 0u;
    }

    *out = strtof(parse_pointer, NULL);
    return 1u;
}

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData)
{
    uint8_t tmp = 0u;
    uint8_t i = 0u;

    do
    {
        if (RB_Read(Buf, &tmp) != RB_OK)
        {
            ReceivedData[i] = 0u;
            return;
        }

        if (tmp == ENDLINE)
        {
            ReceivedData[i] = 0u;
        }
        else if ((tmp != '\r') && (i < (PARSER_LINE_BUFFER_SIZE - 1u)))
        {
            ReceivedData[i++] = tmp;
        }
    } while (tmp != ENDLINE);
}

void Add_SimpleMap_Point(LineFollower_t *LF)
{
    float x = 0.0f;
    float y = 0.0f;
    if (ParseFloatValue(&x) && ParseFloatValue(&y) && (LF->ChangePointsCount < 32))
    {
        LF->ChangePoint[LF->ChangePointsCount][0] = x;
        LF->ChangePoint[LF->ChangePointsCount][1] = y;
        LF->ChangePointsCount++;
    }
}

static void Sensor_treshold_change(LineFollower_t *LF)
{
    float threshold = 0.0f;
    if (ParseFloatValue(&threshold) && (threshold > 1500.0f))
    {
        LF->treshold = (uint16_t)threshold;
    }
}

static void Turbine_Speed_change(LineFollower_t *LF)
{
    float speed = 0.0f;
    if (ParseFloatValue(&speed))
    {
        LF->Turbine_Speed = (speed < 0.0f) ? (48.0f - speed) : (1048.0f + speed);
    }
}

static void Turbine_Prep_Time_change(LineFollower_t *LF)
{
    float prep_time = 0.0f;
    if (ParseFloatValue(&prep_time))
    {
        LF->Turbine_Prep_Time = (uint32_t)prep_time;
    }
}

static void kp_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Kp);
}

static void kd_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Kd);
}

static void Base_speed_change(LineFollower_t *LF)
{
    float speed = 0.0f;
    if (ParseFloatValue(&speed))
    {
        LF->Base_speed_R = speed;
        LF->Base_speed_L = speed;
    }
}

static void Max_speed_change(LineFollower_t *LF)
{
    float speed = 0.0f;
    if (ParseFloatValue(&speed))
    {
        LF->Max_speed_R = speed;
        LF->Max_speed_L = speed;
    }
}

static void Sharp_bend_speed_right_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Sharp_bend_speed_right);
}

static void Sharp_bend_speed_left_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Sharp_bend_speed_left);
}

static void Bend_speed_right_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Bend_speed_right);
}

static void Bend_speed_left_change(LineFollower_t *LF)
{
    ParseFloatValue(&LF->Bend_speed_left);
}

static void Map_p_change(void)
{
    ParseFloatValue(&map.p);
}

static void Map_i_change(void)
{
    ParseFloatValue(&map.i);
}

static void Map_d_change(void)
{
    ParseFloatValue(&map.d);
}

static void Map_speed_change(void)
{
    ParseFloatValue(&map.DefaultPlaybackSpeed);
}

static float ClampPwm(float value)
{
    if (value > ROBOT_PWM_MAX)
    {
        return ROBOT_PWM_MAX;
    }
    if (value < -ROBOT_PWM_MAX)
    {
        return -ROBOT_PWM_MAX;
    }
    return value;
}

static void Tire_clean_speed_change(void)
{
    float speed = 0.0f;
    if (ParseFloatValue(&speed))
    {
        if (speed < 0.0f)
        {
            speed = 0.0f;
        }
        else if (speed > 250.0f)
        {
            speed = 250.0f;
        }
        TireCleanSpeed = speed;
    }
}

static void Manual_drive_change(LineFollower_t *LF)
{
    float left_speed = 0.0f;
    float right_speed = 0.0f;
    if (!ParseFloatValue(&left_speed))
    {
        return;
    }
    if (!ParseFloatValue(&right_speed))
    {
        right_speed = left_speed;
    }

    if ((fabsf(left_speed) < 0.5f) && (fabsf(right_speed) < 0.5f))
    {
        ManualDriveActive = 0u;
        motor_control(LF, 0.0f, 0.0f);
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
        return;
    }

    if (LF->PowerMode == Start)
    {
        UartSend("MANUAL_ERROR,robot_running\r\n");
        return;
    }

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    TireCleaningActive = 0u;
    ManualDriveActive = 1u;
    ManualDriveLastTick = HAL_GetTick();
    LF->LineFollowing = 0u;
    LF->PowerMode = Stop;
    map.Mapping = 0u;
    map.PlaybackActive = 0u;
    TelemetryMode = TELEMETRY_OFF;

    left_speed = ClampPwm(left_speed);
    right_speed = ClampPwm(right_speed);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    motor_control(LF, right_speed, left_speed);
}

static void Tire_clean_change(LineFollower_t *LF)
{
    float enabled = 0.0f;
    if (!ParseFloatValue(&enabled))
    {
        return;
    }

    if (enabled > 0.0f)
    {
        if (LF->PowerMode == Start)
        {
            UartSend("CLEAN_ERROR,robot_running\r\n");
            return;
        }

        if (SdFileOpen != 0u)
        {
            f_close(&SdCardFile);
            SdFileOpen = 0u;
        }

        LF->LineFollowing = 0u;
        LF->PowerMode = Stop;
        map.Mapping = 0u;
        map.PlaybackActive = 0u;
        TelemetryMode = TELEMETRY_OFF;
        ManualDriveActive = 0u;
        TireCleaningActive = 1u;
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
        motor_control(LF, TireCleanSpeed, TireCleanSpeed);
        UartSend("CLEAN,start\r\n");
    }
    else
    {
        TireCleaningActive = 0u;
        motor_control(LF, 0.0f, 0.0f);
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
        UartSend("CLEAN,stop\r\n");
    }
}

void Parser_ServiceManualDriveTimeout(LineFollower_t *LF)
{
    if ((ManualDriveActive != 0u) && ((HAL_GetTick() - ManualDriveLastTick) > 350u))
    {
        ManualDriveActive = 0u;
        motor_control(LF, 0.0f, 0.0f);
        HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    }
}

void Parser_FinishMappingAutoClosed(LineFollower_t *LF)
{
    LF->LineFollowing = 0u;
    LF->PowerMode = Stop;
    map.Mapping = 0u;
    map.PlaybackActive = 0u;
    TireCleaningActive = 0u;
    ManualDriveActive = 0u;

    motor_control(LF, 0.0f, 0.0f);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    UartSend("MAP_AUTO_CLOSED,start_radius_3cm\r\n");
}

static void Telemetry_change(void)
{
    char *parse_pointer = NextValue();
    if (parse_pointer == NULL)
    {
        return;
    }

    if ((!strcmp(parse_pointer, "odom")) || (!strcmp(parse_pointer, "ODOM")) || (!strcmp(parse_pointer, "1")))
    {
        TelemetryMode = TELEMETRY_ODOM;
        UartSend("TELEMETRY,odom\r\n");
    }
    else if ((!strcmp(parse_pointer, "debug")) || (!strcmp(parse_pointer, "DEBUG")) || (!strcmp(parse_pointer, "2")))
    {
        TelemetryMode = TELEMETRY_DEBUG;
        UartSend("TELEMETRY,debug\r\n");
    }
    else
    {
        TelemetryMode = TELEMETRY_OFF;
        UartSend("TELEMETRY,off\r\n");
    }
}

static const char *MapFileFromKind(const char *kind, const char **normalized_kind)
{
    if ((kind != NULL) &&
        ((!strcmp(kind, "optimized")) || (!strcmp(kind, "map")) || (!strcmp(kind, "map.txt"))))
    {
        *normalized_kind = "optimized";
        return ROBOT_MAP_INPUT_FILE;
    }

    *normalized_kind = "recorded";
    return ROBOT_MAP_OUTPUT_FILE;
}

static uint8_t ReadTransferLine(char *line, size_t line_size)
{
    UINT len = 0u;
    uint8_t i = 0u;
    uint8_t sample = 0u;

    while (i < (uint8_t)(line_size - 1u))
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
    return ((i > 0u) || (len != 0u)) ? 1u : 0u;
}

static void SendMapDump(LineFollower_t *LF)
{
    if (LF->PowerMode == Start)
    {
        UartSend("MAP_ERROR,stop_robot_first\r\n");
        return;
    }

    char *requested_kind = NextValue();
    const char *dump_kind = "recorded";
    const char *file_name = MapFileFromKind(requested_kind, &dump_kind);

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    if (SdOpenFile("MAP_ERROR", "open_read", file_name, FA_READ) == 0u)
    {
        return;
    }

    char tx[160];
    char line[112];
    uint16_t count = 0u;
    snprintf(tx, sizeof(tx), "MAP_BEGIN,%s\r\n", dump_kind);
    UartSend(tx);

    while (ReadTransferLine(line, sizeof(line)) != 0u)
    {
        snprintf(tx, sizeof(tx), "MAP,%s\r\n", line);
        UartSend(tx);
        count++;
        HAL_Delay(2);
    }

    f_close(&SdCardFile);
    snprintf(tx, sizeof(tx), "MAP_END,%u\r\n", count);
    UartSend(tx);
}

static void MapUploadBegin(LineFollower_t *LF)
{
    if (LF->PowerMode == Start)
    {
        UartSend("UPLOAD_ERROR,stop_robot_first\r\n");
        return;
    }

    float expected_count = 0.0f;
    (void)ParseFloatValue(&expected_count);

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    if (SdOpenFile("UPLOAD_ERROR", "open_write", ROBOT_MAP_INPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS) == 0u)
    {
        return;
    }

    SdFileOpen = 1u;
    MapUploadPointCount = 0u;
    MapUploadExpectedCount = (uint16_t)expected_count;

    char tx[80];
    snprintf(tx, sizeof(tx), "UPLOAD_READY,%u\r\n", MapUploadExpectedCount);
    UartSend(tx);
}

static void MapUploadPoint(void)
{
    if (SdFileOpen == 0u)
    {
        UartSend("UPLOAD_ERROR,no_file\r\n");
        return;
    }

    float x = 0.0f;
    float y = 0.0f;
    float speed = map.DefaultPlaybackSpeed;
    if ((!ParseFloatValue(&x)) || (!ParseFloatValue(&y)))
    {
        UartSend("UPLOAD_ERROR,bad_point\r\n");
        return;
    }
    (void)ParseFloatValue(&speed);

    char line[64];
    snprintf(line, sizeof(line), "%0.3f\t%0.3f\t%0.3f\n", x, y, speed);
    f_puts((TCHAR *)line, &SdCardFile);
    MapUploadPointCount++;

    if ((MapUploadPointCount % 50u) == 0u)
    {
        char tx[80];
        snprintf(tx, sizeof(tx), "UPLOAD_PROGRESS,%u,%u\r\n", MapUploadPointCount, MapUploadExpectedCount);
        UartSend(tx);
    }
}

static void MapUploadEnd(void)
{
    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    char tx[80];
    snprintf(tx, sizeof(tx), "UPLOAD_DONE,%u,%u\r\n", MapUploadPointCount, MapUploadExpectedCount);
    UartSend(tx);
}

static void SetState(LineFollower_t *LF, uint8_t state)
{
    if (LF->PowerMode == Start)
    {
        UartSend("Stop robot before changing state\r\n");
        return;
    }

    LF->state = state;
    if (state == Mapping)
    {
        UartSend("State: Mapping\r\n");
    }
    else if (state == UnMapping)
    {
        UartSend("State: UnMapping\r\n");
    }
    else
    {
        UartSend("State: PID\r\n");
    }
}

static void StopRobot(LineFollower_t *LF)
{
    LF->LineFollowing = 0u;
    LF->PowerMode = Stop;
    map.Mapping = 0u;
    map.PlaybackActive = 0u;
    TireCleaningActive = 0u;
    ManualDriveActive = 0u;

    motor_control(LF, 0.0f, 0.0f);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    LF->battery_voltage = ((float)LF->Adc1_Values[4] * 8.15f) / 3322.0f;

    char buffer[160];
    snprintf(buffer, sizeof(buffer),
             "Stop\r\nOne Cell = %0.2f\r\nBattery = %0.2f V\r\nAverageSpeed = %0.2f m/s\r\nMaxSpeed = %0.2f m/s\r\n",
             LF->battery_voltage / 2.0f, LF->battery_voltage, LF->AverageSpeed, LF->MaxSpeed);
    UartSend(buffer);

    LF->AverageSpeed = 0.0f;
    LF->MaxSpeed = 0.0f;
    LF->AverageSpeedNum = 0.0f;
    LF->AverageSpeedSum = 0.0f;
}

static void PrepareStoppedStart(LineFollower_t *LF)
{
    LF->LineFollowing = 0u;
    LF->PowerMode = Stop;
    map.Mapping = 0u;
    map.PlaybackActive = 0u;
    TelemetryMode = TELEMETRY_OFF;
    TireCleaningActive = 0u;
    ManualDriveActive = 0u;

    motor_control(LF, 0.0f, 0.0f);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }

    LF->AverageSpeed = 0.0f;
    LF->MaxSpeed = 0.0f;
    LF->AverageSpeedNum = 0.0f;
    LF->AverageSpeedSum = 0.0f;
}

static void StartRobot(LineFollower_t *LF)
{
    float battery_percentage;
    char buffer[160];

    LF->battery_voltage = ((float)LF->Adc1_Values[4] * 8.15f) / 3322.0f;
    battery_percentage = (LF->battery_voltage / 8.48f) * 100.0f;

    if (LF->battery_voltage < 7.0f)
    {
        UartSend("! Low Battery !\r\n");
    }

    LF->Speed_level = ((200.0f - battery_percentage) / 100.0f) - LF->Speed_offset;
    if (LF->Speed_level < 1.0f)
    {
        LF->Speed_level = 1.0f;
    }

    if (SdFileOpen != 0u)
    {
        f_close(&SdCardFile);
        SdFileOpen = 0u;
    }
    TireCleaningActive = 0u;
    ManualDriveActive = 0u;

    Map_Reset(&map);
    Odometry_Reset(&RobotOdom);
    LF->Orientation = 0.0f;
    LF->Yaw = 0.0f;
    LF->Last_error = 0.0f;
    LF->P = 0.0f;
    LF->D = 0.0f;
    LF->Error_P = 0.0f;
    LF->Error_D = 0.0f;
    LF->Last_idle = 0;
    LF->LastEndTimer = HAL_GetTick();
    LF->UnMappingDone = 0u;

    if (LF->state == Mapping)
    {
        if (SdOpenFile("MAP_ERROR", "open_write", ROBOT_MAP_OUTPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS) == 0u)
        {
            return;
        }
        SdFileOpen = 1u;
        map.Mapping = 1u;
    }
    else if (LF->state == UnMapping)
    {
        if (SdOpenFile("MAP_ERROR", "open_playback", ROBOT_MAP_INPUT_FILE, FA_READ) == 0u)
        {
            return;
        }
        SdFileOpen = 1u;
        if (Map_BeginPlayback(&map) == 0u)
        {
            UartSend("map.txt is empty or invalid\r\n");
            f_close(&SdCardFile);
            SdFileOpen = 0u;
            return;
        }
    }

    snprintf(buffer, sizeof(buffer), "Start\r\nBattery = %0.2f V\r\nSpeed_level = %0.2f\r\n",
             LF->battery_voltage, LF->Speed_level);
    UartSend(buffer);

    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    LF->PowerMode = Start;
}

static void StartWithState(LineFollower_t *LF, uint8_t state)
{
    PrepareStoppedStart(LF);
    LF->state = state;
    if (state == Mapping)
    {
        UartSend("STARTING,mapping\r\n");
    }
    else if (state == UnMapping)
    {
        UartSend("STARTING,playback\r\n");
    }
    else
    {
        UartSend("STARTING,normal\r\n");
    }
    StartRobot(LF);
}

static void App_Controll(char rx_data, LineFollower_t *LF)
{
    if (rx_data == 'N')
    {
        StopRobot(LF);
    }
    else if ((rx_data == 'Y') || (rx_data == 'C'))
    {
        if (LF->PowerMode != Start)
        {
            StartRobot(LF);
        }
        else
        {
            UartSend("Already started\r\n");
        }
    }
    else if (rx_data == 'P')
    {
        SetState(LF, PidFollowing);
    }
    else if (rx_data == 'M')
    {
        SetState(LF, Mapping);
    }
    else if (rx_data == 'U')
    {
        SetState(LF, UnMapping);
    }
}

static void Start_command(LineFollower_t *LF, uint8_t state)
{
    (void)NextValue();
    StartWithState(LF, state);
}

static void Mode_change(LineFollower_t *LF)
{
    char *parse_pointer = NextValue();
    if ((parse_pointer != NULL) && (strlen(parse_pointer) > 0u))
    {
        App_Controll(parse_pointer[0], LF);
    }
}

static void State_change(LineFollower_t *LF)
{
    char *parse_pointer = NextValue();
    if (parse_pointer == NULL)
    {
        return;
    }

    if ((!strcmp(parse_pointer, "PID")) || (!strcmp(parse_pointer, "Pid")) || (!strcmp(parse_pointer, "P")))
    {
        SetState(LF, PidFollowing);
    }
    else if ((!strcmp(parse_pointer, "Mapping")) || (!strcmp(parse_pointer, "M")))
    {
        SetState(LF, Mapping);
    }
    else if ((!strcmp(parse_pointer, "UnMapping")) || (!strcmp(parse_pointer, "U")))
    {
        SetState(LF, UnMapping);
    }
}

void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower)
{
    char *parse_pointer = strtok((char *)ReceivedData, "=");
    if (parse_pointer == NULL)
    {
        return;
    }

    if (!strcmp("Kp", parse_pointer))
    {
        kp_change(LineFollower);
    }
    else if (!strcmp("Kd", parse_pointer))
    {
        kd_change(LineFollower);
    }
    else if (!strcmp("Base_speed", parse_pointer))
    {
        Base_speed_change(LineFollower);
    }
    else if (!strcmp("Max_speed", parse_pointer))
    {
        Max_speed_change(LineFollower);
    }
    else if (!strcmp("Sharp_bend_speed_right", parse_pointer))
    {
        Sharp_bend_speed_right_change(LineFollower);
    }
    else if (!strcmp("Sharp_bend_speed_left", parse_pointer))
    {
        Sharp_bend_speed_left_change(LineFollower);
    }
    else if (!strcmp("Bend_speed_right", parse_pointer))
    {
        Bend_speed_right_change(LineFollower);
    }
    else if (!strcmp("Bend_speed_left", parse_pointer))
    {
        Bend_speed_left_change(LineFollower);
    }
    else if (!strcmp("Mode", parse_pointer))
    {
        Mode_change(LineFollower);
    }
    else if (!strcmp("State", parse_pointer))
    {
        State_change(LineFollower);
    }
    else if (!strcmp("StartNormal", parse_pointer))
    {
        Start_command(LineFollower, PidFollowing);
    }
    else if (!strcmp("StartMapping", parse_pointer))
    {
        Start_command(LineFollower, Mapping);
    }
    else if (!strcmp("StartPlayback", parse_pointer))
    {
        Start_command(LineFollower, UnMapping);
    }
    else if (!strcmp("Treshold", parse_pointer))
    {
        Sensor_treshold_change(LineFollower);
    }
    else if (!strcmp("Turbine_Speed", parse_pointer))
    {
        Turbine_Speed_change(LineFollower);
    }
    else if (!strcmp("Turbine_Prep_Time", parse_pointer))
    {
        Turbine_Prep_Time_change(LineFollower);
    }
    else if (!strcmp("Add_Simple_Map_Point", parse_pointer))
    {
        Add_SimpleMap_Point(LineFollower);
    }
    else if (!strcmp("MapP", parse_pointer))
    {
        Map_p_change();
    }
    else if (!strcmp("MapI", parse_pointer))
    {
        Map_i_change();
    }
    else if (!strcmp("MapD", parse_pointer))
    {
        Map_d_change();
    }
    else if (!strcmp("MapSpeed", parse_pointer))
    {
        Map_speed_change();
    }
    else if ((!strcmp("CleanSpeed", parse_pointer)) || (!strcmp("TireCleanSpeed", parse_pointer)))
    {
        Tire_clean_speed_change();
    }
    else if ((!strcmp("Clean", parse_pointer)) || (!strcmp("TireClean", parse_pointer)))
    {
        Tire_clean_change(LineFollower);
    }
    else if ((!strcmp("Manual", parse_pointer)) || (!strcmp("Joystick", parse_pointer)))
    {
        Manual_drive_change(LineFollower);
    }
    else if (!strcmp("Telemetry", parse_pointer))
    {
        Telemetry_change();
    }
    else if (!strcmp("Debug", parse_pointer))
    {
        Telemetry_change();
    }
    else if (!strcmp("MapDump", parse_pointer))
    {
        SendMapDump(LineFollower);
    }
    else if (!strcmp("MapUploadBegin", parse_pointer))
    {
        MapUploadBegin(LineFollower);
    }
    else if (!strcmp("MapPoint", parse_pointer))
    {
        MapUploadPoint();
    }
    else if (!strcmp("MapUploadEnd", parse_pointer))
    {
        MapUploadEnd();
    }
}

void Read_Settings_From_EEprom(LineFollower_t *LF, int mode)
{
    (void)LF;
    (void)mode;
}
