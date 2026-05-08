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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern FRESULT FatFsResult;
extern FIL SdCardFile;
extern Map_t map;
extern Odometry_t RobotOdom;

static uint8_t SdFileOpen = 0u;

static void UartSend(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
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

    Map_Reset(&map);
    Odometry_Reset(&RobotOdom);
    LF->Orientation = 0.0f;
    LF->Yaw = 0.0f;
    LF->UnMappingDone = 0u;

    if (LF->state == Mapping)
    {
        FatFsResult = f_open(&SdCardFile, ROBOT_MAP_OUTPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS);
        if (FatFsResult != FR_OK)
        {
            UartSend("Cannot open mapping output file\r\n");
            return;
        }
        SdFileOpen = 1u;
        map.Mapping = 1u;
    }
    else if (LF->state == UnMapping)
    {
        FatFsResult = f_open(&SdCardFile, ROBOT_MAP_INPUT_FILE, FA_READ);
        if (FatFsResult != FR_OK)
        {
            UartSend("Cannot open map.txt for playback\r\n");
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

static void App_Controll(char rx_data, LineFollower_t *LF)
{
    if (rx_data == 'N')
    {
        StopRobot(LF);
    }
    else if ((rx_data == 'Y') || (rx_data == 'C'))
    {
        StartRobot(LF);
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
}

void Read_Settings_From_EEprom(LineFollower_t *LF, int mode)
{
    (void)LF;
    (void)mode;
}
