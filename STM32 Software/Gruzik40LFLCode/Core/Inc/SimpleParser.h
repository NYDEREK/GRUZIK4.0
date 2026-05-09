/*
 * SimpleParser.h
 *
 * UART command parser for GRUZIK4.0.
 */

#ifndef INC_SIMPLEPARSER_H_
#define INC_SIMPLEPARSER_H_

#include "Line_Follower.h"
#include "RingBuffer.h"
#include "main.h"

#define ENDLINE '\n'
#define MY_NAME "GRUZIK4.0"
#define PARSER_LINE_BUFFER_SIZE 64u

typedef enum
{
    TELEMETRY_OFF = 0,
    TELEMETRY_ODOM = 1,
    TELEMETRY_DEBUG = 2
} TelemetryMode_t;

extern volatile uint8_t TelemetryMode;
extern volatile uint8_t TireCleaningActive;
extern volatile uint8_t ManualDriveActive;

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData);
void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower);
void Parser_FinishMappingAutoClosed(LineFollower_t *LineFollower);
void Parser_ServiceManualDriveTimeout(LineFollower_t *LineFollower);
void Read_Settings_From_EEprom(LineFollower_t *LF, int mode);

#endif /* INC_SIMPLEPARSER_H_ */
