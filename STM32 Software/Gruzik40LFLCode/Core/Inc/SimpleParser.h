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

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData);
void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower);
void Read_Settings_From_EEprom(LineFollower_t *LF, int mode);

#endif /* INC_SIMPLEPARSER_H_ */
