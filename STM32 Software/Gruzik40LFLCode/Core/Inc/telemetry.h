/*
 * telemetry.h
 *
 * UART output for GRUZIK4.0: blocking replies, non-blocking streams and the
 * telemetry frames read by the Android app.
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"

/* Blocking send, waits for a running stream frame first */
void Uart_Send(const char *text);
void Uart_SendLen(const char *data, uint16_t len);
/* Non-blocking send, returns 0 when the previous frame is still being sent */
uint8_t Uart_TrySendAsync(const char *data, uint16_t len);
uint8_t Uart_WaitTxIdle(uint32_t timeout_ms);

void Telemetry_Service(void);
void Telemetry_SendInfo(void);

#endif /* INC_TELEMETRY_H_ */
