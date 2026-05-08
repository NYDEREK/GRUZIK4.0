/*
 * ICM-42688p.h
 *
 * Minimal ICM-42688P gyro driver used by GRUZIK4.0.
 */

#ifndef INC_ICM_42688P_H_
#define INC_ICM_42688P_H_

#include "main.h"

typedef struct
{
    SPI_HandleTypeDef *spi;
    float Gyro_Z;
    float Gyro_Z_RawDps;
    float Gyro_Z_BiasDps;
    float temp_degC;
    float ErrorCompensation;
    uint8_t initialized;
} ICM_t;

uint8_t ICM_Init(ICM_t *icm, SPI_HandleTypeDef *spi);
uint8_t ICM_Read_GyroZ(ICM_t *icm);
uint8_t ICM_Read_Temp(ICM_t *icm);
uint8_t ICM_CalibrateGyroZ(ICM_t *icm, uint16_t samples, uint32_t sample_delay_ms);
void ICM_UpdateGyroZBias(ICM_t *icm, float learn_rate);

#endif /* INC_ICM_42688P_H_ */
