/*
 * ICM-42688p.c
 *
 * Minimal ICM-42688P gyro driver used by GRUZIK4.0.
 */

#include "ICM-42688p.h"
#include "main.h"
#include "robot_config.h"
#include "spi.h"

#define ICM_REG_BANK_SEL       0x76u
#define ICM_REG_WHO_AM_I       0x75u
#define ICM_REG_PWR_MGMT0      0x4Eu
#define ICM_REG_GYRO_CONFIG0   0x4Fu
#define ICM_REG_GYRO_Z1        0x29u
#define ICM_REG_TEMP_DATA1     0x1Du
#define ICM_WHO_AM_I_VALUE     0x47u
#define ICM_SPI_TIMEOUT_MS     2u
#define ICM_GYRO_2000DPS_LSB   16.4f

static void ICM_Select(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

static void ICM_Deselect(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

static uint8_t ICM_Write8(SPI_HandleTypeDef *spi, uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = { (uint8_t)(reg & 0x7Fu), data };

    ICM_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, tx_data, 2, ICM_SPI_TIMEOUT_MS);
    ICM_Deselect();

    return (status == HAL_OK) ? 1u : 0u;
}

static uint8_t ICM_Read(SPI_HandleTypeDef *spi, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t address = (uint8_t)(reg | 0x80u);

    ICM_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, &address, 1, ICM_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(spi, data, len, ICM_SPI_TIMEOUT_MS);
    }
    ICM_Deselect();

    return (status == HAL_OK) ? 1u : 0u;
}

static uint8_t ICM_SelectBank(SPI_HandleTypeDef *spi, uint8_t bank)
{
    return ICM_Write8(spi, ICM_REG_BANK_SEL, bank);
}

uint8_t ICM_Init(ICM_t *icm, SPI_HandleTypeDef *spi)
{
    icm->spi = spi;
    icm->Gyro_Z = 0.0f;
    icm->Gyro_Z_RawDps = 0.0f;
    icm->Gyro_Z_BiasDps = 0.0f;
    icm->initialized = 0u;

    ICM_Deselect();
    HAL_Delay(10);

    if (!ICM_SelectBank(spi, 0x00u))
    {
        return 2u;
    }

    uint8_t who_am_i = 0u;
    if (!ICM_Read(spi, ICM_REG_WHO_AM_I, &who_am_i, 1))
    {
        return 2u;
    }

    if (who_am_i != ICM_WHO_AM_I_VALUE)
    {
        return 0u;
    }

    if (!ICM_Write8(spi, ICM_REG_PWR_MGMT0, 0x1Cu))
    {
        return 2u;
    }
    HAL_Delay(100);

    if (!ICM_SelectBank(spi, 0x01u))
    {
        return 2u;
    }
    ICM_Write8(spi, 0x03u, 0x00u);
    ICM_Write8(spi, 0x0Bu, 0xA0u);
    ICM_Write8(spi, 0x0Cu, 0x0Du);
    ICM_Write8(spi, 0x13u, 0x04u);

    if (!ICM_SelectBank(spi, 0x00u))
    {
        return 2u;
    }

    if (!ICM_Write8(spi, ICM_REG_GYRO_CONFIG0, 0x06u))
    {
        return 2u;
    }

    icm->initialized = 1u;
    return 1u;
}

uint8_t ICM_Read_GyroZ(ICM_t *icm)
{
    uint8_t rx_buf[2] = {0u, 0u};
    if (!ICM_Read(icm->spi, ICM_REG_GYRO_Z1, rx_buf, 2))
    {
        return 0u;
    }

    int16_t gyro_z_raw = (int16_t)((uint16_t)rx_buf[0] << 8 | rx_buf[1]);
    icm->Gyro_Z_RawDps = (float)gyro_z_raw / ICM_GYRO_2000DPS_LSB;
    icm->Gyro_Z = (icm->Gyro_Z_RawDps - icm->Gyro_Z_BiasDps) * ROBOT_IMU_Z_DIRECTION;
    return 1u;
}

uint8_t ICM_Read_Temp(ICM_t *icm)
{
    uint8_t rx[2] = {0u, 0u};
    if (!ICM_Read(icm->spi, ICM_REG_TEMP_DATA1, rx, 2))
    {
        return 0u;
    }

    int16_t temp_raw = (int16_t)((uint16_t)rx[0] << 8 | rx[1]);
    icm->temp_degC = ((float)temp_raw / 132.48f) + 25.0f;
    return 1u;
}

uint8_t ICM_CalibrateGyroZ(ICM_t *icm, uint16_t samples, uint32_t sample_delay_ms)
{
    if ((samples == 0u) || (icm->initialized == 0u))
    {
        return 0u;
    }

    float previous_bias = icm->Gyro_Z_BiasDps;
    icm->Gyro_Z_BiasDps = 0.0f;

    float sum = 0.0f;
    uint16_t valid_samples = 0u;
    for (uint16_t i = 0u; i < samples; i++)
    {
        if (ICM_Read_GyroZ(icm))
        {
            sum += icm->Gyro_Z_RawDps;
            valid_samples++;
        }
        HAL_Delay(sample_delay_ms);
    }

    if (valid_samples == 0u)
    {
        icm->Gyro_Z_BiasDps = previous_bias;
        return 0u;
    }

    icm->Gyro_Z_BiasDps = sum / (float)valid_samples;
    icm->Gyro_Z = 0.0f;
    return 1u;
}

void ICM_UpdateGyroZBias(ICM_t *icm, float learn_rate)
{
    icm->Gyro_Z_BiasDps += (icm->Gyro_Z_RawDps - icm->Gyro_Z_BiasDps) * learn_rate;
}
