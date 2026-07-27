/**
  ******************************************************************************
  * @file    ICM42688_STM32.c
  * @brief   Driver para o IMU ICM-42688-P via SPI (HAL STM32), com configuracao
  *          de full-scale e ODR feita em tempo de execucao (ver ICM42688_Config).
  ******************************************************************************
  */

#include "ICM42688_STM32.h"
#include "spi.h"

/* Sensibilidades calculadas em ICM42688_Init() a partir da config recebida,
 * e usadas depois em ICM42688_ReadData(). Como so existe 1 sensor no projeto,
 * guardar esse estado como estatico do modulo e suficiente. */
static float s_accel_sensitivity_lsb_per_g   = 2048.0f;  /* default: +/-16g */
static float s_gyro_sensitivity_lsb_per_dps  = 16.4f;    /* default: +/-2000dps */

static void ICM42688_CS_Low(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_RESET);
}

static void ICM42688_CS_High(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_SET);
}

static uint8_t ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   /* bit7 = 0 -> escrita */
    tx[1] = value;

    ICM42688_CS_Low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, tx, 2, 100);
    ICM42688_CS_High();

    return (st == HAL_OK) ? 0 : 1;
}

static uint8_t ICM42688_ReadRegs(uint8_t reg, uint8_t *rx, uint16_t len)
{
    uint8_t tx = reg | 0x80; /* bit7 = 1 -> leitura */

    ICM42688_CS_Low();
    HAL_StatusTypeDef st1 = HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &tx, 1, 100);
    HAL_StatusTypeDef st2 = HAL_SPI_Receive(&ICM42688_SPI_HANDLE, rx, len, 100);
    ICM42688_CS_High();

    return (st1 == HAL_OK && st2 == HAL_OK) ? 0 : 1;
}

/* Converte o enum de FS do acelerometro para LSB/g (datasheet, secao 3.2) */
static float ICM42688_AccelSensitivity(ICM42688_AccelFS fs)
{
    switch (fs)
    {
        case ICM42688_ACCEL_FS_2G:  return 16384.0f;
        case ICM42688_ACCEL_FS_4G:  return 8192.0f;
        case ICM42688_ACCEL_FS_8G:  return 4096.0f;
        case ICM42688_ACCEL_FS_16G: default: return 2048.0f;
    }
}

/* Converte o enum de FS do giroscopio para LSB/dps (datasheet, secao 3.1) */
static float ICM42688_GyroSensitivity(ICM42688_GyroFS fs)
{
    switch (fs)
    {
        case ICM42688_GYRO_FS_15_625DPS: return 2097.0f;
        case ICM42688_GYRO_FS_31_25DPS:  return 1048.0f;
        case ICM42688_GYRO_FS_62_5DPS:   return 524.3f;
        case ICM42688_GYRO_FS_125DPS:    return 262.0f;
        case ICM42688_GYRO_FS_250DPS:    return 131.0f;
        case ICM42688_GYRO_FS_500DPS:    return 65.5f;
        case ICM42688_GYRO_FS_1000DPS:   return 32.8f;
        case ICM42688_GYRO_FS_2000DPS:   default: return 16.4f;
    }
}

ICM42688_Config ICM42688_GetDefaultConfig(void)
{
    ICM42688_Config cfg;
    cfg.accel_fs = ICM42688_ACCEL_FS_16G;
    cfg.gyro_fs  = ICM42688_GYRO_FS_2000DPS;
    cfg.odr      = ICM42688_ODR_1KHZ;
    return cfg;
}

uint8_t ICM42688_Init(const ICM42688_Config *config)
{
    uint8_t who_am_i = 0;
    uint8_t gyro_config0_val, accel_config0_val;

    ICM42688_CS_High();
    HAL_Delay(10);

    /* 1) Confere identidade do chip */
    if (ICM42688_ReadRegs(ICM42688_REG_WHO_AM_I, &who_am_i, 1) != 0)
        return 1;

    if (who_am_i != ICM42688_WHO_AM_I_VALUE)
        return 2; /* sensor nao respondeu como esperado - confira fiacao/CS */

    /* 2) Soft reset */
    if (ICM42688_WriteReg(ICM42688_REG_DEVICE_CONFIG, 0x01) != 0)
        return 3;
    HAL_Delay(5); /* datasheet recomenda aguardar apos reset */

    /* 3) Monta e grava GYRO_CONFIG0 = FS_SEL[7:5] | ODR[3:0] */
    gyro_config0_val = (uint8_t)((config->gyro_fs << 5) | (config->odr & 0x0F));
    if (ICM42688_WriteReg(ICM42688_REG_GYRO_CONFIG0, gyro_config0_val) != 0)
        return 4;

    /* 4) Monta e grava ACCEL_CONFIG0 = FS_SEL[7:5] | ODR[3:0] */
    accel_config0_val = (uint8_t)((config->accel_fs << 5) | (config->odr & 0x0F));
    if (ICM42688_WriteReg(ICM42688_REG_ACCEL_CONFIG0, accel_config0_val) != 0)
        return 5;

    /* 5) Liga Accel e Gyro em modo Low-Noise (bits [3:2]=Gyro, [1:0]=Accel = 11) */
    if (ICM42688_WriteReg(ICM42688_REG_PWR_MGMT0, 0x0F) != 0)
        return 6;

    HAL_Delay(1); /* tempo para sensores estabilizarem apos ligar */

    /* 6) Guarda as sensibilidades resultantes para uso em ICM42688_ReadData() */
    s_accel_sensitivity_lsb_per_g  = ICM42688_AccelSensitivity(config->accel_fs);
    s_gyro_sensitivity_lsb_per_dps = ICM42688_GyroSensitivity(config->gyro_fs);

    return 0;
}

uint8_t ICM42688_ReadData(ICM42688_Data *data)
{
    uint8_t raw[14]; /* Temp(2) + Accel XYZ(6) + Gyro XYZ(6) - registradores contiguos */
    int16_t temp_raw, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

    if (ICM42688_ReadRegs(ICM42688_REG_TEMP_DATA1, raw, 14) != 0)
        return 1;

    temp_raw = (int16_t)((raw[0] << 8) | raw[1]);
    ax_raw   = (int16_t)((raw[2] << 8) | raw[3]);
    ay_raw   = (int16_t)((raw[4] << 8) | raw[5]);
    az_raw   = (int16_t)((raw[6] << 8) | raw[7]);
    gx_raw   = (int16_t)((raw[8] << 8) | raw[9]);
    gy_raw   = (int16_t)((raw[10] << 8) | raw[11]);
    gz_raw   = (int16_t)((raw[12] << 8) | raw[13]);

    data->accel_x_g  = ax_raw / s_accel_sensitivity_lsb_per_g;
    data->accel_y_g  = ay_raw / s_accel_sensitivity_lsb_per_g;
    data->accel_z_g  = az_raw / s_accel_sensitivity_lsb_per_g;

    data->gyro_x_dps = gx_raw / s_gyro_sensitivity_lsb_per_dps;
    data->gyro_y_dps = gy_raw / s_gyro_sensitivity_lsb_per_dps;
    data->gyro_z_dps = gz_raw / s_gyro_sensitivity_lsb_per_dps;

    /* Formula de temperatura do datasheet: Temp(C) = (raw / 132.48) + 25 */
    data->temp_c = (temp_raw / 132.48f) + 25.0f;

    return 0;
}
