/*
 * ms5611.c
 * Driver for MS5611 pressure and temperature sensor (I2C, STM32 HAL)
 */

#include "ms5611.h"

/* Conversion delay for OSR=4096 (safe margin) */
#define MS5611_CONV_DELAY_MS   12

/* Coeficientes de calibracao brutos, lidos da PROM. C[0] e reservado (nao usado),
 * C[1]..C[6] sao SENSt1, OFFt1, TCS, TCO, Tref, TEMPSENS (nessa ordem, conforme datasheet) */
static uint16_t C[7];

/* 0 = formula MS5611 (SENSt1=C1*2^15, OFFt1=C2*2^16)
 * 1 = formula MS5607 (SENSt1=C1*2^16, OFFt1=C2*2^17) - alguns modulos/clones
 *     usam esses expoentes; se a pressao sair ~metade do valor real, troque para 1 */
static uint8_t s_mathMode;

/* Send a single command byte to the sensor */
static void MS5611_SendCommand(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    HAL_I2C_Master_Transmit(hi2c, MS5611_I2C_ADDR_HAL, &cmd, 1, HAL_MAX_DELAY);
}

/* Read 24-bit ADC result */
static uint32_t MS5611_ReadADC(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[3];
    HAL_I2C_Mem_Read(hi2c, MS5611_I2C_ADDR_HAL, MS5611_CMD_ADC_READ,
                     I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

/* Read 16-bit PROM value at given index */
static uint16_t MS5611_ReadPROM(I2C_HandleTypeDef *hi2c, uint8_t index)
{
    uint8_t buf[2];
    HAL_I2C_Mem_Read(hi2c, MS5611_I2C_ADDR_HAL,
                     MS5611_CMD_READ_PROM + (index * 2),
                     I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    return (buf[0] << 8) | buf[1];
}

void MS5611_Init(I2C_HandleTypeDef *hi2c, uint8_t mathMode)
{
    s_mathMode = mathMode;

    /* Reset the sensor */
    MS5611_SendCommand(hi2c, MS5611_CMD_RESET);
    HAL_Delay(3);

    /* Read factory calibration coefficients (C[0]=reservado, C[1..6]=coeficientes) */
    for (uint8_t reg = 0; reg < 7; reg++) {
        C[reg] = MS5611_ReadPROM(hi2c, reg);
    }
}

void MS5611_Measure(I2C_HandleTypeDef *hi2c, float *temperature, float *pressure)
{
    /* Start D1 (pressure) conversion */
    MS5611_SendCommand(hi2c, MS5611_CMD_CONV_D1 | MS5611_OSR_4096);
    HAL_Delay(MS5611_CONV_DELAY_MS);
    uint32_t D1 = MS5611_ReadADC(hi2c);

    /* Start D2 (temperature) conversion */
    MS5611_SendCommand(hi2c, MS5611_CMD_CONV_D2 | MS5611_OSR_4096);
    HAL_Delay(MS5611_CONV_DELAY_MS);
    uint32_t D2 = MS5611_ReadADC(hi2c);

    /* Matematica em ponto fixo de 64 bits, igual ao exemplo do datasheet.
     * Evita a perda de precisao do float32 no produto D1*SENS, que e a causa
     * da leitura de pressao errada. */
    int64_t dT   = (int64_t)D2 - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);

    int64_t OFF, SENS;
    if (s_mathMode == 0) {
        /* MS5611 */
        OFF  = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
        SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);
    } else {
        /* MS5607 */
        OFF  = ((int64_t)C[2] << 17) + (((int64_t)C[4] * dT) >> 6);
        SENS = ((int64_t)C[1] << 16) + (((int64_t)C[3] * dT) >> 7);
    }

    /* Compensacao de segunda ordem para baixas temperaturas */
    if (TEMP < 2000) {
        int64_t T2    = (dT * dT) >> 31;
        int64_t tdiff = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2  = (5 * tdiff) / 2;
        int64_t SENS2 = (5 * tdiff) / 4;

        if (TEMP < -1500) {
            int64_t tdiff2 = (TEMP + 1500) * (TEMP + 1500);
            OFF2  += 7 * tdiff2;
            SENS2 += (11 * tdiff2) / 2;
        }

        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = (((int64_t)D1 * SENS) >> 21) - OFF;
    P >>= 15;

    if (temperature) *temperature = TEMP * 0.01f; /* °C */
    if (pressure)    *pressure    = (float)P;      /* Pa */
}