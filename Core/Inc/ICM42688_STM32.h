/**
  ******************************************************************************
  * @file    ICM42688_STM32.h
  * @brief   Driver para o IMU ICM-42688-P (TDK InvenSense) via SPI usando HAL STM32.
  *          Sensibilidade (full-scale) e ODR sao configuraveis em tempo de
  *          execucao, passando uma ICM42688_Config para ICM42688_Init().
  ******************************************************************************
  * Ligacoes (modo SPI 4 fios):
  *   AP_CS    -> CS por software (ICM42688_CS_GPIO_Port/Pin)
  *   AP_SCLK  -> SCK do SPI escolhido
  *   AP_SDI   -> MOSI
  *   AP_SDO   -> MISO
  *   VDD, VDDIO -> 3.3V (capacitores: 0.1uF + 2.2uF em VDD, 10nF em VDDIO)
  *   GND      -> GND
  *   Pinos RESV -> GND ou nao conectados (ver datasheet)
  ******************************************************************************
  */

#ifndef ICM42688_STM32_H
#define ICM42688_STM32_H

#include "main.h"
#include <stdint.h>

/* ---------------- Configuracao de hardware (fiacao fixa da placa) ---------------- */
#define ICM42688_SPI_HANDLE        hspi2
#define ICM42688_CS_GPIO_Port      GPIOB
#define ICM42688_CS_Pin            GPIO_PIN_11

/* ---------------- Registradores (Bank 0) ---------------- */
#define ICM42688_REG_DEVICE_CONFIG     0x11
#define ICM42688_REG_PWR_MGMT0         0x4E
#define ICM42688_REG_GYRO_CONFIG0      0x4F
#define ICM42688_REG_ACCEL_CONFIG0     0x50
#define ICM42688_REG_TEMP_DATA1        0x1D
#define ICM42688_REG_ACCEL_DATA_X1     0x1F
#define ICM42688_REG_GYRO_DATA_X1      0x25
#define ICM42688_REG_WHO_AM_I          0x75
#define ICM42688_WHO_AM_I_VALUE        0x47

/* ---------------- Faixa (full-scale) do acelerometro ----------------
 * Valor do enum = valor a gravar em ACCEL_CONFIG0[7:5] */
typedef enum
{
    ICM42688_ACCEL_FS_16G = 0x00,
    ICM42688_ACCEL_FS_8G  = 0x01,
    ICM42688_ACCEL_FS_4G  = 0x02,
    ICM42688_ACCEL_FS_2G  = 0x03,
} ICM42688_AccelFS;

/* ---------------- Faixa (full-scale) do giroscopio ----------------
 * Valor do enum = valor a gravar em GYRO_CONFIG0[7:5] */
typedef enum
{
    ICM42688_GYRO_FS_2000DPS  = 0x00,
    ICM42688_GYRO_FS_1000DPS  = 0x01,
    ICM42688_GYRO_FS_500DPS   = 0x02,
    ICM42688_GYRO_FS_250DPS   = 0x03,
    ICM42688_GYRO_FS_125DPS   = 0x04,
    ICM42688_GYRO_FS_62_5DPS  = 0x05,
    ICM42688_GYRO_FS_31_25DPS = 0x06,
    ICM42688_GYRO_FS_15_625DPS= 0x07,
} ICM42688_GyroFS;

/* ---------------- Output Data Rate (usada por accel e gyro) ----------------
 * Valor do enum = valor a gravar em *_CONFIG0[3:0] */
typedef enum
{
    ICM42688_ODR_32KHZ  = 0x01,
    ICM42688_ODR_16KHZ  = 0x02,
    ICM42688_ODR_8KHZ   = 0x03,
    ICM42688_ODR_4KHZ   = 0x04,
    ICM42688_ODR_2KHZ   = 0x05,
    ICM42688_ODR_1KHZ   = 0x06,
    ICM42688_ODR_200HZ  = 0x07,
    ICM42688_ODR_100HZ  = 0x08,
    ICM42688_ODR_50HZ   = 0x09,
    ICM42688_ODR_25HZ   = 0x0A,
    ICM42688_ODR_12_5HZ = 0x0B,
} ICM42688_ODR;

/* ---------------- Configuracao do sensor (preenchida pelo usuario) ---------------- */
typedef struct
{
    ICM42688_AccelFS accel_fs;
    ICM42688_GyroFS  gyro_fs;
    ICM42688_ODR     odr;       /* mesma ODR usada para accel e gyro */
} ICM42688_Config;

/* ---------------- Dados lidos, ja convertidos ---------------- */
typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} ICM42688_Data;

/* ---------------- API ---------------- */

/**
 * @brief  Retorna uma configuracao padrao razoavel para CanSat:
 *         Accel +/-16g, Gyro +/-2000dps, ODR 1kHz.
 *         Use como ponto de partida e ajuste os campos que quiser.
 */
ICM42688_Config ICM42688_GetDefaultConfig(void);

/**
 * @brief  Inicializa o ICM-42688-P: confere WHO_AM_I, faz soft reset e aplica
 *         a configuracao (FS de accel/gyro e ODR) recebida em 'config'.
 * @param  config  Ponteiro para a configuracao desejada (ver ICM42688_GetDefaultConfig).
 * @retval 0 = OK, != 0 = erro (ver codigo em ICM42688_STM32.c)
 */
uint8_t ICM42688_Init(const ICM42688_Config *config);

/**
 * @brief  Le accel (g), giro (dps) e temperatura (C) e preenche a struct,
 *         usando a sensibilidade configurada em ICM42688_Init().
 * @retval 0 = OK, != 0 = erro de comunicacao SPI
 */
uint8_t ICM42688_ReadData(ICM42688_Data *data);

#endif /* ICM42688_STM32_H */
