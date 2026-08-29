/**
  ******************************************************************************
  * @file    BMI323_STM32.h
  * @brief   Driver para o IMU BMI323 (Bosch) via SPI2 usando HAL STM32.
  *          O sensor da placa GY-601N1 do projeto e na verdade um BMI323
  *          (confirmado via CHIP_ID 0x43), cabeado no SPI2 conforme o
  *          conector Connector_ICM1 do pinout:
  *            ICM_CS   -> PB12 (GPIO, controlado por software)
  *            ICM_SCLK -> PB13 (SPI2_SCK)
  *            ICM_SDI  -> PB14 (SPI2_MISO no MCU)
  *            ICM_SDO  -> PB15 (SPI2_MOSI no MCU)
  *          OBS: confira fisicamente se SDI/SDO nao estao trocados em
  *          relacao ao MISO/MOSI do MCU. Se estiverem, ative
  *          hspi2.Init.IOSwap = SPI_IO_SWAP_ENABLE na MX_SPI2_Init em vez
  *          de alterar este driver.
  ******************************************************************************
  */

#ifndef BMI323_STM32_H
#define BMI323_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------- Configuração de Hardware ---------------- */
#define BMI323_SPI_HANDLE      hspi2
#define BMI323_CS_GPIO_Port    GPIOB
#define BMI323_CS_Pin          GPIO_PIN_12

/* Guarda o ultimo CHIP_ID lido (byte baixo), para debug via UART */
extern uint8_t g_bmi323_debug_chip_id;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
} BMI323_Data;

/* Inicializa o sensor: dummy read p/ selecionar modo SPI + soft reset +
 * configura accel (+-2g) e giro (+-125dps) a 800Hz em modo alta
 * performance. Retorna 0 em sucesso.
 * 1 = falha de comunicacao no soft reset
 * 2 = CHIP_ID nao bateu com 0x43 (confira fiacao/CS/SCLK/SDI/SDO)
 * 3 = falha ao configurar ACC_CONF
 * 4 = falha ao configurar GYR_CONF
 */
uint8_t BMI323_Init(void);

/* Le accel + giro + temperatura. Retorna 0 em sucesso. */
uint8_t BMI323_ReadData(BMI323_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* BMI323_STM32_H */