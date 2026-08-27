#ifndef BME68X_APP_H_
#define BME68X_APP_H_

#include "main.h"
#include "bme68x.h"

/* Endereco I2C do BME680: 0x76 se SDO=GND, 0x77 se SDO=VCC */
#define BME68X_APP_I2C_ADDR   BME68X_I2C_ADDR_LOW   /* 0x76 - troque para BME68X_I2C_ADDR_HIGH se SDO estiver em VCC */

/* Struct simplificada para retornar as leituras para a main */
typedef struct {
    float temperature;   // em °C
    float pressure;      // em hPa
    float humidity;      // em %
    float gas_resistance;// em Ohms
    uint8_t status;
} BME68X_Data;

/* Protótipos das funções simples */
int8_t BME68X_App_Init(I2C_HandleTypeDef *hi2c);
uint8_t BME68X_App_ReadData(BME68X_Data *data);

#endif /* BME68X_APP_H_ */