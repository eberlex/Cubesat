#include "bme68x_app.h"

/* Guarda o handle I2C e o endereco do dispositivo, passado via intf_ptr */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr; /* endereco de 7 bits */
} BME68X_I2C_Intf;

static struct bme68x_dev bme_dev;
static struct bme68x_conf bme_conf;
static struct bme68x_heatr_conf bme_heatr_conf;
static BME68X_I2C_Intf bme_i2c_intf;

/* Callback de Leitura I2C */
static BME68X_INTF_RET_TYPE bme_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    BME68X_I2C_Intf *i2c = (BME68X_I2C_Intf *)intf_ptr;

    if (HAL_I2C_Mem_Read(i2c->hi2c, (uint16_t)(i2c->dev_addr << 1), reg_addr,
                          I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) != HAL_OK) {
        return BME68X_E_COM_FAIL;
    }
    return BME68X_OK;
}

/* Callback de Escrita I2C */
static BME68X_INTF_RET_TYPE bme_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    BME68X_I2C_Intf *i2c = (BME68X_I2C_Intf *)intf_ptr;

    if (HAL_I2C_Mem_Write(i2c->hi2c, (uint16_t)(i2c->dev_addr << 1), reg_addr,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, 100) != HAL_OK) {
        return BME68X_E_COM_FAIL;
    }
    return BME68X_OK;
}

/* Delay em Microsegundos */
static void bme_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    uint32_t ms = period / 1000;
    HAL_Delay(ms == 0 ? 1 : ms);
}

/* Função de Inicialização Simplificada */
int8_t BME68X_App_Init(I2C_HandleTypeDef *hi2c) {
    int8_t rslt;

    bme_i2c_intf.hi2c = hi2c;
    bme_i2c_intf.dev_addr = BME68X_APP_I2C_ADDR;

    bme_dev.intf = BME68X_I2C_INTF;
    bme_dev.read = bme_i2c_read;
    bme_dev.write = bme_i2c_write;
    bme_dev.delay_us = bme_delay_us;
    bme_dev.intf_ptr = &bme_i2c_intf;
    bme_dev.amb_temp = 25;

    rslt = bme68x_init(&bme_dev);
    if (rslt != BME68X_OK) return rslt;

    bme_conf.filter = BME68X_FILTER_OFF;
    bme_conf.odr = BME68X_ODR_NONE;
    bme_conf.os_hum = BME68X_OS_16X;
    bme_conf.os_pres = BME68X_OS_1X;
    bme_conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&bme_conf, &bme_dev);
    if (rslt != BME68X_OK) return rslt;

    bme_heatr_conf.enable = BME68X_ENABLE;
    bme_heatr_conf.heatr_temp = 300;
    bme_heatr_conf.heatr_dur = 100;
    return bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heatr_conf, &bme_dev);
}

/* Função de Leitura Simplificada */
uint8_t BME68X_App_ReadData(BME68X_Data *data) {
    struct bme68x_data raw_data;
    uint8_t n_fields = 0;
    int8_t rslt;

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_dev);
    if (rslt != BME68X_OK) return 0;

    uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_conf, &bme_dev) + (bme_heatr_conf.heatr_dur * 1000);
    bme_dev.delay_us(del_period, bme_dev.intf_ptr);

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &raw_data, &n_fields, &bme_dev);

    if (rslt == BME68X_OK && n_fields > 0) {
#ifdef BME68X_USE_FPU
        data->temperature = raw_data.temperature;
        data->pressure = raw_data.pressure / 100.0f; // Converte Pa para hPa
        data->humidity = raw_data.humidity;
        data->gas_resistance = raw_data.gas_resistance;
#else
        data->temperature = raw_data.temperature / 100.0f;
        data->pressure = raw_data.pressure / 100.0f;
        data->humidity = raw_data.humidity / 1000.0f;
        data->gas_resistance = (float)raw_data.gas_resistance;
#endif
        data->status = raw_data.status;
        return 1; // Sucesso
    }

    return 0; // Falha
}