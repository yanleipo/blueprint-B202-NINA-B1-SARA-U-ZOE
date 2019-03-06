#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "sensor.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     1

#define PIN_SENSOR_I2C_SCL  3
#define PIN_SENSOR_I2C_SDA  2

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
static uint8_t                      m_req_buffer[2];
#define RX_BUFFER_SIZE              32
static uint8_t                      m_data_buffer[RX_BUFFER_SIZE];

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static int16_t m_temperature=0;

uint32_t i2c_write(nrf_drv_twi_t const * p_instance, uint8_t address, uint8_t const * p_data, uint8_t length, bool no_stop)
{
    ret_code_t err_code = NRF_ERROR_BUSY;
    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(p_instance, address, p_data, length, no_stop);
    NRF_LOG_DEBUG("nrf_drv_twi_tx (Addr: 0x%x data: 0x%x length: 0x%x) returns 0x%x.", address, p_data[0], length, err_code);
    nrf_delay_ms(10);
}

uint32_t i2c_read(nrf_drv_twi_t const * p_instance, uint8_t address, uint8_t * p_data, uint8_t length)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_rx(p_instance, address, p_data, length);
    nrf_delay_ms(10);
    NRF_LOG_DEBUG("nrf_drv_twi_rx (addr: 0x%x length: 0x%x) returns 0x%x. ", address, length, err_code);
}

void sensor_reset()
{
    NRF_LOG_INFO("bmi160_reset.");
    m_req_buffer[0] = CTRL3_C_REG;
    i2c_write(&m_twi, SENSOR_ADDRESS, m_req_buffer, 1, false);
    i2c_read(&m_twi, SENSOR_ADDRESS, m_data_buffer, 1);

    m_req_buffer[1] = m_data_buffer[0] | SW_RESET_VAL;
    i2c_write(&m_twi, SENSOR_ADDRESS, m_req_buffer, 2, false);
    nrf_delay_ms(50);
}

uint8_t sensor_read_chip_id()
{
    m_req_buffer[0] = WHO_AM_I_REG;
    i2c_write(&m_twi, SENSOR_ADDRESS, m_req_buffer, 1, false);
    i2c_read(&m_twi, SENSOR_ADDRESS, m_data_buffer, 1);
    NRF_LOG_INFO("Sensor chip ID: 0x%x.", m_data_buffer[0]);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    //NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(temperature);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bmi160_config = {
       .scl                = PIN_SENSOR_I2C_SCL,
       .sda                = PIN_SENSOR_I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_bmi160_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for main application entry.
 */
int sensor_init(void)
{
    NRF_LOG_INFO("Sensor init");

    twi_init();

    sensor_reset();
    sensor_read_chip_id();
}

int16_t sensor_read(void)
{
    m_req_buffer[0] = TEMPERATURE_REG;
    i2c_write(&m_twi, SENSOR_ADDRESS, m_req_buffer, 1, false);
    i2c_read(&m_twi, SENSOR_ADDRESS, m_data_buffer, 2);
    m_temperature = (int16_t) ((uint16_t)m_data_buffer[0] | ((uint16_t)(m_data_buffer[1]) << 8));

    //NRF_LOG_INFO("Temperature is %d", m_temperature);
    return m_temperature;
}

/** @} */
