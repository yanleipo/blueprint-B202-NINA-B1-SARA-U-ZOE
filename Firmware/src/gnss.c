#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_lns.h"
#include "App_scheduler.h"
#include "app_timer.h"
#include "gnss.h"
#include "main.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

#if defined(BOARD_PCA10040)
#define SPI_SS_PIN    SPIS_CSN_PIN
#define SPI_MISO_PIN  SPIS_MISO_PIN
#define SPI_MOSI_PIN  SPIS_MOSI_PIN
#define SPI_SCK_PIN   SPIS_SCK_PIN
#elif defined(BOARD_PCA10056)
#define SPI_SS_PIN    BSP_QSPI_CSN_PIN
#define SPI_MISO_PIN  BSP_QSPI_IO0_PIN
#define SPI_MOSI_PIN  BSP_QSPI_IO1_PIN
#define SPI_SCK_PIN   BSP_QSPI_SCK_PIN
#else
#error "SPI pins not defined"
#endif

#define m_spi_rx_length 1
#define m_gnss_buf_length 255
static uint8_t       m_rx_buf[1];
static const uint8_t m_length = sizeof(m_rx_buf);
static uint8_t       m_gnss_buffer[m_gnss_buf_length];
static uint8_t       m_gnss_index = 0;                               /**< GNSS buffer index. */
static bool          m_awaiting_new_message = true;                  /**< Awaiting new GNSS message flag. */

static bool                                  m_location_speed_data_valid = false;            /**< Flag indicating if received gnss data are valid. */

static bool                                  m_gnss_fix = false;                             /**< GNSS Fix. */

static ble_lns_loc_speed_t                   m_location_speed;                               /**< Location and speed data. */
static ble_lns_loc_speed_t                   m_last_read_location_speed;                     /**< Last location and speed data read from the  */

#define LOC_AND_NAV_DATA_INTERVAL            APP_TIMER_TICKS(1000)      /**< Location and Navigation data interval (ticks). */
#define SPI_READ_INTERVAL                    APP_TIMER_TICKS(10)        /**< SPI read interval (ticks). */

APP_TIMER_DEF(m_spi_timer_id);                                                               /**< SPI read timer. */

/**@brief hex to int.
 *
 * @details This function will convert a character hex to int.
 *
 * @param[in] ch  A character in hex.
 *
 * @return  The integer of the hexadecimal character.
 */
uint8_t character_hex_to_int(uint8_t ch) 
{
  if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  else if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  else
    return ch - '0';
}

/**@brief Process a GNRMC message.
 */
void process_GNRMC_message(void)
{
    uint8_t message_index, field, field_index;
    int32_t latitude = 0;  //Unit is in degrees with a resolution of 1/(10^7)
    int32_t longitude = 0; //Unit is in degrees with a resolution of 1/(10^7)
    uint8_t status = ' ';
    uint8_t north_south = ' ';
    uint8_t east_west = ' ';
    uint8_t pos_mode = ' ';
    
    //$xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
    //example: $GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57
    
    for (message_index=0,field=0,field_index=0; (message_index < m_gnss_index) && (field <= 13); message_index++)
    {
        if (m_gnss_buffer[message_index] == ',' || m_gnss_buffer[message_index] == '*')
        {
            field++;
            field_index = 0;
        }
        else
        {
            switch (field) {
                
                case 1: //Time
                    //Hour
                    if (field_index == 0)
                        m_last_read_location_speed.utc_time.hours = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        m_last_read_location_speed.utc_time.hours += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Minute
                    if (field_index == 2)
                        m_last_read_location_speed.utc_time.minutes = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        m_last_read_location_speed.utc_time.minutes += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Second
                    if (field_index == 4)
                        m_last_read_location_speed.utc_time.seconds = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 5)
                        m_last_read_location_speed.utc_time.seconds += character_hex_to_int(m_gnss_buffer[message_index]);                    
                    break;
                    
                case 2: //Status
                    status = m_gnss_buffer[message_index];
                    break;
                    
                case 3: //Latitude
                    if (field_index == 0)
                        latitude = 10000000*10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        latitude += 10000000*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 2)
                        latitude += (10000000*10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 3)
                        latitude += (10000000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    //field_index == 4 => '.' -> do nothing
                    if (field_index == 5)
                        latitude += (10000000/10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 6)
                        latitude += (10000000/100*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 7)
                        latitude += (10000000/1000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 8)
                        latitude += (10000000/10000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 9)
                        latitude += (10000000/100000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    break;
                    
                case 4: //North/South
                    north_south = m_gnss_buffer[message_index];
                    break;
                    
                case 5: //Longitude
                    if (field_index == 0)
                        longitude = 10000000*100*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        longitude += 10000000*10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 2)
                        longitude += 10000000*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        longitude += (10000000*10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 4)
                        longitude += (10000000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    //field_index == 5 => '.' -> do nothing
                    if (field_index == 6)
                        longitude += (10000000/10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 7)
                        longitude += (10000000/100*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 8)
                        longitude += (10000000/1000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 9)
                        longitude += (10000000/10000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 10)
                        longitude += (10000000/100000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    break;
                    
                case 6: //East/West
                    east_west = m_gnss_buffer[message_index];
                    break;
                    
                case 9: //Date
                    //Day
                    if (field_index == 0)
                        m_last_read_location_speed.utc_time.day = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        m_last_read_location_speed.utc_time.day += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Month
                    if (field_index == 2)
                        m_last_read_location_speed.utc_time.month = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        m_last_read_location_speed.utc_time.month += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Year
                    if (field_index == 4)
                        m_last_read_location_speed.utc_time.year = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 5)
                        m_last_read_location_speed.utc_time.year += character_hex_to_int(m_gnss_buffer[message_index]);                        
                    break;
                    
                case 12: //posMode
                    pos_mode = m_gnss_buffer[message_index];
                    //NRF_LOG_RAW_INFO("posMode\r\n");
                    break;
                    
                default:
                    break;
            }
            field_index++;
        }
    }

    //Data valid
    if (status == 'A')
        m_location_speed_data_valid = true;
    else
        m_location_speed_data_valid = false;

    //GNSS Fix
    if (pos_mode == 'A' || pos_mode == 'D' || pos_mode == 'E' || pos_mode == 'F' || pos_mode == 'R')
        m_gnss_fix = true;
    else
        m_gnss_fix = false;
    
    //Adjust latitude regarding North/South
    if (north_south == 'N')
        m_last_read_location_speed.latitude = latitude;
    else if (north_south == 'S')
        m_last_read_location_speed.latitude = -latitude;
    else 
        m_last_read_location_speed.latitude = 0;

    //Adjust longitude regarding East/West
    if (east_west == 'E')
        m_last_read_location_speed.longitude = longitude;
    else if (east_west == 'W')
        m_last_read_location_speed.longitude = -longitude;
    else 
        m_last_read_location_speed.longitude = 0;
   
    /* Debug */
    //NRF_LOG_RAW_INFO("%s\r\n",(uint32_t)m_gnss_buffer);
    //NRF_LOG_RAW_INFO("Time: %02d:%02d:%02d\r\n",m_last_read_location_speed.utc_time.hours,\
                     m_last_read_location_speed.utc_time.minutes,m_last_read_location_speed.utc_time.seconds);
    //NRF_LOG_RAW_INFO("Date: %02d-%02d-%02d\r\n",m_last_read_location_speed.utc_time.year, \
                     m_last_read_location_speed.utc_time.month,m_last_read_location_speed.utc_time.day);                     
    //NRF_LOG_RAW_INFO("Status: %c\r\n",status);
    //NRF_LOG_RAW_INFO("pos_mode: %c\r\n",pos_mode);
    //NRF_LOG_RAW_INFO("Lat:  %d deg\r\n",m_last_read_location_speed.latitude);
    //NRF_LOG_RAW_INFO("Long: %d deg\r\n",m_last_read_location_speed.longitude);
}

/**@brief Process a nmea message.
 */
void process_nmea_message(void)
{
    if ((m_gnss_buffer[1] == 'G') &&
        (m_gnss_buffer[2] == 'N') &&
        (m_gnss_buffer[3] == 'R') && 
        (m_gnss_buffer[4] == 'M') && 
        (m_gnss_buffer[5] == 'C'))
    {
        process_GNRMC_message();
    }

    //NRF_LOG_RAW_INFO("%s",(uint32_t)m_gnss_buffer);
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    if (m_awaiting_new_message == true)
    {
        if (m_rx_buf[0] == '$')
        {
            m_awaiting_new_message = false;
            memset(m_gnss_buffer, 0, sizeof(m_gnss_buffer));
            m_gnss_index = 0;
        }        
    }
    if (m_awaiting_new_message == false)
    {
        if (m_gnss_index >= 255)
        {
            m_awaiting_new_message = true;
        }
        else if (m_rx_buf[0] == '\n')
        {
            process_nmea_message();
            m_awaiting_new_message = true;            
        }
        else
        {
            m_gnss_buffer[m_gnss_index++] = m_rx_buf[0];
        }
    }
}

/**@brief SPI functionality initialization.
 */
static void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

/**@brief SPI time-out handler.
 *
 * @details This function will be called each time the SPI timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the time-out handler.
 */
 static void spi_timeout_handler(void * p_context)
{    
    ret_code_t err_code; 
    //NRF_LOG_INFO("spi_timeout_handler");

    //Reset rx buffer
    memset(m_rx_buf, 0, m_length);

    //Read SPI
    err_code = nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf, m_length);
    if(err_code) B202_LOG_ERROR("nrf_drv_spi_transfer failed. Err_code: 0x%x", err_code);
}

/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void gnss_timers_init(void)
{
    uint32_t err_code;
    
    err_code = app_timer_create(&m_spi_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                spi_timeout_handler);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Start application timers.
 */
static void gnss_timers_start(void)
{
    uint32_t err_code;
    
    err_code = app_timer_start(m_spi_timer_id, SPI_READ_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);    
}

void gnss_start(void)
{
    spi_init();
    gnss_timers_init();
    gnss_timers_start();
}

bool gnss_get_lns(int32_t* lat_p, int32_t* log_p, ble_date_time_t* utc_time_p)
{
    if(m_location_speed_data_valid == false)
        return (false);

    *lat_p = m_last_read_location_speed.latitude;
    *log_p = m_last_read_location_speed.longitude;
    *utc_time_p = m_last_read_location_speed.utc_time;

    return (true);
}