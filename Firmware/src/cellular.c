#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "boards.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gnss.h"
#include "app_timer.h"
#include "ble_date_time.h"
#include "sensor.h"
#include "main.h"

extern bool m_dbg_UART_passthrough;                  /**< UART passthrough flag to overwrite LED status */

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

static app_uart_buffers_t m_buffers;
static uint8_t     m_rx_buf[UART_RX_BUF_SIZE];
static uint8_t     m_tx_buf[UART_TX_BUF_SIZE];

uint8_t imei[15];

#define DEFAULT_APN "internet"
#define MAX_APN_SIZE 32
static uint8_t m_apn[MAX_APN_SIZE];
#define MAX_SERVER_URL_SIZE 64
static uint8_t m_server_url[MAX_SERVER_URL_SIZE];

APP_TIMER_DEF(m_cel_timer_id);                                                               /**< cel timer. */
#define CEL_INTERVAL                    APP_TIMER_TICKS(3000)        /**< SPI read interval (ticks). */
#define CEL_COUNTER_MAX                 10/*3*/
#define HTTP_SERVER   "b202-thingsboard.ddns.net"

#define SERVER_ACCESS_TOKEN_SIZE 20
static uint8_t m_server_access_token[SERVER_ACCESS_TOKEN_SIZE];

static bool m_step_ready = false;
static uint8_t m_cel_timer_count = 0;

/* If location is not available, use default u-blox Thalwil location (47.2850880,8.5657170)  */
#define DEFAULT_LAT     472850880
#define DEFAULT_LONG    85657170

typedef enum
{
    CEL_NOT_READY = 0,
    CEL_READY_TO_REGISTER,
    CEL_REGISTERED,
    CEL_CONNECTED
} cel_status_t;

static cel_status_t m_cel_status = CEL_NOT_READY;

typedef enum
{
    OFF,
    GREEN,
    BLUE,
    GREEN_BLUE,
} led_color_t;
static bool m_led_show=false;

void led(led_color_t color)
{
    switch(color)
    {
        case OFF:
            bsp_board_led_off(0);
            bsp_board_led_off(1);
            break;
        case GREEN:
            bsp_board_led_on(0);
            bsp_board_led_off(1);
            break;
        case BLUE:
            bsp_board_led_off(0);
            bsp_board_led_on(1);
            break;
        case GREEN_BLUE:
            bsp_board_led_on(0);
            bsp_board_led_on(1);
            break;
    }
}

void putstring(char* s)
{
    uint32_t err_code;
    
    uint8_t len = strlen((char *) s);
    for (uint8_t i = 0; i < len; i++)
    {
        err_code = app_uart_put(s[i]);
        if(err_code) B202_LOG_ERROR("app_uart_put. Err_code: 0x%x", err_code);
    }
}


uint8_t read_line(char* buf)
{
    uint8_t count = 0;
    uint8_t x = 0;
    char end = '\n';
 
    for (count = 0; (count < UART_RX_BUF_SIZE) && (x >= 0) && (x != end); count++) {
        while (app_uart_get(&x) != NRF_SUCCESS);
        *(buf + count) = (char) x;
    }
 
    return count;
}


bool send_command(char* command)
{
    uint8_t in[UART_RX_BUF_SIZE];

    putstring(command);

    while (true)
    {
        read_line(in);

        if (strstr(in,"OK"))
        {
            return true;
        }
        if (strstr(in,"ERROR"))
        {
            return false;
        }
    }
}


bool get_imei()
{
    uint8_t in[UART_RX_BUF_SIZE];

    putstring("AT+GSN\r\n");

    while (true)
    {
        read_line(in);

        if (strstr(in,"AT+GSN"))
        {
            break;
        }
    }

    read_line(in);
    for (uint8_t i=0; i < 15;i++)
    {
        imei[i] = (uint8_t)in[i];
    }

    B202_LOG_INFO("IMEI: %s", imei);
    return true;
}


bool config_apn_and_activate_ps_data(uint8_t* apn)
{
    bool status;

    //Register (attach) the MT to the GPRS service
    status = send_command("AT+CGATT=1\r\n");
    if (!status)
        return false;

    //Deactivate packet switched data
    status = send_command("AT+UPSDA=0,4\r\n");
    //Do not check status, because it will be false if no active PSD profile.

    //Reset packet switched data
    status = send_command("AT+UPSDA=0,0\r\n");
    if (!status)
        return false;

    //Set up APN for GPRS connection profile "0"
    uint8_t* ps_config_apn_cmd_start = "AT+UPSD=0,1,\"";
    uint8_t* cmd_end = "\"\r\n";

    uint8_t length_cmd = strlen(ps_config_apn_cmd_start) + strlen(apn) + strlen(cmd_end);
    uint8_t* config_apn_cmd = (uint8_t*)malloc(length_cmd * sizeof(uint8_t*));
    memset(config_apn_cmd, 0, length_cmd * sizeof(uint8_t*));

    strcat(config_apn_cmd, ps_config_apn_cmd_start);
    strcat(config_apn_cmd, apn);
    strcat(config_apn_cmd, cmd_end);

    status = send_command(config_apn_cmd);
    free(config_apn_cmd);
    if (!status)
        return false;

    //Set up the dynamic IP address assignment
    status = send_command("AT+UPSD=0,7,\"0.0.0.0\"\r\n");
    if (!status)
        return false;

    //Activate packet switched data
    status = send_command("AT+UPSDA=0,3\r\n");
    if (!status)
        return false;

    return true;
}


bool set_http_server(uint8_t* server_name)
{
    bool status;

    //Reset HTTP profile #0
    status = send_command("AT+UHTTP=0\r\n");
    if (!status)
        return false;

    uint8_t* http_config_cmd_start = "AT+UHTTP=0,1,\"";
    uint8_t* cmd_end = "\"\r\n";

    uint8_t length_cmd = strlen(http_config_cmd_start) + strlen(server_name) + strlen(cmd_end);
    uint8_t* server_name_cmd = (uint8_t*)malloc(length_cmd * sizeof(uint8_t*));
    memset(server_name_cmd, 0, length_cmd * sizeof(uint8_t*));

    //Create complete AT command
    strcat(server_name_cmd, http_config_cmd_start);
    strcat(server_name_cmd, server_name);
    strcat(server_name_cmd, cmd_end);

    //Set HTTP server name
    status = send_command(server_name_cmd);
    free(server_name_cmd);
    if (!status)
        return false;

    send_command("AT+UHTTP=0,5,8080\r\n");
    if (!status)
        return false;

    return true;
}


bool http_post(uint8_t* data)
{
    bool status;
    uint8_t x;
    uint8_t in[UART_RX_BUF_SIZE];

    //Delete previously created file
    status = send_command("AT+UDELFILE=\"jsondata.txt\"\r\n");
    //Do not check status, because it will be false if no file with the specified name exist.

    //Create file with json data
    uint8_t length_of_data_str[12];
    uint8_t length_of_data = strlen(data);
    sprintf(length_of_data_str, "%d", length_of_data);
    uint8_t* create_file_cmd_start = "AT+UDWNFILE=\"jsondata.txt\",";
    uint8_t* cmd_end = "\r\n";

    uint8_t length_cmd = strlen(create_file_cmd_start) + strlen(length_of_data_str) + strlen(cmd_end);
    uint8_t* create_file_cmd = (uint8_t*)malloc(length_cmd * sizeof(uint8_t*));
    memset(create_file_cmd, 0, length_cmd * sizeof(uint8_t*));

    //Create complete AT command
    strcat(create_file_cmd, create_file_cmd_start);
    strcat(create_file_cmd, length_of_data_str);
    strcat(create_file_cmd, cmd_end);

    NRF_LOG_HEXDUMP_DEBUG(create_file_cmd, length_cmd);

    //Send command
    char* cmd_char = (char*) create_file_cmd;
    putstring(create_file_cmd);
    free(create_file_cmd);

    //Input data to file
    do {
        while (app_uart_get(&x) != NRF_SUCCESS);
    } while (x != '>');
    putstring(data);
    
    while (true)
    {
        read_line(in);

        if (strstr(in,"OK"))
        {
            break;
        }
        if (strstr(in,"ERROR"))
        {
            return false;
        }
    }

    /* Add 20ms delay between AT commands */
    nrf_delay_ms(20);

    //HTTP post the file
    //status = send_command("AT+UHTTPC=0,4,\"/http_post\",\"result.txt\",\"jsondata.txt\",4\r\n");
    uint8_t* postcmd_start = "AT+UHTTPC=0,4,\"/api/v1/";
    uint8_t* postcmd_end = "/telemetry\",\"result.txt\",\"jsondata.txt\",4\r\n";
    length_cmd = strlen(postcmd_start) + strlen(postcmd_end) + SERVER_ACCESS_TOKEN_SIZE;
    uint8_t* postcmd = (uint8_t*)malloc(length_cmd * sizeof(uint8_t*));
    memset(postcmd, 0, length_cmd * sizeof(uint8_t*));

    //Create complete AT command
    strcat(postcmd, postcmd_start);
    strncat(postcmd, m_server_access_token, SERVER_ACCESS_TOKEN_SIZE);
    strcat(postcmd, postcmd_end);

    NRF_LOG_HEXDUMP_DEBUG(data, strlen(data));
    NRF_LOG_HEXDUMP_DEBUG(postcmd, length_cmd);

    status = send_command(postcmd);
    free(postcmd);

    if (!status)
        return false;
    
    return true;
}

bool cellular_set_apn(uint8_t* apn)
{
    if(strlen(apn) > MAX_APN_SIZE)
    {
        B202_LOG_ERROR("APN size(%d) is bigger than maximum size(%d)\n", strlen(apn), MAX_APN_SIZE);
        return false;
    }
    memset(m_apn, 0, MAX_APN_SIZE);
    memcpy(m_apn, apn, strlen(apn));
    B202_LOG_INFO("APN set to: %s(%d)\n", m_apn, strlen(m_apn));
    return true;
}

bool cellular_set_server_url(uint8_t* server_url)
{
    if(strlen(server_url) > MAX_SERVER_URL_SIZE)
    {
        B202_LOG_ERROR("Server name size(%d) is bigger than maximum size(%d)\n", strlen(server_url), MAX_SERVER_URL_SIZE);
        return false;
    }
    memset(m_server_url, 0, MAX_SERVER_URL_SIZE);
    memcpy(m_server_url, server_url, strlen(server_url));
    B202_LOG_INFO("Server set to: %s(%d)\n", m_server_url, strlen(m_server_url));
    return true;
}

static void cel_timeout_handler(void * p_context)
{   
    m_step_ready = true;

    if(m_dbg_UART_passthrough)
    {
        led(BLUE);
    } else
    {
        /* Light LED to show CEL status */
        m_led_show = !m_led_show;
        if(m_led_show)
        {
            switch(m_cel_status)
            {
                case CEL_NOT_READY:
                    led(GREEN_BLUE);
                    break;
                case CEL_READY_TO_REGISTER:
                    led(GREEN);
                    break;
                case CEL_REGISTERED:
                    led(GREEN_BLUE);
                    break;
                case CEL_CONNECTED:
                    led(BLUE);
                    break;
            }
        } else
        {
            led(OFF);
        }
    }
}

static void cellular_timers_start(void)
{
    uint32_t err_code;
    
    err_code = app_timer_create(&m_cel_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                cel_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_cel_timer_id, CEL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);    
}

void cellular_init(void* uart_event_handle)
{
    uint32_t err_code;
    bool status;

    cellular_set_apn(DEFAULT_APN);
    memset(m_server_access_token, '0', sizeof(m_server_access_token));
    cellular_set_server_url(HTTP_SERVER);

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          true,                      /*Flow control, enable/disable.*/
          false,                     /*Even parity if TRUE, no parity if FALSE.*/
          NRF_UART_BAUDRATE_115200
      };
                                                                                                   
    m_buffers.rx_buf      = m_rx_buf;
    m_buffers.rx_buf_size = sizeof (m_rx_buf);
    m_buffers.tx_buf      = m_tx_buf;
    m_buffers.tx_buf_size = sizeof (m_tx_buf);
    err_code = app_uart_init(&comm_params, &m_buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
    APP_ERROR_CHECK(err_code);

    cellular_timers_start();
}

void cellular_step(void)
{
    if(m_step_ready == true)
    {
        bool status = false;

        B202_LOG_INFO("cellular status: %d", m_cel_status);

        m_step_ready = false;

        switch(m_cel_status)
        {
            case CEL_NOT_READY:

                //Wait for SARA-U201 module to be ready to operate, typical
                //6 seconds see SARA-U2 series System Integration Manual.
                //Delay set to 10 seconds.
                m_cel_timer_count++;
                B202_LOG_INFO("cellular timer counter: %d", m_cel_timer_count);
                if(m_cel_timer_count>=CEL_COUNTER_MAX)
                {
                    get_imei();
                    m_cel_status = CEL_READY_TO_REGISTER;

                    /* Server access token is 20bytes. It's set as IMEI of the CEL with 0 padding at the end */
                    memcpy(m_server_access_token, imei, sizeof(imei));
                    B202_LOG_INFO("Server access token set to: %s(%d).", m_server_access_token, strlen(m_server_access_token));
                }
                break;
            case CEL_READY_TO_REGISTER:
                status = config_apn_and_activate_ps_data(m_apn);
                if(status)
                {
                    m_cel_status = CEL_REGISTERED;
                    B202_LOG_INFO("Registered to %s and activated ps_data\n", m_apn);
                } else
                {
                    B202_LOG_ERROR("Failed to config apn %s(%d) and activate ps_data\n", m_apn, sizeof(m_apn));
                }
                break;
            case CEL_REGISTERED:
                status = set_http_server(m_server_url);
                if(status)
                {
                    m_cel_status = CEL_CONNECTED;
                    B202_LOG_INFO("Connected to server %s\n", m_server_url);
                } else
                {
                    B202_LOG_ERROR("Failed to set_http_server\n");
                }
                break;
            case CEL_CONNECTED:
                {
                    uint8_t fix = 0;
                    int32_t lat = 0;
                    int32_t log = 0;
                    ble_date_time_t utc_time;
                    int16_t temperature = 0;
                    bool status = false;

                    temperature = sensor_read();
                    status = gnss_get_lns(&lat, &log, &utc_time);
                    if(status)
                    {
                        //B202_LOG_DEBUG("Tracker location: (%d, %d)\n", lat, log);
                        fix = 1;
                    }
                    else
                    {
                        //B202_LOG_DEBUG("Unable to get location. Use default location\n");
                        fix = 0;
                        lat = DEFAULT_LAT;
                        log = DEFAULT_LONG;
                    }
                
                    uint8_t data[100];
                    memset(data, 0, sizeof(data));
                    sprintf(data, "{\"lat\":%d.%d,\"long\":%d.%d,\"fix\":%d,\"temperature\":%d}", lat/10000000, lat<0?-lat%10000000:lat%10000000, log/10000000, log<0?-log%10000000:log%10000000, fix, temperature);
                    status = http_post(data);
                    if(!status)
                    {
                        B202_LOG_ERROR("http_post failed");
                    }
                 }   
                break;
            default:
                B202_LOG_DEBUG("Unhandled celular status: %d\n", m_cel_status);
                break;
       }
   }

}