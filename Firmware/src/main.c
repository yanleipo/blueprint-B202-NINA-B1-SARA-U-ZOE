//u-blox B202 http post demo application version 1.0

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

#define SCAN_INTERVAL                   0x00A0       /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050       /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                    0x0005       /**< Timout when scanning. 0x0000 disables timeout. */

#define APP_BLE_CONN_CFG_TAG            1            /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3            /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

#define MAX_NUMBER_OF_SCAN_RESULTS 9

bool scan_completed = false;
uint8_t imei[15];

typedef enum
{
    OFF,
    GREEN,
    BLUE,
    GREEN_BLUE,
} led_color_t;

typedef struct
{
    uint8_t addr[6];
    int8_t rssi;

} scan_result_t;

scan_result_t scan_result_list[30];
uint8_t scan_result_index;

static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 0,
    .use_whitelist = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
};


void uart_event_handle(app_uart_evt_t* p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


void putstring(char* s)
{
    uint32_t err_code;
    
    uint8_t len = strlen((char *) s);
    for (uint8_t i = 0; i < len; i++)
    {
        err_code = app_uart_put(s[i]);
        APP_ERROR_CHECK(err_code);
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
    send_command("AT+UHTTP=0\r\n");
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

    //HTTP post the file
    status = send_command("AT+UHTTPC=0,4,\"/http_post\",\"result.txt\",\"jsondata.txt\",4\r\n");
    if (!status)
        return false;
    
    return true;
}


void led_init()
{
    //Configure LEDs
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);

    //Set LEDs off
    nrf_gpio_pin_write(LED_1,1);
    nrf_gpio_pin_write(LED_2,1);
}


void led(led_color_t color)
{
    switch(color)
    {
        case OFF:
            nrf_gpio_pin_write(LED_1,1);
            nrf_gpio_pin_write(LED_2,1);
            break;
        case GREEN:
            nrf_gpio_pin_write(LED_1,0);
            nrf_gpio_pin_write(LED_2,1);
            break;
        case BLUE:
            nrf_gpio_pin_write(LED_1,1);
            nrf_gpio_pin_write(LED_2,0);
            break;
        case GREEN_BLUE:
            nrf_gpio_pin_write(LED_1,0);
            nrf_gpio_pin_write(LED_2,0);
            break;
    }
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


static void scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    memset(scan_result_list, 0, sizeof(scan_result_list));
    scan_result_index = 0;

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}


static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    uint8_t addr[6];
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
    strncpy(addr, peer_addr->addr, 6);
    int8_t rssi_dbm = p_ble_evt->evt.gap_evt.params.adv_report.rssi;

    for (uint8_t i=0; i < MAX_NUMBER_OF_SCAN_RESULTS; i++)
    {
        if (memcmp(scan_result_list[i].addr, addr, 6) == 0)
        {
            //Remote device already in list, update RSSI value
            scan_result_list[i].rssi = rssi_dbm;
            break;
        }
        if (i == scan_result_index)
        {
            //Remote device not in list, add device
            strncpy(scan_result_list[i].addr, addr, 6);
            scan_result_list[i].rssi = rssi_dbm;

            scan_result_index++;
            break;
        }
    }
}


static void send_scan_result()
{
    uint8_t temp_str[23];
    uint8_t* scan_result_str  = (uint8_t*)malloc(MAX_NUMBER_OF_SCAN_RESULTS * sizeof(temp_str));
    memset(scan_result_str, 0, MAX_NUMBER_OF_SCAN_RESULTS * sizeof(temp_str));

    for (uint8_t k=0; k < MAX_NUMBER_OF_SCAN_RESULTS; k++)
    {
        sprintf(temp_str, "%02X:%02X:%02X:%02X:%02X:%02X,%03d;",
            scan_result_list[k].addr[5],
            scan_result_list[k].addr[4],
            scan_result_list[k].addr[3],
            scan_result_list[k].addr[2],
            scan_result_list[k].addr[1],
            scan_result_list[k].addr[0],
            scan_result_list[k].rssi);
        
        strcat(scan_result_str, temp_str);
    }

    uint8_t* start_first_str = "{\"name\":\"";
    uint8_t* end_first_str = "\",\"scanResult\":\"";

    uint8_t langd = strlen(imei);
    uint8_t first_str_length = strlen(start_first_str) + strlen(imei) + strlen(end_first_str);
    uint8_t* first_str = (uint8_t*)malloc(first_str_length * sizeof(uint8_t*));
    memset(first_str, 0, first_str_length * sizeof(uint8_t*));
    
    strcat(first_str, start_first_str);
    strcat(first_str, imei);
    strcat(first_str, end_first_str);    
    char* first_str_char = (char*) first_str;

    uint8_t* third_str  = "\"}";
    
    uint8_t data_length = strlen(first_str) + strlen(scan_result_str) + strlen(third_str);
    uint8_t* data = (uint8_t*)malloc(data_length * sizeof(uint8_t*));
    memset(data, 0, data_length * sizeof(uint8_t*));
    
    strcat(data, first_str);
    strcat(data, scan_result_str);
    strcat(data, third_str);
    
    //Post json data
    bool status4 = http_post(data);
    free(data);
    free(scan_result_str);
    free(first_str);
}


static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            on_adv_report(p_ble_evt);
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Scan timeout
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                scan_completed = true;
            }
        } break;

        default:
            // No implementation needed.
            break;
    }
}


static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    uint32_t err_code;
    bool status1, status2, status3;

    nrf_delay_ms(10000);

    led_init();

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

    led(GREEN);

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    timer_init();
    ble_stack_init();

    if (false)
    {
        //Disable flow control
        status1 = send_command("AT&K0\r\n");
    }

    get_imei();

    //Configure for Telenor and IBM Bluemix
    do {
        status2 = config_apn_and_activate_ps_data("services.telenor.se");
    } while(status2 == false);
    do {
        status3 = set_http_server("odin-w2-dice-demo.eu-gb.mybluemix.net");
    } while(status3 == false);

    //Main loop
    scan_start();
    for (;;)
    {
        if (scan_completed == true)
        {
            led(BLUE);
            send_scan_result();
            scan_completed = false;

            nrf_delay_ms(2000);
            led(OFF);

            scan_start();
        }
    }
}
