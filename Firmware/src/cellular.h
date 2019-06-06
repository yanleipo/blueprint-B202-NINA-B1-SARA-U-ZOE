#ifndef __CELLULAR__
#define __CELLULAR__

bool cellular_set_apn(uint8_t* apn);

bool cellular_set_server_url(uint8_t* server_url);

void cellular_init(void* uart_event_handle);

void cellular_step(void);

void process_UART_response(uint8_t* buf, uint8_t len);

#endif