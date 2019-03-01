#ifndef __CELLULAR__
#define __CELLULAR__

bool cellular_set_apn(uint8_t* apn);

void cellular_init(void* uart_event_handle);

void cellular_step(void);

#endif