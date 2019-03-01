#ifndef __GNSS__
#define __GNSS__

#include "ble_date_time.h"

void gnss_start(void);
bool gnss_get_lns(int32_t* lat_p, int32_t* log_p, ble_date_time_t* utc_time_p);

#endif