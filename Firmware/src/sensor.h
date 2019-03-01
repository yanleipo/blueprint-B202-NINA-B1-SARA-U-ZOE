#ifndef __SENSOR_H__
#define __SENSOR_H__

#define SENSOR_ADDRESS              0x6A
#define	WHO_AM_I_REG                0x0F
#define TEMPERATURE_REG             0x20
#define CTRL3_C_REG                 0x12

#define SW_RESET_VAL                0x01

int sensor_init(void);
int16_t sensor_read(void);

#endif //__BMI160_H__
