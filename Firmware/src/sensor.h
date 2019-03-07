#ifndef __SENSOR_H__
#define __SENSOR_H__

#define SENSOR_ADDRESS              0x6A
#define	WHO_AM_I_REG                0x0F
#define TEMPERATURE_REG             0x20
#define CTRL3_C                     0x12

#define CTRL9_XL                    0x18
#define CTRL1_XL                    0x10
#define INT1_CTRL                   0x0D
#define CTRL10_C                    0x19
#define CTRL2_G                     0x11

#define CTRL9_XL_VAL                0x38
#define CTRL1_XL_VAL                0x60
#define CTRL10_C_VAL                0x38
#define CTRL2_G_VAL                 0x60
#define INT1_CTRL_VAL               0x03

#define SW_RESET_VAL                0x01

int sensor_init(void);

/* Output is in 0.01 degree C, e.g. 2510 means 25.1 degree C */
int16_t sensor_read(void);

#endif //__BMI160_H__
