#ifndef __MAIN_H_
#define __MAIN_H_

#if defined(BOARD_PCA10056)
#define SENSOR_NOT_PRESENT 1
#endif

void B202_NUS_LOG(char *format, ...);

#define B202_LOG_ERROR(...) {B202_NUS_LOG(__VA_ARGS__); NRF_LOG_ERROR(__VA_ARGS__) ;}
#define B202_LOG_WARNING(...) {B202_NUS_LOG(__VA_ARGS__); NRF_LOG_WARNING(__VA_ARGS__) ;}
#define B202_LOG_INFO(...) {B202_NUS_LOG(__VA_ARGS__); NRF_LOG_INFO(__VA_ARGS__) ;}
#define B202_LOG_DEBUG(...) {B202_NUS_LOG(__VA_ARGS__); NRF_LOG_DEBUG(__VA_ARGS__) ;}

#endif