#ifndef SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_
#define SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_

#include <FreeRTOS.h>

void bmp180Init(TickType_t maxDelay_ticks);

bool bmp180GetData(TickType_t maxDelay_ticks, float* temperature_C, float* pressure_Pa); //gets the temperature in °C and the pressure in pascals

#endif /* SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_ */
