#ifndef SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_
#define SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_

#include <FreeRTOS.h>
#include <stdbool.h>

void sht21Init(TickType_t maxDelay_ticks);

bool sht21GetData(TickType_t maxDelay_ticks, float* temperature_C, float* relHum_percentage); //gets the temperature in °C and the relative humidity above water as a percentage

#endif /* SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_ */
