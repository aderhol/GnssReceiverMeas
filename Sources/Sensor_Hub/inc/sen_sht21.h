#ifndef SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_
#define SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_

#include <stdbool.h>

void sht21Init(void);

bool sht21GetData(float* temperature_C, float* relHum_percentage); //gets the temperature in °C and the relative humidity above water as a percentage

#endif /* SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_ */
