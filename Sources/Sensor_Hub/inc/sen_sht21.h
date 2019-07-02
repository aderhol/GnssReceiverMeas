#ifdef szarrago
#ifndef SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_
#define SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_

void sht21Init(tI2CMInstance* sensorI2C);

void sht21GetData(float* temperature_C, float* relHum_percentage); //gets the temperature in °C and the relative humidity above water as a ratio (0 - 1)

#endif /* SOURCES_SENSOR_HUB_INC_SEN_SHT21_H_ */
#endif
