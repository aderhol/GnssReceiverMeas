#ifndef SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_
#define SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_


void bmp180Init(void);

bool bmp180GetData(float* temperature_C, float* pressure_Pa); //gets the temperature in °C and the pressure in pascals

#endif /* SOURCES_SENSOR_HUB_INC_SEN_BMP180_H_ */
