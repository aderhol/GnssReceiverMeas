#ifdef szarrago
#ifndef SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_
#define SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_

void isl29023Init(tI2CMInstance* sensorI2C);

void isl29023GetData(float* als, float* ir); //gets the Ambient Light Sense (visible light) and the Infrared light value in lux

#endif /* SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_ */
#endif
