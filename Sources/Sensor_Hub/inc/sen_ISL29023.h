#ifndef SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_
#define SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_

#include <stdbool.h>
#include <stdint.h>

void isl29023Init(void);

bool isl29023GetData(float* als_lx, uint16_t* ir); //gets the Ambient Light Sense (visible light) and the Infrared light value in lux

#endif /* SOURCES_SENSOR_HUB_INC_SEN_ISL29023_H_ */
