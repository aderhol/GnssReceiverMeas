#ifndef SOURCES_SENSOR_HUB_INC_SENSORHUB_H_
#define SOURCES_SENSOR_HUB_INC_SENSORHUB_H_

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>

typedef struct{
    float temperature_C_bmp180;
    float pressure_Pa;

    float visibleLightIntensity_lx;
    uint16_t infraredLighIntensity; //unit unknown

    float temperature_C_sht21;
    float relativeHumidity_percentage; //above water
}SensorData;

bool sampleSensors(TickType_t maxDelay_ticks, SensorData* sensorData); //returns false if it couldn't get the samples, true if it was successful

#endif /* SOURCES_SENSOR_HUB_INC_SENSORHUB_H_ */
