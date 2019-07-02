#ifndef SOURCES_SENSOR_HUB_INC_SENHUBI2C_H_
#define SOURCES_SENSOR_HUB_INC_SENHUBI2C_H_

#include <stdint.h>
#include <stdbool.h>

size_t senHubI2cWrite(uint8_t slaveAddress, uint8_t values[], size_t length); //transmits all the values, returns the number of values successfully transmitted
bool senHubI2cWriteReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t value); //writes a register

size_t senHubI2cRead(uint8_t slaveAddress, uint8_t data[], size_t length); //reads the specified number of bytes, returns the number of values read
size_t senHubI2cReadReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t data[], size_t length); //reads the specified number of bytes starting from the specified register, returns the number of values read

bool senHubI2cReadModifyWriteReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t mask, uint8_t value); //reg = (reg & (~mask)) | value

#endif /* SOURCES_SENSOR_HUB_INC_SENHUBI2C_H_ */
