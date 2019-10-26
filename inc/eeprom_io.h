#ifndef INC_EEPROM_IO_H_
#define INC_EEPROM_IO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//addresses need to align on word boundaries (uint32_t), so the address needs to be divisible by 4
#define EEPROM_ADD_RESET_CAUSE (0) //32 bits + 32 bits CRC
#define EEPROM_ADD_PREV_RESET_CAUSE_VALID (8) //16 bits + 16 bits CRC
#define EEPROM_ADD_PREV_RESET_CAUSE (12) //32 bits + 32 bits CRC
#define EEPROM_ADD_CAUSE    (20) //16 bits + 16 bits CRC
#define EEPROM_ADD_DESCRIPTOR_BASE (24)

bool eepromGetVerifiedValue_ui16_ISR(uint32_t address, uint16_t* data_out);
void eepromWriteWithVerification_ui16_ISR(uint32_t address, uint16_t data);

bool eepromGetVerifiedValue_ui32_ISR(uint32_t address, uint32_t* data_out);
void eepromWriteWithVerification_ui32_ISR(uint32_t address, uint32_t data);

bool eepromGetVerifiedValue_ui8A_ISR(uint32_t address, uint8_t* data_out, size_t length, uint32_t* nxtAddress);
void eepromWriteWithVerification_ui8A_ISR(uint32_t address, const uint8_t* data, size_t length, uint32_t* nxtAddress);

#endif
