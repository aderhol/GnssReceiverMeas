#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include <stdbool.h>
#include <stdint.h>

//adds the checksum to the string, and the $ and * chars, if needed, and the \r\n if requested
//returns true if successful, false otherwise (str wasn't long enough)
bool addChecksum(char str[], size_t maxLength, bool hasStartChar, bool hasEndChar, bool addNewLine);

#endif /* INC_NMEA_H_ */
