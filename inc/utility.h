#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

#include <stdbool.h>
#include <stddef.h>

//appends a float to a string with the specified precision and an optional comma to the very end, the number's maximum length must be specified (in characters)
bool appendFloat(char* str, float val, size_t precision, size_t maxLength, bool addComma);

//appends an int64_t to a string and an optional comma to the very end, the number's maximum length must be specified (in characters)
bool appendInt64(char* str, int64_t val, size_t maxLength, bool addComma);

//
bool appendUInt64(char* str, uint64_t val, size_t maxLength);


char* strtok_r(char* str, const char* delimStr, char** work);

bool strcmp_bool(const char* str1, const char* str2);
#endif /* INC_UTILITY_H_ */
