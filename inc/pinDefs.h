#ifndef HEADER_GUARD_pinDefs
#define HEADER_GUARD_pinDefs

#include <stdint.h>

typedef struct{
    uint32_t periphery;
	uint32_t portBase;
	uint8_t pin; 
}PinDef;


extern const PinDef LED1;
extern const PinDef LED2;
extern const PinDef LED3;
extern const PinDef LED4;
extern const PinDef bp_LED;
extern const PinDef bp_S1;
extern const PinDef bp_S2;








#endif
