//std library includes
#include <stdbool.h>
#include <stdint.h>

//FreeRTOS includes


//driver includes
#include <driverlib/interrupt.h> //interrupts

//user includes
#include <init.h>




void interruptInit(void)
{
    IntPriorityGroupingSet(2); //sets bits 7 & 6 to preemptable priority bits and bit 5 to sub priority bit
}
