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
    IntPriorityGroupingSet(8); //sets all interrupt bits for preemptable priority bits
}
