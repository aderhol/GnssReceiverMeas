//std library includes
#include <stdbool.h>
#include <stdint.h>

//FreeRTOS includes
#include <FreeRTOS.h>


//driver includes
#include <driverlib/sysctl.h> //System Control
#include <driverlib/fpu.h> //floating point unit API

//user includes
#include <init.h>




static const uint32_t CLOCK_FRQ_HZ = 120E6;
uint32_t SystemCoreClock; //actual system core clock frequency in Hz (required by FreeRTOS)


extern void interruptInit(void);
extern void heartBeatLedInit(void);
extern void uartInit(void);
extern void sensorHubInit(TickType_t maxDelay_ticks);
extern void skewMeasInit(void);
extern void commandInit(void);
extern void wdtInit(void);
void eepromIoInit(void);


void init(TickType_t maxDelay_ticks)
{
    //sets the system core clock frequency to CLOCK_FRQ_HZ, returns the actual frequency set
    SystemCoreClock = SysCtlClockFreqSet(((uint32_t)SYSCTL_XTAL_25MHZ |
                                            (uint32_t)SYSCTL_OSC_MAIN |
                                            (uint32_t)SYSCTL_USE_PLL |
                                            (uint32_t)SYSCTL_CFG_VCO_480),
                                         CLOCK_FRQ_HZ);
    
    FPUEnable(); //enables the floating point unit

    eepromIoInit(); //initializes the EEPROM interface

    interruptInit(); //sets up the NVIC
    
    heartBeatLedInit(); //initializes the hearBeatLED

    uartInit(); //initializes the UART peripherals

    sensorHubInit(maxDelay_ticks); //initializes the sensor hub

    skewMeasInit(); //initializes the skew measurement

    commandInit(); //initializes the command terminal




    wdtInit(); //enables the Watchdog Timer
}
