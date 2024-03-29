//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

//driver includes
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"    //interrupt defines (eg. INT_UART0)
#include "inc/hw_types.h" //HWREG() macro
#include "driverlib/interrupt.h"    //interrupt API
#include "driverlib/sysctl.h" //System Control
#include "driverlib/watchdog.h" //Watchdog driver

//user includes
#include <wdt.h>
#include <uartIO.h>
#include <eeprom_io.h>
#include <utility.h>


//configurations
static const float wdtPeriod_ms = 500; //the period of the WDT

static const uint16_t faultCause_none = 0;
static const uint16_t faultCause_wdt0 = 1;
static const uint16_t faultCause_wdt1 = 2;
static const uint16_t faultCause_moscFailNmi = 3;
static const uint16_t faultCause_tamperNmi = 4;
static const uint16_t faultCause_powerNmi = 5;
static const uint16_t faultCause_extNmi = 6;

static const uint16_t faultCause_hardFault = 7;
static const uint16_t faultCause_mpuFault = 8;
static const uint16_t faultCause_busFault = 9;
static const uint16_t faultCause_usageFault = 10;
static const uint16_t faultCause_unexpectedInterrupt = 11;


static const uint32_t eepromAdd_reset_cause = EEPROM_ADD_RESET_CAUSE; //the address of the saved reset cause register
static const uint32_t eepromAdd_prv_reset_cause_valid = EEPROM_ADD_PREV_RESET_CAUSE_VALID; //the address of the value indicating if the saved previous reset cause is valid (1->valid)
static const uint32_t eepromAdd_prv_reset_cause = EEPROM_ADD_PREV_RESET_CAUSE; //the address of the saved previous reset cause register
static const uint32_t eepromAdd_cause = EEPROM_ADD_CAUSE; //the address of the fault reset's cause
static const uint32_t eepromAdd_descriptor_base = EEPROM_ADD_DESCRIPTOR_BASE; //the base address of the fault descriptor


const uint32_t eepromAdd_wdt_totTime = eepromAdd_descriptor_base;
const uint32_t eepromAdd_wdt_PC = eepromAdd_wdt_totTime + 8; //the address of the program counter's (PC) value before the fault occurred
const uint32_t eepromAdd_wdt_nameLen = eepromAdd_wdt_PC + 8;
const uint32_t eepromAdd_wdt_name = eepromAdd_wdt_nameLen + 4;

const uint32_t eepromAdd_unexpectedInt_pend0 = eepromAdd_descriptor_base;
const uint32_t eepromAdd_unexpectedInt_pend1 = eepromAdd_unexpectedInt_pend0 + 8;
const uint32_t eepromAdd_unexpectedInt_pend2 = eepromAdd_unexpectedInt_pend1 + 8;
const uint32_t eepromAdd_unexpectedInt_pend3 = eepromAdd_unexpectedInt_pend2 + 8;
const uint32_t eepromAdd_unexpectedInt_active0 = eepromAdd_unexpectedInt_pend3 + 8;
const uint32_t eepromAdd_unexpectedInt_active1 = eepromAdd_unexpectedInt_active0 + 8;
const uint32_t eepromAdd_unexpectedInt_active2 = eepromAdd_unexpectedInt_active1 + 8;
const uint32_t eepromAdd_unexpectedInt_active3 = eepromAdd_unexpectedInt_active2 + 8;

const uint32_t eepromAdd_hardFault_PC = eepromAdd_descriptor_base;
const uint32_t eepromAdd_hardFault_memoryManagementFaultAddress = eepromAdd_hardFault_PC + 8;
const uint32_t eepromAdd_hardFault_busFaultAddress = eepromAdd_hardFault_memoryManagementFaultAddress + 8;
const uint32_t eepromAdd_hardFault_configurableFaultStatus = eepromAdd_hardFault_busFaultAddress + 8;
const uint32_t eepromAdd_hardFault_hardFaultStatus = eepromAdd_hardFault_configurableFaultStatus + 8;

const uint32_t eepromAdd_mpuFault_PC = eepromAdd_descriptor_base;
const uint32_t eepromAdd_mpuFault_memoryManagementFaultAddress = eepromAdd_mpuFault_PC + 8;
const uint32_t eepromAdd_mpuFault_configurableFaultStatus = eepromAdd_mpuFault_memoryManagementFaultAddress + 8;

const uint32_t eepromAdd_busFault_PC = eepromAdd_descriptor_base;
const uint32_t eepromAdd_busFault_busFaultAddress = eepromAdd_busFault_PC + 8;
const uint32_t eepromAdd_busFault_configurableFaultStatus = eepromAdd_busFault_busFaultAddress + 8;

const uint32_t eepromAdd_usageFault_PC = eepromAdd_descriptor_base;
const uint32_t eepromAdd_usageFault_configurableFaultStatus = eepromAdd_usageFault_PC + 8;

extern uint32_t __get_PSP(void);
extern uint32_t __get_MSP(void);
extern bool getTaskList(TaskHandle_t (*taskList)[], size_t* numberOfTasks, size_t outputArrayLenght);

const uint32_t CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS = 0xE000E000;


void ISR_WDT_HardFault(void)
{
    uint32_t PC = *(((volatile uint32_t*)__get_PSP()) + 6); //gets the program counter where execution would resume after the ISR

    uint32_t memoryManagmentFaultAddress = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD34u);
    uint32_t busFaultAddress = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD38u);

    uint32_t configurableFaultStatus = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD28u);
    uint32_t hardFaultStatus = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD2Cu);

    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_hardFault);

    eepromWriteWithVerification_ui32_ISR(eepromAdd_hardFault_PC, PC);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_hardFault_memoryManagementFaultAddress, memoryManagmentFaultAddress);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_hardFault_busFaultAddress, busFaultAddress);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_hardFault_configurableFaultStatus, configurableFaultStatus);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_hardFault_hardFaultStatus, hardFaultStatus);

    SysCtlReset();
}

void ISR_WDT_MpuFault(void)
{
    uint32_t PC = *(((volatile uint32_t*)__get_PSP()) + 6); //gets the program counter where execution would resume after the ISR

    uint32_t memoryManagmentFaultAddress = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD34u);

    uint32_t configurableFaultStatus = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD28u);

    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_mpuFault);

    eepromWriteWithVerification_ui32_ISR(eepromAdd_mpuFault_PC, PC);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_mpuFault_memoryManagementFaultAddress, memoryManagmentFaultAddress);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_mpuFault_configurableFaultStatus, configurableFaultStatus);

    SysCtlReset();
}

void ISR_WDT_BusFault(void)
{
    uint32_t PC = *(((volatile uint32_t*)__get_PSP()) + 6); //gets the program counter where execution would resume after the ISR

    uint32_t busFaultAddress = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD38u);

    uint32_t configurableFaultStatus = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD28u);

    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_busFault);

    eepromWriteWithVerification_ui32_ISR(eepromAdd_busFault_PC, PC);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_busFault_busFaultAddress, busFaultAddress);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_busFault_configurableFaultStatus, configurableFaultStatus);

    SysCtlReset();
}

void ISR_WDT_UsageFault(void)
{
    uint32_t PC = *(((volatile uint32_t*)__get_PSP()) + 6); //gets the program counter where execution would resume after the ISR

    uint32_t configurableFaultStatus = HWREG(CORE_PREIPHERAL_REGISTERS_BASE_ADDRESS + 0xD28u);

    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_usageFault);

    eepromWriteWithVerification_ui32_ISR(eepromAdd_usageFault_PC, PC);
    eepromWriteWithVerification_ui32_ISR(eepromAdd_usageFault_configurableFaultStatus, configurableFaultStatus);

    SysCtlReset();
}

void ISR_Default(void)
{
    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_unexpectedInterrupt);

    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_pend0, HWREG(NVIC_BASE + 0x200u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_pend1, HWREG(NVIC_BASE + 0x204u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_pend2, HWREG(NVIC_BASE + 0x208u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_pend3, HWREG(NVIC_BASE + 0x20Cu));

    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_active0, HWREG(NVIC_BASE + 0x300u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_active1, HWREG(NVIC_BASE + 0x304u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_active2, HWREG(NVIC_BASE + 0x308u));
    eepromWriteWithVerification_ui32_ISR(eepromAdd_unexpectedInt_active3, HWREG(NVIC_BASE + 0x30Cu));

    SysCtlReset();
}

void ISR_WDT(void);

static uint32_t wdtPeriod_clockTicks;
extern uint64_t debCnt(void);
extern int32_t debTickToMs(uint64_t numOfTicks);
#define TASK_STATUS_ARRAY_LENGTH    (50)
void ISR_WDT(void)
{
    if(WatchdogIntStatus(WATCHDOG0_BASE, true)){ //if the interrupt is caused by WDT0
        static bool firstCall = true;
        static uint64_t timeOfFirstCall;

        if(firstCall){
            firstCall = false;
            timeOfFirstCall = debCnt();

            WatchdogIntClear(WATCHDOG0_BASE);

            size_t numOfTasks;
            static TaskHandle_t tasks[50];
            bool OK = getTaskList(&tasks, &numOfTasks, sizeof(tasks)/sizeof(tasks[0]));

            if(OK){
                size_t i;
                for(i = 0; i < numOfTasks; i++){ //zero all run time counter
                    taskSetRunTimeCount(tasks[i], 0u);
                }
            }
            else{
                goto secondCall;
            }
        }
        else{
            uint32_t totTime;
secondCall:
            totTime = (uint32_t)(debCnt() - timeOfFirstCall);
            eepromWriteWithVerification_ui32_ISR(eepromAdd_wdt_totTime, debTickToMs(totTime));

            eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_wdt0);

            uint32_t PC = *(((volatile uint32_t*)__get_PSP()) + 6); //gets the program counter where execution would resume after the ISR
            eepromWriteWithVerification_ui32_ISR(eepromAdd_wdt_PC, PC);
            TaskHandle_t ct = xTaskGetCurrentTaskHandle();
            char* name = pcTaskGetName(ct);
            size_t len = strlen(name);
            eepromWriteWithVerification_ui16_ISR(eepromAdd_wdt_nameLen, (uint16_t)len);
            uint32_t nxtAdd;
            eepromWriteWithVerification_ui8A_ISR(eepromAdd_wdt_name, name, len, &nxtAdd);

            size_t numOfTasks;
            static TaskHandle_t tasks[50];
            bool OK = getTaskList(&tasks, &numOfTasks, sizeof(tasks)/sizeof(tasks[0]));

            if(OK){
                eepromWriteWithVerification_ui16_ISR(nxtAdd, 1u); //the task list is valid
                eepromWriteWithVerification_ui16_ISR(nxtAdd + 4, (uint16_t)numOfTasks); //the number of tasks
                nxtAdd += 8;

                size_t i;
                for(i = 0; i < numOfTasks; i++){
                    char* name = pcTaskGetName(tasks[i]);
                    size_t len = strlen(name);
                    eepromWriteWithVerification_ui16_ISR(nxtAdd, (uint16_t)len); //length of the name
                    nxtAdd += 4;
                    eepromWriteWithVerification_ui8A_ISR(nxtAdd, name, len, &nxtAdd); //name

                    uint32_t* SP = (uint32_t*)*((uint32_t*)(&(*(tasks[i])))); //gets the tasks stack pointer
                    uint32_t PC = SP[9 + (SP[8] & 0x10 ? 0 : 16) + 6]; //gets the program counter (PC) of the particular task (it is a port specific code)
                    eepromWriteWithVerification_ui32_ISR(nxtAdd, PC); //the PC
                    nxtAdd += 8;

                    eTaskState state = eTaskGetState_ISR(tasks[i]); //gets the tasks state
                    eepromWriteWithVerification_ui16_ISR(nxtAdd, (uint16_t)state); //the PC
                    nxtAdd += 4;

                    uint32_t taskTotTime = (uint32_t)taskGetRunTimeCount(tasks[i]); //gets the tasks state
                    uint32_t taskRelTime = (totTime != 0) ? ((taskTotTime * 100 + (totTime / 2)) / totTime) : 0; //gets the percentage time the task run
                    eepromWriteWithVerification_ui16_ISR(nxtAdd, (uint16_t)taskRelTime);
                    nxtAdd += 4;

                    uint32_t basePriority;
                    uint32_t currentPriority;
                    taskGetPriorities(tasks[i], &basePriority, &currentPriority);
                    eepromWriteWithVerification_ui32_ISR(nxtAdd, basePriority);
                    nxtAdd += 8;
                    eepromWriteWithVerification_ui32_ISR(nxtAdd, currentPriority);
                    nxtAdd += 8;

                    uint32_t stackRem = uxTaskGetStackHighWaterMark(tasks[i]);
                    eepromWriteWithVerification_ui32_ISR(nxtAdd, stackRem);
                    nxtAdd += 8;
                }
            }
            else{
                eepromWriteWithVerification_ui16_ISR(nxtAdd, 0u); //the task list is invalid
            }

            SysCtlReset();
        }
    }

    if(SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG1) && WatchdogIntStatus(WATCHDOG1_BASE, true)){ //if the interrupt is caused by WDT1
        eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_wdt1);
        SysCtlReset();
    }
}

void ISR_WDT_NmiSR(void)
{
    uint32_t source = SysCtlNMIStatus();
    SysCtlNMIClear(source);

    if(source & SYSCTL_NMI_MOSCFAIL){ //the main oscillator is not present or did not start
        eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_moscFailNmi);
        SysCtlReset();
    }

    if(source & SYSCTL_NMI_TAMPER){ //a tamper event has been detected
        eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_tamperNmi);
        SysCtlReset();
    }

    if(source & (SYSCTL_NMI_WDT0 | SYSCTL_NMI_WDT1)){ //watchdog 0 or 1 generated a timeout
        ISR_WDT();
    }

    if(source & SYSCTL_NMI_POWER){ //a power event occurred
        eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_powerNmi);
        SysCtlReset();
    }

    if(source & SYSCTL_NMI_EXTERNAL){ //an external NMI pin asserted
        eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_extNmi);
        SysCtlReset();
    }
}

static uint32_t wdtPeriod_clockTicks;
static void wdtReset_timerService(TimerHandle_t xTimer);
static void wdtReset_task(void* pvParameters);
static TaskHandle_t wdtResetTaskHandle;
static void wdtReset(void);
static void wdtInit_task(void* pvParameters);

void wdtInit(void)
{
    wdtPeriod_clockTicks = (wdtPeriod_ms) * (SystemCoreClock / 1000.f); //calculates the WDT period in clock cycles

    SysCtlResetBehaviorSet(SYSCTL_ONRST_WDOG0_POR | SYSCTL_ONRST_BOR_POR | SYSCTL_ONRST_EXT_POR);//all three (WDT, BOR, and POR) causes a system reset (if the reset is enabled)
    SysCtlVoltageEventConfig(SYSCTL_VEVENT_VDDABO_RST | SYSCTL_VEVENT_VDDBO_RST); //Brown-Out on both, Vdda and Vdd causes a reset

    //
    // Enable the Watchdog 0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    //
    // Wait for the Watchdog 0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0));

    //
    // Check to see if the registers are locked, and if so, unlock them.
    //
    if(WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    //
    // Initialize the watchdog timer.
    //
    WatchdogReloadSet(WATCHDOG0_BASE, wdtPeriod_clockTicks); //set the WDT period

    //
    // Enable the reset.
    //
    WatchdogResetEnable(WATCHDOG0_BASE);

    WatchdogStallEnable(WATCHDOG0_BASE); //enables the debugger to stall the WDT

    //enables the WDT interrupt
    WatchdogIntEnable(WATCHDOG0_BASE);
    IntEnable(INT_WATCHDOG);

    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_NMI); //makes the WDT interrupt non maskable

    //
    // Enable the watchdog timer.
    //
    WatchdogEnable(WATCHDOG0_BASE);

    WatchdogLock(WATCHDOG0_BASE); //locks the watchdog registers



    xTaskCreate(&wdtReset_task,
                "wdtResetTask",
                512,
                NULL,
                0,
                &wdtResetTaskHandle);

    TickType_t wdtResetTimerPeriod_ticks = pdMS_TO_TICKS(wdtPeriod_ms / 2);

    while(wdtResetTimerPeriod_ticks < 10); //halt if WDT needs reseting too often (for ease of debug)

    TimerHandle_t wdtResetTimerHandle = xTimerCreate("wdtResetTimer",
                                                     wdtResetTimerPeriod_ticks,
                                                     pdTRUE,
                                                     (void*) 0,
                                                     &wdtReset_timerService);

    xTimerStart(wdtResetTimerHandle, 0); //starts the software timer

    TaskHandle_t wdtInitTaskHandle;
    xTaskCreate(&wdtInit_task,
                "wdtInitTask",
                512,
                NULL,
                4,
                &wdtInitTaskHandle);
}

static void faultCauseHandler(uint16_t cause);
static void wdtInit_task(void* pvParameters)
{
    uint32_t resetCause = SysCtlResetCauseGet();
    static char msg[100] = {'\0'};


    {//save the reset's cause
        bool crcOK;

        uint32_t prvCause;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_reset_cause, &prvCause);
        if(crcOK){
            eepromWriteWithVerification_ui16_ISR(eepromAdd_prv_reset_cause_valid, 1u); //OK
            eepromWriteWithVerification_ui32_ISR(eepromAdd_prv_reset_cause, prvCause);
        }
        else{
            eepromWriteWithVerification_ui16_ISR(eepromAdd_prv_reset_cause_valid, 2u); //CRC error
            eepromWriteWithVerification_ui32_ISR(eepromAdd_prv_reset_cause, prvCause);
        }

        eepromWriteWithVerification_ui32_ISR(eepromAdd_reset_cause, resetCause);
    }


    if(resetCause & SYSCTL_CAUSE_POR){
            SysCtlResetCauseClear(SYSCTL_CAUSE_POR);
            uartPrint(usb, "\r\n\r\n  >> Power On Reset\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_EXT){
        SysCtlResetCauseClear(SYSCTL_CAUSE_EXT);
        uartPrint(usb, "\r\n\r\n>>>>>>>>>>>>>>>>>>>>>>>>>> EXT RESET <<<<<<<<<<<<<<<<<<<<<<<<<\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_HIB){
        SysCtlResetCauseClear(SYSCTL_CAUSE_HIB);
        uartPrint(usb, "\r\n\r\n!!!!!!!!!!!!!!!!!!!!!!!!!! HIB RESET !!!!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_HSRVREQ){
        SysCtlResetCauseClear(SYSCTL_CAUSE_HSRVREQ);
        uartPrint(usb, "\r\n\r\n?????????????????????????? HSRV RESET ?????????????????????????\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_BOR){
        SysCtlResetCauseClear(SYSCTL_CAUSE_BOR);
        uartPrint(usb, "\r\n\r\n************************** BOR RESET [");

        uint32_t voltageEvents;
        voltageEvents = SysCtlVoltageEventStatus(); // Read the current voltage event status.
        SysCtlVoltageEventClear(voltageEvents); // Clear all the current voltage events.

        bool addComma = false;

        if(voltageEvents & SYSCTL_VESTAT_VDDBOR){
            uartPrint(usb, "Vdd");

            addComma = true;
        }
        if(voltageEvents & SYSCTL_VESTAT_VDDABOR){
            if(addComma){
                uartPrint(usb, ", Vdda");
            }
            else{
                uartPrint(usb, "Vdda");
            }

            addComma = true;
        }


        uartPrint(usb, "] *************************\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_WDOG1){
            SysCtlResetCauseClear(SYSCTL_CAUSE_WDOG1);
            uartPrint(usb, "\r\n\r\n|||||||||||||||||||||||||| WDT1 RESET |||||||||||||||||||||||||\r\n\r\n");
    }

    if(resetCause & SYSCTL_CAUSE_WDOG0){
        SysCtlResetCauseClear(SYSCTL_CAUSE_WDOG0);
        uartPrint(usb, "\r\n\r\n########################## WDT RESET #########################\r\n\r\n");

        bool crcOK;

        uint16_t cause;
        crcOK = eepromGetVerifiedValue_ui16_ISR(eepromAdd_cause, &cause);

        if(!crcOK || cause != faultCause_wdt0){ //cause is damaged or inconsistent with the cause of the reset
            uartPrint(usb, "Fault Cause: ");
            msg[0] = '\0';
            appendInt64(msg, cause, 99, false);
            uartPrint(usb, msg);
            if(!crcOK){
                uartPrint(usb, " (bad CRC)");
            }
            if(cause != faultCause_wdt0){
                uartPrint(usb, " (inconsistent)");
            }
            uartPrintLn(usb, "");
        }
        else
        {
            uartPrintLn(usb, "Fault Cause: WDT0");
        }

        faultCauseHandler(cause);
    }

    if(resetCause & SYSCTL_CAUSE_SW){
        SysCtlResetCauseClear(SYSCTL_CAUSE_SW);
        uartPrint(usb, "\r\n\r\n-------------------------- SW RESET -------------------------\r\n\r\n");

        bool crcOK;

        uint16_t cause;
        crcOK = eepromGetVerifiedValue_ui16_ISR(eepromAdd_cause, &cause);

        if(!crcOK){ //cause is damaged or inconsistent with the cause of the reset
            uartPrint(usb, "Fault Cause: ");
            msg[0] = '\0';
            appendInt64(msg, cause, 99, false);
            uartPrint(usb, msg);
            uartPrintLn(usb, " (bad CRC)");
        }
        else
        {
            if(faultCause_wdt0 == cause){
                uartPrintLn(usb, "Fault Cause: WDT0");
            }
            else if(faultCause_wdt1 == cause){
                uartPrintLn(usb, "Fault Cause: WDT1");
            }
            else if(faultCause_moscFailNmi == cause){
                uartPrintLn(usb, "Fault Cause: MOSC Fail NMI");
            }
            else if(faultCause_tamperNmi == cause){
                uartPrintLn(usb, "Fault Cause: tamper NMI");
            }
            else if(faultCause_powerNmi == cause){
                uartPrintLn(usb, "Fault Cause: power NMI");
            }
            else if(faultCause_extNmi == cause){
                uartPrintLn(usb, "Fault Cause: external NMI");
            }
            else if(faultCause_unexpectedInterrupt == cause){
                uartPrintLn(usb, "Fault Cause: unexpected interrupt");
            }
            else if(faultCause_hardFault == cause){
                uartPrintLn(usb, "Fault Cause: hard fault");
            }
            else if(faultCause_mpuFault == cause){
                uartPrintLn(usb, "Fault Cause: MPU fault");
            }
            else if(faultCause_busFault == cause){
                uartPrintLn(usb, "Fault Cause: bus fault");
            }
            else if(faultCause_usageFault == cause){
                uartPrintLn(usb, "Fault Cause: usage fault");
            }
            else{
                uartPrintLn(usb, "Unexpected cause");
            }
        }

        faultCauseHandler(cause);
    }

    eepromWriteWithVerification_ui16_ISR(eepromAdd_cause, faultCause_none);

    vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
}

static void wdtReset_timerService(TimerHandle_t xTimer)
{
    xTaskNotifyGive(wdtResetTaskHandle); //notify the reset task that the WDT needs reseting
}

static void wdtReset_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //waits for the reset timer to signal

        wdtReset(); //resets the WDT
    }
}

static void wdtReset(void)
{
    if(WatchdogLockState(WATCHDOG0_BASE) == true)
    {
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    WatchdogReloadSet(WATCHDOG0_BASE, wdtPeriod_clockTicks); //resets the WDT timer

    WatchdogLock(WATCHDOG0_BASE); //locks the watchdog registers
}

static void faultCauseHandler(uint16_t cause)
{
    static char msg[100] = {'\0'};

    switch(cause){
    case faultCause_hardFault:{
        bool crcOK;

        uint32_t hardFaultStatus;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_hardFault_hardFaultStatus, &hardFaultStatus);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)hardFaultStatus, 99, false);
        uartPrint(usb, "Hard Fault Status: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t configurableFaultStatus;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_hardFault_configurableFaultStatus, &configurableFaultStatus);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)configurableFaultStatus, 99, false);
        uartPrint(usb, "Configurable Fault Status: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t busFaultAddress;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_hardFault_busFaultAddress, &busFaultAddress);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)busFaultAddress, 99, false);
        uartPrint(usb, "Bus Fault Address: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t memoryManagementFaultAddress;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_hardFault_memoryManagementFaultAddress, &memoryManagementFaultAddress);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)memoryManagementFaultAddress, 99, false);
        uartPrint(usb, "Memory Management Fault Address: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t PC;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_hardFault_PC, &PC);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)PC, 99, false);
        uartPrint(usb, "Program Counter: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }
    }
    break;

    case faultCause_mpuFault:{
        bool crcOK;

        uint32_t configurableFaultStatus;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_mpuFault_configurableFaultStatus, &configurableFaultStatus);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)configurableFaultStatus, 99, false);
        uartPrint(usb, "Configurable Fault Status: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t memoryManagementFaultAddress;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_mpuFault_memoryManagementFaultAddress, &memoryManagementFaultAddress);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)memoryManagementFaultAddress, 99, false);
        uartPrint(usb, "Memory Management Fault Address: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t PC;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_mpuFault_PC, &PC);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)PC, 99, false);
        uartPrint(usb, "Program Counter: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }
    }
    break;

    case faultCause_busFault:{
        bool crcOK;

        uint32_t configurableFaultStatus;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_busFault_configurableFaultStatus, &configurableFaultStatus);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)configurableFaultStatus, 99, false);
        uartPrint(usb, "Configurable Fault Status: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t busFaultAddress;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_busFault_busFaultAddress, &busFaultAddress);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)busFaultAddress, 99, false);
        uartPrint(usb, "Bus Fault Address: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t PC;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_busFault_PC, &PC);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)PC, 99, false);
        uartPrint(usb, "Program Counter: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }
    }
    break;

    case faultCause_usageFault:{
        bool crcOK;

        uint32_t configurableFaultStatus;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_usageFault_configurableFaultStatus, &configurableFaultStatus);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)configurableFaultStatus, 99, false);
        uartPrint(usb, "Configurable Fault Status: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint32_t PC;
        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_usageFault_PC, &PC);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)PC, 99, false);
        uartPrint(usb, "Program Counter: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }
    }
    break;

    case faultCause_unexpectedInterrupt:{
        uint32_t dat;
        bool crcOK;

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_pend0, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "PEND0: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_pend1, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "PEND1: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_pend2, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "PEND2: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_pend3, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "PEND3: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_active0, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "ACTIVE0: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_active1, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "ACTIVE1: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_active2, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "ACTIVE2: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_unexpectedInt_active3, &dat);
        msg[0] = '\0';
        appendInt64(msg, (int64_t)dat, 99, false);
        uartPrint(usb, "ACTIVE3: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }
    }
    break;

    case faultCause_wdt0:
    {
        uint32_t PC;
        bool crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_wdt_PC, &PC);
        msg[0] = '\0';
        appendInt64(msg, PC, 99, false);
        uartPrint(usb, "PC: ");
        uartPrint(usb, msg);
        if(crcOK){
            uartPrintLn(usb, "");
        }
        else{
            uartPrintLn(usb, " (bad CRC)");
        }

        uint16_t nameLen;
        crcOK = eepromGetVerifiedValue_ui16_ISR(eepromAdd_wdt_nameLen, &nameLen);

        uint32_t nxtAdd;

        static char name[100];
        name[nameLen] = '\0';
        crcOK = eepromGetVerifiedValue_ui8A_ISR(eepromAdd_wdt_name, &name[0], nameLen, &nxtAdd);

        uartPrint(usb, "Current Task: ");
        uartPrintLn(usb, name);

        uint16_t listOK;
        crcOK = eepromGetVerifiedValue_ui16_ISR(nxtAdd, &listOK);
        nxtAdd += 4;

        if(crcOK){
            if(1u == listOK){
                uartPrintLn(usb, "");
                uartPrint(usb, "List of Tasks (measured over ");
                uint32_t totTimeMs;
                crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_wdt_totTime, &totTimeMs);
                if(crcOK){
                    msg[0] = '\0';
                    appendInt64(msg, totTimeMs, 99, false);
                    uartPrint(usb, msg);
                }
                else{
                    uartPrint(usb, "ERROR: bad CRC");
                }
                uartPrintLn(usb, " ms)\r\n------------");
                uint16_t numOfTasks;
                crcOK = eepromGetVerifiedValue_ui16_ISR(nxtAdd, &numOfTasks);
                nxtAdd += 4;

                if(crcOK){
                    size_t i;
                    for(i = 0; i < numOfTasks; i++){
                        uint16_t nameLen;
                        crcOK = eepromGetVerifiedValue_ui16_ISR(nxtAdd, &nameLen);
                        nxtAdd += 4;

                        if(crcOK){
                            name[nameLen] = '\0';

                            crcOK = eepromGetVerifiedValue_ui8A_ISR(nxtAdd, &name[0], nameLen, &nxtAdd);
                            if(crcOK){
                                uartPrint(usb, name);
                            }
                            else
                            {
                                uartPrint(usb, "ERROR: task name bad CRC");
                            }
                        }
                        else
                        {
                            uartPrintLn(usb, "ERROR: task name length value bad CRC");
                            break;
                        }

                        uartPrint(usb, "\t");

                        uint32_t PC;
                        crcOK = eepromGetVerifiedValue_ui32_ISR(nxtAdd, &PC);
                        nxtAdd += 8;

                        msg[0] = '\0';
                        appendInt64(msg, PC, 99, false);
                        uartPrint(usb, "PC: ");
                        uartPrint(usb, msg);

                        uartPrint(usb, "\t");

                        uint16_t state;
                        crcOK = eepromGetVerifiedValue_ui16_ISR(nxtAdd, &state);
                        nxtAdd += 4;

                        switch((eTaskState)state){
                        case eRunning:
                            uartPrint(usb, "state: running");
                            break;
                        case eReady:
                            uartPrint(usb, "state: ready");
                            break;
                        case eBlocked:
                            uartPrint(usb, "state: blocked");
                            break;
                        case eSuspended:
                            uartPrint(usb, "state: suspended");
                            break;
                        case eDeleted:
                            uartPrint(usb, "state: deleted");
                            break;
                        case eInvalid:
                            uartPrint(usb, "state: invalid");
                            break;
                        default:
                            uartPrint(usb, "state: ERROR");
                            break;
                        }

                        uartPrint(usb, "\t");

                        uint16_t relTime;
                        crcOK = eepromGetVerifiedValue_ui16_ISR(nxtAdd, &relTime);
                        nxtAdd += 4;

                        if(crcOK){
                            uartPrint(usb, "CPU usage: ");
                            msg[0] = '\0';
                            appendInt64(msg, relTime, 99, false);
                            uartPrint(usb, msg);
                            uartPrint(usb, " %");
                        }
                        else{
                            uartPrint(usb, "CPU usage: bad CRC");
                        }

                        uartPrint(usb, "\t");

                        uint32_t basePriority;
                        crcOK = eepromGetVerifiedValue_ui32_ISR(nxtAdd, &basePriority);
                        nxtAdd += 8;

                        if(crcOK){
                            uartPrint(usb, "base priority: ");
                            msg[0] = '\0';
                            appendInt64(msg, basePriority, 99, false);
                            uartPrint(usb, msg);
                        }
                        else{
                            uartPrint(usb, "base priority: bad CRC");
                        }

                        uartPrint(usb, "\t");

                        uint32_t currentPriority;
                        crcOK = eepromGetVerifiedValue_ui32_ISR(nxtAdd, &currentPriority);
                        nxtAdd += 8;

                        if(crcOK){
                            uartPrint(usb, "current priority: ");
                            msg[0] = '\0';
                            appendInt64(msg, currentPriority, 99, false);
                            uartPrint(usb, msg);
                        }
                        else{
                            uartPrint(usb, "current priority: bad CRC");
                        }

                        uartPrint(usb, "\t");

                        uint32_t stackRem;
                        crcOK = eepromGetVerifiedValue_ui32_ISR(nxtAdd, &stackRem);
                        nxtAdd += 8;

                        if(crcOK){
                            uartPrint(usb, "words unused on stack: ");
                            msg[0] = '\0';
                            appendInt64(msg, stackRem, 99, false);
                            uartPrint(usb, msg);
                        }
                        else{
                            uartPrint(usb, "words unused on stack: bad CRC");
                        }

                        uartPrintLn(usb, "");
                    }
                }
                else{
                    uartPrintLn(usb, "ERROR: number of task value bad CRC");
                }


            }
            else
            {
                uartPrintLn(usb, "ERROR: list not valid");
            }
        }
        else
        {
            uartPrintLn(usb, "ERROR: list valid value bad CRC");
        }
    }
        break;

    case faultCause_wdt1:
        break;

    case faultCause_moscFailNmi:
        break;

    case faultCause_tamperNmi:
        break;

    case faultCause_powerNmi:
        break;

    case faultCause_extNmi:
        break;

    default:
        uartPrintLn(usb, "ERROR: unknown cause");
        break;
    }

    uartPrintLn(usb, "");
}
