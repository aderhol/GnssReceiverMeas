//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>

//driver includes
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"    //interrupt defines (eg. INT_UART0)
#include "inc/hw_timer.h" //timer register offsets
#include "inc/hw_types.h" //HWREG() macro
#include "driverlib/timer.h"    //timer
#include "driverlib/interrupt.h"    //interrupt API
#include "driverlib/pin_map.h"  //pin names
#include "driverlib/gpio.h" //GPIO
#include "driverlib/sysctl.h" //System Control

//user includes
#include <skewMeas.h>
#include <uartIO.h>
#include <nmea.h>
#include <utility.h>
#include <sensorHub.h>



//configurations
//overflow and the PPS signals are in the same priority group but the overflow has a higher sub priority
static const uint8_t OVERFLOW_INTERRUPT_PRIORITY = 2;
static const uint8_t PPS_INTERRUPT_PRIORITY = 3;
static const int16_t D_REF_DUT_MS = 500; //the maximum skew in ms between the ref and dut PPS for them to be deemed a pair
static const int16_t D_PPS_MESSAGE_MS = 500; //the maximum delay in ms between the dut PPS and the start off the NMEA packet for them to be deemed a pair

static int64_t dRefDut_clk; //the maximum skew in clock ticks under witch the ref and dut can be deemed a pair


typedef enum{
    available,
    captured,
    waiting,
    glitch,
    missingRef
}Status;

static const EventBits_t ppsReadyFlag = ((EventBits_t) 1) << 0;

typedef struct{
    int64_t time;
    int64_t skew;
    Status status;
    TickType_t sysTime;
    UartPort port;
    EventGroupHandle_t flags;
}Dut;

static volatile Dut dutA, dutB, refCheck;
static volatile int64_t refCheck_refAbsTime_ticks;

static volatile struct{
    int64_t time;
    bool availableForDutA;
    bool availableForDutB;
    bool availableForRefCheck;
}ref;

void ISR_TIMER0_A(void);
void ISR_TIMER2_A(void);
void ISR_TIMER2_B(void);
void ISR_TIMER3_A(void);
void ISR_TIMER3_B(void);

static TaskHandle_t errTaskHandle;
static void err_task(void* pvParameters);

static TimerHandle_t dutAtimeoutTimerHandle;
static void dutAtimeout_timerService(TimerHandle_t xTimer);

static TimerHandle_t dutBtimeoutTimerHandle;
static void dutBtimeout_timerService(TimerHandle_t xTimer);

static TimerHandle_t refCheckTimeoutTimerHandle;
static void refCheckTimeout_timerService(TimerHandle_t xTimer);

static bool checkRef(int64_t* skew, const Dut* dut);

static uint32_t getTimerSnapshot(uint32_t base, uint32_t timer);

static float calcTimeDiff_ms(TickType_t A, TickType_t B);

static volatile int64_t overflowCount;

static void dutARx_callback(char* str);
static void dutBRx_callback(char* str);

static TaskHandle_t refCheckTaskHandle;
static void refCheck_task(void* pvParameters);

//overflow
void ISR_TIMER0_A(void)
{
    TimerIntClear(TIMER0_BASE, (uint32_t)(~((uint32_t)0))); //clear the interrupt

    uint32_t capTimer2IntStatus = TimerIntStatus(TIMER2_BASE, true); //get capture timer interrupt status
    uint32_t capTimer3IntStatus = TimerIntStatus(TIMER3_BASE, true); //get capture timer interrupt status

    if(capTimer2IntStatus & TIMER_CAPA_EVENT){ //if there is a pending ref PPS interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER2_BASE, TIMER_A); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the ref PPS came before the overflow
            ISR_TIMER2_A(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER2A); //clear the pending interrupt in the NVIC
        }
    }

    if(capTimer2IntStatus & TIMER_CAPB_EVENT){ //if there is a pending dutA PPS interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER2_BASE, TIMER_B); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the DUT PPS came before the overflow
            ISR_TIMER2_B(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER2B); //clear the pending interrupt in the NVIC
        }
    }

    if(capTimer3IntStatus & TIMER_CAPA_EVENT){ //if there is a pending dutB PPS interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER3_BASE, TIMER_A); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the DUT PPS came before the overflow
            ISR_TIMER3_A(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER3A); //clear the pending interrupt in the NVIC
        }
    }

    if(capTimer3IntStatus & TIMER_CAPB_EVENT){ //if there is a pending ref check interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER3_BASE, TIMER_B); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the DUT PPS came before the overflow
            ISR_TIMER3_B(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER3B); //clear the pending interrupt in the NVIC
        }
    }

    overflowCount++;
}

//ref PPS
void ISR_TIMER2_A(void)
{
    uint32_t callers = TimerIntStatus(TIMER2_BASE, true) & (~TIMER_CAPB_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER2_BASE, callers);    //clears the interrupt flags

    ref.availableForDutA = true; //the ref is now available
    ref.availableForDutB = true; //the ref is now available
    ref.availableForRefCheck = true; //the ref is now available
    ref.time = ((int64_t)getTimerSnapshot(TIMER2_BASE, TIMER_A)) + ((int64_t)(overflowCount<<24)); //saving the time of capture


    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if(dutA.status == captured){ //if the dutA PPS is waiting for a ref PPS
        int64_t skew;
        if(checkRef(&skew, &dutA)){ //if the new ref can be paired to the pending dutA PPS
            dutA.skew = skew; //update the skew of the dut
            dutA.status = available; //the dut is now available

            ref.availableForDutA = false; //the ref has been used up / paired
        }
        else{ //if the new ref can't be paired to the captured PPS that means a ref PPS is missing
            dutA.status = missingRef;
        }

        xEventGroupSetBitsFromISR(dutA.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(dutAtimeoutTimerHandle,
                           &higherPriorityTaskWoken);
    }

    if(dutB.status == captured){ //if the dutB PPS is waiting for a ref PPS
        int64_t skew;
        if(checkRef(&skew, &dutB)){ //if the new ref can be paired to the pending dutB PPS
            dutB.skew = skew; //update the skew of the dut
            dutB.status = available; //the dut is now available

            ref.availableForDutB = false; //the ref has been used up / paired
        }
        else{ //if the new ref can't be paired to the captured PPS that means a ref PPS is missing
            dutB.status = missingRef;
        }

        xEventGroupSetBitsFromISR(dutB.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(dutBtimeoutTimerHandle,
                           &higherPriorityTaskWoken);
    }

    if(refCheck.status == captured){ //if the ref check is waiting for a ref PPS
        int64_t skew;
        if(checkRef(&skew, &refCheck)){ //if the new ref can be paired to the pending ref check
            refCheck.skew = skew; //update the skew of the ref check
            refCheck_refAbsTime_ticks = ref.time; //save the refs absolute time
            refCheck.status = available; //the ref check is now available

            ref.availableForRefCheck = false; //the ref has been used up / paired
        }
        else{ //if the new ref can't be paired to the captured PPS that means a ref PPS is missing
            refCheck.status = missingRef;
        }

        xEventGroupSetBitsFromISR(refCheck.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(refCheckTimeoutTimerHandle,
                           &higherPriorityTaskWoken);
    }


    vTaskNotifyGiveFromISR(refCheckTaskHandle, &higherPriorityTaskWoken); //trigger the refCheck routine


    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

//DUT A PPS
void ISR_TIMER2_B(void)
{
    uint32_t callers = TimerIntStatus(TIMER2_BASE, true) & (~TIMER_CAPA_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER2_BASE, callers);    //clears the interrupt flags

    if(dutA.status == waiting){
        dutA.time = ((int64_t)getTimerSnapshot(TIMER2_BASE, TIMER_B)) + ((int64_t)(overflowCount<<24)); //save the capture time
        dutA.sysTime = xTaskGetTickCountFromISR(); //save the current system time

        if(ref.availableForDutA){ //if a ref PPS is available
            int64_t skew;
            if(checkRef(&skew, &dutA)){ //if the new ref can be paired to the new dut PPS
                dutA.skew = skew; //update the skew of the dut
                dutA.status = available; //the dut is now available
            }
            else{ //if the new dut PPS doesn't belong to the ref on record
                dutA.status = captured;

                //start the timeout timer
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(dutAtimeoutTimerHandle,
                                   &higherPriorityTaskWoken);

                //signal to task
                xTaskNotifyFromISR(errTaskHandle,
                                   1, //dutA
                                   eSetValueWithOverwrite,
                                   &higherPriorityTaskWoken);

                portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
            }
            ref.availableForDutA = false; //the ref has been used up: paired or discarded
        }
        else{ //if a ref PPS is not immediately available
            dutA.status = captured;

            //start the timeout timer
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR(dutAtimeoutTimerHandle,
                               &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
    else{ //if a new dut PPS came before the previous got handled
        dutA.status = glitch; //a glitch occurred

        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(dutA.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(dutAtimeoutTimerHandle,
                           &higherPriorityTaskWoken);

        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

//DUT B PPS
void ISR_TIMER3_A(void)
{
    uint32_t callers = TimerIntStatus(TIMER3_BASE, true) & (~TIMER_CAPB_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER3_BASE, callers);    //clears the interrupt flags

    if(dutB.status == waiting){
        dutB.time = ((int64_t)getTimerSnapshot(TIMER3_BASE, TIMER_A)) + ((int64_t)(overflowCount<<24)); //save the capture time
        dutB.sysTime = xTaskGetTickCountFromISR(); //save the current system time

        if(ref.availableForDutB){ //if a ref PPS is available
            int64_t skew;
            if(checkRef(&skew, &dutB)){ //if the new ref can be paired to the new dut PPS
                dutB.skew = skew; //update the skew of the dut
                dutB.status = available; //the dut is now available
            }
            else{ //if the new dut PPS doesn't belong to the ref on record
                dutB.status = captured;

                //start the timeout timer
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(dutBtimeoutTimerHandle,
                                   &higherPriorityTaskWoken);

                //signal to task
                xTaskNotifyFromISR(errTaskHandle,
                                   2, //dutB
                                   eSetValueWithOverwrite,
                                   &higherPriorityTaskWoken);

                portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
            }
            ref.availableForDutB = false; //the ref has been used up: paired or discarded
        }
        else{ //if a ref PPS is not immediately available
            dutB.status = captured;

            //start the timeout timer
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR(dutBtimeoutTimerHandle,
                               &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
    else{ //if a new dut PPS came before the previous got handled
        dutB.status = glitch; //a glitch occurred

        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(dutB.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(dutBtimeoutTimerHandle,
                           &higherPriorityTaskWoken);

        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

//ref check
void ISR_TIMER3_B(void)
{
    uint32_t callers = TimerIntStatus(TIMER3_BASE, true) & (~TIMER_CAPA_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER3_BASE, callers);    //clears the interrupt flags

    if(refCheck.status == waiting){
        refCheck.time = ((int64_t)getTimerSnapshot(TIMER3_BASE, TIMER_B)) + ((int64_t)(overflowCount<<24)); //save the capture time
        refCheck.sysTime = xTaskGetTickCountFromISR(); //save the current system time

        if(ref.availableForRefCheck){ //if a ref PPS is available
            int64_t skew;
            if(checkRef(&skew, &refCheck)){ //if the new ref can be paired to the new ref check
                refCheck.skew = skew; //update the skew of the ref check
                refCheck_refAbsTime_ticks = ref.time; //save the refs absolute time
                refCheck.status = available; //the ref check is now available
            }
            else{ //if the new ref check doesn't belong to the ref on record
                refCheck.status = captured;

                //start the timeout timer
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(refCheckTimeoutTimerHandle,
                                   &higherPriorityTaskWoken);

                //signal to task
                xTaskNotifyFromISR(errTaskHandle,
                                   3, //refCheck
                                   eSetValueWithOverwrite,
                                   &higherPriorityTaskWoken);

                portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
            }
            ref.availableForRefCheck = false; //the ref has been used up: paired or discarded
        }
        else{ //if a ref PPS is not immediately available
            refCheck.status = captured;

            //start the timeout timer
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR(refCheckTimeoutTimerHandle,
                               &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
    else{ //if a new ref check came before the previous got handled
        refCheck.status = glitch; //a glitch occurred

        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(refCheck.flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(refCheckTimeoutTimerHandle,
                           &higherPriorityTaskWoken);

        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

static void dutAtimeout_timerService(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
    {
        if(dutA.status == captured){
            dutA.status = missingRef;
        }
    }
    taskEXIT_CRITICAL();

    xEventGroupSetBits(dutA.flags, ppsReadyFlag);
}

static void dutBtimeout_timerService(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
    {
        if(dutB.status == captured){
            dutB.status = missingRef;
        }
    }
    taskEXIT_CRITICAL();

    xEventGroupSetBits(dutB.flags, ppsReadyFlag);
}

static void refCheckTimeout_timerService(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
    {
        if(refCheck.status == captured){
            refCheck.status = missingRef;
        }
    }
    taskEXIT_CRITICAL();

    xEventGroupSetBits(refCheck.flags, ppsReadyFlag);
}

static bool checkRef(int64_t* skew, const Dut* dut)
{
    int64_t d;

    d = dut->time - ref.time; //calculate the skew

    //save the skew to the output variable if the input pointer is valid
    if(skew != NULL){
        *skew = d;
    }

    d = (d < 0) ? (-d) : d; //take the absolute value

    return (d < dRefDut_clk); //return true if the skew is within the acceptable range
}

extern uint32_t SystemCoreClock; //actual system core clock frequency in Hz (built-in to FreeRTOS)

void skewMeasInit(void)
{
    //initializes the global variables
    dutA.status = waiting;
    dutA.port = usb;
    dutB.status = waiting;
    dutB.port = dutB_toPC;
    refCheck.status = waiting;
    refCheck.port = usb;
    ref.availableForDutA = false;
    ref.availableForDutB = false;
    ref.availableForRefCheck = false;
    overflowCount = 0;
    dRefDut_clk = (((int64_t)SystemCoreClock) / ((int64_t)1000)) * D_REF_DUT_MS;
    //dPpsMessage_clk = (((int64_t)SystemCoreClock) / ((int64_t)1000)) * D_PPS_MESSAGE_MS;

    dutA.flags = xEventGroupCreate();
    xEventGroupClearBits(dutA.flags, ppsReadyFlag);

    dutB.flags = xEventGroupCreate();
    xEventGroupClearBits(dutB.flags, ppsReadyFlag);

    refCheck.flags = xEventGroupCreate();
    xEventGroupClearBits(refCheck.flags, ppsReadyFlag);

    //enable the IO port for the input capture legs
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){}
    }
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){}
    }

    //enables TIMER 0, which needs to be enabled in order to make GPTMSYNC accessible
    //TIMER0 is needed, because timers configured for input capture doesn't raise overflow interrupts
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}
    }

    //TIMER2 is for input capture
    //enables TIMER 2
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)){}
    }

    //TIMER3 is for input capture
    //enables TIMER 3
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)){}
    }
    //configures timers for two-signal edge-separation
    GPIOPinConfigure(GPIO_PM0_T2CCP0);   //sets PM0 as the input capture input for TIMER2 TIMER A
    GPIOPinConfigure(GPIO_PM1_T2CCP1);   //sets PM1 as the input capture input for TIMER2 TIMER B
    GPIOPinConfigure(GPIO_PM2_T3CCP0);   //sets PM2 as the input capture input for TIMER3 TIMER A
    GPIOPinConfigure(GPIO_PD5_T3CCP1);   //sets PD5 as the input capture input for TIMER3 TIMER B

    //configures the pins
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_5);
    /*GPIOPadConfigSet(GPIO_PORTD_BASE,
                     GPIO_PIN_5,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);*/

    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);  //TIMER0 is clocked from the system clock
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);  //TIMER2 is clocked from the system clock
    TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_SYSTEM);  //TIMER3 is clocked from the system clock

    TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP)); //configures TIMER0
    TimerConfigure(TIMER2_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP)); //configures TIMER2 TIMER A/B
    TimerConfigure(TIMER3_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP)); //configures TIMER3 TIMER A/B

    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);   //configures Timer2 A to capture rising edges
    TimerControlEvent(TIMER2_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);   //configures Timer2 B to capture rising edges
    TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);   //configures Timer3 A to capture rising edges
    TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);   //configures Timer3 A to capture rising edges

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //enables overflow interrupt on TIMER0 A
    TimerIntEnable(TIMER2_BASE, (TIMER_CAPB_EVENT | TIMER_CAPA_EVENT));   //enables interrupt for edge capture on TIMER2 TIMER A/B
    TimerIntEnable(TIMER3_BASE, (TIMER_CAPB_EVENT | TIMER_CAPA_EVENT));   //enables interrupt for edge capture on TIMER3 TIMER A/B

    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFF); //sets the max value of the counter
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xFF); //sets the max value of the prescaler
    TimerLoadSet(TIMER2_BASE, TIMER_BOTH, 0xFFFF);
    TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, 0xFF);
    TimerLoadSet(TIMER3_BASE, TIMER_BOTH, 0xFFFF);
    TimerPrescaleSet(TIMER3_BASE, TIMER_BOTH, 0xFF);

    TimerEnable(TIMER2_BASE, TIMER_BOTH);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_BOTH);

    TimerSynchronize(TIMER0_BASE, (TIMER_0A_SYNC | TIMER_2A_SYNC | TIMER_2B_SYNC | TIMER_3A_SYNC | TIMER_3B_SYNC));  //Synchronizes TIMER0 A and TIMER2 A and B and TIMER3 A and B with each other

    IntEnable(INT_TIMER0A);
    IntPrioritySet(INT_TIMER0A, OVERFLOW_INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //the priority bits need to be shifted up, to the implemented bits

    IntEnable(INT_TIMER2A);
    IntPrioritySet(INT_TIMER2A, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));

    IntEnable(INT_TIMER2B);
    IntPrioritySet(INT_TIMER2B, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));

    IntEnable(INT_TIMER3A);
    IntPrioritySet(INT_TIMER3A, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));

    IntEnable(INT_TIMER3B);
    IntPrioritySet(INT_TIMER3B, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));


    xTaskCreate(&err_task,
                "errTask",
                512,
                NULL,
                2,
                &errTaskHandle);

    dutAtimeoutTimerHandle = xTimerCreate("dutAtimeoutTimer",
                                          pdMS_TO_TICKS(D_REF_DUT_MS),
                                          pdFALSE,
                                          (void*) 0,
                                          &dutAtimeout_timerService);

    dutBtimeoutTimerHandle = xTimerCreate("dutBtimeoutTimer",
                                          pdMS_TO_TICKS(D_REF_DUT_MS),
                                          pdFALSE,
                                          (void*) 0,
                                          &dutBtimeout_timerService);

    refCheckTimeoutTimerHandle = xTimerCreate("refCheckTimeoutTimer",
                                              pdMS_TO_TICKS(D_REF_DUT_MS),
                                              pdFALSE,
                                              (void*) 0,
                                              &refCheckTimeout_timerService);

    /******    UART    ******/
    uartSetRxCallback(dutA_toDut, &dutARx_callback);
    uartSetRxCallback(dutB_toDut, &dutBRx_callback);


    xTaskCreate(&refCheck_task,
                "refCheckTask",
                512,
                NULL, //parameter to task
                3, //priority (same as for the dut Rx tasks)
                &refCheckTaskHandle);
}

static void err_task(void* pvParameters)
{
    while(true){
        uint32_t dutNum; //A:1 B:2 check:3
        xTaskNotifyWait(0,
                        UINT32_MAX,
                        &dutNum,
                        portMAX_DELAY);

        if(1 == dutNum){
            uartPrintLn(dutA.port,"$EXTRAREF,dutA*03");
        }
        else if(2 == dutNum){
            uartPrintLn(dutB.port,"$EXTRAREF,dutB*00");
        }
        else if(3 == dutNum){
            uartPrintLn(refCheck.port,"$EXTRAREF,refCheck*10");
        }
    }
}


typedef enum{
    skewOK,
    ppsGlitch,
    ppsOld,
    ppsMissing,
    refMissing
}SkewStatus;

static SkewStatus getSkew(float* D, float* skew_ns, int64_t* absTime_ticks, const Dut* dut);
static const char* printSkew(const Dut* dut, char* str); //with the new line at the end
static const char* printSenMeas(char* str);
static void streamProcessor(char* str, char buff[], size_t buffTotLength, const Dut* dut);

static void refCheck_task(void* pvParams)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //wait for a REF PPS to arrive

        vTaskDelay(pdMS_TO_TICKS(100)); //wait 100 ms, so that the ref check can surely arrive and to simulate the triggering by the NMEA stream


        float D, skew_ns;
        int64_t absTime_ticks, refAbsTime_ticks;
        SkewStatus skewStatus = getSkew(&D, &skew_ns, &absTime_ticks, &refCheck);
        taskENTER_CRITICAL();
        {
            refAbsTime_ticks = refCheck_refAbsTime_ticks;
        }
        taskEXIT_CRITICAL();

        static char str[100];

        strcpy(str, "$REFCHECK,");

        switch(skewStatus){
            case skewOK:
                if(fabsf(skew_ns) < 1.0e9f){ //if the skew is smaller than 1s
                    appendFloat(str, D, 2, 10, true);
                    strcat(str, "ms,");
                    appendFloat(str, skew_ns, 2, 20, true);
                    strcat(str, "ns,");
                    appendInt64(str, refAbsTime_ticks, 20, true);
                    strcat(str, "ticks,");
                    appendInt64(str, absTime_ticks, 20, true);
                    strcat(str, "ticks,OK");
                }
                else{ //this shouldn't be possible, due to how the program is written
                    appendFloat(str, D, 2, 10, true);
                    strcat(str, "ms,");
                    strcat(str, "inf,ns,");
                    appendInt64(str, refAbsTime_ticks, 20, true);
                    strcat(str, "ticks,");
                    appendInt64(str, absTime_ticks, 20, true);
                    strcat(str, "ticks,???");
                }
                break;

            case ppsGlitch:
                appendFloat(str, D, 2, 10, true);
                strcat(str, "ms,NaN,ns,,ticks,,ticks,GLITCH");
                break;

            case ppsOld:
                appendFloat(str, D, 2, 10, true);
                strcat(str, "ms,NaN,ns,");
                appendInt64(str, refAbsTime_ticks, 20, true);
                strcat(str, "ticks,");
                appendInt64(str, absTime_ticks, 20, true);
                strcat(str, "ticks,OLD");
                break;

            case ppsMissing:
                strcat(str, "NaN,ms,NaN,ns,");
                appendInt64(str, refAbsTime_ticks, 20, true);
                strcat(str, "ticks,,ticks,CHECK_MISSING");
                break;

            case refMissing:
                appendFloat(str, D, 2, 10, true);
                strcat(str, "ms,NaN,ns,,ticks,");
                appendInt64(str, absTime_ticks, 20, true);
                strcat(str, "ticks,REF_MISSING");
                break;
        }

        addChecksum(str, 99, true, false, true);

        uartPrint(refCheck.port, str);
    }
}

static const char pattern1[] = "$GNRMC";
static const size_t pattern1Length = (sizeof(pattern1) / sizeof(char)) - 1;
static const char pattern2[] = "$GPRMC";
static const size_t pattern2Length = (sizeof(pattern2) / sizeof(char)) - 1;

#define DUT_A_RX_BUFF_LENGTH   (300)
static void dutARx_callback(char* str)
{
    static char buff[DUT_A_RX_BUFF_LENGTH] = ""; //buffer for the character stream

    streamProcessor(str, buff, DUT_A_RX_BUFF_LENGTH, &dutA); //process the snippet, forward/print to usb
}

#define DUT_B_RX_BUFF_LENGTH   (300)
static void dutBRx_callback(char* str)
{
    static char buff[DUT_B_RX_BUFF_LENGTH] = ""; //buffer for the character stream

    streamProcessor(str, buff, DUT_B_RX_BUFF_LENGTH, &dutB); //process the snippet, forward/print to usb
}

static void streamProcessor(char* str, char buff[], size_t buffTotLength, const Dut* dut)
{
    bool runProcessing;

    if(str[0] != '\0'){ //if the snippet is not an empty string
        size_t strLen = strlen(str); //the length of the snippet (not including the terminating '\0')
        size_t buffLen = strlen(buff); //the length of the buffer

        if((buffLen + strLen + 1) > buffTotLength){ //if the buffer doesn't have enough space
            if((strLen + 1) > buffTotLength){ //if the snippet is just too long (longer than the entire buffer)
                uartPrint(dut->port, buff); //print the buffer
                buff[0] = '\0'; //clean the buffer

                uartPrint(dut->port, str); //print the snippet

                runProcessing = false;
            }
            else{ //if the buffer is large enough: the buffer can't possibly be empty in this case
                uartPrint(dut->port, buff); //print the buffer
                //buff[0] = '\0'; doesn't needed because of strcpy //clean the buffer

                strcpy(buff, str); //move the snippet into the buffer

                runProcessing = true;
            }
        }
        else{ //if the buffer has enough space
            strcat(buff, str); //copy the snippet into the buffer

            runProcessing = true;
        }

        /**** processing ****/
        if(runProcessing){
            const char* pos;
            const char* head = buff; //the head of the current message
            for(pos = buff + buffLen; (*pos) != '\0'; pos++){ //run until the whole buffer is processed, start with the first unprocessed character
                if((*pos) == '\n'){ //if the end of a message is detected
                    (*((char*)pos)) = '\0'; //terminate the message

                    uartPrintLf(dut->port, head); //forward the message

                    //check if this message is a block starter
                    bool trigger = (strncmp(head, pattern1, pattern1Length) == 0);
                    if(!trigger){
                        trigger = (strncmp(head, pattern2, pattern2Length) == 0);
                    }

                    if(trigger){ //if the start of an NMEA block was detected
                        char str[300];
                        uartPrint(dut->port, printSkew(dut, str)); //sends the skew
                        uartPrint(dut->port, printSenMeas(str)); //sends the skew
                    }

                    head = pos + 1;//move the head to the beginning of the next message
                }
            }

            memmove(buff, head, (strlen(head) + 1) * sizeof(char)); //shifts the buffer up
        }
    }
}

//str needs to be at least 300 long
static const char* printSenMeas(char* str)
{
    SensorData sensorData;
    bool OK;

    strcpy(str, "$SENDAT,"); //Initializing str

    OK = getSample(&sensorData); //get the sample

    if(OK){ //if the sample is valid
        float age = calcTimeDiff_ms(sensorData.sysTime, xTaskGetTickCount());
        OK = (fabsf(age) < 500); //is the sample fresh enough?

        appendFloat(str, sensorData.temperature_C_bmp180, 2, 10, true); //append bmp180 temperature
        strcat(str, "C,");
        appendFloat(str, sensorData.pressure_Pa, 2, 10, true); //append pressure
        strcat(str, "Pa,");

        appendFloat(str, sensorData.visibleLightIntensity_lx, 2, 10, true); //append visible light intensity
        strcat(str, "lx,");
        appendFloat(str, ((float)sensorData.infraredLighIntensity), 0, 10, true); //append infrared light intensity
        strcat(str, ",");

        appendFloat(str, sensorData.temperature_C_sht21, 2, 10, true); //append sht21 temperature
        strcat(str, "C,");
        appendFloat(str, sensorData.relativeHumidity_percentage, 2, 10, true); //append relative humidity
        strcat(str, "pct,");

        appendFloat(str, age, 0, 10, true); //append sample age
        strcat(str, "ms,");

        if(OK){ //if the sample is fresh enough
            strcat(str, "OK");
        }
        else{ //if the sample is too old
            strcat(str, "OLD");
        }
    }
    else{ //if the sample is invalid
        strcat(str, "NaN,C,NaN,Pa,NaN,lx,NaN,,NaN,C,NaN,pct,NaN,ms,INVALID");
    }

    addChecksum(str, 299, true, false, true);

    return str;
}

//str needs to be at least 100 long
static const char* printSkew(const Dut* dut, char* str)
{
    float D, skew_ns;
    int64_t absTime_ticks;
    SkewStatus skewStatus = getSkew(&D, &skew_ns, &absTime_ticks, dut);

    strcpy(str, "$SKEW,");

    switch(skewStatus){
        case skewOK:
            if(fabsf(skew_ns) < 1.0e9f){ //if the skew is smaller than 1s
                appendFloat(str, D, 2, 10, true);
                strcat(str, "ms,");
                appendFloat(str, skew_ns, 2, 20, true);
                strcat(str, "ns,");
                appendInt64(str, absTime_ticks, 20, true);
                strcat(str, "ticks,OK");
            }
            else{ //this shouldn't be possible, due to how the program is written
                appendFloat(str, D, 2, 10, true);
                strcat(str, "ms,");
                strcat(str, "inf,ns,");
                appendInt64(str, absTime_ticks, 20, true);
                strcat(str, "ticks,???");
            }
            break;

        case ppsGlitch:
            appendFloat(str, D, 2, 10, true);
            strcat(str, "ms,NaN,ns,,ticks,GLITCH");
            break;

        case ppsOld:
            appendFloat(str, D, 2, 10, true);
            strcat(str, "ms,NaN,ns,");
            appendInt64(str, absTime_ticks, 20, true);
            strcat(str, "ticks,OLD");
            break;

        case ppsMissing:
            strcat(str, "NaN,ms,NaN,ns,,ticks,PPS_MISSING");
            break;

        case refMissing:
            appendFloat(str, D, 2, 10, true);
            strcat(str, "ms,NaN,ns,");
            appendInt64(str, absTime_ticks, 20, true);
            strcat(str, "ticks,REF_MISSING");
            break;
    }

    addChecksum(str, 99, true, false, true);

    return str;
}

static SkewStatus getSkew(float* D, float* skew_ns, int64_t* absTime_ticks, const Dut* dut)
{
    TickType_t invocTime = xTaskGetTickCount();

    Dut dutPps;

    taskENTER_CRITICAL();
    {
        dutPps = (*dut); //copy the dut

        if(dut->status != captured){ //if the DUT PPS is not currently waiting to be paired to a ref PPS
            ((Dut*)dut)->status = waiting; //reset the DUT PPS
        }
        else{ //if the PPS is captured but not paired yet
            xEventGroupClearBits(dut->flags, ppsReadyFlag);//clear the ready flag
        }
    }
    taskEXIT_CRITICAL();

    SkewStatus skewStatus;

    if(dutPps.status != waiting){ //if a PPS was captured
        *absTime_ticks = dutPps.time;

        float d = calcTimeDiff_ms(dutPps.sysTime, invocTime); //calculate the delay

        *D = d;

        if(d < D_PPS_MESSAGE_MS){ //if the PPS is in time
            if(dutPps.status == captured){ //if the PPS was in a captured state
                //wait for it to get finalized
                xEventGroupWaitBits(dut->flags,
                                    ppsReadyFlag,
                                    pdTRUE, //clear the flags
                                    pdTRUE, //all the flags are required (to be set)
                                    portMAX_DELAY); //block indefinitely

                taskENTER_CRITICAL();
                {
                    dutPps = (*dut); //copy the dut
                    ((Dut*)dut)->status = waiting; //reset the DUT PPS
                }
                taskEXIT_CRITICAL();
            }

            switch(dutPps.status){
                case available: {

                    *skew_ns = ((float)dutPps.skew) * (1.0e9f / ((float)SystemCoreClock));

                    skewStatus = skewOK;

                    break;
                }

                case glitch:{
                    skewStatus = ppsGlitch;
                    break;
                }

                case missingRef:{
                    skewStatus = refMissing;
                    break;
                }
            }
        }
        else{ //if there isn't a PPS in the designated time window -> the PPS is missing
            skewStatus = ppsOld;
        }
    }
    else{ //if there was no PPS captured
        skewStatus = ppsMissing;
    }

    return skewStatus;
}

static uint32_t getTimerSnapshot(uint32_t base, uint32_t timer)
{
    uint32_t timerSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAR) : HWREG(base + TIMER_O_TBR));
    uint32_t psSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAPS) : HWREG(base + TIMER_O_TBPS));
    return ((uint32_t)((((uint32_t)0xFFu) & psSnapshot) << 16)) | (((uint32_t)0xFFFFu) & timerSnapshot);
}

static const TickType_t TickType_t_MAX = (((TickType_t)0) - ((TickType_t)1));
//calculates the time difference (in ms) between 2 invocations of the xTaskGetTickCount/xTaskGetTickCountFromISR functions: A(earlier) -> B(older)
static float calcTimeDiff_ms(TickType_t A, TickType_t B)
{
    float diff_ms;

    if(A <= B){ //if there wasn't an overflow
        diff_ms = (B - A) * portTICK_PERIOD_MS;
    }
    else{ //if there was an overflow
        diff_ms = (B + (TickType_t_MAX - A)) * portTICK_PERIOD_MS;
    }

    return diff_ms;
}
