//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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



//configurations
//overflow and the PPS signals are in the same priority group but the overflow has a higher sub priority
static const uint8_t OVERFLOW_INTERRUPT_PRIORITY = 2;
static const uint8_t PPS_INTERRUPT_PRIORITY = 3;
static const int16_t D_REF_DUT_MS = 500; //the maximum skew in ms between the ref and dut PPS for them to be deemed a pair
static const int16_t D_PPS_MESSAGE_MS = 500; //the maximum delay in ms between the dut PPS and the start off the NMEA packet for them to be deemed a pair

static int64_t dRefDut_clk; //the maximum skew in clock ticks under witch the ref and dut can be deemed a pair
static int64_t dPpsMessage_clk; //the maximum delay in ticks between the dut PPS and the start off the NMEA packet for them to be deemed a pair

void ISR_TIMER0_A(void);
void ISR_TIMER2_A(void);
void ISR_TIMER2_B(void);

static TaskHandle_t messageTaskHandle;
static void message_task(void* pvParameters);

static TaskHandle_t errTaskHandle;
static void err_task(void* pvParameters);

static TaskHandle_t timeoutTaskHandle;
static void timeout_task(void* pvParameters);

static bool checkRef(int64_t* skew);

static uint32_t getTimerSnapshot(uint32_t base, uint32_t timer);
static uint32_t getTimerFreerunning(uint32_t base, uint32_t timer);

typedef enum{
    available,
    captured,
    waiting,
    glitch,
    missingRef
}Status;

static volatile EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t ppsReadyFlag = ((EventBits_t) 1) << 0;

typedef struct{
    int64_t time;
    int64_t skew;
    Status status;

}Dut;

static volatile Dut dut;

static volatile struct{
    int64_t time;
    bool available;
}ref;

static volatile int64_t overflowCount;

//overflow
void ISR_TIMER0_A(void)
{
    TimerIntClear(TIMER0_BASE, (uint32_t)(~((uint32_t)0))); //clear the interrupt

    uint32_t capTimerIntStatus = TimerIntStatus(TIMER2_BASE, true); //get capture timer interrupt status

    if(capTimerIntStatus & TIMER_CAPA_EVENT){ //if there is a pending ref PPS interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER2_BASE, TIMER_A); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the ref PPS came before the overflow
            ISR_TIMER2_A(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER2A); //clear the pending interrupt in the NVIC
        }
    }

    if(capTimerIntStatus & TIMER_CAPB_EVENT){ //if there is a pending DUT PPS interrupt
        uint32_t timestamp = getTimerSnapshot(TIMER2_BASE, TIMER_B); //get the timestamp

        if(timestamp > (0xFFFFFFu / 2u)){ //if the timestamp is greater than half of the range of the counter than it is most likely that the DUT PPS came before the overflow
            ISR_TIMER2_B(); //handle the ref PPS interrupt
            IntPendClear(INT_TIMER2B); //clear the pending interrupt in the NVIC
        }
    }

    overflowCount++;
}

//ref PPS
void ISR_TIMER2_A(void)
{
    uint32_t callers = TimerIntStatus(TIMER2_BASE, true) & (~TIMER_CAPB_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER2_BASE, callers);    //clears the interrupt flags

    ref.available = true; //the ref is now available
    ref.time = ((int64_t)getTimerSnapshot(TIMER2_BASE, TIMER_A)) + ((int64_t)(overflowCount<<24)); //saving the time of capture

    if(dut.status == captured){ //if the DUT PPS is waiting for a ref PPS
        int64_t skew;
        if(checkRef(&skew)){ //if the new ref can be paired to the pending dut PPS
            dut.skew = skew; //update the skew of the dut
            dut.status = available; //the dut is now available

            ref.available = false; //the ref has been used up / paired
        }
        else{ //if the new ref can't be paired to the captured PPS that means a ref PPS is missing
            dut.status = missingRef;
        }

        BaseType_t higherPriorityTaskWoken;
        xEventGroupSetBitsFromISR(flags, ppsReadyFlag, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

//DUT PPS
void ISR_TIMER2_B(void)
{
    uint32_t callers = TimerIntStatus(TIMER2_BASE, true) & (~TIMER_CAPA_EVENT);   //determines what triggered the interrupt
    TimerIntClear(TIMER2_BASE, callers);    //clears the interrupt flags

    if(dut.status == waiting){
        dut.time = ((int64_t)getTimerSnapshot(TIMER2_BASE, TIMER_B)) + ((int64_t)(overflowCount<<24)); //save the capture time

        if(ref.available){ //if a ref PPS is available
            int64_t skew;
            if(checkRef(&skew)){ //if the new ref can be paired to the new dut PPS
                dut.skew = skew; //update the skew of the dut
                dut.status = available; //the dut is now available

                ref.available = false; //the ref has been used up / paired
            }
            else{ //if the new dut PPS doesn't belong to the ref on record
                dut.status = captured;

                BaseType_t higherPriorityTaskWoken;
                //signal to task
                vTaskNotifyGiveFromISR(errTaskHandle,
                                &higherPriorityTaskWoken);
                portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
                //error(extraRef);/************************************************************************************************************************************************/
            }
        }
        else{ //if a ref PPS is not immediately available
            dut.status = captured;
        }
    }
    else{ //if a new dut PPS came before the previous got handled
        dut.status = glitch; //a glitch occurred

        BaseType_t higherPriorityTaskWoken;
        xEventGroupSetBitsFromISR(flags, ppsReadyFlag, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

static void timeout_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(D_REF_DUT_MS));

        uartPrintLn(usb,"ttt");
    }
}

static bool checkRef(int64_t* skew)
{
    int64_t d;

    d = dut.time - ref.time; //calculate the skew

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
    dut.status = waiting;
    ref.available = false;
    overflowCount = 0;
    dRefDut_clk = (((int64_t)SystemCoreClock) / ((int64_t)1000)) * D_REF_DUT_MS;
    dPpsMessage_clk = (((int64_t)SystemCoreClock) / ((int64_t)1000)) * D_PPS_MESSAGE_MS;

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, ppsReadyFlag);

    //enable the IO port for the mess interrupt (PK5 pin)
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){}
    }

    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_5);
    GPIOIntTypeSet(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTK_BASE, GPIO_PIN_5);
    IntPrioritySet(INT_GPIOK, 6 << (8 - configPRIO_BITS));
    IntEnable(INT_GPIOK);


    //enable the IO port for the input capture legs
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){}
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

    //configures timers for two-signal edge-separation
    GPIOPinConfigure(GPIO_PM0_T2CCP0);   //sets PM0 as the input capture input for TIMER2 TIMER A
    GPIOPinConfigure(GPIO_PM1_T2CCP1);   //sets PM1 as the input capture input for TIMER2 TIMER B
    GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1); //configures the pins
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);  //TIMER0 is clocked from the system clock
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);  //TIMER2 is clocked from the system clock
    TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP)); //configures TIMER0
    TimerConfigure(TIMER2_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP)); //configures TIMER2 TIMER A/B
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);   //configures Timer A to capture rising edges
    TimerControlEvent(TIMER2_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);   //configures Timer B to capture rising edges
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //enables overflow interrupt on TIMER0 A
    TimerIntEnable(TIMER2_BASE, (TIMER_CAPB_EVENT | TIMER_CAPA_EVENT));   //enables interrupt for edge capture on TIMER2 TIMER A/B
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFF); //sets the max value of the counter
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xFF); //sets the max value of the prescaler
    TimerLoadSet(TIMER2_BASE, TIMER_BOTH, 0xFFFF);
    TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, 0xFF);
    TimerEnable(TIMER2_BASE, TIMER_BOTH);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerSynchronize(TIMER0_BASE, (TIMER_0A_SYNC | TIMER_2A_SYNC | TIMER_2B_SYNC));  //Synchronizes TIMER0 A and TIMER2 A and B with each other
    IntEnable(INT_TIMER0A);
    IntPrioritySet(INT_TIMER0A, OVERFLOW_INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //the priority bits need to be shifted up, to the implemented bits
    IntEnable(INT_TIMER2A);
    IntPrioritySet(INT_TIMER2A, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));
    IntEnable(INT_TIMER2B);
    IntPrioritySet(INT_TIMER2B, PPS_INTERRUPT_PRIORITY << (8 - configPRIO_BITS));


    xTaskCreate(&message_task,
                "messageTask",
                3072,
                NULL,
                3,
                &messageTaskHandle);

    xTaskCreate(&err_task,
                "errTask",
                512,
                NULL,
                2,
                &errTaskHandle);

    xTaskCreate(&timeout_task,
                "timeoutTask",
                512,
                NULL,
                4,
                &timeoutTaskHandle);
}

static void err_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uartPrintLn(usb,"extraRef");
    }
}

void ISR_GPIOK(void)
{
    GPIOIntClear(GPIO_PORTK_BASE, (uint32_t)(~((uint32_t)0)));


    BaseType_t higherPriorityTaskWoken;
    //signal to task
    vTaskNotifyGiveFromISR(messageTaskHandle,
                    &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
}

static void message_task(void* pvParameters)
{
    while(true){
        TickType_t start = xTaskGetTickCount();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //wait for the start of an NMEA package
        TickType_t end = xTaskGetTickCount();

        float waitTime = (end - start) * portTICK_PERIOD_MS;
        char str[100];
        sprintf(str, "Wait Time: %.2f ms\t\t", waitTime);
        uartPrint(usb, str);

        Dut dutPps;
        int64_t time, overflowCount_I;
        uint32_t time0;

        IntMasterDisable();
        {
            time0 = getTimerFreerunning(TIMER0_BASE, TIMER_A);
            overflowCount_I = overflowCount;
            dutPps = dut; //copy the dut

            if(dut.status != captured){ //if the DUT PPS is not currently waiting to be paired to a ref PPS
                dut.status = waiting; //reset the DUT PPS
            }
            else{ //if the PPS is captured but not paired yet
                xEventGroupClearBits(flags, ppsReadyFlag);//clear the ready flag
            }
        }
        IntMasterEnable();

        time = ((int64_t)time0) + ((int64_t)(overflowCount_I << 24)); //get the current time

        if(dutPps.status != waiting){ //if a PPS was captured
            int64_t d;
            d = time - dutPps.time; //calculate the delay

            char str[100];
            sprintf(str, "D: %.2f ms\t\t", ((float)d) * (1.0e3f / ((float)SystemCoreClock)));
            uartPrint(usb, str);

            if(d<0){
                uartPrintLn(usb, "???????");
            }

            if(d < dPpsMessage_clk){ //if the PPS is in time
                if(dutPps.status == captured){ //if the PPS was in a captured state
                    //wait for it to get finalized
                    xEventGroupWaitBits(flags,
                                        ppsReadyFlag,
                                        pdTRUE, //clear the flags
                                        pdTRUE, //all the flags are required (to be set)
                                        portMAX_DELAY); //block indefinitely

                    IntMasterDisable();
                    {
                        dutPps = dut; //copy the dut
                        dut.status = waiting; //reset the DUT PPS
                    }
                    IntMasterEnable();
                }

                switch(dutPps.status){
                case available: {
                    float skew = ((float)dutPps.skew) * (1.0e9f / ((float)SystemCoreClock));

                    char str[500];
                    sprintf(str, "Skew: %.2f ns\t\traw skew: %lld", skew, dutPps.skew);
                    uartPrintLn(usb, str);

                    break;
                }

                case glitch:{
                    uartPrintLn(usb, "glitch");

                    break;
                }

                case missingRef:{
                    uartPrintLn(usb, "missingRef");

                    break;
                }
                }
            }
            else{ //if there isn't a PPS in the designated time window -> the PPS is missing
                uartPrintLn(usb, "missingPPS: old");
            }
        }
        else{ //if there was no PPS captured
            uartPrintLn(usb, "missingPPS: waiting");
        }
    }
}

static uint32_t getTimerSnapshot(uint32_t base, uint32_t timer)
{
    uint32_t timerSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAR) : HWREG(base + TIMER_O_TBR));
    uint32_t psSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAPS) : HWREG(base + TIMER_O_TBPS));
    return ((uint32_t)((((uint32_t)0xFFu) & psSnapshot) << 16)) | (((uint32_t)0xFFFFu) & timerSnapshot);
}

static uint32_t getTimerFreerunning(uint32_t base, uint32_t timer)
{
    uint32_t timerSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAV) : HWREG(base + TIMER_O_TBV));
    uint32_t psSnapshot = ((timer == TIMER_A) ? HWREG(base + TIMER_O_TAPV) : HWREG(base + TIMER_O_TBPV));
    return ((uint32_t)((((uint32_t)0xFFu) & psSnapshot) << 16)) | (((uint32_t)0xFFFFu) & timerSnapshot);
}
