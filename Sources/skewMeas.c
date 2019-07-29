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



//configurations
//overflow and the PPS signals are in the same priority group but the overflow has a higher sub priority
static const uint8_t OVERFLOW_INTERRUPT_PRIORITY = 2;
static const uint8_t PPS_INTERRUPT_PRIORITY = 3;
static const int16_t D_REF_DUT_MS = 500; //the maximum skew in ms between the ref and dut PPS for them to be deemed a pair
static const int16_t D_PPS_MESSAGE_MS = 500; //the maximum delay in ms between the dut PPS and the start off the NMEA packet for them to be deemed a pair

static int64_t dRefDut_clk; //the maximum skew in clock ticks under witch the ref and dut can be deemed a pair

void ISR_TIMER0_A(void);
void ISR_TIMER2_A(void);
void ISR_TIMER2_B(void);

static TaskHandle_t errTaskHandle;
static void err_task(void* pvParameters);

static TimerHandle_t timeoutTimerHandle;
static void timeout_timerService(TimerHandle_t xTimer);

static bool checkRef(int64_t* skew);

static uint32_t getTimerSnapshot(uint32_t base, uint32_t timer);

static float calcTimeDiff_ms(TickType_t A, TickType_t B);

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
    TickType_t sysTime;
}Dut;

static volatile Dut dut;

static volatile struct{
    int64_t time;
    bool available;
}ref;

static volatile int64_t overflowCount;

static void dutARx_callback(char* str);

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

        BaseType_t higherPriorityTaskWoken = pdFALSE;

        xEventGroupSetBitsFromISR(flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(timeoutTimerHandle,
                           &higherPriorityTaskWoken);

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
        dut.sysTime = xTaskGetTickCountFromISR(); //save the current system time

        if(ref.available){ //if a ref PPS is available
            int64_t skew;
            if(checkRef(&skew)){ //if the new ref can be paired to the new dut PPS
                dut.skew = skew; //update the skew of the dut
                dut.status = available; //the dut is now available
            }
            else{ //if the new dut PPS doesn't belong to the ref on record
                dut.status = captured;

                //start the timeout timer
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xTimerStartFromISR(timeoutTimerHandle,
                                   &higherPriorityTaskWoken);

                //signal to task
                vTaskNotifyGiveFromISR(errTaskHandle,
                                &higherPriorityTaskWoken);

                portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
            }
            ref.available = false; //the ref has been used up: paired or discarded
        }
        else{ //if a ref PPS is not immediately available
            dut.status = captured;

            //start the timeout timer
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR(timeoutTimerHandle,
                               &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
    else{ //if a new dut PPS came before the previous got handled
        dut.status = glitch; //a glitch occurred

        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xEventGroupSetBitsFromISR(flags, ppsReadyFlag, &higherPriorityTaskWoken);

        //stop the timeout timer
        xTimerStopFromISR(timeoutTimerHandle,
                           &higherPriorityTaskWoken);

        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

static void timeout_timerService(TimerHandle_t xTimer)
{
    taskENTER_CRITICAL();
    {
        if(dut.status == captured){
            dut.status = missingRef;
        }
    }
    taskEXIT_CRITICAL();

    xEventGroupSetBits(flags, ppsReadyFlag);
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


    xTaskCreate(&err_task,
                "errTask",
                512,
                NULL,
                2,
                &errTaskHandle);

    timeoutTimerHandle = xTimerCreate("timeoutTimer",
                                      pdMS_TO_TICKS(D_REF_DUT_MS),
                                      pdFALSE,
                                      (void*) 0,
                                      &timeout_timerService);

    /******    UART    ******/
    uartSetRxCallback(dutA, &dutARx_callback);
}

static void err_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uartPrintLn(usb,"$EXTRAREF*0B");
    }
}

static bool stateMachine(const char* str, const char** pos, const char* pattern, const char** next);

typedef enum{
    OK,
    ppsGlitch,
    ppsOld,
    ppsMissing,
    refMissing
}SkewStatus;

static SkewStatus getSkew(float* D, float* skew_ns);
static const char* printSkew(void);

static bool safeCat(char* dest, const char* str, size_t destSize_char)
{
    if((strlen(dest) + strlen(str) + 1) <= destSize_char){ //if the dest can hold the resultant string
        strcat(dest, str);
        return true;
    }
    else{
        return false;
    }
}

static void dutARx_callback(char* str)
{
    static const char pattern1[] = "$GNRMC";
    static const char* next1 = pattern1;

    static const char pattern2[] = "$GPRMC";
    static const char* next2 = pattern1;

    static char messBuff[256] = {'\0'};
    const size_t messBuff_length = sizeof(messBuff) / sizeof(char);
    const char*const messBuffEnd = messBuff + (messBuff_length - 1);

    const char* pos;
    static bool triggered = false;

    if(!triggered){ //if the trigger is inactive
        triggered = stateMachine(str, &pos, pattern1, &next1);

        if(!triggered){
            triggered = stateMachine(str, &pos, pattern2, &next2);
        }

        if(triggered){
            for(; ((*pos) != '\n' && (*pos) != '\0'); pos++); //push pos to the nearest new line (end of the starting NMEA message) or to the end of the snippet

            if((*pos) == '\n'){ //if the new line was found
                char nextChar = *(pos + 1); //save the next char
                *((char*)(pos + 1)) = '\0'; //terminate the end of the starting message
                safeCat(messBuff, str, messBuff_length); //forward the (end of the) starting NMEA message
                *((char*)(pos + 1)) = nextChar; //restore the char

                safeCat(messBuff, printSkew(), messBuff_length); //print the skew

                safeCat(messBuff, pos + 1, messBuff_length);//forward the rest of the snippet

                triggered = false; //the trigger has been serviced
            }
            else{ //if the snippet didn't have the new line
                safeCat(messBuff, str, messBuff_length); //forward the snippet
            }
        }
        else{
            safeCat(messBuff, str, messBuff_length); //forward the snippet
        }
    }
    else{ //if the trigger is active
        pos = str; //set pos to the beginning of the snippet

        for(; ((*pos) != '\n' && (*pos) != '\0'); pos++); //push pos to the nearest new line (end of the starting NMEA message) or to the end of the snippet

        if((*pos) == '\n'){ //if the new line was found
            char nextChar = *(pos + 1); //save the next char
            *((char*)(pos + 1)) = '\0'; //terminate the end of the starting message
            safeCat(messBuff, str, messBuff_length); //forward the (end of the) starting NMEA message
            *((char*)(pos + 1)) = nextChar; //restore the char

            safeCat(messBuff, printSkew(), messBuff_length); //print the skew

            safeCat(messBuff, pos + 1, messBuff_length); //forward the rest of the snippet

            triggered = false; //the trigger has been serviced
        }
        else{ //if the snippet didn't have the new line
            safeCat(messBuff, str, messBuff_length); //forward the snippet
        }
    }




    char* buffPos = messBuff;
    char* newHead = messBuff;
    while(true){
        for(; ((*buffPos) != '\n' && (*buffPos) != '\0'); buffPos++); //push buffPos to the nearest new line or to the end of the buffer

        if((*buffPos) == '\n'){ //if the end of an NMEA message was detected
            (*buffPos) = '\0'; //terminate the message
            uartPrintLf(usb, newHead); //send the message

            buffPos++; //go to the next char
            newHead = buffPos; //update the head
        }
        else{
            break; //no more messages to be forwarded
        }
    }

    if(newHead != messBuff){ //if the buffer needs to be left shifted
        memmove(messBuff, newHead, (strlen(newHead) + 1) * sizeof(char));
    }
    else{ //no new lines were found
        if(messBuffEnd == buffPos){ //the buffer is full, and there are no new lines
            uartPrintLn(usb, messBuff); //dump the buffer
            messBuff[0] = '\0'; ///reset the buffer
        }
    }
}

static bool stateMachine(const char* str, const char** pos, const char* pattern, const char** next)
{
    for(; ((*str) != '\0') && ((**next) != '\0'); str++){ //loop while the input string ends or the patter is matched
        if((*str) == (**next)){ //if the current char in the input string matches the next char in the pattern
            (*next)++; //this char of the patter has been matched, go to the next
        }
        else{ //if the current char is not the expected char in the pattern
            *next = pattern; //reset the state machine
        }
    }

    if((**next) == '\0'){ //if the patter was found
        *pos = str - 1; //last char of pattern in input string

        *next = pattern; //reset the state machine

        return true;
    }
    else{ //if the patter hasn't yet been matched
        return false;
    }
}

static bool appendFloat(char* str, float val, size_t maxLength, bool addComma)
{
    const size_t precision = 2;


    if((1 + 1 + precision) > maxLength){ //X.X...X
        return false;
    }

    if(isnan(val)){ //if NaN
        if(3 <= maxLength){
            strcat(str, "NaN");

            return true;
        }
        else{
            return false;
        }
    }

    if(isinf(val)){ //if inf
        if(3 <= maxLength){
            strcat(str, "inf");

            return true;
        }
        else{
            return false;
        }
    }

    size_t i;

    for(; (*str) != '\0'; str++); //get to the end
    char* head = str;


    for(i = 0; i < precision; i++){
        val *= 10.0f;
    }

    //rounding
    if(val >= 0){
        val += 0.5f;
    }
    else{
        val -= 0.5f;
    }


    if(fabsf(val) > ((float)INT64_MAX)){ //if the float is too big
        return false;
    }
    else{
        int64_t iVal = (int64_t)val;

        i = 0;

        if(iVal < 0){ //if negative
            *(head++) = '-';
            maxLength--;
            iVal *= -1;
        }

        if(0 == iVal){
            head[i++] ='0';
            if(precision > 0){
                head[i++] ='.';
            }

            size_t j;
            for(j = 0; j < precision; j++){
                head[i++] = '0';
            }

            return true;
        }

        while(0 != iVal && i < maxLength){
            head[i++] = '0' + (iVal % 10);
            iVal /= 10;

            if(precision == i){
                head[i++] = '.';
                head[i++] = '0' + (iVal % 10);
                iVal /= 10;
            }
        }

        head[i] = '\0';

        if(i >= maxLength && 0 != iVal){ //if the number was too long
            return false;
        }else{ //reverse the order
            char* A = head;
            char* B = head + (i - 1);

            for(; A < B;(A++, B--)){
                char ch = *A;
                *A = *B;
                *B = ch;
            }

            if(addComma){
                head[i++] = ',';
                head[i] = '\0';
            }

            return true;
        }
    }
}

static const char* printSkew(void)
{
    float D, skew_ns;
    SkewStatus skewStatus = getSkew(&D, &skew_ns);

    static char str[100];

    strcpy(str, "$SKEW,");

    switch(skewStatus){
        case OK:
            if(fabsf(skew_ns) < 1.0e9f){ //if the skew is smaller than 1s
                appendFloat(str, D, 10, true);
                appendFloat(str, skew_ns, 20, true);
                strcat(str, "OK");
            }
            else{ //this shouldn't be possible, due to how the program is written
                appendFloat(str, D, 10, true);
                strcat(str, "inf,???");
            }
            break;

        case ppsGlitch:
            appendFloat(str, D, 10, true);
            strcat(str, "NaN,GLITCH");
            break;

        case ppsOld:
            appendFloat(str, D, 10, true);
            strcat(str, "NaN,OLD");
            break;

        case ppsMissing:
            strcat(str, "NaN,NaN,PPS_MISSING");
            break;

        case refMissing:
            appendFloat(str, D, 10, true);
            strcat(str, "NaN,REF_MISSING");
            break;
    }

    addChecksum(str, 99, true, false, true);

    return str;
}

static SkewStatus getSkew(float* D, float* skew_ns)
{
    TickType_t invocTime = xTaskGetTickCount();

    Dut dutPps;

    taskENTER_CRITICAL();
    {
        dutPps = dut; //copy the dut

        if(dut.status != captured){ //if the DUT PPS is not currently waiting to be paired to a ref PPS
            dut.status = waiting; //reset the DUT PPS
        }
        else{ //if the PPS is captured but not paired yet
            xEventGroupClearBits(flags, ppsReadyFlag);//clear the ready flag
        }
    }
    taskEXIT_CRITICAL();

    SkewStatus skewStatus;

    if(dutPps.status != waiting){ //if a PPS was captured
        float d = calcTimeDiff_ms(dutPps.sysTime, invocTime); //calculate the delay

        *D = d;

        if(d < D_PPS_MESSAGE_MS){ //if the PPS is in time
            if(dutPps.status == captured){ //if the PPS was in a captured state
                //wait for it to get finalized
                xEventGroupWaitBits(flags,
                                    ppsReadyFlag,
                                    pdTRUE, //clear the flags
                                    pdTRUE, //all the flags are required (to be set)
                                    portMAX_DELAY); //block indefinitely

                taskENTER_CRITICAL();
                {
                    dutPps = dut; //copy the dut
                    dut.status = waiting; //reset the DUT PPS
                }
                taskEXIT_CRITICAL();
            }

            switch(dutPps.status){
                case available: {

                    *skew_ns = ((float)dutPps.skew) * (1.0e9f / ((float)SystemCoreClock));

                    skewStatus = OK;

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
