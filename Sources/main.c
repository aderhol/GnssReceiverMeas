//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>

//driver includes
#include <driverlib/fpu.h> //floating point unit API

//user includes
#include <init.h>
#include <uartIO.h>
#include <sensorHub.h>
#include <nmea.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"    //interrupt defines (eg. INT_UART0)
#include "inc/hw_timer.h" //timer register offsets
#include "driverlib/timer.h"    //timer
#include "driverlib/interrupt.h"    //interrupt API
#include "driverlib/sysctl.h" //System Control



static TaskHandle_t testerTaskHandle;
static void taskListInit_task(void* pvParameters);
static void tester_task(void* pvParameters);

int main(void)
{
    init(pdMS_TO_TICKS(100));

    TaskHandle_t taskListTask_handle;
    xTaskCreate(&taskListInit_task,
                "Task List Init Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                (configMAX_PRIORITIES - 1), //maximum prioroty,
                &taskListTask_handle);

    /*xTaskCreate(&tester_task,
                "tester",
                3072,
                NULL,
                1,
                &testerTaskHandle);*/

    uartPrintLn(usb, "");
    uartPrintLn(usb, "Started!");

    vTaskStartScheduler();

    while(true){
    }
}

static void tester_task(void* pvParameters)
{
    while(true){

        SensorData sensorData;

        bool OK = sampleSensors(pdMS_TO_TICKS(150), &sensorData);

        char str[500];
        sprintf(str, "OK: %d\t\t\t\ttemperature_bmp180: %.2f °C\tpressure: %.2f Pa\t\t\t\tvisible light intensity: %.2f lx\tinfrared light intensity: %d\t\t\t\ttemperature_sh21: %.2f °C\trelative humidity: %.2f%%",
                OK,
                sensorData.temperature_C_bmp180, sensorData.pressure_Pa,
                sensorData.visibleLightIntensity_lx, sensorData.infraredLighIntensity,
                sensorData.temperature_C_sht21, sensorData.relativeHumidity_percentage
                );
        uartPrintLn(usb, str);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static const size_t taskListLength = 50;
bool taskListValid = false;
static size_t numberOfTasksV = 0;
static TaskHandle_t taskListArray[taskListLength];

static void taskListInit_task(void* pvParameters)
{
    static TaskStatus_t statList[taskListLength];
    numberOfTasksV = uxTaskGetSystemState(statList,
                                         taskListLength,
                                         NULL);

    while(0 == numberOfTasksV); //the task list array was too short

    size_t i;
    for(i = 0; i < numberOfTasksV; i++){ //copies the task handles to the global variable
        taskListArray[i] = statList[i].xHandle;
    }

    taskListValid = true; //setting the valid flag for the task list array and the number of tasks variable

    vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
}

bool getTaskList(TaskHandle_t (*taskList)[], size_t* numberOfTasks, size_t outputArrayLenght)
{
    while(outputArrayLenght < numberOfTasksV);

    size_t i;
    for(i = 0; i < numberOfTasksV; i++){
        (*taskList)[i] = taskListArray[i];
    }

    *numberOfTasks = numberOfTasksV;

    return taskListValid;
}

void vApplicationMallocFailedHook(void)
{
    while(true){
    }
}

void vApplicationStackOverflowHook(TaskHandle_t offendingTaskHandle, signed char* offendingTaskName)
{
    while(true){
    }
}

//usage statistics
void initDebCnt(void)
{
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)){}
    }

    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);  //TIMER0 is clocked from the system clock

    TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP)); //configures TIMER1

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //enables overflow interrupt on TIMER0 A
    TimerLoadSet(TIMER1_BASE, TIMER_A, 1200u); //sets the max value of the counter
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 0); //sets the max value of the prescaler

    TimerEnable(TIMER1_BASE, TIMER_A);

    IntEnable(INT_TIMER1A);
    IntPrioritySet(INT_TIMER1A, 5 << (8 - configPRIO_BITS)); //the priority bits need to be shifted up, to the implemented bits
}

int32_t debTickToMs(uint64_t numOfTicks)
{
    return (numOfTicks * (float)(1201.0 / 120000.0)) + 0.5f;
}

uint64_t debTime = 0;

void ISR_TIMER1_A(void)
{
    TimerIntClear(TIMER1_BASE, (uint32_t)(~((uint32_t)0))); //clear the interrupt

    debTime++;
}

uint64_t debCnt(void)
{
    return debTime;
}
