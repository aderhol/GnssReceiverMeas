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
#include "Sources/Sensor_Hub/inc/sen_bmp180.h"
#include "Sources/Sensor_Hub/inc/sen_ISL29023.h"
#include "Sources/Sensor_Hub/inc/sen_sht21.h"
#include <uartIO.h>
#include "Sources/Sensor_Hub/inc/senHubI2c.h"




static TaskHandle_t testerTaskHandle;
static void tester_task(void* pvParameters);

int main(void)
{
    init();

    xTaskCreate(&tester_task,
                "tester",
                1024,
                NULL,
                1,
                &testerTaskHandle);

    uartPrintLn(usb, "Started!");

    vTaskStartScheduler();

    while(true){
    }
}

static void tester_task(void* pvParameters)
{
    while(true){
        float temp, pressure;
        bool OK = bmp180GetData(&temp, &pressure);

        char str[100];
        sprintf(str, "OK: %d\t\ttemperature: %.1f\tpressure: %.0f", OK, temp, pressure);
        //sprintf(str, "reset: %d    start: %d    start2: %d    calibCnt: %d    rawTempCnt: %d    rawPressureCnt: %d    temp: %.2f    pressure: %d", reset, start, start2, calibCnt, rawTempCnt, rawPressureCnt, temp, pressure);
        uartPrintLn(usb, str);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
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
