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
        float temp_bmp, pressure;
        bool bmp180_OK = bmp180GetData(&temp_bmp, &pressure);

        float als;
        uint16_t ir;
        bool isl29023_OK = isl29023GetData(&als, &ir);

        float temp_sht, humidity;
        bool sht21_OK = sht21GetData(&temp_sht, &humidity);

        char str[200];
        sprintf(str, "OK: %d\t\ttemperature: %.1f °C\tpressure: %.0f Pa\t\t\t\tOK: %d\t\tals: %.2f lx\tir: %d\t\t\t\tOK: %d\t\ttemperature: %.2f °C\tRH: %.2f%%",
                bmp180_OK, temp_bmp, pressure,
                isl29023_OK, als, ir,
                sht21_OK, temp_sht, humidity
                );
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
