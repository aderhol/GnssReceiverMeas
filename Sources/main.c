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

        SensorData sensorData;

        bool OK = sampleSensors(&sensorData);

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
