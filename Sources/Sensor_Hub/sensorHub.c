//std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

//driver includes
#include <driverlib/gpio.h> //GPIO
#include <driverlib/sysctl.h> //System Control

//user includes
#include <sensorHub.h>
#include <sen_bmp180.h>
#include <sen_ISL29023.h>
#include <sen_sht21.h>
#include <pinDefs.h>



//configurations


extern void senHubI2cInit(void);

static void feedbackLED_task(void* pvParameters);
static void sampling_task(void* pvParameters);
static void sampling_timerService(TimerHandle_t xTimer);

static TaskHandle_t feedbackLedTaskHandle;

static TaskHandle_t samplingTaskHandle;
static SemaphoreHandle_t samplingMutex;

void sensorHubInit(TickType_t maxDelay_ticks)
{
    xTaskCreate(&feedbackLED_task,
                "sensor feedback LED task",
                256,
                NULL,
                1,
                &feedbackLedTaskHandle);

    xTaskCreate(&sampling_task,
                "sensor sampling task",
                512,
                NULL,
                1,
                &samplingTaskHandle);

    TimerHandle_t timer = xTimerCreate("samplingTimer",
                                       pdMS_TO_TICKS(400),
                                       pdTRUE,
                                       (void*) 0,
                                       &sampling_timerService);
    xTimerStart(timer, 0);

    samplingMutex = xSemaphoreCreateMutex();


    if(!SysCtlPeripheralReady(bp_LED.periphery)) {
        SysCtlPeripheralEnable(bp_LED.periphery);
        while(!SysCtlPeripheralReady(bp_LED.periphery)){}
    }

    GPIOPinTypeGPIOOutput(bp_LED.portBase, bp_LED.pin);

    senHubI2cInit(); //I2C for the sensors

    bmp180Init(maxDelay_ticks); //temperature and pressure
    isl29023Init(maxDelay_ticks); //visible and IR light intensity
    sht21Init(maxDelay_ticks); //relative humidity and temperature
}

bool sampleSensors(TickType_t maxDelay_ticks, SensorData* sensorData)
{
    bool OK;

    if(
            bmp180GetData(maxDelay_ticks, &(sensorData->temperature_C_bmp180), &(sensorData->pressure_Pa)) &&
            isl29023GetData(maxDelay_ticks, &(sensorData->visibleLightIntensity_lx), &(sensorData->infraredLighIntensity)) &&
            sht21GetData(maxDelay_ticks, &(sensorData->temperature_C_sht21), &(sensorData->relativeHumidity_percentage))
            ){
        OK = true;
    }
    else{
        OK = false;
    }

    xTaskNotify(feedbackLedTaskHandle,
                (OK ? 1 : 0),
                eSetValueWithOverwrite);

    return OK;
}

static volatile SensorData samplePoint;
static bool senorDataValid = false;
static void sampling_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        SensorData sample;
        bool OK = sampleSensors(pdMS_TO_TICKS(150), &sample);
        xSemaphoreTake(samplingMutex, portMAX_DELAY);
        {
            samplePoint = sample;
            senorDataValid = OK;
            samplePoint.sysTime = xTaskGetTickCount();
        }
        xSemaphoreGive(samplingMutex);
    }
}

bool getSample(SensorData* sensorData)
{
    bool OK;

    OK = (pdTRUE == xSemaphoreTake(samplingMutex, 1));
    if(OK){
        *sensorData = samplePoint;
        OK = senorDataValid;

        xSemaphoreGive(samplingMutex);
    }

    return OK;
}

static void sampling_timerService(TimerHandle_t xTimer)
{
    xTaskNotifyGive(samplingTaskHandle);
}

//1 flash: the sensors were sampled successfully, 2 flashes: the sensors couldn't be sampled
static void feedbackLED_task(void* pvParameters)
{
    GPIOPinWrite(bp_LED.portBase, bp_LED.pin, 0); //turn OFF the LED at start up

    while(true){
        uint32_t notificationValue;

        xTaskNotifyWait(0, 0, &notificationValue, portMAX_DELAY); //wait for notification from the 'sampleSesnors' function

        bool OK = (notificationValue == 1); //convert the value back to a bool

        if(OK){ //the sampling was successful; 1 flash
            GPIOPinWrite(bp_LED.portBase, bp_LED.pin, 0xFF); //turn ON the LED

            vTaskDelay(pdMS_TO_TICKS(100)); //leave the LED ON for 100 ms

            GPIOPinWrite(bp_LED.portBase, bp_LED.pin, 0); //turn OFF the LED

            vTaskDelay(pdMS_TO_TICKS(140)); //leave the LED OFF for 140 ms (at least)
        }
        else{ //the sampling was unsuccessful; 2 flashes
            int_fast8_t i;
            for(i = 0; i < 2; i++){
                GPIOPinWrite(bp_LED.portBase, bp_LED.pin, 0xFF); //turn ON the LED

                vTaskDelay(pdMS_TO_TICKS(20)); //leave the LED ON for 20 ms

                GPIOPinWrite(bp_LED.portBase, bp_LED.pin, 0); //turn OFF the LED

                vTaskDelay(pdMS_TO_TICKS(50)); //leave the LED OFF for 50 ms
            }

            vTaskDelay(pdMS_TO_TICKS(100)); //leave the LED OFF for (at least) and additional 100 ms
        }
    }
}
