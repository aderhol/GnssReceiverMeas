//std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>

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

volatile TaskHandle_t feedbackLedTaskHandle;

void sensorHubInit(TickType_t maxDelay_ticks)
{
    TaskHandle_t feedbackLedTaskHandle_;
    xTaskCreate(&feedbackLED_task,
                "sensor feedback LED task",
                256,
                NULL,
                1,
                &feedbackLedTaskHandle_);

    feedbackLedTaskHandle = feedbackLedTaskHandle_;


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
