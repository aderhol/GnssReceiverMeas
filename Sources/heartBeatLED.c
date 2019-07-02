//std library includes
#include <stdbool.h>
#include <stdint.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>

//driver includes
#include <driverlib/gpio.h> //GPIO
#include <driverlib/sysctl.h> //System Control

//user includes
#include <pinDefs.h>
#include <freeRTOSConfig.h>



//configurations
static const uint32_t LED_BLINKING_PERIOD_MS = 2000;
static const uint8_t TASK_PRIORITY = 1;

static void heartBeatLed_timerService(TimerHandle_t xTimer);
static TaskHandle_t heartBeatLedTaskHandle;
static void heartBeatLed_task(void* pvParameters);

void heartBeatLedInit(void)
{
    if(!SysCtlPeripheralReady(LED1.periphery)) {
        SysCtlPeripheralEnable(LED1.periphery);
        while(!SysCtlPeripheralReady(LED1.periphery)){}
    }
    
    GPIOPinTypeGPIOOutput(LED1.portBase, LED1.pin);
    
    TimerHandle_t timer = xTimerCreate("heartBeatLedTimer",
                                       pdMS_TO_TICKS(LED_BLINKING_PERIOD_MS / 2),
                                       pdTRUE,
                                       (void*) 0,
                                       &heartBeatLed_timerService);
    xTimerStart(timer, 0);
    
    xTaskCreate(&heartBeatLed_task,
                "heartBeatLedTask",
                configMINIMAL_STACK_SIZE,
                NULL, //parameter to task
                TASK_PRIORITY, //priority
                &heartBeatLedTaskHandle);   
}

static volatile bool ledIsOn = false;
static void heartBeatLed_task(void* pvParameters)
{
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        GPIOPinWrite(LED1.portBase, LED1.pin, (ledIsOn ? 0 : 0xFF));
        ledIsOn = !ledIsOn;
    }
}


static void heartBeatLed_timerService(TimerHandle_t xTimer)
{
    xTaskNotifyGive(heartBeatLedTaskHandle);
}
