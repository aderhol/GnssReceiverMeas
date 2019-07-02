///std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>
#include <task.h>

//driver includes
#include <sensorlib/hw_sht21.h>
#include <sensorlib/i2cm_drv.h>
#include <sensorlib/sht21.h>

//user includes
#include <sen_sht21.h>



//configurations
static const uint_fast8_t SHT21_I2C_ADDRESS = 0x40;

static void initSensor(tI2CMInstance* sensorI2C);
static tSHT21 sensor;
static EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t sensorIsInitialized = ((EventBits_t) 1) << 0;
static void initCallback(void* pvData, uint_fast8_t status);
static void initSensor(tI2CMInstance* sensorI2C);
static void setUpCallback(void* pvData, uint_fast8_t status);
static SemaphoreHandle_t sensorMutex;
static SemaphoreHandle_t conversionStarted;
static SemaphoreHandle_t dataReady;

void sht21Init(tI2CMInstance* sensorI2C)
{
    initSensor(sensorI2C);

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, sensorIsInitialized);

    sensorMutex = xSemaphoreCreateMutex();

    conversionStarted = xSemaphoreCreateBinary();
    xSemaphoreTake(conversionStarted, 0);

    dataReady = xSemaphoreCreateBinary();
    xSemaphoreTake(dataReady, 0);
}

static void initSensor(tI2CMInstance* sensorI2C)
{
    static tI2CMInstance* sensorI2C_ = NULL;
    sensorI2C_ = (sensorI2C_ == NULL) ? sensorI2C : sensorI2C_; //back up for possible retry due to I2C error

    //initializes the driver
    SHT21Init(&sensor,
              sensorI2C_,
              SHT21_I2C_ADDRESS,
              &initCallback,
              NULL);

}

static void initCallback(void* pvData, uint_fast8_t status) //it is called from the I2C ISR
{
    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        initSensor(NULL); //retry
    }
    else{ //if there weren't any errors
        //sets measurement resolution 12 bits for relative humidity and 14 bits for temperature
        SHT21ReadModifyWrite(&sensor,
                             SHT21_CMD_WRITE_CONFIG,
                             ~((uint8_t)(SHT21_CONFIG_RES_M)),
                             SHT21_CONFIG_RES_12,
                             &setUpCallback,
                             NULL);
    }
}


static void setUpCallback(void* pvData, uint_fast8_t status) //it is called from the I2C ISR
{
    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        initCallback(NULL, I2CM_STATUS_SUCCESS); //retry, but not the initialization
    }
    else{ //if there weren't any errors
        BaseType_t higherPriorityTaskWoken;
        //set the 'sensor is initialized' flag
        xEventGroupSetBitsFromISR(flags,
                                  sensorIsInitialized,
                                  &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
    }
}


static void tempReadyCallback(void* pvData, uint_fast8_t status);
static void rhReadyCallback(void* pvData, uint_fast8_t status);

static void tempReadyCallback(void* pvData, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        SHT21Write(&sensor,
                   SHT21_CMD_MEAS_T_HOLD,
                   sensor.pui8Data,
                   0u,
                   &tempReadyCallback,
                   NULL);
    }
    else{ //if there weren't any errors
        BaseType_t higherPriorityTaskWoken;
        //set the flag
        xSemaphoreGiveFromISR(conversionStarted,
                              &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
    }
}

static void rhReadyCallback(void* pvData, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        SHT21Write(&sensor,
                   SHT21_CMD_MEAS_RH_HOLD,
                   sensor.pui8Data,
                   0u,
                   &rhReadyCallback,
                   NULL);
    }
    else{ //if there weren't any errors
        BaseType_t higherPriorityTaskWoken;
        //set the flag
        xSemaphoreGiveFromISR(conversionStarted,
                              &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
    }
}



static void dataReadyCallback(void* pvData, uint_fast8_t status);

static void dataReadyCallback(void* pvData, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        SHT21DataRead(&sensor,
                      &dataReadyCallback,
                      NULL);
    }
    else{ //if there weren't any errors
        BaseType_t higherPriorityTaskWoken;
        //set the flag
        xSemaphoreGiveFromISR(dataReady,
                              &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
    }
}

void sht21GetData(float* temperature_C, float* relHum_percentage)
{
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    {
        //wait for the sensor to be initialized
        xEventGroupWaitBits(flags,
                            sensorIsInitialized,
                            pdFALSE, //don't clear the flags
                            pdTRUE, //all the flags are required (to be set)
                            portMAX_DELAY); //block indefinitely


        //start temperature conversion
        SHT21Write(&sensor,
                   SHT21_CMD_MEAS_T_HOLD,
                   sensor.pui8Data,
                   0u,
                   &tempReadyCallback,
                   NULL);

        xSemaphoreTake(conversionStarted, portMAX_DELAY); //wait for the I2C operation to complete

        SHT21DataRead(&sensor,
                      &dataReadyCallback,
                      NULL);

        xSemaphoreTake(dataReady, portMAX_DELAY); //wait for the I2C operation to complete

        SHT21DataTemperatureGetFloat(&sensor, temperature_C); //save the measured temperature to the output variable


        //start humidity conversion
        SHT21Write(&sensor,
                   SHT21_CMD_MEAS_RH_HOLD,
                   sensor.pui8Data,
                   0u,
                   &rhReadyCallback,
                   NULL);

        xSemaphoreTake(conversionStarted, portMAX_DELAY); //wait for the I2C operation to complete

        SHT21DataRead(&sensor,
                      &dataReadyCallback,
                      NULL);

        xSemaphoreTake(dataReady, portMAX_DELAY); //wait for the I2C operation to complete

        SHT21DataHumidityGetFloat(&sensor, relHum_percentage); //save the measured humidity to the output variable
    }
    xSemaphoreGive(sensorMutex);
}
