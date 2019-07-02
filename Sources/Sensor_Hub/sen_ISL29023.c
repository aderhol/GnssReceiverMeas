///std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>
#include <task.h>

//driver includes
#include <sensorlib/hw_isl29023.h>
#include <sensorlib/i2cm_drv.h>
#include <sensorlib/isl29023.h>

//user includes
#include <sen_ISL29023.h>



//configurations
static const uint_fast8_t ISL29023_I2C_ADDRESS = 0x44;

static void initSensor(tI2CMInstance* sensorI2C);
static tISL29023 sensor;
static EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t sensorIsInitialized = ((EventBits_t) 1) << 0;
static void initCallback(void* pvData, uint_fast8_t status);
static void initSensor(tI2CMInstance* sensorI2C);
static void setUpCallback(void* pvData, uint_fast8_t status);
static SemaphoreHandle_t sensorMutex;
static SemaphoreHandle_t conversionStarted;
static SemaphoreHandle_t dataReady;
static SemaphoreHandle_t rangeSet;

void isl29023Init(tI2CMInstance* sensorI2C)
{
    initSensor(sensorI2C);

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, sensorIsInitialized);

    sensorMutex = xSemaphoreCreateMutex();

    conversionStarted = xSemaphoreCreateBinary();
    xSemaphoreTake(conversionStarted, 0);

    dataReady = xSemaphoreCreateBinary();
    xSemaphoreTake(dataReady, 0);

    rangeSet = xSemaphoreCreateBinary();
    xSemaphoreTake(rangeSet, 0);
}

static void initSensor(tI2CMInstance* sensorI2C)
{
    static tI2CMInstance* sensorI2C_ = NULL;
    sensorI2C_ = (sensorI2C_ == NULL) ? sensorI2C : sensorI2C_; //back up for possible retry due to I2C error

    //initializes the driver
    ISL29023Init(&sensor,
                 sensorI2C_,
                 ISL29023_I2C_ADDRESS,
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
        //sets up 16-bit ADC and 1K range
        ISL29023ReadModifyWrite(&sensor,
                               ISL29023_O_CMD_II,
                               ~((uint8_t)(ISL29023_CMD_II_ADC_RES_M | ISL29023_CMD_II_RANGE_M)),
                               (ISL29023_CMD_II_ADC_RES_16 | ISL29023_CMD_II_RANGE_1K),
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


static void alsReadyCallback(void* pvData, uint_fast8_t status);
static void irReadyCallback(void* pvData, uint_fast8_t status);

static void alsReadyCallback(void* pvData, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        ISL29023ReadModifyWrite(&sensor,
                                ISL29023_O_CMD_I,
                                ~((uint8_t)ISL29023_CMD_I_OP_MODE_M),
                                ISL29023_CMD_I_OP_MODE_ALS_LOW,
                                &alsReadyCallback,
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

static void irReadyCallback(void* pvData, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        ISL29023ReadModifyWrite(&sensor,
                                ISL29023_O_CMD_I,
                                ~((uint8_t)ISL29023_CMD_I_OP_MODE_M),
                                ISL29023_CMD_I_OP_MODE_IR_ONCE,
                                &irReadyCallback,
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
        ISL29023DataRead(&sensor,
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


static void rangeSetCallback(void* pvData, uint_fast8_t status);

static void rangeSetCallback(void* range, uint_fast8_t status)
{

    // See if an error occurred.
    if(status != I2CM_STATUS_SUCCESS){ //if an error occurred
        //retry
        ISL29023ReadModifyWrite(&sensor,
                                ISL29023_O_CMD_II,
                                ~((uint8_t)ISL29023_CMD_II_RANGE_M),
                                *((uint8_t*)range),
                                rangeSetCallback,
                                range);
    }
    else{ //if there weren't any errors
        BaseType_t higherPriorityTaskWoken;
        //set the flag
        xSemaphoreGiveFromISR(rangeSet,
                              &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
    }
}

void isl29023GetData(float* als, float* ir)
{
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    {
        //wait for the sensor to be initialized
        xEventGroupWaitBits(flags,
                            sensorIsInitialized,
                            pdFALSE, //don't clear the flags
                            pdTRUE, //all the flags are required (to be set)
                            portMAX_DELAY); //block indefinitely


        //read als
        while(true){
            //start conversion
            ISL29023ReadModifyWrite(&sensor,
                                    ISL29023_O_CMD_I,
                                    ~((uint8_t)ISL29023_CMD_I_OP_MODE_M),
                                    ISL29023_CMD_I_OP_MODE_ALS_LOW,
                                    &alsReadyCallback,
                                    NULL);

            xSemaphoreTake(conversionStarted, portMAX_DELAY); //wait for the I2C operation to complete

            vTaskDelay(pdMS_TO_TICKS(91)); //wait for the conversion to finish (conversion time: 90 ms)

            //read data
            ISL29023DataRead(&sensor,
                             &dataReadyCallback,
                             NULL);

            xSemaphoreTake(dataReady, portMAX_DELAY); //wait for the I2C operation to complete

            //get the raw data
            uint16_t raw;
            ISL29023DataLightVisibleGetRaw(&sensor,
                                      &raw);

            if((raw > (uint16_t)(UINT16_MAX * .95f)) && (sensor.ui8Range < ISL29023_CMD_II_RANGE_64K)){ //if the value is closer to the high limit of the current range than 5 FS AND it is not in the highest range mode already
                uint8_t higherRange = sensor.ui8Range + 1; //the new range must be set one higher

                ISL29023ReadModifyWrite(&sensor,
                                        ISL29023_O_CMD_II,
                                        ~((uint8_t)ISL29023_CMD_II_RANGE_M),
                                        higherRange,
                                        rangeSetCallback,
                                        &higherRange);

                xSemaphoreTake(rangeSet, portMAX_DELAY); //wait for the I2C operation to complete

                //retry the measurement with the new, higher range by restarting the loop
                //continue;
            }
            else if((raw < (uint16_t)(UINT16_MAX * .05f)) && (sensor.ui8Range > ISL29023_CMD_II_RANGE_1K)){ //if the value is closer to the low limit of the current range than 5 FS AND it is not in the lowest range mode already
                uint8_t lowerRange = sensor.ui8Range - 1; //the new range must be set one lower

                ISL29023ReadModifyWrite(&sensor,
                                        ISL29023_O_CMD_II,
                                        ~((uint8_t)ISL29023_CMD_II_RANGE_M),
                                        lowerRange,
                                        rangeSetCallback,
                                        &lowerRange);

                xSemaphoreTake(rangeSet, portMAX_DELAY); //wait for the I2C operation to complete

                //retry the measurement with the new, lower range by restarting the loop
                //continue;
            }
            else{
                //the range was appropriate, break out the loop
                break;
            }
        }
        //save the ALS lux value to the output variable
        ISL29023DataLightVisibleGetFloat(&sensor,
                                  als);



        //read ir
        while(true){
            //start conversion
            ISL29023ReadModifyWrite(&sensor,
                                    ISL29023_O_CMD_I,
                                    ~((uint8_t)ISL29023_CMD_I_OP_MODE_M),
                                    ISL29023_CMD_I_OP_MODE_IR_ONCE,
                                    &irReadyCallback,
                                    NULL);

            xSemaphoreTake(conversionStarted, portMAX_DELAY); //wait for the I2C operation to complete

            vTaskDelay(pdMS_TO_TICKS(91)); //wait for the conversion to finish (conversion time: 90 ms)

            //read data
            ISL29023DataRead(&sensor,
                             &dataReadyCallback,
                             NULL);

            xSemaphoreTake(dataReady, portMAX_DELAY); //wait for the I2C operation to complete

            //get the raw data
            uint16_t raw;
            ISL29023DataLightIRGetRaw(&sensor,
                                      &raw);

            if((raw > (uint16_t)(UINT16_MAX * .95f)) && (sensor.ui8Range < ISL29023_CMD_II_RANGE_64K)){ //if the value is closer to the high limit of the current range than 5 FS AND it is not in the highest range mode already
                uint8_t higherRange = sensor.ui8Range + 1; //the new range must be set one higher

                ISL29023ReadModifyWrite(&sensor,
                                        ISL29023_O_CMD_II,
                                        ~((uint8_t)ISL29023_CMD_II_RANGE_M),
                                        higherRange,
                                        rangeSetCallback,
                                        &higherRange);

                xSemaphoreTake(rangeSet, portMAX_DELAY); //wait for the I2C operation to complete

                //retry the measurement with the new, higher range by restarting the loop
                //continue;
            }
            else if((raw < (uint16_t)(UINT16_MAX * .05f)) && (sensor.ui8Range > ISL29023_CMD_II_RANGE_1K)){ //if the value is closer to the low limit of the current range than 5 FS AND it is not in the lowest range mode already
                uint8_t lowerRange = sensor.ui8Range - 1; //the new range must be set one lower

                ISL29023ReadModifyWrite(&sensor,
                                        ISL29023_O_CMD_II,
                                        ~((uint8_t)ISL29023_CMD_II_RANGE_M),
                                        lowerRange,
                                        rangeSetCallback,
                                        &lowerRange);

                xSemaphoreTake(rangeSet, portMAX_DELAY); //wait for the I2C operation to complete

                //retry the measurement with the new, lower range by restarting the loop
                //continue;
            }
            else{
                //the range was appropriate, break out the loop
                break;
            }
        }
        //save the IR lux value to the output variable
        ISL29023DataLightIRGetFloat(&sensor,
                                  ir);
    }
    xSemaphoreGive(sensorMutex);
}
