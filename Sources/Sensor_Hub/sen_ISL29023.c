///std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>
#include <task.h>

//driver includes

//user includes
#include <senHubI2c.h>
#include <sen_ISL29023.h>



//configurations
static const uint8_t ISL29023_I2C_ADDRESS = 0x44;

static const uint8_t ISL29023_CMD_I_REG_ADDRESS = 0x00;
static const uint8_t ISL29023_MODE_M = 0x07 << 5;
static const uint8_t ISL29023_MODE_POWER_OFF = 0x00 << 5;
static const uint8_t ISL29023_MODE_ALS_ONCE = 0x01 << 5;
static const uint8_t ISL29023_MODE_IR_ONCE = 0x02 << 5;
static const uint8_t ISL29023_MODE_ALS_CONT = 0x05 << 5;
static const uint8_t ISL29023_MODE_IR_CONT = 0x06 << 5;

static const uint8_t ISL29023_CMD_II_REG_ADDRESS = 0x01;
static const uint8_t ISL29023_RANGE_M = 0x03;
static const uint8_t ISL29023_RANGE_1K = 0x00;
static const uint8_t ISL29023_RANGE_4K = 0x01;
static const uint8_t ISL29023_RANGE_16K = 0x03;
static const uint8_t ISL29023_RANGE_64K = 0x03;
static const uint8_t ISL29023_RES_M = 0x03 << 2;
static const uint8_t ISL29023_RES_4BIT = 0x03 << 2;
static const uint8_t ISL29023_RES_8BIT = 0x02 << 2;
static const uint8_t ISL29023_RES_12BIT = 0x01 << 2;
static const uint8_t ISL29023_RES_16BIT = 0x00 << 2;

static const uint8_t ISL29023_DATA_REG_START_ADDRESS = 0x02;

static volatile EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t sensorIsInitialized = ((EventBits_t) 1) << 0;

static volatile SemaphoreHandle_t sensorMutex;

static void init_task(void* pvParameters);

static volatile bool initError; //true if initialization was unsuccessful

static bool measAls(float* als); //measures the ALS

void isl29023Init(void)
{
    TaskHandle_t initTaskHandle;
    xTaskCreate(&init_task,
                "ISL29023_initTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                (configMAX_PRIORITIES - 1),
                &initTaskHandle);

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, sensorIsInitialized);

    sensorMutex = xSemaphoreCreateMutex();
}

static void init_task(void* pvParameters)
{
    bool OK;

    int_fast8_t failCnt;

    for(failCnt = 0; failCnt < 5; failCnt++){
        //set 1K range and 16 bit resolution
        OK = senHubI2cReadModifyWriteReg(ISL29023_I2C_ADDRESS, ISL29023_CMD_II_REG_ADDRESS,
                                         ISL29023_RANGE_M | ISL29023_RES_M
                                         , ISL29023_RANGE_1K | ISL29023_RES_16BIT
                                         );

        if(OK){ //if reset command was sent successfully
            break; //go to next step
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
        }
    }

    if(OK){ //if the initialization was successful
        initError = false;
    }
    else{ //if the initialization was unsuccessful
        initError = true;
    }

    xEventGroupSetBits(flags, sensorIsInitialized); //signal that the initialization is complete

    vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
}

typedef enum{
    r1K = 0,
    r4K = 1,
    r16K = 2,
    r64K = 3
}AlsRange;

static bool measAls(float* als)
{
    static AlsRange alsRange = r1K; //the current range setting of the sensor (0: 1K, 1: 4K, 2: 16K, 3: 64K), initially 1K

    bool measOK;

    while(true){
        bool OK;
        int_fast8_t failCnt;

        for(failCnt = 0; failCnt < 5; failCnt++){
            //start the conversion (ALS)
            OK = senHubI2cReadModifyWriteReg(ISL29023_I2C_ADDRESS, ISL29023_CMD_I_REG_ADDRESS,
                                             ISL29023_MODE_M,
                                             ISL29023_MODE_ALS_ONCE
                                             );

            if(OK){ //if the command was sent successfully
                break; //go to next step
            }
            else { //if there was an I2C error
                vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
            }
        }

        if(OK){ //if the measurement has been started
            const uint_fast8_t alsConversionTime_ms = 95; //needs to be at least 90 ms, according to the data sheet (in 16-bit mode)
            vTaskDelay(pdMS_TO_TICKS(alsConversionTime_ms)); //wait for the conversion to complete

            uint8_t dataReg[2]; //{LSB, MSB}
            //read data (ALS)
            for(failCnt = 0; failCnt < 5; failCnt++){
                const size_t dataRegByteCnt = 2;
                size_t dataRegCnt = senHubI2cReadReg(ISL29023_I2C_ADDRESS, ISL29023_DATA_REG_START_ADDRESS, dataReg, dataRegByteCnt); //read the data registers

                OK = (dataRegByteCnt == dataRegCnt); //was the I2C read operation successful?

                if(OK){ //if the measured value was read successfully
                    break; //go to next step
                }
                else { //if there was an I2C error
                    vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                }
            }

            if(OK){ //if the measured value was read successfully
                uint16_t raw = ((uint16_t)dataReg[1]) << 8 | ((uint16_t) dataReg[0]); //concatenate the bytes to get the intensity

                if((raw > (uint16_t)(UINT16_MAX * .95f)) && (alsRange < r64K)){ //if the value is closer to the high limit of the current range than 5 FS AND it is not in the highest range mode already
                    for(failCnt = 0; failCnt < 5; failCnt++){
                        //increase the range by one
                        OK = senHubI2cReadModifyWriteReg(ISL29023_I2C_ADDRESS, ISL29023_CMD_II_REG_ADDRESS,
                                                         ISL29023_RANGE_M,
                                                         ((uint8_t)(alsRange + 1))
                                                         );

                        if(OK){ //if the command was sent successfully
                            break; //go to next step
                        }
                        else { //if there was an I2C error
                            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                        }
                    }

                    if(OK){ //if the range was set
                        alsRange += 1; //increase the stored range to the newly set value

                        //continue;
                    }
                    else{ //if the range couldn't be set
                        measOK = false;
                        break; //break from the loop
                    }
                }
                else if((raw < (uint16_t)(UINT16_MAX * .05f)) && (alsRange > r1K)){ //if the value is closer to the low limit of the current range than 5 FS AND it is not in the lowest range mode already
                    for(failCnt = 0; failCnt < 5; failCnt++){
                        //decrease the range by one
                        OK = senHubI2cReadModifyWriteReg(ISL29023_I2C_ADDRESS, ISL29023_CMD_II_REG_ADDRESS,
                                                         ISL29023_RANGE_M,
                                                         ((uint8_t)(alsRange - 1))
                                                         );

                        if(OK){ //if the command was sent successfully
                            break; //go to next step
                        }
                        else { //if there was an I2C error
                            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                        }
                    }

                    if(OK){ //if the range was set
                        alsRange -= 1; //decrease the stored range to the newly set value

                        //continue;
                    }
                    else{ //if the range couldn't be set
                        measOK = false;
                        break; //break from the loop
                    }
                }
                else{//if the range was appropriate
                    float maxLux;
                    switch(alsRange){ //select the maximum LUX value for the current range
                    case r1K:
                        maxLux = 1000;
                        break;

                    case r4K:
                        maxLux = 4000;
                        break;

                    case r16K:
                        maxLux = 16000;
                        break;

                    case r64K:
                        maxLux = 64000;
                        break;

                    default:
                        while(true){
                            //fault
                        }
                        break;
                    }

                    //calculate the ALS in LUX and save the result to the output variable
                    *als = ((float)raw) * (maxLux / 65536.0f); //65536 = 2^16 = 2^n, in 16 bit mode n := 16

                    measOK = true; //the measurement was successful
                    break; //break from the loop
                }
            }
            else{ //if the data couldn't be fetched form the sensor
                measOK = false;
                break;
            }
        }
        else{ //if the ALS measurement couldn't be started
            measOK = false;
            break; //break from the loop
        }
    }

    return measOK;
}

bool isl29023GetData(float* als, uint16_t* ir)
{
    bool measOK;

    //wait for the sensor to be initialized
    xEventGroupWaitBits(flags,
                        sensorIsInitialized,
                        pdFALSE, //don't clear the flags
                        pdTRUE, //all the flags are required (to be set)
                        portMAX_DELAY); //block indefinitely

    if(!initError){
        bool OK;
        int_fast8_t failCnt;


        xSemaphoreTake(sensorMutex, portMAX_DELAY); //take the mutex protecting the sensor
        {
            OK = measAls(als); //measure ALS

            if(OK){ //if the ALS was measured successfully
                //measure IR

                for(failCnt = 0; failCnt < 5; failCnt++){
                    //start the conversion (IR)
                    OK = senHubI2cReadModifyWriteReg(ISL29023_I2C_ADDRESS, ISL29023_CMD_I_REG_ADDRESS,
                                                     ISL29023_MODE_M,
                                                     ISL29023_MODE_IR_ONCE
                                                     );

                    if(OK){ //if the command was sent successfully
                        break; //go to next step
                    }
                    else { //if there was an I2C error
                        vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                    }
                }

                if(OK){ //if the measurement has been started
                    const uint_fast8_t irConversionTime_ms = 95; //needs to be at least 90 ms, according to the data sheet (in 16-bit mode)
                    vTaskDelay(pdMS_TO_TICKS(irConversionTime_ms)); //wait for the conversion to complete

                    uint8_t dataReg[2]; //{LSB, MSB}
                    //read data (IR)
                    for(failCnt = 0; failCnt < 5; failCnt++){
                        const size_t dataRegByteCnt = 2;
                        size_t dataRegCnt = senHubI2cReadReg(ISL29023_I2C_ADDRESS, ISL29023_DATA_REG_START_ADDRESS, dataReg, dataRegByteCnt); //read the data registers

                        OK = (dataRegByteCnt == dataRegCnt); //was the I2C read operation successful?

                        if(OK){ //if the measured value was read successfully
                            break; //go to next step
                        }
                        else { //if there was an I2C error
                            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                        }
                    }

                    if(OK){ //if the measured value was read successfully
                        *ir = ((uint16_t)dataReg[1]) << 8 | ((uint16_t) dataReg[0]); //concatenate the bytes to get the intensity and save it to the output variable

                        measOK = true;
                    }
                    else{ //if the measurement couldn't be read
                        measOK = false;
                    }
                }
                else{ //if the conversion couldn't be initiated
                    measOK = false;
                }
            }
            else{ //if the ALS couldn't be measured
                measOK = false;
            }
        }
        xSemaphoreGive(sensorMutex); //release the mutex on the sensor
    }
    else{ //if the sensor initialization has failed
        measOK = false;
    }

    return measOK;
}
