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
#include <sen_sht21.h>



//configurations
static const uint8_t SHT21_I2C_ADDRESS = 0x40;

static const uint8_t SHT21_START_TEMPERATURE_MEASUREMENT_NO_HOLD = 0xF3;
static const uint8_t SHT21_START_HUMIDITY_MEASUREMENT_NO_HOLD = 0xF5;
static const uint8_t SHT21_START_TEMPERATURE_MEASUREMENT_HOLD = 0xE3;
static const uint8_t SHT21_START_HUMIDITY_MEASUREMENT_HOLD = 0xE5;

static const uint8_t SHT21_USER_REG_ADDRESS_WRITE = 0xE6;
static const uint8_t SHT21_USER_REG_ADDRESS_READ = 0xE7;
static const uint8_t SHT21_RESOLUTION_M = 0x81;
static const uint8_t SHT21_RESOLUTION_T14_RH12 = 0x00;
static const uint8_t SHT21_RESOLUTION_T12_RH8 = 0x01;
static const uint8_t SHT21_RESOLUTION_T13_RH10 = 0x80;
static const uint8_t SHT21_RESOLUTION_T11_RH11 = 0x81;
static const uint8_t SHT21_HEATER_M = 0x04;
static const uint8_t SHT21_HEATER_ON = 0x04;
static const uint8_t SHT21_HEATER_OFF = 0x00;

static const uint8_t SHT21_RESET = 0xFE;

static volatile EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t sensorIsInitialized = ((EventBits_t) 1) << 0;

static volatile SemaphoreHandle_t sensorMutex;

static void init_task_sht21(void* pvParameters);

static volatile bool initError; //true if initialization was unsuccessful

void sht21Init()
{
    TaskHandle_t initTaskHandle;
    xTaskCreate(&init_task_sht21,
                "SHT21_initTask",
                512,
                NULL,
                (configMAX_PRIORITIES - 1),
                &initTaskHandle);

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, sensorIsInitialized);

    sensorMutex = xSemaphoreCreateMutex();
}

static void init_task_sht21(void* pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(16)); //wait for the sensor to start up after power on

    bool OK;
    int_fast8_t failCnt;

    for(failCnt = 0; failCnt < 5; failCnt++){
        OK = senHubI2cWrite(SHT21_I2C_ADDRESS, &SHT21_RESET, 1); //soft reset

        if(OK){ //if the reset command was sent successfully
            break; //go to next step
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
        }
    }

    if(OK){ //if the reset command was sent successfully
        vTaskDelay(pdMS_TO_TICKS(16)); //wait for start up after reset (per datasheet maximum 15 ms)

        uint8_t userReg;
        for(failCnt = 0; failCnt < 5; failCnt++){
            size_t userRegReadCnt = senHubI2cReadReg(SHT21_I2C_ADDRESS, SHT21_USER_REG_ADDRESS_READ, &userReg, 1); //read the user reg

            OK = (1 == userRegReadCnt); //was the read successful?

            if(OK){ //if the user reg was read
                break; //go to next step
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
            }
        }

        if(OK){ //the user reg was read successfully
            userReg = userReg & ~(SHT21_RESOLUTION_M | SHT21_HEATER_M); //delete fields to be modified

            //sets the resolution to be 14 bit for temperature measurement
            //12 bit for relative humidity measurement
            //and turns the heater off
            userReg = userReg | (SHT21_RESOLUTION_T14_RH12 | SHT21_HEATER_OFF);

            for(failCnt = 0; failCnt < 5; failCnt++){
                OK = senHubI2cWriteReg(SHT21_I2C_ADDRESS, SHT21_USER_REG_ADDRESS_WRITE, userReg); //write the user register with the new setup

                if(OK){ //if the write was successful
                    break; //go to next step
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                }
            }

            if(OK){ //if the sensor was successfully set up
                initError = false;
            }
            else{ //if the sensor couldn't be set up with the new user register value
                initError = true;
            }
        }
        else{ //the user reg couldn't be read
            initError = true;
        }
    }
    else{ //the sensor couldn't be reset
        initError = true;
    }

    xEventGroupSetBits(flags, sensorIsInitialized); //signal that the initialization is complete

    vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
}

bool sht21GetData(float* temperature_C, float* relHum_percentage)
{
    //wait for the sensor to be initialized
    xEventGroupWaitBits(flags,
                        sensorIsInitialized,
                        pdFALSE, //don't clear the flags
                        pdTRUE, //all the flags are required (to be set)
                        portMAX_DELAY); //block indefinitely

    bool measOK;

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    {
        bool OK;
        int_fast8_t failCnt;

        //start the conversion (temperature)
        for(failCnt = 0; failCnt < 5; failCnt++){
            size_t writeCnt = senHubI2cWrite(SHT21_I2C_ADDRESS, &SHT21_START_TEMPERATURE_MEASUREMENT_HOLD, 1);

            OK = (1 == writeCnt); //was the measurement successfully started?

            if(OK){ //if the measurement was successfully started
                break; //go to next step
            }
            else { //if there was an I2C error
                vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
            }
        }

        if(OK){ //if the temperature measurement was started successfully
            vTaskDelay(pdMS_TO_TICKS(66)); //wait for the conversion to finish (typically needs 66 ms to complete in 14 bit mode, per datasheet)

            uint8_t rawTempBytes[2]; //{MSB, LSB}
            for(failCnt = 0; failCnt < 5; failCnt++){
                const size_t rawTempCnt = 2;
                size_t readCnt = senHubI2cRead(SHT21_I2C_ADDRESS, rawTempBytes, rawTempCnt); //read the temperature data, if in 'hold' mode; will block until the data is available

                OK = (rawTempCnt == readCnt); //was the measurement successful?

                if(OK){ //if the data was read successfully
                    break; //go to next step
                }
                else { //if there was an I2C error
                    vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                }
            }

            if(OK){ //the raw temperature data was read successfully
                uint16_t rawTemp = (((uint16_t)rawTempBytes[0]) << 8) | ((uint16_t)(rawTempBytes[1] & ~((uint16_t)0x03))); //concatenate the bytes and delete the status bits

                *temperature_C = -46.85f + ((175.72f / 65536) * rawTemp); //calculate the temperature and save the value to the output variable

                //start the conversion (humidity)
                for(failCnt = 0; failCnt < 5; failCnt++){
                    size_t writeCnt = senHubI2cWrite(SHT21_I2C_ADDRESS, &SHT21_START_HUMIDITY_MEASUREMENT_HOLD, 1);

                    OK = (1 == writeCnt); //was the measurement successfully started?

                    if(OK){ //if the measurement was successfully started
                        break; //go to next step
                    }
                    else { //if there was an I2C error
                        vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                    }
                }

                if(OK){ //if the humidity measurement was started successfully
                    vTaskDelay(pdMS_TO_TICKS(22)); //wait for the conversion to finish (typically needs 22 ms to complete in 12 bit mode, per datasheet)

                    uint8_t rawHumBytes[2]; //{MSB, LSB}
                    for(failCnt = 0; failCnt < 5; failCnt++){
                        const size_t rawHumCnt = 2;
                        size_t readCnt = senHubI2cRead(SHT21_I2C_ADDRESS, rawHumBytes, rawHumCnt); //read the humidity data, if in 'hold' mode; will block until the data is available

                        OK = (rawHumCnt == readCnt); //was the measurement successful?

                        if(OK){ //if the data was read successfully
                            break; //go to next step
                        }
                        else { //if there was an I2C error
                            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                        }
                    }

                    if(OK){ //the raw humidity data was read successfully
                        uint16_t rawHum = (((uint16_t)rawHumBytes[0]) << 8) | ((uint16_t)(rawHumBytes[1] & ~((uint16_t)0x03))); //concatenate the bytes and delete the status bits

                        *relHum_percentage = -6.0f + ((125.0f / 65536) * rawHum); //calculate the humidity and save the value to the output variable

                        measOK = true;
                    }
                    else{ //the humidity data couldn't be read
                        measOK = false;
                    }
                }
                else{ //if the humidity couldn't be measured
                    measOK = false;
                }
            }
            else{ //the temperature data couldn't be read
                measOK = false;
            }
        }
        else{ //if the temperature couldn't be measured
            measOK = false;
        }
    }
    xSemaphoreGive(sensorMutex);

    return measOK;
}
