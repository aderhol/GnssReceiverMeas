//std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <event_groups.h>
#include <semphr.h>

//driver includes

//user includes
#include <senHubI2c.h>
#include <sen_bmp180.h>



//configurations
static const uint8_t BMP180_I2C_ADDRESS = 0x77;

static const uint8_t BMP180_RESET_REG_ADDRESS = 0xE0;
static const uint8_t BMP180_RESET_COMMAND = 0xB6;

static const uint8_t BMP180_CONTROL_REG_ADDRESS = 0xF4;
static const uint8_t BMP180_CONTROL_START_TEMP_COMMAND = 0x2E;
static const uint8_t BMP180_CONTROL_START_PRESSURE_COMMAND = 0xF4; //oss = 3 -> 8 sample average
static const uint8_t BMP180_CONTROL_SCO_BIT = 0x20;

static const uint8_t BMP180_CALIBDAT_REG_START_ADDRESS = 0xAA;

static const uint8_t BMP180_DAT_REG_START_ADDRESS = 0xF6;

static const uint8_t oss = 3; //0, 1, 2, or 3 //over sampling setup

static volatile EventGroupHandle_t flags; //<0>: sensor is initialized, <1>: data is ready
static const EventBits_t sensorIsInitialized = ((EventBits_t) 1) << 0;
static void init_task_bmp180(void* pvParameters);

static volatile SemaphoreHandle_t sensorMutex;

static volatile bool initError; //true if initialization was unsuccessful

//calibration parameters
static volatile int16_t AC1, AC2 , AC3;
static volatile uint16_t AC4, AC5, AC6;
static volatile int16_t B1, B2;
static volatile int16_t MB, MC, MD; //MB isn't used, it is a calibration data though, idk...,  the datasheet sucks

void bmp180Init(void)
{
    TaskHandle_t initTaskHandle;
    xTaskCreate(&init_task_bmp180,
                "bmp180_initTask",
                512,
                NULL,
                (configMAX_PRIORITIES - 1),
                &initTaskHandle);

    flags = xEventGroupCreate();
    xEventGroupClearBits(flags, sensorIsInitialized);

    sensorMutex = xSemaphoreCreateMutex();
}


static void init_task_bmp180(void* pvParameters)
{
    bool OK;

    int_fast8_t failCnt;

    for(failCnt = 0; failCnt < 5; failCnt++){
        OK = senHubI2cWriteReg(BMP180_I2C_ADDRESS, BMP180_RESET_REG_ADDRESS, BMP180_RESET_COMMAND); //soft reset

        if(OK){ //if reset command was sent successfully
            break; //go to next step
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
        }
    }

    if(OK){ //if the reset command was sent successfully
        vTaskDelay(pdMS_TO_TICKS(20)); //wait for start up after reset (empirically needs to be at least 10 ms)

        uint8_t rawCalib[22]; //raw calibration data
        for(failCnt = 0; failCnt < 5; failCnt++){
            const size_t numOfcalibBytes = 22;
            size_t calibCnt = senHubI2cReadReg(BMP180_I2C_ADDRESS, BMP180_CALIBDAT_REG_START_ADDRESS, rawCalib, numOfcalibBytes); //read the bytes
            OK = (numOfcalibBytes == calibCnt); //all 22 byte was read successfully?

            if(OK){ //if all the calibration data was read
                break; //go to next step
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
            }
        }

        if(OK){ //if the calibration data was read successfully
            initError = false;

            //concatenate bytes to get the calibration values
            AC1 = ((int16_t)(((int16_t)((int8_t)(rawCalib[(0 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(0 * 2) + 1]);
            AC2 = ((int16_t)(((int16_t)((int8_t)(rawCalib[(1 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(1 * 2) + 1]);
            AC3 = ((int16_t)(((int16_t)((int8_t)(rawCalib[(2 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(2 * 2) + 1]);
            AC4 = ((uint16_t)(((uint16_t)rawCalib[(3 * 2) + 0]) << 8)) | ((uint16_t)rawCalib[(3 * 2) + 1]);
            AC5 = ((uint16_t)(((uint16_t)rawCalib[(4 * 2) + 0]) << 8)) | ((uint16_t)rawCalib[(4 * 2) + 1]);
            AC6 = ((uint16_t)(((uint16_t)rawCalib[(5 * 2) + 0]) << 8)) | ((uint16_t)rawCalib[(5 * 2) + 1]);
            B1 = ((int16_t)(((int16_t)((int8_t)(rawCalib[(6 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(6 * 2) + 1]);
            B2 = ((int16_t)(((int16_t)((int8_t)(rawCalib[(7 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(7 * 2) + 1]);
            MB = ((int16_t)(((int16_t)((int8_t)(rawCalib[(8 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(8 * 2) + 1]); //is not used (ik, ... idk, just datasheet things...)
            MC = ((int16_t)(((int16_t)((int8_t)(rawCalib[(9 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(9 * 2) + 1]);
            MD = ((int16_t)(((int16_t)((int8_t)(rawCalib[(10 * 2) + 0]))) << 8)) | ((int16_t)rawCalib[(10 * 2) + 1]);
        }
        else{ //if the calibration data couldn't be read
            initError = true;
        }
    }
    else{ //if the resetting failed
        initError = true;
    }

    xEventGroupSetBits(flags, sensorIsInitialized); //signal that the initialization is complete

    vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
}


bool bmp180GetData(float* temperature_C, float* pressure_Pa)
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
            for(failCnt = 0; failCnt < 5; failCnt++){
                OK = senHubI2cWriteReg(BMP180_I2C_ADDRESS, BMP180_CONTROL_REG_ADDRESS, BMP180_CONTROL_START_TEMP_COMMAND); //start temperature measurement

                if(OK){ //if the measurement was started successfully
                    break; //go to next step
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                }
            }

            if(OK){ //if the temperature measurement was initiated successfully
                const int_fast8_t tempConversionTime = 4; //empirical value
                vTaskDelay(pdMS_TO_TICKS(tempConversionTime)); //wait for conversion

                //wait for the conversion to be ready
                for(failCnt = 0; failCnt < 5; failCnt++){
                    uint8_t cntrRegSco_temp;
                    OK = (1 == senHubI2cReadReg(BMP180_I2C_ADDRESS, BMP180_CONTROL_REG_ADDRESS, &cntrRegSco_temp, 1)); //get the SCO bit (0 if the conversion has finished)

                    OK = OK && (0 ==(cntrRegSco_temp & BMP180_CONTROL_SCO_BIT)); //if the I2C operation was successful AND the SCO bit is 0

                    if(OK){ //if the conversion is ready
                        break; //go to next step
                    }
                    else{
                        vTaskDelay(pdMS_TO_TICKS(1)); //wait a little
                    }
                }

                if(OK){ //if the conversion of the temperature was successfully completed
                    uint8_t rawTemp[2];
                    for(failCnt = 0; failCnt < 5; failCnt++){
                        const size_t tempDatByteCnt = 2;
                        size_t rawTempCnt = senHubI2cReadReg(BMP180_I2C_ADDRESS, BMP180_DAT_REG_START_ADDRESS, rawTemp, tempDatByteCnt); //get raw temperature measurement
                        OK = (tempDatByteCnt == rawTempCnt);

                        if(OK){ //if all the temperature data was read
                            break; //go to next step
                        }
                        else
                        {
                            vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                        }
                    }

                    if(OK){ //if the raw temperature data was read successfully
                        //concatenate the temperature data bytes
                        int64_t UT = ((uint32_t)((uint32_t)(((uint32_t)rawTemp[0]) << 8))
                                | ((uint32_t)rawTemp[1]));

                        for(failCnt = 0; failCnt < 5; failCnt++){
                            OK = senHubI2cWriteReg(BMP180_I2C_ADDRESS, BMP180_CONTROL_REG_ADDRESS, BMP180_CONTROL_START_PRESSURE_COMMAND); //start pressure measurement

                            if(OK){ //if the measurement was started successfully
                                break; //go to next step
                            }
                            else
                            {
                                vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                            }
                        }

                        if(OK){ //if the pressure conversion was started successfully
                            //determines conversion time depending on the over-sampling setup
                            int_fast8_t pressureConversionTime;
                            switch(oss){
                            case 0:
                                pressureConversionTime = 4;
                                break;

                            case 1:
                                pressureConversionTime = 7;
                                break;

                            case 2:
                                pressureConversionTime = 13;
                                break;

                            case 3:
                                pressureConversionTime = 17; //empirical value
                                break;

                            default:
                                while(true){ //invalid oss value
                                }
                                break;
                            }

                            vTaskDelay(pdMS_TO_TICKS(pressureConversionTime)); //wait for the conversion to complete

                            //wait for the conversion to be ready
                            for(failCnt = 0; failCnt < 5; failCnt++){
                                uint8_t cntrRegSco_pressure;
                                OK = (1 == senHubI2cReadReg(BMP180_I2C_ADDRESS, BMP180_CONTROL_REG_ADDRESS, &cntrRegSco_pressure, 1)); //get the SCO bit (0 if the conversion has finished)

                                OK = OK && (0 ==(cntrRegSco_pressure & BMP180_CONTROL_SCO_BIT)); //if the I2C operation was successful AND the SCO bit is 0

                                if(OK){ //if the conversion is ready
                                    break; //go to next step
                                }
                                else{
                                    vTaskDelay(pdMS_TO_TICKS(1)); //wait a little
                                }
                            }

                            if(OK){ //if the conversion was successfully completed
                                uint8_t rawPressure[3];
                                for(failCnt = 0; failCnt < 5; failCnt++){
                                    const size_t pressureDatByteCnt = 3;
                                    size_t rawPressureCnt = senHubI2cReadReg(BMP180_I2C_ADDRESS, BMP180_DAT_REG_START_ADDRESS, rawPressure, pressureDatByteCnt); //get raw pressure data

                                    OK = (pressureDatByteCnt == rawPressureCnt);

                                    if(OK){ //if all the pressure data was read
                                        break; //go to next step
                                    }
                                    else
                                    {
                                        vTaskDelay(pdMS_TO_TICKS(1)); //wait a little and then try again
                                    }
                                }

                                if(OK){ //if the raw pressure data was read successfully
                                    measOK = true; //the measurement was successful (the I2C is no lonnger needed)

                                    //concatenate bytes
                                    int64_t UP = (uint32_t)(((((uint32_t)(((uint32_t)rawPressure[0]) << 16))
                                            | ((uint32_t)(((uint32_t)rawPressure[1]) << 8))
                                            | ((uint32_t)rawPressure[2])) >> (8 - oss)));


                                    //calculating compensated valued (temperature and pressure) based on the data sheet
                                    //(I have no idea what's happening, code was written based on the datasheet and the Bosch written driver)

                                    int64_t X1, X2, X3; //intermediate variables

                                    //calculating the temperature
                                    X1 = ((UT - AC6) * AC5) / (((int64_t)1u) << 15);
                                    X2 = (MC * (((int64_t)1u) << 11)) / (X1 + MD);
                                    int64_t B5 = X1 + X2;
                                    *temperature_C = 0.1f * ((B5 + 8u) / (((int64_t)1u) << 4)); //saving the temperature value to the output variable

                                    //calculating the compensated pressure
                                    int64_t B6 = B5 - 4000;
                                    X1 = (B2 * ((B6 * B6) / (((int64_t)1u) << 12))) / (((int64_t)1u) << 11);
                                    X2 = (AC2 * B6) / (((int64_t)1u) << 11);
                                    X3 = X1 + X2;
                                    int64_t B3 = (((((int64_t)AC1 * 4) + X3) << oss) + 2) / 4;
                                    X1 = (AC3 * B6) / (((int64_t)1u) << 13);
                                    X2 = (B1 * ((B6 * B6) / (((int64_t)1u) << 12))) / (((int64_t)1u) << 16);
                                    X3 = ((X1 + X2) + 2) / (((int64_t)1u) << 2);
                                    uint64_t B4 = (AC4 * ((uint64_t)(X3 + 32768))) / (((uint64_t)1u) << 15);
                                    uint64_t B7 = (((uint64_t)UP) - B3) * (50000u >> oss);
                                    uint64_t p = (B7 < 0x80000000) ? ((B7 * 2) / B4) : ((B7 / B4) * 2);
                                    X1 = (p / (((uint64_t)1u) << 8)) * (p / (((uint64_t)1u) << 8));
                                    X1 = (X1 * 3038) / (((int64_t)1u) << 16);
                                    X2 = ((int64_t)((-7357 * p) / (((int64_t)1u) << 16))); //I know, it changes sign, what can you do...
                                    *pressure_Pa = (int32_t)(p + ((X1 + X2 + 3791) / (((int64_t)1u) << 4))); //saving the pressure value to the output variable
                                }
                                else{ //if the raw pressure data couldn't be read
                                    measOK = false;
                                }
                            }
                            else{ //if the completion of the pressure measurement couldn't be verified
                                measOK = false;
                            }
                        }
                        else{ //if the pressure conversion couldn't be started
                            measOK = false;
                        }
                    }
                    else{ //if the raw temperature data couldn't be read
                        measOK = false;
                    }
                }
                else{ //if the completion of the temperature measurement couldn't be verified
                    measOK = false;
                }
            }
            else{ //if the temperature measurement couldn't be initiated
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
