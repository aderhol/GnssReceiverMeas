//std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <semphr.h>
#include <event_groups.h>

//driver includes
#include <driverlib/pin_map.h>  //pin names
#include <driverlib/gpio.h> //GPIO
#include <driverlib/interrupt.h> //interrupts
#include <inc/hw_memmap.h>
#include <inc/hw_i2c.h> //i2c reg definitions
#include <inc/hw_types.h> //reg level access macros
#include <inc/hw_ints.h>    //interrupt defines (eg. INT_UART0)
#include <driverlib/sysctl.h> //System Control
#include <driverlib/i2c.h>

//user includes
#include <senHubI2c.h>


//configurations
static const uint8_t INTERRUPT_PRIORITY = 4;


extern uint32_t SystemCoreClock; //actual system core clock frequency in Hz


static volatile SemaphoreHandle_t mutex; //protects the I2C
static volatile SemaphoreHandle_t ready; //bus operation completed

static bool isIdle(void);

void senHubI2cInit(void)
{
   //enables I2C pins' port
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    }

    //enables I2C
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C7)){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C7)){ //wait for I2C2 to be ready
        }
    }

    //sets pins as I2C pins
    GPIOPinConfigure(GPIO_PD1_I2C7SDA);
    GPIOPinConfigure(GPIO_PD0_I2C7SCL);

    //configures I2C pins (input/output)
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);

    IntPrioritySet(INT_I2C7, INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //sets the interrupt priority (interrupt priority needs to be shifted to the upper 3 bits)

    //
    // Initialize the I2C master module for slow operation
    //
    I2CMasterInitExpClk(I2C7_BASE, SystemCoreClock, false);

    //
    // Enable the I2C interrupt.
    //
    IntEnable(INT_I2C7);
    I2CMasterIntEnableEx(I2C7_BASE, (I2C_MASTER_INT_DATA)); //transaction ready



    mutex = xSemaphoreCreateMutex();

    ready = xSemaphoreCreateBinary();
    xSemaphoreTake(ready, 0);
}


void I2C7_ISR(void)
{
    uint32_t callers = I2CMasterIntStatusEx(I2C7_BASE, true);  //determines what triggered the interrupt
    I2CMasterIntClear(I2C7_BASE);  //clears the interrupt flags

    BaseType_t higherPriorityTaskWoken;
    //signal to task
    xSemaphoreGiveFromISR(ready,
                          &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken); //return to the higher priority (than the currently interrupted) task from the ISR, if one has been woken
}

size_t senHubI2cWrite(uint8_t slaveAddress, uint8_t values[], size_t length)
{
#ifdef DEBUG
    while(0 == length){
    }
#endif

    size_t cnt = 0;

    xSemaphoreTake(mutex, portMAX_DELAY);
    {
        I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, false); //sets the slave address

        if(length == 1){ //single send
            I2CMasterDataPut(I2C7_BASE, values[0]); //sets the value of the byte to be written
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_SINGLE_SEND); //initiate the write

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

            cnt = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE) ? 1 : 0; //cnt remains 0 if there was an error
        }
        else{ //burst
            bool OK;

            I2CMasterDataPut(I2C7_BASE, values[0]); //sets the value of the byte to be written to the first value
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_START); //initiate the write

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

            OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE);

            if(OK){
                cnt++;//one more byte is sent

                size_t i;
                for(i = 1; i < (length - 1); i++){
                    I2CMasterDataPut(I2C7_BASE, values[i]); //sets the value of the byte to be written
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); //send next byte

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                    OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //check for errors

                    if(!OK & !isIdle()){ //there was an error, and the controller isn't already idle
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP); //send error stop

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                        break;
                    }
                    //there was no error
                    cnt++; //one more byte is sent
                }

                if(OK){ //if the loop didn't end due to an error
                    //send last byte
                    I2CMasterDataPut(I2C7_BASE, values[length - 1]); //sets the value of the byte to be written to the last value
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); //send next byte

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                    OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //check for errors

                    if(OK){
                        cnt++; //the last byte was sent
                    }
                }
            }
            else if(!isIdle()){ //if the controller is not already idle
                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP); //send error stop

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
            }
        }
    }
    xSemaphoreGive(mutex);

    return cnt;
}

bool senHubI2cWriteReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t value) //writes a register
{
    uint8_t values[2] = {regAddress, value};

    size_t cnt;
    cnt = senHubI2cWrite(slaveAddress, values, 2);

    bool isOK;
    if(2 == cnt){
        isOK = true;
    }
    else{
        isOK = false;
    }

    return isOK;
}

size_t senHubI2cRead(uint8_t slaveAddress, uint8_t data[], size_t length)
{
#ifdef DEBUG
    while(0 == length){
    }
#endif
    size_t cnt = 0;

    xSemaphoreTake(mutex, portMAX_DELAY);
    {
        I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, true); //sets the slave address

        bool OK;

        if(1 == length){ //single byte read
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //initiates the read

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

            OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

            if(OK){ //if the byte was successfully received
                data[0] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable

                cnt++; //a byte was received
            }
        }
        else{ //burst read
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); //initiates the read

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

            OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

            if(OK){ //if the first byte was successfully received
                data[0] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable

                cnt++; //a byte was received

                size_t i = 0;
                for(i = 1; i < (length - 1); i++){
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); //continues the read

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                    OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                    if(OK){ //if the byte was read successfully
                        data[i] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable
                        cnt++; //a byte was received
                    }
                    else if(!isIdle()){ //if there was an error and the controller isn't yet idles
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP); //send error stop

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                        break;
                    }
                }

                if(OK){ //if the loop completed without an error
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); //finish the read

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                    OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                    if(OK){ //if the last data was received successfully
                        data[length - 1] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable
                        cnt++; //a byte was received
                    }
                }
            }
            else if(!isIdle()){ //if the first byte wasn't received and the controller is not already idle
                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP); //send error stop

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
            }
        }
    }
    xSemaphoreGive(mutex);

    return cnt;
}

size_t senHubI2cReadReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t data[], size_t length)
{
    size_t cnt = 0;

    xSemaphoreTake(mutex, portMAX_DELAY);
    {
        I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, false); //sets the slave address

        bool OK;

        I2CMasterDataPut(I2C7_BASE, regAddress); //sends the registers address
        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_START); //initiates the write

        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

        OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

        if(OK){ //if the register address was sent successfully
            I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, true); //sets the slave address (modifies it to read)

            if(1 == length){ //single byte read
                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //initiates the read

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                if(OK){ //if the byte was successfully received
                    data[0] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable

                    cnt++; //a byte was received
                }
            }
            else{ //burst read
                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); //initiates the read

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                if(OK){ //if the byte was successfully received
                    data[0] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable

                    cnt++; //a byte was received

                    size_t i = 0;
                    for(i = 1; i < (length - 1); i++){
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); //continues the read

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                        OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                        if(OK){ //if the byte was read successfully
                            data[i] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable
                            cnt++; //a byte was received
                        }
                        else if(!isIdle()){ //if the controller is not yet idle
                            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP); //send error stop

                            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                            break;
                        }
                    }

                    if(OK){ //if the loop completed without an error
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); //read the lst byte

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                        OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                        if(OK){ //if the last data was received successfully
                            data[length - 1] = I2CMasterDataGet(I2C7_BASE); //save the data to the output variable
                            cnt++; //a byte was received
                        }
                    }
                }
                else if(!isIdle()){ //if the first byte wasn't received and  the controller isn't yet idle
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP); //send error stop

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
                }
            }
        }
        else if(!isIdle()){ //sending the register address was unsuccessful and the controller isn't yet idle
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP); //send error stop

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
        }
    }
    xSemaphoreGive(mutex);

    return cnt;
}

bool senHubI2cReadModifyWriteReg(uint8_t slaveAddress, uint8_t regAddress, uint8_t mask, uint8_t value)
{
    bool isOK = false;

    xSemaphoreTake(mutex, portMAX_DELAY);
        {
            I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, false); //sets the slave address

            bool OK;

            I2CMasterDataPut(I2C7_BASE, regAddress); //sends the registers address
            I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_START); //initiates the write

            xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

            OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

            if(OK){ //if the register address was sent successfully
                I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, true); //sets the slave address (modifies it to read)

                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); //initiates the read

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                if(OK){ //if the byte was successfully received
                    uint8_t reg = I2CMasterDataGet(I2C7_BASE); //gets the registers value

                    reg = (reg & (~mask)) | value; //calculates the modified value

                    I2CMasterSlaveAddrSet(I2C7_BASE, slaveAddress, false); //sets the slave address (modifies it to write)

                    I2CMasterDataPut(I2C7_BASE, regAddress); //sends the registers address
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_START); //initiates the write

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                    OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                    if(OK){ //if the regAddress was sent successfully
                        I2CMasterDataPut(I2C7_BASE, reg); //sends the registers value
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); //initiates the write

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish

                        OK = (I2CMasterErr(I2C7_BASE) == I2C_MASTER_ERR_NONE); //checks if the transaction was successful

                        if(OK){ //the register was successfully written
                            isOK = true;
                        }
                    }
                    else if(!isIdle()){ //if the regAddress wasn't sent and the controller is not yet idle
                        I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP); //send error stop

                        xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
                    }
                }
                else if(!isIdle()){ //if the first byte wasn't received and the controller is not yet idle
                    I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP); //send error stop

                    xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
                }
            }
            else if(!isIdle()){ //sending the register address was unsuccessful and the controller is not yet idle
                I2CMasterControl(I2C7_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP); //send error stop

                xSemaphoreTake(ready, portMAX_DELAY); //wait for the operation to finish
            }
        }
        xSemaphoreGive(mutex);

        return isOK;
}

static bool isIdle(void)
{
    //
    // Get the raw error state
    //
    uint32_t mcsReg = HWREG(I2C7_BASE + I2C_O_MCS);

    return mcsReg & ((uint32_t)I2C_MCS_IDLE);
}
