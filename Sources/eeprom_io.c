//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>

//driver includes
#include "inc/hw_memmap.h"
#include "driverlib/eeprom.h"
#include "driverlib/crc.h"
#include "driverlib/sysctl.h" //System Control
#include <driverlib/interrupt.h> //interrupts

//user includes
#include <eeprom_io.h>



//configurations




void eepromIoInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));

    //checks if EEPROM is healthy
    if(EEPROMInit() == EEPROM_INIT_ERROR)
        while(1);
}

void eepromWriteWithVerification_ui16_ISR(uint32_t address, uint16_t data)
{
    uint32_t bytes[2] = {(uint32_t)((uint8_t)data), (uint32_t)((uint8_t)(data >> 8))};
    uint32_t dataOut;

    if(address % 4)
        while(1);

    bool isWriteSuccessful;

    bool enableInerrupts = !IntMasterDisable();
    {
        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_8BIT | CRC_CFG_TYPE_P1021));
        dataOut = (CRCDataProcess(CCM0_BASE, bytes, 2, false) << 16) | data; //[CRC-data]
        isWriteSuccessful = !EEPROMProgram(&dataOut, address, 4);
    }
    if(enableInerrupts)
        IntMasterEnable();

    while(!isWriteSuccessful);
}

bool eepromGetVerifiedValue_ui16_ISR(uint32_t address, uint16_t *data_out)
{
    uint32_t data;
    if(address % 4)
        while(1);

    uint32_t crcRes;

    bool enableInerrupts = !IntMasterDisable();
    {
        EEPROMRead(&data, address, 4);

        *data_out = (uint16_t) data;
        uint32_t bytes[2] = {(uint32_t)((uint8_t)data), (uint32_t)((uint8_t)(data >> 8))};

        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_8BIT | CRC_CFG_TYPE_P1021));
        crcRes = CRCDataProcess(CCM0_BASE, bytes, 2, false);

    }
    if(enableInerrupts)
        IntMasterEnable();

    return (crcRes == (data >> 16));
}


void eepromWriteWithVerification_ui32_ISR(uint32_t address, uint32_t data)
{
    if(address % 4)
        while(1);

    uint32_t dataOut[2] = {data, /*CRC*/};

    bool isWriteSuccessful;

    bool enableInerrupts = !IntMasterDisable();
    {
        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_32BIT | CRC_CFG_TYPE_P1021));
        dataOut[1] = CRCDataProcess(CCM0_BASE, &data, 1, false);
        isWriteSuccessful = !EEPROMProgram(&dataOut[0], address, (4 * 2));
    }
    if(enableInerrupts)
        IntMasterEnable();

    while(!isWriteSuccessful);
}

bool eepromGetVerifiedValue_ui32_ISR(uint32_t address, uint32_t *data_out)
{
    uint32_t data[2]; //{data, CRC}
    if(address % 4)
        while(1);

    bool enableInerrupts = !IntMasterDisable();
    {
        EEPROMRead(&data[0], address, (4 * 2));
    }
    if(enableInerrupts)
        IntMasterEnable();

    *data_out = data[0];

    uint32_t crcRes;
    enableInerrupts = !IntMasterDisable();
    {
        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_32BIT | CRC_CFG_TYPE_P1021));
        crcRes = CRCDataProcess(CCM0_BASE, &data[0], 1, false);
    }
    if(enableInerrupts)
        IntMasterEnable();

    return (((uint16_t)crcRes) == ((uint16_t)data[1]));
}



void eepromWriteWithVerification_ui8A_ISR(uint32_t address, const uint8_t* data, size_t length, uint32_t* nxtAddress)
{
    if(address % 4)
        while(1);

    if(nxtAddress != NULL){
        *nxtAddress = address + 4 * (((length + 3) / 4) + 1);
    }

    bool isWriteSuccessful;
    uint32_t crcResult;

    bool enableInerrupts = !IntMasterDisable();
    {
        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_8BIT | CRC_CFG_TYPE_P1021));

        size_t i;
        for(i = 0; i < length; i++){
            CRCDataWrite(CCM0_BASE,
                         data[i]);
        }

        crcResult = CRCResultRead(CCM0_BASE,
                                  false);

        isWriteSuccessful = !EEPROMProgram((uint32_t*)data, address, ((length / 4) * 4));

        if(length % 4 != 0){
            uint32_t wrd = 0;

            size_t i;
            for(i = 0; i < (length % 4); i++){
                wrd |= ((uint32_t)data[((length / 4) * 4) + i]) << (i*8);
            }

            isWriteSuccessful &= !EEPROMProgram(&wrd, address + ((length / 4) * 4), 4);
            isWriteSuccessful &= !EEPROMProgram(&crcResult, address + ((length / 4) * 4) + 4, 4);
        }
        else
        {
            isWriteSuccessful &= !EEPROMProgram(&crcResult, address + ((length / 4) * 4), 4);
        }

    }
    if(enableInerrupts)
        IntMasterEnable();

    while(!isWriteSuccessful);
}

bool eepromGetVerifiedValue_ui8A_ISR(uint32_t address, uint8_t* data_out, size_t length, uint32_t* nxtAddress)
{
    if(address % 4)
        while(1);

    if(nxtAddress != NULL){
        *nxtAddress = address + 4 * (((length + 3) / 4) + 1);
    }

    uint32_t crcRes, crc;

    bool enableInerrupts = !IntMasterDisable();
    {
        EEPROMRead((uint32_t*)data_out, address, (length / 4) * 4);

        if(length % 4 != 0){
            uint32_t wrd;
            EEPROMRead(&wrd, address + (length / 4) * 4, 4);

            size_t i;
            for(i = 0; i < (length % 4); i++){
                data_out[(length / 4) * 4 + i] = (uint8_t)(wrd >> (i * 8));
            }

            EEPROMRead(&crc, address + (length / 4) * 4 + 4, 4);
        }
        else{
            EEPROMRead(&crc, address + (length / 4) * 4, 4);
        }

        CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_0 | CRC_CFG_SIZE_8BIT | CRC_CFG_TYPE_P1021));
        size_t i;
        for(i = 0; i < length; i++){
            CRCDataWrite(CCM0_BASE,
                         data_out[i]);
        }

        crcRes = CRCResultRead(CCM0_BASE,
                                  false);
    }
    if(enableInerrupts)
        IntMasterEnable();

    return (((uint16_t)crcRes) == ((uint16_t)crc));
}
