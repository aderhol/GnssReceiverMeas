//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>

//driver includes
#include <driverlib/sysctl.h>

//user includes
#include <uartIO.h>
#include <utility.h>
#include <eeprom_io.h>



//configurations
const size_t bufferSize_byte = 1024;

static MessageBufferHandle_t commandBuffer;
static TaskHandle_t interpreterTaskHandle;
static void interpreter_task(void* pvParameters);

static void usbRx_callback(char* str);
static void dutB_toPCRx_callback(char* str);

void commandInit(void)
{
    uartSetRxCallback(usb, &usbRx_callback);
    uartSetRxCallback(dutB_toPC, &dutB_toPCRx_callback);

    commandBuffer = xMessageBufferCreate(bufferSize_byte);

    xTaskCreate(&interpreter_task,
                "interpreter task",
                2048,
                NULL,
                1,
                &interpreterTaskHandle);
}

extern size_t xPortGetFreeHeapSize( void );
static void printResetCause(UartPort port, uint32_t resetCause, char* msg);
static void interpreter_task(void* pvParameters)
{
    static char str[512];
    static char msg[512]; //work buffer for string building

    while(true){
        bool OK = 0 != xMessageBufferReceive(commandBuffer,
                                             str,
                                             sizeof(str),
                                             portMAX_DELAY);

        if(!OK){ //if something went wrong
            uartPrintLn(usb, "ERROR: interpreter stopped.");
            vTaskSuspend(xTaskGetCurrentTaskHandle()); //suspend task for ever
        }
        else{
            char* work;
            const char delimStr[] = " ";

            char* command = strtok_r(str, delimStr, &work);
            char* p;
            for (p = command; '\0' != (*p); p++){
                *p = tolower(*p);
            }

            if(strcmp_bool(command, "sitrep")){ //sitrep command
                if(NULL == strtok_r(NULL, delimStr, &work)){ //no other tokens
                    uartPrintLn(usb, "running");
                }
                else{ //incorrect usage
                    const char usage[] = "Error: incorrect usage\r\n"
                            "\tUsage:\r\n"
                            "\t\tsitrep\r\n";
                    uartPrint(usb, usage);
                }
            }
            else if(strcmp_bool(command, "heaprem")){
                if(NULL == strtok_r(NULL, delimStr, &work)){ //no other tokens
                    size_t rem = xPortGetFreeHeapSize();

                    msg[0] = '\0';

                    strcat(msg, "There are ");
                    appendInt64(msg, rem, 15, false);
                    strcat(msg, " free bytes remaining in the heap.");

                    uartPrintLn(usb, msg);
                }
                else{ //incorrect usage
                    const char usage[] = "Error: incorrect usage\r\n"
                            "\tUsage:\r\n"
                            "\t\theapRem\r\n";
                    uartPrint(usb, usage);
                }
            }
            else if(strcmp_bool(command, "resetcause")){
                if(NULL == strtok_r(NULL, delimStr, &work)){ //no other tokens
                    bool crcOK;
                    msg[0] = '\0';
                    const uint32_t eepromAdd_reset_cause = EEPROM_ADD_RESET_CAUSE; //the address of the saved reset cause register
                    const uint32_t eepromAdd_prv_reset_cause_valid = EEPROM_ADD_PREV_RESET_CAUSE_VALID; //the address of the value indicating if the saved previous reset cause is valid (1->valid)
                    const uint32_t eepromAdd_prv_reset_cause = EEPROM_ADD_PREV_RESET_CAUSE; //the address of the saved previous reset cause register

                    strcat(msg, "Reset cause(s): ");

                    uint32_t cause;
                    crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_reset_cause, &cause);
                    printResetCause(usb, cause, msg);
                    if(crcOK){
                        strcat(msg, "\r\n");
                    }
                    else{
                        strcat(msg, " (bad CRC)\r\n");
                    }

                    strcat(msg, "Previous reset's cause(s): ");

                    uint32_t prvCause;
                    crcOK = eepromGetVerifiedValue_ui32_ISR(eepromAdd_prv_reset_cause, &prvCause);
                    printResetCause(usb, prvCause, msg);
                    if(!crcOK){
                        strcat(msg, " (bad CRC)");
                    }

                    uint16_t prvValid;
                    crcOK = eepromGetVerifiedValue_ui16_ISR(eepromAdd_prv_reset_cause_valid, &prvValid);
                    if(!crcOK){
                        strcat(msg, " (valid: bad CRC)");
                    }

                    switch(prvValid){
                    case 1u:
                        break;

                    case 2u:
                        strcat(msg, " (invalid: saved status had bad CRC)");
                        break;

                    default:
                        strcat(msg, " (invalid: ");
                        appendInt64(msg, prvValid, 10, false);
                        strcat(msg, ")");
                        break;
                    }

                    strcat(msg, "\r\n-----\r\n");

                    uartPrintLn(usb, msg);
                }
                else{ //incorrect usage
                    const char usage[] = "Error: incorrect usage\r\n"
                            "\tUsage:\r\n"
                            "\t\tresetCause\r\n";
                    uartPrint(usb, usage);
                }
            }
            else{ //unknown command
                uartPrintLn(usb, "ERROR: unknown command");
            }
        }

        //needs heap2
        /*vTaskGetRunTimeStats(str);
        uartPrint(usb, str);*/
    }
}

static void printResetCause(UartPort port, uint32_t resetCause, char* msg)
{
    bool useComma = false;

    if(resetCause & SYSCTL_CAUSE_POR){
        strcat(msg, "POR");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_EXT){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "EXT");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_HIB){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "HIB");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_HSRVREQ){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "HSRV");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_BOR){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "BOR");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_WDOG0){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "WDOG0");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_WDOG1){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "WDOG1");
        useComma = true;
    }

    if(resetCause & SYSCTL_CAUSE_SW){
        if(useComma){
            strcat(msg, ", ");
        }
        strcat(msg, "SW");
        useComma = true;
    }
}

#define BUFFER_LENGTH (500)
const size_t buffTotLength = BUFFER_LENGTH;

static void usbRx_callback(char* str)
{
    static char buff[BUFFER_LENGTH];

    bool runProcessing;

    if(str[0] != '\0'){ //if the snippet is not an empty string
        size_t strLen = strlen(str); //the length of the snippet (not including the terminating '\0')
        size_t buffLen = strlen(buff); //the length of the buffer

        if((buffLen + strLen + 1) > buffTotLength){ //if the buffer doesn't have enough space
            if((strLen + 1) > buffTotLength){ //if the snippet is just too long (longer than the entire buffer)
                buff[0] = '\0'; //clean the buffer

                runProcessing = false;
            }
            else{ //if the buffer is large enough: the buffer can't possibly be empty in this case
                //buff[0] = '\0'; doesn't needed because of strcpy //clean the buffer

                strcpy(buff, str); //move the snippet into the buffer

                runProcessing = true;
            }
        }
        else{ //if the buffer has enough space
            strcat(buff, str); //copy the snippet into the buffer

            runProcessing = true;
        }

        /**** processing ****/
        if(runProcessing){
            const char* pos;
            const char* head = buff; //the head of the current message
            for(pos = buff + buffLen; (*pos) != '\0'; pos++){ //run until the whole buffer is processed, start with the first unprocessed character
                if((*pos) == '\n'){ //if the end of a message is detected
                    (*((char*)pos)) = '\0'; //terminate the message

                    if(head[0] == '$'){ //if the message is an NMEA message
                        uartPrintLf(dutA_toDut, head); //forward the message to DUT A
                    }
                    else{ //if it is a command
                        size_t totLen = (pos - head) + 1;
                        if(totLen > 1){ //if the command is longer than just the '\n'
                            if('\r' == (*(pos - 1))){ //if the previous char is a carriage return
                                (*((char*)(pos - 1)))  = '\0'; //remove the carriage return

                                if(totLen > 2){ //if it is not an empty command
                                    bool OK = 0 != xMessageBufferSend(commandBuffer,
                                                                      head,
                                                                      ((totLen - 1) * sizeof(char)),
                                                                      0);

                                    if(!OK){ //if the queue was full
                                        uartPrintLn(usb, "ERROR: interpreter is busy");
                                    }
                                }
                            }
                            else{ //if there is no '\r'
                                bool OK = 0 != xMessageBufferSend(commandBuffer,
                                                                  head,
                                                                  (totLen * sizeof(char)),
                                                                  0);

                                if(!OK){ //if the queue was full
                                    uartPrintLn(usb, "ERROR: interpreter is busy");
                                }
                            }
                        }
                    }

                    head = pos + 1;//move the head to the beginning of the next message
                }
            }

            memmove(buff, head, (strlen(head) + 1) * sizeof(char)); //shifts the buffer up
        }
    }

}


static void dutB_toPCRx_callback(char* str)
{
    uartPrint(dutB_toDut, str);
}
