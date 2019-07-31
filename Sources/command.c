//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <message_buffer.h>

//driver includes

//user includes
#include <uartIO.h>



//configurations
const size_t bufferSize_byte = 1024;

static MessageBufferHandle_t commandBuffer;

static void usbRx_callback(char* str);

void commandInit(void)
{
    uartSetRxCallback(usb, &usbRx_callback);

    commandBuffer = xMessageBufferCreate(bufferSize_byte);
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
                        /*size_t totLen = (pos - head) + 1;
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
                        }*/
                    }

                    head = pos + 1;//move the head to the beginning of the next message
                }
            }

            memmove(buff, head, (strlen(head) + 1) * sizeof(char)); //shifts the buffer up
        }
    }

}
