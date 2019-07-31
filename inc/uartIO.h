#ifndef INC_UARTIO_H_
#define INC_UARTIO_H_

#include <stdbool.h>

void uartInit(void);

//available UART ports
typedef enum{
    usb = 0,
    dutA_toDut = 3,
    dutB_toDut = 6,
    dutB_toPC = 4
}UartPort;

typedef void (*UartRx_Callback)(char*);

void uartPrint(UartPort port, const char string[]); //send a character string to the specified port, the string is guaranteed to be sent as a whole
void uartPrintLn(UartPort port, const char string[]); //same as uartPrint, just appends a new line
void uartPrintLf(UartPort port, const char string[]); //same as uartPrintLn, but appends only the \n

void uartSetRxCallback(UartPort port, UartRx_Callback callback); //sets the Rx callback function, not thread safe

#endif /* INC_UARTIO_H_ */
