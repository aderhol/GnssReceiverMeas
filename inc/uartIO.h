#ifndef INC_UARTIO_H_
#define INC_UARTIO_H_

#include <stdbool.h>

void uartInit(void);

//available UART ports
typedef enum{
    usb = 0
}UartPort;

void uartPrint(UartPort port, const char string[]); //send a character string to the specified port, the string is guaranteed to be sent as a whole
void uartPrintLn(UartPort port, const char string[]); //same as uartPrint, just appends a new line

#endif /* INC_UARTIO_H_ */
