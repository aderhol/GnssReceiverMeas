//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

//driver includes
#include <driverlib/uart.h>
#include <driverlib/pin_map.h>  //pin names
#include <driverlib/gpio.h> //GPIO
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>    //interrupt defines (eg. INT_UART0)
#include <driverlib/sysctl.h> //System Control
#include <driverlib/interrupt.h> //interrupt API

//user includes
#include <uartIO.h>



//configurations
static const uint8_t TX_PRIORITY = 1;
static const uint16_t TX_QUEUE_LENGTH = 1024;
static const uint8_t INTERRUPT_PRIORITY = 4;
static const char newLine[] = "\r\n";

extern uint32_t SystemCoreClock;

typedef struct{
    uint32_t base;
    TaskHandle_t task;
    QueueHandle_t queue;
    SemaphoreHandle_t mutex;
}Port;
static const Port* getPort(UartPort portNum);
static void fillTxFifo(const Port* port);
static bool uartPutch(const Port* port, char ch); //sends a character to the specified port, returns true if successful and false otherwise

static void uart0Tx_task(void* pvParameters);
static Port uart0 = {UART0_BASE, NULL, NULL, NULL};

static void uart0Init(uint32_t baud);
void uartInit(void)
{
    uart0Init(1000000);
}

static void uart0Init(uint32_t baud)
{
    //enable UART pins
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    }

    //enables UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){} //wait for UART to be ready

    //sets pins as UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //configures UART pins (input/output)
    GPIOPinTypeUART(GPIO_PORTA_BASE, (uint8_t)GPIO_PIN_0 | (uint8_t)GPIO_PIN_1);

    //configure the UART modules
    //UART0: baud-rate: baud, 8-bit, 1 STOP-bit, no parity-bit
    //AND enables UART
    UARTConfigSetExpClk(UART0_BASE, SystemCoreClock, baud, ((uint32_t)UART_CONFIG_WLEN_8 | (uint32_t)UART_CONFIG_STOP_ONE | (uint32_t)UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART0_BASE); //enables FIFOs
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX2_8, UART_FIFO_RX4_8);
    IntEnable(INT_UART0); //enables the UART0 interrupt
    UARTIntEnable(UART0_BASE, (/*UART_INT_RT | UART_INT_RX |*/ UART_INT_TX)); //unmasks Receive Timeout, receive FIFO, and transmit FIFO interrupts
    IntPrioritySet(INT_UART0, INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //sets the interrupt priority (interrupt priority needs to be shifted to the upper 3 bits)

    //Tx task creation
    xTaskCreate(&uart0Tx_task,
                "uart0TxTask",
                256,
                NULL, //parameter to task
                TX_PRIORITY, //priority
                &(uart0.task)); //task handle

    uart0.queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char)); //Tx queue creation
    uart0.mutex = xSemaphoreCreateMutex();
}

void uart0_ISR(void)
{
    uint32_t callers = UARTIntStatus(UART0_BASE, true);  //determines what triggered the interrupt
    UARTIntClear(UART0_BASE, callers);  //clears the interrupt flags

    if(0 != (callers & (uint32_t)UART_INT_TX)){ //if the Tx FIFO is almost empty
        BaseType_t higherPriorityTaskWoken = pdFALSE; //variable indicating whether the notification woke a task with a higher priority than the current task (which was interrupted)
        vTaskNotifyGiveFromISR(uart0.task, &higherPriorityTaskWoken); //send notification for the task, so that it refills the FIFO from the queue
        portYIELD_FROM_ISR(higherPriorityTaskWoken); //request context switch if a higher priority task has been woken
    }
}

//sends the characters form the queue to the UART periphery, if notified
static void uart0Tx_task(void* pvParameters)
{
    while(true){
        fillTxFifo(&uart0); //fills UART0's Tx FIFO from the queue

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //waits for notification
    }
}

//gets the port struct
static const Port* getPort(UartPort portNum)
{
    const Port* port;
    switch(portNum){
        case (UartPort)0:
            port = &uart0;
            break;

        default:
            while(true)
            {
            }
            break;
    }

    return port;
}

//fills the Tx FIFO of the UART periphery from the corresponding queue
static void fillTxFifo(const Port* port)
{
    char ch = 0; //the received character from the queue
    //filling the FIFO
    while(xQueuePeek(port->queue, &ch, 0) == pdTRUE){ //loop while the queue is not empty
        if(UARTCharPutNonBlocking(port->base, ch)){ //if putting the character into the FIFO has succeeded
            xQueueReceive(port->queue, &ch, 0); //remove the peeked character from the queue
        }
        else{ //if the FIFO is full
            break; //break from the loop
        }
    }
}

static bool uartPutch(const Port* port, char ch)
{
#ifdef DEBUG
    bool res = (xQueueSendToBack(port->queue, &ch, 0) == pdTRUE); //sends the character to the port
#else
    bool res = (xQueueSendToBack(port->queue, &ch, portMAX_DELAY) == pdTRUE); //sends the character to the port
#endif

    if(res && (uxQueueMessagesWaiting(port->queue) == 1)){ //if character was sent successfully to the queue and there is only one character on the queue
        xTaskNotifyGive(port->task); //notify the Tx task, to make sure the character gets sent even if the Tx FIFO is below the threshold and thus it won't generate an interrupt
    }

    return res;
}

void uartPrint(UartPort port, const char string[])
{
    const Port* port_ = getPort(port); //get the port

    size_t length = strlen(string);

    xSemaphoreTake(port_->mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        if(uxQueueSpacesAvailable(port_->queue) >= length){ //if there is enough space in the queue
            //send all characters to the queue
            size_t i;
            for(i = 0; i < length; i++){
                uartPutch(port_, string[i]);
            }
        }
        else{
            while(true){} //the buffer (queue) is not large enough
        }
    }
    xSemaphoreGive(port_->mutex);
}

#ifdef DEBUG
void uartPrintLn(UartPort port, const char string[])
{
    const Port* port_ = getPort(port); //get the port

    size_t length = strlen(string);
    size_t newLineLength = strlen(newLine);

    xSemaphoreTake(port_->mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        if(uxQueueSpacesAvailable(port_->queue) >= (length + newLineLength)){ //if there is enough space in the queue
            size_t i;

            //send all characters to the queue
            for(i = 0; i < length; i++){
                uartPutch(port_, string[i]);
            }

            //send the new line
            for(i = 0; i < newLineLength; i++){
                uartPutch(port_, newLine[i]);
            }
        }
        else{
            while(true){} //the buffer (queue) is not large enough
        }
    }
    xSemaphoreGive(port_->mutex);
}
#else
void uartPrintLn(UartPort port, const char* string)
{
    const Port* port_ = getPort(port); //get the port

    xSemaphoreTake(port_->mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        size_t i;

        //send all characters to the queue
        for(i = 0; '\0' != string[i]; i++)
            uartPutch(port_, string[i]);

        //send the new line
        for(i = 0; '\0' != newLine[i]; i++)
            uartPutch(port_, newLine[i]);
    }
    xSemaphoreGive(port_->mutex);
}
#endif
