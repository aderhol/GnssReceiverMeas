//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stream_buffer.h>

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
static const uint8_t RX_PRIORITY = 1;
static const uint16_t TX_QUEUE_LENGTH = 1024;
static const uint16_t RX_STREAM_LENGTH = 1024;
static const uint8_t INTERRUPT_PRIORITY = 4;
static const char newLine[] = "\r\n";
#define UART_FIFO_LENGTH (16)

extern uint32_t SystemCoreClock;

typedef struct{
    uint32_t base;
    TaskHandle_t Tx_task;
    TaskHandle_t Rx_task;
    QueueHandle_t Tx_queue;
    StreamBufferHandle_t  Rx_stream;
    SemaphoreHandle_t Tx_mutex;
    UartRx_Callback Rx_callback;
}Port;
static const Port* getPort(UartPort portNum);
static void fillTxFifo(const Port* port);
static bool uartPutch(const Port* port, char ch); //sends a character to the specified port, returns true if successful and false otherwise

//usb
static Port uart0 = {UART0_BASE, NULL, NULL, NULL, NULL, NULL, NULL};
static void uart0Init(uint32_t baud);
static void uart0Tx_task(void* pvParameters);
static void uart0Rx_task(void* pvParameters);

//dutA
static Port uart3 = {UART3_BASE, NULL, NULL, NULL, NULL, NULL, NULL};
static void uart3Init(uint32_t baud);
static void uart3Tx_task(void* pvParameters);
static void uart3Rx_task(void* pvParameters);

//dutB
static Port uart6 = {UART6_BASE, NULL, NULL, NULL, NULL, NULL, NULL};
static void uart6Init(uint32_t baud);
static void uart6Tx_task(void* pvParameters);
static void uart6Rx_task(void* pvParameters);


void uartInit(void)
{
    uart0Init(1000000);

    uart3Init(9600);

    uart6Init(9600);
}

/**************************************************    UART0   **************************************************/

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
    UARTIntEnable(UART0_BASE, (UART_INT_RT | UART_INT_RX | UART_INT_TX)); //unmasks Receive Timeout, receive FIFO, and transmit FIFO interrupts
    IntPrioritySet(INT_UART0, INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //sets the interrupt priority (interrupt priority needs to be shifted to the upper 3 bits)

    //Tx task creation
    xTaskCreate(&uart0Tx_task,
                "uart0TxTask",
                256,
                NULL, //parameter to task
                TX_PRIORITY, //priority
                &(uart0.Tx_task)); //task handle

    uart0.Tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char)); //Tx queue creation
    uart0.Tx_mutex = xSemaphoreCreateMutex();

    //Rx task creation
    xTaskCreate(&uart0Rx_task,
                "uart0RxTask",
                512,
                NULL, //parameter to task
                RX_PRIORITY, //priority
                &(uart0.Rx_task)); //task handle

    uart0.Rx_stream = xStreamBufferCreate(RX_STREAM_LENGTH, 1); //Rx stream creation (trigger level: 1 byte)
}

void uart0_ISR(void)
{
    uint32_t callers = UARTIntStatus(UART0_BASE, true);  //determines what triggered the interrupt
    UARTIntClear(UART0_BASE, callers);  //clears the interrupt flags

    BaseType_t higherPriorityTaskWoken = pdFALSE; //variable indicating whether the notification woke a task with a higher priority than the current task (which was interrupted)

    if(0 != (callers & (uint32_t)UART_INT_TX)){ //if the Tx FIFO is almost empty

        vTaskNotifyGiveFromISR(uart0.Tx_task, &higherPriorityTaskWoken); //send notification for the task, so that it refills the FIFO from the queue
    }

    if(0 != (callers & (uint32_t)(UART_INT_RX | UART_INT_RT))){ //if the Tx FIFO is half full, or it timed out
        char buff[UART_FIFO_LENGTH + 1]; //buffer for the bytes read from the FIFO, longer by 1 in case a byte becomes available during processing
        size_t cnt; //number of bytes received
        int32_t ch = 0;

        for(cnt = 0; ((ch = UARTCharGetNonBlocking(uart0.base)) != -1) && (cnt < (UART_FIFO_LENGTH + 1)); cnt++){ //loop while the receive FIFO is emptied or the buffer is full
            buff[cnt] = (char)ch; //save the read char to the buffer
         }

        //send the received data to the stream
        xStreamBufferSendFromISR(uart0.Rx_stream,
                                 &buff,
                                 (sizeof(char) * cnt),
                                 &higherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken); //request context switch if a higher priority task has been woken
}

//sends the characters form the queue to the UART periphery, if notified
static void uart0Tx_task(void* pvParameters)
{
    while(true){
        fillTxFifo(&uart0); //fills UART0's Tx FIFO from the queue

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //waits for notification
    }
}

#define UART0_RX_CALLBACK_BUFFER_LENGTH (20) //maximum amount of chars that can be put into the buffer
//calls the callback function with the freshly received bytes
static void uart0Rx_task(void* pvParameters)
{
    static char buff[UART0_RX_CALLBACK_BUFFER_LENGTH + 1]; //buffer for the received data
    while(true){

        //wait for data to be received
        size_t cnt = xStreamBufferReceive(uart0.Rx_stream,
                                          &buff,
                                          (sizeof(char) * UART0_RX_CALLBACK_BUFFER_LENGTH),
                                          portMAX_DELAY);

        buff[cnt] = '\0'; //terminate the string

        if(NULL != uart0.Rx_callback){ //if there is a callback set
            uart0.Rx_callback(buff); //call the callback and pass the buffer
        }
    }
}

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    UART0   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



/**************************************************    UART3   **************************************************/

static void uart3Init(uint32_t baud)
{
    //enable UART pins
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    }

    //enables UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART3)){} //wait for UART to be ready

    //sets pins as UART pins
    GPIOPinConfigure(GPIO_PA4_U3RX);
    GPIOPinConfigure(GPIO_PA5_U3TX);

    //configures UART pins (input/output)
    GPIOPinTypeUART(GPIO_PORTA_BASE, (uint8_t)GPIO_PIN_4 | (uint8_t)GPIO_PIN_5);

    //configure the UART modules
    //UART3: baud-rate: baud, 8-bit, 1 STOP-bit, no parity-bit
    //AND enables UART
    UARTConfigSetExpClk(UART3_BASE, SystemCoreClock, baud, ((uint32_t)UART_CONFIG_WLEN_8 | (uint32_t)UART_CONFIG_STOP_ONE | (uint32_t)UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART3_BASE); //enables FIFOs
    UARTFIFOLevelSet(UART3_BASE, UART_FIFO_TX2_8, UART_FIFO_RX4_8);
    IntEnable(INT_UART3); //enables the UART3 interrupt
    UARTIntEnable(UART3_BASE, (UART_INT_RT | UART_INT_RX | UART_INT_TX)); //unmasks Receive Timeout, receive FIFO, and transmit FIFO interrupts
    IntPrioritySet(INT_UART3, INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //sets the interrupt priority (interrupt priority needs to be shifted to the upper 3 bits)

    //Tx task creation
    xTaskCreate(&uart3Tx_task,
                "uart3TxTask",
                256,
                NULL, //parameter to task
                TX_PRIORITY, //priority
                &(uart3.Tx_task)); //task handle

    uart3.Tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char)); //Tx queue creation
    uart3.Tx_mutex = xSemaphoreCreateMutex();

    //Rx task creation
    xTaskCreate(&uart3Rx_task,
                "uart3RxTask",
                1024,
                NULL, //parameter to task
                3, //priority
                &(uart3.Rx_task)); //task handle

    uart3.Rx_stream = xStreamBufferCreate(RX_STREAM_LENGTH, 1); //Rx stream creation (trigger level: 1 byte)
}

void uart3_ISR(void)
{
    uint32_t callers = UARTIntStatus(UART3_BASE, true);  //determines what triggered the interrupt
    UARTIntClear(UART3_BASE, callers);  //clears the interrupt flags

    BaseType_t higherPriorityTaskWoken = pdFALSE; //variable indicating whether the notification woke a task with a higher priority than the current task (which was interrupted)

    if(0 != (callers & (uint32_t)UART_INT_TX)){ //if the Tx FIFO is almost empty

        vTaskNotifyGiveFromISR(uart3.Tx_task, &higherPriorityTaskWoken); //send notification for the task, so that it refills the FIFO from the queue
    }

    if(0 != (callers & (uint32_t)(UART_INT_RX | UART_INT_RT))){ //if the Tx FIFO is half full, or it timed out
        char buff[UART_FIFO_LENGTH + 1]; //buffer for the bytes read from the FIFO, longer by 1 in case a byte becomes available during processing
        size_t cnt; //number of bytes received
        int32_t ch = 0;

        for(cnt = 0; ((ch = UARTCharGetNonBlocking(uart3.base)) != -1) && (cnt < (UART_FIFO_LENGTH + 1)); cnt++){ //loop while the receive FIFO is emptied or the buffer is full
            buff[cnt] = (char)ch; //save the read char to the buffer
         }

        //send the received data to the stream
        xStreamBufferSendFromISR(uart3.Rx_stream,
                                 &buff,
                                 (sizeof(char) * cnt),
                                 &higherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken); //request context switch if a higher priority task has been woken
}

//sends the characters form the queue to the UART periphery, if notified
static void uart3Tx_task(void* pvParameters)
{
    while(true){
        fillTxFifo(&uart3); //fills UART3's Tx FIFO from the queue

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //waits for notification
    }
}

#define UART3_RX_CALLBACK_BUFFER_LENGTH (20) //maximum amount of chars that can be put into the buffer
//calls the callback function with the freshly received bytes
static void uart3Rx_task(void* pvParameters)
{
    static char buff[UART3_RX_CALLBACK_BUFFER_LENGTH + 1]; //buffer for the received data
    while(true){

        //wait for data to be received
        size_t cnt = xStreamBufferReceive(uart3.Rx_stream,
                                          &buff,
                                          (sizeof(char) * UART3_RX_CALLBACK_BUFFER_LENGTH),
                                          portMAX_DELAY);

        buff[cnt] = '\0'; //terminate the string

        if(NULL != uart3.Rx_callback){ //if there is a callback set
            uart3.Rx_callback(buff); //call the callback and pass the buffer
        }
    }
}

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    UART3   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



/**************************************************    UART6   **************************************************/

static void uart6Init(uint32_t baud)
{
    //enable UART pins
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    }

    //enables UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6)){} //wait for UART to be ready

    //sets pins as UART pins
    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);

    //configures UART pins (input/output)
    GPIOPinTypeUART(GPIO_PORTP_BASE, (uint8_t)GPIO_PIN_0 | (uint8_t)GPIO_PIN_1);

    //configure the UART modules
    //UART6: baud-rate: baud, 8-bit, 1 STOP-bit, no parity-bit
    //AND enables UART
    UARTConfigSetExpClk(UART6_BASE, SystemCoreClock, baud, ((uint32_t)UART_CONFIG_WLEN_8 | (uint32_t)UART_CONFIG_STOP_ONE | (uint32_t)UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART6_BASE); //enables FIFOs
    UARTFIFOLevelSet(UART6_BASE, UART_FIFO_TX2_8, UART_FIFO_RX4_8);
    IntEnable(INT_UART6); //enables the UART6 interrupt
    UARTIntEnable(UART6_BASE, (UART_INT_RT | UART_INT_RX | UART_INT_TX)); //unmasks Receive Timeout, receive FIFO, and transmit FIFO interrupts
    IntPrioritySet(INT_UART6, INTERRUPT_PRIORITY << (8 - configPRIO_BITS)); //sets the interrupt priority (interrupt priority needs to be shifted to the upper 3 bits)

    //Tx task creation
    xTaskCreate(&uart6Tx_task,
                "uart6TxTask",
                256,
                NULL, //parameter to task
                TX_PRIORITY, //priority
                &(uart6.Tx_task)); //task handle

    uart6.Tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(char)); //Tx queue creation
    uart6.Tx_mutex = xSemaphoreCreateMutex();

    //Rx task creation
    xTaskCreate(&uart6Rx_task,
                "uart6RxTask",
                1024,
                NULL, //parameter to task
                3, //priority
                &(uart6.Rx_task)); //task handle

    uart6.Rx_stream = xStreamBufferCreate(RX_STREAM_LENGTH, 1); //Rx stream creation (trigger level: 1 byte)
}

void uart6_ISR(void)
{
    uint32_t callers = UARTIntStatus(UART6_BASE, true);  //determines what triggered the interrupt
    UARTIntClear(UART6_BASE, callers);  //clears the interrupt flags

    BaseType_t higherPriorityTaskWoken = pdFALSE; //variable indicating whether the notification woke a task with a higher priority than the current task (which was interrupted)

    if(0 != (callers & (uint32_t)UART_INT_TX)){ //if the Tx FIFO is almost empty

        vTaskNotifyGiveFromISR(uart6.Tx_task, &higherPriorityTaskWoken); //send notification for the task, so that it refills the FIFO from the queue
    }

    if(0 != (callers & (uint32_t)(UART_INT_RX | UART_INT_RT))){ //if the Tx FIFO is half full, or it timed out
        char buff[UART_FIFO_LENGTH + 1]; //buffer for the bytes read from the FIFO, longer by 1 in case a byte becomes available during processing
        size_t cnt; //number of bytes received
        int32_t ch = 0;

        for(cnt = 0; ((ch = UARTCharGetNonBlocking(uart6.base)) != -1) && (cnt < (UART_FIFO_LENGTH + 1)); cnt++){ //loop while the receive FIFO is emptied or the buffer is full
            buff[cnt] = (char)ch; //save the read char to the buffer
         }

        //send the received data to the stream
        xStreamBufferSendFromISR(uart6.Rx_stream,
                                 &buff,
                                 (sizeof(char) * cnt),
                                 &higherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken); //request context switch if a higher priority task has been woken
}

//sends the characters form the queue to the UART periphery, if notified
static void uart6Tx_task(void* pvParameters)
{
    while(true){
        fillTxFifo(&uart6); //fills UART6's Tx FIFO from the queue

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //waits for notification
    }
}

#define UART6_RX_CALLBACK_BUFFER_LENGTH (20) //maximum amount of chars that can be put into the buffer
//calls the callback function with the freshly received bytes
static void uart6Rx_task(void* pvParameters)
{
    static char buff[UART6_RX_CALLBACK_BUFFER_LENGTH + 1]; //buffer for the received data
    while(true){

        //wait for data to be received
        size_t cnt = xStreamBufferReceive(uart6.Rx_stream,
                                          &buff,
                                          (sizeof(char) * UART6_RX_CALLBACK_BUFFER_LENGTH),
                                          portMAX_DELAY);

        buff[cnt] = '\0'; //terminate the string

        if(NULL != uart6.Rx_callback){ //if there is a callback set
            uart6.Rx_callback(buff); //call the callback and pass the buffer
        }
    }
}

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    UART6   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

//gets the port struct
static const Port* getPort(UartPort portNum)
{
    const Port* port;
    switch(portNum){
        case (UartPort)0:
            port = &uart0;
            break;

        case (UartPort)3:
            port = &uart3;
            break;

        case (UartPort)6:
            port = &uart6;
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
    while(xQueuePeek(port->Tx_queue, &ch, 0) == pdTRUE){ //loop while the queue is not empty
        if(UARTCharPutNonBlocking(port->base, ch)){ //if putting the character into the FIFO has succeeded
            xQueueReceive(port->Tx_queue, &ch, 0); //remove the peeked character from the queue
        }
        else{ //if the FIFO is full
            break; //break from the loop
        }
    }
}

static bool uartPutch(const Port* port, char ch)
{
#ifdef DEBUG
    bool res = (xQueueSendToBack(port->Tx_queue, &ch, 0) == pdTRUE); //sends the character to the port
#else
    bool res = (xQueueSendToBack(port->Tx_queue, &ch, portMAX_DELAY) == pdTRUE); //sends the character to the port
#endif

    if(res && (uxQueueMessagesWaiting(port->Tx_queue) == 1)){ //if character was sent successfully to the queue and there is only one character on the queue
        xTaskNotifyGive(port->Tx_task); //notify the Tx task, to make sure the character gets sent even if the Tx FIFO is below the threshold and thus it won't generate an interrupt
    }

    return res;
}

void uartPrint(UartPort port, const char string[])
{
    const Port* port_ = getPort(port); //get the port

    size_t length = strlen(string);

    xSemaphoreTake(port_->Tx_mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        if(uxQueueSpacesAvailable(port_->Tx_queue) >= length){ //if there is enough space in the queue
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
    xSemaphoreGive(port_->Tx_mutex);
}

#ifdef DEBUG
void uartPrintLn(UartPort port, const char string[])
{
    const Port* port_ = getPort(port); //get the port

    size_t length = strlen(string);
    size_t newLineLength = strlen(newLine);

    xSemaphoreTake(port_->Tx_mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        if(uxQueueSpacesAvailable(port_->Tx_queue) >= (length + newLineLength)){ //if there is enough space in the queue
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
    xSemaphoreGive(port_->Tx_mutex);
}
#else
void uartPrintLn(UartPort port, const char* string)
{
    const Port* port_ = getPort(port); //get the port

    xSemaphoreTake(port_->Tx_mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        size_t i;

        //send all characters to the queue
        for(i = 0; '\0' != string[i]; i++)
            uartPutch(port_, string[i]);

        //send the new line
        for(i = 0; '\0' != newLine[i]; i++)
            uartPutch(port_, newLine[i]);
    }
    xSemaphoreGive(port_->Tx_mutex);
}
#endif

#ifdef DEBUG
void uartPrintLf(UartPort port, const char string[])
{
    const Port* port_ = getPort(port); //get the port

    size_t length = strlen(string);

    xSemaphoreTake(port_->Tx_mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        if(uxQueueSpacesAvailable(port_->Tx_queue) >= (length + 1)){ //if there is enough space in the queue
            size_t i;

            //send all characters to the queue
            for(i = 0; i < length; i++){
                uartPutch(port_, string[i]);
            }

            uartPutch(port_, '\n');
        }
        else{
            while(true){} //the buffer (queue) is not large enough
        }
    }
    xSemaphoreGive(port_->Tx_mutex);
}
#else
void uartPrintLf(UartPort port, const char* string)
{
    const Port* port_ = getPort(port); //get the port

    xSemaphoreTake(port_->Tx_mutex, portMAX_DELAY); //take the mutex, so that the string can be sent out continuously
    {
        size_t i;

        //send all characters to the queue
        for(i = 0; '\0' != string[i]; i++)
            uartPutch(port_, string[i]);

        uartPutch(port_, '\n');
    }
    xSemaphoreGive(port_->Tx_mutex);
}
#endif


//NOT thread safe
void uartSetRxCallback(UartPort port, UartRx_Callback callback)
{
    Port* port_ = (Port*)getPort(port);
    port_->Rx_callback = callback;
}

