/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "uart.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

UART_DATA uartData;
static uint8_t uart_tx_buf[] = "Hello World. Printf now works on UART 2\r\n";
static uint8_t uart_rx_buf[10];
static enum 
{
    USART_BM_INIT,
    USART_BM_WORKING,
    USART_BM_DONE,
} usartBMState;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
static void USART_Task (void)
{
    switch (usartBMState)
    {
        default:
        case USART_BM_INIT:
        {
            uartData.tx_count = 0;
            uartData.rx_count = 0;
            usartBMState = USART_BM_DONE;
            
            printf("%s",uart_tx_buf );
            break;
        }

        case USART_BM_WORKING:
        {
            if (uartData.tx_count < sizeof(uart_tx_buf)) 
            {
                if(!DRV_USART_TransmitBufferIsFull(uartData.handleUSART0))
                {
                    DRV_USART_WriteByte(uartData.handleUSART0, uart_tx_buf[uartData.tx_count]);
                    uartData.tx_count++;
                }
            }

            if (uartData.rx_count < sizeof(uart_rx_buf)) 
            {
                if(!DRV_USART_ReceiverBufferIsEmpty(uartData.handleUSART0))
                {
                    uart_rx_buf[uartData.rx_count] = DRV_USART_ReadByte(uartData.handleUSART0);
                    uartData.rx_count++;
                }
            }

            /* Have we finished? */
            if (uartData.tx_count == sizeof(uart_tx_buf) && uartData.rx_count == sizeof(uart_rx_buf))
            {
                usartBMState = USART_BM_DONE;
            }
            break;
        }

        case USART_BM_DONE:
        {
            break;
        }
    }
}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_Initialize ( void )

  Remarks:
    See prototype in uart.h.
 */

void UART_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uartData.state = UART_STATE_INIT;

    uartData.handleUSART0 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void UART_Tasks ( void )

  Remarks:
    See prototype in uart.h.
 */

void UART_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( uartData.state )
    {
        /* Application's initial state. */
        case UART_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (uartData.handleUSART0 == DRV_HANDLE_INVALID)
            {
                uartData.handleUSART0 = DRV_USART_Open(UART_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= ( DRV_HANDLE_INVALID != uartData.handleUSART0 );
            }
        
            if (appInitialized)
            {
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                uartData.state = UART_STATE_SERVICE_TASKS;
            }
            break;
        }

        case UART_STATE_SERVICE_TASKS:
        {
			USART_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
