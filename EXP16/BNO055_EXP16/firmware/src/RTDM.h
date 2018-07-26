/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    rtdm.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _RTDM_H
#define _RTDM_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define RTDM_RXBUFFERSIZE	32		// This is the buffer size used by RTDM to handle messaages 
#define RTDM_MAX_XMIT_LEN   0x1000	//This the size in bytes of the max num of bytes allowed in 
									//the RTDM protocol Frame
#define RTDM_POLLING		YES		// This defines the mode that RTDM will be operating in 
									//user's application. If it is YES then the user should place the 
									//RTDM_ProcessMsgs()	function in the main loop. 
									//In order to make sure that the messages are being preoccessed
									// it is recommended that the main loop always polls this 
/****************************************************/
/* DEFINITIONS FOR  RTDM captured variables */
/****************************************************/


     
    #define DATA_BUFFER_SIZE 50  //Size in 16-bit Words of the snap */
                                  // the value depends on the dsPIC mem
    #define SNAPDELAY	5 // In number of PWM Interrupts
    #define	DEBUG_VARIABLE1		ParkParm.Ia*1000    //Current in mA
    #define	DEBUG_VARIABLE2		ParkParm.Ib*1000    //Current in mA
    #define DEBUG_VARIABLE3		(EstimParm.qVelEstim*60)/(2*M_PI*NOPOLESPAIRS) //Translating Speed in Rad/sec to  RPM
    #define DEBUG_VARIABLE4		EstimParm.qRho*57.28//Converting angle from radians to degrees => 360/2*PI = 57.28

/****************************************************/			


/*+++++++++++++++++++++++++++++++ RTDM Variables ++++++++++++++++++++++++++++++++++++++++*/
/* Received data is stored in array RTDMRxBuffer  */
extern unsigned char RTDMRxBuffer[RTDM_RXBUFFERSIZE];
extern unsigned char * RTDMRxBufferLoLimit;
extern unsigned char * RTDMRxBufferHiLimit;
extern unsigned char * RTDMRxBufferIndex;
extern unsigned char * RTDMRxBufferStartMsgPointer;
extern unsigned char * RTDMRxBufferEndMsgPointer;
/* Data to be transmitted using UART communication module */
extern const unsigned char RTDMTxdata[] ;
extern const unsigned char RTDMSanityCheckOK[]; 
extern const unsigned char RTDMWriteMemoryOK[];
extern const unsigned char RTDMErrorIllegalFunction[];
extern unsigned char RTDMErrorFrame[] ;
extern DRV_HANDLE RTDM_UART_HANDLE;
/* Temp variables used to calculate the CRC16*/
extern unsigned int RTDMcrcTemp,RTDMcrcTempH,RTDMcrcTempL;
typedef struct DMCIFlags{
    		    unsigned Recorder : 1;	// Flag needs to be set to start buffering data
				unsigned StartStop : 1;
    			unsigned unused : 14;  
    } DMCIFLAGS;
/*Structure enclosing the RTDM flags*/
typedef struct RTDMFlags {
			unsigned MessageReceived  :		1;
			unsigned TransmitNow	  :		1;
			unsigned unused :		14;     
		}	RTDMFLAGS; 

    
extern    DMCIFLAGS DMCIFlags;
extern 	  RTDMFLAGS	RTDMFlags;

extern int SpeedReference;
extern signed short int SnapShotDelay;
extern    int RecorderBuffer1[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer2[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer3[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer4[DATA_BUFFER_SIZE] __attribute__ ((aligned));


// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	RTDM_STATE_INIT=0,
	RTDM_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} RTDM_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    RTDM_STATES state;

    /* TODO: Define any additional data used by the application. */

} RTDM_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void RTDM_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    RTDM_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void RTDM_Initialize ( void );


/*******************************************************************************
  Function:
    void RTDM_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    RTDM_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void RTDM_Tasks( void );

/**********************  RTDM FUNCTIONS **************************/
void RTDM_RecBufferUpdate();
int RTDM_RTDM_ProcessMsgs();
int RTDM_RTDM_Start();
unsigned int RTDM_RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);
void  RTDM_RTDMRXInterruptTasks ();
void APP_MC_DBG_Init(void);
void APP_MC_DBG_SnapStart(void);
void RTDM_SnapUpdate(void);

#endif /* _RTDM_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

