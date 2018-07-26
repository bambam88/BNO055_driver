/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "../../../../bno055.h"
#include <math.h>

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
/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
BNO055DEV bno055;


APP_DATA appData;
// This is the string to write to the slave device.  Be aware that the double quotes adds a null byte at the end of the string.
// So, writing "Hello World!" actually transmits 13 bytes.
uint8_t appWriteString[1024] = {0};
uint8_t appReadString[1024] = {0};
const I2C_SLAVE_ADDRESS_VALUE appSlaveAddress = 0x28;
uint16_t writeStringLen = 0;

#define I2C_BUFFER_LEN 1024
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)
#define WRITE_THEN_READ_ADDRESS         0x02
#define WRITE_THEN_READ_OVERHEAD        0x02


// TImer 
#define PRINT_PERIOD 0x0034 // SYS Clock/TMR Prescaler/ticks per second
#define APP_HEARTBEAT_TMR_IS_PERIODIC true
#define APP_HEARTBEAT_TMR_PERIOD PRINT_PERIOD
#define APP_ALARM_PERIOD_IN_MS  167


void _mon_putc (char c)
 {
   while (U2STAbits.UTXBF); //Wait till transmission is complete
   U2TXREG = c;
 }




// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    u16 idx = BNO055_INIT_VALUE;
    s32 BNO055_iERROR = BNO055_INIT_VALUE;

    u8 stringpos = BNO055_INIT_VALUE;
    writeStringLen = cnt;

    appWriteString[idx++] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++) {
        appWriteString[idx++] = *(reg_data + stringpos);
    }

    // Send data over i2c
    appData.I2CBufferHandle = DRV_I2C_Transmit(appData.handleI2C0,
            dev_addr,
            appWriteString,
            idx,
            NULL);

    appData.I2CBufferEvent = DRV_I2C_BUFFER_EVENT_PENDING;

    // wait until the write operation is done
    while (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_PENDING) {
        appData.I2CBufferEvent = DRV_I2C_TransferStatusGet(appData.handleI2C0, appData.I2CBufferHandle);
        if (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_COMPLETE) {
            BNO055_iERROR = BNO055_SUCCESS;
        }
        if (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_ERROR) {
            BNO055_iERROR = BNO055_ERROR;
        }
    }

    return (s8) BNO055_iERROR;
}

/*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 BNO055_iERROR = BNO055_INIT_VALUE;




#if 1                         
    appData.I2CBufferHandle = DRV_I2C_TransmitThenReceive(appData.handleI2C0,
            dev_addr << 1,
            &appWriteString[0],
            1,
            reg_data,
            cnt,
            NULL);
#else
    // set the read address
    BNO055_I2C_bus_write(dev_addr, 0x08, reg_data, 1);

    // Start the read
    appData.I2CBufferHandle = DRV_I2C_Receive(appData.handleI2C0, dev_addr,
            appWriteString,
            cnt,
            NULL);
#endif

    // wait until all the data has been transferred
    appData.I2CBufferEvent = DRV_I2C_BUFFER_EVENT_PENDING;

    // wait until the write operation is done
    while (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_PENDING) {
        appData.I2CBufferEvent = DRV_I2C_TransferStatusGet(appData.handleI2C0, appData.I2CBufferHandle);
        if (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_COMPLETE) {
            BNO055_iERROR = BNO055_SUCCESS;
        }
        if (appData.I2CBufferEvent == DRV_I2C_BUFFER_EVENT_ERROR) {
            BNO055_iERROR = BNO055_ERROR;
        }
    }




    return (s8) BNO055_iERROR;

}

/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BNO055_delay_msek(u32 msek) {
    /*Here you can write your own delay routine*/
}

typedef struct _TAG_EULER_ANGLES {
    float pitch;
    float roll;
    float yaw;
} EULER_ANGLES, *pEULER_ANGLES;

typedef struct _TAG_QUATERNION_VECTOR {
    float q0;
    float q1;
    float q2;
    float q3;
} QUATERNION_VECTOR, *pQUATERNION_VECTOR;


//Source: http://docs.ros.org/latest-lts/api/dji_sdk_lib/html/DJI__Flight_8cpp_source.html#l00152

EULER_ANGLES toEulerianAngle(QUATERNION_VECTOR data) {
    EULER_ANGLES ans;

    double q2sqr = data.q2 * data.q2;
    double t0 = -2.0 * (q2sqr + data.q3 * data.q3) + 1.0;
    double t1 = +2.0 * (data.q1 * data.q2 + data.q0 * data.q3);
    double t2 = -2.0 * (data.q1 * data.q3 - data.q0 * data.q2);
    double t3 = +2.0 * (data.q2 * data.q3 + data.q0 * data.q1);
    double t4 = -2.0 * (data.q1 * data.q1 + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    ans.pitch = asin(t2);
    ans.roll = atan2(t3, t4);
    ans.yaw = atan2(t1, t0);

    return ans;
}

QUATERNION_VECTOR toQuaternion(EULER_ANGLES data) {
    QUATERNION_VECTOR ans;
    double t0 = cos(data.yaw * 0.5);
    double t1 = sin(data.yaw * 0.5);
    double t2 = cos(data.roll * 0.5);
    double t3 = sin(data.roll * 0.5);
    double t4 = cos(data.pitch * 0.5);
    double t5 = sin(data.pitch * 0.5);

    ans.q0 = t2 * t4 * t0 + t3 * t5 * t1;
    ans.q1 = t3 * t4 * t0 - t2 * t5 * t1;
    ans.q2 = t2 * t5 * t0 + t3 * t4 * t1;
    ans.q3 = t2 * t4 * t1 - t3 * t5 * t0;
    return ans;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
/*--------------------------------------------------------------------------*
 *	The following API is used to map the I2C bus read, write, delay and
 *	device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 init_BNO055_I2C_IFC(pBNO055DEV pBNO055dev) {
    pBNO055dev->bus_write = BNO055_I2C_bus_write;
    pBNO055dev->bus_read = BNO055_I2C_bus_read;
    pBNO055dev->delay_msec = BNO055_delay_msek;
    pBNO055dev->dev_addr = BNO055_I2C_ADDR1;

    return BNO055_INIT_VALUE;
}

/* Application's i2c Setup Function */
static void I2C_Setup(void) {
    appData.i2cStates = APP_I2C_START;
}

/******************************************************************************
  Function:
    static void APP_I2C_Task (void)
    
   Remarks:
    Allows a polled state machine to manage i2c testing.

 */

/*************read raw Euler data************/
/* variable used to read the euler h data */
s16 euler_data_h = BNO055_INIT_VALUE;
/* variable used to read the euler r data */
s16 euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data */
s16 euler_data_p = BNO055_INIT_VALUE;
/* structure used to read the euler hrp data */
struct bno055_euler_t euler_hrp;

/************read raw quaternion data**************/
/* variable used to read the quaternion w data */
s16 quaternion_data_w = BNO055_INIT_VALUE;
/* variable used to read the quaternion x data */
s16 quaternion_data_x = BNO055_INIT_VALUE;
/* variable used to read the quaternion y data */
s16 quaternion_data_y = BNO055_INIT_VALUE;
/* variable used to read the quaternion z data */
s16 quaternion_data_z = BNO055_INIT_VALUE;
/* structure used to read the quaternion wxyz data */
struct bno055_quaternion_t quaternion_wxyz;

/*****************read raw gravity sensor data****************/
/* variable used to read the gravity x data */
s16 gravity_data_x = BNO055_INIT_VALUE;
/* variable used to read the gravity y data */
s16 gravity_data_y = BNO055_INIT_VALUE;
/* variable used to read the gravity z data */
s16 gravity_data_z = BNO055_INIT_VALUE;
/* structure used to read the gravity xyz data */
struct bno055_gravity_t gravity_xyz;

/************read raw linear acceleration data***********/
/* variable used to read the linear accel x data */
s16 linear_accel_data_x = BNO055_INIT_VALUE;
/* variable used to read the linear accel y data */
s16 linear_accel_data_y = BNO055_INIT_VALUE;
/* variable used to read the linear accel z data */
s16 linear_accel_data_z = BNO055_INIT_VALUE;
/* structure used to read the linear accel xyz data */
struct bno055_linear_accel_t linear_acce_xyz;

uint8_t accel_calib_status;
uint8_t gyro_calib_status;
uint8_t mag_calib_status;
uint8_t sys_calib_status;

void APP_TimerAlarmSetup ( void )
{
    uint32_t tmrFreq, countsPerAlarmPeriod, timerPeriod;
    uint16_t countMax;
 
    tmrFreq = DRV_TMR_CounterFrequencyGet(appData.heartbeatTimer); // timer increment rate
 
    // number of timer counts needed for 1 alarm period
    // alarm period = timer increment rate * # of seconds per alarm period
    countsPerAlarmPeriod = tmrFreq * APP_ALARM_PERIOD_IN_MS / 1000;
 
    // number of timer interrupts needed per alarm period (16-bit timer)
    countMax = (countsPerAlarmPeriod/65536) + 1;            // +1 to round up
 
    timerPeriod = countsPerAlarmPeriod / countMax;          // timer period value
 
    appData.tmrPeriod = timerPeriod;
    appData.alarmCountMax = countMax;
}


void APP_TimerCallback ( uintptr_t context, uint32_t alarmCount ) {

    appData.alarmHasFired = true; 
}


static void APP_I2C_Task(void) {
    /* Variable used to return value of
communication routine*/
    s32 comres = BNO055_ERROR;
    static uint32_t tickCounter = 0;
    static SYS_TMR_HANDLE handle;

    switch (appData.i2cStates) {
        default:

        case APP_I2C_START:
        {
            tickCounter = 0;
            // Switch to the reading the euler angles
            appData.i2cStates = APP_I2C_DONE;
#if defined (USE_FUSION_OPS)            
            /************************* START READ RAW FUSION DATA ********
                For reading fusion data it is required to set the
                operation modes of the sensor
                operation mode can set from the register
                page - page0
                register - 0x3D
                bit - 0 to 3
                for sensor data read following operation mode have to set
             *FUSION MODE
             *0x08 - BNO055_OPERATION_MODE_IMUPLUS
             *0x09 - BNO055_OPERATION_MODE_COMPASS
             *0x0A - BNO055_OPERATION_MODE_M4G
             *0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
             *0x0C - BNO055_OPERATION_MODE_NDOF
        based on the user need configure the operation mode*/
            comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
#endif
            /************************* END READ RAW FUSION DATA  ************/
              // Configure alarm (fireoff at a rate of 1/6 hz)   
#if 1            
            appData.heartbeatTimer = SYS_TMR_CallbackPeriodic (10, 0, &APP_TimerCallback);
#else       
            appData.heartbeatTimer = DRV_TMR_Open ( DRV_TMR_INDEX_0,
                                                  DRV_IO_INTENT_EXCLUSIVE );
            if ( DRV_HANDLE_INVALID != appData.heartbeatTimer )  // Timer Driver opened?
            {
                APP_TimerAlarmSetup();
                DRV_TMR_AlarmRegister ( appData.heartbeatTimer,
                                        appData.tmrPeriod,
                                        true,
                                        0,
                                        APP_TimerCallback );
                DRV_TMR_Start(appData.heartbeatTimer);     // Start the timer
            }  
#endif
            appData.alarmHasFired = false;
            
            U2MODEbits.UARTEN = 0x01; //Enable UART module
   U2STAbits.UTXEN = 0x01;// Enable UART transmission
   printf ("Hello World!\r\n");
            

            

            break;
        }
        case APP_I2C_READ_EULER_ANGLES:
        {
#if defined (USE_FUSION_OPS)                        
            /*	Raw Euler H, R and P data can read from the register
                 page - page 0
                 register - 0x1A to 0x1E */
            comres += bno055_read_euler_h(&euler_data_h);
            comres += bno055_read_euler_r(&euler_data_r);
            comres += bno055_read_euler_p(&euler_data_p);
            comres += bno055_read_euler_hrp(&euler_hrp);
            /*	Raw Quaternion W, X, Y and Z data can read from the register
                page - page 0
                register - 0x20 to 0x27 */
            comres += bno055_read_quaternion_w(&quaternion_data_w);
            comres += bno055_read_quaternion_x(&quaternion_data_x);
            comres += bno055_read_quaternion_y(&quaternion_data_y);
            comres += bno055_read_quaternion_z(&quaternion_data_z);
            comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
            /*	Raw Linear accel X, Y and Z data can read from the register
                page - page 0
                register - 0x28 to 0x2D */
            comres += bno055_read_linear_accel_x(&linear_accel_data_x);
            comres += bno055_read_linear_accel_y(&linear_accel_data_y);
            comres += bno055_read_linear_accel_z(&linear_accel_data_z);
            comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);
            /*	Raw Gravity sensor X, Y and Z data can read from the register
                page - page 0
                register - 0x2E to 0x33 */
            comres += bno055_read_gravity_x(&gravity_data_x);
            comres += bno055_read_gravity_y(&gravity_data_y);
            comres += bno055_read_gravity_z(&gravity_data_z);
            comres += bno055_read_gravity_xyz(&gravity_xyz);
#endif            
            appData.i2cStates = APP_I2C_DONE;
            Nop();

            break;
        }
        case APP_I2C_READ_CALIBRATION_STATUS:

            accel_calib_status = 0;
            gyro_calib_status = 0;
            mag_calib_status = 0;
            sys_calib_status = 0;
#if defined (USE_FUSION_OPS)            
            bno055_get_accel_calib_stat(&accel_calib_status);
            bno055_get_gyro_calib_stat(&gyro_calib_status);
            bno055_get_mag_calib_stat(&mag_calib_status);
            bno055_get_sys_calib_stat(&sys_calib_status);
#endif            
            appData.i2cStates = APP_I2C_DONE;

            break;

        case APP_I2C_DONE:
        {
            if (appData.alarmHasFired)
            {
                appData.alarmHasFired = false;
                if (((tickCounter++)%6) == 0 )
                {
                    // every second get the calibration flags
                    appData.i2cStates = APP_I2C_READ_CALIBRATION_STATUS;
                }
                else
                {
                    /// other wise read the the euler angles
                    appData.i2cStates = APP_I2C_READ_EULER_ANGLES;
                }
            }
            break;
        }
        case APP_I2C_ERROR:
        {
            break;
        }

    }
}

/* TODO:  Add any necessary local functions.
 */

void wait_ms(uint32_t millisecs) {
    do {
        millisecs--;

    } while (millisecs);
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    appData.handleI2C0 = DRV_HANDLE_INVALID;
    
     appData.heartbeatTimer = DRV_HANDLE_INVALID;   // don't use until initialized
    appData.alarmCount = 0;  

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */

}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
#if defined (USE_FUSION_OPS)    
            if (appData.handleI2C0 == DRV_HANDLE_INVALID) {
                appData.handleI2C0 = DRV_I2C_Open(APP_DRV_I2C_INDEX, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= (DRV_HANDLE_INVALID != appData.handleI2C0);

                /*--------------------------------------------------------------------------*
                 *  This API used to assign the value/reference of
                 *	the following parameters
                 *	I2C address
                 *	Bus Write
                 *	Bus read
                 *	Chip id
                 *	Page id
                 *	Accel revision id
                 *	Mag revision id
                 *	Gyro revision id
                 *	Boot loader revision id
                 *	Software revision id
                 *-------------------------------------------------------------------------*/
                bno055.bus_write = BNO055_I2C_bus_write;
                bno055.bus_read = BNO055_I2C_bus_read;
                bno055.delay_msec = wait_ms;
                bno055.dev_addr = appSlaveAddress;
                
             
                bno055_init(&bno055);

            }
            
#endif
            if (appInitialized) {
                I2C_Setup();

                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            /* Run the state machine for servicing I2C */
            APP_I2C_Task();

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
