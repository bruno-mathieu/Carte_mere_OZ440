/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>


#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "define.h"        /* board level definitions*/
#include "ecanpoll.h"      /* CAN library header file*/


#include "Can_HL.h"

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

// structure used to count 10ms tick
unsigned char TickCounter;

//  variable for CAN  FIFO buffer
struct CANTxFifo CANTxFifo;
struct CANRxFifo CANRxFifo;

//variables for onboard voltages measurement
unsigned int BoardVoltage[6];


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    char TempVar;

    //  variable for CAN TX FIFO buffer
    struct CANTxMsg TempCANTxMsg;

    //  variable for CAN RX FIFO buffer
    struct CANRxMsg TempCANRxMsg;

    // variable for motor drive
    unsigned char MotorCurrentPosition;

    unsigned char MotorOrder;
    unsigned char NewReqOrder;

    unsigned char MotorReqPosition;
    unsigned char NewReqPosition;

    unsigned char MotorMoving;

    unsigned char I2CData;
    
    

    //------------ ----------------------------------------
    //----------  CPU internal configurations: -----------
    //----------------------------------------------------

    /* Configure the oscillator for the CPU */
    ConfigureOscillator();
    __delay_ms(10);             // wait for Oscillator to be stabilized
    __delay_ms(10);             // wait for Oscillator to be stabilized
    __delay_ms(10);             // wait for Oscillator to be stabilized
    __delay_ms(10);             // wait for Oscillator to be stabilized


    // configure CPU GPIO for motherboard
    ConfigureGPIO();

    //CAN controller Initialize
    ECANInitialize();
    //Set MASK and Filters for CAN
    ECANFiltersInit();

    // Timers configuration
    ConfigureTimers();

    // I2c Peripheral configuration
     
    //----------------------------------------------------
    //----------  Global variables initialisation --------
    //----------------------------------------------------
    
    // initialize CAN tx FIFO
    CANTxFifoInit();
    CANRxFifoInit();

    TickCounter=0;
    
    MotorOrder=STOP;
    NewReqOrder=FALSE;

    MotorReqPosition=0;
    NewReqPosition=FALSE;

    MotorMoving=FALSE;



    //----------------------------------------------------
    //------  external peripheral configurations: --------
    //----------------------------------------------------
 
    
    //----------------------------------------------------
    //----------    Ready to go in main loop:  -----------
    //----------    interrupts activation      -----------
    //----------------------------------------------------

    ConfigureInterrupts();


    //-----------------------------------------------------
    //-------------  infinite main loop ----------
    //----------------------------------------------------

    while(1)
    {

        //--------------------------------------------------------------------------------
        //-------------  periodic tasks occures according to TickCounter variable----------
        //--------------------------------------------------------------------------------

        if(TickCounter>TICK_PERIOD)
        {
            CLRWDT();                                 // clear watchdog timer each real time cycles

            UpdateBoardVoltages();
           
            TickCounter=0;                          // reset  tick counter to 0

            // ----- send sensor input states to CANFIFO each RT tick---
            TempVar =(PORTA>>1) & 0b111;
            TempVar = TempVar | ((PORTA>>2)&0b1000);

            TempCANTxMsg.data_TX[0]=TempVar;

            TempCANTxMsg.dataLen= SENSOR_RD_MESSAGE_LEN;
            TempCANTxMsg.id = (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SENSOR_RD_MESSAGE_ADRESS );
            TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
            PutCANTxFifo(TempCANTxMsg);
            
            
            //--- direct motor command requested... --------

            if(MotorOrder==STOP && NewReqOrder==TRUE)
            {
                MOTOR_EXT_PIN=0;
                MOTOR_RET_PIN=0;
                NewReqOrder=FALSE;
                MotorMoving=FALSE;
            }
            if(MotorOrder==EXTEND && NewReqOrder==TRUE)
            {
                MOTOR_EXT_PIN=1;
                MOTOR_RET_PIN=0;
                NewReqOrder=FALSE;
                MotorMoving=TRUE;
            }
            if(MotorOrder==RETRACT && NewReqOrder==TRUE)
            {
                MOTOR_EXT_PIN=0;
                MOTOR_RET_PIN=1;
                NewReqOrder=FALSE;
                MotorMoving=TRUE;
            }

            //--- motor position request... --------

            if(NewReqPosition==TRUE)
            {
                TempVar = GetActuatorPosition();        //return actuator position

                if(TempVar>MotorReqPosition)
                {
                    MOTOR_EXT_PIN=0;
                    MOTOR_RET_PIN=1;
                    MotorMoving=TRUE;
                }
                if(TempVar<MotorReqPosition)
                {
                    MOTOR_EXT_PIN=1;
                    MOTOR_RET_PIN=0;
                    MotorMoving=TRUE;
                }
                if(TempVar>(MotorReqPosition-1) & TempVar<(MotorReqPosition+1)) // +/-1% error allowed for stop
                {
                    MOTOR_EXT_PIN=0;
                    MOTOR_RET_PIN=0;
                    MotorMoving=FALSE;
                    NewReqPosition=FALSE;
                }
            }
            
        }

        

        //--------------------------------------------------------------------------------
        //-------------   Permanent tasks: executed as fast as possible  ----------------
        //--------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------
        // --------  Check if CAN RX fifo is empty or not and perform treatment  ---------
        //--------------------------------------------------------------------------------
        
        if(!CANRxFifo.FifoEmpty && !CANTxFifo.Fifofull)
        {
            TempCANRxMsg = GetCANRxFifo();

            // ------------------  Return software version to CAN if RTR message detected  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SOFT_VERSION_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=MAJOR_SW_VERSION;
                TempCANTxMsg.data_TX[1]=MINOR_SW_VERSION;
                TempCANTxMsg.dataLen= SOFT_VERSION_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SOFT_VERSION_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }

            // ------------------  Return Internal voltage monitor to CAN if RTR message detected  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | INTERNAL_VOLTAGE_MONITOR_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=(unsigned char)(BoardVoltage[0] & 0x00FF);
                TempCANTxMsg.data_TX[1]=(unsigned char)((BoardVoltage[0] & 0xFF00)>>8);
                TempCANTxMsg.data_TX[2]=(unsigned char)(BoardVoltage[1] & 0x00FF);
                TempCANTxMsg.data_TX[3]=(unsigned char)((BoardVoltage[1] & 0xFF00)>>8);
                TempCANTxMsg.data_TX[4]=(unsigned char)(BoardVoltage[2] & 0x00FF);
                TempCANTxMsg.data_TX[5]=(unsigned char)((BoardVoltage[2] & 0xFF00)>>8);
                TempCANTxMsg.data_TX[6]=(unsigned char)(BoardVoltage[5] & 0x00FF);

                TempCANTxMsg.dataLen= INTERNAL_VOLTAGE_MONITOR_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | INTERNAL_VOLTAGE_MONITOR_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }
            // ------------------  Return External voltage monitor to CAN if RTR message detected  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | EXTERNAL_VOLTAGE_MONITOR_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=(unsigned char)(BoardVoltage[3] & 0x00FF);
                TempCANTxMsg.data_TX[1]=(unsigned char)((BoardVoltage[3] & 0xFF00)>>8);
                TempCANTxMsg.data_TX[2]=(unsigned char)(BoardVoltage[4] & 0x00FF);
                TempCANTxMsg.data_TX[3]=(unsigned char)((BoardVoltage[4] & 0xFF00)>>8);

                TempCANTxMsg.dataLen= EXTERNAL_VOLTAGE_MONITOR_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | EXTERNAL_VOLTAGE_MONITOR_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }

            // ------------------  Return motor position and status to CAN if RTR message detected  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | POS_DATA_MESSAGE_ADRESS ) && TempCANRxMsg.flags== ECAN_RX_RTR_FRAME )
            {
                TempCANTxMsg.data_TX[0]=GetActuatorPosition();      // return motor position in first Byte

                if(MotorMoving==TRUE)                                     //return moving status and error status in second byte
                    TempVar=1;
                else
                    TempVar=0;

                if(MOTOR_FAULT_N==0)
                    TempVar=TempVar+2;

                TempCANTxMsg.data_TX[1]=TempVar;
                
                TempCANTxMsg.dataLen= POS_DATA_MESSAGE_LEN;
                TempCANTxMsg.id = (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | POS_DATA_MESSAGE_ADRESS );
                TempCANTxMsg.flags = ECAN_TX_STD_FRAME;
                PutCANTxFifo(TempCANTxMsg);
            }


            // ------------------  Get Host requested position, and store it  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | POS_REQ_DATA_MESSAGE_ADRESS ) && TempCANRxMsg.dataLen== POS_REQ_DATA_MESSAGE_LEN )
            {
                if(TempCANRxMsg.data_RX[0]>=0 && TempCANRxMsg.data_RX[0]<=100)        // only if order is valid (between 0-100%)
                {
                    MotorReqPosition = TempCANRxMsg.data_RX[0];
                    NewReqPosition=TRUE;
                    NewReqOrder=FALSE;
                }
                
            }

            // ------------------  Get Host requested Command, and store it  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | MOTOR_COMMAND_MESSAGE_ADRESS ) && TempCANRxMsg.dataLen== MOTOR_COMMAND_MESSAGE_LEN )
            {
                if(TempCANRxMsg.data_RX[0]==STOP || TempCANRxMsg.data_RX[0]==EXTEND || TempCANRxMsg.data_RX[0]==RETRACT)
                {
                    MotorOrder = TempCANRxMsg.data_RX[0];
                    NewReqOrder=TRUE;
                    NewReqPosition=FALSE;
                }
            }

            // ------------------  Get Host sensor & USB Command, and store it  ---------------------------

            if( TempCANRxMsg.id == (CAN_MESSAGE_ACTUATOR_TYPE << 7 | CAN_DEVICE_ADRESS <<4 | SENSOR_WR_MESSAGE_ADRESS ) && TempCANRxMsg.dataLen== SENSOR_WR_MESSAGE_LEN )
            {
                if( (TempCANRxMsg.data_RX[0] & 0b00000001) == 0b00000001)
                    SENSOR1_OUT=1;
                else
                    SENSOR1_OUT=0;
                
                if( (TempCANRxMsg.data_RX[0] & 0b00000010) == 0b00000010)
                    SENSOR2_OUT=1;
                else
                    SENSOR2_OUT=0;

                if( (TempCANRxMsg.data_RX[0] & 0b00000100) == 0b00000100)
                    SENSOR3_OUT=1;
                else
                    SENSOR3_OUT=0;

                if( (TempCANRxMsg.data_RX[0] & 0b00001000) == 0b00001000)
                    SENSOR4_OUT=1;
                else
                    SENSOR4_OUT=0;

                if( (TempCANRxMsg.data_RX[1] & 0b00000001) == 0b00000001)
                    USB0_EN_PIN=1;
                else
                    USB0_EN_PIN=0;

                if( (TempCANRxMsg.data_RX[1] & 0b00000010) == 0b00000010)
                    USB1_EN_PIN=1;
                else
                    USB1_EN_PIN=0;
                
            }

        }
        

        //--------------------------------------------------------------------------------
        // ---  Send can message if TXB0 buffer free, and data available in CAN TX FIFO --
        //--------------------------------------------------------------------------------
        
        if(!CANTxFifo.FifoEmpty && !TXB0CONbits.TXREQ)          // if fifo is not empty and buffer0 empty
        {
            TempCANTxMsg=GetCANTxFifo();
            ECANSendMessage(TempCANTxMsg.id,TempCANTxMsg.data_TX,TempCANTxMsg.dataLen,TempCANTxMsg.flags);  // fill tx buffer with Fifo data
            
        }


    }

}

