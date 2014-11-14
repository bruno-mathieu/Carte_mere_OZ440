
#include "ECANPoll.h"
#include "Define.h"

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        8000000L
#define FCY             SYS_FREQ/4


/******************************************************************************/
/* Variable types declaration                                                      */
/******************************************************************************/



struct CANTxMsg{

    unsigned long id;
    BYTE data_TX[8];
    BYTE dataLen;
    ECAN_TX_MSG_FLAGS flags;

};

struct CANRxMsg{

    unsigned long id;
    BYTE data_RX[8];
    BYTE dataLen;
    ECAN_RX_MSG_FLAGS flags;

};

struct CANTxFifo{

    struct CANTxMsg CANMsg[CANTX_FIFO_SIZE];

    unsigned char LowIndex;
    unsigned char HighIndex;
    unsigned char Fifofull;
    unsigned char FifoEmpty;

};

struct CANRxFifo{

    struct CANRxMsg CANMsg[CANRX_FIFO_SIZE];

    unsigned char LowIndex;
    unsigned char HighIndex;
    unsigned char Fifofull;
    unsigned char FifoEmpty;

};



 
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/


/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

void ConfigureOscillator(void);     /* Handles clock switching/osc initialization */
void ConfigureGPIO(void);           /* Handles GPIO configuration*/
void ConfigureInterrupts(void);     /* Handles Interrupts configuration*/
void ConfigureTimers(void);         /* handles timers configuration*/
char GetActuatorPosition(void);     /*returns actuator position in % : 0%--> retracted / 100%--> extended */