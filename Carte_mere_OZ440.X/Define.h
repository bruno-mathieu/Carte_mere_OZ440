/* 
 * File:   Define.h
 * Author: Bruno
 *
 * Created on 13 juin 2014, 16:13
 */

#ifndef DEFINE_H
#define	DEFINE_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* DEFINE_H */
#define MAJOR_SW_VERSION 1
#define MINOR_SW_VERSION 0

#define BOARD_NUMBER 1
#define BOARD_REVISION 1

#define _XTAL_FREQ 64000000

#define TMR0H_INIT  0xFB              // TMR0 value for 10ms tick
#define TMR0L_INIT  0x1E              // TMR0 value for 10ms tick

#define TICK_PERIOD 1                 //  RT tick each 20ms

#define CAN_MESSAGE_ACTUATOR_TYPE 8

#define CAN_DEVICE_ADRESS 0

#define POS_REQ_DATA_MESSAGE_ADRESS 0x0
#define POS_REQ_DATA_MESSAGE_LEN 1

#define POS_DATA_MESSAGE_ADRESS 0x1
#define POS_DATA_MESSAGE_LEN 2

#define MOTOR_COMMAND_MESSAGE_ADRESS 0x2
#define MOTOR_COMMAND_MESSAGE_LEN 1

#define SENSOR_RD_MESSAGE_ADRESS 0x3
#define SENSOR_RD_MESSAGE_LEN 1

#define SENSOR_WR_MESSAGE_ADRESS 0x4
#define SENSOR_WR_MESSAGE_LEN 2

#define SOFT_VERSION_MESSAGE_ADRESS 0x5
#define SOFT_VERSION_MESSAGE_LEN 2

#define CANTX_FIFO_SIZE 10
#define CANRX_FIFO_SIZE 10

#define MOTOR_EXT_PIN   LATCbits.LATC2
#define MOTOR_RET_PIN   LATCbits.LATC6

#define USB0_EN_PIN LATDbits.LATD4
#define USB1_EN_PIN LATDbits.LATD5

#define SENSOR1_OUT LATDbits.LATD0
#define SENSOR2_OUT LATDbits.LATD1
#define SENSOR3_OUT LATDbits.LATD2
#define SENSOR4_OUT LATDbits.LATD3

#define MOTOR_FAULT_N   PORTCbits.RC1

#define ADC_ACTUATOR_GAIN_DIVIDER 34    //gain of 34 on analog stage

#define TRUE    1
#define FALSE   0

#define STOP 0
#define RETRACT 1
#define EXTEND  2

