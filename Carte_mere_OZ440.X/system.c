/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "system.h"
#include "define.h"

#endif

/******************************************************************************/
/* Extern global variables declarations                                       */
/******************************************************************************/

extern unsigned int BoardVoltage[6];

/* Refer to the device datasheet for information about available
oscillator configurations. */

void ConfigureOscillator(void)
{

    OSCCONbits.IDLEN=1; /* Iddle mode when SLEEP instruction*/
    OSCCONbits.IRCF=0;  /* don't care*/
    OSCCONbits.SCS=0;   /*Primary oscillator selected*/

}

/* ---------------   GPIO configurations ---------------*/

void ConfigureGPIO(void)
{

    ADCON0bits.CHS = 0;     // AN0 selection
    ADCON0bits.ADON=1;      // A/D is operating

    ADCON1=0;
    ADCON1bits.VCFG = 0b10; // Vref+=2V internal

    ADCON2bits.ADFM = 1;    // result right justified
    ADCON2bits.ACQT = 0x07; // 20TAD acisition time
    ADCON2bits.ADCS = 0x06; // Fosc/64 clock source for ADC

    ANCON0 = 0b0000001;     // only AN0 configured as analog
    ANCON1 = 0;             // all digital (AN8--AN14)

    TRISA = 0b00111111;    // inputs for analog, and sensor inputs
    
    TRISB = 0b11111000;    // output for TX CAN and NU pin
    LATBbits.LATB2=1;      // fix output TXCAN

    TRISC = 0b00011010;    // input for motor fault, and I2C lines
    MOTOR_EXT_PIN = 0;     // init of motor lines
    MOTOR_RET_PIN=0;
  
    TRISD = 0x00;           // output for sensor out, and for USB activation signals
    USB0_EN_PIN = 1;        // enable by default USB lines
    USB1_EN_PIN = 1;

    TRISE = 0b00000000;     // nothing connected: output
    LATE=0;

}


void ConfigureInterrupts(void)
{
    //all interrupts are low priority, except CAN module

    RCONbits.IPEN = 1;           // enables priority logic for Interrupts.
            
    INTCON = 0b00100000;        // GIE interrupts disabled for the moment, TMRO OVF activated
    INTCON2 = 0b11111000;       // low priority for TMR0 overflow
    INTCON3 = 0b00000000;       // no external interrupt

    PIR1 = 0;                   //clear IT flags
    PIR2 = 0;
    PIR3 = 0;
    PIR4 = 0;
    PIR5 = 0;

    PIE1 = 0b00100000;          // enable USART1  RX interrupts.
    PIE2 = 0;
    PIE3 = 0b00100000;          // enable USART2  RX interrupts.
    PIE4 = 0;
    PIE5 = 0b00000011;          // enable CAN RX interrupts.

    IPR1= 0 ;                   // low priority IT's
    IPR2= 0 ;                   // low priority IT's
    IPR3= 0 ;                   // low priority IT's
    IPR4= 0 ;                   // low priority IT's
    IPR5= 0b00000011 ;          // High priority for CAN RX.
   
    INTCONbits.GIE=1;           // general IT enable
    INTCONbits.PEIE=1;          // peripheral IT enable
}

void ConfigureTimers(void)
{
    T0CONbits.T08BIT = 0;       //TMR0 in 16 bits mode
    T0CONbits.T0CS = 0;         // TMRO source is internal clock
    T0CONbits.PSA = 0;          // PSA assigned to TIMER0
    T0CONbits.T0PS=6;           // 1:128 prescaler

    TMR0H = TMR0H_INIT;
    TMR0L = TMR0L_INIT;         // preload for 10ms overflow (TMR0 = 1250)

    T0CONbits.TMR0ON = 1;       // enable TIMER0

}

/*******************************************/
/**********  HIGH level I2C functions *****/
/*******************************************/
void UpdateBoardVoltages(void)
{

    //********* update 12V voltage ********

        WriteISPPACRegister(ISPPAC_ADC_MUX_ADRESS, P12V_ISPPAC);
        __delay_us(50);
        BoardVoltage[0]=ReadISPPACRegister(ISPPAC_ADC_HIGH_ADRESS);
        BoardVoltage[0]=BoardVoltage[0]<<4;
        BoardVoltage[0]+=(ReadISPPACRegister(ISPPAC_ADC_LOW_ADRESS) & 0xF0)>>4;
        BoardVoltage[0]=BoardVoltage[0]<<1;                                         // gain ISPPAC 2mV=/LSB
        BoardVoltage[0]=(unsigned int)((float) BoardVoltage[0] * P12V_RESISTOR_GAIN);         // gain pont diviseur

        //********* update 3V3 voltage ********

        WriteISPPACRegister(ISPPAC_ADC_MUX_ADRESS, P3V3_ISPPAC);
        __delay_us(50);
        BoardVoltage[1]=ReadISPPACRegister(ISPPAC_ADC_HIGH_ADRESS);
        BoardVoltage[1]=BoardVoltage[1]<<4;
        BoardVoltage[1]+=(ReadISPPACRegister(ISPPAC_ADC_LOW_ADRESS) & 0xF0)>>4;
        BoardVoltage[1]=BoardVoltage[1]<<1;        // gain ISPPAC 2mV=/LSB

        //********* update 5V voltage ********

        WriteISPPACRegister(ISPPAC_ADC_MUX_ADRESS, P5V_ISPPAC);
        __delay_us(50);
        BoardVoltage[2]=ReadISPPACRegister(ISPPAC_ADC_HIGH_ADRESS);
        BoardVoltage[2]=BoardVoltage[2]<<4;
        BoardVoltage[2]+=(ReadISPPACRegister(ISPPAC_ADC_LOW_ADRESS) & 0xF0)>>4;
        BoardVoltage[2]=BoardVoltage[2]<<1;        // gain ISPPAC 2mV=/LSB

        //********* update 24V voltage ********

        WriteISPPACRegister(ISPPAC_ADC_MUX_ADRESS, P24V_ISPPAC);
        __delay_us(50);
        BoardVoltage[3]=ReadISPPACRegister(ISPPAC_ADC_HIGH_ADRESS);
        BoardVoltage[3]=BoardVoltage[3]<<4;
        BoardVoltage[3]+=(ReadISPPACRegister(ISPPAC_ADC_LOW_ADRESS) & 0xF0)>>4;
        BoardVoltage[3]=BoardVoltage[3]<<1;                                             // gain ISPPAC 2mV=/LSB
        BoardVoltage[3]=(unsigned int)((float)BoardVoltage[3]* P24V_RESISTOR_GAIN);     // gain pont diviseur
                
        //********* update batt voltage ********

        WriteISPPACRegister(ISPPAC_ADC_MUX_ADRESS, BATT_ISPPAC);
        __delay_us(50);
        BoardVoltage[4]=ReadISPPACRegister(ISPPAC_ADC_HIGH_ADRESS);
        BoardVoltage[4]=BoardVoltage[4]<<4;
        BoardVoltage[4]+=(ReadISPPACRegister(ISPPAC_ADC_LOW_ADRESS) & 0xF0)>>4;
        BoardVoltage[4]=BoardVoltage[4]<<1;        // gain ISPPAC 2mV=/LSB

        //********* update 12V flag values ********

        BoardVoltage[5]= ~ReadISPPACRegister(P12V_FLAG_ISPPAC_ADRESS);
        BoardVoltage[5]= ( BoardVoltage[5] & 0x000C) >> 2;
                
}

/*******************************************/
/***********   MID level I2C functions *****/
/*******************************************/

unsigned char ReadISPPACRegister(unsigned char LocalAdress)
{
    unsigned char I2CData;

    /******* write adress to read *****/

    Configure_I2C();
    Idle_I2C();
    Start_I2C();
    Idle_I2C();
    Write_I2C(ISPPAC_I2C_ADRESS<<1 | ISPPAC_WRITE); // adresse I2C, ecriture
    Idle_I2C();

    Write_I2C(LocalAdress); // adresse I2C, lecture
    Idle_I2C();
    NotAck_I2C();
    Idle_I2C();
    Stop_I2C();
    
    /*********************/
    
    Configure_I2C();
    Idle_I2C();
    Start_I2C();
    Idle_I2C();
    Write_I2C(ISPPAC_I2C_ADRESS<<1 | ISPPAC_READ); // adresse I2C, lecture
    Idle_I2C();

    I2CData=Read_I2C();
    Idle_I2C();
    NotAck_I2C();
    Idle_I2C();
    Stop_I2C();

    return I2CData;

}

void WriteISPPACRegister(unsigned char LocalAdress, unsigned char LocalData)
{

    Configure_I2C();
    Idle_I2C();
    Start_I2C();
    Idle_I2C();
    Write_I2C(ISPPAC_I2C_ADRESS<<1 | ISPPAC_WRITE); // adresse I2C, ecriture
    Idle_I2C();

    Write_I2C(LocalAdress); // adresse ISPPAC a ecrite
    Idle_I2C();
    Write_I2C(LocalData); // donnée ISPPAC a ecrite
    Idle_I2C();
    NotAck_I2C();
    Idle_I2C();
    Stop_I2C();

}

/*******************************************/
/***********   LOW level I2C functions *****/
/*******************************************/

void Configure_I2C(void)
{
  SSPSTAT &= 0x3F;               // power on state 
  SSPCON1 = 0x00;                // power on state
  SSPCON2 = 0x00;                // power on state
  SSPCON1 |= 0x8;                //  master serial mode 
  SSPSTATbits.SMP=1;             // slew rate off 

  PORTCbits.RC3 = 1;
  PORTCbits.RC4 = 1;
  
  SSPCON1bits.SSPEN = 1;          // enable synchronous serial port
  SSPADD = 0x54;                  // 200kHz Baud clock
}

void Idle_I2C( void )
{
  while ( ( SSPCON2 & 0x1F ) || ( SSPSTATbits.R_W ) )
     continue;
}

unsigned char Read_I2C( void )
{
    Idle_I2C();                 // ensure module is idle
    SSPCON2bits.RCEN = 1;           // enable master for 1 byte reception
    while ( !SSPSTATbits.BF );      // wait until byte received
    return ( SSPBUF );              // return with read byte
}

signed char Write_I2C( unsigned char data_out )
{
  SSPBUF = data_out;           // write single byte to SSPBUF
  if ( SSPCON1bits.WCOL )      // test if write collision occurred
   return ( -1 );              // if WCOL bit is set return negative #
  else
  {
      while( SSPSTATbits.BF );   // wait until write cycle is complete
      Idle_I2C();                 // ensure module is idle
      if ( SSPCON2bits.ACKSTAT ) // test for ACK condition received
        return ( -2 );			// return NACK
      else return ( 0 );              //return ACK


  }
}

void Start_I2C(void)
{
    SSPCON2bits.SEN=1;
    while(SSPCON2bits.SEN);
}

void Stop_I2C()
{
    SSPCON2bits.PEN=1;
    while(SSPCON2bits.PEN);
}

void NotAck_I2C(void)
{
    SSPCON2bits.ACKDT=1;
    SSPCON2bits.ACKEN=1;
    while(SSPCON2bits.ACKEN);
}

char GetActuatorPosition(void)
{
    unsigned int adc_result;

    ADCON0bits.GO=1;
    while(ADCON0bits.GO==1);
    adc_result=(ADRESH);
    adc_result=adc_result<<8;
    adc_result=adc_result | ADRESL;

    adc_result=adc_result/ADC_ACTUATOR_GAIN_DIVIDER;

    if(adc_result>100)      //restrict value to 0--100%
        adc_result=100;
    
        
    return (char) adc_result;

}