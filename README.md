# Inverter---dsPIC33FJ09GS302-
1000W Inverter based on the dspic33f miicrocontroller
/*
 * File:   Main.c
 * Author: jewalker
 *
 * Created on October 12, 2016, 3:18 PM
 */


#include "xc.h"
#include "p33fxxxx.h"

#include <math.h>
//#include "pps.h"



#define FOSC    (80000000ULL)
#define FCY     (FOSC/2)
#define high 1
#define low 0

 /*******************************
 * Set device configuration values
********************************/
_FOSCSEL(FNOSC_FRC    & IESO_ON);
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF & POSCMD_NONE & IOL1WAY_ON);
_FWDT(FWDTEN_OFF);
_FICD(JTAGEN_OFF); 
//_FPOR(FPWRT_PWR128);
/*Global Variables*/

int j;
int i,k;
int duty;
int TxData[] = {'T','E','S','T'};
int time;







int sinetable2[] = {
    
48	,
95	,
143	,
190	,
237	,
284	,
331	,
378	,
424	,
471	,
516	,
562	,
607	,
652	,
697	,
741	,
784	,
827	,
870	,
912	,
953	,
994	,
1034	,
1073	,
1112	,
1150	,
1188	,
1224	,
1260	,
1295	,
1330	,
1363	,
1396	,
1427	,
1458	,
1488	,
1517	,
1545	,
1572	,
1598	,
1623	,
1647	,
1670	,
1692	,
1712	,
1732	,
1751	,
1768	,
1785	,
1800	,
1814	,
1827	,
1839	,
1850	,
1859	,
1868	,
1875	,
1881	,
1886	,
1889	,
1892	,
1893	,
1893	,
1892	,
1889	,
1886	,
1881	,
1875	,
1868	,
1860	,
1850	,
1840	,
1828	,
1815	,
1801	,
1786	,
1769	,
1752	,
1733	,
1714	,
1693	,
1671	,
1648	,
1624	,
1599	,
1573	,
1547	,
1519	,
1490	,
1460	,
1429	,
1398	,
1365	,
1332	,
1298	,
1262	,
1227	,
1190	,
1153	,
1115	,
1076	,
1036	,
996	,
956	,
914	,
872	,
830	,
787	,
743	,
699	,
655	,
610	,
565	,
519	,
473	,
427	,
381	,
334	,
287	,
240	,
193	,
146	,
98	,
51	,
3	,


};




 // Prototypes 

void PWMcenter();
void InitUART();
void InitPWM();
void DirtyDelay();
void AnotherDirtyDelay();
void CheckPWM(); 
void InitTimer1();
void PWMOnAllSwitches();
void PWMOnOneSide();
void InitOutPutPins();
void SendData();
void DirtyDelay(int);
void InitRemappablePins();
void TestPWM();
void PWMLowFreqBottom();
void DirtyDelay3();
void PWMLowFreqBottom2();
void PWMup();
void PWMdown();
void PwmDutyUpdate();





void PwmDutyUpdate(void)
{
    if(j<124)
    {
        PWMup();
    }
    else if(j>=124)
    {
        PWMdown();
    }

    if(j>=249)
    {
        j=0;
        i=0;
    }
   
    j++;
   
}


void PWMup(void)

{
        duty = sinetable2[i];
        LATBbits.LATB9 = low;
        LATBbits.LATB10 = high;
        PDC1 = duty;
        PDC2 = 0;
        i++;
 
 
}


void PWMdown(void)
{        
        duty = sinetable2[i];
        LATBbits.LATB9 = high;
        LATBbits.LATB10 = low;     
        PDC1 = 0;
        PDC2 = duty;
        i--;  

 
  
    
      
    
}
void InitOutPutPins(void)
{
         
       TRISA = 0;
       TRISB = 0;
       ADPCFG = 0xfff;
       
       //LATBbits.LATB9 = high;
       //LATBbits.LATB10 = low;
       
       
}


void InitRemappablePins(void)
{
    OSCCONL = 0x46;
OSCCONL = 0x57;
OSCCONbits.IOLOCK = 0; // Unlock remappable I/O



RPINR20bits.SDI1R = 0;//RP0
RPINR20bits.SCK1R = 2;// CLK Pin RP2
RPOR0bits.RP1R = 7;//SDO Pin RP1
RPOR1bits.RP2R = 8;//CLK Pin RP2
//RPOR1bits.RP3R = 9;//Slave Select RP3 

OSCCONL = 0x46;
OSCCONL = 0x57;
OSCCONbits.IOLOCK = 1; // Lock remappable I/O


}
/*The below pwm function initailizes the pwm generator to do the following: Generates an interrupt 
 * every x pwm cycles. Where x is determined by the TRGCON1bits.TRGDIV register. 
 * The idea here was to generate an interrupt at a frequency of 60Hz. I would use 
 * this interrupt to turn off the lower fets of the H-bridge. However, the slowest
 * frequency I can get to is 784Hz. I think I can get lower if I changed the main pwm clock 
 * and added pre and/or post scalers in the pwm generator. Then I would have to change the PTPER 
 * accordingly. */

void InitPWM(void)
 {
     PTCONbits.PTEN = 0; // 1 = PWM timer base ON. 0 = PWM timer base OFF
     //SEVTCMP = 1150;
     PTCONbits.PTSIDL = 0; //PWM rums in idle mode.
    
     //PTCONbits.SESTAT = 0; // Special event status bit not pending. Hardware Clear/Set
     PTCONbits.EIPU = 0; //Update PWM on cycle boundaries
     PTCONbits.SYNCPOL = 0; // Sync input/output bit. active high
     PTCONbits.SYNCOEN = 0; //sync output disabled
     PTCONbits.SYNCEN = 0;// sync of primary time base disabled
    // PTCONbits.SYNCSRC --> sync source election. Not using
     
     PTCONbits.SEVTPS0 = 0; //--> special event postscaler. Not using
     //PTCONbits.SEIEN = 1; // 0=Special event  disabled : 1=enabled    
     
     
     
     //PTCON2bits.PCLKDIV =6;// PWM input clock prescaler. 0 = divide by 64
     PTCON2bits.PCLKDIV = 0;
     
     
    //PTPER = 1893; // Empirically found to give 60Hz PWM with a PCLKDIV of 64
     PHASE1 = 2500; //1893;//65093;// was 2332 --- 2100 as of 6/2/17
     //SPHASE1  = 2500;
     //PHASE1 = 61137;
     
     PHASE2 = 2500; //gives roughly 15KHz with PCLKDIV at 3?
    //SPHASE2 = 2500;
     
     PWMCON1bits.ITB = 1; //  =0 ->Use PTPER register for pwm timing.1 -> use
     //phase and sphase
     PWMCON1bits.MDCS = 0; //use PDCx registers for duty cycle
     PWMCON1bits.DTC = 0; // 2=deadtime control disabled, 0=positive deadtime
     PWMCON1bits.CAM = 1;//center alligned mode disabled
     PWMCON1bits.XPRES = 0;// exteranl pins have no effect on pwm time base
    // DTR1 = 50;
    // ALTDTR1 = 200;
     PWMCON1bits.IUE = 0; // update PDCx with PWM time base
     //DTR1 = 500;
     
     IOCON1bits.PENH = 1; //PWMmodule controls this pin
     IOCON1bits.PENL = 1; //PWM module controls this pin
     IOCON1bits.POLH = 0; //active high pin
     IOCON1bits.POLL = 0; //active high pin
     IOCON1bits.PMOD = 3; //0=complementary mode,2=push pull mode,3 = independent mode
     IOCON1bits.OVRENH = 0; //PWMmodule provides data for this pin
     IOCON1bits.OVRENL = 0; //PWM module provide data for this pin
     //IOCON1bits.OVRDAT = 0;
     //IOCON1bits.OSYNC = 1;
     
     
     
     PWMCON2bits.ITB = 1; //  =0 ->USe PTPER register for pwm timing.1 -> use
     //phase and sphase
     PWMCON2bits.MDCS = 0; //use PDCx registers for duty cycle
     PWMCON2bits.DTC = 0; // dead-time control disabled
     PWMCON2bits.CAM = 1;//center aligned mode disabled
     PWMCON2bits.XPRES = 0;// external pins have no effect on pwm time base
     PWMCON2bits.IUE = 0; // update PDCx with PWM time base
     
     //DTR2 = 50;
    // ALTDTR2 = 200;//was 100
     
     IOCON2bits.PENH = 1; //PWMmodule controls this pin
     IOCON2bits.PENL = 1; //PWM module controls this pin
     IOCON2bits.POLH = 0; //active high pin
     IOCON2bits.POLL = 0; //active high pin
     IOCON2bits.PMOD = 3; //0=complementary mode,2=push pull
     IOCON2bits.OVRENH = 0; //PWMmodule provides data for this pin
     IOCON2bits.OVRENL = 0; //PWM module provide data for this pin
     //IOCON2bits.OSYNC = 1;
     
     // to generate a 60Hz interrupt I may have to come up with a function to 
     // increment the trigger counter because the scalers are maxed out and it is only at
     // 786Hz
     //TRIG1 = 0; --> Only used when pwm generator is local time base?
     
     //TRGCON1bits.TRGDIV = 3;// trigger output every 4th event/period(was a 3))
     //TRGCON1bits.TRGSTRT = 0;//first trigger event occurs after four trigger match events(was 0))
     //PWMCON1bits.TRGIEN = 1;//trigger event generates an interrupt request

     
    IOCON2bits.OVRDAT0 = 1; // bottom FET (Q4) held high while PDC1 PWMs
    IOCON2bits.OVRDAT1 = 0; // Top FET (Q3) Held low while it doesn't pulse        
    IOCON1bits.OVRDAT1 = 0; // top FET goes low when not pulsing
    IOCON1bits.OVRDAT0 = 1;  // bottom fet goes high when not pulsing
     
     
     PTCONbits.PTEN = 1; // 1 = PWM timer base ON. 0 = PWM timer base OFF
     //IEC5bits.PWM1IE = 1; //PWM interrupt ON
 }


void InitTimer1(void)
{
    T1CONbits.TSIDL = 0; //continue timer even when device is in idle mode.
    T1CONbits.TGATE = 0;// --> when TCS = 0 this bit is ignored
    T1CONbits.TCKPS1 = 2; // 0=1:1, 1=1:8, 2=1:64, 3=1:256
    //T1CONbits.TSYNC = 0 --> Ignored when TCS=0
    T1CONbits.TCS = 0; //Internal clock. (FOSC/2)
    PR1 = (2625);//was 5208/2
    
    IPC0bits.IC1IP = 1; //set interrupt priority
    IFS0bits.T1IF = 0; //clear the timer1 interrupt flag
    IEC0bits.T1IE = 1; //Enable timer1 interrupt
    
    T1CONbits.TON = 1; //Timer ON
}


void InitUART(void)
{
    //U1MODEbits.USIDL = 0; //operate in idle mode
    //U1MODEbits.IREN = 0; // IrDA encoder/decoder bit. 0=diesabled
    //U1MODEbits.RTSMD = 1; // 1=Simplex mode. 0= flow control mode (start simple with no flow control)
    //U1MODEbits.UEN = 0; // Tx/Rx are enabled and used. CTS/RTS are controlled by port latches
    //U1MODEbits.WAKE = 0; // wake up when start bit is detected is disabled.
    //U1MODEbits.LPBACK = 0; // loopback mode disabled. Use for debugging??
    //U1MODEbits.URXINV = 0; // UxRX idel state = 1???
    
    U1MODEbits.ABAUD = 0; //Auto baud rate disabled. Requires a "Sync Field"
    U1MODEbits.BRGH = 0; // 0=low speed baud rate
    U1MODEbits.PDSEL = 0; // 3=no parity??
    U1MODEbits.STSEL = 0; //1 stop bit
    
    U1BRG = 259;
    
    U1STAbits.UTXISEL0 = 0; //Interrupt after one Tx character 
    U1STAbits.UTXISEL1 = 0;
    
    //IPC3bits.U1TXIP = 1; // Interrupt Tx Priority = 1??
    
    //IEC0bits.U1TXIE = 1; //Enable UART Interrupt
    
    U1MODEbits.UARTEN = 1; // Enable UART
    
    U1STAbits.UTXEN = 1; //Enable UART Tx
    DirtyDelay(1);
    
}

int main(void) 
{
 /* **************Configure PLL for FOSC=80MHz using 7.37MHz FRC*****************/
         PLLFBD = 41; // M = 43
         CLKDIVbits.PLLPRE  = 0;  // N1 = 2
         CLKDIVbits.PLLPOST = 0;  // N2 = 2   
 /*******************************************************************************/    
  
/***************Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)************/
         
     __builtin_write_OSCCONH( 0x01 );
     __builtin_write_OSCCONL( 0x01 );
  
      // Wait for Clock switch to occur
     while ( OSCCONbits.COSC != 0b001 );
 
     // Wait for PLL to lock
     while ( OSCCONbits.LOCK != 1 );
     
/*****************************************************************************************/


    
     

     
     
     
 
     //OSCCON = 70;// First step in unlocking peripheral pins
     //OSCCON = 87;// Second step in unlocking peripheral pins
     //OSCCONbits.IOLOCK = 0;// Clear the IOLOCK bit. This actually unlocks the pins
     //RPOR0bits.RP0R0 = 3; //RP0 --> UART1 Tx??
     //OSCCONbits.IOLOCK = 1;//Set the IOLOCK to relock the pin
       
       InitPWM(); 
       InitTimer1();
       InitOutPutPins();
       InitUART();
       
//     PPSUnLock;
//     PPSOutput(OUT_FN_PPS_U1TX,OUT_PIN_PPS_RP0); //Sets RP0 to UART Tx
//     PPSLock;  
       
     while(1)
     
     {

        //T1CONbits.TON = 1;
      //CheckPWM();
      //PWMOnAllSwitches();
      //PWMOnOneSide();
        //PWMdown();
        //PWMLowFreqBottom(); 
        //SendData();
       //  PWMLowFreqBottom2();

       
         
     } 
       return(0);
 }



void SendData(void)
{
     
    DirtyDelay(1);// need a delay before and after writing or there will be com errors
    U1TXREG = 0x65;// test number
    DirtyDelay(1);// need a delay before and after writing or there will be com errors
 
}
void CheckPWM(void)
{
    
    //IOCON1bits.OVRDAT1 = 1;
    //IOCON1bits.OVRDAT0 = 0;
    //IOCON1bits.OVRENH = 1;
    //IOCON1bits.OVRENL = 1;
    
    

   
    //PDC1 = PTPER/2;
    //PDC2 = PTPER/2;
   // PDC1 = (PHASE1/2);
    //SDC1 = (SPHASE1/3);
    
    //SDC1 = (SPHASE1/4);
    //SDC2 = (SPHASE2/5);
    
}


void PWMLowFreqBottom (void) // this PWM function does not operate in complementary mode

{
    

          i=0;
         
         DirtyDelay(500);
         LATBbits.LATB9 = low;
         LATBbits.LATB10 = high;
        DirtyDelay(500);
           
           while(i<124)// was i<259
         {
             duty = sinetable2[i];
             PDC1 = duty;
             i++;
             //T1CONbits.TON = 1;
             DirtyDelay(330);// this delay needs to be tied to the pwm module somehow

         }
        
         
         PDC1 = 0;
         PDC2 = 0;
         LATBbits.LATB10 = low;
         LATBbits.LATB9 = low;
         DirtyDelay(5000);
        
        // DirtyDelay(50);
           
           
           DirtyDelay(500);
           LATBbits.LATB10 = low;
           LATBbits.LATB9 = high;
           DirtyDelay(500);   
           
           // i=0;
           while(i>0)
         {
             duty = sinetable2[i];
             PDC2 = duty;
             i--;
             //T1CONbits.TON = 1;
             DirtyDelay(330);// this delay needs to be tied to the pwm module somehow(375))

         }
            
            
         PDC1 = 0;
         PDC2 = 0;
         LATBbits.LATB10 = low;
         LATBbits.LATB9 = low;
         DirtyDelay(5000);// 5000 provides roughly 1.15ms delay
 

}

void PWMOnAllSwitches(void)
 {
  
    
            IOCON1bits.OVRENL = 0;
            IOCON2bits.OVRENL = 1;
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            //__builtin_nop();
            //__builtin_nop();
            //__builtin_nop();
           IOCON1bits.OVRENH = 0; 
           IOCON2bits.OVRENH = 1; 

        
           i=0;
           while(i<124)// was i<259
         {
             duty = sinetable2[i];
             PDC1 = duty;
             i++;
             //T1CONbits.TON = 1;
             //AnotherDirtyDelay();// this delay needs to be tied to the pwm module somehow
             DirtyDelay(375);
         }
           
               
           
           //PDC1=0;
            IOCON2bits.OVRENH = 0;
            IOCON1bits.OVRENH = 1;
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            __builtin_nop();
            //__builtin_nop();
            //__builtin_nop();
           // __builtin_nop();
            IOCON2bits.OVRENL = 0;
            IOCON1bits.OVRENL = 1;
  
           
          i=0;
           while(i<124)
         {
             duty = sinetable2[i];
             PDC2 = duty;
             i++;
             //T1CONbits.TON = 1;
             //AnotherDirtyDelay();// this delay needs to be tied to the pwm module somehow
             DirtyDelay(375);

         }
          
         
  
 }


/*
void PWMOnOneSide(void)
//This is used in independent mode. Use pdc register to change duty cycle for PWMH and sdc for duty cycle for PWML
{
    i=0;
          // PDC2 = 106;
           while(i<124)// was i<259
         {
             duty = sinetable2[i];
             
             PDC1 = duty;
             //PDC2 = duty;
             i++;
             //T1CONbits.TON = 1;
             AnotherDirtyDelay();// this delay needs to be tied to the pwm module somehow
          }      
    
    
    
           i = 124; 
           while(i<249)// was i<259
         {
             duty = sinetable2[i];
             
             SDC1 = duty;
             //PDC2 = duty;
             i++;
             //T1CONbits.TON = 1;
             AnotherDirtyDelay();// this delay needs to be tied to the pwm module somehow
          }      
    
    
    
}

*/


void DirtyDelay3(void)
{
     {
     j=0;
     
     while(j<100) //while(j<630) //roughly 0.254ms delay. 315=~60hz
     {
         j++;
     }
 }
}


void DirtyDelay(time)
{
    j=0;
     {
        
     //j=0;
     
     while(j<time) //while(j<630) //roughly 0.254ms delay. 315=~60hz
     {
         j++;
     }
 }
}
 void AnotherDirtyDelay(void)
 {
     j=0;
     
     while(j<315) //while(j<630) //roughly 0.254ms delay. 315=~60hz
     {
         j++;
     }
 }
void __attribute__((interrupt, no_auto_psv))_PWM1Interrupt( void )
{
    LATBbits.LATB9 = ~LATBbits.LATB9;
    LATBbits.LATB10 = ~LATBbits.LATB10;
    IFS5bits.PWM1IF = 0;// clearing PWM1 interrupt flag
}
 
/*Timer1 Interrupt Service Routine*/
void __attribute__((interrupt, auto_psv))_T1Interrupt( void )
{   
    
    PwmDutyUpdate();
        
    IFS0bits.T1IF = 0; // clear interrupt flag
 
}

void __attribute__((interrupt, auto_psv))_U1TXInterrupt( void )
{
    IFS0bits.U1TXIF = 0; //Clear Flag
    U1TXREG = 'B'; //Transmit character
    LATBbits.LATB9 = ~LATBbits.LATB9;
}
