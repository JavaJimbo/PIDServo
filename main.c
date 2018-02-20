/* ========================================================================================
 * 
 * Winch 18 - For PIC 18F2520 
 * Adapted from Dual Servo
 *
 *  12-12-15: Removed all encoder code. Uses both pot inputs
 *  12-15-15: Works OK
 *      24V supply, PWM_A_MAX 1000, PWM_OFFSET = 200, 
 *      kP_A = 500, kI_A = 2, kD_A = 400, 
 *      MAXSUM_A 300, MAXDER_A 30, 
 *      SLEWING: PWM_STEP 2
 *      DIRECTION_DELAY 20
 *  12-16-15: Swapped motors, changed DIRECTION definitions
 *  12-17-15: short kP_A = 5000, kI_A = 12, kD_A = 4000, PWM_A_MAX 700, no offset
 *          Works well with 1.5" drive wheel and 5.9:1 motor.
 *          PID UPDATE LOOP RATE: 250 Hz, (4 ms between loops)
 * 12-19-15: SMALL SERVO O
 * 12-21-15: Added second pot for two servos. 
 *          Using internal 8 Mhz oscillator & 4x PLL for 32Mhz clock.
 *          PWM B servo = 512, A servo = 1020
 * 7-13-16: MOTOR A: BIG SERVO, MOTOR B: LITTLE SERVO. Big servo divides pot by four and adds offset = 512.
 * 2-19-18: Commented out MOTOR A. Got MOTOR B working at 12.4 volts DC as a servo with pot and HN-GH12-1634T motor
 * =========================================================================================\
 */

#include 	<pic18.h>
#include 	"DELAY16.H"
#include 	<string.h>
#include	<ctype.h>
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>
#include 	<htc.h>

#define ESC 27
#define CR 13
#define BACKSPACE 8

#pragma config IESO = OFF, OSC = INTIO7, FCMEN = OFF, BOREN = OFF, PWRT = ON, WDT = OFF, CCP2MX = PORTC, PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON, DEBUG = OFF, STVREN = OFF, XINST = OFF, LVP = OFF, CP0 =	OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF	

#define STX	0x02
#define ETX	0x03
#define DLE	0x10

#define DIRECTION_A		PORTCbits.RC4
#define DIRECTION_B		PORTCbits.RC5
#define FAULT 			PORTCbits.RC3
#define TESTOUT			LATBbits.LATB0

#define MAXBUFFER 64
#define FALSE 0
#define TRUE !FALSE
#define true TRUE
#define false FALSE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include <math.h>
#include "DELAY16.H"

short value;
unsigned char UARTflag = false;
unsigned char UARTbuffer[MAXBUFFER + 1];
unsigned char displayFlag = true;
#define MAXNUM 16
unsigned char NUMbuffer[MAXNUM + 1];
void initializePorts(void);
void putch(char byte);
short readAD(void);
void ADsetChannel(unsigned char channel);
// short PIDcontrolA(short error);
short PIDcontrol(short error);
void SetDutyPWM_A(unsigned short dutyCycle);
void SetDutyPWM_B(unsigned short dutyCycle);
short windowFilter(short *arrPtr);

#define NUM_WIN_VALUES 7
short servoWindowArray_A[NUM_WIN_VALUES];
short servoWindowArray_B[NUM_WIN_VALUES];

//#define MAXSUM_A 150  
#define MAXSUM_A 1
short errorIntegrator_A[MAXSUM_A];
// #define MAXSUM_B 50
//#define MAXSUM_B 125
//short errorIntegrator_B[MAXSUM_B];

#define MAXDER_A 15   
short errorDerivative_A[MAXDER_A];
// #define MAXDER_B 5
#define MAXDER_B 3
short errorDerivative_B[MAXDER_B];

#define FILTERSIZE_A 128
short commandPot_AFilter[FILTERSIZE_A];

#define FILTERSIZE_B 32
short commandPot_BFilter[FILTERSIZE_B];

unsigned char Timer2Flag = false;
short kP_A = 5000, kI_A = 20, kD_A = 4000;
// short kP_B = 1750, kI_B = 2, kD_B = 900;
// short kP_B = 900, kI_B = 6, kD_B = 900;
short kP_B = 2000, kI_B = 20, kD_B = 6000;

#define PWM_A_MAX 1020 // 700
#define PWM_B_MAX 1000 // 512
#define DOWN 1
#define UP 0

unsigned short PWMoffset = 30;
 
void initializeErrorArrays(void){
unsigned short i;
    for(i=0; i<MAXSUM_A; i++) errorIntegrator_A[0] = 0;
    for(i=0; i<MAXDER_A; i++) errorDerivative_A[i] = 0;
    //for(i=0; i<MAXSUM_B; i++) errorIntegrator_B[0] = 0;
    for(i=0; i<MAXDER_B; i++) errorDerivative_B[i] = 0;
    for (i = 0; i < NUM_WIN_VALUES; i++) servoWindowArray_A[i] = 0;
    for (i = 0; i < NUM_WIN_VALUES; i++) servoWindowArray_B[i] = 0;
    for (i = 0; i < FILTERSIZE_A; i++) commandPot_AFilter[i] = 0;
    for (i = 0; i < FILTERSIZE_B; i++) commandPot_BFilter[i] = 0;
}

#define STANDBY 0
#define RUN 1

void main(void) {
    unsigned char i = 0, j = 0, k = 0, l = 0, m = 0, n = 0, p = 0, q = 0, command, ch;    
    short commandPot_A, commandPot_B, servoPot_A, servoPot_B, PWMout_A = 0, PWMout_B = 0, error;
    long temp, sumFilter_A = 0,  sumFilter_B = 0;
    unsigned char testFlag = FALSE;
    unsigned char mode = TRUE;
             
    OSCCON = OSCCON | 0b01110000;   // Internal RC oscillator
    OSCTUNEbits.PLLEN = 1;          // x4 PLL = 64 Mhz clock
    initializePorts();
    initializeErrorArrays();
    SetDutyPWM_A(0);
    SetDutyPWM_B(0);       
    
    DIRECTION_A = 1;
    DIRECTION_B = 1;
    printf("\r\rTesting PID stuff at 32 Mhz...\r");         

    while (1) {
        if (UARTflag) {
            UARTflag = FALSE;
            
            q = 0;
            command = 0;
            for (p = 0; p < MAXBUFFER; p++) {
                ch = toupper (UARTbuffer[p]);
                if (isalpha(ch)) command = ch;
                else if (ch == ' ') command = ' ';
                putch(ch);
                if (ch == '\r' || ch == ' ')break;
                if ((isdigit(ch) || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) {
                NUMbuffer[q] = '\0';
                value = atoi(NUMbuffer);
            }
            if (command) {
                switch (command) {
                    case 'P':
                        if (q) kP_B = value;
                        break;
                    case 'I':
                        if (q) kI_B = value;
                        break;
                    case 'D':
                        if (q) kD_B = value;
                        break;
                    case 'O':
                        if (q) PWMoffset = value;
                        break;
                    case ' ':
                        if (mode) mode = FALSE;
                        else mode = TRUE;
                        break;
                    case 'M':
                        if (displayFlag){
                            displayFlag = FALSE;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = TRUE;
                            printf("\rDisplay ON");
                        }                            
                    default:
                        break;
                }
                printf("\rkP_B=%d, kI_B=%d, kD_B=%d, OFFSET: %d", kP_B, kI_B, kD_B, PWMoffset);
                command = 0;
            }

            putch('\r');
        }

        if (!mode) SetDutyPWM_B(0);
        else if (Timer2Flag) {
            Timer2Flag = false;        

            if (TESTOUT) TESTOUT = 0;
            else TESTOUT = 1;                              
            
            /*
            if (testFlag){
                TESTOUT = 1;
                testFlag = FALSE;
            }
            else {
                TESTOUT = 0 ;
                testFlag = TRUE;
             }
             */
            
            // TESTOUT = 1;
            // SERVO A - GET COMMAND POT DATA & FILTER IT            
            //ADsetChannel(2);
            //commandPot_A = readAD();                 
            /*
            ADsetChannel(0);
            commandPot_A = readAD();             
            
            commandPot_AFilter[k++] = commandPot_A;
            if (k >= FILTERSIZE_A) k = 0;            
            sumFilter_A = 0;
            for (m = 0; m < FILTERSIZE_A; m++)
                sumFilter_A = sumFilter_A + (long) commandPot_AFilter[m];             
            temp = sumFilter_A / FILTERSIZE_A;            
            commandPot_A = (short) (temp / 4) + 512;           

            
            // GET SERVO A POSITION AND FILTER IT
            ADsetChannel(0);
            servoPot_A = readAD();                              
                                 
            servoWindowArray_A[i++] = servoPot_A;
            if (i >= NUM_WIN_VALUES) i = 0;                        
            servoPot_A = windowFilter(servoWindowArray_A);                                
            
            // GET ERROR, COMPUTE PID CORRECTION, SET PWM
            error = servoPot_A - commandPot_A;
               
            PWMout_A = PIDcontrolA(error);                 
            if (PWMout_A < 0){
                PWMout_A = abs(PWMout_A);                                
                DIRECTION_A = DOWN;                
            }
            else DIRECTION_A = UP;      
            if (PWMout_A > PWM_A_MAX) PWMout_A = PWM_A_MAX;                
            SetDutyPWM_A(PWMout_A);                   
*/                      
            
            // SERVO B - GET COMMAND POT DATA & FILTER IT
            //ADsetChannel(3);
            ADsetChannel(0);
            commandPot_B = readAD();    
                        
            commandPot_BFilter[l++] = commandPot_B;
            if (l >= FILTERSIZE_B) l = 0;            
            sumFilter_B = 0;
            
            for (n = 0; n < FILTERSIZE_B; n++)
                sumFilter_B = sumFilter_B + (long) commandPot_BFilter[n];             
            temp = sumFilter_B / FILTERSIZE_B;            
            commandPot_B = (short) temp; 
                       
            
            // GET SERVO B POSITION AND FILTER IT            
            ADsetChannel(1);
            servoPot_B = readAD();                              
                                    
            servoWindowArray_B[j++] = servoPot_B;
            if (j >= NUM_WIN_VALUES) j = 0;                        
            servoPot_B = windowFilter(servoWindowArray_B);   
            
            // GET ERROR, COMPUTE PID CORRECTION, SET PWM
            error = servoPot_B - commandPot_B;          
            
            // GET ERROR, COMPUTE PID CORRECTION, SET PWM               
            PWMout_B = PIDcontrol(error);                   
            
            if (PWMout_B < 0){
                PWMout_B = abs(PWMout_B);   
                DIRECTION_B = DOWN;                
            }
            else DIRECTION_B = UP;      
            if (PWMout_B > PWM_B_MAX) PWMout_B = PWM_B_MAX;
            SetDutyPWM_B(PWMout_B);                          
            
            //TESTOUT = 0;
        }
    }
}

void initializePorts(void) {
    // Initialize ports	
    ADCON1 = 0b00001010; // Set up Port A for five analog inputs, use VCC and VSS for references.
    ADCON2 = 0b10111111; // Use FRC internal oscillator for A/D clock, right justified result
    ADCON0 = 0b00000000; // Initialize A/D converter, turn it off for now, set for sensorNumber 0

    TRISA = 0b11111111; // All inputs
    TRISB = 0b11111110; // Port B 

    RBPU = 0; // Enable Port B pullups
    TRISC = 0b10000001; // Input: RC7 is RX

    // Set up Timer 2 for 18 kHz PWM, interrupts every 0.888 ms    		
    T2CON = 0x00;
    T2CONbits.T2OUTPS0 = 1; // POstscaler = 1:16
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS3 = 1;
    T2CONbits.T2CKPS0 = 0; // Prescaler = 1:1
    T2CONbits.T2CKPS1 = 0;
    PR2 = 0xFF;
    T2CONbits.TMR2ON = 1; // Start Timer 2

    CCPR1L = 0x80; // Initial duty cycle
    CCP1CON = 0b00001100; // PWM #1 
    CCPR2L = 0x80; // Initial duty cycle
    CCP2CON = 0b00001100; // PWM #2

    CCP1CONbits.DC1B0 = 0; // PWM #1 Bit 0
    CCP1CONbits.DC1B1 = 0; // PWM #1 Bit 1	
    CCP2CONbits.DC2B0 = 0; // PWM #2 Bit 0
    CCP2CONbits.DC2B1 = 0; // PWM #2 Bit 1	

    BRGH = 1; // high speed baud rate	
    // SPBRG = 19; // set the baud rate to 57,600 for 18.432 Mhz clock
    SPBRG = 103; // set the baud rate to 19,200 for 8 Mhz internal clock

    SYNC = 0; // asynchronous 
    SPEN = 1; // enable serial port pins 
    CREN = 1; // enable reception 
    SREN = 0; // no effect 
    TXIE = 0; // Disable TX interrupts 
    RCIE = 1; // Enable RX interrupts 
    TX9 = 0; // 8- or 9-bit transmission 
    RX9 = 0; // 8- or 9-bit reception 
    TXEN = 1; // enable the transmitter 

    INTCON = 0x00; // First, clear all interrupts
    PIE1 = 0; // Clear all peripheral interrupts
    SSPIE = 0; // Disable SSP interrupts

    TXIE = 0; // Disable UART Tx interrupts 
    RCIE = 1; // Enabled UART Rx interrupts

    PEIE = 1; // Enable peripheral interrupts.
    TMR1IE = 0; // Disable timer 1 interrupts.
    TMR2IF = 0;
    TMR2IE = 1; // Enable Timer2 interrupts
    ADIE = 0; // AD interrupts disabled
    GIE = 1; // Enable global interrupts
}

static void interrupt
isr(void) {
    unsigned char dummy, ch;
    static unsigned char i = 0;
    
    static unsigned int Timer2Counter = 0;

    if (OERR == 1) { // If overrun occurs, flush buffer and reset receive enable.		
        CREN = 0;
        CREN = 1;
        dummy = RCREG;
        dummy = RCREG;
    }

    if (RCIF == 1) {
        RCIF = 0;
        ch = RCREG;
        if (ch != 0) {
            if (ch == BACKSPACE) {
                if (i != 0) i--;
                UARTbuffer[i] = '\0';
                while (!TXIF);
                TXREG = ' ';
                while (!TXIF);
                TXREG = BACKSPACE;
            } else if (i < MAXBUFFER) {
                UARTbuffer[i] = toupper(ch);
                i++;
            }
            if ('\r' == ch || ch == ' ') {
                UARTflag = TRUE;
                i = 0;
            }
        }
    }
    if (TMR2IF) {
        TMR2IF = 0;
        Timer2Counter++;                        
        if (Timer2Counter >= 8) {
            Timer2Counter = 0;
            Timer2Flag = true;
           
        }      
        
    }
}

void putch(char byte) {
    while (!TXIF) /* set when register is empty */
        continue;
    TXREG = byte;
    return;
}

// Starts AD conversion for current AD channel

void ADsetChannel(unsigned char channel) {
    ADCON0 = (channel << 2) | 0x03; // Set AD channel. Use +5V & GND for reference.
    ADCON0bits.ADON = 1; // Turn on converter
    ADCON0bits.GO_DONE = 1; // Start acquisition/conversion
    PIR1bits.ADIF = 0; // Make sure interrupt flag is cleared
}



// This accepts a ten bit positive value 0 to PWM_MAX 
// and writes it to PWM 

void SetDutyPWM_B(unsigned short dutyCycle) {
    unsigned short lowBits;
    unsigned short ndutyCycle;
    unsigned char maskReg = 0x00;

    ndutyCycle = dutyCycle;

    CCPR1L = (ndutyCycle >> 2) & 0xFF; // Shift duty cycle down to get 8 MSBs and write to CCPxL register 
    lowBits = (ndutyCycle << 4) & 0b00110000; // Shift up two LSBs and mask off other bits `X
    maskReg = CCP1CON & 0b11001111; // Mask off bits 4 and 5 of CCPCON register
    CCP1CON = maskReg | lowBits; // OR in LSBs     
}

void SetDutyPWM_A(unsigned short dutyCycle) {
    unsigned short lowBits;
    unsigned short ndutyCycle;
    unsigned char maskReg = 0x00;

    ndutyCycle = dutyCycle;

    CCPR2L = (ndutyCycle >> 2) & 0xFF; // Shift duty cycle down to get 8 MSBs and write to CCPxL register 
    lowBits = (ndutyCycle << 4) & 0b00110000; // Shift up two LSBs and mask off other bits `X
    maskReg = CCP2CON & 0b11001111; // Mask off bits 4 and 5 of CCPCON register
    CCP2CON = maskReg | lowBits; // OR in LSBs     
}


/*
#define RUN 1
#define STANDBY 0
#define DIVIDER 256

short PIDcontrolA(short error){
    short PWMout_A;
    long PIDcorrection, lngError;    
    long diffError, pastError;
    static long sumError = 0;    
    static unsigned short i = 0, j = 0, k = 0;        
    long PCorr = 0, ICorr = 0, DCorr = 0;
    //short intPCorr, intICorr, intDCorr;
    
    lngError = (long) error;   
    sumError = sumError + lngError;
    
    errorIntegrator_A[j] = error;
    j++; if (j >= MAXSUM_A) j = 0;           
    sumError = 0;
    for (k = 0; k < MAXSUM_A; k++) 
        sumError = sumError + (short) errorIntegrator_A[k];
    
    errorDerivative_A[i] = error;
    i++; if (i >= MAXDER_A) i = 0;    
    
    pastError = (long) errorDerivative_A[i];
    diffError = lngError - pastError;
    
    PCorr = lngError * (long) kP_A;
    ICorr = sumError * (long) kI_A;
    DCorr = diffError * (long) kD_A;

    PIDcorrection = PCorr + ICorr + DCorr;
    PIDcorrection = PIDcorrection / DIVIDER; 
    if (PIDcorrection > PWM_A_MAX) PIDcorrection = PWM_A_MAX;
    if (PIDcorrection < -PWM_A_MAX) PIDcorrection = -PWM_A_MAX;
    PWMout_A = (short) PIDcorrection;
    
    //intPCorr = (short) (PCorr / DIVIDER);
    //intDCorr = (short) (DCorr / DIVIDER);
    //intICorr = (short) (ICorr / DIVIDER);

    return (PWMout_A);
}
*/

#define DIVIDER 256

 short PIDcontrol(short error){
    short PWMout;
    long PIDcorrection, lngError;    
    long diffError, pastError;
    static long sumError = 0;    
    static unsigned short i = 0, j = 0, k = 0;        
    long PCorr = 0, ICorr = 0, DCorr = 0;
    short intPCorr, intICorr, intDCorr;
    static short DisplayCounter = 25;
    static unsigned char saturationFlag = FALSE;
    
    lngError = (long) error;   
    if (!saturationFlag) sumError = sumError + lngError;    
    //sumError = sumError - errorIntegrator_B[j];    
    //errorIntegrator_B[j] = error;    
    //j++; if (j >= MAXSUM_B) j = 0;           
    
    pastError = (long) errorDerivative_B[i];    
    diffError = lngError - pastError;
    
    errorDerivative_B[i] = error;
    i++; if (i >= MAXDER_B) i = 0;     
    
    PCorr = lngError * (long) kP_B;
    ICorr = sumError * (long) kI_B;
    DCorr = diffError * (long) kD_B;

    PIDcorrection = PCorr + ICorr + DCorr;
    PIDcorrection = PIDcorrection / DIVIDER; 
    if (PIDcorrection > PWM_B_MAX) 
    {
        PIDcorrection = PWM_B_MAX;
        saturationFlag = TRUE;
    }
    else if (PIDcorrection < -PWM_B_MAX) {
        PIDcorrection = -PWM_B_MAX;
        saturationFlag = TRUE;
    }
    else saturationFlag = FALSE;
    
    PWMout = (short) PIDcorrection + PWMoffset;    
    
    intPCorr = (short) (PCorr / DIVIDER);   
    intICorr = (short) (ICorr / DIVIDER);
    intDCorr = (short) (DCorr / DIVIDER);
    
    if (displayFlag){
        if (DisplayCounter) DisplayCounter--;
        else {
            printf("\rERR: %d, P: %d, I %d, D: %d, PWM: %d", error, intPCorr, intICorr, intDCorr, PWMout);
            DisplayCounter = 25;
        }           
    }

    return (PWMout);
}

short readAD(void) {
    short ADresult;
    while (!ADIF); // wait for conversion complete
    ADresult = (short) ADRESL;
    ADresult = ADresult | (ADRESH << 8);
    return (ADresult);
}

// WINDOW FILTER - NOT CURRENTLY USED
// Copies and sorts values in input array *arrPtr,
// After sorting, returns center value.
// This provides and excellent means 
// of filtering out spikes
// from noisy A/D readings.
#define MIDPOINT ((NUM_WIN_VALUES-1)/2)  // Used below by Window Filter

short windowFilter(short *arrPtr) {
    short sortArr[NUM_WIN_VALUES];
    short lowData, highData, midData; 
    unsigned char i, j;

    if (arrPtr == NULL) return (0);

    for (i = 0; i < NUM_WIN_VALUES; i++)
        sortArr[i] = arrPtr[i];

    for (i = 0; i < NUM_WIN_VALUES; i++) {
        lowData = sortArr[i];
        j = i + 1;
        while (j < NUM_WIN_VALUES) {
            highData = sortArr[j];
            if (highData < lowData) {
                sortArr[i] = highData;
                sortArr[j] = lowData;
            }
            j++;
        }
    }
    midData = sortArr[MIDPOINT];
    return (midData);
}

