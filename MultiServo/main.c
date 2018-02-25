/* ========================================================================================
 * 
 * MultiServo - For PIC 18F2520 
 * Adapted from Dual Servo
 * Compiled with XC8 V1.33
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
 * 02-25-18: 
 * =========================================================================================\
 */

#include	<ctype.h>
#include	<stdlib.h>
#include	<stdio.h>
#include    <xc.h>

#define ESC 27
#define CR 13
#define BACKSPACE 8
// OSC = INTIO7
#pragma config IESO = OFF, OSC = HS, FCMEN = OFF, BOREN = OFF, PWRT = ON, WDT = OFF, CCP2MX = PORTC, PBADEN = OFF, LPT1OSC = OFF, MCLRE = ON, DEBUG = OFF, STVREN = OFF, XINST = OFF, LVP = OFF, CP0 =	OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF	

#define STX	0x02
#define ETX	0x03
#define DLE	0x10

#define DIRECTION_A		PORTCbits.RC4
#define DIRECTION_B		PORTCbits.RC5
#define FAULT 			PORTCbits.RC3
#define TESTOUT			LATBbits.LATB0

#define MAXBUFFER 8
#define FALSE 0
#define TRUE !FALSE
#define true TRUE
#define false FALSE

#define NUMSERVOS 2



struct PIDtype
{
    long sumError;
    short derIndex;
    unsigned char saturationFlag;
    short kP;
    short kI;
    short kD;
    unsigned short PWMoffset;
} PID[NUMSERVOS];

short value;
unsigned char UARTflag = false;
unsigned char UARTbuffer[MAXBUFFER + 1];
unsigned char displayFlag = FALSE;
#define MAXNUM 16
unsigned char NUMbuffer[MAXNUM + 1];
void initializePorts(void);
void putch(char byte);
short readAD(void);
void ADsetChannel(unsigned char channel);
short PIDcontrol(short servoID, short error);
 
void setServoPWM (unsigned char servoID, short PWMout);
void SetDutyPWM_A(unsigned short dutyCycle);
void SetDutyPWM_B(unsigned short dutyCycle);
short windowFilter(short servoID);
short commandPot = 0;

#define NUM_WIN_VALUES 7
short servoWindowArray[NUMSERVOS][NUM_WIN_VALUES];

#define MAXDER 3
short errorDerivative[NUMSERVOS][MAXDER];

unsigned char Timer2Flag = false;

#define PWM_MAX 1000 // 512
#define DOWN 1
#define UP 0

#define SERVO_A 0
#define SERVO_B 1
 
void initializeErrorArrays(void){
unsigned short i, j;

    for(j = 0; j < NUMSERVOS; j++){
        for(i = 0; i < MAXDER; i++) errorDerivative[j][i] = 0;
    }

    for(j = 0; j < NUMSERVOS; j++){
        for (i = 0; i < NUM_WIN_VALUES; i++) servoWindowArray[j][i] = 0;
    }
}

unsigned char channel[NUMSERVOS] = {0, 1};
unsigned char servoIDselect = 1;

short readServoPot(short servoID, unsigned char filterIndex);

#define STANDBY 0
#define RUN 1

void main(void) {
    unsigned char i = 0, filterIndex = 0, servoID = 0, windowFilterIndex = 0, p = 0, q = 0, command, ch;    
    short servoPot, PWMout = 0, error;        
    unsigned char mode = TRUE;  
    unsigned char runEnable = FALSE;    
#define FILTERSIZE 32
    short arrFilter[FILTERSIZE];
    long filterSum = 0;
    
    for(i = 0; i < FILTERSIZE; i++) arrFilter[i] = 0;
    
    for (int i = 0; i < NUMSERVOS; i++)
    {
        PID[i].sumError = 0;
        PID[i].derIndex = 0;
        PID[i].saturationFlag = FALSE;
        PID[i].kP = 2000;
        PID[i].kI = 20;
        PID[i].kD = 6000;
        PID[i].PWMoffset = 30;
    }
             
    // OSCCON = OSCCON | 0b01110000;   // Internal RC oscillator
    // OSCTUNEbits.PLLEN = 1;          // x4 PLL = 64 Mhz clock
    initializePorts();
    initializeErrorArrays();
    SetDutyPWM_A(0);
    SetDutyPWM_B(0);       
    
    DIRECTION_A = 1;
    DIRECTION_B = 1;
    printf("\r\rTesting PID RUN mode.\r");         
    
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
                        if (q) PID[servoIDselect].kP = value;
                        break;
                    case 'I':
                        if (q) PID[servoIDselect].kI = value;
                        break;
                    case 'D':
                        if (q) PID[servoIDselect].kD = value;
                        break;
                    case 'O':
                        if (q) PID[servoIDselect].PWMoffset = value;
                        break;
                    case ' ':
                        if (mode){
                            mode = FALSE;
                            printf("\rHALT");
                        }
                        else {
                            mode = TRUE;
                            printf("\rRUN");
                        }
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
                        printf("\rCommand: %c", command);
                        break;
                }
                printf("\rkP=%d, kI=%d, kD=%d, OFFSET: %d", PID[servoIDselect].kP, PID[servoIDselect].kI, PID[servoIDselect].kD, PID[servoIDselect].PWMoffset);
                command = 0;
            }

            putch('\r');
        }
        
        
        if (!mode) SetDutyPWM_B(0);
        else if (Timer2Flag) {
            Timer2Flag = false;        

            if (TESTOUT) TESTOUT = 0;
            else TESTOUT = 1;      
            
            // GET COMMAND POT DATA & FILTER IT                
            ADsetChannel(2);
            filterSum = filterSum - (long) arrFilter[filterIndex];
            arrFilter[filterIndex] = readAD();
            filterSum = filterSum + (long) arrFilter[filterIndex];
            filterIndex++;
            if (filterIndex >= FILTERSIZE)
            {
               runEnable = TRUE;
               filterIndex = 0;
            }
            commandPot = (short)(filterSum / FILTERSIZE);
            
            if (runEnable)
            {
                for (servoID = 1; servoID < NUMSERVOS; servoID++){                       
            
                    // GET SERVO POSITION AND FILTER IT            
                    servoPot = readServoPot(servoID, windowFilterIndex);                
            
                    // GET ERROR, COMPUTE PID CORRECTION, SET PWM
                    error = servoPot - commandPot;          
            
                    // GET ERROR, COMPUTE PID CORRECTION, SET PWM               
                    PWMout = PIDcontrol(servoID, error);
            
                    // Set PWM OUTPUT
                    setServoPWM (servoID, PWMout);
            
                    //TESTOUT = 0;
                }
            }
            windowFilterIndex++; if (windowFilterIndex >= NUM_WIN_VALUES)windowFilterIndex = 0;
        }
    }
}

void setServoPWM (unsigned char servoID, short PWMout)
{
    switch(servoID)
    {
        case SERVO_A:
            if (PWMout < 0)
            {
                PWMout = abs(PWMout);   
                DIRECTION_A = DOWN;               
            }
            else DIRECTION_A = UP;      
            if (PWMout > PWM_MAX) PWMout = PWM_MAX;
                SetDutyPWM_A(PWMout);     
            break;

        case SERVO_B:
            if (PWMout < 0)
            {
                PWMout = abs(PWMout);   
                DIRECTION_B = DOWN;               
            }
            else DIRECTION_B = UP;      
            if (PWMout > PWM_MAX) PWMout = PWM_MAX;
                SetDutyPWM_B(PWMout);     
            break;
        default:
            break;
    }
}

short readServoPot(short servoID, unsigned char filterIndex)
{
    short servoPotValue, servoPotFiltered;
    
    ADsetChannel(channel[servoID]);
    servoPotValue = readAD();    
    servoWindowArray[servoID][filterIndex] = servoPotValue;
    servoPotFiltered = windowFilter(servoID);       
    
    return servoPotFiltered;
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

    // Set up Timer 2 for 19692 kHz PWM and interrupts every 0.809 ms    		
    T2CON = 0x00;
    T2CONbits.T2OUTPS0 = 1; // Postscaler = 1:16
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS3 = 1;
    T2CONbits.T2CKPS0 = 0; // Prescaler = 1:1
    T2CONbits.T2CKPS1 = 0;
    PR2 = 233;
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
    SPBRG = 19; // set the baud rate to 57,600 for 18.432 Mhz clock
    // SPBRG = 103; // set the baud rate to 19,200 for 8 Mhz internal clock

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
    // Timer 2 creates 1236 interrupts per second,
    // or one interrupt every 0.809 milliseconds
    // Dividing 1236 interrupts by 5 gives us 
    // the servo loop period, about 4 milliseconds.
    // So the servo gets updated every 4 milliseconds,
    // or 250 times per second:
    if (TMR2IF) {
        TMR2IF = 0;                
        Timer2Counter++;                        
        if (Timer2Counter >= 5) {
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

#define DIVIDER 256
 short PIDcontrol(short servoID, short error){
    short PWMout;
    long PIDcorrection, lngError;    
    long diffError, pastError;
    long PCorr = 0, ICorr = 0, DCorr = 0;
    short intPCorr, intICorr, intDCorr;
    static short DisplayCounter = 25;
    
    lngError = (long) error;   
    if (!PID[servoID].saturationFlag) PID[servoID].sumError = PID[servoID].sumError + lngError;    
    
    pastError = (long) errorDerivative[servoID][PID[servoID].derIndex];    
    diffError = lngError - pastError;
    
    errorDerivative[servoID][PID[servoID].derIndex] = error;
    PID[servoID].derIndex++; if (PID[servoID].derIndex >= MAXDER) PID[servoID].derIndex = 0;     
    
    PCorr = lngError * (long) PID[servoID].kP;
    ICorr = PID[servoID].sumError * (long) PID[servoID].kI;
    DCorr = diffError * (long) PID[servoID].kD;

    PIDcorrection = PCorr + ICorr + DCorr;
    PIDcorrection = PIDcorrection / DIVIDER; 
    if (PIDcorrection > PWM_MAX) 
    {
        PIDcorrection = PWM_MAX;
        PID[servoID].saturationFlag = TRUE;
    }
    else if (PIDcorrection < -PWM_MAX) {
        PIDcorrection = -PWM_MAX;
        PID[servoID].saturationFlag = TRUE;
    }
    else PID[servoID].saturationFlag = FALSE;
    
    if (PIDcorrection < 0) 
        PWMout = (short) PIDcorrection - PID[servoID].PWMoffset;    
    else PWMout = (short) PIDcorrection + PID[servoID].PWMoffset;
    
    intPCorr = (short) (PCorr / DIVIDER);   
    intICorr = (short) (ICorr / DIVIDER);
    intDCorr = (short) (DCorr / DIVIDER);
    
    if (displayFlag){
        if (DisplayCounter) DisplayCounter--;
        else {
            // printf("\rPOT: %d, ERR: %d, P: %d, I %d, D: %d, PWM: %d", commandPot, error, intPCorr, intICorr, intDCorr, PWMout);
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

short windowFilter(short servoID)
{
    short sortArr[NUM_WIN_VALUES];
    short lowData, highData, midData; 
    unsigned char i, j;
    
    for (i = 0; i < NUM_WIN_VALUES; i++)
        sortArr[i] = servoWindowArray[servoID][i];

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

