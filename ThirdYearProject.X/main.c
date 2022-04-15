#include <xc.h>
#include "Init.h"   //this file includes all of the port definitions
#include "Comms.h"  //this file includes comms functions
#include "ADC.h"    //this file includes ADC functions
#include "DAC.h"    //this file includes DAC functions
#include "PWM.h"    //this file includes PWM functions
#include "SPI.h"    //this file includes SPI and DREV8711 functions
#include "Timer.h"  //this file includes timer functions

//CONFIG1
#pragma config FCMEN 	= OFF 		// Fail safe clock disabled
#pragma config IESO 	= OFF  		// Internal/external switch over disable
#pragma config MCLRE    = OFF       // MCLR pin is a digital input
#pragma config WDTE     = OFF       // watch dog off
#pragma config FOSC	 	= HS    	// crystal oscillator

//CONFIG2
#pragma config LVP      = OFF       // low voltage programming disabled

//define constants
#define     STEPPER_MOTOR       1
#define     DC_MOTOR            2

//define strings
const unsigned char WelcomeMessage[] = "\r\n\r\n\ **** Stepper Motor Torque Test Experiment ****\r\n";
const unsigned char OptionMessage1[] = " 1: Lever method\r\n";
const unsigned char OptionMessage2[] = "2: Dual-Motor method\r\n";
const unsigned char OptionSelectMessage[] = "\r\nEnter option number: ";
const unsigned char CRLF[] = "\r\n";

//stepper/DC motor test options list
const unsigned char StepperOptionMessage[] = "\r\n\r\n **** Lever Method ****\r\n";
const unsigned char MotorOptionMessage1[] = " 1: Start motor\r\n";
const unsigned char MotorOptionMessage2[] = " 2: Return to main menu\r\n";
const unsigned char MotorRunningMessage[] = "\r\n Motor running. Enter a to move anticlockwise, d to move clockwise and x to stop: "; // IMPLEMENT THIS

//stepper/DC Motor drive status messages
const unsigned char StepperMotorStatusMessage[] = "\r\n\r\n*** STEPPER MOTOR DRIVE STATUS ***\r\n";
const unsigned char StepperMotorStatusMessage1[] = "\r\n      Direction: ";
const unsigned char StepperMotorStatusMessage2[] = "\r\n  Step interval: ";
const unsigned char Clockwise[] = "CLOCKWISE";
const unsigned char AntiClockwise[] = "ANTICLOCKWISE";

//string error messages
const unsigned char MessageTooLong[] = "\r\n String entered is too long";
const unsigned char MessageNoValue[] = "\r\n No Value Entered";
const unsigned char InvalidNumber[] = "\r\n Value out of range";
const unsigned char TooManyDecimalPoints[] = "\r\n Too many decimal points";
const unsigned char TooLarge[] = "\r\n Value too large";
const unsigned char TooSmall[] = "\r\n Value too small";

//Stepper motor speed message
const unsigned char StepperMotorSpeedMessage[] = "\r\n Enter a step interval in microseconds. Value between 500 and 9999: ";

//global variables
volatile unsigned int GLOBAL_TimerEventCounter = 0;
volatile unsigned int GLOBAL_TimerEventFlag = 0;
volatile unsigned int GLOBAL_MasterTimeOutCounter = 0;
volatile unsigned int GLOBAL_MasterTimeOutFlag = 0;
volatile unsigned char GLOBAL_ResultString[RESULT_STRING_LENGTH];
volatile unsigned char GLOBAL_RxString[RX_STRING_LENGTH];
volatile unsigned int GLOBAL_PWM1_PulseTime;
volatile unsigned int GLOBAL_PWM2_PulseTime;
volatile unsigned int GLOBAL_PWM3_PulseTime;
volatile unsigned int GLOBAL_PWM4_PulseTime;
volatile unsigned int GLOBAL_StepperMotorSpeed;
volatile unsigned int GLOBAL_DirectionStatus;

//list functions
void    DisplaySystemOptionsList(void);
void    DisplayStringError(unsigned int);
void    DisplaySpeedControl(void);
void    LeverMethod(void);
void    DualMotorMethod(void);
void    SetStepperMotorSpeed(void);
void    DisplayStepperMotorStatus(void);

//main function
void main(void) {
    
    unsigned int StringStatus;
    unsigned int Value;
    unsigned int SPIAddress;
    unsigned int SPIValue;

    //wait for PLL to stabilise
    while(OSCSTATbits.PLLR == 0);
    
    InitialisePorts(); 
    InitialiseComms();
    InitialiseADC();
    InitialiseDAC();
    InitialisePWM_10Bit();
    InitialisePWM_16Bit();
    InitialiseTimers();
    InitialiseSPI();
    InitialiseDRV8711();
    
    //enable interrupts
    INTCONbits.PEIE = 1;        //enable peripheral interrupts
    INTCONbits.GIE = 1;         //enable interrupts
    
    
    //main loop
    while(1)
    {
        //display options list
        DisplaySystemOptionsList();
        
        //test for any string entry
        StringStatus = GetString(2,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //string ok
            //convert string to binary value
            Value = StringToInteger(GLOBAL_RxString);
            //test for required action
            switch(Value)
            {
            
                case 1:    //lever method
                    LeverMethod();
                    break;
                    
                case 2:    // dual-motor
                    DualMotorMethod();
                    break;

                default:    //invalid entry
                    SendMessage(InvalidNumber);
                    
            }
        }
    }
    //end of main loop. Should never get to this point
    return;
}

//*********************************************
//test stepper motor.
//this has a secondary HMI for selecting direction, speed and motor on/off
void    LeverMethod(void)
{
    unsigned int Status = 0;
    unsigned int StringStatus;
    unsigned int Value;
    unsigned int MotorStopStatus;
    
    //set for stepper mode
    SetDRV8711_Mode(STEPPER_MODE);
    
    //set direction to clockwise
    GLOBAL_DirectionStatus = 0;

    //run motor control options menu
    while(Status == 0)
    {
        //display status
        DisplayStepperMotorStatus();
        //display options list
        SendMessage(MotorOptionMessage1);
        SendMessage(MotorOptionMessage2);
        SendMessage(OptionSelectMessage);

        //test for any string entry
        StringStatus = GetString(1,GLOBAL_RxString);
        if(StringStatus != STRING_OK)
        {
            //string error
            DisplayStringError(StringStatus);
        }
        else
        {
            //string ok
            //convert string to binary value
            Value = StringToInteger(GLOBAL_RxString);
            //test for required action
            switch(Value)
            {
                case 1:     //switch motor on until character received
                    SendMessage(MotorRunningMessage);
                    //stepper is off therefore turn it on
                    MotorOn();
                    //enable the stepper interrupt timer
                    StepperTimerOn();
                    GLOBAL_StepperMotorSpeed = 2500; // calibrated value for lever, speed is set
                    //wait for character to exit
                    MotorStopStatus = 0; // starts the motor
                    while(MotorStopStatus == 0)
                    {
                        Value = GetChar();
                        if(Value == 0x0061) //a
                        {
                            //to clockwise
                            DRV8711_DIR_WRITE = 0b0;
                            GLOBAL_DirectionStatus = 0;
                        }
                        else if(Value == 0x0064) //d
                        {
                            //to anticlockwise
                            DRV8711_DIR_WRITE = 0b1;
                            GLOBAL_DirectionStatus = 1;
                        }
                        else if (Value == 0x0078) //x
                        {
                            MotorStopStatus = 1; // stops the motor
                        }
                    }
                    //disable the stepper interrupt timer
                    StepperTimerOff();
                    //set the step output to 0
                    DRV8711_STEP_WRITE = 0b0;
                    //switch motor drive off
                    MotorOff();
                    break;

                case 2:     //return to main screen
                    Status = 1;
                    break;

                default:    //invalid entry
                    SendMessage(InvalidNumber);
            }
        }
    }
    //ensure the stepper interrupt timer is off
    StepperTimerOff();
    //set the step output to 0
    DRV8711_STEP_WRITE = 0b0;
    //ensure that motor drive is off
    MotorOff();
    //set stepper direction output to 0 and clockwise
    DRV8711_DIR_WRITE = 0; 
}

//***********************************************
//Dual-Motor Method
void DualMotorMethod(void)
{
    SendMessage(WelcomeMessage);
}

//***********************************************
//Set stepper motor speed. load a value between 500 and 9999
void    SetStepperMotorSpeed(void)
{
    unsigned int    Value;
    unsigned int    StringStatus; 
    
    //send the command string
    SendMessage(StepperMotorSpeedMessage);
    //get the string, maximum 4 characters
    StringStatus = GetString(4,GLOBAL_RxString);
    if(StringStatus != STRING_OK)
    {
        //string error
        DisplayStringError(StringStatus);
    }
    else
    {
        //convert string to binary
        Value = StringToInteger(GLOBAL_RxString);
        //test for value too small
        if(Value < 500)
        {
            DisplayStringError(VALUE_TOO_SMALL);
        }
        else
        {
            //load the speed value into the speed control register
            GLOBAL_StepperMotorSpeed = Value;
        }
    }
}    

//*********************************************
//option list
void    DisplaySystemOptionsList(void)
{
    //list the user options
    SendMessage(WelcomeMessage);
    SendMessage(CRLF);
    SendMessage(OptionMessage1);
    SendMessage(OptionMessage2);
    SendMessage(OptionSelectMessage);
}

//*********************************************
//display stepper motor status
void    DisplayStepperMotorStatus(void)
{
    //send status header message
    SendMessage(StepperMotorStatusMessage);
    
    //send motor direction information
    SendMessage(StepperMotorStatusMessage1);
    if(GLOBAL_DirectionStatus == 0)
    {
        SendMessage(Clockwise);
    }
    else
    {
        SendMessage(AntiClockwise);
    }

    //send motor speed information
    SendMessage(StepperMotorStatusMessage2);
    //convert the integer value into a string
    DecimalToResultString(GLOBAL_StepperMotorSpeed, GLOBAL_ResultString, 4);
    //display the result
    SendString(GLOBAL_ResultString);
}

//*********************************************
//display string error
void    DisplayStringError(unsigned int ErrorValue)
{
    switch(ErrorValue)
    {
        case TOO_LONG:  //string is too long
            SendMessage(MessageTooLong);
            break;
            
        case NO_DATA:  //string has no data
            SendMessage(MessageNoValue);
            break;
            
        case INVALID_STRING:  //string has too many decimal points
            SendMessage(TooManyDecimalPoints);
            break;
            
        case VALUE_TOO_LARGE:  //string exceeds maximum value
            SendMessage(TooLarge);
            break;
            
        case VALUE_TOO_SMALL:  //string exceeds minimum value
            SendMessage(TooSmall);
            break;
            
        //No default
    }
}