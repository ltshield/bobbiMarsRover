#include "xc.h"
#include <math.h>
#pragma config FNOSC = FRCDIV // 8MHz oscillator
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

// 1 : 1/8, 2: 1/4, 4: 1/2, 8: 1
#define FACTOR 1

#define QRD_THRESHOLD 2048
#define QRD_BALL_THRESHOLD 1000

// decreasing speeds her up (runs fine at 76)
#define LINE_FOLLOWING_SPEED 76 // 1/(8MHz/(2*64)) -> (0.5/400)/0.000016 -1 = 77

// best range seems to be .3-.4? .5 worked too but .3 was maybe too sensitive
#define SHARP_THRESHOLD_LEFT .3
#define SHARP_THRESHOLD_FRONT 1
#define QTR_TURN 900/FACTOR
//#define CANYON_SPEED 3088
#define CANYON_SPEED 76

#define QRD_CENTER ADC1BUF4
#define QRD_LEFT ADC1BUF13
#define QRD_RIGHT ADC1BUF14
#define QRD_BALL_RETURN ADC1BUF9

#define PHOTODIODE_SATELLITE ADC1BUF12
#define PHOTODIODE_BALLCOLLECT ADC1BUF0

#define BR_FORWARD 700/FACTOR
#define BR_BACKWARD 800/FACTOR
#define BC_FORWARD 1200/FACTOR
#define BC_BACKWARD 2300/FACTOR

#define BR_SERVO_START 95
#define BR_SERVO_END 64

void config_ad(void)
{
    
    _ADON = 0;    // AD1CON1<15> -- Turn off A/D during config
    
    // AD1CON1 register
    _ADSIDL = 0;  // AD1CON1<13> -- A/D continues in idle mode
    _MODE12 = 1;  // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;    // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;    // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;    // AD1CON1<2> -- Auto sampling

    // AD1CON2 register
    _PVCFG = 0;   // AD1CON2<15:14> -- Use VDD as positive
                  // ref voltage
    _NVCFG = 0;   // AD1CON2<13> -- Use VSS as negative
                  // ref voltage
    _BUFREGEN = 1;// AD1CON2<11> -- Result appears in buffer
                  // location corresponding to channel
    _CSCNA = 1;   // AD1CON2<10> -- Scans inputs specified
                  // in AD1CSSx registers
    _SMPI = 7;	  // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
                  // to buffer (if sampling 10 channels)
    _ALTS = 0;    // AD1CON2<0> -- Sample MUXA only

    _CN2PDE = 1; 
    
    // AD1CON3 register
    _ADRC = 0;    // AD1CON3<15> -- Use system clock
    _SAMC = 0;    // AD1CON3<12:8> -- Auto sample every A/D
                  // period TAD
    _ADCS = 0b00000010;    // AD1CON3<7:0> -- TAD needs to be at least 750 ns. Thus, _ADCS = 0b00000010 will give us the fastest AD clock given a 4 MHz system clock.

    // CENTER QRD
    _CSS4 = 1;
    // LEFT QRD
    _CSS13 = 1;
    // RIGHT QRD
    _CSS14 = 1;
    
    // LEFT SHARP
    _CSS10 = 1;
    // FRONT SHARP
    _CSS11 = 1;
    
    // QRD BALL
    _CSS9 = 1;
    
    // Photodiode Satellite
    _CSS15 = 1;
    
    // Photodiode Ball Pickup
    _CSS0 = 1;
    
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

static int step = 0;

void _ISR _OC2Interrupt(void){
    step += 1;
    _OC2IF = 0;
}

int main(void) {
    _TON = 1;
    // clock source
    _TCS = 0;
    // prescale is 64
    _TCKPS = 0b10;
    
    _RCDIV = 0b110;
    
    //  = PWM from pin 5 to ENB on right wheel
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON1bits.OCM = 0b110;
    OC3CON2bits.OCTRIG = 0;
    
    // B0 = PWM from pin 4 to ENB on left wheel
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON1bits.OCM = 0b110;
    OC2CON2bits.OCTRIG = 0;
    
    // RA6 = PWM from pin 14 to servo
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON1bits.OCM = 0b110;
    OC1CON2bits.OCTRIG = 0;
    
    // PWM period for servo (50 hz)
    OC1RS = 1249;
    // Use INT: Min pulse width 0.5ms (31.25), Max 2.5ms (156.25)
    OC1R = BR_SERVO_START;
    
    // SHARP Sensor Setup
    // FRONT
    _TRISB13 = 1;
    _ANSB13 = 1;
    // LEFT
    _TRISB14 = 1;
    _ANSB14 = 1;
    
    // QRD SETUP
    //CENTER
    _TRISB2 = 1;    // Set as input
    _ANSB2 = 1;
    //LEFT
    _TRISA2 = 1;    // Set as input
    _ANSA2 = 1; 
    //RIGHT
    _TRISA3 = 1;
    _ANSA3 = 1;
    
    // IR Sensor Satellite Setup
    _ANSB4 = 1;
    _TRISB4 = 1;
    
    // IR Sensor Ball Pickup
    _ANSA0 = 1;
    _TRISA0 = 1;
    
    config_ad();
    
    // B7 = pin 11, to DIR on driver, chooses direction
    _TRISB7 = 1;
    
    // A1 = pin 3, to DIR on driver, choose direction
    _TRISA1 = 0;
    _ANSA1 = 0;
    
    // choose PWM period for right wheel
    OC3RS = LINE_FOLLOWING_SPEED;
    OC3R = OC3RS/2;
    
    // choose PWM period for left wheel
    OC2RS = LINE_FOLLOWING_SPEED;
    OC2R = OC2RS/2;
    
    // choose DIR for right wheel
    _LATB7 = 1;
    // choose DIR for left wheel
    _LATA1 = 1;
    
    // configure count interrupt
    _OC2IF = 0;
    _OC2IE = 1;
    _OC2IP = 4;
    
    // Laser setup
    _TRISB9 = 0;
    
    static int didCanyon = 0;
    static int ballCollected = 0;
    static int ballReturned = 0;
    
    // LF - Line Following
    // CN - Canyon Navigation
    enum State {Finished, BallCollect, Satellite, LeaveLander, BallReturn, LFTurnLeft, LFDriveForward, LFTurnRight, CNTurnLeft, CNTurnRight, CNDriveForward};
    static enum State state = LFDriveForward;
    
    // Pause before going to allow microcontroller to start up
    OC2R = 0;
    OC3R = 0;
    step = 0;
    while (step < 250/FACTOR) {}
    
    while(1) {
        
        
        double SHARP_FRONT = ADC1BUF11*3.3/4095;
        double SHARP_LEFT = ADC1BUF10*3.3/4095;
        
        // LEDs
        
        switch(state)
        {
            case Finished:
                break;
                
            case LeaveLander:
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // sees white on all three?
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    step = 0;
                    while (step < 750/FACTOR) {}
                    _LATA1 = 0;
                    step = 0;
                    while (step < QTR_TURN) {}
                    _LATA1 = 1;
                    state = LFDriveForward;
                }
                
                break;
                
            case LFDriveForward:
                _LATB7 = 1;
                _LATA1 = 1;
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // see all black, and wall on left, go to CNDrive
                if (QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNDriveForward;
                }
                
                // see white and left sees box and ball has been collected, return ball
                else if (ballCollected && !ballReturned && QRD_CENTER < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = BallReturn;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD) {
                    state = LFTurnRight;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    state = LFTurnLeft;
                }
                
                // if center sees black
                else if (QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD) {
                    state = LFTurnRight;
                }
                else if (QRD_CENTER > QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    state = LFTurnLeft;
                }
                
                // see all black and already did canyon and ball collect, in lander?
                else if (didCanyon && ballReturned && QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD) {
                    OC3R = 0;
                    OC2R = 0;
                    // maybe move forward some amount to get farther into the lander?
                    state = Satellite;
                }
                
                else if (!ballCollected && PHOTODIODE_BALLCOLLECT > 600) {
                    state = BallCollect;
                }
                    
                break;
                
            case LFTurnLeft:
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                OC2RS = LINE_FOLLOWING_SPEED*4;
                OC2R = OC2RS/2;
                
                // if center sees black
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                break;
                
            case LFTurnRight:
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED*4;
                OC3R = OC3RS/2;
                
                // if center sees black
                if (QRD_CENTER < QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                break;
            
            case Satellite:
                // move servo up until infrared sensor passes threshold, then shoot laser
                while (OC1R < 110 && PHOTODIODE_SATELLITE < 600) {
                    OC1R++;
                }
                
                // Laser on
                _LATB9 = 1;
                state = Finished;
                break;
                
            case BallCollect:
                // move forward, turn left, back up into ball collect, collect ball, wait, move forward, turn right, line following
                ballCollected = 1;
                step = 0;
                while (step < BC_FORWARD) {}
                
                // turn 90 degrees left
                _LATA1 = 0;
                step = 0;
                while (step < QTR_TURN) {
                }
                
                // back up to box
                _LATB7 = 0;
                step = 0;
                while (step < BC_BACKWARD) {}
                OC2R = 0;
                OC3R = 0;
                step = 0;
                while (step < 1000/FACTOR) {}
                OC2R = OC2RS/2;
                OC3R = OC3RS/2;

                // drive forward same amount
                _LATB7 = 1;
                _LATA1 = 1;
                step = 0;
                while (step < BC_BACKWARD) {}
                // turn 90 degrees left
                _LATB7 = 0;
                step = 0;
                while (step < QTR_TURN) {}
                _LATB7 = 1;
                state = LFDriveForward;
                break;
                
            case BallReturn:
                ballReturned = 1;
                // if White goes low
                if (QRD_BALL_RETURN < QRD_BALL_THRESHOLD) {
                    step = 0;
                    while (step < BR_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATB7 = 0;
                    while (step < QTR_TURN) {
                    }
                    step = 0;
                    // back up to box
                    _LATA1 = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    // drop servo to drop ball (31 - 156)
                    while (OC1R > BR_SERVO_END) {OC1R--;}
                    // leave servo down
                    OC2R = 0;
                    OC3R = 0;
                    step = 0;
                    
                    // this causes her to wait for a second for the ball to drop
                    while (step < 1000/FACTOR) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

                    // drive forward same amount
                    _LATB7 = 1;
                    _LATA1 = 1;
                    step = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    
                    // turn 90 degrees left
                    _LATA1 = 0;
                    while (step < QTR_TURN) {}
                    step = 0;
                    _LATA1 = 1;
                    state = LFDriveForward;
                }
                else {
                    ballReturned = 1;
                    step = 0;
                    while (step < BR_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATA1 = 0;
                    while (step < QTR_TURN) {
                    }
                    step = 0;
                    // back up to box
                    _LATB7 = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    // drop servo to drop ball (31 - 156)
                    while (OC1R > BR_SERVO_END) {OC1R--;}
                    // leave servo down
                    OC2R = 0;
                    OC3R = 0;
                    step = 0;
                    while (step < 1000/FACTOR) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

                    // drive forward same amount
                    _LATB7 = 1;
                    _LATA1 = 1;
                    step = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    // turn 90 degrees left
                    _LATB7 = 0;
                    while (step < QTR_TURN) {}
                    step = 0;
                    _LATB7 = 1;
                    state = LFDriveForward;
                }
                break;
            
            case CNDriveForward:
                // Don't move. Turn LEDs on.
                // both motors forward
                _LATA1 = 1;
                _LATB7 = 1;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                // if you see front but not left, turn left
                if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT < SHARP_THRESHOLD_LEFT) {
                    state = CNTurnLeft;
                }
                // if you see front and left, turn right
                else if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNTurnRight;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT < SHARP_THRESHOLD_LEFT) {
                    didCanyon = 1;
                    step = 0;
                    
                    // drive forward for a sec, then turn
                    while (step < 750/FACTOR) {}
                    step = 0;
                    state = CNTurnLeft;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    didCanyon = 1;
                    
                    // drive forward for a sec, then turn
                    step = 0;
                    while (step < 750/FACTOR) {}
                    step = 0;
                    state = CNTurnRight;
                }
                
                break;
                
            case CNTurnLeft:
                // switch direction left wheel
                _LATA1 = 0;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                step = 0;
                if (step > QTR_TURN && didCanyon) {
                    state = LFDriveForward;
                    step = 0;
                }
                // if step counter runs out
                else if (step > QTR_TURN) {
                    state = CNDriveForward;
                    step = 0;
                }
                break;
                
            case CNTurnRight:
                // switch direction right wheel
                _LATB7 = 0;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                
                step = 0;
                if (step > QTR_TURN && didCanyon) {
                    state = LFDriveForward;
                    step = 0;
                }
                
                // if step counter runs out
                else if (step > QTR_TURN) {
                    state = CNDriveForward;
                    step = 0;
                }
                break;
                
            default:
                OC3RS = 0;
                OC3R = 0;
                OC2RS = 0;
                OC2R = 0;
        }
    }
    
    return 0;
}