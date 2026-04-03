
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */



// 1 : 1/8, 2: 1/4, 4: 1/2, 8: 1
#define FACTOR 1

#define QRD_THRESHOLD 2048
#define QRD_BALL_THRESHOLD 1000

// decreasing speeds her up (runs fine at 76)
#define LINE_FOLLOWING_SPEED 76 // 1/(8MHz/(2*64)) -> (0.5/400)/0.000016 -1 = 77

// best range seems to be .3-.4? .5 worked too but .3 was maybe too sensitive
#define SHARP_THRESHOLD_LEFT 0.8
#define SHARP_THRESHOLD_FRONT 1
#define SHARP_LANDER_THRESHOLD 0.8
#define QTR_TURN 950/FACTOR
//#define CANYON_SPEED 3088
#define CANYON_SPEED 76

#define QRD_CENTER ADC1BUF4
#define QRD_LEFT ADC1BUF13
#define QRD_RIGHT ADC1BUF14
#define QRD_FAR_LEFT ADC1BUF1
#define QRD_BALL_RETURN ADC1BUF9

#define PHOTODIODE_SATELLITE ADC1BUF0
#define PHOTODIODE_BALLCOLLECT ADC1BUF12

#define BR_FORWARD 800/FACTOR
#define BR_BACKWARD 800/FACTOR
#define BC_FORWARD 1200/FACTOR
#define BC_BACKWARD 2150/FACTOR

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
    _SMPI = 8;	  // AD1CON2<6:2> -- Every 9th conversion sent (number of channels sampled -1)
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
    // FAR_LEFT QRD
    _CSS1 = 1;
    
    // LEFT SHARP
    _CSS10 = 1;
    // FRONT SHARP
    _CSS11 = 1;
    
    // QRD BALL
    _CSS9 = 1;
    
    // Photodiode Satellite
    _CSS0 = 1;
    
    // Photodiode Ball Pickup
    _CSS12 = 1;
    
    _ADON = 1;    // AD1CON1<15> -- Turn on A/D
}

static int step = 0;

void _ISR _OC2Interrupt(void){
    step += 1;
    _OC2IF = 0;
}

void timer_config() {
    _TON = 1;
    // clock source
    _TCS = 0;
    // prescale is 64
    _TCKPS = 0b10;
    
    _RCDIV = 0b110;
}

void PWM_config() {
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
}

void sensor_config() {
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
    //FAR_LEFT, pin 
    _TRISA1 = 1;
    _ANSA1 = 1;
    
    // IR Sensor Ball Pickup Setup
    _ANSB12 = 1;
    _TRISB12 = 1;
    
    // IR Sensor Satellite
    _ANSA0 = 1;
    _TRISA0 = 1;
}

void motor_config() {
    // B7 = pin 11, to DIR on driver, chooses direction
    _TRISB7 = 0;
    
    // B8 = pin 12, to DIR on driver, choose direction
    _TRISB8 = 0;
    
    // choose PWM period for right wheel
    OC3RS = LINE_FOLLOWING_SPEED;
    OC3R = OC3RS/2;
    
    // choose PWM period for left wheel
    OC2RS = LINE_FOLLOWING_SPEED;
    OC2R = OC2RS/2;
    
    // choose DIR for right wheel
    _LATB7 = 1;
    // choose DIR for left wheel
    _LATB8 = 1;
}


void CI_laser_LED_config() {
    // configure count interrupt
    _OC2IF = 0;
    _OC2IE = 1;
    _OC2IP = 4;
    
    // Laser setup
    _TRISB9 = 0;
}