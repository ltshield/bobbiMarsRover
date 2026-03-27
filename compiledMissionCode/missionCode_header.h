/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

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



#pragma config FNOSC = FRCDIV // 8MHz oscillator
#pragma config OSCIOFNC = OFF

// 1 : 1/8, 2: 1/4, 4: 1/2, 8: 1
#define FACTOR 2

#define QRD_THRESHOLD 2048
#define QRD_BALL_THRESHOLD 1000

// decreasing speeds her up (runs fine at 76)
#define LINE_FOLLOWING_SPEED 50 // 1/(8MHz/(2*64)) -> (0.5/400)/0.000016 -1 = 77

// best range seems to be .3-.4? .5 worked too but .3 was maybe too sensitive
#define SHARP_THRESHOLD_LEFT .375
#define SHARP_THRESHOLD_FRONT 1
#define QTR_TURN 1000/FACTOR
//#define CANYON_SPEED 3088
#define CANYON_SPEED 25

#define QRD_CENTER ADC1BUF4
#define QRD_LEFT ADC1BUF13
#define QRD_RIGHT ADC1BUF14
#define QRD_BALL_RETURN ADC1BUF9

#define BC_FORWARD 700/FACTOR
#define BC_BACKWARD 800/FACTOR
#define BC_SERVO_START 95
#define BC_SERVO_END 64


// configs analog pins
void config_ad(void)
// OC2 PWM interrupt 
void _ISR _OC2Interr
// configs before ADC 
void pre_adc_pin_config(void)
// config pins after ADC