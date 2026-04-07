#include "xc.h"
#include <math.h>
#include "missionCode_header.h"
#pragma config FNOSC = FRCDIV // 8MHz oscillator
#pragma config OSCIOFNC = OFF
#pragma config SOSCSRC = DIG

int main(void) {
    timer_config();
    PWM_config();
    sensor_config();
    config_ad();
    motor_config();
    CI_laser_LED_config();
    
    static int didCanyon = 0;
    static int ballCollected = 0;
    static int ballReturned = 0;
    static int inLander = 0;
    
    // satellite set up configs
    static float IR_READ_VAL = 0.0;
    static float ALPHA = .6;
    static int SERVO_STOPPED = 0;
    static int MAX_READING = 0;
    static int BEST_SERVO = BR_SERVO_END;
    static int SERVO_COUNTER = BR_SERVO_END;

    // LF - Line Following
    // CN - Canyon Navigation
    enum State {Finished, ReturnToLine, BallCollect, Satellite, LeaveLander, BallReturn, LFTurnLeft, LFDriveForward, LFTurnRight, CNTurnLeft, CNTurnRight, CNDriveForward};
    static enum State state = CNDriveForward;
    
    // Pause before going to allow microcontroller to start up
    OC2R = 0;
    OC3R = 0;
    step = 0;
    while (step < START_WAIT) {}
    
    while(1) {
        
        double SHARP_FRONT = ADC1BUF11*3.3/4095;
        double SHARP_LEFT = ADC1BUF10*3.3/4095;
        
        switch(state)
        {
            case Finished:
                // Laser on
                _LATB9 = 1;
//                OC1RS = 1249;
//                OC1R = OC1R;
//                while(1) {
//                    OC1RS = 1249;
//                    OC1R = BEST_SERVO+50;
//                }
                break;
                
            case ReturnToLine:
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // sees white on all three?
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN/FACTOR) {}
                    _LATB7 = 0;
                    step = 0;
                    while (step < QTR_TURN) {}
                    _LATB7 = 1;
                    state = LFDriveForward;
                }
                
                break;
                
            case LeaveLander:
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // sees white on all three?
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN/FACTOR) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < QTR_TURN) {}
                    _LATB8 = 1;
                    state = LFDriveForward;
                }
                
                break;
                
            case LFDriveForward:
                _LATB7 = 1;
                _LATB8 = 1;
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                if (!inLander && QRD_CENTER < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && QRD_FAR_LEFT < QRD_THRESHOLD && didCanyon && ballReturned) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN/FACTOR) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < QTR_TURN) {}
                    _LATB8 = 1;
                    inLander = 1;
                }
                
                else if (inLander && SHARP_FRONT > SHARP_LANDER_THRESHOLD){
                    OC3R = 0;
                    OC2R = 0;
                    state = Satellite;
                }
                
                else if (!ballCollected && PHOTODIODE_BALLCOLLECT > BALL_IR_THRESHOLD) {
                    state = BallCollect;
                }
                
                // see all black, and wall on left, go to CNDrive
                else if (!didCanyon && QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
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
                    
                break;
                
            case LFTurnLeft:
                OC3RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                OC3R = OC3RS/2;
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                
                // if center sees black
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                
                break;
                
            case LFTurnRight:
                OC2RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // if center sees black
                if (QRD_CENTER < QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                
                break;
            
            case Satellite:
                OC3R = 0;
                OC2R = 0;
                OC1R = BR_SERVO_END;
                // move servo up until infrared sensor passes threshold, then shoot laser
                while (SERVO_COUNTER < 130 && SERVO_STOPPED == 0) {
                      IR_READ_VAL = PHOTODIODE_SATELLITE;
//                    IR_READ_VAL = IR_READ_VAL*ALPHA + (1-ALPHA)*PHOTODIODE_SATELLITE;
                    if (IR_READ_VAL > MAX_READING) {
                        MAX_READING = IR_READ_VAL;
                        BEST_SERVO = SERVO_COUNTER;
                    }
                    
                    OC1R = SERVO_COUNTER;
                    SERVO_COUNTER++;
                    step = 0;
                    while (step < 100) {}

                }

                SERVO_STOPPED = 1;
                OC1R = BEST_SERVO;
                
                step = 0;
                while (step < 1000) {
                }
                
                state = Finished;
                break;

                
            case BallCollect:
                // move forward, turn left, back up into ball collect, collect ball, wait, move forward, turn right, line following
                ballCollected = 1;
                step = 0;
                while (step < BC_FORWARD) {}
                
                // turn 90 degrees left
                _LATB8 = 0;
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
                while (step < BALL_WAIT) {}
                OC2R = OC2RS/2;
                OC3R = OC3RS/2;

                // drive forward same amount
                _LATB7 = 1;
                _LATB8 = 1;
                state = ReturnToLine;
//                step = 0;
//                while (step < BC_BACKWARD-200) {}
//                // turn 90 degrees left
//                _LATB7 = 0;
//                step = 0;
//                while (step < QTR_TURN) {}
//                _LATB7 = 1;
//                state = LFDriveForward;
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
                    _LATB8 = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    // drop servo to drop ball (31 - 156)
                    while (OC1R > BR_SERVO_END) {OC1R--;}
                    // leave servo down
                    OC2R = 0;
                    OC3R = 0;
                    step = 0;
                    
                    // this causes her to wait for a second for the ball to drop
                    while (step < BALL_WAIT) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

//                    state = LeaveLander;
//                    // drive forward same amount
                    _LATB7 = 1;
                    _LATB8 = 1;
                    step = 0;
                    while (step < BR_BACKWARD) {}
                    step = 0;
                    
                    // turn 90 degrees left
                    _LATB8 = 0;
                    while (step < QTR_TURN) {}
                    step = 0;
                    _LATB8 = 1;
                    state = LFDriveForward;
                    
                }
                else {
                    ballReturned = 1;
                    step = 0;
                    while (step < BR_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATB8 = 0;
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
                    while (step < BALL_WAIT/FACTOR) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

//                    // drive forward same amount
                    _LATB7 = 1;
                    _LATB8 = 1;
                    step = 0;
                    while (step < BR_BACKWARD-50) {}
                    step = 0;
                    // turn 90 degrees left
                    _LATB7 = 0;
                    while (step < QTR_TURN+30) {}
                    step = 0;
                    _LATB7 = 1;
                    state = LFDriveForward;
//                    state = ReturnToLine;
                }
                break;
            
            case CNDriveForward:
                // Don't move. Turn LEDs on.
                // both motors forward
                _LATB8 = 1;
                _LATB7 = 1;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                
                // if you see front but not left, turn left
                if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT < SHARP_THRESHOLD_LEFT) {
                    state = CNTurnLeft;
                    step = 0;
                }
                // if you see front and left, turn right
                else if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNTurnRight;
                    step = 0;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT < SHARP_THRESHOLD_LEFT) {
                    didCanyon = 1;
                    step = 0;
                    
                    // drive forward for a sec, then turn
                    while (step < (FORWARD_BEFORE_TURN)/FACTOR) {}
                    step = 0;
                    state = CNTurnLeft;
                    step = 0;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    didCanyon = 1;
                    
                    // drive forward for a sec, then turn
                    step = 0;
                    while (step < (FORWARD_BEFORE_TURN)/FACTOR) {}
                    step = 0;
                    state = CNTurnRight;
                }
                
                break;
                
            case CNTurnLeft:
                // switch direction left wheel
                _LATB8 = 0;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                if (step > QTR_TURN && didCanyon) {
                    state = LFDriveForward;
                }
                // if step counter runs out
                else if (step > QTR_TURN) {
                    state = CNDriveForward;
                }
                break;
                
            case CNTurnRight:
                // switch direction right wheel
                _LATB7 = 0;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                
                if (step > QTR_TURN && didCanyon) {
                    state = LFDriveForward;
                }
                
                // if step counter runs out
                else if (step > QTR_TURN) {
                    state = CNDriveForward;
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