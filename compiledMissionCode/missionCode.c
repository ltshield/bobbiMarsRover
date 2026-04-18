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
    
    // flags for state transitions and unit testing
    static int DID_FIRST_TURN = 0;
    static int didCanyon = 0;
    static int ballCollected = 0;
    static int ballReturned = 0;
    static int inLander = 0;
    
    // satellite setup
    static float IR_READ_VAL = 0.0;
    static float ALPHA = .8;
    static int SERVO_STOPPED = 0;
    static int MAX_READING = 0;
    static int BEST_SERVO = 85;
    static int SERVO_COUNTER = BR_SERVO_END;

    // LF - Line Following
    // CN - Canyon Navigation
    enum State {Finished, BallCollect, Satellite, LeaveLander, BallReturn, LFTurnLeft, LFDriveForward, LFTurnRight, CNTurnLeft, CNTurnRight, CNDriveForward};
    static enum State state = LeaveLander;
    
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
                
                break;
                
            case LeaveLander:
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // sees white on all three?
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                    }
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
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                        SERVOSTEPS = 0;
                    }
                    
                    inLander = 1;
                }
                
                else if (inLander && SERVOSTEPS > 50) {
                    
                    while (SHARP_FRONT < SHARP_LANDER_THRESHOLD) {
                        SHARP_FRONT = ADC1BUF11*3.3/4095;
                    }
                    
                    OC3R = 0;
                    OC2R = 0;
                    state = Satellite;
                    
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
                else if (SERVOSTEPS > 50 && !didCanyon && QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNDriveForward;
                }
                
                // see white and left sees box and ball has been collected, return ball
                else if (ballCollected && !ballReturned && QRD_CENTER < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = BallReturn;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD) {
                    state = LFTurnRight;
                    SERVOSTEPS = 0;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    state = LFTurnLeft;
                    SERVOSTEPS = 0;
                }
                
                // if center sees black
                else if (QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD) {
                    state = LFTurnRight;
                    SERVOSTEPS = 0;
                }
                else if (QRD_CENTER > QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    state = LFTurnLeft;
                    SERVOSTEPS = 0;
                }
                    
                break;
                
            case LFTurnLeft:
                OC3RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                OC3R = OC3RS/2;
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                
                if (!inLander && QRD_CENTER < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && QRD_FAR_LEFT < QRD_THRESHOLD && didCanyon && ballReturned) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                        SERVOSTEPS = 0;
                    }
                    
                    inLander = 1;
                }
                
                else if (!DID_FIRST_TURN && QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD) {
                    DID_FIRST_TURN = 1;
                    state = LFDriveForward;
                }
                
                else if (DID_FIRST_TURN && !didCanyon && QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNDriveForward;
                    int turnBack = SERVOSTEPS;
                    OC2RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                    OC2R = OC2RS/2;
                    OC3RS = LINE_FOLLOWING_SPEED;
                    OC3R = OC3RS/2;
                    SERVOSTEPS = 0;
                    while (SERVOSTEPS <= turnBack) {}
                }
                
                // if center sees black
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                
                break;
                
            case LFTurnRight:
                OC2RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                if (!inLander && QRD_CENTER < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && QRD_FAR_LEFT < QRD_THRESHOLD && didCanyon && ballReturned) {
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                    }
                    
                    inLander = 1;
                }
                
                else if (!didCanyon && QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNDriveForward;
                    int turnBack = SERVOSTEPS;
                    OC2RS = LINE_FOLLOWING_SPEED;
                    OC2R = OC2RS/2;
                    OC3RS = LINE_FOLLOWING_SPEED*TURN_FACTOR;
                    OC3R = OC3RS/2;
                    SERVOSTEPS = 0;
                    while (SERVOSTEPS <= turnBack) {}
                }
                
                // if center sees black
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD) {
                    state = LFDriveForward;
                }
                
                break;
            
            case Satellite:
                OC3R = 0;
                OC2R = 0;
                OC1R = 75;
                // move servo up until infrared sensor passes threshold, then shoot laser
                while (SERVO_COUNTER < 120 && SERVO_STOPPED == 0) {
//                      IR_READ_VAL = PHOTODIODE_SATELLITE;
                    IR_READ_VAL = IR_READ_VAL*ALPHA + (1-ALPHA)*PHOTODIODE_SATELLITE;
                    if (IR_READ_VAL > MAX_READING) {
                        MAX_READING = IR_READ_VAL;
                        BEST_SERVO = SERVO_COUNTER;
                    }
                    
                    OC1R = SERVO_COUNTER;
                    SERVO_COUNTER++;
                    step = 0;
                    while (step < 50) {}

                }

                SERVO_STOPPED = 1;
                OC1R = BEST_SERVO;
                
                step = 0;
                while (step < 600) {}
                
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
                while (step < QTR_TURN-30) {
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

                while (QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD) {}
                
                step = 0;
                while (step < FORWARD_BEFORE_TURN) {}
                _LATB7 = 0;
                step = 0;
                while (step < PARTIAL_TURN) {}
                _LATB7 = 1;

                if (QRD_CENTER > QRD_THRESHOLD) {
                    state = LFTurnRight;
                }
                else {
                    state = LFDriveForward;
                }
                
                OC1R = OC1R+20;

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
                    while (step < QTR_TURN-20) {
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

                    // drive forward same amount
                    _LATB7 = 1;
                    _LATB8 = 1;

                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                    }
                    else {
                        state = LFDriveForward;
                    }
                }
                else {
                    ballReturned = 1;
                    step = 0;
                    while (step < BR_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATB8 = 0;
                    while (step < QTR_TURN-20) {
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
                    while (step < BALL_WAIT) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

                    // drive forward same amount
                    _LATB7 = 1;
                    _LATB8 = 1;

                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB7 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB7 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnRight;
                    }
                    
                    else {
                        state = LFDriveForward;
                    }
                    
                }
                
                break;
            
            case CNDriveForward:
                // both motors forward
                _LATB8 = 1;
                _LATB7 = 1;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                
                // if you see front but not left, turn left
                if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT < SHARP_CANYON_LEFT) {
                    state = CNTurnLeft;
                    step = 0;
                }
                // if you see front and left, turn right
                else if (SHARP_FRONT > SHARP_THRESHOLD_FRONT && SHARP_LEFT > SHARP_CANYON_LEFT) {
                    state = CNTurnRight;
                    step = 0;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT < SHARP_CANYON_LEFT) {
                    didCanyon = 1;

                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB8 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB8 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnLeft;
                    }
                    SERVOSTEPS = 0;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT > SHARP_CANYON_LEFT) {
                    didCanyon = 1;
                    
                    step = 0;
                    while (step < FORWARD_BEFORE_TURN) {}
                    _LATB7 = 0;
                    step = 0;
                    while (step < PARTIAL_TURN) {}
                    _LATB7 = 1;
                    
                    if (QRD_CENTER > QRD_THRESHOLD) {
                        state = LFTurnRight;
                    }
                    SERVOSTEPS = 0;
                }
                
                break;
                
            case CNTurnLeft:
                // switch direction left wheel
                _LATB8 = 0;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                
                // if step counter runs out
                if (step > QTR_TURN) {
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
                
                // if step counter runs out
                if (step > QTR_TURN) {
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