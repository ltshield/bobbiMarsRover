#include "xc.h"
#include "missionCode_header.h"

static int step = 0;

int main(void) {
    PWM_and_timer_config();
    pre_adc_pin_config();
    
    config_ad();
    
    post_adc_pin_config();
    
    static int didCanyon = 0;
    static int ballReturned = 0;
    
    // LF - Line Following
    // CN - Canyon Navigation
    enum State {LeaveLander, BallReturn, LFTurnLeft, LFDriveForward, LFTurnRight, CNTurnLeft, CNTurnRight, CNDriveForward};
    static enum State state = LeaveLander;
    
    OC2R = 0;
    OC3R = 0;
    step = 0;
    while (step < 250/FACTOR) {}
    
    while(1) {
        
        
        double SHARP_FRONT = ADC1BUF11*3.3/4095;
        double SHARP_LEFT = ADC1BUF10*3.3/4095;
        
        // LEDs
//        _LATB7 = (SHARP_FRONT > SHARP_THRESHOLD_FRONT);
        _LATB7 = didCanyon;
//        _LATB8 = (SHARP_LEFT > SHARP_THRESHOLD_LEFT);
        _LATB8 = (QRD_BALL_RETURN*3.3/4095 > 1);
        
        switch(state)
        {
            case LeaveLander:
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // sees white on all three?
                if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD) {
                    step = 0;
                    while (step < 650/FACTOR) {}
                    _LATA1 = 0;
                    step = 0;
                    while (step < QTR_TURN) {}
                    _LATA1 = 1;
                    state = LFDriveForward;
                }
                
                break;
                
            case LFDriveForward:
                
                // Laser on
                _LATB12 = 1;
                
                _LATA0 = 1;
                _LATA1 = 1;
                
                OC2RS = LINE_FOLLOWING_SPEED;
                OC2R = OC2RS/2;
                OC3RS = LINE_FOLLOWING_SPEED;
                OC3R = OC3RS/2;
                
                // see all black, and wall on left, go to CNDrive
                if (QRD_CENTER > QRD_THRESHOLD && QRD_RIGHT > QRD_THRESHOLD && QRD_LEFT > QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    state = CNDriveForward;
                }
                // see white and left sees box
                else if (!ballReturned && QRD_CENTER < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
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
            
            case BallReturn:
                ballReturned = 1;
                // if White goes low
                if (QRD_BALL_RETURN < QRD_BALL_THRESHOLD) {
                    step = 0;
                    while (step < BC_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATA0 = 0;
                    while (step < QTR_TURN) {
                    }
                    step = 0;
                    // back up to box
                    _LATA1 = 0;
                    while (step < BC_BACKWARD) {}
                    step = 0;
                    // drop servo to drop ball (31 - 156)
                    while (OC1R > BC_SERVO_END) {OC1R--;}
                    // leave servo down
                    OC2R = 0;
                    OC3R = 0;
                    step = 0;
                    while (step < 1000/FACTOR) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

                    // drive forward same amount
                    _LATA0 = 1;
                    _LATA1 = 1;
                    step = 0;
                    while (step < BC_BACKWARD) {}
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
                    while (step < BC_FORWARD) {}
                    step = 0;
                    // turn 90 degrees right
                    _LATA1 = 0;
                    while (step < QTR_TURN) {
                    }
                    step = 0;
                    // back up to box
                    _LATA0 = 0;
                    while (step < BC_BACKWARD) {}
                    step = 0;
                    // drop servo to drop ball (31 - 156)
                    while (OC1R > BC_SERVO_END) {OC1R--;}
                    // leave servo down
                    OC2R = 0;
                    OC3R = 0;
                    step = 0;
                    while (step < 1000/FACTOR) {}
                    OC2R = OC2RS/2;
                    OC3R = OC3RS/2;

                    // drive forward same amount
                    _LATA0 = 1;
                    _LATA1 = 1;
                    step = 0;
                    while (step < BC_BACKWARD) {}
                    step = 0;
                    // turn 90 degrees left
                    _LATA0 = 0;
                    while (step < QTR_TURN) {}
                    step = 0;
                    _LATA0 = 1;
                    state = LFDriveForward;
                }
                break;
            
            case CNDriveForward:
                // Don't move. Turn LEDs on.
                // both motors forward
                _LATA1 = 1;
                _LATA0 = 1;
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
                    while (step < 400/FACTOR) {}
                    step = 0;
                    state = CNTurnLeft;
                }
                
                else if (QRD_CENTER < QRD_THRESHOLD && QRD_RIGHT < QRD_THRESHOLD && QRD_LEFT < QRD_THRESHOLD && SHARP_LEFT > SHARP_THRESHOLD_LEFT) {
                    didCanyon = 1;
                    step = 0;
                    while (step < 400/FACTOR) {}
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
                _LATA0 = 0;
                OC2RS = CANYON_SPEED;
                OC2R = OC2RS/2;
                OC3RS = CANYON_SPEED;
                OC3R = OC3RS/2;
                
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

while (1) {}