static unsigned char MAX_READING;
    static unsigned char BEST_SERVO;
    MAX_READING = 0;
    BEST_SERVO = 0;
    static int STOPPED;
    STOPPED = 0;
    
    while(1) {
        
        // move servo up until infrared sensor passes threshold, then shoot laser
        while (OC1R < 130 && !STOPPED) {
            if (ADC1BUF0 > MAX_READING) {
                MAX_READING = ADC1BUF0;
                BEST_SERVO = OC1R;
            }
            OC1R++;
        }
        
        STOPPED = 1;
        OC1R = BEST_SERVO;
        
        // Laser on
        _LATB9 = 1;
        }
    
    return 0;
}
