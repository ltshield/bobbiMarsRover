# bobbiMarsRover

## B0BB1's Complete Run Video
<p align="center" width="100%">
    <video src="./IMG_4463 2.mov" width="80%" controls></video>
</p>

## B0BB1's Purpose
B0BB1 was a robotic rover created as a class project for BYU's Mechatronics class. B0BB1 was built to autonomously explore the surface of Mars and complete a variety of tasks. These tasks included:
- following GPS waypoints to navigate an unfamiliar environment
- collecting and analyzing a rock sample
- navigating a GPS-denied environment represented by a canyon
- returning to the lander
- transmitting data to a satellite in orbit around Mars

## B0BB1's Project Outcomes
Hands-on experience with:
- PIC24F16KA301 microcontroller configuration and setup for analog and digital
- PWMs
- Timers/Interrupts
- Laser diode
- Stepper motors and respective motor drivers
- Buck converters
- IR Sharp sensors
- QRDs
- Op amps
- Photodiode circuits for IR sensing
- Servo motor
- MOSFET transistor
- Soldering to a protoboard
- Debugging with oscilloscopes and multimeters

## B0BB1's Tasks
Due to lack of funding, B0BB1 was developed and tested on a course meant to mimic the mission requirements of Mars with each task being abstracted as described below:
- **GPS waypoints** were represented by a 1-inch white line on black tiles
- **collecting and analyzing a rock sample** included:
    - detecting the location of a sample collection area by an infrared beacon
    - repositioning to push a section of the rock wall with enough force to dislodge the rock sample (represented by a ping-pong ball)
    - returning to the GPS waypoints
    - following said GPS waypoints until sample deposit bins are detected
    - determining the type of rock (represented by ping-pong ball color)
    - dropping the rock sample in the corresponding deposit bin
- **GPS-denied environment navigation** resulted in the disappearance of the GPS waypoints (ie. the white line) and upright walls that depicted the walls of the canyon that was to be navigated through
- **Data transmission** required the lander to locate and re-enter the lander and detect the vertical location of a satellite above the lander by sensing an IR emitter in the center of the target satellite. Upon locating the vertical location of the satellite, B0BB1 was to turn on a laser thereby representing the transmission of the data that had been gathered during the run.

## B0BB1's Hardware
*Initial CAD Design*
<p>
    <img src="./CAD V1.png" width="368" /> <img src="./CAD V1.2.png" width="300" />
</p>

*Initial Circuit Diagram*
<img width="558" height="567" alt="image" src="https://github.com/user-attachments/assets/12123f23-8d9e-4004-885d-f6f20dd45baa" />

*Final CAD Design*
(CAD Render After)

*Microcontroller Pin Inventory*
<img width="557" height="700" alt="image" src="https://github.com/user-attachments/assets/720da2cb-6159-482b-af24-d1b947bf9fcd" />

## B0BB1's Software
(FSM Diagram)

Implemented Alpha Smoothing Filter for Data Transmission Milestone For Noise Reduction:
IR_READ_VAL = IR_READ_VAL*ALPHA + (1-ALPHA)*PHOTODIODE_SATELLITE;
