/*
sketch for the karaburan motor controller
history:
24-jun-2024 : initial version intergating compass and pwm controller with hard-coded mission
03-jul-2024 : add API via serial i/f
02-nov-2024 : change outputs to drive LED on a second HW (dummy boat with LEDs instead of motors)
*/

//board Arduino Nano
//sensor 
//QMC5883L
/*
QMC5883L   ->   Arduino Nano
===========================
VCC        ->   3.3V / 5V
GND        ->   GND
SDA        ->   A4
SCL        ->   A5
DRDY       ->   D2 (optional)
*/

//motor A
/*
via H bridge  IBT-4 module  powered separately, control pins being:
IN1 --> pin9
IN2 --> pin10
*/

//motor B
/*
via H bridge  IBT-4 module  powered separately, control pins being:
IN1 --> pin3
IN2 --> pin11
*/

//serial via USB
/*
115200 baud, see Yaml file for details, API list here below:
POST PWM 
POST SPEED
POST MOVE
POST NAV
POST WD
POST CALIB
GET PWM
GET SPEED
GET NAV
GET CALIB
GET HEADING
GET WD
*/

// function to get sign of a value
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

#include <QMC5883LCompass.h>    // library required for the compass
#include <Wire.h>               // required?

// Define motor control pins
const int motorAPin1 = 9;       //green led//pin 9 on boat; // Timer1 output A
const int motorAPin2 = 10;      //red led//pin 10 on boat; // Timer1 output B
const int motorBPin1 = 3;       //led//pin 3 on boat;
const int motorBPin2 = 11;      //led//pin 11 on boat;
const int relayPin = 17;

// Define compass DRDY pin
const int drdyPin = 2;

// Constants for control
const float Kp = 0.0625 * 180.0 / 255.0;        // Proportional gain for heading correction
const int targetFrequency = 30000;      // 30kHz
const int MOVE_DURATION_MS = 4000;      // move during 4sec when receiving a move command


// Variables
int targetHeading = 0;          // Command direction
//unsigned long duration = 0;

int speedCommand = 120;         // Speed command (0 to 255)
int maxPWMA = 255;              // 50% of 255
int maxPWMB = 255;              // 50% of 255
int pwm_tmp = 0;
bool motor_enable = false;
int seconds = 0;
int pwmA = 0;                   //target PWM motor A
int internal_pwmA = 0;          //actual PWM motor A
int pwmB = 0;                   //target PWM motor B
int internal_pwmB = 0;          //actual PWM motor B
int headingError;
int counter = 0;
bool auto_nav = false;
bool mode_pwm = false;
struct FloatArray {
    float values[6];
} calibVal;


unsigned long moveEndTime = 0;
unsigned long EndTimeout = 0;
unsigned long previousMillis = 0;       // Will store last time the function was called
unsigned long timeLeft = 0;


enum State {
    go_straight,
    slight_left,
    turn_left,
    go_left,
    slight_right,
    turn_right,
    go_right,
    go_back,
    stop_engine
};

enum State next_state;
enum State nav_state;


// Compass object
QMC5883LCompass compass;
volatile bool newDataAvailable = false;


void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize motor control pins
    pinMode(motorAPin1, OUTPUT);
    pinMode(motorAPin2, OUTPUT);
    pinMode(motorBPin1, OUTPUT);
    pinMode(motorBPin2, OUTPUT);
    //safe_relay output pin (gate of Open Drain)
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, HIGH);       //control of relay (low=relay open NC==> motor controlled by boat)

    // Initialize Timer1 for 30kHz PWM
    setupTimer1PWM();

    // Initialize compass
    Wire.begin();
    compass.init();
    //compass.setMode(0x01, 0x00, 0x00, 0x40);//(MODE=continous, ODR=10Hz, RNG=2G, OSR=256);
    //compass.setCalibration(-2500, 1750, 1875, -1900, 1847, -2555);

    //offset and scales obatin with Example sketch QMC5883L calibration:
    //compass.setCalibrationOffsets(-759.00, 470.00, -561.00);
    //compass.setCalibrationScales(0.92, 0.90, 1.25);
    //compass.setCalibrationOffsets(-432.00, -711.00, -806.00);
    //compass.setCalibrationScales(1.13, 1.01, 0.89);
    //compass.setCalibrationOffsets(151.00, 1245.00, -738.00);
    //compass.setCalibrationScales(1.18, 0.74, 1.25);
    //use precalibration values to start with:
    //compass.setCalibrationOffsets(-69.00, 523.00, -907.00);
    //compass.setCalibrationScales(0.97, 1.00, 1.03);
    compass.setCalibrationOffsets(294.00, -176.00, -794.00);
    compass.setCalibrationScales(0.90, 0.80, 1.54);

    compass.setSmoothing(4, true);      //set smoothing to max (1..10)

    // Initialize DRDY pin and interrupt
    //pinMode(drdyPin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(drdyPin), dataReadyISR, RISING);
    motor_enable = true;
    digitalWrite(relayPin, LOW);
    Serial.print("Karaburan_motor_drive v0.001 \n OK");
}

void setPWMtarget()
{
    if (mode_pwm == false) {
        //maxPWMA = constrain(speedCommand, 0, 192);
        maxPWMA = constrain(speedCommand, 0, 255);
        //maxPWMB = constrain(speedCommand, 0, 192);
        maxPWMB = constrain(speedCommand, 0, 255);
        switch (nav_state) {
        case go_straight:
            pwmA = maxPWMA;     //constrain(pwmA+5, 0, 80);
            pwmB = maxPWMB;     //constrain(pwmB+5, 0, 50);
            //next_state=go_straight;
            break;
        case slight_left:
            pwmA = maxPWMA;
            pwmB = maxPWMB / 3;
            //next_state=stop_engine;
            break;
        case turn_left:
            pwmA = maxPWMA;
            pwmB = 0;
            //next_state=stop_engine;
            break;
        case go_left:
            pwmA = maxPWMA;
            pwmB = -maxPWMB;
            //next_state=stop_engine;
            break;
        case slight_right:
            pwmA = maxPWMA / 3;
            pwmB = maxPWMB;
            //next_state=stop_engine;
            break;
        case turn_right:
            pwmA = 0;
            pwmB = maxPWMB;
            //next_state=stop_engine;
            break;
        case go_right:
            pwmA = -maxPWMA;
            pwmB = maxPWMB;
            //next_state=stop_engine;
            break;
        case stop_engine:
            pwmA = 0;
            pwmB = 0;
            //next_state=stop_engine;
            break;
        case go_back:
            pwmA = -maxPWMA;    //constrain(pwmA+5, -80, 0);
            pwmB = -maxPWMB;    //constrain(pwmB+5, -50, 0);
            //next_state=go_back;
            break;
        default:
            // code block
            break;
        }
        //correct offset/drift
        pwmA = 1.5 * pwmA;
        pwmB = 1.5 * pwmB;
        //correct wiring polarity motor A  (=motor left) and motor B
        pwmA = -pwmA;
        pwmB = -pwmB;

        if (abs(pwmA) < 20) {
            pwmA = 0;           //pwm value below 20 only whistle but actually do not move propeller
        }
        if (abs(pwmB) < 20) {
            pwmB = 0;           //pwm value below 20 only whistle but actually do not move propeller
        }

        if (motor_enable == false) {
            pwmA = 0;
            pwmB = 0;
        }
    }                           //end pwm_mode is false
}



void updatePWM()
{
    int deltaA = sgn(pwmA - internal_pwmA) * 10;
    int deltaB = sgn(pwmB - internal_pwmB) * 10;

    internal_pwmA = constrain(internal_pwmA + deltaA, -abs(pwmA), abs(pwmA));
    internal_pwmB = constrain(internal_pwmB + deltaB, -abs(pwmB), abs(pwmB));
    if (internal_pwmA < 0) {
        // Apply PWM values to motors _always going forward!
        analogWrite(motorAPin2, 0);
        analogWrite(motorAPin1, -internal_pwmA);
    } else {
        analogWrite(motorAPin2, internal_pwmA);
        analogWrite(motorAPin1, 0);
    }
    if (internal_pwmB < 0) {
        analogWrite(motorBPin2, 0);
        analogWrite(motorBPin1, -internal_pwmB);
    } else {
        analogWrite(motorBPin2, internal_pwmB);
        analogWrite(motorBPin1, 0);
    }
    //Serial.print(" PWM A: ");
    //Serial.print(internal_pwmA);
    //Serial.print(" PWM B: ");
    //Serial.println(internal_pwmB);
    //Serial.println(nav_state);
    //Serial.println();
    //delay(250); // Control loop delay /8
}


void setupTimer1PWM()
{
    // Set Timer1 for 30kHz PWM
    TCCR1A = 0;                 // Clear control register A
    TCCR1B = 0;                 // Clear control register B

    // Set mode 14 (fast PWM, TOP = ICR1)
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    TCCR1A |= (1 << WGM11);

    // Set non-inverting mode for both channels
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

    // Set prescaler to 1 and start the timer
    TCCR1B |= (1 << CS10);

    // Set TOP value for 30kHz frequency
    ICR1 = (16000000 / targetFrequency) - 1;

    // Set initial duty cycle to 0
    OCR1A = 0;
    OCR1B = 0;
}


FloatArray calibrateCompass()
{
    //float calib[6];
    FloatArray data;
    //make sure vehicle moves
    speedCommand = 255;         //128;//max speed
    nav_state = go_left;        //to turn on itself
    setPWMtarget();             // update pwm set point
    //actual smooth change of PWM values, in step of 10 (max 26 steps)
    for (int i = 0; i < 27; i++) {
        updatePWM();
        delay(100);             //wait 100ms to reduce current peak and smoothly change pwm
    }
    //start chip calibration 
    compass.calibrate();        //last about 5 seconds
    data.values[0] = compass.getCalibrationOffset(0);
    data.values[1] = compass.getCalibrationOffset(1);
    data.values[2] = compass.getCalibrationOffset(2);
    data.values[3] = compass.getCalibrationScale(0);
    data.values[4] = compass.getCalibrationScale(1);
    data.values[5] = compass.getCalibrationScale(2);
    compass.setCalibrationOffsets(data.values[0], data.values[1], data.values[2]);
    compass.setCalibrationScales(data.values[3], data.values[4], data.values[5]);
    //set smoothing back to factor (1..10) : (is this needed?)
    compass.setSmoothing(3, true);
    //Serial.println("CALIBRATION DONE.");
    nav_state = stop_engine;
    return data;
}

void handlePWMcommand(String command)
{
    if (motor_enable == true) {
        Serial.println("OK");
        int space1 = command.indexOf(' ', 8);
        int space2 = command.indexOf(' ', space1 + 1);

        int newPwmA = command.substring(space1 + 1, space2).toInt();
        int newPwmB = command.substring(space2 + 1).toInt();

        mode_pwm = true;
        pwmA = newPwmA;
        pwmB = newPwmB;
        Serial.println("OK");
    } else {                    //motor disable
        Serial.println("ERR");
    }
}

void handleSPEEDcommand(String command)
{
    speedCommand = command.substring(11).toInt();
    Serial.println("OK");
}

void handleMOVEcommand(String command)
{
    if (motor_enable == true) {
        Serial.println("OK");
        char moveType = command.charAt(10);
        moveEndTime = millis() + MOVE_DURATION_MS;
        mode_pwm = false;       //reste pwm mode if it was set.
        switch (moveType) {
        case 'S':
            nav_state = stop_engine;
            break;
        case 'F':
            nav_state = go_straight;
            break;
        case 'l':
            nav_state = slight_left;
            break;
        case 'L':
            nav_state = turn_left;
            break;
        case 'r':
            nav_state = slight_right;
            break;
        case 'R':
            nav_state = turn_right;
            break;
        case 'B':
            nav_state = go_back;
            break;
        case 'U':
            nav_state = go_left;
            break;
        }
        Serial.println("OK");
    } else {                    //motor disable
        Serial.println("ERR");
    }
}

void handleNAVcommand(String command)
{
    mode_pwm = false;           //reset pwm mode if it was set.
    if (motor_enable == true) {
        Serial.println("OK");
        // Example implementation; adjust based on actual compass reading and control logic
        int space1 = command.indexOf(' ', 8);
        int space2 = command.indexOf(' ', space1 + 1);

        int duration = command.substring(space1 + 1, space2).toInt();   //in sec
        targetHeading = command.substring(space2 + 1).toInt();  //in deg

        moveEndTime = millis() + duration * 1000;
        // Example: aim for the heading using duration
        // Add actual compass handling logic here
        navigateOnCompass();    //set directly requested settings
        auto_nav = true;        //enable flag for the duration time
    } else {                    //motor disable
        Serial.println("ERR");
    }
}

void handleWDcommand(String command)
{
    int timeout = command.substring(8).toInt();
    if (timeout == 0) {
        EndTimeout = 0;
        Serial.println("OK");
    } else if (timeout > 32 or timeout < 0) {
        Serial.println("ERR value");
    } else {
        // Set a timer for the watchdog
        // test on timeout handle in the main loop
        EndTimeout = millis() + timeout * 1000;
        Serial.println("OK");
    }

}

void handleCALIBcommand()
{
    if (motor_enable == true) {
        mode_pwm = false;
        calibVal = calibrateCompass();  //can take 10sec
        Serial.println("OK");
    } else {                    // motor is not enable, calibration will fail because of vehicle not moving
        Serial.println("ERR");
    }
}

void returnPWMcommand()
{
    Serial.print("{pwma:");
    Serial.print(internal_pwmA);
    Serial.print(",pwmb:");
    Serial.print(internal_pwmB);
    Serial.println("} OK");
}

void returnNAVcommand()
{
    Serial.print("{targetHeading:");
    Serial.print(targetHeading);
    Serial.print(", nav_state:");
    Serial.print(nav_state);
    Serial.print(",timeLeft:");
    Serial.print(timeLeft);
    Serial.print(",auto_nav:");
    Serial.print(auto_nav);
    Serial.println("} OK");
}

void returnCALIBcommand()
{
    Serial.print("{calib:");
    for (int i = 0; i < 5; i++) {
        Serial.print(calibVal.values[i]);
        Serial.print(",");
    }
    Serial.print(calibVal.values[5]);
    Serial.println("} OK");
}

void returnHEADINGcommand()
{
    Serial.print("{heading:");
    Serial.print(getCompassHeading());
    Serial.println("} OK");
}

void returnSPEEDcommand()
{
    Serial.print("{speed:");
    Serial.print(speedCommand);
    Serial.println("} OK");
}

void returnWDcommand()
{
    Serial.print("{enable:");
    Serial.print(motor_enable);
    Serial.println("} OK");
}

void navigateOnCompass()
{
    setPWMtarget();
    updatePWM();
    int currentHeading = getCompassHeading();
    int headingError = targetHeading - currentHeading;  //in degree
    if (headingError > 180) {
        headingError -= 360;
    } else if (headingError < -180) {
        headingError += 360;
    }
    //Serial.print(" heading error: ");
    //Serial.println(headingError);
    // Print debug information
    //Serial.print("Heading: ");
    //Serial.print(currentHeading);

    //Serial.print(" Command A: ");
    //Serial.print(pwmA);
    //Serial.print(" command B: ");
    //Serial.println(pwmB);

    if (headingError > 90) {
        nav_state = go_left;
    } else if (headingError > 30) {
        nav_state = turn_left;
    } else if (headingError > 10) {
        nav_state = slight_left;
    } else if (headingError < -10) {
        nav_state = slight_right;
    } else if (headingError < -30) {
        nav_state = turn_right;
    } else if (headingError < -90) {
        nav_state = go_right;
    } else {
        nav_state = go_straight;
    }
}

int getCompassHeading()
{
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    float heading = atan2(y, x) * 180.0 / PI;
    if (heading < 0) {
        heading += 360;
    }
    return (int) heading;
}

//void dataReadyISR() {
//  newDataAvailable = true;
//}


void loop()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 100) {        //every 100ms 
        // Save the last time the function was called
        previousMillis = currentMillis;
        setPWMtarget();
        updatePWM();            //update smooth PWM with step of 10 every 100ms
        getCompassHeading();    // refresh at 10mms so that smoothing value is up to date when heading requested.
        counter++;              //counter to retrieve second tick for navigation updates
        if (auto_nav == true && (counter % 20) == 0) {  //loop of 2 second
            // Call the function to perform actions every 2 seconds
            navigateOnCompass();
        }
    }

    timeLeft = moveEndTime - currentMillis;
    // Handle move/auto timeout
    if (timeLeft <= 0) {
        nav_state = stop_engine;
        auto_nav = false;
        moveEndTime = 0;
        timeLeft = 0;
        //Serial.println(" time left = 0 ");
    }
    /*
       //motor_enable=true;
       if (EndTimeout == 0 || currentMillis > EndTimeout) {
       //switch relays to boat
       if (motor_enable==true){// to inform timeout only once
       motor_enable=false;
       nav_state=stop_engine;//reset PWMs
       digitalWrite(relayPin, HIGH); 
       Serial.println("ERROR {WD:TIMEOUT}");
       }
       }else{
       motor_enable=true;
       digitalWrite(relayPin, LOW); 
       }
     */

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');  // Read the incoming command

        if (command.startsWith("POST PWM")) {
            handlePWMcommand(command);
        } else if (command.startsWith("POST SPEED")) {
            handleSPEEDcommand(command);
        } else if (command.startsWith("POST MOVE")) {
            handleMOVEcommand(command);
        } else if (command.startsWith("POST NAV")) {
            handleNAVcommand(command);
        } else if (command.startsWith("POST WD")) {
            handleWDcommand(command);
        } else if (command.startsWith("POST CALIB")) {
            handleCALIBcommand();
        } else if (command.equals("GET PWM")) {
            returnPWMcommand();
        } else if (command.equals("GET SPEED")) {
            returnSPEEDcommand();
        } else if (command.equals("GET NAV")) {
            returnNAVcommand();
        } else if (command.equals("GET CALIB")) {
            returnCALIBcommand();
        } else if (command.equals("GET HEADING")) {
            returnHEADINGcommand();
        } else if (command.equals("GET WD")) {
            returnWDcommand();
        }
    }

    // test directiosn
    /*
       counter+=1;
       if ((counter%4)==0){//loop of 1 second
       seconds+=1;
       }
       //nav_state=next_state;
       if (headingError>60){
       nav_state=go_left;
       }else if (headingError<-60){
       nav_state=go_right;
       }else if (headingError>15){
       nav_state=turn_left;
       }else if(headingError<-15){
       nav_state=turn_right;
       }else{
       nav_state=go_straight;
       }

       if (seconds<=25){
       nav_state=go_straight;
       }
       if (seconds>25 and seconds<=30){
       nav_state=turn_right;
       }
       if (seconds>90){
       targetHeading-=180;//make U turn
       }
     */
}
