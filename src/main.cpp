/**
 * @created: 10/12/2016 15:42
 * @author : Peter Withers <peter@gthb-bambooradical.com>
 * @author : Harmjan Lever <harmjanfl@gmail.com>
 */

#include <NewPing.h>
#include <Servo.h>
#include <PID_v1.h>

#define TRIGGER_PIN_HEIGHT  3
#define ECHO_PIN_HEIGHT     2
//#define TRIGGER_PIN_FORWARD  3
//#define ECHO_PIN_FORWARD     2

//#define TRIGGER_PIN_RIGHT
//#define ECHO_PIN_RIGHT
// @todo: add defines for TRIGGER_PIN_RIGHT and ECHO_PIN_RIGHT once the desired pins have been assigned
// @todo: add left and back sonar definitions
#define MAX_DISTANCE 400

NewPing sonarHeight(TRIGGER_PIN_HEIGHT, ECHO_PIN_HEIGHT, MAX_DISTANCE);

//NewPing sonarForward(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD, MAX_DISTANCE);
//NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

volatile unsigned long throttlePulseLastChangeMs = 0;
volatile unsigned long trimpotPulseLastChangeMs = 0;
volatile unsigned long switchPulseLastChangeMs = 0;
volatile int pulseWidthThrottle = 0;
volatile int pulseWidthTrimpot = 0;
volatile int pulseWidthSwitch = 0;
volatile bool trimpotPinLast = false;
volatile bool switchPinLast = false;

#define THROTTLE_PIN A0
#define THROTTLE_PIN_OUT 13
#define TRIMPOT_PIN 9
#define SWITCH_PIN 8

//#define YAW_PIN A1

//PID variables
double pidSetPoint, pidInput, pidOutput;
PID throttlePID(&pidInput, &pidOutput, &pidSetPoint, 2, 5, 1, DIRECT);

Servo throttleOutputServo;

ISR(PCINT0_vect) {
    if (digitalRead(TRIMPOT_PIN) == 0 && trimpotPinLast) {
        pulseWidthTrimpot = micros() - trimpotPulseLastChangeMs;
        trimpotPinLast = false;
    } else if (digitalRead(TRIMPOT_PIN) == 1 && !trimpotPinLast) {
        trimpotPulseLastChangeMs = micros();
        trimpotPinLast = true;
    }
    if (digitalRead(SWITCH_PIN) == 0 && switchPinLast) {
        pulseWidthSwitch = micros() - switchPulseLastChangeMs;
        switchPinLast = false;
    } else if (digitalRead(SWITCH_PIN) == 1 && !switchPinLast) {
        switchPulseLastChangeMs = micros();
        switchPinLast = true;
    }
}

ISR(PCINT1_vect) {
    if (digitalRead(THROTTLE_PIN) == 0) {
        pulseWidthThrottle = micros() - throttlePulseLastChangeMs;
    } else {
        throttlePulseLastChangeMs = micros();
    }
}

void setup() {
    Serial.begin(57600);
    delay(10);
    cli();
    PCICR |= (1 << PCIE0);
    PCICR |= (1 << PCIE1);

    PCMSK0 |= (1 << PCINT0); // pin 8 switch
    PCMSK0 |= (1 << PCINT1); // pin 9 trimpot
    PCMSK1 |= (1 << PCINT8); // pin A1 throttle

    sei();
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(SWITCH_PIN, INPUT);
    pinMode(TRIMPOT_PIN, INPUT);
    throttleOutputServo.attach(THROTTLE_PIN_OUT);
    pidInput = 60; //arbitrary initial value
    pidSetPoint = 60;
    throttlePID.SetMode(AUTOMATIC);
}

void loop() {
    delay(100);
    //int forwardDistance = sonarForward.ping_cm();
    int verticalDistance = sonarHeight.ping_cm();
    //    int rightDistance = sonarRight.ping_cm(); // @todo
    //Serial.print("forward: ");
    //Serial.print(forwardDistance);
    Serial.print("vertical: ");
    Serial.print(verticalDistance);

//    Serial.print("cm vertical:");
//    Serial.print("(todo)"); // @ todo
//    Serial.print("cm in ");
//    Serial.print(millis());
//    Serial.print("ms ");

    pidInput = verticalDistance;
    throttlePID.Compute();
    Serial.print("PID out: ");
    Serial.print(pidOutput);

    Serial.print(" Throttle in: ");
    Serial.print(pulseWidthThrottle);
    Serial.print("us ");

    Serial.print("Switch in: ");
    Serial.print(pulseWidthSwitch);
    Serial.print("us ");

    Serial.print("Trimpot in: ");
    Serial.print(pulseWidthTrimpot);
    Serial.print("us ");

    int throttleInputDegrees = map(pulseWidthThrottle, 1000, 2000, 0, 180);
    Serial.print("Throttle in: ");
    Serial.print(throttleInputDegrees);

    int throttleOutputDegrees = throttleInputDegrees * pidOutput / 255;

    Serial.print(" Throttle out: ");
    Serial.println(throttleOutputDegrees);
    throttleOutputServo.write(throttleOutputDegrees);
}
