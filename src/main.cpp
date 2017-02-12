/**
 * @created: 10/12/2016 15:42
 * @author : Peter Withers <peter@gthb-bambooradical.com>
 * @author : Harmjan Lever <harmjanfl@gmail.com>
 */

#include <NewPing.h>
//#include <Servo.h>
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
int cyclesSincePrintLine = 0;

#define THROTTLE_PIN A0
#define THROTTLE_PIN_OUT 13
#define TRIMPOT_PIN 9
#define SWITCH_PIN 8

//#define YAW_PIN A1

//PID variables
double pidSetPoint, pidInput, pidOutput;
PID throttlePID(&pidInput, &pidOutput, &pidSetPoint, 2, 5, 1, DIRECT);

//Servo throttleOutputServo;
unsigned int timer2DesiredValue;
volatile int timerTriggerCount;

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
        digitalWrite(THROTTLE_PIN_OUT, LOW);
        TIMSK2 &= ~(1 << TOIE2); // disable the timer overflow interrupt
    } else {
        throttlePulseLastChangeMs = micros();
        digitalWrite(THROTTLE_PIN_OUT, HIGH);
        TCNT2 = timer2DesiredValue; // set the timer count before interrupt
        timerTriggerCount = 0;
        TIMSK2 |= (1 << TOIE2); // enable the timer overflow interrupt
    }
}

ISR(TIMER2_OVF_vect) {
    if (timerTriggerCount > 0) {
        // turn off the throttle PWM sooner that that provided by the RC receiver based on the distance of the sonar 
        digitalWrite(THROTTLE_PIN_OUT, LOW);
        TIMSK2 &= ~(1 << TOIE2); // disable the timer overflow interrupt
    } else {
        timerTriggerCount++;
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

    TIMSK2 &= ~(1 << TOIE2); // disable the timer overflow interrupt

    TCCR2A &= ~((1 << WGM21) | (1 << WGM20)); // set timer2 to counting only
    TCCR2B &= ~(1 << WGM22);
    ASSR &= ~(1 << AS2); // set the timer2 source to the CPU clock
    TIMSK2 &= ~(1 << OCIE2A); // disable the compare match on timer2

    TCCR2B |= (1 << CS22); // set the timer2 pre scaler to 1024
    TCCR2B |= (1 << CS21); // set the timer2 pre scaler to 1024
    TCCR2B |= (1 << CS20); // set the timer2 pre scaler to 1024

    timer2DesiredValue = 0; // setting an arbitary value before any sonar data is avaiable

    sei();
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(SWITCH_PIN, INPUT);
    pinMode(TRIMPOT_PIN, INPUT);
    pinMode(THROTTLE_PIN_OUT, OUTPUT);
    //throttleOutputServo.attach(THROTTLE_PIN_OUT);
    pidInput = 60; //arbitrary initial value
    pidSetPoint = 60;
    throttlePID.SetMode(AUTOMATIC);
}

void loop() {
    delay(100);
    //int forwardDistance = sonarForward.ping_cm();
    int verticalDistance = sonarHeight.ping_cm();
    //    int rightDistance = sonarRight.ping_cm(); // @todo

    pidInput = verticalDistance;
    throttlePID.Compute();
    int throttleInputDegrees = map(pulseWidthThrottle, 1000, 2000, 0, 180);
    int throttleOutputDegrees = throttleInputDegrees * pidOutput / 255;
    
    // todo: the timer2DesiredValue value needs to be set according the the sonar input
    timer2DesiredValue = 200;
    //throttleOutputServo.write(throttleOutputDegrees);
    if (cyclesSincePrintLine > 10) {
        //Serial.print("forward: ");
        //Serial.print(forwardDistance);
        Serial.print("vertical: ");
        Serial.print(verticalDistance);

        //    Serial.print("cm vertical:");
        //    Serial.print("(todo)"); // @ todo
        //    Serial.print("cm in ");
        //    Serial.print(millis());
        //    Serial.print("ms ");

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

        Serial.print("Throttle in: ");
        Serial.print(throttleInputDegrees);

        Serial.print(" Throttle out: ");
        Serial.println(throttleOutputDegrees);
        cyclesSincePrintLine = 0;
    } else {
        cyclesSincePrintLine++;
    }
}
