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

volatile unsigned long throttlePulseLastChangeMicros = 0;
volatile unsigned long trimpotPulseLastChangeMicros = 0;
volatile unsigned long switchPulseLastChangeMicros = 0;
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
unsigned int timer1DesiredValue;
volatile unsigned long timer1ChangeMicros = 0;
volatile unsigned long timer2StartMicros = 0;

ISR(PCINT0_vect) {
    if (digitalRead(TRIMPOT_PIN) == 0 && trimpotPinLast) {
        pulseWidthTrimpot = micros() - trimpotPulseLastChangeMicros;
        trimpotPinLast = false;
    } else if (digitalRead(TRIMPOT_PIN) == 1 && !trimpotPinLast) {
        trimpotPulseLastChangeMicros = micros();
        trimpotPinLast = true;
    }
    if (digitalRead(SWITCH_PIN) == 0 && switchPinLast) {
        pulseWidthSwitch = micros() - switchPulseLastChangeMicros;
        switchPinLast = false;
    } else if (digitalRead(SWITCH_PIN) == 1 && !switchPinLast) {
        switchPulseLastChangeMicros = micros();
        switchPinLast = true;
    }
}

ISR(PCINT1_vect) {
    if (digitalRead(THROTTLE_PIN) == 0) {
        pulseWidthThrottle = micros() - throttlePulseLastChangeMicros;
        digitalWrite(THROTTLE_PIN_OUT, LOW);
        // TIMSK1 &= ~(1 << TOIE1); // disable the timer overflow interrupt
        // timer1ChangeMicros = 0;
    } else {
        throttlePulseLastChangeMicros = micros();
        digitalWrite(THROTTLE_PIN_OUT, HIGH);
        TCNT1 = timer1DesiredValue; // set the timer count before interrupt
        timer2StartMicros = micros();
        TIMSK1 |= (1 << TOIE1); // enable the timer overflow interrupt
    }
}

ISR(TIMER1_OVF_vect) {
    // turn off the throttle PWM sooner than that provided by the RC receiver based on the distance of the sonar 
    digitalWrite(THROTTLE_PIN_OUT, LOW);
    TIMSK1 &= ~(1 << TOIE1); // disable the timer overflow interrupt
    timer1ChangeMicros = micros() - timer2StartMicros;
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

    TIMSK1 &= ~(1 << TOIE1); // disable the timer overflow interrupt

    // set timer1 to counting only
    TCCR1A &= ~(1 << WGM10);
    TCCR1A &= ~(1 << WGM11);
    TCCR1B &= ~(1 << WGM12);
    TCCR1B &= ~(1 << WGM13);

    TIMSK1 &= ~(1 << OCIE1A); // disable the compare match on timer2

    TCCR1B |= (1 << CS12); // set the timer2 pre scaler to 1024
    TCCR1B |= (1 << CS11); // set the timer2 pre scaler to 1024
    TCCR1B |= (1 << CS10); // set the timer2 pre scaler to 1024

    timer1DesiredValue = 0; // setting an arbitary value before any sonar data is avaiable

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

    //throttleOutputServo.write(throttleOutputDegrees);
    if (cyclesSincePrintLine > 10) {
        //Serial.print("forward: ");
        //Serial.print(forwardDistance);

        Serial.print(timer1ChangeMicros);

        Serial.print(" : ");
        Serial.print(timer1DesiredValue);

        Serial.print("vertical: ");
        Serial.print(verticalDistance);

        //    Serial.print("cm vertical:");
        //    Serial.print("(todo)"); // @ todo
        //    Serial.print("cm in ");
        //    Serial.print(millis());
        //    Serial.print("ms ");

        Serial.print(" PID out: ");
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
    if (Serial.available() > 0) {
        int inByte = Serial.read();
        switch (inByte) {
            case 'p':

                break;
            case 'P':

                break;
            case 'i':

                break;
            case 'I':

                break;
            case 'd':

                break;
            case 'D':

                break;
            case 'Q':
                TCCR1B |= (1 << CS12); // set the timer2 pre scaler to 1024
                break;
            case 'W':
                TCCR1B |= (1 << CS11); // set the timer2 pre scaler to 1024
                break;
            case 'E':
                TCCR1B |= (1 << CS10); // set the timer2 pre scaler to 1024
                break;
            case 'q':
                TCCR1B &= ~(1 << CS12); // set the timer2 pre scaler to 1024
                break;
            case 'w':
                TCCR1B &= ~(1 << CS11); // set the timer2 pre scaler to 1024
                break;
            case 'e':
                TCCR1B &= ~(1 << CS10); // set the timer2 pre scaler to 1024
                break;
            case 'a':
                timer1DesiredValue = 2000;
                break;
            case 's':
                timer1DesiredValue = 100;
                break;
        }
    }
}
