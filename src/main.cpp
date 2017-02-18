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
double throttleP = 4, throttleI = 0.2, throttleD = 1;
PID throttlePID(&pidInput, &pidOutput, &pidSetPoint, throttleP, throttleI, throttleD, DIRECT);

volatile unsigned long timer1DesiredValue;
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
        timer1ChangeMicros = TCNT1; // collecting the timer value here can provide useful debug data
        TIMSK1 &= ~(1 << OCIE1A); // disable the compare interrupt
    } else {
        throttlePulseLastChangeMicros = micros();
        digitalWrite(THROTTLE_PIN_OUT, HIGH);
        TCNT1 = 0; // reset the timer count before interrupt
        OCR1A = timer1DesiredValue;
        timer2StartMicros = micros();
        TIFR1 |= (1 << OCF1A); // clear the timer interrupt flag
        TIMSK1 |= (1 << OCIE1A); // enable the compare interrupt
    }
}

ISR(TIMER1_COMPA_vect) {
    // turn off the throttle PWM sooner than that provided by the RC receiver based on the distance of the sonar 
    digitalWrite(THROTTLE_PIN_OUT, LOW);
    //    timer1ChangeMicros = micros() - timer2StartMicros;
    TIMSK1 &= ~(1 << OCIE1A); // disable the compare interrupt
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
    TIMSK1 &= ~(1 << OCIE1A); // disable the compare match on timer2

    // set timer1 mode
    TCCR1A &= ~(1 << WGM10);
    TCCR1A &= ~(1 << WGM11);
    TCCR1B &= ~(1 << WGM12); // normal mode
    TCCR1B &= ~(1 << WGM13);


    TCCR1B &= ~(1 << CS12); // set the timer2 pre scaler
    TCCR1B &= ~(1 << CS11); // set the timer2 pre scaler
    TCCR1B |= (1 << CS10); // set the timer2 pre scaler
    // when timer1 is directly connected to CLK the high portion of the PPM input lasts between 16000 - 32000 timer counts depending on the throttle position.

    timer1DesiredValue = 25000; // setting an arbitary value before any sonar data is available
    sei();
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(SWITCH_PIN, INPUT);
    pinMode(TRIMPOT_PIN, INPUT);
    pinMode(THROTTLE_PIN_OUT, OUTPUT);
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
    timer1DesiredValue = (map(pulseWidthThrottle, 1000, 2000, 0, 16000) * pidOutput / 255) + 16000;

    if (cyclesSincePrintLine > 10) {
        //Serial.print("forward: ");
        //Serial.print(forwardDistance);

        Serial.print(timer1ChangeMicros);

        Serial.print(" : ");
        Serial.print(timer1DesiredValue);

        Serial.print(" vertical: ");
        Serial.print(verticalDistance);

        //    Serial.print("cm vertical:");
        //    Serial.print("(todo)"); // @ todo
        //    Serial.print("cm in ");
        //    Serial.print(millis());
        //    Serial.print("ms ");

        Serial.print(" P: ");
        Serial.print(throttleP);
        Serial.print(" I: ");
        Serial.print(throttleI);
        Serial.print(" D: ");
        Serial.print(throttleD);
        Serial.print(" PID: ");
        Serial.print(pidOutput);

        Serial.print(" Throttle in: ");
        Serial.print(pulseWidthThrottle);
        Serial.print("us ");

        Serial.print("Switch in: ");
        Serial.print(pulseWidthSwitch);
        Serial.print("us ");

        Serial.print("Trimpot in: ");
        Serial.print(pulseWidthTrimpot);
        Serial.println("us ");

        cyclesSincePrintLine = 0;
    } else {
        cyclesSincePrintLine++;
    }
    if (Serial.available() > 0) {
        int inByte = Serial.read();
        switch (inByte) {
            case 'p':
                throttleP -= 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
                break;
            case 'P':
                throttleP += 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
                break;
            case 'i':
                throttleI -= 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
                break;
            case 'I':
                throttleI += 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
                break;
            case 'd':
                throttleD -= 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
                break;
            case 'D':
                throttleD += 0.1;
                throttlePID.SetTunings(throttleP, throttleI, throttleD);
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
        }
    }
}
