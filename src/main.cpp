/**
 * @created: 10/12/2016 15:42
 * @author : Peter Withers <peter@gthb-bambooradical.com>
 * @author : Harmjan Lever <harmjanfl@gmail.com>
 */

#include <NewPing.h>

#define TRIGGER_PIN_FORWARD  3
#define ECHO_PIN_FORWARD     2
// @todo: add defines for TRIGGER_PIN_RIGHT and ECHO_PIN_RIGHT once the desired pins have been assigned
// @todo: add left and back sonar definitions
#define MAX_DISTANCE 400

NewPing sonarForward(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD, MAX_DISTANCE);
//NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

volatile unsigned long throttlePulseLastChangeMs = 0;
volatile int pulseWidthThrottle = 0;
#define THROTTLE_PIN A0

void throttleSignalInterrupt() {
    if ((digitalRead(THROTTLE_PIN) == 0)) {
        pulseWidthThrottle = millis() - throttlePulseLastChangeMs;
    } else {
        throttlePulseLastChangeMs = millis();
    }
}

void setup() {
    Serial.begin(57600);
    delay(10);
    digitalWrite(13, 1);
    pinMode(THROTTLE_PIN, INPUT);
    //    digitalWrite(THROTTLE_PIN, HIGH); // the receiver should not need the pull up enabled
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleSignalInterrupt, CHANGE);
}

void loop() {
    delay(1000);
    int forwardDistance = sonarForward.ping_cm();
    //    int rightDistance = sonarRight.ping_cm(); // @todo
    Serial.print("forward: ");
    Serial.print(forwardDistance);
    Serial.print("cm right:");
    Serial.print("(todo)"); // @ todo
    Serial.print("cm in ");
    Serial.print(millis());
    Serial.println("ms");
    if (forwardDistance > 100) {
        digitalWrite(13, 1);
    } else {
        digitalWrite(13, 0);
    }
    Serial.print("Throttle: ");
    Serial.print(pulseWidthThrottle);
    Serial.println("ms");
}