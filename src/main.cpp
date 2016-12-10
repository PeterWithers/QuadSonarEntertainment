/**
 * @created: 10/12/2016 15:42
 * @author : Peter Withers <peter@gthb-bambooradical.com>
 * @author : Harmjan Lever <harmjanfl@gmail.com>
 */

#include <NewPing.h>

#define TRIGGER_PIN  3
#define ECHO_PIN     2
#define MAX_DISTANCE 400

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
    Serial.begin(57600);
    delay(10);
    digitalWrite(13, 1);
}

void loop() {
    delay(1000);
    Serial.print("Ping: ");
    int distanceValue = sonar.ping_cm();
    Serial.print(distanceValue);
    Serial.print("cm: ");
    Serial.println(millis());
    if (distanceValue > 100)
        digitalWrite(13, 1);
    else
        digitalWrite(13, 0);
}