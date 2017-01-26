#include <NewPing.h>

#define TRIGGER_PIN_HEIGHT  3
#define ECHO_PIN_HEIGHT     2
#define MAX_DISTANCE 400

NewPing sonarHeight(TRIGGER_PIN_HEIGHT, ECHO_PIN_HEIGHT, MAX_DISTANCE);
