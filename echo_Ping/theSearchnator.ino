// import the library
#include <Servo.h>
#include <NewPing.h>

// create an instance of the servo library
Servo myServo;
const int piezo = A0;
const int greenLed = 4;
const int redLed = 5;

const int trigPin = 3;//orange
const int echoPin = 2;//red white - vcc grey -gnd
// Variables for the duration and the distance
long duration;
int distance;

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
