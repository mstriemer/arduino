#include <Servo.h>

// Define pins, setup servo limits
#define BUTTON_PIN 2
#define LED_PIN 13
#define SERVO_PIN 9
#define SERVO_MIN 600
#define SERVO_MAX 2400

// Initialize variables
Servo servo;
int prevVal = LOW;
int currVal;
int on = false;
int servoPos = 0;
int servoDir = 1;

void setup() {
  // Setup serial communication for debugging
  Serial.begin(9600);

  // Setup the in/out pins
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Ensure the LED is off
  digitalWrite(LED_PIN, LOW);

  // Setup the servo
  servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  servo.write(servoPos);
}

void loop() {
  // Get the current state of the button
  currVal = digitalRead(BUTTON_PIN);

  // Change states if the button is pressed and the button
  // was not pressed the last time through the loop. This
  // will cause the servo to turn on or off everytime you
  // press the button instead of having to hold the button
  // down to run the servo.
  if (currVal == HIGH && currVal != prevVal) {
    // Set on to false if it is true, or true if it is false
    on = !on;

    // Turn the LED on or off based on our state.
    // This is exploiting the fact that true is equal to 1
    // and HIGH is equal to 1. Likewise false=0 and LOW=0.
    digitalWrite(LED_PIN, on);
  }

  // If we're in the on state, run the servo
  if (on)
    sweepServo();

  // Store the current value of the button so that we can
  // track when it changes.
  prevVal = currVal;

  // Wait for a bit to slow down the servo
  delay(10);
}

void sweepServo() {
  // Move the servo in the current direction
  servoPos += servoDir;

  // If the servo is set to something less than 1 or greater
  // than or equal to 180, change the servo direction.
  if (!(servoPos >= 1 && servoPos < 180))
    servoDir = -servoDir;

  // Move the servo to the position we just calculated
  servo.write(servoPos);
}
