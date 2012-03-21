#include <Servo.h>

#define BUTTON_PIN 2
#define LED_PIN 13
#define SERVO_PIN 9
#define SERVO_MIN 600
#define SERVO_MAX 2400

Servo servo;
int prevVal = LOW;
int currVal;
int on = false;
int servoPos = 0;
int servoDir = 1;

void setup() {
//  servo.attach(9, 600, 2400); // Run the servo on pin 9
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  servo.write(servoPos);
}

void loop() {
  currVal = digitalRead(BUTTON_PIN);
  if (currVal == HIGH && currVal != prevVal) {
    on = !on;
    Serial.println(currVal);
    digitalWrite(LED_PIN, on);
  }
  if (on)
    sweepServo();
  prevVal = currVal;
  delay(10);
}

void sweepServo() {
  servoPos += servoDir;
  if (!(servoPos >= 1 && servoPos < 180))
    servoDir = -servoDir;
  servo.write(servoPos);
}
