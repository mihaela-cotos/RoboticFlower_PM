#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Pini hardware
const int TRIG_PIN = 9;     // PB1
const int ECHO_PIN = 10;    // PB2
const int BUTTON_PIN = 12;  // PB4

const int DISTANCE_THRESHOLD = 30;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define SERVO_MIN 150
#define SERVO_MAX 600

float duration_us, distance_cm;
bool isOpen = false;
unsigned long danceStart = 0;
bool isDancing = false;

void setup() {
  Serial.begin(9600);

  // TRIG - OUTPUT (PB1)
  DDRB |= (1 << PB1);

  // ECHO - INPUT (PB2)
  DDRB &= ~(1 << PB2);

  // BUTTON - INPUT_PULLUP (PB4)
  DDRB &= ~(1 << PB4);     // input
  PORTB |= (1 << PB4);     // pull-up activ

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  close();
  isOpen = false;
}

void loop() {
  checkDistance();
  checkButton();
  if (isDancing) {
    performDance();
  }
  delay(100);
}

void checkDistance() {
  // Trigger - pulse 10us HIGH
  PORTB |= (1 << PB1);
  delayMicroseconds(10);
  PORTB &= ~(1 << PB1);

  // Echo - măsurare HIGH
  unsigned long timeout = micros() + 30000; // max 30 ms

  // Așteaptă HIGH
  while (!(PINB & (1 << PB2))) {
    if (micros() > timeout) return;
  }
  unsigned long start = micros();

  // Așteaptă să revină LOW
  while (PINB & (1 << PB2)) {
    if (micros() > timeout) return;
  }
  unsigned long end = micros();

  duration_us = end - start;
  distance_cm = 0.017 * duration_us;

  Serial.print("Distance: ");
  Serial.println(distance_cm);

  if (!isDancing) {
    if (distance_cm > 0 && distance_cm < DISTANCE_THRESHOLD && !isOpen) {
      open();
      isOpen = true;
    } else if (distance_cm >= DISTANCE_THRESHOLD && isOpen) {
      close();
      isOpen = false;
    }
  }
}

void checkButton() {
  if (!(PINB & (1 << PB4)) && !isDancing) { // buton apăsat = LOW
    Serial.println("Button pressed! Starting dance!");
    isDancing = true;
    danceStart = millis();
  }
}

void performDance() {
  if (millis() - danceStart >= 10000) {  // 10 secunde
    Serial.println("Dance finished.");
    isDancing = false;
    return;
  }

  // Poziții aleatoare pentru servouri
  pwm.setPWM(0, 0, random(SERVO_MIN, SERVO_MAX));
  pwm.setPWM(1, 0, random(SERVO_MIN, SERVO_MAX));
  pwm.setPWM(2, 0, random(SERVO_MIN, SERVO_MAX));
  pwm.setPWM(3, 0, random(SERVO_MIN, SERVO_MAX));

  delay(200);
}

void open() {
  Serial.println("Opening petals...");
  for (int pos = SERVO_MIN; pos < SERVO_MAX; pos += 9) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    delay(20);
  }
}

void close() {
  Serial.println("Closing petals...");
  for (int pos = SERVO_MAX; pos > SERVO_MIN; pos -= 9) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
  }
}
