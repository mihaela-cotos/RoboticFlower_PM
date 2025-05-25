#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int BUTTON_PIN = 12;  // pinul butonului
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

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // folosește rezistor intern

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
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;

  Serial.print("Distance: ");
  Serial.println(distance_cm);

  if (!isDancing) {
    if (distance_cm > 0 && distance_cm < DISTANCE_THRESHOLD && !isOpen) {
      open();
      isOpen = true;
    }
    else if (distance_cm >= DISTANCE_THRESHOLD && isOpen) {
      close();
      isOpen = false;
    }
  }
}

void checkButton() {
  if (digitalRead(BUTTON_PIN) == LOW && !isDancing) {
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

  // Alege aleator o poziție pentru servouri
  int randPos1 = random(SERVO_MIN, SERVO_MAX);
  // int randPos2 = random(SERVO_MIN, SERVO_MAX);
  pwm.setPWM(0, 0, randPos1);
  pwm.setPWM(1, 0, random(SERVO_MIN, SERVO_MAX));
  pwm.setPWM(2, 0, random(SERVO_MIN, SERVO_MAX));
  pwm.setPWM(3, 0, random(SERVO_MIN, SERVO_MAX));

  delay(200);  // scurtă pauză între pașii de dans
}

void open() {
  Serial.println("Opening petals...");

  for (int pos = SERVO_MIN; pos < SERVO_MAX; pos += 5) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    pwm.setPWM(4, 0, pos);
    pwm.setPWM(5, 0, pos);
    pwm.setPWM(6, 0, pos);
    pwm.setPWM(7, 0, pos);
    pwm.setPWM(8, 0, pos);
    pwm.setPWM(9, 0, pos);
    pwm.setPWM(10, 0, pos);
    pwm.setPWM(11, 0, pos);
    pwm.setPWM(12, 0, pos);
    pwm.setPWM(13, 0, pos);
    pwm.setPWM(14, 0, pos);
    pwm.setPWM(15, 0, pos);
    delay(20);
  }
  // delay(5000);
}

void close() {
  Serial.println("Closing petals...");
  for (int pos = SERVO_MAX; pos > SERVO_MIN; pos -= 9) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    pwm.setPWM(4, 0, pos);
    pwm.setPWM(5, 0, pos);
    pwm.setPWM(6, 0, pos);
    pwm.setPWM(7, 0, pos);
    pwm.setPWM(8, 0, pos);
    pwm.setPWM(9, 0, pos);
    pwm.setPWM(10, 0, pos);
    pwm.setPWM(11, 0, pos);
    pwm.setPWM(12, 0, pos);
    pwm.setPWM(13, 0, pos);
    pwm.setPWM(14, 0, pos);
    pwm.setPWM(15, 0, pos);
    delay(50);
  }
}

// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// #define TRIG_PIN 9
// #define ECHO_PIN 10
// #define DISTANCE_THRESHOLD 20  // centimeters

// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// // Servo min and max values for your servos
// #define SERVO_MIN 150
// #define SERVO_MAX 600

// void setup() {
//   Serial.begin(9600);
//   pwm.begin();
//   pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz

//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
// }

// long readDistanceCM() {
//   digitalWrite(TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   long duration = pulseIn(ECHO_PIN, HIGH);
//   long distance = duration * 0.034 / 2;
//   return distance;
// }

// void moveAllServos(int pulse) {
//   for (uint8_t i = 0; i < 16; i++) {
//     pwm.setPWM(i, 0, pulse);
//   }
// }

// void loop() {
//   long distance = readDistanceCM();
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");

//   if (distance < DISTANCE_THRESHOLD) {
//     moveAllServos(SERVO_MAX);  // Move to one position
//     delay(1000);
//     moveAllServos(SERVO_MIN);  // Move to another
//     delay(1000);
//   }

//   delay(200);
// }