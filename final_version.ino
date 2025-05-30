#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/interrupt.h>

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int BUTTON_PIN = 2;  // PD2 (INT0)
const int DISTANCE_THRESHOLD = 30;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define SERVO_MIN 150
#define SERVO_MAX 600

float duration_us, distance_cm;
bool isOpen = false;
unsigned long danceStart = 0;
volatile bool isDancing = false;
volatile bool buttonPressed = false;
volatile unsigned long lastButtonTime = 0;

// Filtering variables
#define FILTER_SIZE 5
float distanceBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;
float filteredDistance = 0;

void setup() {
  Serial.begin(9600);

  // Configurare pini pentru ultrasonic sensor
  DDRB |= (1 << PB1);     // TRIG_PIN (pin 9) ca output
  DDRB &= ~(1 << PB2);    // ECHO_PIN (pin 10) ca input
  
  // Configurare pin pentru buton
  DDRD &= ~(1 << PD2);    // BUTTON_PIN (pin 2) ca input
  PORTD |= (1 << PD2);    // Activează pull-up intern
  
  // Configurare interrupt extern INT0 (pin 2)
  EICRA |= (1 << ISC01);  // INT0 pe falling edge
  EICRA &= ~(1 << ISC00); // Clear ISC00 pentru falling edge
  EIMSK |= (1 << INT0);   // Activează INT0
  
  sei();  // Activează întreruperi globale

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  close();
  isOpen = false;
  
  // Initialize distance buffer
  for (int i = 0; i < FILTER_SIZE; i++) {
    distanceBuffer[i] = DISTANCE_THRESHOLD + 10; // Initialize with safe distance
  }
}

// Interrupt Service Routine pentru INT0 (pin 2)
ISR(INT0_vect) {
  unsigned long currentTime = millis();
  
  // Debouncing - ignoră apăsările în următoarele 200ms
  if (currentTime - lastButtonTime > 200) {
    if (!isDancing) {
      buttonPressed = true;
    }
    lastButtonTime = currentTime;
  }
}

void loop() {
  checkDistance();
  handleButtonPress();
  
  if (isDancing) {
    performDance();
  }
  
  delay(100);
}

void handleButtonPress() {
  if (buttonPressed) {
    buttonPressed = false;
    Serial.println("Button pressed! Starting dance!");
    isDancing = true;
    danceStart = millis();
  }
}

void checkDistance() {
  // Trigger pulse
  PORTB |= (1 << PB1);    // Set TRIG_PIN HIGH
  delayMicroseconds(10);
  PORTB &= ~(1 << PB1);   // Set TRIG_PIN LOW

  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;
  
  // Filter out invalid readings (too close, too far, or timeout)
  if (distance_cm < 2 || distance_cm > 400 || duration_us == 0) {
    // Keep previous filtered value if reading is invalid
    distance_cm = filteredDistance;
  } else {
    // Add to moving average filter
    distanceBuffer[bufferIndex] = distance_cm;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    
    if (bufferIndex == 0) {
      bufferFilled = true;
    }
    
    // Calculate moving average
    float sum = 0;
    int count = bufferFilled ? FILTER_SIZE : bufferIndex;
    
    for (int i = 0; i < count; i++) {
      sum += distanceBuffer[i];
    }
    
    filteredDistance = sum / count;
  }

  Serial.print("Raw: ");
  Serial.print(distance_cm);
  Serial.print(" cm, Filtered: ");
  Serial.println(filteredDistance);

  if (!isDancing) {
    if (filteredDistance > 0 && filteredDistance < DISTANCE_THRESHOLD && !isOpen) {
      open();
      isOpen = true;
    }
    else if (filteredDistance >= DISTANCE_THRESHOLD && isOpen) {
      close();
      isOpen = false;
    }
  }
}

void performDance() {
  if (millis() - danceStart >= 10000) {
    Serial.println("Dance finished.");
    isDancing = false;
    return;
  }

  int randPos1 = random(SERVO_MIN, SERVO_MAX);
  pwm.setPWM(0, 0, randPos1);
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
    delay(20);
  }
}