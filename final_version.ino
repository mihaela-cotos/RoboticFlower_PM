#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/interrupt.h>

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

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
  // Inițializare UART
  UART_Init(MYUBRR);

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
    UART_Print("Button pressed! Starting dance!\r\n");
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

  UART_Print("Raw: ");
  UART_PrintFloat(distance_cm, 1);
  UART_Print(" cm, Filtered: ");
  UART_PrintFloat(filteredDistance, 1);
  UART_Print("\r\n");

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
    UART_Print("Dance finished.\r\n");
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
  UART_Print("Opening petals...\r\n");

  for (int pos = SERVO_MIN; pos < SERVO_MAX; pos += 9) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    delay(20);
  }
}

void close() {
  UART_Print("Closing petals...\r\n");
  for (int pos = SERVO_MAX; pos > SERVO_MIN; pos -= 9) {
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    delay(20);
  }
}

// Funcții UART folosind registrii AVR
void UART_Init(unsigned int ubrr) {
  // Set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  
  // Set frame format: 8data, 2stop bit
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void UART_Transmit(unsigned char data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  
  // Put data into buffer, sends the data
  UDR0 = data;
}

void UART_Print(const char* str) {
  while (*str) {
    UART_Transmit(*str++);
  }
}

void UART_PrintFloat(float value, int decimals) {
  // Handle negative numbers
  if (value < 0) {
    UART_Transmit('-');
    value = -value;
  }
  
  // Print integer part
  int intPart = (int)value;
  UART_PrintInt(intPart);
  
  if (decimals > 0) {
    UART_Transmit('.');
    
    // Print decimal part
    float decPart = value - intPart;
    for (int i = 0; i < decimals; i++) {
      decPart *= 10;
      int digit = (int)decPart;
      UART_Transmit('0' + digit);
      decPart -= digit;
    }
  }
}

void UART_PrintInt(int value) {
  if (value == 0) {
    UART_Transmit('0');
    return;
  }
  
  char buffer[10];
  int i = 0;
  
  // Handle negative numbers
  bool negative = false;
  if (value < 0) {
    negative = true;
    value = -value;
  }
  
  // Convert to string (reverse order)
  while (value > 0) {
    buffer[i++] = '0' + (value % 10);
    value /= 10;
  }
  
  // Add negative sign if needed
  if (negative) {
    UART_Transmit('-');
  }
  
  // Print digits in correct order
  for (int j = i - 1; j >= 0; j--) {
    UART_Transmit(buffer[j]);
  }
}
