#include <Arduino.h>
#define PRESCALER 8

const float ticks = uint16_t(INTERVAL / (PRESCALER / double(CPU_SPEED)) + .5);

volatile uint8_t head;
volatile bool busy;
volatile uint16_t data[256];

uint8_t lastR, lastG, lastB, lastBusy;
bool modified;
uint8_t lastHead;
String command;

void IRData();

void setup() {
  // Debug output
  Serial.begin(112500);

  // Trim resistors
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT); 
 
  // IR Signal
  pinMode(IR, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), IRData, CHANGE);
  
  // LED Output (Sink)
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // Configure timer 1 registers
  TCCR1A = B00000000; // Normal pin operation, no PWM
  TCCR1B = B00000000; // Stop the timer
  TIMSK1 = B00000011; // Enable counter1 and overflow interrupts
  TCNT1  = 0;

  // Set the Compare register to 10 Cycles.  Max length is only 7, so we must be done.
  OCR1A = uint16_t(ticks * MAX_SEQUENCE);
  
  // Enable interupts globally
  interrupts();
}

void loop() {
  uint8_t r = uint8_t(map(analogRead(A2), 0, 1023, 255, 0));
  uint8_t g = uint8_t(map(analogRead(A1), 0, 1023, 255, 0));
  uint8_t b = uint8_t(map(analogRead(A0), 0, 1023, 255, 0));

  if (r != lastR) { lastR = r; modified = true;}
  if (g != lastG) { lastG = g; modified = true; }
  if (b != lastB) { lastB = b; modified = true; }

  if (modified) {
    Serial.print("r=");
    Serial.print(lastR);
    Serial.print(", g=");
    Serial.print(lastG);
    Serial.print(", b=");
    Serial.println(lastB);

    analogWrite(RED, lastR);
    analogWrite(GREEN, lastG);
    analogWrite(BLUE, lastB);

    modified = false;
  }

  if (lastBusy != TCCR1B) {
    Serial.println(lastBusy ? "Done" : "Busy");
    if (lastBusy) {
      if (command.length() > 0) {
        Serial.print("Command: ");
        Serial.println(command);
        if (command=="010101010101010101111111010111111101110111011101110101011101110101010101010111010101010101010111011101110111110") {
          analogWrite(RED, 0);
          analogWrite(GREEN, 255);
          analogWrite(BLUE, 255);          
          modified = true;
        }else if (command=="010101010101010101111111010111111101110111011101110101011101110101010101010111010101010101011101011101110111011") {
          analogWrite(RED, 255);
          analogWrite(GREEN, 0);
          analogWrite(BLUE, 255);
          modified = true;
        }else if (command=="010101010101010101111111010111111101110111011101110101011101110101010101010111110101010101011101011101110111010") {
          analogWrite(RED, 255);
          analogWrite(GREEN, 255);
          analogWrite(BLUE, 0);
          modified = true;
        } else {
          Serial.println(command);
        }
        if (modified) {
          delay(200);
          analogWrite(RED, 255);
          analogWrite(GREEN, 255);
          analogWrite(BLUE, 255);
        }
        command = "";
      }
    } else {
    }
    lastBusy = TCCR1B;
  }

  while (lastHead != head) {
    command += String(uint8_t(data[lastHead++]/ticks+.5));
  }
}

void IRData() {
  if (TCCR1B) {
    data[head++] = TCNT1;
    TCNT1  = 0;
  } else {
    TCCR1B = B00000010;
  }
}

ISR(TIMER1_COMPA_vect) {
  TCCR1B = B00000000; // Prescaler to 0, stopping the timer
  TCNT1  = 0;
}