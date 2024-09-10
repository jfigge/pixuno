#include <Arduino.h>

uint8_t lastR, lastG, lastB, lastBusy;
bool modified;

volatile uint8_t head;
volatile bool busy;
volatile uint16_t data[64];
uint8_t lastHead;
bool irState = false;
const float ticks = uint16_t(INTERVAL / (PRESCALER / double(CPU_SPEED)) + .5);
String command;

void IRData();

void setup() {
  #ifdef DEBUG
  Serial.begin(112500);
  #endif

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
  if (lastBusy != TCCR1B) {
    #ifdef DEBUG
    if (lastBusy) {
      Serial.println("Command: " + command);
      command = "";
    }
    #endif
    lastBusy = TCCR1B;
  }


  while (lastHead != head) {
    command += char('0'+data[lastHead++]);
  }
}

// Stop the timer as we've exceeded the max character sequence
ISR(TIMER1_COMPA_vect) {
  TCCR1B = B00000000; // Prescaler to 0, stopping the timer
  TCNT1  = 0;
}

// IR character input
void IRData() {
  if (TCCR1B) {
    data[head++] = uint16_t(TCNT1/ticks+.5);
    TCNT1  = 0;
  } else {
    TCCR1B = B00000010;
  }
}

// Tx chharacter input
void serialEventRun() {
  while (Serial.available()) {
    command += (char)Serial.read();
    TCNT1  = 0;
    TCCR1B = B00000010;
  }
}