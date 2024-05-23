// Ali.b
/* 
----------------------------HOW TO USE:------------------------------
#include "buzzer_handler.h"


void setup() {
  buzzer::setup(9);
  Serial.begin(9600);
}

void loop() {
  buzzer::HandleBuzzer();

  if (Serial.available()) { // this represents a triggering event
    int c = Serial.read();
    buzzer::beep(5, 50);
  }
}
---------------------------------------------------------------------
 */

// Global variables for buzzer functionality
byte BUZZER_PIN; // Pin connected to the buzzer
unsigned long previousTimeBuzzer; // Previous time recorded for buzzer operation
unsigned long currentTimeBuzzer; // Current time for buzzer operation
bool BuzzerCalled = false; // Flag indicating whether the buzzer is active
unsigned short int counterBuzzer = 0; // Counter for the number of beeps
unsigned short int beeps; // Total number of beeps to produce
unsigned short int eventInterval; // Time interval between each beep

namespace buzzer {
  /*
   * HandleBuzzer function manages the buzzer operation
  */
  void HandleBuzzer() {
    if (!BuzzerCalled) return; // If buzzer not called, return
    currentTimeBuzzer = millis(); // Get the current time

    // If all beeps are done, turn off the buzzer
    if (counterBuzzer == beeps * 2) {
      BuzzerCalled = false;
      digitalWrite(BUZZER_PIN, LOW);
      return;
    }

    // If not enough time passed, return
    if (currentTimeBuzzer - previousTimeBuzzer < eventInterval) return;
    
    // Toggle buzzer state and update counters
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    counterBuzzer++;
    previousTimeBuzzer = currentTimeBuzzer;
  }

  /*
   * Setup function initializes the buzzer pin
   */
  void setup(int buzzer) {
    BUZZER_PIN = buzzer; // Set buzzer pin
    pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
  }

  /*
   * beep function activates the buzzer with specified parameters
  */
  void beep(short int numberOfBeeps, short int Duration) {
    BuzzerCalled = true; // Set the flag to indicate buzzer is called
    counterBuzzer = 0; // Reset beep counter
    beeps = numberOfBeeps; // Set total number of beeps
    eventInterval = Duration; // Set time interval between beeps
  }
}
