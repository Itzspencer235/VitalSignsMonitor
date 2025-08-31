#include <Wire.h>
#include <Keypad.h>
#include <MAX30100_PulseOximeter.h>

#define REPORTING_PERIOD_MS 1000

PulseOximeter pox;
uint32_t tsLastReport = 0;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {2, 3, 4, 5}; // Connect to Arduino pins 2, 3, 4, 5
byte colPins[COLS] = {6, 7, 8, 9}; // Connect to Arduino pins 6, 7, 8, 9
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String phoneNumber = "";
const int MAX_PHONE_LENGTH = 13; // Allows up to 13 chars (e.g., +1234567890)
const int MIN_PHONE_LENGTH = 9;  // Minimum 9 digits

void onBeatDetected() {
  Serial.println("Beat!");
}

void setup() {
  Serial.begin(9600);
  Serial.print("Initializing pulse oximeter..");
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;);
  } else {
    Serial.println("SUCCESS");
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  keypad.setDebounceTime(50);
}

void loop() {
  pox.update();

  // Handle keypad input
  char key = keypad.getKey();
  if (key) {
    if (key == '#') {
      if (phoneNumber.length() >= MIN_PHONE_LENGTH) {
        Serial.print("PHONE:");
        Serial.print(phoneNumber);
        Serial.print("\n");
        phoneNumber = "";
      } else {
        Serial.print("PHONE_ERROR:Invalid\n");
        phoneNumber = "";
      }
    } 
    else if (key == '*') {
      phoneNumber = "";
      Serial.print("PHONE_UPDATE:");
      Serial.print(phoneNumber);
      Serial.print("\n");
    } 
    else if (phoneNumber.length() < MAX_PHONE_LENGTH) {
      if (isDigit(key) || key == '+' || key == 'A') { // Allow digits, +, or A (for +)
        if (key == 'A') {
          phoneNumber += '+';
        } else {
          phoneNumber += key;
        }
        Serial.print("PHONE_UPDATE:");
        Serial.print(phoneNumber);
        Serial.print("\n");
      }
    }
  }

  // Send vital signs periodically
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    float heartRate = pox.getHeartRate();
    float spo2 = pox.getSpO2();
    Serial.print("BPM:");
    Serial.print(heartRate, 1);
    Serial.print(",SpO2:");
    Serial.print(spo2, 1);
    Serial.print("\n");
    tsLastReport = millis();
  }
}
