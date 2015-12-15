const int numkeys = 8;
int myPins[numkeys];
uint8_t prevState[numkeys];
uint8_t curState[numkeys];
uint8_t light_state;

#include "pitches.h"
int freqs[numkeys] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};

const int tonePin = 11;
int currentlyPlayingPin;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numkeys; ++i) {
    myPins[i] = i + 2;
    pinMode(i + 2, INPUT);
    prevState[i] = LOW;
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, light_state = HIGH);
  Serial.println("Hello");
}

String print_state(uint8_t state) {
  if (state == HIGH) {
    return "high";
  }
  return "low";
}

// LOW => note is being played
void loop() {
  for (int i = 0; i < numkeys; ++i) {
    curState[i] = digitalRead(myPins[i]);
    String res = print_state(curState[i]);
    if (curState[i] != prevState[i]) {
      digitalWrite(13, light_state = !light_state);
      Serial.print("Key ");
      Serial.print(i);
      Serial.print(" has gone to ");
      Serial.print(res);
      Serial.println();
      // if note now being played
      if (curState[i] == LOW) {
        currentlyPlayingPin = i;
        tone(tonePin, freqs[i]);
      } else {
        if (i == currentlyPlayingPin) {
          noTone(tonePin);
        }
      }
    }
    prevState[i] = curState[i];
  }
}
