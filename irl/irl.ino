#include <Servo.h>
#include <stdlib.h>
#include <string.h>

const uint8_t SERVO_PINS[5] = {3, 5, 6, 9, 10};
const uint8_t SERVO_COUNT = 5;

Servo servos[SERVO_COUNT];
char inputLine[40];
uint8_t inputLength = 0;
bool servosAttached = false;

bool applyPositions(char *line) {
  int positions[SERVO_COUNT];
  uint8_t count = 0;
  char *token = strtok(line, ",");

  while (token != NULL && count < SERVO_COUNT) {
    char *end = NULL;
    long value = strtol(token, &end, 10);
    if (end == token || *end != '\0' || value < 0 || value > 180) {
      return false;
    }
    positions[count++] = (int)value;
    token = strtok(NULL, ",");
  }

  if (count != SERVO_COUNT || token != NULL) {
    return false;
  }

  if (!servosAttached) {
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      servos[i].attach(SERVO_PINS[i]);
    }
    servosAttached = true;
  }
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    servos[i].write(positions[i]);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    char character = (char)Serial.read();
    if (character == '\n') {
      inputLine[inputLength] = '\0';
      if (!applyPositions(inputLine)) {
        Serial.println("ERR");
      } else {
        Serial.println("OK");
      }
      inputLength = 0;
    } else if (character != '\r') {
      if (inputLength < sizeof(inputLine) - 1) {
        inputLine[inputLength++] = character;
      } else {
        inputLength = 0;
      }
    }
  }
}
