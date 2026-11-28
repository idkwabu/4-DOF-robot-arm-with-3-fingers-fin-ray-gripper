/*
 * irl_serial - six-servo controller with serial position feedback.
 *
 * Same wiring and command protocol as ../irl.ino, but each accepted pose is
 * echoed back over serial as:  OK,<p0>,<p1>,<p2>,<p3>,<p4>,<p5>
 * where the values are the 0..180 angles actually written to the servos.
 *
 * Pins: 3, 5, 6, 9, 10, 11 in the order
 *       base, joint1, joint2, joint3, gripper_roll, gripper.
 * Serial: 115200 baud, one newline-terminated CSV command per pose.
 *
 * servo_controller.py accepts both the legacy "OK" reply and this
 * "OK,<csv>" echo, so the host bridge works with either sketch.
 */

#include <Servo.h>
#include <stdlib.h>
#include <string.h>

const uint8_t SERVO_PINS[6] = {3, 5, 6, 9, 10, 11};
const uint8_t SERVO_COUNT = 6;

Servo servos[SERVO_COUNT];
char inputLine[40];
uint8_t inputLength = 0;
bool lineTooLong = false;
bool servosAttached = false;

// Parse exactly six comma-separated integers in 0..180 into positions[].
bool parsePositions(char *line, int positions[SERVO_COUNT]) {
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
  return true;
}

void attachServos() {
  if (!servosAttached) {
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      servos[i].attach(SERVO_PINS[i]);
    }
    servosAttached = true;
  }
}

void echoPositions(const int positions[SERVO_COUNT]) {
  Serial.print("OK");
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(',');
    Serial.print(positions[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    char character = (char)Serial.read();

    if (character == '\n') {
      if (lineTooLong) {
        Serial.println("ERR");
      } else {
        inputLine[inputLength] = '\0';
        int positions[SERVO_COUNT];

        if (parsePositions(inputLine, positions)) {
          attachServos();
          for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
            servos[i].write(positions[i]);
          }
          echoPositions(positions);
        } else {
          Serial.println("ERR");
        }
      }
      inputLength = 0;
      lineTooLong = false;
    } else if (character != '\r') {
      if (inputLength < sizeof(inputLine) - 1) {
        inputLine[inputLength++] = character;
      } else {
        // Discard the whole overlong line instead of accepting a fragment.
        lineTooLong = true;
      }
    }
  }
}
