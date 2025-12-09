#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <protothreads.h>
#include "pitches.h"

struct songThread {
  struct pt pt;
  int note;
  unsigned long lastNoteTime;  // Optional: for timing control
};
struct songThread ptSong;

struct lineFollowThread {
  struct pt pt;
  bool left;
  bool center;
  bool right;
  bool done;
};
struct lineFollowThread ptLineFollow;

Adafruit_SSD1306 display(128, 32, &Wire, -1);

const byte BUZZER = 4;

const float WHEEL_DIAMETER = 2.6875; // Inches
const int DEFAULT_SPEED = 100;

const byte HIGHBEAM_LEFT = 47;
const byte HIGHBEAM_RIGHT = 41;

const byte FRONT_BLINKER_LEFT = 49;
const byte FRONT_BLINKER_RIGHT = 43;

const byte BRAKE_LIGHT_LEFT = 27;
const byte BRAKE_LIGHT_RIGHT = 33;

const byte MOTOR_PWM_A = 4;
const byte ENCODER_A_1 = 2;
const byte ENCODER_A_2 = 3;
const byte INA1A = 32;
const byte INA2A = 34;

const byte MOTOR_PWM_B = 5;
const byte ENCODER_B_1 = 18;
const byte ENCODER_B_2 = 19;
const byte INA1B = 30;
const byte INA2B = 36;

const byte LINE_TRACKING_LEFT = 8;
const byte LINE_TRACKING_CENTER = 7;
const byte LINE_TRACKING_RIGHT = 6;

static volatile int16_t INA1A_count = 0;
static volatile int16_t INA1B_count = 0;
static volatile int16_t INA2A_count = 0;
static volatile int16_t INA2B_count = 0;
void ISR_1A() {
  INA1A_count++;
}
void ISR_2A() {
  INA2A_count++;
}
void ISR_1B() {
  INA1B_count++;
}
void ISR_2B() {
  INA2B_count++;
}

void setup() {
  Serial2.begin(38400);
  Serial.begin(9600);

  PT_INIT(&ptSong.pt);
  PT_INIT(&ptLineFollow.pt);

  pinMode(BUZZER, OUTPUT);

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(FRONT_BLINKER_LEFT, OUTPUT);
  pinMode(FRONT_BLINKER_RIGHT, OUTPUT);
  pinMode(BRAKE_LIGHT_LEFT, OUTPUT);
  pinMode(BRAKE_LIGHT_RIGHT, OUTPUT);

  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);

  pinMode(LINE_TRACKING_LEFT, INPUT_PULLUP);
  pinMode(LINE_TRACKING_CENTER, INPUT_PULLUP);
  pinMode(LINE_TRACKING_RIGHT, INPUT_PULLUP);

  analogWrite(MOTOR_PWM_A, 0);
  analogWrite(MOTOR_PWM_B, 0);
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, LOW);
  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), ISR_1A, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), ISR_1B, FALLING);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
}

int playSong(struct songThread* thread) {
  int melody[] = {
    NOTE_AS4, NOTE_AS4, NOTE_AS4,
    NOTE_F5, NOTE_C6,
    NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
    NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
    NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5,
    NOTE_F5, NOTE_C6,
    NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,

    NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
    NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5,
    NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
    NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_C5,
    NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,

    NOTE_C6, NOTE_G5, NOTE_G5, REST, NOTE_C5,
    NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
    NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C6, NOTE_C6,
    NOTE_F6, NOTE_DS6, NOTE_CS6, NOTE_C6, NOTE_AS5, NOTE_GS5, NOTE_G5, NOTE_F5,
    NOTE_C6
  };

  int durations[] = {
    8, 8, 8,
    2, 2,
    8, 8, 8, 2, 4,
    8, 8, 8, 2, 4,
    8, 8, 8, 2, 8, 8, 8,
    2, 2,
    8, 8, 8, 2, 4,

    8, 8, 8, 2, 4,
    8, 8, 8, 2, 8, 16,
    4, 8, 8, 8, 8, 8,
    8, 8, 8, 4, 8, 4, 8, 16,
    4, 8, 8, 8, 8, 8,

    8, 16, 2, 8, 8,
    4, 8, 8, 8, 8, 8,
    8, 8, 8, 4, 8, 4, 8, 16,
    4, 8, 4, 8, 4, 8, 4, 8,
    1
  };
  
  PT_BEGIN(&thread->pt);

  int size = sizeof(durations) / sizeof(int);

  for (thread->note = 0; thread->note < size; thread->note++) {
    Serial.println(thread->note);
    int duration = 1000 / durations[thread->note];
    tone(BUZZER, melody[thread->note], duration);

    int pauseBetweenNotes = duration * 1.30;
    PT_SLEEP(&thread->pt, pauseBetweenNotes);

    noTone(BUZZER);
  }
  
  thread->note = 0;  // Reset for next play

  PT_END(&thread->pt);
}

/// @brief Turns on or off highbeams
///
/// @param state - State to set highbeams to
void toggleHighbeams(bool state) {
  digitalWrite(HIGHBEAM_LEFT, state);
  digitalWrite(HIGHBEAM_RIGHT, state);
}

/// @brief Turns on or off left blinker
///
/// @param state - State to set left blinker to
void toggleLeftBlinker(bool state) {
  Serial.println(state);
  digitalWrite(FRONT_BLINKER_LEFT, state);
}

/// @brief Turns on or off right blinker
///
/// @param state - State to set right blinker to
void toggleRightBlinker(bool state) {
  digitalWrite(FRONT_BLINKER_RIGHT, state);
}

/// @brief Turns on or off brake lights
///
/// @param state - State to set brake lights to
void toggleBrakeLights(bool state) {
  digitalWrite(BRAKE_LIGHT_LEFT, state);
  digitalWrite(BRAKE_LIGHT_RIGHT, state);
}

/// @brief Reads a message sent via serial
///
/// Message will be converted to an all-lowercase string
///
/// @return The message sent via serial
String readMessage() {
  String message = "";

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') break; // End of string

    message += char(tolower(c));
    delay(2); // Small delay to let bytes arrive
  }

  return message;
}

/// @brief Moves the car foward
///
/// Car will move forward until stop function is called
///
/// @param speed - PWM speed value (optional)
void forward(int speed=DEFAULT_SPEED) {
  analogWrite(MOTOR_PWM_A, speed);
  analogWrite(MOTOR_PWM_B, speed);
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

/// @brief Stops the car
///
/// Turns off all motors
void stop() {
  toggleBrakeLights(true); // Brake lights on while stopping

  analogWrite(MOTOR_PWM_A, 0);
  analogWrite(MOTOR_PWM_B, 0);
  digitalWrite(INA1A, LOW);
  digitalWrite(INA1B, LOW);
  digitalWrite(INA2A, LOW);
  digitalWrite(INA2B, LOW);
}

/// @brief Moves the car forward a given distance
///
/// @param distance - Distance to travel (inches)
/// @param speed - PWM speed value (optional)
void travelDistance(float distance, int speed=DEFAULT_SPEED) {
  toggleBrakeLights(false); // Brake lights off while moving

  unsigned long previous_millis = millis();
  float distance_traveled = 0; // Inches
  float rpm = 0;
  INA1B_count = 0;
  do {
    forward(speed);
    // Update RPMs every 50ms
    if (millis() - previous_millis >= 50) {
      rpm = INA1B_count * 6.25;
      distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
      INA1B_count = 0;
      previous_millis = millis();
    }
  } while (distance_traveled < distance);
  stop();
}

/// @brief Turns the car in place a given angle
///
/// @param direction - Direction to turn in (true = right, false = left)
/// @param angle - Angle amount to turn (degrees)
/// @param speed - PWM speed value (optional)
void turn(bool direction, float angle, int speed=60) {
  toggleBrakeLights(false); // Brake lights off while turning

  unsigned long previous_millis = millis();
  float distance_traveled = 0; // Inches
  float rpm = 0;
  INA1B_count = 0;
  INA1A_count = 0;

  // Right
  if (direction) {
    do {
      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, LOW);
      digitalWrite(INA2B, HIGH);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA2A, LOW);

      // Update RPMs every 50ms
      if (millis() - previous_millis >= 50) {
        toggleRightBlinker(!digitalRead(FRONT_BLINKER_RIGHT)); // Flip blinker state
        rpm = INA1B_count * 6.25;
        distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
        INA1B_count = 0;
        previous_millis = millis();
      }
    } while (distance_traveled < (WHEEL_DIAMETER * 18) * (angle / 360));
    stop();
    toggleRightBlinker(false); // Blinker off
  }
  // Left
  else {
    do {
      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2B, LOW);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, LOW);
      digitalWrite(INA2A, HIGH);

      // Update RPMs every 50ms
      if (millis() - previous_millis >= 25) {
        toggleLeftBlinker(!digitalRead(FRONT_BLINKER_LEFT)); // Flip blinker state
        rpm = INA1A_count * 12.5;
        distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
        INA1A_count = 0;
        previous_millis = millis();
      }
    } while (distance_traveled < (WHEEL_DIAMETER * 18) * (angle / 360));
    stop();
    toggleLeftBlinker(false); // Blinker off
  }
}

int adjustToLine(struct lineFollowThread* thread, int speed=60) {
  PT_BEGIN(&thread->pt);

  while (1) {  // Infinite loop - thread never ends
    thread->done = false;

    // Read sensors
    thread->left = digitalRead(LINE_TRACKING_LEFT);
    thread->center = digitalRead(LINE_TRACKING_CENTER);
    thread->right = digitalRead(LINE_TRACKING_RIGHT);
    // Serial.print(thread->left);
    // Serial.print(thread->center);
    // Serial.print(thread->right);
    // Serial.println();

    // On center - done immediately
    if (thread->center) {
      thread->done = true;
      PT_YIELD(&thread->pt);  // Yield but keep running
    }
    // Off center to right - need to turn left
    else if (thread->left) {
      while (!thread->center) {
        thread->center = digitalRead(LINE_TRACKING_CENTER);

        // Turn left
        analogWrite(MOTOR_PWM_B, speed);
        digitalWrite(INA1B, HIGH);
        digitalWrite(INA2B, LOW);

        analogWrite(MOTOR_PWM_A, speed);
        digitalWrite(INA1A, LOW);
        digitalWrite(INA2A, HIGH);

        PT_YIELD(&thread->pt);
      }
      stop();
      thread->done = true;
    }
    // Off center to left - need to turn right
    else if (thread->right) {
      while (!thread->center) {
        thread->center = digitalRead(LINE_TRACKING_CENTER);

        // Turn right
        analogWrite(MOTOR_PWM_B, speed);
        digitalWrite(INA1B, LOW);
        digitalWrite(INA2B, HIGH);

        analogWrite(MOTOR_PWM_A, speed);
        digitalWrite(INA1A, HIGH);
        digitalWrite(INA2A, LOW);

        PT_YIELD(&thread->pt);
      }
      stop();
      thread->done = true;
    }
    else {
      stop();
      thread->done = false;
      PT_YIELD(&thread->pt);  // Yield even when not on line
    }
  }
  PT_END(&thread->pt);
}

void loop() {
  playSong(&ptSong);
  adjustToLine(&ptLineFollow, 75);

  if (ptLineFollow.done) {
    forward(60);
  }
  //delay(500);
}
