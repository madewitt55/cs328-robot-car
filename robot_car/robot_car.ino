#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <protothreads.h>
#include "pitches.h"

struct songThread {
  struct pt pt;
  int note;
  unsigned long last_note_time;
};
struct songThread ptSong;

struct lineFollowThread {
  struct pt pt;
  unsigned long last_blink;
  bool left;
  bool center;
  bool right;
  bool done;
};
struct lineFollowThread ptLineFollow;

struct distanceThread {
  struct pt pt;
  unsigned long last_measurement;
  float distance;
  bool obstacle;
};
struct distanceThread ptDistance;

struct pt ptDisplay;

// OLED
Adafruit_SSD1306 display(128, 32, &Wire, -1);

// Ultrasonic range sensor
const byte TRIG = 11;
const byte ECHO = 12;
const unsigned long MEASUREMENT_INTERVAL = 100; // ms between measurements
const float OBSTACLE_THRESHOLD = 17; // Inches

// Buzzer
const byte BUZZER = 13;
static const int MELODY[] = {
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
static const int DURATIONS[] = {
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

// Motors and encoders
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
const float WHEEL_DIAMETER = 2.6875; // Inches
const int DEFAULT_TRAVEL_SPEED = 100;
const int DEFAULT_TURNING_SPEED = 60;

// Lights
const byte HIGHBEAM_LEFT = 47;
const byte HIGHBEAM_RIGHT = 41;
const byte FRONT_BLINKER_LEFT = 49;
const byte REAR_BLINKER_LEFT = 31;
const byte FRONT_BLINKER_RIGHT = 43;
const byte REAR_BLINKER_RIGHT = 37;
const byte BRAKE_LIGHT_LEFT = 27;
const byte BRAKE_LIGHT_RIGHT = 33;
const unsigned long BLINK_DELAY = 200;

// Line-following
const byte LINE_TRACKING_LEFT = 8;
const byte LINE_TRACKING_CENTER = 7;
const byte LINE_TRACKING_RIGHT = 6;

void setup() {
  Serial.begin(9600);

  // Initialize threads
  PT_INIT(&ptSong.pt);
  PT_INIT(&ptLineFollow.pt);
  PT_INIT(&ptDistance.pt);
  PT_INIT(&ptDisplay);

  // Motors and encoders
  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), ISR_1A, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), ISR_1B, FALLING);
  analogWrite(MOTOR_PWM_A, 0);
  analogWrite(MOTOR_PWM_B, 0);
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, LOW);
  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, LOW);

  // Lights
  pinMode(FRONT_BLINKER_LEFT, OUTPUT);
  pinMode(REAR_BLINKER_LEFT, OUTPUT);
  pinMode(FRONT_BLINKER_RIGHT, OUTPUT);
  pinMode(REAR_BLINKER_RIGHT, OUTPUT);
  pinMode(BRAKE_LIGHT_LEFT, OUTPUT);
  pinMode(BRAKE_LIGHT_RIGHT, OUTPUT);
  toggleLeftBlinker(false);
  toggleRightBlinker(false);
  toggleBrakeLights(false);

  // Ultrasonic range sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Buzzer
  pinMode(BUZZER, OUTPUT);

  // Line-following
  pinMode(LINE_TRACKING_LEFT, INPUT_PULLUP);
  pinMode(LINE_TRACKING_CENTER, INPUT_PULLUP);
  pinMode(LINE_TRACKING_RIGHT, INPUT_PULLUP);

  // OLED
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
}

/// @brief Plays a song through a buzzer
///
/// @param thread - Thread to to be used
PT_THREAD(playSong(struct songThread* thread)) {
  static const int size = sizeof(DURATIONS) / sizeof(int);
  static unsigned long note_start_time = 0;
  static unsigned long note_duration = 0;
  
  PT_BEGIN(&thread->pt);

  for (thread->note = 0; thread->note < size; thread->note++) {
    // Calculate not duration
    note_duration = (1000 / DURATIONS[thread->note]) * 1.30;
    
    // Start note
    if (MELODY[thread->note] != REST) {
      tone(BUZZER, MELODY[thread->note]);
    }
    note_start_time = millis();

    // Wait for note duration completion
    while ((millis() - note_start_time) < note_duration) {
      PT_YIELD(&thread->pt);
    }

    noTone(BUZZER); // Turn off
  }
  
  // Song finished, restart
  thread->note = 0;
  PT_RESTART(&thread->pt);

  PT_END(&thread->pt);
}

/// @brief Measures distance using ultrasonic range sensor
///
/// @return Distance in inches
float measureDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  float duration = pulseIn(ECHO, HIGH); // Read duration
  float distance = (duration*.0343)/2;  // Calculate distance
  
  return distance;
}

/// @brief Continuously monitors obstacle distance (cm)
///
/// Updates distance reading at regular intervals and sets obstacle flag
///
/// @param thread - Thread to be used
PT_THREAD(monitorDistance(struct distanceThread* thread)) {
  PT_BEGIN(&thread->pt);
  
  while(1) {
    // Check if enough time has passed since last measurement
    if (millis() - thread->last_measurement >= MEASUREMENT_INTERVAL) {
      thread->distance = measureDistance(); // Measure distance
      
      // Check for obstacle
      if (thread->distance > 0 && thread->distance < OBSTACLE_THRESHOLD) {
        thread->obstacle = true;
      } else {
        thread->obstacle = false;
      }
      
      thread->last_measurement = millis();
    }
    
    PT_YIELD(&thread->pt);
  }
  
  PT_END(&thread->pt);
}

/// @brief Displays obstacle distance data on OLED
///
/// @param thread - Thread to be used
PT_THREAD(displayOLED(struct pt* thread)) {
  PT_BEGIN(thread);
  
  while(1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Distance: ");
    display.setCursor(0, 10);
    display.print(ptDistance.distance);
    display.print(" cm");
    display.setCursor(0, 20);
    display.print(ptDistance.obstacle ? "OBSTACLE!" : "Clear");
    display.display();
    
    PT_YIELD(thread);
  }
  
  PT_END(thread);
}

/// @brief Turns the car to adjust to followed line
///
/// Only adjusts the car to the line, does not drive the car forward
///
/// @param thread - Thread to be used
PT_THREAD(adjustToLine(struct lineFollowThread* thread, int speed=DEFAULT_TURNING_SPEED)) {
  PT_BEGIN(&thread->pt);

  thread->done = false;

  // Read sensors
  thread->left = digitalRead(LINE_TRACKING_LEFT);
  thread->center = digitalRead(LINE_TRACKING_CENTER);
  thread->right = digitalRead(LINE_TRACKING_RIGHT);

  // On center
  if (thread->center) {
    thread->done = true;
    PT_YIELD(&thread->pt);
  }
  // Off center to right - need to turn left
  else if (thread->left) {
    thread->last_blink = millis();
    while (!thread->center) {
      thread->center = digitalRead(LINE_TRACKING_CENTER);

      if (millis() - thread->last_blink >= BLINK_DELAY) {
        toggleLeftBlinker(!digitalRead(FRONT_BLINKER_LEFT));
        thread->last_blink = millis();
      }

      // Turn left
      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2B, LOW);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, LOW);
      digitalWrite(INA2A, HIGH);

      PT_YIELD(&thread->pt);
    }
    toggleLeftBlinker(false);
    thread->done = true;
  }
  // Off center to left - need to turn right
  else if (thread->right) {
    thread->last_blink = millis();
    while (!thread->center) {
      thread->center = digitalRead(LINE_TRACKING_CENTER);

      if (millis() - thread->last_blink >= BLINK_DELAY) {
        toggleRightBlinker(!digitalRead(FRONT_BLINKER_RIGHT));
        thread->last_blink = millis();
      }

      // Turn right
      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, LOW);
      digitalWrite(INA2B, HIGH);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA2A, LOW);

      PT_YIELD(&thread->pt);
    }
    toggleRightBlinker(false);
    thread->done = true;
  }
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
  digitalWrite(FRONT_BLINKER_LEFT, state);
  digitalWrite(REAR_BLINKER_LEFT, state);
}

/// @brief Turns on or off right blinker
///
/// @param state - State to set right blinker to
void toggleRightBlinker(bool state) {
  digitalWrite(FRONT_BLINKER_RIGHT, state);
  digitalWrite(REAR_BLINKER_RIGHT, state);
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
void forward(int speed=DEFAULT_TRAVEL_SPEED) {
  toggleBrakeLights(false); // Brake lights off while moving
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
void travelDistance(float distance, int speed=DEFAULT_TRAVEL_SPEED) {
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
void turn(bool direction, float angle, int speed=DEFAULT_TURNING_SPEED) {
  toggleBrakeLights(false); // Brake lights off while turning

  unsigned long previous_millis = millis();
  unsigned long last_blink = millis();
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
        rpm = INA1B_count * 6.25;
        distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
        INA1B_count = 0;
        previous_millis = millis();
      }
      // Flash blinker
      if (millis() - last_blink >= BLINK_DELAY) {
        toggleRightBlinker(!digitalRead(FRONT_BLINKER_RIGHT)); // Flip blinker state
        last_blink = millis();
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
      if (millis() - previous_millis >= 50) {
        rpm = INA1A_count * 6.25;
        distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
        INA1A_count = 0;
        previous_millis = millis();
      }
      // Flash blinker
      if (millis() - last_blink >= BLINK_DELAY) {
        toggleLeftBlinker(!digitalRead(FRONT_BLINKER_LEFT)); // Flip blinker state
        last_blink = millis();
      }
    } while (distance_traveled < (WHEEL_DIAMETER * 18) * (angle / 360));
    stop();
    toggleLeftBlinker(false); // Blinker off
  }
}

void loop() {
  // Schedule all threads
  PT_SCHEDULE(monitorDistance(&ptDistance));
  PT_SCHEDULE(adjustToLine(&ptLineFollow, 75));
  PT_SCHEDULE(displayOLED(&ptDisplay));
  PT_SCHEDULE(playSong(&ptSong));

  // Obstacle detected
  if (ptDistance.obstacle) {
    stop();
    toggleHighbeams(true);
    delay(500);
    while (ptDistance.obstacle) {
      // Ensure all threads continue to run
      PT_SCHEDULE(monitorDistance(&ptDistance));
      PT_SCHEDULE(adjustToLine(&ptLineFollow, 85));
      PT_SCHEDULE(displayOLED(&ptDisplay));
      PT_SCHEDULE(playSong(&ptSong));

      // Too close - force turn
      if (ptDistance.distance <= 6.25) {
        unsigned long last_blink = millis();

        // Turn right until back on line
        while (!ptLineFollow.done) {
          PT_SCHEDULE(adjustToLine(&ptLineFollow, 85));
          PT_SCHEDULE(displayOLED(&ptDisplay));
          PT_SCHEDULE(playSong(&ptSong));

          analogWrite(MOTOR_PWM_B, 100);
          digitalWrite(INA1B, LOW);
          digitalWrite(INA2B, HIGH);
          analogWrite(MOTOR_PWM_A, 100);
          digitalWrite(INA1A, HIGH);
          digitalWrite(INA2A, LOW);

          // Flash blinker
          if (millis() - last_blink >= BLINK_DELAY) {
            toggleRightBlinker(!digitalRead(FRONT_BLINKER_RIGHT));
            last_blink = millis();
          }
        }
        toggleRightBlinker(false);
      }

      // Car is on target - proceed forward
      if (ptLineFollow.done) {
        toggleBrakeLights(false);
        forward(60);
      }
    }
  }

  // Car is on target - proceed forward
  else if (ptLineFollow.done) {
    toggleBrakeLights(false);
    forward(60);
  }
}
