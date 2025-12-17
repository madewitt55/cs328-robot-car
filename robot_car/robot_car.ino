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
  unsigned long last_blink;
  bool left;
  bool center;
  bool right;
  bool done;
};
struct lineFollowThread ptLineFollow;

struct distanceThread {
  struct pt pt;
  unsigned long lastMeasurement;
  float distance;
  bool obstacle;
};
struct distanceThread ptDistance;

struct pt ptDisplay;

Adafruit_SSD1306 display(128, 32, &Wire, -1);
// 11 output 12 input

const byte TRIG = 11;
const byte ECHO = 12;

const byte BUZZER = 13;

const float WHEEL_DIAMETER = 2.6875; // Inches

const int DEFAULT_SPEED = 100;

const unsigned long MEASUREMENT_INTERVAL = 100; // ms between measurements
const float OBSTACLE_THRESHOLD = 17; // inches - distance to consider as obstacle

const byte HIGHBEAM_LEFT = 47;
const byte HIGHBEAM_RIGHT = 41;

const byte FRONT_BLINKER_LEFT = 49;
const byte REAR_BLINKER_LEFT = 31;
const byte FRONT_BLINKER_RIGHT = 43;
const byte REAR_BLINKER_RIGHT = 37;

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
  PT_INIT(&ptDistance.pt);
  PT_INIT(&ptDisplay);

  pinMode(BUZZER, OUTPUT);

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(FRONT_BLINKER_LEFT, OUTPUT);
  pinMode(REAR_BLINKER_LEFT, OUTPUT);
  pinMode(FRONT_BLINKER_RIGHT, OUTPUT);
  pinMode(REAR_BLINKER_RIGHT, OUTPUT);
  pinMode(BRAKE_LIGHT_LEFT, OUTPUT);
  pinMode(BRAKE_LIGHT_RIGHT, OUTPUT);
  toggleLeftBlinker(false);
  toggleRightBlinker(false);

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

PT_THREAD(playSong(struct songThread* thread)) {
  static int melody[] = {
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

  static int durations[] = {
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
  
  static const int size = sizeof(durations) / sizeof(int);
  static unsigned long noteStartTime = 0;
  static unsigned long noteDuration = 0;
  
  PT_BEGIN(&thread->pt);

  for (thread->note = 0; thread->note < size; thread->note++) {
    // Calculate duration for this note
    noteDuration = (1000 / durations[thread->note]) * 1.30;
    
    // Start the note
    if (melody[thread->note] != REST) {
      tone(BUZZER, melody[thread->note]);
    }
    noteStartTime = millis();

    // Wait for note duration while yielding
    while ((millis() - noteStartTime) < noteDuration) {
      PT_YIELD(&thread->pt);
    }

    noTone(BUZZER);
  }
  
  // Song finished, restart
  thread->note = 0;
  PT_RESTART(&thread->pt);

  PT_END(&thread->pt);
}

/// @brief Measures distance using ultrasonic sensor
///
/// @return Distance in inches
float measureDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Read echo pulse duration
  float duration = pulseIn(ECHO, HIGH);
  float distance = (duration*.0343)/2;  
  
  return distance;
}

/// @brief Protothread for continuous distance monitoring
///
/// Updates distance reading at regular intervals and sets obstacle flag
///
/// @param thread - Pointer to distance thread structure
PT_THREAD(monitorDistance(struct distanceThread* thread)) {
  PT_BEGIN(&thread->pt);
  
  while(1) {
    // Check if enough time has passed since last measurement
    if (millis() - thread->lastMeasurement >= MEASUREMENT_INTERVAL) {
      // Measure distance
      thread->distance = measureDistance();
      
      // Check for obstacle
      if (thread->distance > 0 && thread->distance < OBSTACLE_THRESHOLD) {
        thread->obstacle = true;
      } else {
        thread->obstacle = false;
      }
      
      // Debug output
      Serial.print("Distance: ");
      Serial.print(thread->distance);
      Serial.print(" inches, Obstacle: ");
      Serial.println(thread->obstacle ? "YES" : "NO");
      
      thread->lastMeasurement = millis();
    }
    
    PT_YIELD(&thread->pt);
  }
  
  PT_END(&thread->pt);
}

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

PT_THREAD(adjustToLine(struct lineFollowThread* thread, int speed=60)) {
  const unsigned long BLINK_DELAY = 200;
  PT_BEGIN(&thread->pt);

  thread->done = false;

  // Read sensors
  thread->left = digitalRead(LINE_TRACKING_LEFT);
  thread->center = digitalRead(LINE_TRACKING_CENTER);
  thread->right = digitalRead(LINE_TRACKING_RIGHT);

  // On center - done immediately
  if (thread->center) {
    thread->done = true;
    PT_YIELD(&thread->pt);  // Yield but keep running
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
    //stop();
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
    //stop();
    toggleRightBlinker(false);
    thread->done = true;
  }
  PT_END(&thread->pt);
}

void loop() {
  //Serial.println(measureDistance());
  PT_SCHEDULE(monitorDistance(&ptDistance));
  PT_SCHEDULE(adjustToLine(&ptLineFollow, 75));
  PT_SCHEDULE(displayOLED(&ptDisplay));
  PT_SCHEDULE(playSong(&ptSong));
  
  if (ptDistance.obstacle) {
    stop();
    toggleHighbeams(true);
    delay(500);
    while (ptDistance.obstacle) {
      PT_SCHEDULE(monitorDistance(&ptDistance));
      PT_SCHEDULE(adjustToLine(&ptLineFollow, 85));
      PT_SCHEDULE(displayOLED(&ptDisplay));
      PT_SCHEDULE(playSong(&ptSong));

      // Too close; force turn
      if (ptDistance.distance <= 6.25) {
        // Turn right until back on line
        unsigned long last_blink = millis();
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

          if (millis() - last_blink >= 200) {
            toggleRightBlinker(!digitalRead(FRONT_BLINKER_RIGHT));
            last_blink = millis();
          }
        }
        toggleRightBlinker(false);
      }

      if (ptLineFollow.done) {
        toggleBrakeLights(false);
        forward(60);
      }
    }
    toggleBrakeLights(false);
    toggleHighbeams(false);
  }
  else if (ptLineFollow.done) {
    toggleBrakeLights(false);
    forward(60);
  }
}
