#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 32, &Wire, -1);

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

unsigned long startingMillis = millis();

/// @brief Updates the OLED screen with elapsed time
///
/// Timer starting time taking from variable startingMillis
void updateTimer() {
  unsigned long millisPassed = millis() - startingMillis;
  display.clearDisplay();
  display.setCursor(0, 0);
  String timeString = "";
  if (millisPassed >= 60000) {
    timeString = String(millisPassed / 60000) + ":";
  }
  timeString += String((millisPassed % 60000) / 1000) + "." + String(millisPassed % 1000);
  display.print(timeString);
  display.display();
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
    updateTimer();
    
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
      updateTimer();

      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2B, HIGH);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA1B, LOW);

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
      updateTimer();

      analogWrite(MOTOR_PWM_B, speed);
      digitalWrite(INA1B, HIGH);
      digitalWrite(INA2B, LOW);

      analogWrite(MOTOR_PWM_A, speed);
      digitalWrite(INA1A, HIGH);
      digitalWrite(INA1B, HIGH);

      // Update RPMs every 50ms
      if (millis() - previous_millis >= 50) {
        toggleLeftBlinker(!digitalRead(FRONT_BLINKER_LEFT)); // Flip blinker state
        rpm = INA1A_count * 6.25;
        distance_traveled += (rpm * (100 / 60000.0)) * (3.14 * WHEEL_DIAMETER);
        INA1A_count = 0;
        previous_millis = millis();
      }
    } while (distance_traveled < (WHEEL_DIAMETER * 18) * (angle / 360));
    stop();
    toggleLeftBlinker(false); // Blinker off
  }
}

void setup() {
  Serial2.begin(38400);
  Serial.begin(9600);

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

int travel_speed = 60; // PWM
int turning_speed = 80; // PWM
int pause = 35; // Delay between driving and turning
void loop() {
  String command = readMessage();

  if (command == "start") {
    startingMillis = millis();
    travelDistance(54, travel_speed);
    delay(pause);
    turn(true, 85, turning_speed);
    delay(pause);
    travelDistance(52, travel_speed);
    delay(pause);
    turn(true, 79, turning_speed);
    delay(pause);
    travelDistance(53, travel_speed);
    delay(pause);
    turn(true, 79, turning_speed);
    delay(pause);
    travelDistance(46, travel_speed);
  }

  command = "";
}
