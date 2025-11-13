const byte HIGHBEAM_LEFT = 47;
const byte HIGHBEAM_RIGHT = 41;

const byte FRONT_BLINKER_LEFT = 49;
const byte FRONT_BLINKER_RIGHT = 43;

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

void toggleHighbeams(void* arg) {
  bool state = *(bool*)arg;
  digitalWrite(HIGHBEAM_LEFT, state);
  digitalWrite(HIGHBEAM_RIGHT, state);
}

void toggleLeftBlinker(void* arg) {
  bool state = *(bool*)arg;
  Serial.println(state);
  digitalWrite(FRONT_BLINKER_LEFT, state);
}

void toggleRightBlinker(void* arg) {
  bool state = *(bool*)arg;
  digitalWrite(FRONT_BLINKER_RIGHT, state);
}

struct Command {
  String command;
  void (*action)(void*);
  void* arg;  
};

bool TRUE_ARG = true;
bool FALSE_ARG = false;
const Command VALID_COMMANDS[] = {
  {"highbeams-on", toggleHighbeams, &TRUE_ARG},
  {"highbeams-off", toggleHighbeams, &FALSE_ARG},
  {"left-blinker-on", toggleLeftBlinker, &TRUE_ARG},
  {"left-blinker-off", toggleLeftBlinker, &FALSE_ARG},
  {"right-blinker-on", toggleRightBlinker, &TRUE_ARG},
  {"right-blinker-off", toggleRightBlinker, &FALSE_ARG}
};

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

void setup() {
  Serial2.begin(38400);
  Serial.begin(9600);

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

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
}

unsigned int speed = 0;
String pwm_data = "";
String rpm_data = "";

void loop() {
  String command = readMessage();
    if (command == "waylon likes boys") {
      while (speed <= 255) {
        analogWrite(MOTOR_PWM_A, speed);
        digitalWrite(INA1A, HIGH);
        analogWrite(MOTOR_PWM_B, speed);
        digitalWrite(INA1B, HIGH);

        delay(30);

        INA1B_count = 0;
        delay(100);

        pwm_data += (String(speed) + ",");
        rpm_data += (String(INA1B_count * 3.125) + ",");
        Serial2.println("PWM: " + String(speed) + ", RPMS: " + String(INA1B_count * 3.125)); // Print individual data point
        speed += 5;
      }
      analogWrite(MOTOR_PWM_A, 0);
      analogWrite(MOTOR_PWM_B, 0);

        // Print full data
        Serial2.println("PWM: " + pwm_data);
        Serial2.println("RPM: " + rpm_data);
      

      // Sequence complete
      // if (speed > 255) {
      //   analogWrite(MOTOR_PWM_A, 0);
      //   analogWrite(MOTOR_PWM_B, 0);

      //   // Print full data
      //   Serial2.println("PWM: " + pwm_data);
      //   Serial2.println("RPM: " + rpm_data);

      //   while (true) {} // Wait to stop speed up sequence 
      // };
    }
  }
