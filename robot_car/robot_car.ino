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

void toggleHighbeams(void* _) {
  bool state = digitalRead(HIGHBEAM_LEFT);
  digitalWrite(HIGHBEAM_LEFT, state);
  digitalWrite(HIGHBEAM_RIGHT, state);
}

void toggleBlinker(void* arg) {
  const char* side = (const char*)arg;

  if (side == "left") {
    digitalWrite(FRONT_BLINKER_LEFT, HIGH);
  }


}

struct Command {
  String command;
  void (*action)(void*);
  void* arg;  
};

const Command VALID_COMMANDS[] = {
  {"highbeams-on", toggleHighbeams, nullptr},
  {"left-blinker", toggleBlinker, (void*)"left"}
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

int INA1A_count = 0;
int INA1B_count = 0;
int INA2A_count = 0;
int INA2B_count = 0;

void encoderIncrement() {
  INA1A_count++;
  INA1B_count++;
  INA2A_count++;
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), encoderIncrement, FALLING);
}

float readRPMs(int* counter) {
  *counter = 0;
  //delay(100);
  return (*counter) * 3.125;
}

void loop() {
  analogWrite(MOTOR_PWM_A, 50);
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  Serial.println(readRPMs(INA1A_count));

  // String command = readMessage();
  // for (Command com: VALID_COMMANDS) {
  //   if (com.command == command) {
  //     com.action();
  //   }
  // }
}
