const byte HIGHBEAM_LEFT = 47;
const byte HIGHBEAM_RIGHT = 41;

void toggleHighbeams() {
  bool state = digitalRead(HIGHBEAM_LEFT);
  Serial.println(state);
  digitalWrite(HIGHBEAM_LEFT, !state);
  digitalWrite(HIGHBEAM_RIGHT, !state);
}

struct Command {
  String command;
  void (*action)();
};

const Command VALID_COMMANDS[] = {
  {"toggle-highbeams", toggleHighbeams}
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

void setup() {
  Serial2.begin(38400);
  Serial.begin(9600);
}

void loop() {
  String command = readMessage();
  for (Command com: VALID_COMMANDS) {
    if (com.command == command) {
      com.action();
    }
  }
}
