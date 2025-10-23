const String VALID_COMMANDS[] = {
  "toggle-headlights"
};

/// @brief Reads a message sent via serial
///
/// Message will be converted to an all-lowercase string
///
/// @param serial Reference to a hardware serial port
/// @return The message sent via serial
String readMessage(HardwareSerial &serial) {
  String message = "";

  while (serial.available()) {
    char c = serial.read();
    if (c == '\n') break; // End of string

    message += tolower(c);
    delay(2); // Small delay to let bytes arrive
  }

  return message;
}

void setup() {
  Serial2.begin(38400);
  Serial.begin(9600);
}

void loop() {
  String command = readMessage(&Serial2);
  switch(command) {
    case "toggle-headlights":
      Serial.println("toggling headlights");
      break;
  }
}
