#include <Wire.h>
#include <SoftWire.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>

#define OLED_SDA 4
#define OLED_SCL 5

SoftWire myWire(OLED_SDA, OLED_SCL); // SDA, SCL
// U8g2 object for SH1106 128x64 using Software I2C
U8G2_SH1106_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

TinyGPSPlus gps;

HardwareSerial GPS(1);
HardwareSerial GSM(2);

String content[4] = {"72", "96", "0", "+910123456789"};
String arr[3] = {"Safe", "Fall Prob Detected", "Fall Detected"};
String Location = "0.0,0.0";
String readString = "";

volatile bool allowReceive = false;  // Global flag to allow data reception

void receiveEvent(int bytes) {
  if (!allowReceive) {
    return;
  }
  String readString = "";
  while (Wire.available()) {
    char c = Wire.read();
    readString = readString + String(c);
  }
  int prevIndex = 0;
  for (int i = 0; i < 4; i++) {
    int delimIndex = readString.indexOf(',', prevIndex);
    if (delimIndex == -1) delimIndex = readString.length();
    content[i] = readString.substring(prevIndex, delimIndex);
    prevIndex = delimIndex + 1;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Monitor Started...");

  while (!Wire.begin(8)); // Slave address = 0x08
  Wire.onReceive(receiveEvent);
  allowReceive = false;

  Serial.println("I2C bus started");
  delay(500);

  GPS.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX for GPS
  Serial.println("GPSSerial Monitor Started...");
  delay(500);

  GSM.begin(9600, SERIAL_8N1, 26, 25);  // RX, TX for GSM
  Serial.println("GSMSerial Monitor Started...");
  delay(500);

  Serial.println("Initializing SIM800L...");
  sendCommand("AT");
  sendCommand("AT+CMGF=1");
  sendCommand("AT+CSCS=\"GSM\"");

  pinMode(12, INPUT_PULLUP);

  myWire.begin(); // Start the software I2C
  delay(1000);

  display.begin(); // Start U8g2
  display.clearBuffer();
  // display.setFont(u8g2_font_ncenB08_tr);
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, 10, "Display Started...");
  display.sendBuffer();

  Serial.println("Display Started...");
  text_display("Booting up the device");

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  allowReceive = true;
}

void loop() {
  Location = getLocation();
  if (!digitalRead(12)) {
    allowReceive = false;
    // Location = getLocation();
    text_display("Called for help on  " + content[3]);
    String temp = "Urgent need for help\n\nLocation: https://maps.google.com/maps?q=" + Location;
    SOS(content[3], temp);
    delay(1000);
    allowReceive = true;
  }
  Serial.println("BPM: " + content[0]);
  Serial.println("SpO2: " + content[1]);
  Serial.println("Decision: " + content[2]);
  Serial.println("Phone Number: " + content[3]);

  if (content[2] == "2") {
    allowReceive = false;
    // Location = getLocation();
    digitalWrite(14, HIGH);
    OLED_display(content[0], content[1], "Fall detected calling on " + content[3]);
    String temp = "Patient Fell Down\n\nLocation: https://maps.google.com/maps?q=" + Location;
    SOS(content[3], temp);
    digitalWrite(14, LOW);
    allowReceive = true;

  } else {
    OLED_display(content[0], content[1], arr[content[2].toInt()]);
    if (content[2] == "1") {
      allowReceive = false;
      unsigned long startBlink = millis();
      while (millis() - startBlink < 3000) {
        digitalWrite(14, HIGH);
        delay(500);
        digitalWrite(14, LOW);
        delay(500);
      }
      allowReceive = true;
    }
  }
}

void sendCommand(const String& cmd) {
  GSM.println(cmd);
  delay(1000);

  while (GSM.available()) {
    Serial.write(GSM.read());
  }
}

void SOS(String number, String message) {
  sendCommand("AT");
  sendCommand("AT+CMGF=1");
  sendCommand("AT+CSCS=\"GSM\"");
  sendSMS(number.c_str(), message.c_str());
}

void sendSMS(const char* number, const char* message) {
  GSM.print("AT+CMGS=\"");
  GSM.print(number);
  GSM.println("\"");
  delay(1000);
  GSM.print(message);
  GSM.write(26); // Ctrl+Z to send SMS

  while (GSM.available()) {
    Serial.write(GSM.read());
  }
  delay(2000);

  Serial.println("SMS Sent!");
}

// void readMessage() {
//   int prevIndex = 0;
//   for (int i = 0; i < 4; i++) {
//     int delimIndex = readString.indexOf(',', prevIndex);
//     if (delimIndex == -1) delimIndex = readString.length();
//     content[i] = readString.substring(prevIndex, delimIndex);
//     prevIndex = delimIndex + 1;
//   }
// }

void OLED_display(String bpm, String spo2, String state) {
  display.clearBuffer();
  // display.setFont(u8g2_font_ncenB08_tr);

  display.setFont(u8g2_font_5x7_tr);

  // display.setCursor(5, 10);
  display.drawStr(5,10, "Patient Status:");

  // display.setCursor(5, 25);
  display.drawStr(5,20, "BPM: ");
  display.drawStr(30, 20, bpm.c_str());   // Shift to the right for the value

  // display.setCursor(5, 40);
  display.drawStr(5, 30, "SpO2: ");
  display.drawStr(30, 30, spo2.c_str());  // Shift to the right for the value
  display.drawStr(50, 30, "%");           // % sign next to spo2

  // display.setCursor(5, 40);
  display.drawStr(5, 40, state.c_str());  // Patient state

  display.sendBuffer();
}
void text_display(String text) {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);  // Use smaller font for more text

  int y = 10;  // Starting y position (a bit down)
  int lineHeight = 10;  // Height of each line (adjust if needed)

  while (text.length() > 0) {
    String line = text.substring(0, min(20, int(text.length()))); // Take first 20 characters
    display.drawStr(0, y, line.c_str());  // Draw the line
    text = text.substring(min(20, int(text.length())));  // Remove the part we just drew
    y += lineHeight;  // Move to next line
  }

  display.sendBuffer();
}

String getLocation() {
  String Loc = "Can't Detect Location";
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
    if (gps.location.isUpdated()) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      Loc = String(lat, 10) + "," + String(lng, 10);
      break;
    }
  }
  return Loc;
}
