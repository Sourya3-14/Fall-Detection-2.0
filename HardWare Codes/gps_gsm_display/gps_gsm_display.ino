#include <Wire.h>
#include <SoftWire.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>

#define OLED_SDA 4
#define OLED_SCL 5

#define BAT1_PIN 34  // ADC1 pin for Battery 1 (ESP32 side)
#define BAT2_PIN 35  // ADC1 pin for Battery 2 (GSM side)
int percent = 0;

SoftWire myWire(OLED_SDA, OLED_SCL); // SDA, SCL
// U8g2 object for SH1106 128x64 using Software I2C
U8G2_SH1106_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);

TinyGPSPlus gps;

HardwareSerial GPS(1);
HardwareSerial GSM(2);

String content[5] = {"72", "96", "0", "+910123456789","0"};
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
  for (int i = 0; i < 5; i++) {
    int delimIndex = readString.indexOf(',', prevIndex);
    if (delimIndex == -1) delimIndex = readString.length();
    content[i] = readString.substring(prevIndex, delimIndex);
    prevIndex = delimIndex + 1;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Monitor Started...");

  while (!Wire.begin(8)); // Slave address = 0x08
  Wire.onReceive(receiveEvent);
  allowReceive = false;

  Serial.println("I2C bus started");
  delay(100);

  GPS.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX for GPS
  Serial.println("GPSSerial Monitor Started...");
  delay(100);

  GSM.begin(9600, SERIAL_8N1, 26, 25);  // RX, TX for GSM
  Serial.println("GSMSerial Monitor Started...");
  delay(100);

  Serial.println("Initializing SIM800L...");
  sendCommand("AT");
  sendCommand("AT+CMGF=1");
  sendCommand("AT+CSCS=\"GSM\"");

  pinMode(12, INPUT_PULLUP);

  myWire.begin(); // Start the software I2C
  delay(1000);

  display.begin(); // Start U8g2
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  Serial.println("Display Started...");
  text_display("Booting up the device");

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(10);//debounce delay
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  allowReceive = true;
}

void loop() {
  getBattery();
  if(content[5]=="1" && percent > 0){
    // Location = getLocation();
    if (!digitalRead(12)) {
      allowReceive = false;
      Location = getLocation();
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
    if(content[0]=="0" && content[1]=="0"){
      text_display("Please insert finger at the Oximeter");
    }
    else if (content[2] == "2") {
      allowReceive = false;
      Location = getLocation();
      digitalWrite(2, HIGH);
      OLED_display(content[0], content[1], "Fall detected alert sent on" + content[3]);
      String temp = "Patient Fell Down\n\nLocation: https://maps.google.com/maps?q=" + Location;
      SOS(content[3], temp);
      digitalWrite(2, LOW);
      content[2] == "0";
      allowReceive = true;
      delay(200);
    } 
    else if(content[2] == "1") {
      OLED_display(content[0], content[1], arr[content[2].toInt()]);
      allowReceive = false;
      unsigned long startBlink = millis();
      while (millis() - startBlink < 3000) {
        digitalWrite(2, HIGH);
        delay(500);
        digitalWrite(2, LOW);
        delay(500);
      }
      content[2] == "0";
      allowReceive = true;
      delay(200);
    }
    else{
      OLED_display(content[0], content[1], arr[content[2].toInt()]);
    }
  }
  else if(percent>0){
    text_display("Device started waiting for the data(Use the app)");
  }
  else{
    text_display("battery too low put the device on charging");
    display.clearBuffer();
    delay(100);
    while(precent<=0)
      getBattery();
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
void OLED_display(String bpm, String spo2, String state) {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(5,10, "Patient Status:");
  display.drawStr(5,20, "BPM: ");
  display.drawStr(30, 20, bpm.c_str());   // Shift to the right for the value
  display.drawStr(5, 30, "SpO2: ");
  display.drawStr(30, 30, spo2.c_str());  // Shift to the right for the value
  display.drawStr(50, 30, "%");           // % sign next to spo2

  int y = 40;  // Starting y position (a bit down)
  int lineHeight = 10;  // Height of each line (adjust if needed)
  while (state.length() > 0) {
    String line = state.substring(0, min(20, int(state.length()))); // Take first 20 characters
    display.drawStr(5, y, line.c_str());  // Draw the line
    state = (state.substring(min(20, int(state.length())))).trim();  // Remove the part we just drew
    y += lineHeight;  // Move to next line
  }
  displayBattery();
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
  displayBattery();
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
void displayBattery(){
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(80,60,"Bat: ");
  display.drawStr(110,60,String(percent).c_str());
  display.drawStr(120,60,"%");
}
void getBattery(){
  int raw1 = analogRead(BAT1_PIN);
  int raw2 = analogRead(BAT2_PIN);

  // Convert ADC value to voltage (using 3.3V reference and 10k-10k divider)
  float voltage1 = raw1 * (3.3 / 4095.0) * 2;
  float voltage2 = raw2 * (3.3 / 4095.0) * 2;

  // Get battery percentage
  int percent1 = getBatteryPercent(voltage1);
  int percent2 = getBatteryPercent(voltage2);
  if(percent != 0){
    int smoothed = percent;
    percent = min(percent1,percent2);
    percent = 0.1 * percent + (1 - 0.1) * smoothed; // smoothing factor 0.1
  }
  else{
    percent = min(percent1,percent2);
  }
}
int getBatteryPercent(float voltage) {
  const float minVoltage = 2.5; // 0%
  const float maxVoltage = 3.6; // 100%

  if (voltage <= minVoltage) return 0;
  if (voltage >= maxVoltage) return 100;

  return (int)(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100);
}
