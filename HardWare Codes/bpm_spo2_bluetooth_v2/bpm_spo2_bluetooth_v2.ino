#include "MAX30100_PulseOximeter.h"
#include <MPU6050_light.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <Wire.h>


BluetoothSerial SerialBT;
Preferences prefs;

String incomingData = "";
String phoneNumber = "";
String wifiID = "";
String wifiPassword = "";

MPU6050 mpu(Wire);
const float threshold1 = 4.5;  // rad/s
// const float threshold2 = 8;  // m/s
float acc = 0;
float angle = 0;
float pitch1;
float roll1;


#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;
uint32_t tsLastReport = 0;
float hrv = 0.0;
float gap = 0.0;
float intervals[5] = {0};
float bpm=0.0,spo2=0.0;
float lower[3]={56,91,21};
float upper[3]={178,100,120};
int intervalindex = 0;
void onBeatDetected() {}

int pred = 0;
int count = 0;
HardwareSerial mySerial(2);
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Wire.begin())delay(1000); // SDA=21, SCL=22
  while(!Wire1.begin(4,5))delay(1000); // SDA=4, SCL=5

  SerialBT.begin("ESP32_BT");  // This is the name Android will search for
  Serial.println("ESP32 Bluetooth started. Waiting for data...");

  recieveBluetoothDataFirstTime();

  // if (!pox.begin()) {
  //   Serial.println("Failed to find MAX30100 chip");
  //   while (1);
  // }

  // Try to initialize!
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  //setupt motion detection
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  //buzzer beeps after the needed initilalization in done
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(500);
  digitalWrite(12, LOW);

  if (!pox.begin()) {
    Serial.println("Failed to find MAX30100 chip");
    while (1);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  static unsigned long prevPredTime1 = 0;
  static unsigned long prevPredTime2 = 0;
  mpu.update();
  acc = suddenAccelaration();
  angle = calculateAngle();
  if(pred==1 && millis()-prevPredTime1 < 500 && decision2(hrv, spo2, acc, angle)!=2 )
    pred=pred;
  else if(pred==2 && (millis()-prevPredTime2 < 300 || angle == 1) )
    pred=pred;
  else{
    pred = decision2(hrv, spo2, acc, angle);
    if((pred==2 ||pred==1) && spo2==0 || bpm==0)
      pred=0;
    if(pred==2)
      prevPredTime2 = millis();
    else if(pred==1)
      prevPredTime1 = millis();
    
  }
  // Check if data is available from the phone
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      // Complete message received
      processIncomingData(incomingData);
      incomingData = "";
    } else {
      incomingData += incomingChar;
    }
  }
  float tempSpo2=0;
  float tempBpm=0;
  pox.update();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    tempSpo2 = pox.getSpO2();
    tempBpm = pox.getHeartRate();
    if(tempSpo2 == 0 && tempBpm ==0){
      spo2 = tempSpo2;
      bpm = tempBpm;
    }
    else{
      spo2 = constrain(tempSpo2,lower[1],upper[1]);
      bpm = constrain(tempBpm,lower[0],upper[0]);
    }
    intervals[intervalindex] = (bpm == 0) ? 0 : 60000 / bpm;
    intervalindex = int((intervalindex + 1) % 5);

    if (intervalindex == 0) {
      hrv = constrain(calculateHRV(),lower[2],upper[2]);
    }
    tsLastReport = millis();
  }

  String msg = String(int(bpm)) + "," + String(int(spo2)) + "," + String(pred) + "," + phoneNumber + "," + "1";
  Wire1.beginTransmission(8);
  Wire1.write((const uint8_t*)msg.c_str(), strlen(msg.c_str()));
  Wire1.endTransmission();  
  Serial.println("Sent to Slave:" + msg );

  if (millis() - lastSendTime > 100) {
    sendSensorData(int(bpm),int(spo2),pred);
    Serial.println("Phone: " + phoneNumber);
    Serial.println("WiFi ID: " + wifiID);
    Serial.println("WiFi Pass: " + wifiPassword);
    lastSendTime = millis();
  }
}
//Bluetooth functions
void recieveBluetoothData() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      // Complete message received
      processIncomingData(incomingData);

      prefs.begin("fall-data", false);  // Open Preferences
      prefs.clear();                    // Clear it completely
      prefs.end();  

      prefs.begin("fall-data", false);  // Open Preferences in read/write mode
      prefs.putString("wifi_ssid", wifiID);
      prefs.putString("wifi_pass", wifiPassword);
      prefs.putString("phone_num", phoneNumber);
      prefs.putBool("initialized", true);
      prefs.end();
      incomingData = "";
    } else {
      incomingData += incomingChar;
    }
  }
}
void recieveBluetoothDataFirstTime() {
  prefs.begin("fall-data", false);  // Open Preferences in read/write mode
  if (!prefs.isKey("initialized")) {
    while (true){
      if (SerialBT.available()) {
        char incomingChar = SerialBT.read();
        if (incomingChar == '\n') {
          // Complete message received
          processIncomingData(incomingData);
          // prefs.begin("fall-data", false);  // Open Preferences in read/write mode
          prefs.clear();
          prefs.putString("wifi_ssid", wifiID);
          prefs.putString("wifi_pass", wifiPassword);
          prefs.putString("phone_num", phoneNumber);
          prefs.putBool("initialized", true);
          prefs.end();
          incomingData = "";
          break;
        }    
        else {
          incomingData += incomingChar;
        }
      }
    }
  }
  else {
    prefs.end();
    readPreferenceData();
  }
}
void readPreferenceData(){
  prefs.begin("fall-data", true);  // Open Preferences in read/write mode
  wifiID = prefs.getString("wifi_ssid", "");
  wifiPassword = prefs.getString("wifi_pass", "");
  phoneNumber = prefs.getString("phone_num", "");
  prefs.end();
}
// Parse phone number, WiFi ID, and password
void processIncomingData(String data) {
  Serial.println("Received from phone: " + data);
  
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > firstComma) {
    phoneNumber = data.substring(0, firstComma);
    wifiID = data.substring(firstComma + 1, secondComma);
    wifiPassword = data.substring(secondComma + 1);

    Serial.println("Phone: " + phoneNumber);
    Serial.println("WiFi ID: " + wifiID);
    Serial.println("WiFi Pass: " + wifiPassword);
  } else {
    Serial.println("Invalid data format.");
  }
}

// Simulate BPM, SpO2 and decision data sending
void sendSensorData(int bpm,int spo2,int dec) {
  String message = String(bpm) + "," + String(spo2) + "," + String(dec);
  SerialBT.print(message);
  Serial.println("Sent to phone: " + message);
}

//Helper ones
int decision2(float hrv, float spo2, float acc,float angle){
  hrv = (hrv - 95.657002) / 17.576499;
  spo2 = (spo2 - 83.563649) / 11.111592;
  acc = (acc - 0.661599) / 0.473282;
  if (spo2 <= 0.579246) 
    return 1;
  else{
    if (acc <= -0.341527)
      return 0;
    else if(angle == 1)
      return 2;
    else 
      return 1;
  }
}
void printData(){
  Serial.print(bpm);
  Serial.print(",");
  Serial.print(spo2);
  Serial.print(",");
  Serial.print(hrv);
  Serial.print(",");
  Serial.print(acc);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.println(pred);

}
float calculateHRV() {
  float mean = 0;
  float variance = 0;

  // Calculate mean
  for (int i = 0; i < 5; i++) {
    mean += intervals[i];
  }
  mean /= 5;

  // Calculate variance
  for (int i = 0; i < 5; i++) {
    float diff = intervals[i] - mean;
    variance += diff * diff;
  }
  variance /= 5;

  // HRV is the square root of variance
  return sqrt(variance);
}
float suddenAccelaration() {
  float gyroX = mpu.getGyroX()* PI / 180.0;  // X axis (rad/s)
  float gyroY = mpu.getGyroY()* PI / 180.0;  // Y axis (rad/s)
  float gyroZ = mpu.getGyroZ()* PI / 180.0;  // Z axis (rad/s)

  // Check if acceleration exceeds the threshold on any axis
  if (abs(gyroX) > threshold1 || abs(gyroY) > threshold1 || abs(gyroZ) > threshold1) {
    return 1.0;
  } 
  else {
    return 0.0;
  }
}
float calculateAngle() {
  if (abs(mpu.getAngleX()) > 60 || abs(mpu.getAngleY()) > 60 || abs(mpu.getAngleZ()) > 60)
    return 1.0;
  else
    return 0.0;
}