#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MPU6050.h>
#include "BluetoothSerial.h"
#include <Preferences.h>
#include <Wire.h>

// SoftWire myWire(4, 5); // SDA, SCL

BluetoothSerial SerialBT;
Preferences prefs;

String incomingData = "";
String phoneNumber = "";
String wifiID = "";
String wifiPassword = "";

Adafruit_MPU6050 mpu;
const float threshold = 2.5;  // rad/s
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
int intervalindex = 0;
void onBeatDetected() {}

int pred = 0;
int count = 0;
HardwareSerial mySerial(2);
unsigned long lastSendTime = 0;

void setup() {
  while(!Wire.begin(21,22))delay(1000); // SDA=21, SCL=22
  while(!Wire1.begin(4,5))delay(1000); // SDA=21, SCL=22
  // myWire.begin(); // Start the software I2C
  // delay(1000);
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");  // This is the name Android will search for
  Serial.println("ESP32 Bluetooth started. Waiting for data...");

  recieveBluetoothDataFirstTime();

  // mySerial.begin(9600, SERIAL_8N1, 26, 25); // RX, TX pins

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while(1)delay(10);
  }
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  roll1 = atan2(a.acceleration.y, a.acceleration.z) * 180 / 3.14159;
  pitch1 = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / 3.14159;

  //buzzer beeps after the needed initilalization in done
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(500);
  digitalWrite(12, LOW);

  if (!pox.begin()) {
    while (1);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  static unsigned long prevPredTime1 = 0;
  static unsigned long prevPredTime2 = 0;

  acc = suddenAccelaration();
  angle = calculateAngle();
  if(pred==1 && millis()-prevPredTime1 < 5000 && decision2(hrv, spo2, acc, angle)!=2 )
    pred=pred;
  // else{
  //   pred = decision2(hrv, spo2, acc, angle);
  //   prevPredTime1 = millis();
  //   if(pred==2 && spo2==0 || bpm==0)
  //     pred=0;
  // }
  else if(pred==2 && (millis()-prevPredTime2 < 7000 || angle == 1) )
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
  pox.update();
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    spo2 = constrain(pox.getSpO2(),0,100);
    bpm = constrain(pox.getHeartRate(),0,120);
    intervals[intervalindex] = (bpm == 0) ? 0 : 60000 / bpm;
    intervalindex = int((intervalindex + 1) % 5);

    if (intervalindex == 0) {
      hrv = constrain(calculateHRV(),0,250);
    }
    tsLastReport = millis();
  }

  
  // if(pred == 2){
  //     prevPredTime = millis();
  //     // count=1;
      // sendSensorData(bpm,spo2,pred);
  // }
  // if(millis()-prevPredTime > 5000 ){
    String msg = String(bpm) + "," + String(spo2) + "," + String(pred) + "," + phoneNumber;
    // mySerial.println(msg);
    Wire1.beginTransmission(8);
    Wire1.write((const uint8_t*)msg.c_str(), strlen(msg.c_str()));
    Wire1.endTransmission();  
    Serial.println("Sent to Slave:" + msg );

  if (millis() - lastSendTime > 100) {
    sendSensorData(bpm,spo2,pred);
    Serial.println("Phone: " + phoneNumber);
    Serial.println("WiFi ID: " + wifiID);
    Serial.println("WiFi Pass: " + wifiPassword);
    lastSendTime = millis();
  }
    // count=0;
  // }
  
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
  if (spo2 <= -0.323044 ) 
    return 2;
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
  // Read acceleration values from the sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert gyroscope values from degrees per second (°/s) to radians per second (rad/s)
  float gyroX = g.gyro.x;  // X axis (rad/s)
  float gyroY = g.gyro.y;  // Y axis (rad/s)
  float gyroZ = g.gyro.z;  // Z axis (rad/s)

  // Check if acceleration exceeds the threshold on any axis
  if (abs(gyroX) > threshold || abs(gyroY) > threshold || abs(gyroZ) > threshold) {
    return 1.0;
  } else {
    return 0.0;
  }
}
float calculateAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate roll and pitch from accelerometer data
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Calculate pitch and roll angles in degrees
  float roll = atan2(accelY, accelZ) * 180 / 3.14159;
  float pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 180 / 3.14159;

  if (abs(pitch - pitch1) > 60 || abs(roll - roll1) > 60)
    return 1.0;
  else
    return 0.0;
}