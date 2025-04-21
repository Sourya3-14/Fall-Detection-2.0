// #include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MPU6050.h>
#include<SoftwareSerial.h>

SoftwareSerial mySerial(6,5);//Rx Tx

Adafruit_MPU6050 mpu;
const float threshold = 3;  // rad/s
float acc = 0;
float angle = 0;
float pitch1;
float roll1;


#define REPORTING_PERIOD_MS 500
PulseOximeter pox;
uint32_t tsLastReport = 0;
float hrv = 0.0;
float gap = 0.0;
float intervals[5] = {0};
float bpm=0.0,spo2=0.0;
int intervalindex = 0;
void onBeatDetected() {}

int pred = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

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
  pox.update();
  float p1=bpm,p2=spo2,p3=pred;
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    spo2 = constrain(pox.getSpO2(),0,100);
    bpm = constrain(pox.getHeartRate(),0,120);
    intervals[intervalindex] = (bpm == 0) ? 0 : 60000 / bpm;
    intervalindex = int((intervalindex + 1) % 5);

    if (intervalindex == 0) {
      hrv = constrain(calculateHRV(),0,250);
    }
    tsLastReport = millis();
    if(p1 != bpm || p2 != spo2 || p3 != pred)
      send_data();

  }

  // float bpm = movingAverage(bpms);
  Serial.print(bpm);
  Serial.print(",");

  // float spo2 = movingAverage(spo2s);
  Serial.print(spo2);
  Serial.print(",");

  Serial.print(hrv);
  Serial.print(",");

  acc = suddenAccelaration();
  Serial.print(acc);
  Serial.print(",");

  angle = calculateAngle();
  if (angle == 1.0 && pred == 2){
    Serial.println(pred);
  } 
  else {
    pred = decision2(hrv, spo2, acc, angle);
    Serial.println(pred);
  }

}
void send_data(){
  mySerial.print(bpm);
  mySerial.print(",");
  mySerial.print(spo2);
  mySerial.print(";");
  if(pred==2 && acc==0 && angle !=1 )
    mySerial.print("0");
  else
    mySerial.print(pred);
  mySerial.println("|");
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

  if (abs(pitch - pitch1) > 70 || abs(roll - roll1) > 70)
    return 1.0;
  else
    return 0.0;
}