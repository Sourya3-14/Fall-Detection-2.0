🛡️ Fall Detection System
A smart IoT-based fall detection system using the MPU6050 sensor and ESP32 to detect and report falls in elderly individuals. It tracks angular changes (roll, pitch), uses a threshold-based classifier, and sends alerts via GSM/Bluetooth.

📦 Features
Real-time angle and accelaration tracking using MPU6050
Fall detection based on decision tree based trained model

Alerts through:
Buzzer (on-device warning)
SMS via GSM
Bluetooth to Android app
Battery-powered and portable
Simple and reliable threshold logic

🔧 Hardware Used
✅ ESP32
✅ MPU6050 (Accelerometer + Gyroscope)
✅ Buzzer
✅ GSM Module (SIM800L)
✅ Power supply (Li-ion battery)
✅ OLED display 
✅ NEO 6m GPS module

📐 Fall Detection Logic
Continuously monitor roll, pitch angles,acceleration change in any axis,BPM,HRV,SpO2 and used the trained decision tree to detct fall
If any axis changes by more than 70° within a short window, flag as potential fall.
Trigger alert mechanism through GSM sms and bluetooth app also sends location of the patient where fall occured in the sms


