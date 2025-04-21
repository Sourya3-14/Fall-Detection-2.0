#include <WiFi.h>
#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include <addons/TokenHelper.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

#include <Wire.h>
#include <Adafruit_SH110X.h>

#define OLED_RESET -1
Adafruit_SH1106G display = Adafruit_SH1106G(128,64, &Wire, OLED_RESET); /* Object of class Adafruit_SSD1306 */

// #define FIREBASE_DEBUG

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Solo"
#define WIFI_PASSWORD "Password"
#define NUMBER "+917439928541"

#define API_KEY "AIzaSyCGSUHLgBqeEmIpSTd3ttLtRpsrCyOMpyk"
#define DATABASE_URL "https://lesson05-esp32-rtdb-1e8bf-default-rtdb.firebaseio.com/" 

FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOk = false;

HardwareSerial mySerial(1); // Use UART1
unsigned long recieveDataPrevMillis = 0;
HardwareSerial sim800(2);   // URAT2
String lastData = "";
String arr[3] = {"Safe","Fall Prob Detected","Fall Detected"};
String content[3] = {"0","0","0"} ;
String prevPred = "";

void setup()
{  
  Serial.begin(9600);
  mySerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins
  sim800.begin(9600, SERIAL_8N1, 26, 25);  // RX, TX for GSM
  delay(1000);
  Serial.println("Initializing SIM800L...");
  sendCommand("AT");                // Check if module is responding
  sendCommand("AT+CMGF=1");         // Set SMS mode to Text
  sendCommand("AT+CSCS=\"GSM\"");   // Set character set to GSM
  
  pinMode(12,INPUT_PULLUP);

  display.begin(0x3C); /* Initialize display with address 0x3C */
  display.clearDisplay(); /* Clear display */
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1); /* Select font size of text. Increases with size of argument. */ 

  text_display("Booting Up the device");
  delay(200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  String temp = "Connecting to Wi-Fi";
  Serial.print(temp);
  text_display(temp);
  unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    temp = temp+".";
    text_display(temp);
    delay(500);
    if (millis() - ms > 10000){
      text_display("Cant Connect to WiFi");
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      delay(100);
      break;
    }
  }
  Serial.println();
  text_display("Connected");
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if(Firebase.signUp(&config,&auth,"","")){
    Serial.println("Signup OK");
    text_display("Firebase Signup Done");
    signupOk=true;
    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
    Firebase.begin(&config, &auth);
    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectWiFi(true);
  }
  else{
    Serial.println(config.signer.signupError.message.c_str());
    text_display("Firebase Signup error\nplease reset the \ndevice or press the \nbutton to use without \ninternet");
    while(1){
      if(!digitalRead(12)){
        text_display("Alerts will only be sent through SMS");
        delay(1000);
        break;
      }
    }
  }
  pinMode(12,INPUT_PULLUP);
  text_display("Starting the device");
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  delay(1000);
}

void loop()
{
  if(!digitalRead(12)){
    text_display("Called for help Stay strong Help will arrive \nsoon");
    SOS(NUMBER,"Urgent need for help");
    delay(100);
    // delay(5000);
  }
  if (mySerial.available() > 0)
    readMessage();
  
  // Serial.println("BPM: " + content[0]);
  // Serial.println("SpO2: " + content[1]);
  // Serial.println("Decision: " + content[2]);

  if(content[2]=="2" ){
    digitalWrite(14,HIGH);
    OLED_display(content[0],content[1],"Fall detected \ncallingfor help");
    if(prevPred == "2" && millis() - recieveDataPrevMillis > 30000 || recieveDataPrevMillis==0){
      // if (Firebase.ready() && signupOk)
      //   FireBaseWrite();
      recieveDataPrevMillis = millis();
      SOS(NUMBER,"Patient Fell Down");
    }
    else if(prevPred != "2"){
      SOS(NUMBER,"Patient Fell Down");
      if (signupOk && WiFi.status() == WL_CONNECTED && Firebase.ready())
        FireBaseWrite();
    }
    digitalWrite(14,LOW);
  }
  else{
    OLED_display(content[0],content[1],arr[content[2].toInt()]);
    if (signupOk && WiFi.status() == WL_CONNECTED && Firebase.ready()) 
        FireBaseWrite();
    if(content[2]=="1"){
      unsigned long startBlink = millis();
      while (millis() - startBlink < 5000) {
        digitalWrite(14, HIGH);
        delay(100);
        digitalWrite(14, LOW);
        delay(100);
      }
    }
  }
}
void sendCommand(const String& cmd) {
  sim800.println(cmd);
  delay(500);

  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}
void SOS(const char *number, const char *message){
  sendCommand("AT");                // Check if module is responding
  delay(300);
  sendCommand("AT+CMGF=1");         // Set SMS mode to Text
  delay(300);
  // sendCommand("AT+CSCS=\"GSM\"");   // Set character set to GSM
  // delay(300);
  sendSMS(number,message);

}
void sendSMS(const char *number, const char *message) {
  sim800.print("AT+CMGS=\"");
  sim800.print(number);
  sim800.println("\"");
  delay(1000);

  sim800.print(message);
  sim800.write(26); // Ctrl+Z to send SMS
  delay(1000);      // Wait for SMS to be sent

  // Serial.println("SMS Sent!");
}
void readMessage(){
  String readString = mySerial.readStringUntil('\n');
  char delimiters[] = {',', ';', '|'};
  int prevIndex = 0;
  prevPred = content[2];
  for (int i = 0; i < 3; i++) {
    int delimIndex = readString.indexOf(delimiters[i], prevIndex);
    if (delimIndex == -1) delimIndex = readString.length();
    content[i] = readString.substring(prevIndex, delimIndex);
    prevIndex = delimIndex + 1;
  }
}
void OLED_display(String bpm,String spo2,String state){
  display.clearDisplay();
  display.setCursor(5,5);
  display.println(("Patient Status:"));
  display.setCursor(5,15);
  display.println(("BPM: " + bpm));
  display.setCursor(5,25);
  display.println(("SpO2: " + spo2 + "%"));
  display.setCursor(5,35);
  display.println((state));
  display.display(); // without this display will not be updated
}
void text_display(String text){
  display.clearDisplay();
  display.setCursor(0,5);
  display.println((text));
  display.display(); // without this display will not be updated
}
void FireBaseWrite(){
  int decisionIndex = content[2].toInt();
  if (decisionIndex < 0 || decisionIndex > 2) {
    decisionIndex = 0; // default to Safe
  }
  content[0]=String(constrain(content[0].toFloat(),0.0,120.0));
  content[1]=String(constrain(content[1].toFloat(),0.0,100.0));

  String readString = content[0]+","+content[1]+";"+arr[decisionIndex]+"|";
  if(lastData!=readString){
    if (Firebase.RTDB.setString(&fbdo, "Sensor/data", readString)){
      Serial.print(arr[decisionIndex]);
      Serial.println(" successfully saved to:" +fbdo.dataPath()+" ("+fbdo.dataType() + ")");
      lastData = readString;
    }
    else{
      Serial.println("FAILED:"+ fbdo.errorReason());
    }
  }
}
