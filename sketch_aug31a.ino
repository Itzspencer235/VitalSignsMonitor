#include <WiFi.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <TimeLib.h>

#define REPORTING_PERIOD_MS 1000
#define RESET_BUTTON 23
#define WIFI_STATUS_LED 2
#define MUTE_BUTTON 39
#define EMERGENCY_BUTTON 15 // New button on pin 15
#define PHONE_INPUT_TIMEOUT_MS 10000
#define SERIAL_BUFFER_MAX 50
#define MQTT_MAX_PACKET_SIZE 1024
#define BUTTON_DEBOUNCE_MS 50 // Debounce time for button
bool buzzerMuted = false;
bool previousAlarmState = false;
bool lastButtonState = HIGH; // For button debouncing (HIGH = not pressed)
unsigned long lastButtonPress = 0;
const char* aws_endpoint = "a1t4z1928sw433-ats.iot.ap-southeast-2.amazonaws.com";
const char* thing_name = "ESP32Device";
const char* certificatePemCrt = "-----BEGIN CERTIFICATE-----\n"

                                "-----END CERTIFICATE-----\n";
    const char* privateKey = "-----BEGIN RSA PRIVATE KEY-----\n"
                         ;
     const char* rootCA = "-----BEGIN CERTIFICATE-----\n"
                    
                     "-----END CERTIFICATE-----\n";

String phoneNumber = "";
const int MAX_PHONE_LENGTH = 13;
const int MIN_PHONE_LENGTH = 9;
WiFiClientSecure espClient;
PubSubClient client(espClient);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
Preferences preferences;
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
#define ECG_PIN 34
#define LO_PLUS 35
#define LO_MINUS 32
#define BUZZER_PIN 26
#define LED_PIN 26
#define RXD1 18
#define TXD1 19
#define RXD2 16
#define TXD2 17
#define TEMP_MIN 20.0
#define TEMP_MAX 37.0
#define HUMIDITY_MIN 30.0
#define HUMIDITY_MAX 65
#define ECG_BUFFER_SIZE 50
float ecgBuffer[ECG_BUFFER_SIZE];
uint8_t ecgBufferIndex = 0;
const uint32_t ECG_SAMPLE_INTERVAL_MS = 20;
uint32_t lastEcgSampleTime = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

String serialBuffer = "";
float bpm = 0;
float spo2 = 0;
bool smsSent = false;
bool buzzerActive = false;
bool wifiConfigured = false;
bool phoneInputActive = false;
uint32_t lastPhoneInputTime = 0;

void resetWiFi() {
  WiFiManager wm;
  wm.resetSettings();
  for (int i = 0; i < 5; i++) {
    digitalWrite(WIFI_STATUS_LED, HIGH);
    delay(300);
    digitalWrite(WIFI_STATUS_LED, LOW);
    delay(300);
  }
  ESP.restart();
}

void loadPhoneNumber() {
  preferences.begin("settings", false);
  phoneNumber = preferences.getString("phoneNumber", "");
  preferences.end();
  if (phoneNumber != "" && phoneNumber.length() >= MIN_PHONE_LENGTH) {
    Serial.println("Loaded phone number: " + phoneNumber);
  } else {
    phoneNumber = "";
    Serial.println("No valid phone number loaded");
  }
}

void savePhoneNumber() {
  preferences.begin("settings", false);
  preferences.putString("phoneNumber", phoneNumber);
  preferences.end();
  Serial.println("Saved phone number: " + phoneNumber);
}

String sendCommand(String command, unsigned long timeout = 2000) {
  if (command != "") {
    Serial2.println(command);
  }
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (Serial2.available()) {
      char c = Serial2.read();
      response += c;
    }
  }
  if (command != "") {
    Serial.print("Command: ");
    Serial.print(command);
    Serial.print(", Response: ");
    Serial.println(response);
  }
  return response;
}

bool sendSMS(String message) {
  if (phoneNumber.length() < MIN_PHONE_LENGTH) {
    Serial.println("No valid phone number set (minimum " + String(MIN_PHONE_LENGTH) + " characters)");
    return false;
  }

  // Optionally prepend +250 if needed (uncomment if SIM900 requires it)
  String smsNumber = phoneNumber;
  // if (!smsNumber.startsWith("+")) {
  //   smsNumber = "+250" + smsNumber;
  //   Serial.println("Prepended +250 for SMS: " + smsNumber);
  // }

  // Set SMS text mode
  String response = sendCommand("AT+CMGF=1", 2000);
  if (response.indexOf("OK") < 0) {
    Serial.println("Failed to set SMS text mode");
    return false;
  }

  // Send SMS command
  response = sendCommand("AT+CMGS=\"" + smsNumber + "\"\r", 3000);
  if (response.indexOf(">") < 0) {
    Serial.println("Failed to get SMS prompt: " + response);
    return false;
  }

  // Send message content
  Serial2.print(message);
  Serial2.write(26); // Ctrl+Z
  Serial.println("Sending SMS: " + message);
  response = sendCommand("", 5000); // Wait for response
  if (response.indexOf("OK") >= 0) {
    Serial.println("SMS sent successfully");
    return true;
  } else {
    Serial.println("SMS send failed: " + response);
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");
  Wire.begin();
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "Initializing...");
  u8g2.sendBuffer();
  dht.begin();
  ds18b20.begin();
  pinMode(ECG_PIN, INPUT);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  pinMode(MUTE_BUTTON, INPUT_PULLUP);
  pinMode(EMERGENCY_BUTTON, INPUT_PULLUP); // Configure emergency button
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(WIFI_STATUS_LED, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(WIFI_STATUS_LED, LOW);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Initialize GSM module
  String response = sendCommand("AT", 2000);
  if (response.indexOf("OK") >= 0) {
    Serial.println("SIM900 responded OK");
  } else {
    Serial.println("SIM900 not responding");
  }
  response = sendCommand("AT+CSQ", 2000);
  if (response.indexOf("+CSQ:") >= 0) {
    Serial.println("Signal quality: " + response);
  }
  response = sendCommand("AT+CREG?", 2000);
  if (response.indexOf("+CREG: 0,1") >= 0 || response.indexOf("+CREG: 0,5") >= 0) {
    Serial.println("SIM900 registered on network");
  } else {
    Serial.println("SIM900 not registered on network");
  }
  sendCommand("AT+CMGF=1", 2000); // Set SMS text mode

  for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
    ecgBuffer[i] = 0.0;
  }
  loadPhoneNumber();
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  setupNetwork();
  timeClient.begin();
}

void setupNetwork() {
  WiFiManager wm;
  wm.setTimeout(30);
  if (wm.autoConnect("ESP32_Config")) {
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(WIFI_STATUS_LED, HIGH);
    wifiConfigured = true;
    timeClient.update();
    Serial.print("Current time: ");
    Serial.println(timeClient.getFormattedTime());
    espClient.setCACert(rootCA);
    espClient.setCertificate(certificatePemCrt);
    espClient.setPrivateKey(privateKey);
    client.setServer(aws_endpoint, 8883);
    connectAWS();
  } else {
    Serial.println("WiFi not configured");
    digitalWrite(WIFI_STATUS_LED, LOW);
    wifiConfigured = false;
  }
}

void connectAWS() {
  if (!client.connected() && wifiConfigured) {
    Serial.print("Connecting to AWS IoT Core...");
    if (client.connect(thing_name)) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      Serial.print("WiFi status: ");
      Serial.println(WiFi.status());
    }
  }
}

void updateDisplay(float dhtTemp, float humidity, float ds18b20Temp, float ecgValue, float bpm, float spo2) {
  if (phoneInputActive) {
    return;
  }
  u8g2.clearBuffer();
  String statusStr = wifiConfigured ? 
                    (client.connected() ? "AWS OK" : "AWS Disconn") : 
                    "No WiFi";
  String envStr = "T:" + String(dhtTemp, 1) + "C H:" + String(humidity, 1) + "%";
  String vitalStr = "BPM:" + String(bpm, 0) + " SpO2:" + String(spo2, 0) + "%";
  String ds18Str = "DS18:" + String(ds18b20Temp, 1) + "C";
  String ecgStr = "ECG:" + String(ecgValue, 2) + "V";
  u8g2.drawStr(0, 10, statusStr.c_str());
  u8g2.drawStr(0, 20, envStr.c_str());
  u8g2.drawStr(0, 30, vitalStr.c_str());
  u8g2.drawStr(0, 40, ds18Str.c_str());
  u8g2.drawStr(0, 50, ecgStr.c_str());
  u8g2.sendBuffer();
}

void publishToAWS(float dhtTemp, float humidity, float ds18b20Temp, float ecgValue, float bpm, float spo2, bool alarmCondition) {
  if (wifiConfigured && WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected, checking AWS connection...");
    if (!client.connected()) {
      connectAWS();
    }
    if (client.connected()) {
      Serial.println("AWS client connected, preparing payload...");
      client.loop();
      delay(50);
      StaticJsonDocument<512> doc;
      timeClient.update();
      unsigned long epochTime = timeClient.getEpochTime();
      char isoTime[25];
      snprintf(isoTime, sizeof(isoTime), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               year(epochTime), month(epochTime), day(epochTime),
               hour(epochTime), minute(epochTime), second(epochTime));
      doc["timestamp"] = isoTime;
      doc["dht_temperature"] = dhtTemp;
      doc["humidity"] = humidity;
      doc["ds18b20_temperature"] = ds18b20Temp;
      doc["ecg_voltage"] = ecgValue;
      doc["bpm"] = bpm;
      doc["spo2"] = spo2;
      JsonArray ecgData = doc.createNestedArray("ecg_buffer");
      for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
        int index = (ecgBufferIndex + i) % ECG_BUFFER_SIZE;
        ecgData.add(ecgBuffer[index]);
      }
      doc["alarm"] = alarmCondition;
      char payload[512];
      size_t payloadSize = serializeJson(doc, payload);
      Serial.print("Payload size: ");
      Serial.println(payloadSize);
      Serial.print("Payload: ");
      Serial.println(payload);
      Serial.print("Free heap: ");
      Serial.println(ESP.getFreeHeap());
      Serial.print("Publishing to topic 'vital_signs'...");
      if (client.publish("vital_signs", payload)) {
        Serial.println("Publish successful");
      } else {
        Serial.print("Publish failed, rc=");
        Serial.println(client.state());
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
      }
    } else {
      Serial.println("AWS client not connected");
    }
  } else {
    Serial.println("WiFi not connected");
  }
}

void loop() {
  client.loop();
  
  // Check emergency button (pin 15)
  bool currentButtonState = digitalRead(EMERGENCY_BUTTON);
  if (currentButtonState == LOW && lastButtonState == HIGH && (millis() - lastButtonPress > BUTTON_DEBOUNCE_MS)) {
    Serial.println("Emergency button pressed");
    // Read sensor data for SMS
    float humidity = dht.readHumidity();
    float dhtTemp = dht.readTemperature();
    ds18b20.requestTemperatures();
    float ds18b20Temp = ds18b20.getTempCByIndex(0);
    float ecgValue = (digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH) ? 
                     -1 : analogRead(ECG_PIN) * (3.3 / 4095.0);
    // Format SMS with sensor data (no alarm reason)
    String message = "T:" + String(dhtTemp != -1 ? dhtTemp : 0.0, 1) + "C,H:" + 
                     String(humidity != -1 ? humidity : 0.0, 1) + "%,DS18:" + 
                     String(ds18b20Temp != -1 ? ds18b20Temp : 0.0, 1) + "C,BPM:" + 
                     String(bpm, 0) + ",SpO2:" + String(spo2, 0) + "%,ECG:" + 
                     String(ecgValue != -1 ? ecgValue : 0.0, 2) + "V";
    if (sendSMS(message)) {
      Serial.println("Emergency SMS sent");
    }
    lastButtonPress = millis();
  }
  lastButtonState = currentButtonState;

  // Existing mute button logic
  if (digitalRead(MUTE_BUTTON) == LOW) {
    buzzerMuted = true;
    Serial.println("Buzzer muted by user.");
    delay(500);
  }

  // Serial1 data from Arduino
  while (Serial1.available()) {
    char c = Serial1.read();
    serialBuffer += c;
    if (serialBuffer.length() > SERIAL_BUFFER_MAX) {
      serialBuffer = "";
      Serial.println("Serial buffer overflow, cleared");
    }
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.startsWith("BPM:")) {
        int commaIndex = serialBuffer.indexOf(",");
        String bpmStr = serialBuffer.substring(4, commaIndex);
        String spo2Str = serialBuffer.substring(commaIndex + 6);
        bpmStr.trim();
        spo2Str.trim();
        bpm = bpmStr.toFloat();
        spo2 = spo2Str.toFloat();
        Serial.print("Received - BPM: ");
        Serial.print(bpm);
        Serial.print(", SpO2: ");
        Serial.println(spo2);
        phoneInputActive = false;
      }
      else if (serialBuffer.startsWith("PHONE:")) {
        String newPhoneNumber = serialBuffer.substring(6);
        newPhoneNumber.trim();
        if (newPhoneNumber.length() >= MIN_PHONE_LENGTH) {
          phoneNumber = newPhoneNumber;
          savePhoneNumber();
          u8g2.clearBuffer();
          u8g2.drawStr(0, 10, "Number Saved:");
          u8g2.drawStr(0, 20, phoneNumber.c_str());
          u8g2.sendBuffer();
          delay(2000);
          phoneInputActive = false;
        }
      }
      else if (serialBuffer.startsWith("PHONE_UPDATE:")) {
        String tempPhoneNumber = serialBuffer.substring(13);
        tempPhoneNumber.trim();
        u8g2.clearBuffer();
        u8g2.drawStr(0, 10, "Enter Phone #:");
        u8g2.drawStr(0, 20, tempPhoneNumber.length() > 0 ? tempPhoneNumber.c_str() : "");
        u8g2.sendBuffer();
        phoneInputActive = true;
        lastPhoneInputTime = millis();
      }
      else if (serialBuffer.startsWith("PHONE_ERROR:")) {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 10, "Invalid Number");
        u8g2.drawStr(0, 20, "Need >= 9 chars");
        u8g2.drawStr(0, 30, "Enter digits, then #");
        u8g2.sendBuffer();
        delay(2000);
        u8g2.clearBuffer();
        u8g2.drawStr(0, 10, "Enter Phone #:");
        u8g2.drawStr(0, 20, "");
        u8g2.sendBuffer();
        phoneInputActive = true;
        lastPhoneInputTime = millis();
      }
      serialBuffer = "";
    }
  }

  if (phoneInputActive && (millis() - lastPhoneInputTime > PHONE_INPUT_TIMEOUT_MS)) {
    phoneInputActive = false;
  }

  if (digitalRead(RESET_BUTTON) == LOW) {
    resetWiFi();
  }

  if (millis() - lastEcgSampleTime >= ECG_SAMPLE_INTERVAL_MS) {
    bool loPlus = digitalRead(LO_PLUS);
    bool loMinus = digitalRead(LO_MINUS);
    float ecgValue = (loPlus || loMinus) ? -1 : analogRead(ECG_PIN) * (3.3 / 4095.0);
    ecgBuffer[ecgBufferIndex] = ecgValue;
    ecgBufferIndex = (ecgBufferIndex + 1) % ECG_BUFFER_SIZE;
    lastEcgSampleTime = millis();
    Serial.print("ECG sample: LO+=");
    Serial.print(loPlus);
    Serial.print(", LO-=");
    Serial.print(loMinus);
    Serial.print(", Value=");
    Serial.println(ecgValue);
  }

  // Alarm-based SMS logic
  float humidity = dht.readHumidity();
  float dhtTemp = dht.readTemperature();
  ds18b20.requestTemperatures();
  float ds18b20Temp = ds18b20.getTempCByIndex(0);
  float ecgValue = (digitalRead(LO_PLUS) == HIGH || digitalRead(LO_MINUS) == HIGH) ? 
                   -1 : analogRead(ECG_PIN) * (3.3 / 4095.0);
  bool alarmCondition = false;
  String alarmReason = "";
  if (dhtTemp != -1 && (dhtTemp < TEMP_MIN || dhtTemp > TEMP_MAX)) {
    alarmCondition = true;
    alarmReason += "Temp out of range ";
  }
  if (ds18b20Temp != -1 && (ds18b20Temp < TEMP_MIN || ds18b20Temp > TEMP_MAX)) {
    alarmCondition = true;
    alarmReason += "DS18 out of range ";
  }
  if (humidity != -1 && (humidity < HUMIDITY_MIN || humidity > HUMIDITY_MAX)) {
    alarmCondition = true;
    alarmReason += "Humidity out of range ";
  }
  if (alarmCondition) {
    Serial.println("Alarm condition detected: " + alarmReason);
    if (!previousAlarmState) {
      buzzerMuted = false;
      previousAlarmState = true;
      smsSent = false; // Reset smsSent on new alarm
    }
    buzzerActive = true;
    if (!buzzerMuted) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    digitalWrite(LED_PIN, HIGH);
    if (!smsSent) {
      String message = "T:" + String(dhtTemp != -1 ? dhtTemp : 0.0, 1) + "C,H:" + 
                       String(humidity != -1 ? humidity : 0.0, 1) + "%,DS18:" + 
                       String(ds18b20Temp != -1 ? ds18b20Temp : 0.0, 1) + "C,BPM:" + 
                       String(bpm, 0) + ",SpO2:" + String(spo2, 0) + "%,ECG:" + 
                       String(ecgValue != -1 ? ecgValue : 0.0, 2) + "V,Alarm:" + alarmReason;
      if (sendSMS(message)) {
        smsSent = true;
      }
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    buzzerActive = false;
    smsSent = false;
    previousAlarmState = false;
  }

  digitalWrite(WIFI_STATUS_LED, wifiConfigured && WiFi.status() == WL_CONNECTED ? HIGH : LOW);
  static uint32_t tsLastReport = 0;
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    updateDisplay(dhtTemp, humidity, ds18b20Temp, ecgValue, bpm, spo2);
    publishToAWS(dhtTemp, humidity, ds18b20Temp, ecgValue, bpm, spo2, alarmCondition);
    tsLastReport = millis();
  }
}