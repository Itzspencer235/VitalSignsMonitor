#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>

#define REPORTING_PERIOD_MS 1000
#define RESET_BUTTON 23
#define WIFI_STATUS_LED 2
#define MQTT_MAX_PACKET_SIZE 1024

const char* aws_endpoint = "a1t4z1928sw433-ats.iot.ap-southeast-2.amazonaws.com";
const char* thing_name = "ESP32Device";
const char* certificatePemCrt = "-----BEGIN CERTIFICATE-----\n"
                                "MIIDWjCCAkKgAwIBAgIVALbBvvZfoNtAq9CaI9JgPWEi7/SyMA0GCSqGSIb3DQEB\n"
                                "CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\n"
                                "IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNTAzMjgxNDI1\n"
                                "MDRaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\n"
                                "dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDyMqg95NtFY7oHZ/BA\n"
                                "a9JOHkUD8j2/oMFAOuqlVAlFbPbvcCIwqxOfTzY9oAxkeYmcsngzmyXRdQ0P9ziJ\n"
                                "qLi5g6rGuLGdeyGFWfl3sYcgjD/4xD3vSb5pgBsVtR1+316iez+/nRTnxasuLWXJ\n"
                                "T7i0e5ZGDFeA3AjXikmw8g5Eg+6S968pN/EZJe0E5piUTjx5JncPL7V1slrehrUr\n"
                                "SBMyBWFkS2yT14FYPIoondFWhVTiIo+p0+6yp0aQHr8cnpmKpVP9tpx8nLkrqv5c\n"
                                "x7LffmxTPc1FaWRAxGLbC9PqkMq4sz4a9qEne4G6P8PsHE1FuMQsPHp02DCm8W/Y\n"
                                "6xwhAgMBAAGjYDBeMB8GA1UdIwQYMBaAFFu/e/4rnObUE9xw1/AhECx++wcpMB0G\n"
                                "A1UdDgQWBBRcJx/Wjxu8ORIFz/W2PelqBiNSrzAMBgNVHRMBAf8EAjAAMA4GA1Ud\n"
                                "DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAd8JyUli+IEt/+CWb/ZMOJCm0\n"
                                "Rir6Kj1MbPAsQU3v6DqJJwf39NuKxWF/DoWtTigddsjk9tJHv3V5Y2UZP9YNZiAS\n"
                                "QenpMnhnV1INt/DoeB3g62oxbgdqpNmCwy7cqCgYyGZyKGk8p/OwyJ6Q7iMlYE6i\n"
                                "eANhkIWuaSAwRt6J4FestHAnFLV2CFayMJOIugWsIg9jSj9fYXIJv2pFu7iQYq/f\n"
                                "j84UAmmXMFkzIgv9S2HWF9nCkg4hNyS0f5DqgXKGwCmwlCjMttZL5d/5Su0+P6pq\n"
                                "mk2DF/n0aY7JAwtApPnBcxrazaGpbac5e83FfEsslgCTjnWGN8CG8RCHgu7oFg==\n"
                                "-----END CERTIFICATE-----\n";
const char* privateKey = "-----BEGIN RSA PRIVATE KEY-----\n"
                         "MIIEpgIBAAKCAQEA8jKoPeTbRWO6B2fwQGvSTh5FA/I9v6DBQDrqpVQJRWz273Ai\n"
                         "MKsTn082PaAMZHmJnLJ4M5sl0XUND/c4iai4uYOqxrixnXshhVn5d7GHIIw/+MQ9\n"
                         "70m+aYAbFbUdft9eons/v50U58WrLi1lyU+4tHuWRgxXgNwI14pJsPIORIPukvev\n"
                         "KTfxGSXtBOaYlE48eSZ3Dy+1dbJa3oa1K0gTMgVhZEtsk9eBWDyKKJ3RVoVU4iKP\n"
                         "qdPusqdGkB6/HJ6ZiqVT/bacfJy5K6r+XMey335sUz3NRWlkQMRi2wvT6pDKuLM+\n"
                         "GvahJ3uBuj/D7BxNRbjELDx6dNgwpvFv2OscIQIDAQABAoIBAQCMnvLl9NGG6U8i\n"
                         "a6uni7KwXozNrMFDWK+7fjmswi0b3RdGkAGZ5kpfTdt9TkbDs3k9vLVjqSn9AgwS\n"
                         "gASYkJioRZVLCgM+HzaoOURJxY5iTGv8INyg6V0f3hxbryuv9Tr59bteonJ322H3\n"
                         "Tq5xgtMIH1Vx2EqZ66dKleURUokMJ7iewCUXJki03+eh6konTq1ZuZTPcypIfIRg\n"
                         "BVO+WoJ4MBo+xAYhC4u3c5cdcHXnSP0Olzab+AFJvGHfrM+5zKwK14lQlIaauoop\n"
                         "2xNYlozqwboDy+Zh/M1k5Ul3WKCZF5iZ4nnfYlV5JtesE9ehHsp316Lhubqe/5MX\n"
                         "Fh6jgOcNAoGBAPp/JweImobEGZp2+dxA/ZLCRXndI6q6Q64gzFWwoLF5Muk45dbH\n"
                         "64zghsGoDWRjfYPHcdyVaNnlzEcSo7vaaq4Nd6TOdliWnJOeg6rVTXcNUDVQCjUx\n"
                         "XabXlHSR+rSWbyxOzIaiu3A0s7aBBPyeD0ybBKXJqXlhE360W/YkzQdnAoGBAPeE\n"
                         "1Jd1X2tmn7EZ/5K0A9xMT46afxpPNA5nuL8VXyPBDZqd5wI/mhX3JXDfS2w+n/2s\n"
                         "ECAfd8uK9XDhXR5M4I5+YrSE2Z4kw2wTjbO5yevU/Qk/FGOmMxoMVQTQBrONDByv\n"
                         "TFCkiFAmEH7q0ECYGdsByaZI0OlE8oSEVy1+UzM3AoGBALXJ20A7YbFV1SLlHxU5\n"
                         "0QxVkmgLFVxMStuuj7vMMAI767eXhC0n7wjQ1Nrro7RsA2XEMxybDXm8rFAT9eFZ\n"
                         "dDwWJ/Lhpr9Jph33VqR+ofY5IjFegdnyln2DTAiA/ElcNxgXo5Q9uvuGM7nJzh2q\n"
                         "c33rribGwAgHbEdyip6KPIUXAoGBAKtCE9oL0w67vehK5cN6nGUzw6QWj7UDcjA5\n"
                         "YeGMhvx1GdWORYUPvqxOCXDUfxoRJp/dCnc/bqK9rd8Bf210obbWCbN/bBb4tuIz\n"
                         "kUr2VJjBEKbyJ0hfeWIfgD/JM6nOAMBkJfhyLwLNo4I9NARNeS5ISV6taSbNlsNL\n"
                         "0gnfKu/3AoGBAMAmofRWT7+hLwj6GnBp0ZXqso7JqcQRhBmwwKLboel2xWyV1mbF\n"
                         "LtvkpDvJhThhy4jIB8o+B2oeAvPLmN+J5Ll4Z0TKe28eKUDotAZHU0b0TDtYhS/m\n"
                         "ZhLt1vll7Ih8YY5DrWveW9MG7qTACYBinG9fgyaleuq5KqBOAi6fZ/om\n"
                         "-----END RSA PRIVATE KEY-----\n";
const char* rootCA = "-----BEGIN CERTIFICATE-----\n"
                     "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
                     "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
                     "b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
                     "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
                     "b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
                     "ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
                     "9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
                     "IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
                     "VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
                     "93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
                     "jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
                     "AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
                     "A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
                     "U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
                     "N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
                     "o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
                     "5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
                     "rqXRfboQnoZsG4q5WTP468SQvvG5\n"
                     "-----END CERTIFICATE-----\n";

WiFiClientSecure espClient;
PubSubClient client(espClient);
String serialBuffer = "";
bool wifiConfigured = false;

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

void setupNetwork() {
  WiFiManager wm;
  wm.setTimeout(30);
  if (wm.autoConnect("ESP32_Config")) {
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(WIFI_STATUS_LED, HIGH);
    wifiConfigured = true;
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

void publishToAWS(float dhtTemp, float humidity, float ds18b20Temp, float ecgValue, float bpm, float spo2, JsonArray ecgData, bool alarmCondition) {
  if (wifiConfigured && WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      connectAWS();
    }
    if (client.connected()) {
      client.loop();
      delay(50);
      StaticJsonDocument<512> doc;
      doc["dht_temperature"] = dhtTemp;
      doc["humidity"] = humidity;
      doc["ds18b20_temperature"] = ds18b20Temp;
      doc["ecg_voltage"] = ecgValue;
      doc["bpm"] = bpm;
      doc["spo2"] = spo2;
      doc["ecg_buffer"] = ecgData;
      doc["alarm"] = alarmCondition;

      char payload[512];
      size_t payloadSize = serializeJson(doc, payload);
      Serial.print("Publishing to topic 'vital_signs'...");
      if (client.publish("vital_signs", payload)) {
        Serial.println("Publish successful");
      } else {
        Serial.print("Publish failed, rc=");
        Serial.println(client.state());
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 18, 19); // For Mega
  pinMode(WIFI_STATUS_LED, OUTPUT);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  digitalWrite(WIFI_STATUS_LED, LOW);
  setupNetwork();
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
}

void loop() {
  if (digitalRead(RESET_BUTTON) == LOW) {
    resetWiFi();
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    serialBuffer += c;
    if (c == '\n') {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, serialBuffer);
      if (!error) {
        float dhtTemp = doc["dht_temperature"];
        float humidity = doc["humidity"];
        float ds18b20Temp = doc["ds18b20_temperature"];
        float ecgValue = doc["ecg_voltage"];
        float bpm = doc["bpm"];
        float spo2 = doc["spo2"];
        JsonArray ecgData = doc["ecg_buffer"];
        bool alarmCondition = doc["alarm"];
        publishToAWS(dhtTemp, humidity, ds18b20Temp, ecgValue, bpm, spo2, ecgData, alarmCondition);
      } else {
        Serial.println("JSON parse error: " + String(error.c_str()));
      }
      serialBuffer = "";
    }
  }

  digitalWrite(WIFI_STATUS_LED, wifiConfigured && WiFi.status() == WL_CONNECTED ? HIGH : LOW);
}