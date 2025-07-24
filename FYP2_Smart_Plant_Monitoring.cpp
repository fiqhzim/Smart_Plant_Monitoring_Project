#define BLYNK_TEMPLATE_ID "TMPL6JIKLSH0S"
#define BLYNK_TEMPLATE_NAME "testing pir"
#define BLYNK_AUTH_TOKEN "rPg3-zAD71NQe2GS8zM86k38zR-bSssq"

//#define BLYNK_TEMPLATE_ID "TMPL63tXzCIsk"
//#define BLYNK_TEMPLATE_NAME "soil moistureTest"
//#define BLYNK_AUTH_TOKEN "i9L0Q0ehs9xZKcBxoJbBG6BP4UOn0ZsG"

// Including libraries
#include "WiFi.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DHTesp.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Defining constants and pins
//#define On_Board_LED_PIN  21 (KIV)
#define DHTPIN  16
#define DHTTYPE DHT22

// DS18B20 setup
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// TDS sensor setup
#define TDS_PIN 35
const float VREF = 3.3;
const int SAMPLES = 10;


// PIR sensor settings
#define PIR 23      // D5 PIR Motion Sensor
#define buzzerPin 21
int pirstate = 0;
int awayHomeMode = 0;
int lastPIRState = LOW; 

//int PIR_ToggleValue;

//const int buzzerPin = 21;
const int SOIL_MOISTURE_PIN = 34;
const int TRIG_PIN = 12;  // Ultrasonic sensor TRIG pin
const int ECHO_PIN = 13;  // Ultrasonic sensor ECHO pin
const float TANK_HEIGHT_CM = 20.0; // Tank height in cm

// Define pin configurations for the water pump
const int relay_1 = 27;
bool relay1_Status = 0;

// Define auto mode condition
int autoMode = 0;
int waterThreshold = 0;        // The soil moisture percentage threshold to activate watering
//float moisturePercentage = 0;  // The calculated soil moisture percentage


// Wi-Fi and Blynk credentials
const char* ssid = "13-1@unifi";  
const char* password = "diniunikl1234"; 
const char* auth = BLYNK_AUTH_TOKEN;

// Google script Web App URL
String Web_App_URL = "https://script.google.com/macros/s/AKfycbwhTKwx7Ot2MKJn11OE-r62ix0ribebI8tsq53zkG9xZ7RFnPSVoOy4H7T6jTUOSltI/exec";

// Variables for sensor readings
float Temp;
float Humd;
float soilmoisture = 0;
float waterLevelPercentage = 0;
float ecValue = 0;
String Status_Read_Sensor = "Failed";
String Status_Read_Sensor1 = "Failed";
String Status_Read_Sensor2 = "Failed";

// Timer for periodic tasks
BlynkTimer timer;

// DHT sensor object
DHTesp dht;

// Function to read DHT11 sensor data
void Getting_DHT11_Sensor_Data() {
  Humd = dht.getHumidity();
  Temp = dht.getTemperature();

  if (isnan(Humd) || isnan(Temp)) {
    Serial.println("Failed to read from DHT sensor!");
    Status_Read_Sensor = "Failed";
    Temp = 0.0;
    Humd = 0.0;
  } else {
    Status_Read_Sensor = "Success";
  }

  Serial.print("Temp: ");
  Serial.print(Temp);
  Serial.print(" °C, Humidity: ");
  Serial.print(Humd);
  Serial.println(" %");
}

// Function to read soil moisture sensor data
void soil_moisture_data() {
  int rawValue = analogRead(SOIL_MOISTURE_PIN);
  int soilMoisturePercentage = map(rawValue, 0, 4095, 0, 100);

  if (rawValue < 0 || rawValue > 4095) {
    Serial.println("Failed to read from soil moisture sensor!");
    Status_Read_Sensor1 = "Failed";
    soilmoisture = 0;
  } else {
    Status_Read_Sensor1 = "Success";
    soilmoisture = soilMoisturePercentage;
  }

  Serial.print("Soil Moisture: ");
  Serial.print(soilmoisture);
  Serial.println(" %");
}

// Function to read ultrasonic sensor data
void ultrasonic_sensor_data() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Calculate distance in cm

  if (distance < 0 || distance > TANK_HEIGHT_CM) {
    Serial.println("Failed to read from ultrasonic sensor!");
    Status_Read_Sensor2 = "Failed";
    waterLevelPercentage = 0;
  } else {
    Status_Read_Sensor2 = "Success";
    waterLevelPercentage = ((TANK_HEIGHT_CM - distance) / TANK_HEIGHT_CM) * 100;
  }

  Serial.print("Water Level: ");
  Serial.print(waterLevelPercentage);
  Serial.println(" %");
}

// This function is called every time the Virtual Pin 1 state changes
BLYNK_WRITE(V1) {
  waterThreshold = param.asInt();  // Update watering threshold
  Serial.print("Received threshold.   waterThreshold:");
  Serial.println(waterThreshold);
}

// This function is called every time the Virtual Pin 2 state changes
BLYNK_WRITE(V2) {
  autoMode = param.asInt();  // Update auto mode status

  if (autoMode == 1) {
    Serial.println("The switch on Blynk has been turned on.");
  } else {
    Serial.println("The switch on Blynk has been turned off.");
  }
}

BLYNK_WRITE(V3) {
  awayHomeMode = param.asInt();  // Set incoming value from pin V0 to a variable

  if (awayHomeMode == 1) {
    Serial.println("The switch on Blynk has been turned on.");
    Blynk.virtualWrite(V4, "Detecting signs of intrusion...");
  } else {
    Serial.println("The switch on Blynk has been turned off.");
    Blynk.virtualWrite(V4, "Away home mode close");
  }
}


//void PIRTimerEvent() {
//  // Please don't send more that 10 values per second.
//  pirData();  // Call function to send sensor data to Blynk app
//}



// Function to send sensor data to Blynk app
//void pirData() {
//  if (awayHomeMode == 1) {
//    pirstate = digitalRead(PIR);  // Read the state of the PIR sensor
//
//    Serial.print("state:");
//    Serial.println(pirstate);
//
//    // If the sensor detects movement, send an alert to the Blynk app
//    if (pirstate == HIGH ) {
//      Serial.println("ringing buzzer!");
//      tone(buzzerPin, 1000);
//      Blynk.virtualWrite(V4, "intruder! Ringing buzzer!");
//      Blynk.logEvent("intrusion_detected");
//      delay(2000);
//      noTone(buzzerPin);
//    } else {  // No motion detected
//      Blynk.virtualWrite(V4, "No intruder");
//      noTone(buzzerPin);  // Turn off the buzzer
//    } 
//  }
//}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V3);
}

// Function to send data to Google Sheets
void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected. Skipping Google Sheets update.");
    return;
  }

  String Send_Data_URL = Web_App_URL + "?sts=write";
  Send_Data_URL += "&srs=" + Status_Read_Sensor;
  Send_Data_URL += "&temp=" + String(Temp, 2);
  Send_Data_URL += "&humd=" + String(Humd, 2);
  Send_Data_URL += "&swtc1=" + String(soilmoisture);
  Send_Data_URL += "&swtc2=" + String(ecValue, 2);

  Serial.println("Sending data to Google Sheets...");
  Serial.println("URL: " + Send_Data_URL);

  WiFiClientSecure client;
  client.setInsecure();  // Disable SSL certificate validation for testing
  HTTPClient http;

  http.begin(client, Send_Data_URL);
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("Payload: " + payload);
  } else {
    Serial.print("HTTP GET failed, error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

// Function to send data to Blynk
void sendToBlynk() {
  Blynk.virtualWrite(V7, Temp);
  Blynk.virtualWrite(V8, Humd);
  Blynk.virtualWrite(V0, soilmoisture);
  Blynk.virtualWrite(V5, waterLevelPercentage);
  Blynk.virtualWrite(V6, ecValue);  // Send EC to Blynk V6

}

// Wi-Fi connection setup
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWi-Fi connected!");
}

// Function to control automatic watering based on soil moisture and user settings
void autoWater() {
  if (autoMode == 1 && soilmoisture < waterThreshold) {

    if (!relay1_Status) {
      turnOnPump();
      Serial.println("-----------------------------");
      Serial.println(waterThreshold);
      Serial.print("  moisturePercentage:");
      Serial.println(soilmoisture);
      Serial.println("Watering...");

      // Turn off pump after 2 seconds
      timer.setTimeout(2000L, turnOffPump);
    }
  }
}

// Function to turn on the water pump
void turnOnPump() {
  digitalWrite(relay_1, HIGH);
  relay1_Status = 1;
}


// Function to turn off the water pump
void turnOffPump() {
  digitalWrite(relay_1, LOW);
  relay1_Status = 0;
}

void autoWaterTimer() {
  autoWater();
}

void sendECValue(){
  // Read temperature
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  // Read analog
  int analogValue = 0;
  for (int i = 0; i < SAMPLES; i++) {
    analogValue += analogRead(TDS_PIN);
    delay(10);
  }
  analogValue /= SAMPLES;

  float voltage = analogValue * VREF / 4095.0;

  // Temperature compensation
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensatedVoltage = voltage / compensationCoefficient;

  // EC in µS/cm (from sensor formula)
  ecValue = 133.42 * pow(compensatedVoltage, 3)
          - 255.86 * pow(compensatedVoltage, 2)
          + 857.39 * compensatedVoltage;

  // TDS using 0.64 factor
  float tdsValue = ecValue * 0.64;

  // Output
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("EC: ");
  Serial.print(ecValue, 2);
  Serial.println(" µS/cm");

  Serial.print("TDS: ");
  Serial.print(tdsValue, 2);
  Serial.println(" ppm");

  Serial.println("-------------------------");


}

// Setup function
void setup() {
  Serial.begin(115200);
  sensors.begin();
  analogReadResolution(12);
//  pinMode(On_Board_LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR, INPUT);  // Initialize PIR sensor pin as input
  pinMode(relay_1, OUTPUT);    // set relay1 as output
  digitalWrite(relay_1, LOW);  // Keep pump1B low
  pinMode(buzzerPin, OUTPUT);
  noTone(buzzerPin);  // Ensure it's off at boot
  
  dht.setup(DHTPIN, DHTesp::DHT22);
  Blynk.begin(auth, ssid, password);

  connectToWiFi();
  
//WRITE LABEL IN WIDGET IN BLYNK
  Blynk.virtualWrite(V94, "1200-1500 uS/cm");
  Blynk.virtualWrite(V95, "1800-2200 uS/cm");
  Blynk.virtualWrite(V96, "2000-2500 uS/cm");
  Blynk.virtualWrite(V97, "2200-2700 uS/cm");
//////////////////////////////////////////////////////////////
 
  timer.setInterval(5000L, sendECValue);
  timer.setInterval(5000L, Getting_DHT11_Sensor_Data);
  timer.setInterval(5000L, soil_moisture_data);
  timer.setInterval(5000L, ultrasonic_sensor_data);
  timer.setInterval(8000L, sendToGoogleSheets);
  timer.setInterval(5000L, sendToBlynk);
//  timer.setInterval(3000L, sendPIR);  // Call PIR sensor check every 3 seconds
//  timer.setInterval(2000L, PIRTimerEvent);
  timer.setInterval(10000L, autoWaterTimer);   // Check watering conditions every 10 seconds
}

// Main loop
void loop() {
  Blynk.run();
    if (awayHomeMode == 1) {
    int currentState = digitalRead(PIR);

    if (currentState != lastPIRState) {
      lastPIRState = currentState;

      if (currentState == HIGH) {
        Serial.println("Motion Detected!");
        tone(buzzerPin, 1000);
        Blynk.virtualWrite(V4, "Motion Detected!");
        Blynk.logEvent("intrusion_detected");
      } else {
        Serial.println("No Motion.");
        Blynk.virtualWrite(V4, "No Motion");
        noTone(buzzerPin);
      }
    }
  } else {
  noTone(buzzerPin);
  Blynk.virtualWrite(V4, "Away mode off");
  }
  timer.run();
}