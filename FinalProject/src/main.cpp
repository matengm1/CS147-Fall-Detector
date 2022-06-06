// Final Project
// Note: in the event of a file size error, the board can be repartitioned to accept it
#include "SparkFunLSM6DSO.h"
#include <TFT_eSPI.h>
#include "Wire.h"
#include <SPI.h>
#include <WiFi.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <HttpClient.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUTTON_PIN 27
#define BUZZER_PIN 12

const char* ssid = "MatthewiPhone"; // your network SSID (name of wifi network)
const char* pass = "password"; // your network password
const char kHostname[] = "13.56.19.48";
// Path to download (this is the bit after the hostname in the URL
// that you want to download
char kPath[50];
char output[50];

// Number of milliseconds to wait without receiving any data before we give up
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 500;
TFT_eSPI tft = TFT_eSPI();
LSM6DSO myIMU;
BLECharacteristic *pCharacteristic;

/***** SETUP *****/
int fallState;
#define SAFE 0 // Safe
#define TRIG1 1 // Possible Fall based on acceleration
#define TRIG2 2 // Likely fall based on no movement (gyroscope)
#define TRIG3 3 // Fall deteccted
#define FALL 4

void setup() {
  Serial.begin(9600);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  ledcSetup(0, 500, 8); // buzzer with PWM functionalitites
  ledcAttachPin(BUZZER_PIN, 0); // attach the channel to the GPIO to be controlled
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  fallState = SAFE;

  // SETUP 6DoF
  delay(500);
  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }
  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

  // SETUP BLE
  Serial.println("Data:");
  BLEDevice::init("Fall Detector");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setValue("No fall");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

unsigned long currTime;
int currVelocity;
int t1Time;
int t2Time;
int fallTime;
void loop() {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(10);
  tft.setCursor(100, 40);
  currTime = millis();
  int buttonState = digitalRead(BUTTON_PIN);
  
  float accl_x = myIMU.readFloatAccelX();
  float accl_y = myIMU.readFloatAccelY();
  float accl_z = myIMU.readFloatAccelZ();
  float gyro_x = myIMU.readFloatGyroX();
  float gyro_y = myIMU.readFloatGyroY();
  float gyro_z = myIMU.readFloatGyroZ();

  float acclMag = pow(pow(accl_x,2) + pow(accl_y,2) + pow(accl_z,2), 0.5);
  float gyroMag = pow(pow(gyro_x,2) + pow(gyro_y,2) + pow(gyro_z,2), 0.5);

  sprintf(output, "/?var=Accel:%f;Gyro:%f", acclMag, gyroMag);
  if (buttonState==false) { // Push button to mark safe
    fallState = SAFE;
    ledcWrite(0,0);
    sprintf(output, "MANUAL;DEACTIVATION");
  }

  switch(fallState) {
    case (SAFE):
      tft.fillScreen(TFT_GREEN);
      pCharacteristic->setValue("SAFE");
      if (acclMag > 5){
        fallState = TRIG1;
        t1Time = currTime;
      }
      break;
    
    case (TRIG1):
      tft.fillScreen(TFT_ORANGE);
      if (currTime - t1Time > 3000) { // Wait 3 secs for possible fall to complete/button press before checking movement
        fallState = TRIG2;
        t2Time = currTime;
        Serial.println("Likely fall");
        sprintf(output, "LIKELY;FALL");
      }
      break;

    case (TRIG2): // Likely fall
      tft.fillScreen(TFT_ORANGE);
      if (gyroMag>=60 && gyroMag<=360){ // If angle changes, person is moving
        fallState = SAFE;
        fallTime = currTime;
        Serial.print("Movement detected, cancelling alert: ");
        Serial.println(gyroMag);
      }
      if (currTime - t2Time > 3000) {
        fallState = FALL;
        fallTime = currTime;
        Serial.print("Angle changed: ");
        Serial.println(gyroMag);
      }
      sprintf(output, "LIKELY;FALL");
      break;

    case (FALL): // Fall reporting
      pCharacteristic->setValue("FALL");
      ledcWrite(0,60);
      tft.fillScreen(TFT_RED);
      sprintf(output, "FALL;DETECTED");
      if (currTime - fallTime >= 5000) { // Timeout and send alert to BLE (simulates sending a text/call with phone app)
        pCharacteristic->setValue("SAFE");
        fallState = SAFE;
        ledcWrite(0,0);
      }
      break;
  }

  int err =0;
  WiFiClient c;
  HttpClient http(c);
  
  err = http.get(kHostname,5000, output/*kPath*/);
  if (err == 0)
  {
    Serial.println("startedRequest ok");

    err = http.responseStatusCode();
    if (err >= 0)
    {
      Serial.print("Got status code: ");
      Serial.println(err);

      err = http.skipResponseHeaders();
      if (err >= 0)
      {
        int bodyLen = http.contentLength();
      
        // Now we've got to the body, so we can print it out
        unsigned long timeoutStart = millis();
        char c;
        // Whilst we haven't timed out & haven't reached the end of the body
        while ( (http.connected() || http.available()) &&
               ((millis() - timeoutStart) < 30000) )
        {
            if (http.available())
            {
                c = http.read();
                // Print out this character
                Serial.print(c);
               
                bodyLen--;
                // We read something, reset the timeout counter
                timeoutStart = millis();
            }
            else
            {
                // We haven't got any data, so let's pause to allow some to
                // arrive
                delay(30000);
            }
        }
      }
    }
  }

  http.stop();
}