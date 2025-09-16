/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>

// Conversion Factors and Limits
#define uS_TO_S_FACTOR 1000000
#define SLEEP_TIME 10

#define NUM_TRIES 10000

// Pin connections
#define THERMISTOR_PIN A0
#define PHOTISTOR_PIN A3

#define LED_RED_PIN 5
#define LED_GREEN_PIN 10
#define LED_BLUE_PIN 9

enum states{
  IDLE_STATE,
  THERMISTOR_STATE,
  PHOTISTOR_STATE,
  BOTH_STATE,
};

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0x83, 0x98, 0xf9, 0xb6, 0x10};

int tries = 0;

// Define variables to store BME280 readings to be sent
int thermistor;
int photistor;
int state = IDLE_STATE;

// Define variables to store incoming readings
int incomingThermistor;
int incomingPhotistor;
int incomingState = -1;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int thermistor_data;
    int photistor_data;
    int state;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingThermistor = incomingReadings.thermistor_data;
  incomingPhotistor = incomingReadings.photistor_data;
  incomingState = incomingReadings.state;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Init RGB LED
  pinMode(LED_RED_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(LED_GREEN_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(LED_BLUE_PIN, OUTPUT_OPEN_DRAIN);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // Ensure the connection is established.
  delay(70);
  ACK();
  //delay(100);
  

  thermistor = analogRead(THERMISTOR_PIN);
  photistor = analogRead(PHOTISTOR_PIN);
  state = incomingState;
  Readings.thermistor_data = thermistor;
  Readings.photistor_data = photistor;
  Readings.state = state;

  esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  if (tries >= NUM_TRIES) { incomingState = IDLE_STATE; }
  switch(incomingState){
    case IDLE_STATE:
      // Enter deep sleep mode
      Serial.println("Going to sleep now");
      // Enable sleep timer.
      esp_sleep_enable_timer_wakeup(SLEEP_TIME * uS_TO_S_FACTOR);
      setColor(0, 0, 0);
      Serial.flush(); 
      esp_deep_sleep_start();
      break;
    case THERMISTOR_STATE:
      Serial.println("Thermistor Data Being Sent");
      setColor(255, 0, 0);
      break;
    case PHOTISTOR_STATE:
      Serial.println("Photistor Data Being Sent");
      setColor(0, 255, 0);
      break;
    case BOTH_STATE:
      Serial.println("Both Sensors Data Being Sent");
      setColor(0, 0, 255);
      break;
    default:
      Serial.println("Default: Should not print");
  }
}

void ACK(){
  tries = 0;
  while(incomingState == -1 && ++tries < NUM_TRIES) {
    Serial.println("Awaiting Response...");
    esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
  } // Request acknowledged
}

void setColor(int redValue, int greenValue,  int blueValue) {
  analogWrite(LED_RED_PIN, redValue);
  analogWrite(LED_GREEN_PIN,  greenValue);
  analogWrite(LED_BLUE_PIN, blueValue);
}




