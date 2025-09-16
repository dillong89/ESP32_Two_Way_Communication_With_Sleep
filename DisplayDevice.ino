/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "string.h"
#include "FS.h"
#include "SD.h"

/* * * * * * *
 * Constants *
 * * * * * * */

#define uS_TO_S_FACTOR 1000000
#define M_TO_S_FACTOR 1000

#define SLEEP_TIME 10
#define WAIT_TIME 30

#define NUM_TRIES 1000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

/* * * * * * * * * *
 * Pin Connections *
 * * * * * * * * * */

#define SDA 21
#define SCL 22
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x3Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SD_CS 5
#define SCK 18
#define MISO 19
#define MOSI 23

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex

#define THERMISTOR_BUTTON_PIN 25
#define PHOTISTOR_BUTTON_PIN 26
#define BOTH_BUTTON_PIN 39

// Define bitmask for multiple GPIOs
uint64_t bitmask = BUTTON_PIN_BITMASK(THERMISTOR_BUTTON_PIN) | 
                   BUTTON_PIN_BITMASK(PHOTISTOR_BUTTON_PIN) |
                   BUTTON_PIN_BITMASK(BOTH_BUTTON_PIN);

/* * * * * * * * * * *
 * Global Variables  *
 * * * * * * * * * * */

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  /* * * * * * * * *
   * File Strings  *
   * * * * * * * * */

String WAIT = "";

String topDirStr = "/Measurement_Data";

String thermDirStr = "/Thermistor_Data";
String photoDirStr = "/Photistor_Data";
String bothDirStr = "/Both_Data";

String runningFileStr = "/Running_Data_Log";
String thermStr = "/Thermistor_Data_Log_";
String photoStr = "/Photo_Data_Log_";
String bothStr = "/Both_Data_Log_";

String suffix = ".txt";

String fileStr;

String headerLine;
String measLine;
String dataStr;

  /* * * * * * *
   * Counters  *
   * * * * * * */

int file_num = 0; // Incrimenting data log file number 
int tries = 0; // Wifi connection attempts

RTC_DATA_ATTR long int measurementCount = 0; // Measurement count for datalog time keeping

enum STATE_ENUM{
  IDLE_STATE,
  THERMISTOR_STATE,
  PHOTISTOR_STATE,
  BOTH_STATE,
};

int wakeupThermistorFlag = 0;
int wakeupPhotistorFlag = 0;
int wakeupBothFlag = 0;

// Timer Variables
long int timeout;
long int now;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0x83, 0x98, 0xfa, 0x78, 0x04};

// Define variables to store BME280 readings to be sent
int thermistor;
int photistor;
int state = IDLE_STATE;

// Define variables to store incoming readings
int incomingThermistor = 0;
int incomingPhotistor = 0;
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

/* * * * * * * * * * * * * * * *
 * BEGIN MAIN DRIVER FUNCTION  *
 * * * * * * * * * * * * * * * */

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Begin Wifi Setup
  //
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

  // Set unnecessary outgoing readings to 0.
  //  This is just for simplicity and because they have to have the same message.
  Readings.thermistor_data = 0;
  Readings.photistor_data = 0;
  //
  // END Wifi Setup

  // BEGIN SD Setup
  //
  SD.begin(5);

  // SD directory creation
  //    The checks are to not waste time or overwrite data.
  if (!SD.exists(topDirStr)) { createDir(SD, topDirStr); } // Create containing file
  if (!SD.exists(topDirStr + thermDirStr)) { createDir(SD, topDirStr + thermDirStr); }
  if (!SD.exists(topDirStr + photoDirStr)) { createDir(SD, topDirStr + photoDirStr); }
  if (!SD.exists(topDirStr + bothDirStr)) { createDir(SD, topDirStr + bothDirStr); }

  // Running file creation
  //    Only creates a running file if it does not already exist.
  if (!SD.exists(topDirStr + runningFileStr + suffix)) {
    writeFile(SD, topDirStr + runningFileStr + suffix, "Running Data Log\nt, Therm, Photo\n");
  }
  //
  // END SD Setup
  
  //delay(50); 

  // Print Wakeup Reason (Establish Connection if Button is pressed)
  print_wakeup_reason();

  // Init buttons last so they do not mess with setup
  pinMode(THERMISTOR_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(PHOTISTOR_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BOTH_BUTTON_PIN, INPUT_PULLDOWN);
}
  
void loop() {
  // For Data Log "time" keeping
  ++measurementCount;

  now = millis();

  // Event-Based FSM
  //    Used to control the measurement device state.
  //    Determines the state then creates message packet.
 
  if(digitalRead(THERMISTOR_BUTTON_PIN) || wakeupThermistorFlag == 1){
    Serial.println("Button Pressed for Thermistor\n");
    if (state != THERMISTOR_STATE) createFileFromInput(THERMISTOR_BUTTON_PIN);
    state = THERMISTOR_STATE;
    wakeupThermistorFlag = 0;
    timeout = now + WAIT_TIME*M_TO_S_FACTOR;
  }
  else if(digitalRead(PHOTISTOR_BUTTON_PIN) || wakeupPhotistorFlag == 1){
    Serial.println("Button Pressed for Photistor\n");
    if (state != PHOTISTOR_STATE) createFileFromInput(PHOTISTOR_BUTTON_PIN);
    state = PHOTISTOR_STATE;
    wakeupPhotistorFlag = 0;
    timeout = now + WAIT_TIME*M_TO_S_FACTOR;
  }
  else if(digitalRead(BOTH_BUTTON_PIN) || wakeupBothFlag == 1) {
    Serial.println("Button Pressed for Both\n");
    if (state != BOTH_STATE) createFileFromInput(BOTH_BUTTON_PIN);
    state = BOTH_STATE;
    wakeupBothFlag = 0;
    timeout = now + WAIT_TIME*M_TO_S_FACTOR;
  }
  else {
    Serial.println("No Button Pressed. Idling\n");
  }

  Readings.state = state;
  esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  ACK();

  if (tries > NUM_TRIES) { state = IDLE_STATE; }

  delay(50);
  
  // SAVE TO SD CARD
  dataStr = String(measurementCount) + "," + String(incomingThermistor) + "," + String(incomingPhotistor) + "\n";
  appendFile(SD, topDirStr + runningFileStr + suffix, dataStr);

  // State Centric FSM to determine what the chip should do
  switch(state){
    case IDLE_STATE:
      //display_sleeping();
      // Enable sleep timer.
      esp_sleep_enable_timer_wakeup(SLEEP_TIME * uS_TO_S_FACTOR);

      // Enable Wakeup Buttons
      esp_sleep_enable_ext1_wakeup_io(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);

      // Enter deep sleep mode
      Serial.println("Going to sleep now");
      Serial.flush(); 
      esp_deep_sleep_start();
      break;

    case THERMISTOR_STATE:
      dataStr = String(measurementCount) + "," + String(incomingThermistor )+ "\n";
      appendFile(SD, fileStr, dataStr);
      display_thermistor();
      break;

    case PHOTISTOR_STATE:
      dataStr = String(measurementCount) + "," + String(incomingPhotistor) + "\n";
      appendFile(SD, fileStr, dataStr);
      display_photistor();
      break;

    case BOTH_STATE:
      // Timer Check
      dataStr = String(measurementCount) + "," + String(incomingThermistor) + "," + String(incomingPhotistor) + "\n";
      appendFile(SD, fileStr, dataStr);
      display_both();
      break;

    default:
      Serial.print("Default: Should not print");
  }

  if(now > timeout && timeout != 0){
      Serial.println("Timer Elapsed, Returning to Idle\n");
      display_idle();
      state = IDLE_STATE;
  }

  Serial.print("\nThermistor: "); Serial.print(incomingThermistor);
  Serial.print("\nPhotistor: "); Serial.print(incomingPhotistor);
  Serial.print("\nState: "); Serial.print(state);
  Serial.print("\n");
}



/* * * * * * * * * * *
 * Set-Up Functions  *
 * * * * * * * * * * */

  /* * * * * * * * * * * * * * 
   * Wifi (ESPNOW) Functions *
   * * * * * * * * * * * * * */

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

void ACK(){
  tries = 0;
  while(incomingState == -1 && ++tries < NUM_TRIES) {
    Serial.println("Awaiting Response...\n");
    esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
  } // Request acknowledged
}

  /* * * * * * * * * * * * * * * * * * * 
   * Sleep (ESP Deep Sleep) Functions  *
   * * * * * * * * * * * * * * * * * * */

void button_GPIO_wake_up(){
  // Determine what button woke up the ESP
  int GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  int wakeup_GPIO = (log(GPIO_reason))/log(2);
  Serial.print("GPIO that triggered the wake up: GPIO ");
  Serial.println(wakeup_GPIO, 0);


  switch(wakeup_GPIO) {
    case THERMISTOR_BUTTON_PIN:
      wakeupThermistorFlag = 1;
      Readings.state = THERMISTOR_STATE;
      break;
    case PHOTISTOR_BUTTON_PIN:
      wakeupPhotistorFlag = 1;
      Readings.state = PHOTISTOR_STATE;
      break;
    case BOTH_BUTTON_PIN:
      wakeupBothFlag = 1;
      Readings.state = BOTH_STATE;
      break;
    default:
      Readings.state = IDLE_STATE;
      Serial.println("Error? Returning to Idle state.");
  }
    
  delay(20);
  // Initialize display
  begin_display();
  int cnt = 0;
  // Establish a connection to the other board.
  while(incomingState == -1) {
    Serial.println("Button Awaiting Response...\n");
    if (++cnt > 5000) {
      display_connecting();
      cnt = 0;
    }
    else 
      cnt++;
    esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));
    delay(5);
  } // Request acknowledged
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      button_GPIO_wake_up();
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

  /* * * * * * * * * * 
   * SD/FS Functions *
   * * * * * * * * * */

void createDir(fs::FS &fs, String path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void writeFile(fs::FS &fs, String path, String message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

  /* * * * * * * * * * *
   * Display Functions *
   * * * * * * * * * * */

void begin_display() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void display_connecting() {
  display.clearDisplay();
  display.setTextSize(2 );             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Connecting"); display.println(WAIT);
  if(WAIT == ". . . ") { WAIT = ""; } 
  else WAIT += ". "; 
  display.display();
}

/* * * * * * * * * * * *
 * Run-Time Functions  *
 * * * * * * * * * * * */

  /* * * * * * * * * *
   * SD/FS Functions *
   * * * * * * * * * */

void appendFile(fs::FS &fs, String path, String message){
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(!file.print(message)){
    Serial.println("Append failed");
  }
  file.close();
}

void createFileFromInput(int button_num){
  // Event-Centric FSM to tell the loop function what state to enter
  //  also controls file writing.
  file_num = 0;
  switch(button_num) {
    case THERMISTOR_BUTTON_PIN:
      fileStr = topDirStr + thermDirStr + thermStr;
      headerLine = "Thermistor Data Log ";
      measLine = "t, Therm\n";
      break;
    case PHOTISTOR_BUTTON_PIN:
      fileStr = topDirStr + photoDirStr + photoStr;
      headerLine = "Photistor Data Log ";
      measLine = "t, Photo\n";
      break;
    case BOTH_BUTTON_PIN:
      fileStr = topDirStr + bothDirStr + bothStr;
      headerLine = "Both Data Log ";
      measLine = "t, Therm, Photo\n";
      break;
    default:
      state = IDLE_STATE;
      Serial.println("Error? Returning to Idle state.");
  } 

  while (SD.exists(fileStr + String(file_num) + suffix)) { file_num++; } // scan files in Measurement_data directory to find new data file number

  fileStr += String(file_num) + (suffix); // create new file string fre accessability
  
  writeFile(SD, (fileStr), headerLine + String(file_num) + "\n" + measLine); // write new file to store data
}

  /* * * * * * * * * * *
   * Display Functions *
   * * * * * * * * * * */

void display_idle() {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.println("IDLE");;
  display.display();
  //delay(10);
}

void display_thermistor() {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Thermistor: "); display.println(incomingThermistor);
  display.display();
  //delay(10);
}

void display_photistor() {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Photistor: "); display.println(incomingPhotistor);
  display.display();
  //delay(10);
}

void display_both() {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Thermistor: "); display.println(incomingThermistor);
  display.print("Photistor: "); display.println(incomingPhotistor);
  display.display();
  //delay(10);
}