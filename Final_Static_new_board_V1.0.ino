/*
   Author: Sakshi Kshirsagar
   Reviewer: Prasad Sawant
   Firmware Version: 1.0
   Firmware Date: 25/03/2025
   Flow Chart:

   1) After power on and welcome audio, device bluetooth is on.
   2) For skipping and configuring new user, settings can be done using Payment Box app using bluetooth.
   3) After user authentication, current user is displayed, user can continue with same or can change.
   4) parameters for user- Bank details and UPI details (bank name, last 4 AC no.)(UPI ID, User name, Currency).
   5) Network connection, configuring cellular with AT commands.
   6) QR code, user details, bank details are set for current user.
   7) Updating the transaction after receiving message, giving alert with audio and showing on screen.
*/

#include<Preferences.h>
Preferences pref;

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"                           // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"                        // RX characteristic UUID
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"                        // TX characteristic UUID

#include <SPI.h>
#include <TFT_eSPI.h>
#include "qrcode.h"
#include <ArduinoJson.h>
#include <BluetoothSerial.h>                                                                 // Include BluetoothSerial library
#include <Arduino.h>
#include <Wire.h>
#include "Audio.h"
#include <SD.h>
#include <SoftwareSerial.h>                                                                  // Include SoftwareSerial library
#include <FS.h>
#include <SPIFFS.h>
#include <driver/adc.h>
/**************************** // Create TFT and Bluetooth objects  ************************************/
TFT_eSPI tft = TFT_eSPI();
Audio audio;
BluetoothSerial SerialBT;

// Initialize SPI for SD card on HSPI
SPIClass hspi(HSPI);
/**************************** // General-purpose I/O pins  ************************************/
#define GPIO0  0    // Boot mode selection, often used as a button or input.
#define GPIO1  1    // UART0 TX, can also be used as a general-purpose I/O.
#define GPIO2  2    // UART0 RX, can also be used as a general-purpose I/O.
#define GPIO3  3    // UART0 RX, can also be used as a general-purpose I/O.
#define GPIO4  4    // General-purpose I/O, used in various functions.
#define GPIO5  5    // General-purpose I/O, often used for SPI CS or other functions.
#define GPIO6  6    // Typically used for the internal flash memory; not recommended for general use.
#define GPIO7  7    // Typically used for the internal flash memory; not recommended for general use.
#define GPIO8  8    // Typically used for the internal flash memory; not recommended for general use.
#define GPIO9  9    // Typically used for the internal flash memory; not recommended for general use.
#define GPIO10 10   // Typically used for the internal flash memory; not recommended for general use.
#define GPIO11 11   // Typically used for the internal flash memory; not recommended for general use.
#define GPIO12 12   // General-purpose I/O, also used for boot mode selection.
#define GPIO13 13   // General-purpose I/O, often used for SPI or other functions.
#define GPIO14 14   // General-purpose I/O, often used for SPI or other functions.
#define GPIO15 15   // General-purpose I/O, also used for boot mode selection.
#define GPIO16 16   // General-purpose I/O, used in various functions.
#define GPIO17 17   // General-purpose I/O, used in various functions.
#define GPIO18 18   // SPI CLK, can also be used as a general-purpose I/O.
#define GPIO19 19   // SPI MISO, can also be used as a general-purpose I/O.
#define GPIO20 20   // General-purpose I/O, not always available.
#define GPIO21 21   // I2C SDA, can also be used as a general-purpose I/O.
#define GPIO22 22   // I2C SCL, can also be used as a general-purpose I/O.
#define GPIO23 23   // SPI MOSI, can also be used as a general-purpose I/O.
#define GPIO24 24   // General-purpose I/O, used in various functions.
#define GPIO25 25   // DAC1 output, also used as a general-purpose I/O.
#define GPIO26 26   // DAC2 output, also used as a general-purpose I/O.
#define GPIO27 27   // General-purpose I/O, used in various functions.
#define GPIO28 28   // General-purpose I/O, not always available.
#define GPIO29 29   // General-purpose I/O, not always available.
#define GPIO30 30   // General-purpose I/O, not always available.
#define GPIO31 31   // General-purpose I/O, not always available.
#define GPIO32 32   // ADC1 input, also used as a general-purpose I/O.
#define GPIO33 33   // ADC1 input, also used as a general-purpose I/O.
#define GPIO34 34   // Input-only, often used for ADC.
#define GPIO35 35   // Input-only, often used for ADC.
#define GPIO36 36   // Input-only, often used for ADC, also known as VP (Voltage Reference Pin).
#define GPIO37 37   // Input-only, often used for ADC.
#define GPIO38 38   // Input-only, often used for ADC.
#define GPIO39 39   // Input-only, often used for ADC, also known as VN (Negative Voltage Pin).

/**************************** USED PINS   ***************************************/

/**************************** MICRO SD CARD PINS   ******************************/
/* BOARD 1*/
#define SD_CS         5
#define SPI_MOSI      18
#define SPI_MISO      32
#define SPI_SCK       19


/*BOARD 2
  #define SD_CS         18
  #define SPI_MOSI      22
  #define SPI_MISO      21

  #define SPI_SCK       23
*/
/**************************** I2S AUDIO AMPLIFIRE PINS    *********************************/
/* BOARD 1*/
#define I2S_DOUT      15
#define I2S_BCLK      2
#define I2S_LRC       4

/*BOARD 2
  #define I2S_DOUT      26
  #define I2S_BCLK      27
  #define I2S_LRC       14
*/
/**************************** SIM800L GSM PINS     ************************************/
/* BOARD 1*/
#define GSM_TX        17
#define GSM_RX        16

/*BOARD 2
  #define GSM_TX        12
  #define GSM_RX        13
*/

/**************************** PREV BUTTON PIN     ************************************/
/* BOARD 1*/
#define BUTTON_PIN 35                                                                      // Define the button pin 
#define pwr_key 23
#define increaseVol_PIN 36
#define decreaseVol_PIN 39
int currentVolume = 100;
bool increaseVol = false, decreaseVol = false;
/*BOARD 2
  #define BUTTON_PIN 33
*/

/* BOARD 1*/
const int batteryPin = 34;                                                                // Define the analog pin connected to the battery voltage

/**************************** TFT PINS USED IN LIBERARY    ****************************/
//#define TFT_MISO 19                                                                    // (leave TFT SDO disconnected if other SPI devices share MISO)
//#define TFT_MOSI 23
//#define TFT_SCLK 18
//#define TFT_CS   15                                                                    // Chip select control pin
//#define TFT_DC    2                                                                    // Data Command control pin
//#define TFT_RST   4                                                                    // Reset pin (could connect to RST pin)
//
//// Optional touch screen chip select
//#define TOUCH_CS 5                                                                     // Chip select pin (T_CS) of touch screen

/*************************************************************************************/
#define TFT_MISO 33                                                                     // (leave TFT SDO disconnected if other SPI devices share MISO)
#define TFT_MOSI 25
#define TFT_SCLK 27
#define TFT_CS   13                                                                     // Chip select control pin
#define TFT_DC   14                                                                     // Data Command control pin
#define TFT_RST  12                                                                     // Reset pin (could connect to RST pin)

// Optional touch screen chip select
//#define TOUCH_CS 5                                                                    // Chip select pin (T_CS) of touch screen

/**************************** FILE NAMES  ************************************/
#define CALIBRATION_FILE "/TouchCalData1"
#define QR_CODE_FILE "/qr_code.dat"
#define QR_DFLT_FILE "/qr_dflt.dat"
#define REPEAT_CAL false

/**************************** KEYPAD CONSTANT  ************************************/
#define KEY_X 40
#define KEY_Y 96
#define KEY_W 62
#define KEY_H 30
#define KEY_SPACING_X 18
#define KEY_SPACING_Y 20
#define KEY_TEXTSIZE 1
#define LABEL1_FONT &FreeSansOblique12pt7b
#define LABEL2_FONT &FreeSansBold12pt7b
#define DISP_X 1
#define DISP_Y 10
#define DISP_W 238
#define DISP_H 50
#define DISP_TSIZE 3
#define DISP_TCOLOR TFT_BLUE
#define NUM_LEN 8
#define FORMAT_SPIFFS_IF_FAILED true

/**************************** // variable declarations ************************************/
unsigned long bluetoothStartTime;
const unsigned long bluetoothDuration = 2 * 60 * 1000;                                       // 2 minutes in milliseconds
volatile bool Connected = false;
volatile bool Parameter_flag = false;
volatile bool Password_flag = false;
volatile bool Timeout_flag = false;

char Recieved[100] = "abcd";
// All below variables Are used for Bt Through configuration
BLECharacteristic *pCharacteristic;
#define PASSKEY 654321
String PASSWORD  = "Embel01";

#define debounceTime 900
unsigned long lastInterruptTime;
bool Button_flag = false;
hw_timer_t *timer = NULL;
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic;
bool m_connection = false;
unsigned long startTime;

String final_amount = "";
String res = "";
String response = "";
char* lower = "12341197";
char* promsg = "";
char* msg = "+cmt: \"8446100765\"";
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;
char keyLabel[15][5] = {"New", "Del", "Gen.", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#" };
uint16_t keyColor[15] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE };
TFT_eSPI_Button key[15];
enum class Page { Keypad, QRCode , defaultQRCode};
Page currentPage = Page::Keypad;
// User input variables
String deviceConfig = "1";
String bankName = "";
String bankID = "";
String bankID_4 = "";
String upiID = "";
String userName = "";
String currency = "";
String apnname = "";
bool para_setting = false;
bool decision = false;

float amount = 0;
char amount_File[] = "/amount.txt";
int fs_size = 0;

int Button_cnt = 0;
int Msg_cnt = 0;

/**************************** BATTERY MANAGEMENTS RELATED VARIABLES  ************************************/
const float referenceVoltage = 3.3;                                                    // Reference voltage of ESP32 ADC
const int adcMaxValue = 4095;                                                          // Maximum ADC value for 12-bit resolution
// Battery voltage range
const float minBatteryVoltage = 1.3; //1.3;                                            // Minimum battery voltage for 0% charge (adjust as needed)
const float maxBatteryVoltage = 2.1; //2.1;                                            // Maximum battery voltage for 100% charge (adjust as needed)
// Thresholds for battery percentage
const float lowThreshold = 25.0;                                                       // Below 20% is considered low
const float mediumThreshold = 50.0;                                                    // 50% and above is considered medium
const float fullThreshold = 100.0;                                                      // 90% and above is considered full
int ADCvalue;
float batvoltage;
int batperc;
unsigned long bat_cal = 0;

// Variable to store the last battery status
String lastBatteryStatus = "";
int counter = 0; int check_tp = 30;
int countdownTime = 0;                                                                 // Countdown time in seconds
unsigned long previousMillis = 0;
unsigned long interval = 1000;                                                         // Interval for 1 second
/**************************** // Flags to track if the messages have been printed ************************************/
bool lastButtonState = HIGH;                                                           // Track the last button state
bool lowBatteryMessagePrinted = false;
bool fullBatteryMessagePrinted = false;
bool audioPlaying = false;
bool networkAvailable = false;

#define BUFFER_SIZE 50
String messageBuffer[BUFFER_SIZE];
int index1 = 0;

/**************************** GSM SIM800L ************************************/

#define Serial Serial
// Define the serial port to be used for debugging and monitoring
// This typically maps to the USB serial connection

#define DUMP_AT_COMMANDS
// Define this macro to enable debugging of AT commands
// Comment this out if AT command debugging is not needed

#define TINY_GSM_DEBUG Serial
// Set the debug output for the TinyGSM library to use the Serial port
// This enables logging of library debug information to the serial monitor

#define TINY_GSM_MODEM_SIM800
// Define the modem type as SIM800
// This configures the TinyGSM library to use the SIM800 modem

#define TINY_GSM_RX_BUFFER 1024
// Set the receive buffer size for the TinyGSM library
// This allocates 1KB of memory for the RX buffer to store incoming data

#include <TinyGsmClient.h>
// Include the TinyGSM library header
// This library provides functionality for communicating with GSM modems

// Set serial port for AT commands (to communicate with the GSM module)
#define Serial2 Serial2
// Define Serial2 for communicating with the GSM module
// On most boards, Serial2 maps to specific hardware UART pins

SoftwareSerial mySoftwareSerial(GSM_RX, GSM_TX);
// Create a SoftwareSerial object for communication with the GSM module
// GSM_RX and GSM_TX are the pin definitions for receive and transmit

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
// Include the StreamDebugger library if DUMP_AT_COMMANDS is defined
// This library provides functionality to log and debug AT commands

StreamDebugger debugger(Serial2, Serial);
// Create a StreamDebugger object that wraps the Serial2 and Serial
// This allows for debugging AT commands sent to and received from the modem

TinyGsm modem(debugger);
// Initialize the TinyGsm modem object using the debugger stream
#else
TinyGsm modem(Serial2);
// Initialize the TinyGsm modem object using the Serial2 port directly
#endif

/**************************** BLE CLASSIC  ************************************/
void IRAM_ATTR onTimer() {                                                              // For bluetooth configuration setting
  Serial.println("No connection within 2 minutes. Deinitializing BLE...");
  Timeout_flag = true;
}

void IRAM_ATTR Buttonhandler() {                                                       // button press count handler
  Serial.println("Buttonhandler..........");
  timerAlarmDisable(timer);
  timerAlarmDisable(timer);
  Serial.println("Button_cnt is :");
  Serial.println(Button_cnt);
  Button_flag = true;
}

void IRAM_ATTR buttonpress() {                                                         // button press handler
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > 30) {
    Serial.println("IRAM_ATTR buttonPress :###############");

    if (Button_cnt == 0) {
      timer = timerBegin(0, 80, true);                                                 // Timer 0, prescaler 80, counting up
      timerAttachInterrupt(timer, &Buttonhandler, true);
      timerAlarmWrite(timer, 1400000, true);                                           // Trigger every 6 seconds
      timerAlarmEnable(timer);
    }
    Button_cnt++;
    lastInterruptTime = currentTime;
  }
}

void IRAM_ATTR increasePress() {                                                            // previous button press handler
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > 20) {
    Serial.println("increase volume button");
    increaseVol = true;                                                                    // setting flag to complete action
    lastInterruptTime = currentTime;

  }
}

void IRAM_ATTR decreasePress() {                                                            // previous button press handler
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > 20) {
    Serial.println("decrease volume button");
    decreaseVol = true;                                                                   // setting flag to complete action
    lastInterruptTime = currentTime;

  }
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Client Connected!");
    }
    void onDisconnect(BLEServer* pServer) {
      Serial.println("Client Disconnected!");
      // Restart advertising after disconnection
      pServer->startAdvertising();
      startTime = millis();                                                               // Reset the timeout
      audioPlayer(1007);
    }
};

class SecurityCallback : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() {
      return 000000;
    }
    void onPassKeyNotify(uint32_t pass_key) {}
    bool onConfirmPIN(uint32_t pass_key) {
      vTaskDelay(2000);
      return true;
    }
    bool onSecurityRequest() {
      return true;
    }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
      if (cmpl.success) {
        Serial.println("   - SecurityCallback - Authentication Success");
        Connected = true;
        audioPlayer(1005);                                                              //Bluetooth Connected audio
      } else {
        Serial.println("   - SecurityCallback - Authentication Failure*");
        Connected = false;
        //pServer->removePeerDevice(pServer->getConnId(), true);
      }
      BLEDevice::startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0)
      {
        Serial.println("## Received Value: " + String(rxValue.c_str()));
        for (int i = 0; i < rxValue.length(); i++)
        {
          Recieved[i] = rxValue[i];
        }
        Parameter_flag = true;
      }
    }
};

void ble_init_param_set_fun()
{
  BLEDevice::init("StaticPaymentSoundBox"); // Give it a name
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("WELCOME ");
  bleSecurity();
}

void bleSecurity() {
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSKEY;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void setup() {
  pinMode(0, INPUT);
  digitalWrite(0, LOW);
  pinMode(20, INPUT_PULLDOWN);
  pinMode(21, INPUT_PULLDOWN);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  pinMode(24, INPUT_PULLDOWN);
  pinMode(28, INPUT_PULLDOWN);
  pinMode(29, INPUT_PULLDOWN);
  pinMode(30, INPUT_PULLDOWN);
  pinMode(31, INPUT_PULLDOWN);
  pinMode(37, INPUT_PULLDOWN);
  pinMode(38, INPUT_PULLDOWN);

  pref.begin("data", false);

  // Set microSD Card CS as OUTPUT and set HIGH
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode( TFT_CS , OUTPUT);
  digitalWrite( TFT_CS , HIGH);
  pinMode( TOUCH_CS , OUTPUT);
  digitalWrite( TOUCH_CS , HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(increaseVol_PIN, INPUT_PULLUP);
  pinMode(decreaseVol_PIN, INPUT_PULLUP);

  attachInterrupt(BUTTON_PIN, buttonpress, FALLING);                                     // attaching interrupt for button press

  Serial.println("## CODE IS STARTED NOW.............................");

  pinMode(batteryPin, INPUT);                                                            // Set the battery pin as input

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);                                             // UART for cellular
  Serial.println("## CODE IS STARTED NOW.............................");

  /**************************** // Initialize the SPIFFS ********************************/
  // Initialize SPIFFS
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  Serial.println("SPIFFS initialized successfully.");

  /**************************** // Initialize the SD CARD ******************************/
  // Initialize SPI bus for microSD Card
  hspi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);                                             // initializing spi
  if (!SD.begin(SD_CS, hspi)) {
    Serial.println("Error accessing microSD card!");
    while(1);
    //return;
  }
  Serial.println("SD card initialized.");

  /**************************** // Initialize the I2S **********************************/
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);                                               // Setup I2S
  audio.setVolume(currentVolume);                                                             // Set Volume
  audioPlayer(1001);

  /**************************** // Initialize the TFT screen  **************************/

  Serial.println("Initializing TFT screen...");
  tft.init();
  tft.setRotation(0);
  //touch_calibrate();
  Serial.println("TFT screen initialized.");
  welcomePage();                                                                             // WITHOUT NETWORK AND BATTERY SYMBOL
  //audioPlayer(1002);                                                                       // Embel audio
  audioPlayer(1015);                                                                         // Vishwaguru audio
  Serial.println("Welcome page displayed.");

  /**************************** // BLE Connection  ************************************/
  ble_init_param_set_fun();                                                                  // BLE initialization
  timer = timerBegin(0, 80, true);                                                           // Timer 0, prescaler 80, counting up
  timerAttachInterrupt(timer, &onTimer, true);                                               // Attach the interrupt handler
  timerAlarmWrite(timer, 120000000, true);                                                   // Set the timer alarm to trigger every 1 second (1,000,000 microseconds)
  timerAlarmEnable(timer);                                                                   // Start the timer
  Serial.println("Waiting for Connection");

  tft.fillScreen(TFT_WHITE);
  bleconnectivityPage();

  while (!(Connected) && !(Timeout_flag))
  {
    //Serial.println("Connection");
    vTaskDelay(10 / portTICK_PERIOD_MS);                                                      // Wait for 10ms and yield control
  }
  if (Connected)
  {
    tft.drawString(" CONNECTED ", 30, 270);
    delay(2500);
    audioPlayer(1006);                                                                        // Bluetooth Configuration audio
  }

  Serial.println("Connection-waiting for 2nd authentication password");
  while (!(Parameter_flag) && !(Timeout_flag))
  {
    String response = "\n Enter Ur password to set UPI details";
    Serial.println("## Sent data  is : " + response);
    pCharacteristic->setValue(response.c_str());
    pCharacteristic->notify();
    vTaskDelay(2000 / portTICK_PERIOD_MS);                                                   // Wait for 10ms and yield control
  }
  delay(100);
  Parameter_flag = false;
  while (!(Password_flag) && !(Timeout_flag))
  {
    passwordVerification();
    vTaskDelay(2000 / portTICK_PERIOD_MS);                                                   // Wait for 2000ms and yield control
  }
  delay(100);
  readUpidetails();
  Serial.println("Showing current user");                                                   // showing previous user details
  String user = "\n Current User: ";
  Serial.println("## Sent data  is : " + user);
  pCharacteristic->setValue(user.c_str());
  pCharacteristic->notify();
  delay(10);
  user = userName;
  Serial.println("## Sent data  is : " + user);
  pCharacteristic->setValue(user.c_str());
  pCharacteristic->notify();
  delay(10);
  user = "\n UPI ID: ";
  Serial.println("## Sent data  is : " + user);
  pCharacteristic->setValue(user.c_str());
  pCharacteristic->notify();
  delay(10);
  user = upiID;
  Serial.println("## Sent data  is : " + user);
  pCharacteristic->setValue(user.c_str());
  pCharacteristic->notify();
  delay(10);

  String response = "\n Do you want change the user? (yes/no)";                              // skipping option for setting user
  Serial.println("## Sent data  is : " + response);
  pCharacteristic->setValue(response.c_str());
  pCharacteristic->notify();
  delay(10);
  Parameter_flag = false;
  while (!(Parameter_flag) && !(Timeout_flag))
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);                                                     // Wait for 10ms and yield control
  }
  userResponse();
  Parameter_flag = false;

  if (para_setting) {
    Serial.println("Connection-waiting for responce1");
    String response1 = "\n Enter Ur Bank ID and bank account number with this fomat: bank_name,bank_accountno";
    Serial.println("## Sent data  is : " + response1);
    pCharacteristic->setValue(response1.c_str());
    pCharacteristic->notify();
    Parameter_flag = false;
    while (!(Parameter_flag) && !(Timeout_flag))
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);                                                  // Wait for 10ms and yield control
    }
    getAcc_details();
    delay(100);
    String response2 = "\n Enter UPI details with this format only: upi_id,user name,currency";
    Serial.println("## Sent data  is : " + response2);
    pCharacteristic->setValue(response2.c_str());
    pCharacteristic->notify();
    Parameter_flag = false;
    Serial.println("Connection-waiting for responce2");
    while (!(Parameter_flag) && !(Timeout_flag))
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);                                                  // Wait for 10ms and yield control
    }
    getUpi_details();
    delay(100);
    String response4 = "\n Enter APN details with this format only: APN_name";
    Serial.println("## Sent data  is : " + response4);
    pCharacteristic->setValue(response4.c_str());
    pCharacteristic->notify();
    Parameter_flag = false;
    Serial.println("Connection-waiting for responce4");
    while (!(Parameter_flag) && !(Timeout_flag))
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);                                                  // Wait for 10ms and yield control
    }
    getApn_details();
    delay(100);
  }

  Serial.println("** BLE deinit function ");
  BLEDevice::deinit();                                                                    // Bluetooth deinitialization
  if (Connected == true) {
    tft.drawString(" DISCONNECTED ", 20, 270);
  }
  timerAlarmDisable(timer);
  Serial.println("Timer stopped.");

  Parameter_flag = false;                                                                // Resetting flags
  Timeout_flag = false;

  if (Password_flag)
  {
    writeUpidetails();                                                                  // Saving user details in spiff
    Connected = false;
  }

  readUpidetails();                                                                    // Reading user details from spiff
  /*****************************************************************************/
  getMsgcnt();                                                                         // getting message count
  welcomePage();//WITHOUT NETWORK AND BATTERY SYMBOL
  battery_measurement();

  Serial.println("Initializing modem...");

  Serial.println("Powering on the cellular module");
  digitalWrite(pwr_key, HIGH);                                                         // powering on the cellular using pwr key
  delay(2500);
  digitalWrite(pwr_key, LOW);
  //delay(90000);
  delay(1000);
  Serial.println("Power on sequence completed");

  Serial.println("Waiting for network...");
  char data1[50] = {0};
  sprintf(data1, "AT+CGDCONT=1,\"IP\",\"%s\"", apnname.c_str());                       // APN setting
  Serial.println(apnname);
  Serial.println(data1);
  Serial2.println(data1);
  updateSerial();

  if (!modem.waitForNetwork(30000L)) {
    delay(10000);
    //return;
  }
  if (modem.isNetworkConnected()) {
    networkAvailable = true;
    displayNetworkBars(networkAvailable);
    audioPlayer(1003);
    Serial.println("Network connected");
  }
  Serial2.println("AT+CMGF=1");
  updateSerial();
  Serial2.println("AT+QURCCFG=\"URCPORT\", \"UART1\"");                                       // For UART communication
  updateSerial();
  Serial2.println("AT+CNMI=1,2,0,0,0");
  updateSerial();
  Serial2.println("AT+QSIMDET=1,0");                                                          // SIM detection, if SIM removed- +CPIN: NOT READY, SIM inserted- READY
  Serial2.println("AT+QSCLK=1");                                                              // enabling sleep mode
  updateSerial();
  Serial2.println("AT+QGPS?");                                                                // GPS STATUS
  updateSerial();
  Serial2.println("AT+CTZU=3");                                                               // Timing zone and reporting
  updateSerial();
  Serial2.println("AT+CCLK?");                                                                // Getting time
  updateSerial();
  Serial.println("Modem configured for SMS.");

  currentPage = Page::defaultQRCode;                                                          // Switch to the QR code page
  generateAndWriteQRCode_default();                                                           // Generate and write the QR code based on the amount
  defaultQrpage();                                                                            // Draw the QR code page on the screen
  battery_measurement();
  displayNetworkBars(networkAvailable);
  Serial.println("Static QR  page displayed.");
  attachInterrupt(36, increasePress, FALLING);                               // Interrupt for increase volume button
  attachInterrupt(39, decreasePress, FALLING);
}

void loop() {
  if (networkAvailable == false) {                                                           // checking for network if not connected
    if (!modem.waitForNetwork(2000L)) {
      //return;
    }
    if (modem.isNetworkConnected()) {
      networkAvailable = true;
      displayNetworkBars(networkAvailable);
      audioPlayer(1003);
      Serial.println("Network connected");
    }
  }
  counter++;
  /*  Serial.print("counter: ");
    Serial.println(counter);*/

  updateSerial();

  /**************************** // Handle touch input on the TFT screen ************************************/
  /* uint16_t t_x = 0, t_y = 0;
    bool pressed = tft.getTouch(&t_x, &t_y);

    if(pressed)
    {
      // Serial.println("Handling touch input on the TFT screen...");

       if (currentPage == Page::Keypad)
       {
         Serial.println("Handling Keypad touch");
         handleKeypadTouch(t_x, t_y, pressed);
         Serial.println("Handled Keypad touch");
       }
       else if (currentPage == Page::defaultQRCode)
       {
         Serial.println("Handling defaultQRCode touch");
         handledefaultQRCodeTouch(t_x, t_y, pressed);
         Serial.println("Handled defaultQRCode touch");
       }
    }
    else
    {
     if (currentPage == Page::QRCode)
       {
         Serial.println("Handling QR Code touch");
         handleQRCode();
         Serial.println("Handled QR Code touch");
       }
    }*/

  /******************************Volume Adjustment*****************************************************/
  // Check if the increase volume button is pressed
  //  if (digitalRead(increaseVol_PIN) == LOW) {                                                  // checking to increase volume
  //    increaseVolume();
  //    delay(10);
  //  }
  //
  //  // Check if the decrease volume button is pressed
  //  if (digitalRead(decreaseVol_PIN) == LOW) {                                                 // checking to decrease volume
  //    decreaseVolume();
  //    delay(10);
  //  }

  if (increaseVol) {
    increaseVolume();
    delay(10);
    increaseVol = false;
  }
  if (decreaseVol) {
    decreaseVolume();
    delay(10);
    decreaseVol = false;
  }

  /****************************  // Check the state of the button ************************************/
  if ((Button_flag != false) && (Serial2.available() <= 0))                                 // previous transaction button action
  {
    Button_flag = false;
    if (Button_cnt < 6) {
      readMsgs();
    }
    Button_cnt = 0;
  }

  if ((Button_flag != true) && (Serial2.available() <= 0))                                 // network status
  {
    if ((counter % 400) == 0)
    {
      networkStatus();
    }
  }

  /***************************** Battery monitoring ***************************************/
  if ((millis() - bat_cal) > 120000) {
    detachInterrupt(36);
    detachInterrupt(39);
    if (Serial2.available() < 0) {                                                       // Battery calculation
      battery_measurement();
      bat_cal = millis();
    }
    attachInterrupt(36, increasePress, FALLING);
    attachInterrupt(39, decreasePress, FALLING);
  }

  if (((100 < counter) || (batperc < lowThreshold)) && ((Button_flag != true) && (Serial2.available() <= 0)))
  {
    counter = 0;
    battery_measurement();
    /**************************** // Determine the current battery status and print it ************************************/
    //Serial.println("Determining current battery status...");
    if (batperc < lowThreshold)
    {
      if (lastBatteryStatus != "Battery is Low")
      {
        tft.fillScreen(TFT_WHITE);                                                        // low battery page
        tft.setFreeFont(&FreeSansBold12pt7b);
        tft.setTextColor(TFT_RED);
        tft.drawString("LOW BATTERY", 30, 60);
        tft.setFreeFont(&FreeSans18pt7b);
        tft.setTextColor(TFT_BLUE);
        tft.drawString(" PLEASE ", 40, 130);
        tft.drawString(" CONNECT ", 26, 180);
        tft.drawString(" CHARGER ", 26, 230);

        audioPlayer(1009);                                                                // Play low battery audio
        Serial.println("Playing low battery audio");
        Serial.println("Battery is Low");
        lastBatteryStatus = "Battery is Low";
        battery_measurement();
        Serial.println(batperc);
        currentPage = Page::defaultQRCode;                                               // Switch to the QR code page
        generateAndWriteQRCode_default();                                                // Generate and write the QR code based on the amount
        defaultQrpage();                                                                 // Draw the QR code page on the screen
        battery_measurement();
        displayNetworkBars(networkAvailable);
        Serial.println("Static QR  page displayed.");
        delay(100);
      }
    }
    else if (batperc >= fullThreshold)                                     // checking full battery
    {
      if (lastBatteryStatus != "Battery is Full")
      {
        audioPlayer(1008); // Play full battery audio
        Serial.println("Playing full battery audio");
        Serial.println("Battery is Full");
        Serial.println(batperc);
        lastBatteryStatus = "Battery is Full";
      }
    }
    else
    {
      if (lastBatteryStatus != "")
      {
        Serial.println("Battery is in medium range");
        lastBatteryStatus = "";
      }
    }
  }
}

/**************************** managing cellular data **************************************************************************/
void updateSerial() {
  int cmtstart = 0, cmtlast = 0, cmtmid = 0;                                             // to check multiple messages

  while (Serial2.available())
  {
    String receivedString = Serial2.readString();
    response = receivedString;
    Serial.println(response);

    cmtstart = response.indexOf("+CMT: ");
    cmtmid = response.indexOf("+CMT: ", cmtstart + 1);
    cmtlast = response.lastIndexOf("+CMT: ");

    const int length = response.length();                                                 // get the length of the text
    lower = (char*)malloc(length + 1);                                                    // allocate 'length' bytes + 1 (for null terminator) and cast to char*
    lower[length] = 0;                                                                    // set the last byte to a null terminator

    for (int i = 0; i < length; i++) {
      lower[i] = tolower(response[i]);
    }

    if ((strstr(lower, "upi")) || (strstr(lower, "ref"))) {                               // checking for upi messages
      if ((strstr(lower, bankName.c_str()))) {
        if (cmtstart == cmtlast) {
          messageBuffer[index1] = response;
          Serial.print("Message stored at: "); Serial.println(index1);
          index1++;
        }
        else {
          messageBuffer[index1] = response;
          Serial.print("Message stored at: "); Serial.println(index1);
          index1++;

          while (cmtmid != cmtlast) {                                                     // storing multiple messages in buffer
            messageBuffer[index1] = response.substring(cmtmid);
            Serial.print("Message stored at: "); Serial.println(index1);
            index1++;
            cmtmid = response.indexOf("+cmt: ", cmtmid + 1);
          }
          messageBuffer[index1] = response.substring(cmtlast);
          Serial.print("Message stored at: "); Serial.println(index1);
          index1++;
        }
        messageBuffer[index1++] = "";
      }
    }

    if (messageBuffer[0] != "") {                                                        // checking for empty buffer
      String msg = messageBuffer[0];
      const int length1 = msg.length();                                                  // get the length of the text
      promsg = (char*)malloc(length1 + 1);                                               // allocate 'length' bytes + 1 (for null terminator) and cast to char*
      promsg[length1] = 0;                                                               // set the last byte to a null terminator

      for (int i = 0; i < length1; i++) {
        promsg[i] = tolower(msg[i]);
      }
      Serial.print("Whole Msg - ");
      Serial.println(promsg);
      Serial.println("\n");

      // Convert String to C-style stringString bankID_4=
      const char* bankIDCStr = bankID_4.c_str(); Serial.println("bankIDCStr: "); Serial.println(bankIDCStr);
      const char* bankNameCStr = bankName.c_str(); Serial.println("bankNameCStr: "); Serial.println(bankNameCStr);

      if ((strstr(lower, bankNameCStr)))                                                  // Check with bank
      {
        Serial.println(" BANK SMS ID IS VERIFIED");
        if ((strstr(lower,  bankIDCStr)))                                                 // Check with last 4 digit of bank account no with last 4 digit.
        {
          Serial.println(" BANK ACCOUNT IS VERIFIED");
          if ((strstr(lower, "credited")) || (strstr(lower, "debited")) || (strstr(lower, "sent")) || (strstr(lower, "received"))) //Check with credit/debit_keyword
          {
            Serial.println(" BANK AMOUNT CREDITED/DEBITED IS VERIFIED");
            if ((strstr(lower, "rs ")) || (strstr(lower, "rs.")) || (strstr(lower, "inr ")) || (strstr(lower, "rs:"))) //Check with currency_keyword
            {
              Serial.println(" BANK AMOUNT RS /RS./INR /RS:/ IS VERIFIED");

              // Check for specific keywords in the lower string
              if (strstr(lower, "rs "))                                                    // If "rs " is found in lower
              {
                CheckWordInString_rs(lower);                                              // Call the function to process the "rs " keyword
              }
              if (strstr(lower, "rs."))                                                   // If "rs." is found in lower
              {
                CheckWordInString_rsDot(lower);                                           // Call the function to process the "rs." keyword
              }
              if (strstr(lower, "rs:"))                                                   // If "rs:" is found in lower
              {
                CheckWordInString_rsCol(lower);                                          // Call the function to process the "rs:" keyword
              }
              if (strstr(lower, "inr "))                                                 // If "inr " is found in lower
              {
                CheckWordInString_inrDot(lower);                                         // Call the function to process the "inr." keyword
              }

              /* String str1 =final_amount;
                pref.putString("str1", final_amount);

                String  str2 ="";
                 Serial.println("Testing string1: " + str2);
                String str_test = pref.getString("str1", str2);
                Serial.println("Testing string2: " + str_test);*/

              /*   // Write the final amount to SPIFFS
                  writeFile(SPIFFS, amount_File, final_amount.c_str());
                  // Execute voice commands with the extracted amount
                  voice_commands(final_amount);
                 // Read the amount back from SPIFFS and print it (for verification)
                  readFile(SPIFFS, amount_File);*/
              saveMsgs();

              if (countdownTime > 0) {
                voice_commands(final_amount);
                Serial.println("Done with Voice commands.");
              }
              else {
                tft.fillScreen(TFT_WHITE);
                tft.setFreeFont(&FreeSansBold12pt7b);                                    //(&FreeSans18pt7b);
                tft.setTextColor(DISP_TCOLOR, TFT_WHITE);                               // Set text color to blue and background color to white
                //Payment received successfully for -amount
                tft.drawString(" PAYMENT ", 53, 70);                                     // Display amount starting from left
                tft.drawString(" RECEIVED ", 48, 120);                                   // Display amount starting from left
                tft.drawString("SUCCESSFULLY !! ", 10, 170);                             // Display amount starting from leftsuccessful
                tft.drawString(currency + " " + String(final_amount), 50, 210);
                voice_commands(final_amount);
                Serial.println("Done with Voice commands.");

                currentPage = Page::defaultQRCode;                                       // Switch to the QR code page
                generateAndWriteQRCode_default();                                        // Generate and write the QR code based on the amount
                defaultQrpage();                                                         // Draw the QR code page on the screen
                battery_measurement();
                displayNetworkBars(networkAvailable);
                Serial.println("Static QR  page displayed.");
              }
              if (messageBuffer[1] == "") {
                messageBuffer[0] = "";
                memset(messageBuffer, 0, sizeof(messageBuffer));
                index1 = 0;
              }
            }
          }
        }
      }
    }
    else if ((strstr(lower, "+cmti:  "))) {
      String result = (String)res;
      int firstComma = result .indexOf("+cmti: "); Serial.println("firstComma"); Serial.println(firstComma);
      int secondComma = result .indexOf(','); Serial.println("secondComma"); Serial.println(secondComma);
      // Extract the csq from the received string
      String memory = result .substring(firstComma, secondComma);
      String strindex = result.substring(secondComma);
      Serial2.println("AT+CMGR=strindex");
    }

    else if ((strstr(lower, "+creg:")))                                                 // Check with "+CREG"
    {
      if ((strstr(lower, " 0,0"))) {
        networkAvailable = false;
        displayNetworkBars(networkAvailable);                                          // updating network status on display
        audioPlayer(1004);
        Serial.println("*****************No Network is availaible ************************");
      }
      else {
        Serial.println("*****************Ur device is in network. ************************");
        networkAvailable = true;
        displayNetworkBars(networkAvailable);                                         // updating network status on display
      }
    }
    else if ((strstr(lower, "+cpin:")))                                               // Check with "+CPIN:READY"
    {
      if ((strstr(lower, "not")))
      {
        Serial.println("*****************SIM is not inserted ,Please check it. ************************");
        audioPlayer(1013);
      }
      else
      {
        Serial.println("*****************SIM is READY************************");
      }
    }

    for (int i = 0; i < BUFFER_SIZE - 1; i++) {                                       // Resetting buffer after processing message
      if (messageBuffer[i] == "") {
        index1 = i;
        break;
      }
      messageBuffer[i] = messageBuffer[i + 1];                                        // Rearranging message positions
      messageBuffer[i + 1] = "";
      if (index1 > 0) {
        index1--;
      }
    }

    // Clear variables for next iteration
    res = "";
    free(lower);                                                                      // freeing the memory of heap
    lower = NULL;
    promsg = "";
    msg = "";
  }
}
/**************************** updateSerial() function end *********************************************************************/

/**************************** void CheckWordInString_rs(char* str)  ***********************************************************/
void CheckWordInString_rs(char* str)
{
  int i = 0;
  while (lower[i] != 'r' || lower[i + 1] != 's' || lower[i + 2] != ' ') i++;         // Find the position of the first occurrence of "rs"

  String result = (String)lower;                                                     // Convert the lower string to a String object
  result = result.substring(i + 3);                                                  // Get the substring after "rs"
  Serial.print("RESULT FROM SUBSTRING IS :");
  Serial.println(result);

  i = 0;
  while (result[i] != ' ') i++;                                                     // Find the position of the first space after "rs"

  final_amount = result.substring(0, i);                                            // Extract the substring from the start to the first space (the amount)
  Serial.print("Final Amount - ");
  Serial.println(final_amount);
}
/**************************** CheckWordInString_rs(char* str) function end ****************************************************/

/**************************** Finding "rs" or "rs.": The functions search for the position of "rs" or "rs." in the lower string using a while loop. They increment i until the target substring is found. ************************************/
void CheckWordInString_rsDot(char* str)
{
  int i = 0;
  while (lower[i] != 'r' || lower[i + 1] != 's' || lower[i + 2] != '.') i++;       // Find the position of the first occurrence of "rs."

  String result = (String)lower;                                                   // Convert the lower string to a String object
  result = result.substring(i + 3);                                                // Get the substring after "rs."

  i = 0;
  while (result[i] != ' ') i++;                                                    // Find the position of the first space after "rs."

  final_amount = result.substring(0, i);                                           // Extract the substring from the start to the first space (the amount)
  Serial.print("Final Amount - ");
  Serial.println(final_amount);
}
/**************************** CheckWordInString_rsDot(char* str) function end *************************************************/

/**************************** checking rs: in message string ******************************************************************/
void CheckWordInString_rsCol(char* str)
{
  int i = 0;
  while (lower[i] != 'r' || lower[i + 1] != 's' || lower[i + 2] != ':') i++;      // Find the position of the first occurrence of "rs."

  String result = (String)lower;                                                  // Convert the lower string to a String object
  result = result.substring(i + 3);                                               // Get the substring after "rs."

  i = 0;
  while (result[i] != ' ') i++;                                                   // Find the position of the first space after "rs."

  final_amount = result.substring(0, i);                                          // Extract the substring from the start to the first space (the amount)
  Serial.print("Final Amount - ");
  Serial.println(final_amount);
}
/************************** CheckWordInString_rsCol(char* str) function end ***************************************************/

/************************** checking inr in message string ********************************************************************/
void CheckWordInString_inrDot(char* str)
{
  int i = 0;
  while (lower[i] != 'i' || lower[i + 1] != 'n' || lower[i + 2] != 'r' || lower[i + 3] != ' ') i++;  // Find the position of the first occurrence of "rs."

  String result = (String)lower;                                                  // Convert the lower string to a String object
  result = result.substring(i + 4);                                               // Get the substring after "inr "

  i = 0;
  // Find the position of the first space after "inr "
  while (result[i] != ' ') i++;

  final_amount = result.substring(0, i);                                          // Extract the substring from the start to the first space (the amount)
  Serial.print("Final Amount - ");
  Serial.println(final_amount);
}
/**************************** CheckWordInString_inrDot(char* str) function end ************************************************/

/**************************** getting bank account details ********************************************************************/
void getAcc_details(void)
{
  String received ;
  received = String(Recieved);
  Serial.println("## received is : " + received);
  if (received.length() > 0)
  {
    int firstComma = received.indexOf(',');                                       // Find the positions of the first and second commas in the received string
    // Check if both commas were found and their positions are valid
    if (firstComma > 0)
    {
      bankName = received.substring(0, firstComma);                               // Extract the bank name andacc num from the received string
      bankName.toLowerCase();
      bankID = received.substring(firstComma + 1);
      bankID_4 =  bankID.substring( bankID.length() - 4);
      Serial.println("bankName: " + bankName + ", bankID: " + bankID + ", bankID_4 : " + bankID_4 );
    }
  }
  memset(Recieved, 0, sizeof(Recieved));                                         // Resetting buffer
}
/**************************** getAcc_details(void) function end ***************************************************************/

/**************************** getting upi details *****************************************************************************/
void getUpi_details(void)
{
  String received ;
  received = String(Recieved);
  Serial.println("## received is : " + received);
  if (received.length() > 0)
  {
    int firstComma = received.indexOf(',');                                       // Find the positions of the first and second commas in the received string
    int secondComma = received.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma)                               // Check if both commas were found and their positions are valid
    {
      upiID = received.substring(0, firstComma);                                  // Extract the UPI ID, user name, and currency from the received string
      userName = received.substring(firstComma + 1, secondComma);
      currency = received.substring(secondComma + 1);

      Serial.println("UPI ID: " + upiID + ", Name: " + userName + ", Currency: " + currency);
    }
  }
  memset(Recieved, 0, sizeof(Recieved));                                          // Resetting buffer
}
/************************** getUpi_details(void) function end *****************************************************************/

/************************** getting apn details *******************************************************************************/
void getApn_details(void)
{
  String received ;
  received = String(Recieved);
  Serial.println("## received is : " + received);
  if (received.length() > 0)
  {
    apnname = received.substring(0);
  }
  memset(Recieved, 0, sizeof(Recieved));                                                 // Resetting buffer
}
/************************* getApn_details(void) function end ******************************************************************/

/************************* getting device configuration ***********************************************************************/
void getConfig_details(void)
{
  String received ;
  received = String(Recieved);
  Serial.println("## received is : " + received);
  if (received.length() > 0)
  {
    deviceConfig = received.substring(0);
  }
  memset(Recieved, 0, sizeof(Recieved));                                                // Resetting buffer
}
/************************* getConfig_details(void) function end ***************************************************************/

/************************* password verification for entering user details ****************************************************/
void passwordVerification(void)
{
  String received ;
  received = String(Recieved);
  Serial.println("## received is : " + received);
  if (received.length() > 0)
  {
    String get = received.substring(0);
    if (get == PASSWORD)
    {
      String response1 = "\n Password Verified Succeessfully";
      Serial.println("## Sent data  is : " + response1);
      pCharacteristic->setValue(response1.c_str());
      pCharacteristic->notify();
      Parameter_flag = false;
      Password_flag = true;
    }
    else
    {
      String response1 = "\n U have eneterd wrong password.Please enter correct password";
      Serial.println("## Sent data  is : " + response1);
      pCharacteristic->setValue(response1.c_str());
      pCharacteristic->notify();
      Parameter_flag = false;
    }
  }
  memset(Recieved, 0, sizeof(Recieved));                                               // Resetting buffer
}
/************************* passwordVerification(void) function end ************************************************************/

/************************* checking network ***********************************************************************************/
void networkStatus(void)
{
  Serial2.println("AT+CREG?");
  Serial.println("Get signal quality");
}
/************************** networkStatus(void) function end *****************************************************************/

/************************** writing upi details in spiff *********************************************************************/
void writeUpidetails() {
  File file1 = SPIFFS.open("/Upistring1.txt", FILE_WRITE);
  if (!file1) {
    Serial.println("Failed to open file 1 for writing");
    return;
  }
  file1.println(bankName);
  file1.close();

  File file2 = SPIFFS.open("/Upistring2.txt", FILE_WRITE);
  if (!file2) {
    Serial.println("Failed to open file 2 for writing");
    return;
  }
  file2.println(bankID);
  file2.close();

  File file3 = SPIFFS.open("/Upistring3.txt", FILE_WRITE);
  if (!file3) {
    Serial.println("Failed to open file 3 for writing");
    return;
  }
  file3.println(upiID);
  file3.close();
  File file4 = SPIFFS.open("/Upistring4.txt", FILE_WRITE);
  if (!file4) {
    Serial.println("Failed to open file 4 for writing");
    return;
  }
  file4.println(userName);
  file4.close();

  File file5 = SPIFFS.open("/Upistring5.txt", FILE_WRITE);
  if (!file5) {
    Serial.println("Failed to open file 5 for writing");
    return;
  }
  file5.println(currency);
  file5.close();

  File file6 = SPIFFS.open("/Upistring6.txt", FILE_WRITE);
  if (!file6) {
    Serial.println("Failed to open file 6 for writing");
    return;
  }
  file6.println(apnname);
  file6.close();

  Serial.println("WRITE UPI DETAILS TO SSPIFFS");

  /*   Serial.println(bankName);
     Serial.println(bankID);
     bankID_4 =  bankID.substring(bankID.length() - 4);
     Serial.println(bankID_4);
     Serial.println( upiID);
     Serial.println(userName);
     Serial.println(currency);
     Serial.println(apnname);

     Serial.println("write bankName len :");   Serial.println(bankName.length());
     Serial.println("write bankID len :");   Serial.println(bankID.length());
     Serial.println("write bankID_4 len :"); Serial.println(bankID_4.length());
     Serial.println("write upiID len :");   Serial.println(upiID.length());
     Serial.println("write userName len :");   Serial.println(userName.length());
     Serial.println("write currency len :"); Serial.println(currency.length());
     Serial.println("write apnname len :"); Serial.println(apnname.length());
  */
}
/************************* writeUpidetails() function end *********************************************************************/

/************************* reading upi details from spiff *********************************************************************/
void readUpidetails() {
  File file1 = SPIFFS.open("/Upistring1.txt", FILE_READ);
  if (!file1) {
    Serial.println("Failed to open file 1 for reading");
    return;
  }
  String input1 = file1.readStringUntil('\0');
  file1.close();

  File file2 = SPIFFS.open("/Upistring2.txt", FILE_READ);
  if (!file2) {
    Serial.println("Failed to open file 2 for reading");
    return;
  }

  String input2 = file2.readStringUntil('\0');
  file2.close();

  File file3 = SPIFFS.open("/Upistring3.txt", FILE_READ);
  if (!file3) {
    Serial.println("Failed to open file 3 for reading");
    return;
  }
  String input3 = file3.readStringUntil('\0');
  file3.close();

  File file4 = SPIFFS.open("/Upistring4.txt", FILE_READ);
  if (!file4) {
    Serial.println("Failed to open file 4 for reading");
    return;
  }
  String input4 = file4.readStringUntil('\0');
  file4.close();

  File file5 = SPIFFS.open("/Upistring5.txt", FILE_READ);
  if (!file5) {
    Serial.println("Failed to open file 5 for reading");
    return;
  }
  String input5 = file5.readStringUntil('\0');
  file5.close();

  File file6 = SPIFFS.open("/Upistring6.txt", FILE_READ);
  if (!file6) {
    Serial.println("Failed to open file 6 for reading");
    return;
  }
  String input6 = file6.readStringUntil('\0');
  file6.close();

  // Now you can use these strings as needed
  Serial.println("READ UPI DETAILS FROM SSPIFFS");

  /*  String get_id =  bankID.substring(0, bankID.length() - 2);
    Serial.println("get_id is ");  Serial.println(get_id);Serial.println("get_id len :");   Serial.println(get_id.length());
    bankID_4 =  get_id.substring(get_id.length() - 4);  */

  bankName =  input1.substring(0, input1.length() - 2);
  bankID =  input2.substring(0, input2.length() - 2);
  bankID_4 =  bankID.substring(bankID.length() - 4);
  upiID =  input3.substring(0, input3.length() - 2);
  userName =  input4.substring(0, input4.length() - 2);
  currency =  input5.substring(0, input5.length() - 2);
  apnname =  input6.substring(0, input6.length() - 2);

  /*  Serial.println(bankName);
    Serial.println(bankID);
    Serial.println(bankID_4);
    Serial.println(upiID);
    Serial.println(userName);
    Serial.println(currency);

    Serial.println("READ bankName len :");   Serial.println(bankName.length());
    Serial.println("READ bankID len :");   Serial.println(bankID.length());
    Serial.println("READ bankID_4 len :"); Serial.println(bankID_4.length());
    Serial.println("READ upiID len :");   Serial.println(upiID.length());
    Serial.println("READ userName len :");   Serial.println(userName.length());
    Serial.println("READ currency len :"); Serial.println(currency.length());
  */
}
/************************** readUpidetails() function end *********************************************************************/

/************************** welcome page **************************************************************************************/
void welcomePage(void)
{
  tft.fillScreen(TFT_WHITE);                                                     // setting welcome page
  tft.setFreeFont(&FreeSansBold12pt7b);                                          // (&FreeSans18pt7b);
  tft.setTextColor(DISP_TCOLOR, TFT_WHITE);                                      // Set text color to blue and background color to white
  tft.drawString(" WELCOME ", 53, 60);
  tft.drawString(" TO ", 98, 100);
  //tft.drawString(" EMBEL ", 73, 140);
  tft.drawString(" VISHWAGURU ", 27, 140);                                       // tft.drawString(" EMBEL ", 73, 140);
  tft.drawString(" PAYMENT ", 53, 180);
  tft.drawString(" SOUND BOX ", 37, 220);
}
/************************** welcomePage(void) function end ********************************************************************/

/************************** Bluetooth connectivity page ***********************************************************************/
void bleconnectivityPage(void)
{
  //int thickness = 3;                                                          // for small
  int thickness = 5;                                                            // for big

  //for (int i = 0; i < thickness; i++) {

  //FOR SMALL

  /* tft.drawLine(155 + i, 13, 160 + i, 20, TFT_BLACK);
    tft.drawLine(155 + i, 28, 160 + i, 20, TFT_BLACK);
    tft.drawLine(160 + i, 10, 160 + i, 30, TFT_BLACK);
    tft.drawLine(160 + i, 10, 165 + i, 15, TFT_BLACK);
    tft.drawLine(160 + i, 20, 165 + i, 15, TFT_BLACK);
    tft.drawLine(160 + i, 20, 165 + i, 25, TFT_BLACK);
    tft.drawLine(160 + i, 30, 165 + i, 25, TFT_BLACK);*/


  //for full screen

  /* tft.drawLine(40 + i, 107, 100 + i, 150, TFT_BLACK);
    tft.drawLine(40 + i, 193, 100 + i, 150, TFT_BLACK);
    tft.drawLine(100 + i, 40, 100 + i, 260, TFT_BLACK);
    tft.drawLine(100 + i, 40, 200 + i, 105, TFT_BLACK);
    tft.drawLine(100 + i, 150, 200 + i, 105, TFT_BLACK);
    tft.drawLine(100 + i, 150, 200 + i, 195, TFT_BLACK);
    tft.drawLine(100 + i, 260, 200 + i, 195, TFT_BLACK);*/

  //medium range
  /*tft.drawLine(60 + i, 98, 100 + i, 140, TFT_BLACK);   //40 - 60,  200 - 180, y1=40-60, y2 = 260-240
    tft.drawLine(60 + i, 182, 100 + i, 140, TFT_BLACK);   //        100
    tft.drawLine(100 + i, 70, 100 + i, 210, TFT_BLACK);   //150-130, 105- 85,  195- 175
    tft.drawLine(100 + i, 70, 170 + i, 95, TFT_BLACK);    //107- 88, 193- 172
    tft.drawLine(100 + i, 140, 170 + i, 95, TFT_BLACK);
    tft.drawLine(100 + i, 140, 170 + i, 175, TFT_BLACK);
    tft.drawLine(100 + i, 210, 170 + i, 175, TFT_BLACK);*/
  //}
  // for small symbol
  //tft.fillTriangle(160, 10, 160, 15, 160, 20, TFT_BLUE);                            // Top triangle
  //tft.fillTriangle(160, 30, 160, 25, 160, 20, TFT_BLUE);                            // Bottom triangle

  // for big symbol
  //tft.fillTriangle(104, 43, 200, 104, 104, 149, TFT_BLUE);                          // Top triangle
  //tft.fillTriangle(104, 257, 200, 194, 104, 151, TFT_BLUE);                         // Bottom triangle

  //medium range
  //tft.fillTriangle(104, 73, 104, 139, 169, 94, TFT_BLUE);                           // Top triangle
  //tft.fillTriangle(104, 141, 104, 209, 169, 174, TFT_BLUE);                         // Bottom triangle

  tft.fillCircle(120, 135, 100, TFT_BLUE);
  for (int i = 0; i < thickness; i++) {
    tft.drawLine(70 + i, 110, 100 + i, 140, TFT_WHITE);                                //40 - 60,  200 - 180, y1=40-60, y2 = 260-240
    tft.drawLine(70 + i, 167, 100 + i, 140, TFT_WHITE);                                //        100
    tft.drawLine(100 + i, 70, 100 + i, 210, TFT_WHITE);                                //150-130, 105- 85,  195- 175
    tft.drawLine(100 + i, 70, 170 + i, 95, TFT_WHITE);                                 //107- 88, 193- 172
    tft.drawLine(100 + i, 140, 170 + i, 95, TFT_WHITE);
    tft.drawLine(100 + i, 140, 170 + i, 175, TFT_WHITE);
    tft.drawLine(100 + i, 210, 170 + i, 175, TFT_WHITE);
  }
}
/************************ bleconnectivityPage(void) function end **************************************************************/

/************************ network bars based on network availability **********************************************************/
void displayNetworkBars(bool networkAvailable) {
  int numBars = networkAvailable ? 4 : 2;                                              // Display 4 bars for full signal, 2 bars for no network
  int barWidth = 3;                                                                    // Reduced width of each bar to create closer lines
  uint16_t color = TFT_BLACK;                                                          // Color of the bars
  // Draw network signal bars in the top left corner
  if (networkAvailable == true) {
    drawNetworkBars(177, 10, numBars, barWidth, color);                                // Position (10, 10), numBars, barWidth, color
  }
  else {
    drawNetworkBars(177, 20, numBars, barWidth, color);
  }
}
/************************* displayNetworkBars(bool networkAvailable) function end *********************************************/

/************************* Function to draw network bars ***********************************************************************/
void drawNetworkBars(int x, int y, int numBars, int barWidth, uint16_t color) {
  int barSpacing = 2;                                                                  // Reduced space between bars
  int maxHeight = 20;                                                                  // Maximum height of bars

  if (numBars == 2)
  {
    maxHeight = 10;
  }
  for (int i = 0; i < numBars; i++) {
    int barHeight = (i + 1) * (maxHeight / numBars);                                  // Calculate bar height for each bar

    // Draw each bar using fillRect for solid bars
    tft.fillRect(x + i * (barWidth + barSpacing), y + maxHeight - barHeight, barWidth, barHeight, color);
  }
}
/************************* drawNetworkBars(int x, int y, int numBars, int barWidth, uint16_t color) function end **************/

/************************* drawing battery status on display ******************************************************************/
void drawBatteryStatus(int x, int y, int width, int height, int batteryLevel) {
  // Define colors
  uint16_t batteryColor = TFT_BLACK;                                                  // Color for the battery status
  uint16_t borderColor = TFT_BLACK;                                                   // Color for the battery border
  // Define dimensions for the battery "bubble"
  int bubbleWidth = width / 8;                                                        // Width of the bubble, adjust as needed
  int bubbleHeight = height / 3;                                                      // Height of the bubble, adjust as needed

  // Draw battery border
  tft.drawRect(x, y, width, height, borderColor);

  // Draw and fill the small bubble on the right side
  tft.fillRect(x + width, y + (height - bubbleHeight) / 2, bubbleWidth, bubbleHeight, borderColor);

  // Calculate fill width manually based on battery level
  int fillWidth = (batteryLevel * (width - 4)) / 100;                                 // Scale battery level to fill width

  // Draw battery fill
  tft.fillRect(x + 2, y + 2, fillWidth, height - 4, batteryColor);
}
/*************** drawBatteryStatus(int x, int y, int width, int height, int batteryLevel) function end ************************/

/***************************** measuring battery percentage *******************************************************************/
void battery_measurement()
{
  /**************************** // Read the ADC value from the battery pin ************************************/
  //Serial.println("Reading ADC value from battery pin...");
  adc_power_acquire();
  ADCvalue = readStableADC(100);
  adc_power_release();
  //Serial.print("ADC Value: ");
  //Serial.println(ADCvalue);

  /****************************  // Convert ADC value to voltage ************************************/
  //Serial.println("Converting ADC value to voltage...");
  batvoltage = ((ADCvalue / float(adcMaxValue)) * referenceVoltage) + 0.2; // intentionally added 0.2v to compensate adc error
  //Serial.print("Battery Voltage: ");
  //Serial.print(batvoltage);
  //Serial.println(" V");

  /**************************** // Calculate the battery percentage ************************************/
  //Serial.println("Calculating battery percentage...");
  batperc = mapVoltageToPercentage(batvoltage);
  //Serial.print("Battery Percentage: ");
  //Serial.print(batperc);
  //Serial.println(" %");

  drawBatteryStatus(202, 10, 30, 20, batperc);
}
/************************** battery_measurement() function end ****************************************************************/

/********************************** averaging adc *****************************************************************************/
int readStableADC(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(batteryPin);                                                  // Reading adc value
    delay(2);
  }
  return sum / samples;
}
/*********************************** readStableADC(int samples) function end **************************************************/

/************************** saving message count ******************************************************************************/
void saveMsgscnt()
{
  pref.putInt("Msgcnt", Msg_cnt);
}
/************************** saveMsgscnt() function end ************************************************************************/

/************************** getting message count *****************************************************************************/
void getMsgcnt(void)
{
  int getValue = 0;
  Msg_cnt = pref.getInt("Msgcnt", getValue);
  Serial.println("GET VALUE OF Msg_cnt: "); Serial.println(Msg_cnt);
}
/************************** getMsgcnt(void) function end **********************************************************************/

/************************** saving messages in spiff **************************************************************************/
void saveMsgs()
{
  Msg_cnt++;
  Serial.println("Msg_cnt: "); Serial.println(Msg_cnt);

  if (Msg_cnt == 6)
  {
    Msg_cnt = 1;
  }

  saveMsgscnt();
  if (Msg_cnt == 1)
  {
    pref.putString("str1", final_amount);
  }
  else if (Msg_cnt == 2)
  {
    pref.putString("str2", final_amount);
  }
  else if (Msg_cnt == 3)
  {
    pref.putString("str3", final_amount);
  }
  else if (Msg_cnt == 4)
  {
    pref.putString("str4", final_amount);
  }
  else if (Msg_cnt == 5)
  {
    pref.putString("str5", final_amount);
  }

}
/*************************** saveMsgs() function end **************************************************************************/

/*************************** reading messages in spiff ************************************************************************/
void readMsgs()
{
  String storedAmount = "";
  String str = "";
  if (Msg_cnt == 5)                                                                            // If message count is 5
  {
    if (Button_cnt == 1)
    {
      storedAmount = pref.getString("str5", str);
    }
    else if (Button_cnt == 2)
    {
      storedAmount = pref.getString("str4", str);
    }
    else if (Button_cnt == 3)
    {
      storedAmount = pref.getString("str3", str);
    }
    else if (Button_cnt == 4)
    {
      storedAmount = pref.getString("str2", str);
    }
    else if (Button_cnt == 5)
    {
      storedAmount = pref.getString("str1", str);
    }
  }
  else if (Msg_cnt == 4)                                                                       // If message count is 4
  {
    if (Button_cnt == 1)
    {
      storedAmount = pref.getString("str4", str);
    }
    else if (Button_cnt == 2)
    {
      storedAmount = pref.getString("str3", str);
    }
    else if (Button_cnt == 3)
    {
      storedAmount = pref.getString("str2", str);
    }
    else if (Button_cnt == 4)
    {
      storedAmount = pref.getString("str1", str);
    }
    else if (Button_cnt == 5)
    {
      storedAmount = pref.getString("str5", str);
    }
  }
  else if (Msg_cnt == 3)                                                                       // If message count is 3
  {
    if (Button_cnt == 1)
    {
      storedAmount = pref.getString("str3", str);
    }
    else if (Button_cnt == 2)
    {
      storedAmount = pref.getString("str2", str);
    }
    else if (Button_cnt == 3)
    {
      storedAmount = pref.getString("str1", str);
    }
    else if (Button_cnt == 4)
    {
      storedAmount = pref.getString("str5", str);
    }
    else if (Button_cnt == 5)
    {
      storedAmount = pref.getString("str4", str);
    }
  }
  else if (Msg_cnt == 2)                                                                         // If message count is 2
  {
    if (Button_cnt == 1)
    {
      storedAmount = pref.getString("str2", str);
    }
    else if (Button_cnt == 2)
    {
      storedAmount = pref.getString("str1", str);
    }
    else if (Button_cnt == 3)
    {
      storedAmount = pref.getString("str5", str);
    }
    else if (Button_cnt == 4)
    {
      storedAmount = pref.getString("str4", str);
    }
    else if (Button_cnt == 5)
    {
      storedAmount = pref.getString("str3", str);
    }
  }
  else if (Msg_cnt == 1)                                                                       // If message count is 1
  {
    if (Button_cnt == 1)
    {
      storedAmount = pref.getString("str1", str);
    }
    else if (Button_cnt == 2)
    {
      storedAmount = pref.getString("str5", str);
    }
    else if (Button_cnt == 3)
    {
      storedAmount = pref.getString("str4", str);
    }
    else if (Button_cnt == 4)
    {
      storedAmount = pref.getString("str3", str);
    }
    else if (Button_cnt == 5)
    {
      storedAmount = pref.getString("str2", str);
    }
  }

  /* String storedAmount = "";
    String str = "";
    String keys[] = {"str5", "str4", "str3", "str2", "str1"};

    if (Msg_cnt >= 1 && Msg_cnt <= 5 && Button_cnt >= 1 && Button_cnt <= 5) {
    int index = (5 + Button_cnt - Msg_cnt) % 5; // Calculate index dynamically
    if(index >= 1){
    storedAmount = pref.getString(keys[index-1].c_str(), str);
    }
    }*/

  Serial.print("Stored Amount22: ");
  Serial.println(storedAmount);

  tft.fillScreen(TFT_WHITE);
  tft.setFreeFont(&FreeSansBold12pt7b);                                             //(&FreeSans18pt7b);
  tft.setTextColor(DISP_TCOLOR, TFT_WHITE);                                         // Set text color to blue and background color to white
  //Payment received successfully for -amount
  tft.drawString(" PREVIOUS ", 53, 70);                                             // Display amount starting from left
  tft.drawString(" RECEIVED ", 48, 120);                                            // Display amount starting from left
  tft.drawString(" AMOUNT IS ", 40, 170);                                           // Display amount starting from leftsuccessful
  tft.drawString(currency + " " + String(storedAmount), 50, 210);

  audioPlayer(1010);                                                                // Play intro audio
  voice_commands(storedAmount);
  Serial.println("Executed voice commands");

  currentPage = Page::defaultQRCode;                                                // Switch to the QR code page
  generateAndWriteQRCode_default();                                                 // Generate and write the QR code based on the amount
  defaultQrpage();                                                                  // Draw the QR code page on the screen
  battery_measurement();
  displayNetworkBars(networkAvailable);
  Serial.println("Static QR  page displayed.");
}
/**************************** readMsgs() function end *************************************************************************/

/**************************** Amount Conversion  ******************************************************************************/
void voice_commands(String amount)
{
  const char *float_amount = amount.c_str();                                        // Convert the amount from String to const char* for further processing
  int amount_int = amount.toInt();                                                  // Convert the amount from String to integer
  String amt_conversion = (String)amount_int;                                       // Convert the integer amount back to String for length calculation
  int amount_len = amt_conversion.length();                                         // Get the length of the integer amount string

  Serial.println(amount);

  // Additional code for voice command processing would go here
  // Only play audio if final_amount is set and audio is not already playing
  if (!final_amount.isEmpty() && !audioPlaying)
  {
    audioPlayer(1010);                                                                       // Play intro audio
  }

  final_amount = "";                                                                        // Reset final_amount after speaking once
  //}
  // if amount is less than and equal to 20.
  if (amount_int <= 20)
  {
    audioPlayer(amount_int);
    Serial.println(amount_int);
  }
  // if amount is greater than 20 and length of string is 2.
  if (amount_len == 2 && amount_int > 20)
  {
    int a[2];
    a[0] = amount_int / 10;
    a[1] = amount_int % 10;

    int value_1 = a[0] * 10;
    int value_2 = a[1];

    audioPlayer(value_1);
    Serial.println(value_1);
    if (value_2 != 0)
    {
      //  myDFPlayer.playMp3Folder(value_2);
      audioPlayer(value_2);
      Serial.println(value_2);
    }
  }

  if (amount_len == 3)                                                                       //if length of amount is 3.
  {
    int a[3];
    a[0] = amount_int / 100;
    int unit = amount_int % 100;
    a[1] = unit / 10;
    a[2] = unit % 10;

    int value_1 = a[0];                                                                     //final values to initiate voice command is placed in value_....
    int value_2 = a[1] * 10;
    int value_3 = a[2];

    audioPlayer(value_1);
    audioPlayer(100);

    if (unit <= 20 && value_2 != 0)                                                        // unit stores the value placed at tens and ones place 120
    {
      audioPlayer(unit);
      Serial.println(unit);
    }

    else if (value_2 != 0 || value_3 != 0)                                                 // if unit value is greater than 20
    {
      if (value_2 != 0)
      {
        audioPlayer(value_2);
        Serial.println(value_2);
      }

      if ( value_3 != 0)
      {
        audioPlayer(value_3);
        Serial.println(value_3);
      }
    }
  }                                                                                          //amount_len == 3 loop end here.

  if (amount_len == 4)                                                                       // if amount is 4 digit number.
  {

    int a[4];
    a[0] = amount_int / 1000;
    int  unit = amount_int % 1000;
    a[1] = unit / 100;
    int unit_1 = amount_int % 1000;
    String unit_str = (String)unit_1;
    int unit_len = unit_str.length();

    int value_1 = a[0];
    int value_2 = a[1];

    audioPlayer(value_1);

    Serial.println(value_1);

    // myDFPlayer.playMp3Folder(1000);                                                       //play specific mp3 in SD:/MP3/1000.mp3; File Name(0~65535)
    audioPlayer(1000);

    if (value_2 != 0)                                                                        //value present at hundreds place.
    {
      audioPlayer(value_2);
      audioPlayer(100);
      Serial.println(value_2);
    }

    if (unit_len == 3 )                                                                    // condition satisfied when value placed at hundred is not zero..
    {
      int unit_2 = unit_1 % 100;
      a[2] = unit_2 / 10;
      a[3] = unit_2 % 10;

      int value_3 = a[2] * 10;
      int value_4 = a[3];

      if (unit_2 <= 20)
      {
        audioPlayer(unit_2);
        Serial.println(unit_2);
      }

      else if (value_3 != 0)
      {
        audioPlayer(value_3);
        Serial.println(value_3);
      }

      if ( value_4 != 0 && unit_2 >= 20)
      {

        //    myDFPlayer.playMp3Folder(value_4);
        audioPlayer(value_4);
        Serial.println(value_4);
      }
    }

    if (unit_len == 2 || unit_len == 1)                                                      // condition satisfied when value placed at hundred position is zero.
    {
      int unit_2 = unit_1 / 10;
      a[2] = unit_2;
      a[3] = unit_1 % 10;

      int value_3 = a[2] * 10;
      int value_4 = a[3];

      if (unit_1 <= 20)
      {
        audioPlayer(unit_1);
        Serial.println(unit_1);
      }

      else if (value_3 != 0) {
        audioPlayer(value_3);
        Serial.println(value_3);
      }

      if ( value_4 != 0 && unit_1 >= 21)
      {
        audioPlayer(value_4);
        Serial.println(value_4);
      }
    }
  }                                                                                         //amount_len == 4 loop end here

  if (amount_len == 5)
  {
    int a[5];
    a[0] = amount_int / 1000;
    int unit_1 = amount_int % 1000;
    String unit_str = (String)unit_1;
    int unit_len = unit_str.length();
    int value_1 = a[0];

    if (value_1 <= 20)                                                                     //if amount is less than and equal to 20.
    {
      audioPlayer(value_1);
      audioPlayer(1000);
    }

    else if (value_1 > 20)                                                                 // if amount is greater than 20 and length of string is 2.
    {
      int a[2];
      a[0] = value_1 / 10;
      a[1] = value_1 % 10;

      int value_1 = a[0] * 10;
      int value_2 = a[1];

      audioPlayer(value_1);
      Serial.println(value_1);

      if (value_2 != 0) {
        audioPlayer(value_2);
        Serial.println(value_2);
      }
      audioPlayer(1000);
    }

    if (unit_len == 3 )                                                                   // condition satisfied when value placed at hundred is not zero..
    {
      int a[3];
      a[0] = unit_1 / 100;
      int unit_2 = unit_1 % 100;
      a[1] = unit_2 / 10;
      a[2] = unit_2 % 10;

      int value_1 = a[0];                                                               //final values to initiate voice command is placed in value_....
      int value_2 = a[1] * 10;
      int value_3 = a[2];

      audioPlayer(value_1);

      // myDFPlayer.playMp3Folder(100);                                                //play specific mp3 in SD:/MP3/100.mp3; File Name(0~65535)
      audioPlayer(100);

      if (unit_2 <= 20 && value_2 != 0)                                               // unit stores the value placed at tens and ones place 120

      {
        audioPlayer(unit_2);
        Serial.println(unit_2);
      }

      else if (value_2 != 0 || value_3 != 0)                                          // if unit value is greater than 20
      {
        if (value_2 != 0)
        {
          audioPlayer(value_2);
          Serial.println(value_2);
        }

        if ( value_3 != 0)
        {
          audioPlayer(value_3);
          Serial.println(value_3);
        }
      }
    }
    if (unit_len == 2 || unit_len == 1)                                              // condition satisfied when value placed at hundred position is zero.
    {
      int unit_2 = unit_1 / 10;
      a[2] = unit_2;
      a[3] = unit_1 % 10;

      int value_3 = a[2] * 10;
      int value_4 = a[3];

      if (unit_1 <= 20)
      {
        audioPlayer(unit_1);
        Serial.println(unit_1);
      }

      else if (value_3 != 0)
      {
        //  myDFPlayer.playMp3Folder(value_3);
        audioPlayer(value_3);

        Serial.println(value_3);
      }

      if ( value_4 != 0 && unit_1 >= 21)
      {
        //  myDFPlayer.playMp3Folder(value_4);
        audioPlayer(value_4);
        Serial.println(value_4);
      }
    }
  }

  if (strstr(float_amount, "."))                                                         //loop to extract decimal value from amount...
  {
    int i = 0;
    while (amount[i] != '.')i++;

    String sliced_amount = amount.substring(i + 1);
    int decimal_value = sliced_amount.toInt();
    if (decimal_value != 0)
    {
      audioPlayer(1011);                                                                 // voice command for (and) is stored at 65000..
      if (decimal_value <= 20)
      {
        // myDFPlayer.playMp3Folder(decimal_value);
        audioPlayer(decimal_value);
      }
      if (decimal_value > 20)
      {
        int a[2];
        a[0] = decimal_value / 10;
        a[1] = decimal_value % 10;

        int value_1 = a[0] * 10;
        int value_2 = a[1];

        //  myDFPlayer.playMp3Folder(value_1);
        audioPlayer(value_1);
        audioPlayer(value_2);
      }
      // voice command for paise is stored at 64000..
      audioPlayer(1012);
    }
  }
}
/**************************** voice_commands(String amount) function end ******************************************************/

/**************************** void audioPlayer(int val) ***********************************************************************/
void audioPlayer(int val)
{
  if (!audioPlaying && !audio.isRunning())
  {
    int rcv_value = val;
    switch (rcv_value)
    {
      case 1: audio.connecttoFS(SD, "/ModAudio/0001.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 2: audio.connecttoFS(SD, "/ModAudio/0002.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 3: audio.connecttoFS(SD, "/ModAudio/0003.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 4: audio.connecttoFS(SD, "/ModAudio/0004.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 5: audio.connecttoFS(SD, "/ModAudio/0005.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 6: audio.connecttoFS(SD, "/ModAudio/0006.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 7: audio.connecttoFS(SD, "/ModAudio/0007.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 8: audio.connecttoFS(SD, "/ModAudio/0008.mp3");                                    // Adjusted file path
        audioPlaying = true;
        break;
      case 9: audio.connecttoFS(SD, "/ModAudio/0009.mp3");                                   // Adjusted file path
        audioPlaying = true;
        break;
      case 10: audio.connecttoFS(SD, "/ModAudio/0010.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 11: audio.connecttoFS(SD, "/ModAudio/0011.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 12: audio.connecttoFS(SD, "/ModAudio/0012.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 13: audio.connecttoFS(SD, "/ModAudio/0013.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 14: audio.connecttoFS(SD, "/ModAudio/0014.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 15: audio.connecttoFS(SD, "/ModAudio/0015.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 16: audio.connecttoFS(SD, "/ModAudio/0016.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 17: audio.connecttoFS(SD, "/ModAudio/0017.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 18: audio.connecttoFS(SD, "/ModAudio/0018.mp3");                                  // Adjusted file path
        audioPlaying = true;
        break;
      case 19: audio.connecttoFS(SD, "/ModAudio/0019.mp3");                                 // Adjusted file path
        audioPlaying = true;
        break;
      case 20: audio.connecttoFS(SD, "/ModAudio/0020.mp3");                                 // Adjusted file path
        audioPlaying = true;
        break;
      case 30: audio.connecttoFS(SD, "/ModAudio/0030.mp3");                                 // Adjusted file path
        audioPlaying = true;
        break;
      case 40: audio.connecttoFS(SD, "/ModAudio/0040.mp3");                                 // Adjusted file path
        audioPlaying = true;
        break;
      case 50: audio.connecttoFS(SD, "/ModAudio/0050.mp3");                                 // Adjusted file path
        audioPlaying = true;
        break;
      case 60: audio.connecttoFS(SD, "/ModAudio/0060.mp3");                                // Adjusted file path
        audioPlaying = true;
        break;
      case 70: audio.connecttoFS(SD, "/ModAudio/0070.mp3");                                // Adjusted file path
        audioPlaying = true;
        break;
      case 80: audio.connecttoFS(SD, "/ModAudio/0080.mp3");                                // Adjusted file path
        audioPlaying = true;
        break;
      case 90: audio.connecttoFS(SD, "/ModAudio/0090.mp3");                                // Adjusted file path
        audioPlaying = true;
        break;
      case 100: audio.connecttoFS(SD, "/ModAudio/0100.mp3");                               // Adjusted file path
        audioPlaying = true;
        break;
      case 1000: audio.connecttoFS(SD, "/ModAudio/1000.mp3");                              // Adjusted file path
        audioPlaying = true;
        break;
      case 1001: audio.connecttoFS(SD, "/ModAudio/1_Poweron.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      case 1002: audio.connecttoFS(SD, "/ModAudio/2_Welcome.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      case 1003: audio.connecttoFS(SD, "/ModAudio/3_Netconn.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      case 1004: audio.connecttoFS(SD, "/ModAudio/4_Netdiscon.mp3");                       // Adjusted file path
        audioPlaying = true;
        break;
      case 1005: audio.connecttoFS(SD, "/ModAudio/5_BLEconn.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      case 1006: audio.connecttoFS(SD, "/ModAudio/6_BLEconfig.mp3");                       // Adjusted file path
        audioPlaying = true;
        break;
      case 1007: audio.connecttoFS(SD, "/ModAudio/7_BLEdiscon.mp3");                       // Adjusted file path
        audioPlaying = true;
        break;
      case 1008: audio.connecttoFS(SD, "/ModAudio/8_Batfull.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      case 1009: audio.connecttoFS(SD, "/ModAudio/9_Batlow.mp3");                          // Adjusted file pathN
        audioPlaying = true;
        break;
      case 1010: audio.connecttoFS(SD, "/ModAudio/10_Intro.mp3");                          // Welcome audio
        audioPlaying = true;
        break;
      case 1011: audio.connecttoFS(SD, "/ModAudio/11_AND.mp3");                            // Adjusted file path
        audioPlaying = true;
        break;
      case 1012: audio.connecttoFS(SD, "/ModAudio/12_Paise.mp3");                          // Adjusted file path
        audioPlaying = true;
        break;
      case 1013: audio.connecttoFS(SD, "/ModAudio/13_SIMloss.mp3");                        // Adjusted file path
        audioPlaying = true;
        break;
      case 1014: audio.connecttoFS(SD, "/ModAudio/14_UserConfig.mp3");                     // Adjusted file path
        audioPlaying = true;
        break;
      case 1015: audio.connecttoFS(SD, "/Vishwaguru/welcome.mp3");                         // Adjusted file path
        audioPlaying = true;
        break;
      default:
        break;
    }
  }
  // Check if audio is done playing
  if (audioPlaying)
  {
    while (audioPlaying && audio.isRunning())
    {
      audio.loop();
    }
    audioPlaying = false;
  }
}
/****************************** audioPlayer(int val) function end *************************************************************/

/*********** File Opening: The function attempts to open a file named "amount.txt" in write mode using SPIFFS.open("/amount.txt", FILE_WRITE).********/
void writeToSPIFFS(String amount)
{
  File file = SPIFFS.open("/amount.txt", FILE_WRITE);                                      // Open the file in write mode

  if (!file)                                                                               // Check if the file was opened successfully
  {
    // Print an error message if the file could not be opened
    Serial.println("Failed to open file for writing");
    return;                                                                                // Exit the function if file opening failed
  }

  file.print(amount);                                                                     // Write the amount to the file

  file.close();                                                                           // Close the file to save changes

  // Print a success message
  Serial.println("Amount written to SPIFFS successfully");
}
/***************************** writeToSPIFFS(String amount) function end ******************************************************/

/** File Opening: The function attempts to open a file named "amount.txt" in read mode using SPIFFS.open("/amount.txt", FILE_READ).**/
String readFromSPIFFS()
{
  // Open the file in read mode
  File file = SPIFFS.open("/amount.txt", FILE_READ);

  if (!file)                                                                                // Check if the file was opened successfully
  {
    // Print an error message if the file could not be opened
    Serial.println("Failed to open file for reading");
    return "";                                                                              // Return an empty string if file opening failed
  }

  String amount = file.readString();                                                        // Read the entire content of the file as a string
  file.close();                                                                             // Close the file after reading

  return amount;                                                                            // Return the read amount
}
/*************************** readFromSPIFFS() function end ********************************************************************/

/************* The function calculates and prints the size of a specified file in the SPIFFS file system. ********************/
void file_size(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);                                              // Print the file path being read

  File file = fs.open(path);                                                                // Open the file at the specified path

  if (!file)                                                                                // Check if the file was opened successfully
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.print("  FILE: ");                                                                // Print the file name and its size
  Serial.print(file.name());
  Serial.print("\tSIZE: ");
  Serial.println(file.size());

  fs_size = file.size();                                                                  // Store the file size in a global variable fs_size
  Serial.println(fs_size);                                                                // Print the file size
  file.close();                                                                           // Close the file
}
/**************************** file_size(fs::FS &fs, const char *path) function end *********************************************/

/**************** The function calculates and prints the size of a specified file in the SPIFFS file system. *******************/
void listDir(fs::FS &fs, const char *dirname, uint8_t levels) // list of files in directory
{
  Serial.printf("Listing directory: %s\r\n", dirname);                                   // Print the directory name being listed
  File root = fs.open(dirname);                                                          // Open the directory

  if (!root)
  {
    // Print an error message if the directory could not be opened
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    // Print an error message if the path is not a directory
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();                                                         // Open the first file in the directory
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");                                                           // If the file is a directory, print its name
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);                                            // If levels is greater than 0, list the contents of the subdirectory
      }
    } else {
      Serial.print("  FILE: ");                                                          // If the file is not a directory, print its name and size
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();                                                         // Open the next file in the directory
  }
}
/***************************** listDir(fs::FS &fs, const char *dirname, uint8_t levels) function end *************************/

/***************************** Writes a message to a specified file. *********************************************************/
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);                                          // Print the file path being written to
  File file = fs.open(path, FILE_WRITE);                                                 // Open the file in write mode
  if (!file)
  {
    // Print an error message if the file could not be opened
    Serial.println("- failed to open file for writing");
    return;
  }

  if (file.print(message))                                                              // Write the message to the file
  {
    // Print a success message if the write operation was successful
    Serial.println("- file written");
  } else
  {
    // Print an error message if the write operation failed
    Serial.println("- write failed");
  }

  file.close();                                                                         // Close the file to save changes
}
/************************** writeFile(fs::FS &fs, const char *path, const char *message) function end *************************/

/********************* Reads the contents of a specified file and prints it to the serial monitor. ****************************/
void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);                                         // Print the file path being read

  File file = fs.open(path);                                                           // Open the file in read mode
  if (!file || file.isDirectory())
  {
    // Print an error message if the file could not be opened or is a directory
    Serial.println("- failed to open file for reading");
    return;
  }

  String String_receive = "";                                                          // Read the file content
  Serial.println("- read from file:");
  if (file.available())
  {
    String_receive = file.readString();                                                // Read the content of the file into a string
  }

  Serial.print("String_receive = ");                                                   // Print the read content
  Serial.println(String_receive);
  file.close();                                                                        // Close the file
}

/*********************The function attempts to delete the file at the specified path using fs.remove(path). *******************/
void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\r\n", path);                                       // Print the file path being deleted

  if (fs.remove(path))                                                                // Delete the file
  {
    // Print a success message if the file was deleted successfully
    Serial.println("- file deleted");
  }
  else
  {
    // Print an error message if the file deletion failed
    Serial.println("- delete failed");
  }
}
/************************ deleteFile(fs::FS &fs, const char *path) function end **********************************************/

/************************T he function ensures that the input voltage is within the defined range ****************************/
float mapVoltageToPercentage(float voltage)
{
  // Clamp voltage to the range of minBatteryVoltage and maxBatteryVoltage
  if (voltage < minBatteryVoltage)                                                          // Ensure voltage does not go below the minimum battery voltage
    voltage = minBatteryVoltage;

  if (voltage > maxBatteryVoltage)                                                          // Ensure voltage does not exceed the maximum battery voltage
    voltage = maxBatteryVoltage;


  // The percentage is calculated as the ratio of the current voltage
  // within the range of minBatteryVoltage to maxBatteryVoltage
  float percentage = (voltage - minBatteryVoltage) / (maxBatteryVoltage - minBatteryVoltage) * 100.0;    // Calculate the battery percentage based on clamped voltage

  // Return the calculated percentage
  return percentage;
}

/**This function processes incoming Bluetooth data, parses it based on a predefined format, and provides feedback for debugging**/
void handleBluetoothInput()
{
  Serial.println("\n handleBluetoothInput() ");
  String received = SerialBT.readStringUntil('\n');                                        // Read a line of input from the Bluetooth Serial connection until a newline character is encountered
  Serial.println("Received: " + received);

  int firstComma = received.indexOf(',');                                                  // Find the positions of the first and second commas in the received string
  int secondComma = received.indexOf(',', firstComma + 1);

  if (firstComma > 0 && secondComma > firstComma)                                          // Check if both commas were found and their positions are valid
  {
    upiID = received.substring(0, firstComma);                                             // Extract the UPI ID, user name, and currency from the received string
    userName = received.substring(firstComma + 1, secondComma);
    currency = received.substring(secondComma + 1);
    Serial.println("UPI ID: " + upiID + ", Name: " + userName + ", Currency: " + currency);
  }
}
/************************* handleBluetoothInput() function end ****************************************************************/

/*******This function handles both the visual updates of the keypad and the specific actions associated with button presses,**/
void handleKeypadTouch(uint16_t t_x, uint16_t t_y, bool pressed)
{
  for (uint8_t b = 0; b < 15; b++)                                                           // Loop through each key to check if it is pressed or not
  {
    if (pressed && key[b].contains(t_x, t_y))                                                // Check if the touch coordinates are within the key bounds and the key is pressed
    {
      key[b].press(true);                                                                    // Mark the key as pressed
    }
    else
    {
      key[b].press(false);                                                                   // Mark the key as not pressed
    }
  }

  for (uint8_t b = 0; b < 15; b++)                                                           // Loop through each key again to update its visual state and handle button actions
  {
    if (b < 3)                                                                               // Set the font based on the key index
    {
      tft.setFreeFont(LABEL1_FONT);                                                          // Use LABEL1_FONT for the first three keys
    } else {
      tft.setFreeFont(LABEL2_FONT);                                                          // Use LABEL2_FONT for the remaining keys
    }

    if (key[b].justReleased())                                                               // Draw the button in its current state (released or pressed)
    {
      key[b].drawButton();                                                                   // Draw button in released state
    }

    if (key[b].justPressed())
    {
      key[b].drawButton(true);                                                               // Draw button in pressed state
      delay(100);
      handleKeypadButtonPress(b);                                                            // Handle the button press action

      if (b == 2)                                                                            // Special action for the button at index 2
      {
        currentPage = Page::QRCode;                                                          // Switch to the QR code page
        amount = atof(numberBuffer);                                                         // Convert the number buffer to an integer amount
        generateAndWriteQRCode(amount);                                                      // Generate and write the QR code based on the amount
        drawQRCodePage();                                                                    // Draw the QR code page on the screen
        battery_measurement();
        displayNetworkBars(networkAvailable);
        countdownTime = 60;                                                                  // Countdown time in seconds
      }
      updateNumberDisplay();                                                                 // Update the display to show the current number
      key[b].press(false);                                                                   // Mark the key as not pressed
    }
  }
}
/***************************** handleKeypadTouch(uint16_t t_x, uint16_t t_y, bool pressed) function end ***********************/

/***This function handles touch input for a specific region of the screen to allow the user to navigate back to the keypad page from the QR code page.***/
void handleQRCode()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)                                               // ADD FUNCTION FOR QR WITH TIMER
  {
    previousMillis = currentMillis;
    if (countdownTime > 0)
    {
      tft.setFreeFont(&FreeSansBold12pt7b);//(&FreeSans18pt7b);
      countdownTime--;
      char time1[50] = {0};
      sprintf(time1, "Time left : %02d:%02d", countdownTime / 60, countdownTime % 60);          // Display amount starting from left
      String time2 = String(time1);
      tft.drawString(time2 , 30, 280);                                                          // Display amount starting from left
    }
    else
    {
      currentPage = Page::defaultQRCode;                                                        // Switch to the QR code page
      generateAndWriteQRCode_default();                                                         // Generate and write the QR code based on the amount
      defaultQrpage();                                                                          // Draw the QR code page on the screen
      battery_measurement();
      displayNetworkBars(networkAvailable);
      Serial.println("Static QR  page displayed.");
    }
  }
}
/************************* handleQRCode() function end ***********************************************************************/

/************************* detecting touch to generate QR with amount ********************************************************/
void handledefaultQRCodeTouch(uint16_t t_x, uint16_t t_y, bool pressed)
{
  Serial.println("handledefaultQRCodeTouch is *************************** "); Serial.println(t_x); Serial.println(t_y); Serial.println(pressed);

  if (pressed && t_x >= 0 && t_x <= 80 && t_y >= 0 && t_y <= 20)                                // Check if the touch event is within the bounds of the designated QR code area
  {
    Serial.println("handledefaultQRCodeTouch is IN LOOP ");
    numberIndex = 0;                                                                            //"BACK" should clear amount  -->DONE TESTING PENDING
    numberBuffer[numberIndex] = 0;                                                              //8. "BACK" should clear amount  -->DONE TESTING PENDING

    currentPage = Page::Keypad;                                                                 // Change the current page to the keypad page

    drawKeypadPage();                                                                           // Draw the keypad page on the screen
    Serial.println("handledefaultQRCodeTouch is OUT LOOP ");
  }
}
/************************* handledefaultQRCodeTouch(uint16_t t_x, uint16_t t_y, bool pressed) function end *******************/

/************************* drawing keypad page *******************************************************************************/
void drawKeypadPage() {
  tft.fillScreen(TFT_DARKGREY);
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);                                      // Change display area to white
  tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);                                      // Draw the white border

  for (uint8_t row = 0; row < 5; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;
      if (b < 3) {
        tft.setFreeFont(LABEL1_FONT);
      } else {
        tft.setFreeFont(LABEL2_FONT);
      }
      key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X), KEY_Y + row * (KEY_H + KEY_SPACING_Y), KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE, keyLabel[b], KEY_TEXTSIZE);
      key[b].drawButton();
    }
  }
}
/*************************** drawKeypadPage() function end *******************************************************************/

/*************************** setting QR code page ****************************************************************************/
void drawQRCodePage() {
  tft.fillScreen(TFT_WHITE);                                                                     // Clear the entire screen with white

  File qrFile = SPIFFS.open(QR_CODE_FILE, FILE_READ);
  if (!qrFile) {
    Serial.println("Failed to open file for reading");
    return;
  }

  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(6)];
  qrcode_initText(&qrcode, qrcodeData, 6, 0, numberBuffer);

  int qrSize = qrcode.size > 0 ? qrcode.size : 1;
  int moduleSize = 200 / qrSize;                                                                 // Slightly reduced size to add some margin

  int xOffset = (240 - (qrSize * moduleSize)) / 2;                                               // Calculate margins to center the QR code
  int yOffset = ((320 - (qrSize * moduleSize)) / 2) + 20;                                        // Adjusted downwards

  for (uint8_t y = 0; y < qrcode.size; y++) {                                                    // Draw QR code
    for (uint8_t x = 0; x < qrcode.size; x++) {
      int color = qrFile.read() == 0x01 ? TFT_BLACK : TFT_WHITE;
      tft.fillRect(xOffset + x * moduleSize, yOffset + y * moduleSize, moduleSize, moduleSize, color);
    }
  }

  qrFile.close();
  tft.setFreeFont(LABEL1_FONT);                                                                 // Display UPI ID
  tft.setTextColor(TFT_BLACK);

  tft.setTextDatum(TL_DATUM);
  tft.drawString(upiID, 10, 60);                                                                // Display UPI ID slightly lower
}
/*************************** drawQRCodePage() function end *******************************************************************/

/*************************** handling keypad touch ***************************************************************************/
void handleKeypadButtonPress(uint8_t b) {
  if (b >= 3) {
    if (numberIndex < NUM_LEN) {
      numberBuffer[numberIndex] = keyLabel[b][0];
      numberIndex++;
      numberBuffer[numberIndex] = 0;
    }
    key[b].drawButton(false, keyLabel[b]);
    status("");
  }

  if (b == 1) {
    numberBuffer[numberIndex] = 0;
    if (numberIndex > 0) {
      numberIndex--;
      numberBuffer[numberIndex] = 0;
    }
    key[b].drawButton(false, keyLabel[b]);
    status("");
  }

  if (b == 0) {
    key[b].drawButton(false, keyLabel[b]);
    status("");
    status("Value cleared");
    numberIndex = 0;
    numberBuffer[numberIndex] = 0;
  }
}
/************************** handleKeypadButtonPress(uint8_t b) function end ***************************************************/

/************************** Showing amount for QR on display ******************************************************************/
void updateNumberDisplay()
{
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);                                      // Clear the display area with white color
  tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);                                      // Draw the border again

  tft.setTextDatum(TL_DATUM);                                                                   // Set text alignment to top-left
  tft.setFreeFont(&FreeSansBold12pt7b);
  tft.setTextColor(DISP_TCOLOR, TFT_WHITE);                                                     // Set text color to blue and background color to white

  tft.drawString(currency + " " + String(numberBuffer), DISP_X + 4, DISP_Y + 12);               // Display amount starting from left
}
/**************************** updateNumberDisplay() function end **************************************************************/

/**************************** generating QR code with amount ******************************************************************/
void generateAndWriteQRCode(float amount) {
  String upiLink = "upi://pay?pa=" + upiID + "&pn=" + userName + "&am=" + String(amount) + "&cu=" + currency;    // Construct UPI payment link with dynamic amount

  StaticJsonDocument<128> jsonDoc;                                                              // Create JSON object for UPI payment details
  JsonObject upiPayment = jsonDoc.createNestedObject("upi_payment");
  upiPayment["pa"] = upiID;
  upiPayment["pn"] = userName;
  upiPayment["am"] = amount;
  upiPayment["cu"] = currency;

  String jsonString;                                                                           // Convert JSON object to string
  serializeJson(jsonDoc, jsonString);

  generateQRCode(upiLink.c_str());                                                             // Generate QR code from UPI link and write to file
}
/************************** generateAndWriteQRCode(float amount) function end ************************************************/

/************************** generating QR code link **************************************************************************/
void generateAndWriteQRCode_default() {
  String upiLink = "upi://pay?pa=" + upiID + "&pn=" + userName;                               // Construct UPI payment link with dynamic amount

  StaticJsonDocument<128> jsonDoc;                                                            // Create JSON object for UPI payment details
  JsonObject upiPayment = jsonDoc.createNestedObject("upi_payment");
  upiPayment["pa"] = upiID;
  upiPayment["pn"] = userName;

  String jsonString;                                                                          // Convert JSON object to string
  serializeJson(jsonDoc, jsonString);

  generateQRCode(upiLink.c_str());                                                            // Generate QR code from UPI link and write to file
}
/************************** generateAndWriteQRCode_default() function end *****************************************************/

/************************** generating default QR page ************************************************************************/
void defaultQrpage() {
  // ADD FUNCTION FOR defaultQrpage Page TFT PAGE WITH "NEW" BUTTON
  tft.fillScreen(TFT_WHITE);                                                                  // Clear the entire screen with white

  File qrFile = SPIFFS.open(QR_DFLT_FILE, FILE_READ);
  if (!qrFile) {
    Serial.println("Failed to open file for reading");
    return;
  }

  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(6)];
  qrcode_initText(&qrcode, qrcodeData, 6, 0, numberBuffer);

  int qrSize = qrcode.size > 0 ? qrcode.size : 1;
  int moduleSize = 200 / qrSize;                                                             // Slightly reduced size to add some margin

  // Calculate margins to center the QR code
  int xOffset = (240 - (qrSize * moduleSize)) / 2;
  int yOffset = ((320 - (qrSize * moduleSize)) / 2) + 20;                                    // Adjusted downwards

  // Draw QR code
  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      int color = qrFile.read() == 0x01 ? TFT_BLACK : TFT_WHITE;
      tft.fillRect(xOffset + x * moduleSize, yOffset + y * moduleSize, moduleSize, moduleSize, color);
    }
  }

  qrFile.close();

  //  // Draw "New" button
  //  tft.setFreeFont(LABEL1_FONT);
  //  //key[0].initButton(&tft, 200, 300, 80, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "New", KEY_TEXTSIZE); // Adjusted y position
  //  key[0].initButton(&tft, 40, 20, 80, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "New ", KEY_TEXTSIZE); // Adjusted y position
  //  key[0].drawButton();

  // Display UPI ID
  tft.setFreeFont(LABEL1_FONT);
  tft.setTextColor(TFT_BLACK);

  tft.setTextDatum(TL_DATUM);
  tft.drawString(upiID, 10, 60);                                                                // Display UPI ID slightly lower
}
/*************************** defaultQrpage() function end *********************************************************************/

/******************** Function to generate a QR code from a given text and save it to a file in SPIFFS ************************/
void generateQRCode(const char* text) {
  QRCode qrcode;                                                                                // Create a QRCode object to store the QR code data

  uint8_t qrcodeData[qrcode_getBufferSize(6)];                                                  // Create a buffer to hold the QR code data, with a size based on the QR code version (6)

  qrcode_initText(&qrcode, qrcodeData, 6, 0, text);                                             // Initialize the QR code with the provided text, setting the version to 6 and the error correction level to 0
  File qrFile ;
  if (currentPage == Page::QRCode)
  {
    qrFile = SPIFFS.open(QR_CODE_FILE, FILE_WRITE);                                             // Open the file in SPIFFS for writing the QR code data
  }
  else if (currentPage == Page::defaultQRCode)
  {
    qrFile = SPIFFS.open(QR_DFLT_FILE, FILE_WRITE);                                             // Open the file in SPIFFS for writing the QR code data
  }

  if (!qrFile) {                                                                                // Check if the file was opened successfully
    // If not, print an error message to the serial monitor
    Serial.println("Failed to open file for writing");
    return;                                                                                    // Exit the function if the file could not be opened
  }

  for (uint8_t y = 0; y < qrcode.size; y++) {                                                  // Iterate over each row of the QR code
    for (uint8_t x = 0; x < qrcode.size; x++) {                                                // Iterate over each column of the QR code
      qrFile.write(qrcode_getModule(&qrcode, x, y) ? 0x01 : 0x00);                             // Write the module data to the file (1 if the module is black, 0 if it's white)
    }
  }

  qrFile.close();                                                                              // Close the file after writing all QR code data

  // Print a success message to the serial monitor
  Serial.println("QR code generated and saved to SPIFFS");
}
/***************************** generateQRCode(const char* text) function end **************************************************/

/** This function ensures that the touch screen is calibrated correctly and that the calibration data is saved and reused properly. **/
void touch_calibrate()
{
  uint16_t calData[5];                                                                        // Array to store calibration data
  uint8_t calDataOK = 0;                                                                      // Flag to indicate if calibration data is valid

  if (!SPIFFS.begin())                                                                        // Initialize the SPIFFS file system
  {
    Serial.println("Formatting file system");
    SPIFFS.format();                                                                          // Format the file system if initialization fails
    SPIFFS.begin();                                                                           // Re-initialize the file system
  }

  if (SPIFFS.exists(CALIBRATION_FILE))                                                        // Check if the calibration file exists
  {
    if (REPEAT_CAL)
    {
      SPIFFS.remove(CALIBRATION_FILE);                                                        // Remove file if REPEAT_CAL is true
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");                                            // Open file for reading
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)                                           // Read 14 bytes of calibration data
        {
          calDataOK = 1;                                                                      // Set flag to indicate data is OK
        }
        f.close();                                                                            // Close the file
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {                                                            // If valid calibration data is found and REPEAT_CAL is not set
    tft.setTouch(calData);                                                                   // Apply calibration data to the TFT
  }
  else
  {
    tft.fillScreen(TFT_BLACK);                                                               // Clear screen with black background
    tft.setCursor(20, 0);                                                                    // Set cursor position
    tft.setTextFont(2);                                                                      // Set text font
    tft.setTextSize(1);                                                                      // Set text size
    tft.setTextColor(TFT_WHITE, TFT_BLACK);                                                  // Set text color

    tft.println("Touch corners as indicated");                                               // Display instruction

    tft.setTextFont(1);                                                                      // Set font for additional text
    tft.println();                                                                           // Print a new line

    if (REPEAT_CAL)                                                                          // If REPEAT_CAL is true, display a warning message
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }                                                                                       // Start the touch calibration process

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);                                 // Calibrate touch with given parameters

    tft.setTextColor(TFT_GREEN, TFT_BLACK);                                                 // Set text color for completion message
    tft.println("Calibration complete!");                                                   // Display completion message

    // Save the calibration data to the file
    File f = SPIFFS.open(CALIBRATION_FILE, "w");                                            // Open file for writing
    if (f)
    {
      f.write((const unsigned char *)calData, 14);                                          // Write calibration data to file
      f.close(); // Close the file
    }
  }
}
/**************************** touch_calibrate() function end *******************************************************************/

/*****The status() function provides a way to display status messages both on the Serial Monitor and on the TFT screen. ********/
void status(const char *msg)
{
  Serial.println(msg);                                                                     // Print status message to Serial Monitor
  tft.setTextPadding(240);                                                                 // Set text padding for wrapping text (optional, depends on your use case)
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);                                               // Set text color to white with a dark grey background
  tft.setTextFont(0);                                                                      // Set the text font (font 0 is typically the default font)
  tft.setTextDatum(TC_DATUM);                                                              // Set text datum to center (TC_DATUM centers text)
  tft.setTextSize(1);                                                                      // Set text size to 1 (default size)
  tft.drawString(msg, 120, 65);                                                            // Draw the string message at position (120, 65) on the TFT screen
}
/****************************** status(const char *msg) function end **********************************************************/

/****************************** Volume Adjustment Functions *******************************************************************/
void setVolume(int volume) {
  volume = constrain(volume, 0, 100);                                                      // Ensure volume is within range
  audio.setVolume(volume);
  Serial.printf("Volume set to: %d\n", volume);
}
/****************************** setVolume(int volume) function end *************************************************************/

/****************************** Function to increase volume by 10 **************************************************************/
void increaseVolume() {
  currentVolume += 10;                                                                            // increase the volume by 10
  if (currentVolume > 100) currentVolume = 100;
  setVolume(currentVolume);
}
/****************************** increaseVolume() function end *****************************************************************/

/****************************** Function to decrease volume by 10 *************************************************************/
void decreaseVolume() {
  currentVolume -= 10;                                                                           // decrease the volume by 10
  if (currentVolume < 0) currentVolume = 0;
  setVolume(currentVolume);
}
/****************************** decreaseVolume() function end *****************************************************************/

/****************************** To check user response for setting details ****************************************************/
void userResponse() {
  String res;

  while (!(decision) && !(Timeout_flag)) {                                                      // checking for user choice
    if (Parameter_flag) {
      res = String(Recieved);
      if ((res == "NO" || res == "No") || res == "no") {                                        // configuration skipped?
        decision = true;
        break;
      }
      else if ((res == "YES" || res == "Yes") || res == "yes") {                                // want configuration?
        decision = true;
        para_setting = true;
        break;
      }
      else {
        pCharacteristic->setValue("Please enter correct word \"yes\",\"no\"\n");               // not valid response
        pCharacteristic->notify();
      }
      Parameter_flag = false;
    }

  }
}
/******************************* userResponse() function end *****************************************************************/
