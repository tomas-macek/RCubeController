/*
  Updated by Tomas Macek to support:
     handling attached i2c display 
     measure time of the cube assembly from first move (after reboot) to last moov leading to assembled cube 
     triggering commands by cube move sequence
  This project is based on:
    DhirajGehlot: smartCube github project https://github.com/DhirajGehlot/smartCube/tree/master
    playfultechnology: esp32-smartcube fithub project https://github.com/playfultechnology/esp32-smartcube
    Juan Andrés Claramunt Pérez: Xiaomi Mi Smart Rubik Cube: Hacking and having fun, https://medium.com/@juananclaramunt/xiaomi-mi-smart-rubik-cube-ff5a22549f90
    Suraj Gehlot: Hacking smart Rubik’s cube to control home appliances http://www.blogtheorem.com/hacking-smart-rubiks-cube-to-control-home-appliances
    
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

//PIN mapping
const byte timeResetPin = 0;
const byte assembledPin = 1;
const byte notAssembledPin = 2;
#define OLED_SDA 5
#define OLED_SCL 4 
#define NEO_PIN 14
#define OLED_RST 16
#define SERVO1_PIN 17
#define SERVO2_PIN 18
#define SERVO3_PIN 19

static BLEAddress *pServerAddress = new BLEAddress("D0:6A:A1:CD:E0:C0");

BLEServer *pServer = NULL;
// The remote service we wish to connect to
static BLEUUID serviceUUID("0000aadb-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we want to track
static BLEUUID charUUID("0000aadc-0000-1000-8000-00805f9b34fb");
// The following constants are used to decrypt the data representing the state of the cube
// see https://github.com/cs0x7f/cstimer/blob/master/src/js/bluetooth.js
const uint8_t decryptionKey[] = {176, 81, 104, 224, 86, 137, 237, 119, 38, 26, 193, 161, 210, 126, 150, 81, 93, 13, 236, 249, 89, 235, 88, 24, 113, 81, 214, 131, 130, 199, 2, 169, 39, 165, 171, 41};
// This pin will have a HIGH pulse sent when the cube is solved

// Buffer of move history
#define HISTORY_LENGTH 10   // length of the history circle buffer storing moves
static int history[HISTORY_LENGTH]; // last n moves - circulating buffer
static int lastHistory;             // index where the last move has been written

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//NEO PIXELS configuration
#define NO_PIXELS 3
#define PIXEL_A   0
#define PIXEL_B   1

//RGB values for Neopixel A and B
static byte A_R;
static byte A_G;
static byte A_B;
static byte B_R;
static byte B_G;
static byte B_B;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NO_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

Servo servo1, servo2, servo3;   // servo object to control a servo
int pos1 = 0;                   // variables to store the servo position
int pos2 = 0;           
int pos3 = 0;           

// Data array representing a solved cube
const byte solution[16] = {0x12,0x34,0x56,0x78,0x33,0x33,0x33,0x33,0x12,0x34,0x56,0x78,0x9a,0xbc,0x00,0x00};

// BLUETOOTH
// Have we found a cube with the right MAC address to connect to?
static boolean deviceFound = false;
// Are we currently connected to the cube?
static boolean connected = false;
// Properties of the device found via scan
static BLEAdvertisedDevice* myDevice;
// BT characteristic of the connected device
static BLERemoteCharacteristic* pRemoteCharacteristic;

// The cube assembly measurement
static int solved = false;    // flag indicating if the cube is in solved state
static int reco = 0;          // flag indicating recognized gesture
static int lastMoveDirection; // last move direction
static int lastMoveFace;      //last move 
static String solvedStr;      // string versions of solved, lastMoveDirection and lastMoveFace
static String lastMoveDirectionStr;
static String lastMoveFaceStr; 
static unsigned long startTime= 0;  // time when measurement started, if 0 - it did not started yet, next move will start it 
static unsigned long duration = 0;  // duration - time measured from start to completed
static int moveCount;               // counter of cube moves

/* Returns the ith bit of the val */
int getBit(uint8_t* val, int i) {
  int n = ((i / 8) | 0);
  int shift = 7 - (i % 8);
  return (val[n] >> shift) & 1;    
}

/**
 * Return the ith nibble (half-byte, i.e. 16 possible values)
 */
uint8_t getNibble(uint8_t val[], int i) {
  if(i % 2 == 1) {
    return val[(i/2)|0] % 16;
  }
  return 0|(val[(i/2)|0] / 16);
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  // The onResult callback is called for every advertised device found by the scan  
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Print the MAC address of this device
    Serial.print(" - ");
    Serial.print(advertisedDevice.getAddress().toString().c_str());
    // Does this device match the MAC address we're looking for?
    if(advertisedDevice.getAddress().equals(*pServerAddress)) {
      // Stop scanning for further devices
      advertisedDevice.getScan()->stop();
      // Create a new device based on properties of advertised device
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      // Set flag
      deviceFound = true;
      Serial.println(F(" - Connecting!"));
    }
    else {
      Serial.println(F("... MAC address does not match"));
    }
  }
};

/**
 * Callbacks for device we connect to
 */
class ClientCallbacks : public BLEClientCallbacks {
  // Called when a new connection is established
  void onConnect(BLEClient* pclient) {
    //digitalWrite(LED_BUILTIN, HIGH);
    connected = true;
  }
  // Called when a connection is lost
  void onDisconnect(BLEClient* pclient) {
    //digitalWrite(LED_BUILTIN, LOW);
    connected = false;
  }
};

/************************************************************************************************************
 ************************************************************************************************************
 ************************************************************************************************************/
bool isSeq(int sequence[], int seqSize){
  for(int i=0; i< seqSize; i++){  
    int index= (lastHistory+HISTORY_LENGTH-i-1) % HISTORY_LENGTH;
    if (history[index] != sequence[i]){
      return false;
    }
  }
  return true;
}
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // DECRYPT DATA
  // Early Bluetooth cubes used an unencrypted data format that sent the corner/edge indexes as a 
  // simple raw string, e.g. pData for a solved cube would be 1234567833333333123456789abc000041414141
  // However, newer cubes e.g. Giiker i3s encrypt data with a rotating key, so that same state might be
  // 706f6936b1edd1b5e00264d099a4e8a19d3ea7f1 then d9f67772c3e9a5ea6e84447abb527156f9dca705 etc.
  
  // To find out whether the data is encrypted, we first read the penultimate byte of the characteristic data. 
  // As in the two examples above, if this is 0xA7, we know it's encrypted
  bool isEncrypted = (pData[18] == 0xA7);

  // If it *is* encrypted...
  if(isEncrypted) {
    // Split the last byte into two 4-bit values 
    int offset1 = getNibble(pData, 38);
    int offset2 = getNibble(pData, 39);

    // Retrieve a pair of offset values from the decryption key 
    for (int i=0; i<20; i++) {
      // Apply the offset to each value in the data
      pData[i] += (decryptionKey[offset1 + i] + decryptionKey[offset2 + i]);
    }
  }

  // First 16 bytes represent state of the cube - 8 corners (with 3 orientations), and 12 edges (can be flipped)
  Serial.print("Current State: ");
  for (int i=0; i<16; i++) {
    Serial.print(pData[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");

  // Byte 17 represents the last twist made - first half-byte is face, and second half-byte is direction of rotation
  lastMoveFace = getNibble(pData, 32);
  lastMoveDirection = getNibble(pData, 33);
  if (moveCount > 0){
    history[lastHistory]= pData[16];
    lastHistory= (lastHistory+1)%HISTORY_LENGTH;
  }
  /*
  Serial.print("Lastchar:");
  Serial.println(pData[16], HEX);
  for(int i=0; i< HISTORY_LENGTH; i++){
    Serial.print(" ");
    Serial.print(history[i]);
  }
  Serial.println("");
  for(int i=0; i< HISTORY_LENGTH; i++){  
    Serial.print(" ");
    Serial.print(history[(lastHistory+HISTORY_LENGTH-i-1) % HISTORY_LENGTH]);
  }
  Serial.println("");
  */
  
  if (startTime == 0 and moveCount > 0){ // initial call after reboot is not the one to start counting
    startTime= millis();
    Serial.print("Starting timer !!!!");
  }
  moveCount++;
  Serial.print("moveCount=");
  Serial.println(moveCount);
  
  if(memcmp(pData, solution, 16) == 0) {
    solved = 1;
    duration= millis() - startTime;
    /*
    Serial.print("CUBE SOLVED!");
    Serial.print("Duration:" + duration);
    Serial.print("solved:" + solved);
    Serial.print("lastMoveFace:" + lastMoveFace);
    Serial.print("lastMoveDirection:" + lastMoveDirection);
    */
    //Serial.println(duration);  
    digitalWrite(assembledPin, HIGH);
    digitalWrite(notAssembledPin, LOW);
     //pixels.setPixelColor(1, pixels.Color(0, 0, 255));
    //pixels.show(); 

  }else{
    if (solved==1){
       //pixels.setPixelColor(1, pixels.Color(0, 255, 255));
       //pixels.show(); 
    }
    solved= 0;
    digitalWrite(assembledPin, LOW);
    digitalWrite(notAssembledPin, HIGH);
  }

  int reco1[]= {19,17,19};
  if (isSeq(reco1, 3)){
    //pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    //pixels.show(); 
    reco = 1;
    Serial.print("reco:1");
    return;
  }
  reco = 0;

  int reco2[]= {67, 65, 67, 65};
  if (isSeq(reco2, 4)){
    //pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    //pixels.show(); 
    startTime = 0;
    reco = 2;
    Serial.print("reco:2");
    return;
  }
  reco = 0;

  move2colorA(pData[16]);  //mapping to colors based on table
  move2colorB(pData[16]);  //incrementing/decrementing
  move2pos(pData[16]);     //move servos
  return; 
}
/*------------------------------------------------------------------------------------------------------*/

int chColor( int color, int diff){
  color+= diff;
  color= (color > 255)? 255: color;
  color= (color < 0)? 0: color;
  return color;
}
int chServo( int pos, int diff){
  pos+= diff;
  pos= (pos > 180)? 180: pos;
  pos= (pos < 0)? 0: pos;
  return pos;
}

bool move2colorA(byte m){
  byte m2c[] = {
              17, 0, 0, 100, //F
              19, 0, 0, 200, //F'
              49, 0, 100, 0, //R
              51, 0, 200, 0, //R'
              81, 100, 0, 0, //L
              83, 250, 0, 0, //L'
              97, 100, 0, 200, //B
              99, 200, 0, 200, //B'
              33, 100, 200, 0, //D
              35, 200, 200, 0, //D'
              65, 100, 200, 200, //T
              67, 200, 200, 200};//T'

  for(int i=0; i < 12*4; i+=4){
    if (m2c[i] == m){
       A_R = m2c[i+1]; A_G = m2c[i+2]; A_B = m2c[i+3];
       Serial.print("RGB:");
       Serial.print(m2c[i + 1]);
       Serial.print(", ");
       Serial.print(m2c[i + 2]);
       Serial.print(", ");
       Serial.println(m2c[i + 3]);
       return true;
    }
  }
  return false;
}

bool move2colorB(byte m){
  B_R = (m==17)? chColor(B_R, 10):B_R;   //F
  B_R = (m==19)? chColor(B_R, -10):B_R;  //F'
  B_R = (m==97)? chColor(B_R, 50):B_R;   //B
  B_R = (m==99)? chColor(B_R, -50):B_R;  //B'

  B_G = (m==49)? chColor(B_G, 10):B_G;   //R
  B_G = (m==51)? chColor(B_G, -10):B_G;  //R'
  B_G = (m==81)? chColor(B_G, 50):B_G;   //L
  B_G = (m==83)? chColor(B_G, -50):B_G;  //L'

  B_B = (m==65)? chColor(B_B, 10):B_B;   //D
  B_B = (m==67)? chColor(B_B, -10):B_B;  //D'
  B_B = (m==33)? chColor(B_B, 50):B_B;   //T
  B_B = (m==35)? chColor(B_B, -50):B_B;  //T'

  Serial.print("RGB:");
  Serial.print(B_R);
  Serial.print(", ");
  Serial.print(B_G);
  Serial.print(", ");
  Serial.println(B_B);
}


bool move2pos(byte m){
  pos1 = (m==17)? chServo(pos1, 10):pos1;   //F
  pos1 = (m==19)? chServo(pos1, -10):pos1;  //F'
  pos1 = (m==97)? chServo(pos1, 50):pos1;   //B
  pos1 = (m==99)? chServo(pos1, -50):pos1;  //B'

  pos2 = (m==49)? chServo(pos2, 10):pos2;   //R
  pos2 = (m==51)? chServo(pos2, -10):pos2;  //R'
  pos2 = (m==81)? chServo(pos2, 50):pos2;   //L
  pos2 = (m==83)? chServo(pos2, -50):pos2;  //L'

  pos3 = (m==65)? chServo(pos3, 10):pos3;   //D
  pos3 = (m==67)? chServo(pos3, -10):pos3;  //D'
  pos3 = (m==33)? chServo(pos3, 50):pos3;   //T
  pos3 = (m==35)? chServo(pos3, -50):pos3;  //T'

  Serial.print("Servo:");
  Serial.print(pos1);
  Serial.print(", ");
  Serial.print(pos2);
  Serial.print(", ");
  Serial.print(pos3);
}
/*
 * Connect to the BLE server of the correct MAC address
 */
bool connectToServer() {
    Serial.print(F("Creating BLE client... "));
    BLEClient* pClient = BLEDevice::createClient();
    delay(500);
    Serial.println(F("Done."));

    Serial.print(F("Assigning callbacks... "));
    pClient->setClientCallbacks(new ClientCallbacks());
    delay(500);
    Serial.println(F(" - Done."));

    // Connect to the remove BLE Server.
    Serial.print(F("Connecting to "));
    Serial.print(myDevice->getAddress().toString().c_str());
    Serial.print(F("... "));
    pClient->connect(myDevice);
    delay(500);
    Serial.println(" - Done.");
    
    // Obtain a reference to the service we are after in the remote BLE server.
    Serial.print(F("Finding service "));
    Serial.print(serviceUUID.toString().c_str());
    Serial.print(F("... "));
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    delay(500);
    if (pRemoteService == nullptr) {
      Serial.println(F("FAILED."));
      return false;
    }
    Serial.println(" - Done.");
    delay(500);

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    Serial.print(F("Finding characteristic "));
    Serial.print(charUUID.toString().c_str());
    Serial.print(F("... "));
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.println(F("FAILED."));
      return false;
    }
    Serial.println(" - Done.");
    delay(500);
    
    Serial.print(F("Registering for notifications... "));
    if(pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      Serial.println(" - Done.");
    }
    else {
      Serial.println(F("FAILED."));
      return false;
    }
    Serial.println("READY!");
}

/**
 * Search for any advertised devices
 */
void scanForDevices(){
  Serial.println("Scanning for Bluetooth devices...");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup_oled() {
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("STARTING... ");
  display.display();
 
  display.display();
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  startTime = 0;  // after reboot we are ready to start measuring time with the first move
  moveCount = 0;
  lastHistory = 0;
  for (int i=0; i < HISTORY_LENGTH; i++){
    history[i]= 0;  //invalidate all the history in order to not trigger an action by accident
  }

  // attaches the servos on pin 
  servo1.attach(SERVO1_PIN);  
  servo2.attach(SERVO2_PIN); 
  servo3.attach(SERVO3_PIN); 

  // Set ctrl pins
  pinMode(timeResetPin, INPUT_PULLUP);
  pinMode(assembledPin, OUTPUT);
  digitalWrite(assembledPin, LOW);
  pinMode(notAssembledPin, OUTPUT);
  digitalWrite(notAssembledPin, HIGH);
  
  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
    setup_oled();

  // NEO pixels init
  A_R=0; A_G=0; A_B=0;
  B_R=0; B_G=0; B_B=0;
  pixels.begin(); // Initializes the NeoPixel library.
  pixels.setPixelColor(PIXEL_A, pixels.Color(0, 0, 0));
  pixels.setPixelColor(PIXEL_B, pixels.Color(0, 0, 0));
  pixels.show(); 

  pos1 = 75; pos2 = 75; pos3 = 75;
  servo1.write(pos1);              
  servo2.write(pos2);              
  servo3.write(pos3);              

  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  display_status(); //LCD update

  // SERVO update
  servo1.write(pos1);              
  servo2.write(pos2);              
  servo3.write(pos3);              

  // NEOOPIXELS update
  pixels.setPixelColor(PIXEL_A, pixels.Color(A_R, A_G, A_B));
  pixels.setPixelColor(PIXEL_B, pixels.Color(B_R, B_G, B_B));
  pixels.show(); 

  // check reset_time button
  if (!digitalRead(timeResetPin)){
    startTime= 0;    
  }

  if (deviceFound) {
    if(!connected) {
      connectToServer();
    }
  }
  else {
    scanForDevices();
  }
  // Introduce a little delay
  delay(100);
}

void status2text(){
  // Byte 17 represents the last twist made - first half-byte is face, and second half-byte is direction of rotation
  char* faceNames[6] = {"F", "D", "R", "T", "L", "B"};
  lastMoveFaceStr= faceNames[lastMoveFace-1];
  lastMoveDirectionStr= (lastMoveDirection == 1 ? " " : "\'");
  solvedStr = (solved ? "SOLVED" : "NOT SOLVED");
}

/* 
 *  Convert time in ms to string mm:s.ms (mm-minutes, s-seconds, ms - miliseconds)
*/
String ms2time(long duration){
  String ret; 
  long micro_sec= duration%60000;
  int sec= micro_sec/1000;
  return String(duration/60000) + ":" + String(sec) + "." + String(micro_sec%1000);
}

/*
 * Display status on i2C LCD
 */
void display_status(){
   display.clearDisplay();
   display.setCursor(0,10);

   if (deviceFound) {
    if (connected) {
      status2text();
      /*
      Serial.print("solved= " + String(solved));
      Serial.println("Duration= " + String(duration));
      Serial.println("lastMove= " + lastMoveFaceStr + lastMoveDirectionStr);
      Serial.println("----"); 
      */
      display.setCursor(0,10);
      display.print("Status= ");
      display.print(solvedStr);
      display.print(", ");
      display.print(reco);
      
      display.setCursor(0,20);
      display.print("moveCount= ");
      display.print(moveCount);

      display.setCursor(0,30);
      display.print(lastMoveFaceStr);
      display.print(lastMoveDirectionStr);
      
      display.print(",");
      for(int i=0; i< 6; i++){  // display last 6 moves
        int value= history[(lastHistory+HISTORY_LENGTH-i-1) % HISTORY_LENGTH];
        if (value >0){
          display.print(" ");       
          display.print(value);
        }
      }
      display.setCursor(0,00);
      display.print("End");
      display.setCursor(0,40);
      
      if (!startTime){
        display.print("0:0.0");
        display.display();          
      } else{
        if (!solved){
          display.print(ms2time(millis() - startTime));
        } else{
          display.print(ms2time(duration));
          display.print(", (");
          display.print(duration);
          display.print(")");
        }      
      }
    }else{
      display.print("BT connecting");
    }
  }else{
    display.print("BT searching");
  }
  display.display();          
}
