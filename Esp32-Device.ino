#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <time.h>
#include <stdlib.h>
#include <BLEAdvertisedDevice.h>

#define BATTERY_LEVEL_SERVICE_UUID        "180F"
#define BATTERY_LEVEL_CHARACTERISTIC_UUID "2A19"

#define TX_POWER_SERVICE_UUID "1804"
#define TX_POWER_LEVEL_CHARACTERISTIC_UUID "2A07"

#define BAUD_RATE 115200

//device name should change and be unique for each device. 
//TODO Modify to be Uniquilly generated at compile time?
#define DEVICE_NAME "SphereLink-1357"

bool deviceConnected = false;

unsigned long previousBatteryUpdateTime = 0;
unsigned long batteryUpdateInterval = 10000; // 1 minutes in milliseconds


// class used to easily verify whether the device is connected.
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("onConnect Event Received.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("oNDisconnect Event Received.");
      //Start Advertising Again
      BLEDevice::startAdvertising();
    }

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

        Serial.println("onWrite was Called");
    }

    void onNotify(BLECharacteristic* pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
        Serial.println("onNotify was Called");
        Serial.print("New value: ");
        //Serial.println(value);
    }

   void onRead(BLECharacteristic* pCharacteristic) {

      Serial.println("onRead was called");
   }
};

uint8_t batteryLevel = 100;
BLECharacteristic *pBatteryCharacteristic;
uint8_t txPowerLevel = 12; // Use the same value as the one you set for advertising
BLECharacteristic *pTxPowerCharacteristic;

void setup() {
  
  Serial.begin(BAUD_RATE);
  Serial.println("Starting SphereLink ESP-32 Device!");

  BLEDevice::init(DEVICE_NAME);
  
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());  // Set the function that handles Server Callbacks
  
  
  BLEService *pService = pServer->createService(BATTERY_LEVEL_SERVICE_UUID);
  //Create the battery service and Characteristic
  pBatteryCharacteristic = pService->createCharacteristic(
                                         BATTERY_LEVEL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  pBatteryCharacteristic->addDescriptor(new BLE2902());
  pBatteryCharacteristic->setValue(&batteryLevel, 1);
  // Add the Characteristic User Description descriptor
  BLEDescriptor *pUserDescriptionDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  pUserDescriptionDescriptor->setValue("Percentage 0 - 100");
  pBatteryCharacteristic->addDescriptor(pUserDescriptionDescriptor);

  //TODO Fix, currently not working.  
  //Also May not be needed... Check into.
  // Create the TX Power service and characteristic
  BLEService *pTxPowerService = pServer->createService(TX_POWER_SERVICE_UUID);
  pTxPowerCharacteristic = pTxPowerService->createCharacteristic(
      TX_POWER_LEVEL_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ);
  // Set the initial value of the TX Power Level characteristic
  pTxPowerCharacteristic->setValue(&txPowerLevel, 1);


  pService->start();


  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(64);
  pAdvertising->addServiceUUID(BATTERY_LEVEL_SERVICE_UUID);
  pAdvertising->addServiceUUID(TX_POWER_SERVICE_UUID); 
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06); // Minimum connection interval: 7.5 ms
  pAdvertising->setMaxPreferred(0x12); // Maximum connection interval: 18 ms
  
  //TODO need to find way to set the TXpower, so we can match the value on the TX Service.
  //pAdvertising-> setPower(12);  
  BLEDevice::startAdvertising();

  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (deviceConnected) {

    unsigned long currentTime = millis();

    if (currentTime - previousBatteryUpdateTime >= batteryUpdateInterval) {

      previousBatteryUpdateTime = currentTime;

      //batteryLevel = convertVoltageToBatteryPercentage(readBatteryLevel());
      uint8_t batteryLevel =  random(1, 101);
      
      Serial.print("Battery level: ");
      Serial.println(batteryLevel);
      
      pBatteryCharacteristic->setValue(&batteryLevel, 1);
      pBatteryCharacteristic->notify();
    }
  }

  delay(2000);
}
