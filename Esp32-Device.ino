#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <time.h>
#include <stdlib.h>
#include <driver/adc.h> // Include the required header for the ADC
#include <BLEAdvertisedDevice.h>

#define BATTERY_LEVEL_SERVICE_UUID        "180F"
#define BATTERY_LEVEL_CHARACTERISTIC_UUID "2A19"

#define BAUD_RATE 115200

//device name should change and be unique for each device
#define DEVICE_NAME "SphereLink-5467"

// Define the ADC pin and conversion factor
#define BATTERY_PIN 34
#define VOLTAGE_DIVIDER_RATIO 2.0 // Adjust this based on your voltage divider

bool deviceConnected = false;

unsigned long previousBatteryUpdateTime = 0;
unsigned long batteryUpdateInterval = 600000; // 10 minutes in milliseconds


// class used to easily verify whether the device is connected.
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

uint8_t batteryLevel = 100;
BLECharacteristic *pBatteryCharacteristic;

void setup() {
  
  Serial.begin(BAUD_RATE);
  Serial.println("Starting BLE work!");

  // Configure the ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());  // Set the function that handles Server Callbacks
  BLEService *pService = pServer->createService(BATTERY_LEVEL_SERVICE_UUID);

  //Create the battery Characteristic
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

  //Add additional characteristics

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(64);
  pAdvertising->addServiceUUID(BATTERY_LEVEL_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Minimum connection interval: 7.5 ms
  pAdvertising->setMaxPreferred(0x12); // Maximum connection interval: 18 ms
  BLEDevice::startAdvertising();

  Serial.println("Characteristic and battery level defined! Now you can read them in your phone!");
}

void loop() {

  if (deviceConnected) {

    unsigned long currentTime = millis();
    if (currentTime - previousBatteryUpdateTime >= batteryUpdateInterval) {
    previousBatteryUpdateTime = currentTime;
    batteryLevel = convertVoltageToBatteryPercentage(readBatteryLevel());
    pBatteryCharacteristic->setValue(&batteryLevel, 1);
    pBatteryCharacteristic->notify();
    Serial.print("Battery level: ");
    Serial.println(batteryLevel);
    }
  }

  delay(2000);
}

uint8_t readBatteryLevel() {

    // Read the battery voltage
    int adc_raw = adc1_get_raw(ADC1_CHANNEL_0);
    float voltage = ((float)adc_raw / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
}

// This function converts the battery voltage to a percentage based on the battery's discharge curve
uint8_t convertVoltageToBatteryPercentage(float voltage) {

  // Replace the following lines with the conversion logic specific to your battery's discharge curve
  float minVoltage = 3.0; // Minimum voltage when the battery is considered empty
  float maxVoltage = 5.0; // Maximum voltage when the battery is considered full

  uint8_t percentage = (uint8_t)((voltage - minVoltage) / (maxVoltage - minVoltage) * 100);
  return percentage;
}
