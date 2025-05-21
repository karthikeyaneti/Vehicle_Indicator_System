#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>

// Pin definitions
const uint8_t LEFT_LED = 33;
const uint8_t RIGHT_LED = 19;
const uint8_t LEFT_BUTTON = 36;
const uint8_t RIGHT_BUTTON = 22;

// UART Configuration
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD 115200
HardwareSerial SerialUART(2);

unsigned long lastHazardToggleTime = 0;
const unsigned long hazardCooldown = 100;

// Global variables
volatile bool leftIndicatorOn = false;
volatile bool rightIndicatorOn = false;
volatile bool hazardOn = false;
volatile unsigned long lastCycleStart = 0;
volatile unsigned long leftButtonPressTime = 0;
volatile unsigned long rightButtonPressTime = 0;
volatile bool leftButtonPressed = false;
volatile bool rightButtonPressed = false;
volatile bool leftButtonActive = false;
volatile bool rightButtonActive = false;
volatile bool bothButtonsPressed = false;
volatile unsigned long bothButtonsPressTime = 0;

// Gamma correction
const int maxDuty = 255;
const float gammaVal = 2.2;

// Timing constants (ms)
const int FADE_DURATION = 100;
const int HOLD_DURATION = 200;
const int OFF_HOLD_DURATION = 200;
const int STEP_DELAY = 3;
const int BUTTON_PRESS_DURATION = 1000; // 1 second for button press

// BLE Service UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8" // For sending to phone
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // For receiving from phone

void logEvent(const char* event) {
  // Send to both serial ports for debugging
  Serial.printf("%lu,%s,%d,%d,%d,%d,%d\n",
    millis(),
    event,
    leftIndicatorOn ? 1 : 0,
    rightIndicatorOn ? 1 : 0,
    hazardOn ? 1 : 0,
    digitalRead(LEFT_BUTTON),
    digitalRead(RIGHT_BUTTON)
  );
  
  SerialUART.printf("%lu,%s,%d,%d,%d,%d,%d\n\r",
    millis(),
    event,
    leftIndicatorOn ? 1 : 0,
    rightIndicatorOn ? 1 : 0,
    hazardOn ? 1 : 0,
    digitalRead(LEFT_BUTTON),
    digitalRead(RIGHT_BUTTON)
  );
  
  SerialUART.flush();
}

void handleBLECommand(String cmd);

BLECharacteristic *pCharacteristic;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
      logEvent("Bluetooth_Connected");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      logEvent("Bluetooth_Disconnected");
      pServer->startAdvertising(); // Restart advertising
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      handleBLECommand(value);
    }
};

void setupBLE() {
  BLEDevice::init("ESP32_Indicator");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for sending data to phone
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE Ready");
}

void sendDataToPhone(const String &message) {
  if (deviceConnected) {
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    Serial.println("Sent to phone: " + message);
  }
}

void handleBLECommand(String cmd) {
  // String cmd;
  // for (char c : command) {
  //   cmd += c;
  // }
  cmd.trim();

  if (cmd == "LEFT_ON") {
    leftIndicatorOn = true;
    rightIndicatorOn = false;
    hazardOn = false;
  } 
  else if (cmd == "RIGHT_ON") {
    leftIndicatorOn = false;
    rightIndicatorOn = true;
    hazardOn = false;
  }
  else if (cmd == "HAZARD_ON") {
    leftIndicatorOn = true;
    rightIndicatorOn = true;
    hazardOn = true;
  }
  else if (cmd == "ALL_OFF") {
    leftIndicatorOn = false;
    rightIndicatorOn = false;
    hazardOn = false;
  }
  
  updateIndicators();
  
  // For logging
  String logMsg = "BLE_CMD_" + cmd;
  logEvent(logMsg.c_str());
}

void setup() {
  delay(500);

  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Wait for serial ports to initialize
  while (!Serial && millis() < 2000); // Wait up to 2 seconds for Serial
  while (!SerialUART && millis() < 2000); // Wait up to 2 seconds for SerialUART
  
  Serial.println("USB Serial ready");
  SerialUART.println("UART Serial ready");
  SerialUART.println("Timestamp,Event,LeftIndicator,RightIndicator,Hazard,LeftButton,RightButton");

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  // Initialize PWM
  ledcAttach(LEFT_LED, 5000, 8);
  ledcAttach(RIGHT_LED, 5000, 8);

  ledcWrite(LEFT_LED, 0);
  ledcWrite(RIGHT_LED, 0);

  setupBLE();

}

void processButtonPresses() {
  bool leftPressed = digitalRead(LEFT_BUTTON) == HIGH;
  bool rightPressed = digitalRead(RIGHT_BUTTON) == HIGH;

  // Check for simultaneous press
  if (leftPressed && rightPressed) {
    if (!bothButtonsPressed) {
      bothButtonsPressTime = millis();
      bothButtonsPressed = true;
      logEvent("BOTH_BUTTONS_PRESSED");
    }
    return;
  } else {
    if (bothButtonsPressed) {
      if (millis() - bothButtonsPressTime >= BUTTON_PRESS_DURATION) {
        leftButtonPressed = true;
        rightButtonPressed = true;
        logEvent("BOTH_BUTTONS_HELD");
      }
      bothButtonsPressed = false;
    }
  }

  // Left button processing
  if (leftPressed) {
    if (!leftButtonActive) {
      leftButtonPressTime = millis();
      leftButtonActive = true;
      logEvent("LEFT_BUTTON_PRESSED");
    }
  } else {
    if (leftButtonActive && (millis() - leftButtonPressTime >= BUTTON_PRESS_DURATION)) {
      leftButtonPressed = true;
    }
    leftButtonActive = false;
  }

  // Right button processing
  if (rightPressed) {
    if (!rightButtonActive) {
      rightButtonPressTime = millis();
      rightButtonActive = true;
      logEvent("RIGHT_BUTTON_PRESSED");
    }
  } else {
    if (rightButtonActive && (millis() - rightButtonPressTime >= BUTTON_PRESS_DURATION)) {
      rightButtonPressed = true;
    }
    rightButtonActive = false;
  }
}

void handleButtonEvents() {
  unsigned long currentMillis = millis();
  //hazard activation (when both buttons are held)
  if (leftButtonPressed && rightButtonPressed) {
    if (currentMillis - lastHazardToggleTime > hazardCooldown) {
      leftButtonPressed = false;
      rightButtonPressed = false;
      
      hazardOn = !hazardOn;
      lastHazardToggleTime = currentMillis;

      if (hazardOn) {
        leftIndicatorOn = true;
        rightIndicatorOn = true;
        // Serial.println("Hazard lights activated");
        logEvent("HAZARD_LIGHTS_ACTIVATED");
      } else {
        leftIndicatorOn = false;
        rightIndicatorOn = false;
        logEvent("HAZARD_LIGHTS_DEACTIVATED");
      }
    }
    return;
  }

  if (currentMillis - lastHazardToggleTime <= hazardCooldown) {
    return; // Skip single-button handling during cooldown
  }

  // Handle left button press
  if (leftButtonPressed) {
    leftButtonPressed = false;
    
    if (hazardOn) {
      hazardOn = false;
      leftIndicatorOn = false;
      rightIndicatorOn = false;
      // Serial.println("Hazard lights deactivated by left button");
      logEvent("HAZARD_LIGHTS_DEACTIVATED");
    } else {
      if (rightIndicatorOn) {
        rightIndicatorOn = false;
        leftIndicatorOn = true;
        // Serial.println("Switched from right to left indicator");
        logEvent("RIGHT_INDICATOR_OFF");
        logEvent("LEFT_INDICATOR_ON");
      } else {
        leftIndicatorOn = !leftIndicatorOn;
        // Serial.println(leftIndicatorOn ? "Left indicator ON" : "Left indicator OFF");
        if(leftIndicatorOn) logEvent("LEFT_INDICATOR_ON");
        else logEvent("LEFT_INDICATOR_OFF");
      }
    }
  }

  // Handle right button press
  if (rightButtonPressed) {
    rightButtonPressed = false;
    
    if (hazardOn) {
      hazardOn = false;
      leftIndicatorOn = false;
      rightIndicatorOn = false;
      // Serial.println("Hazard lights deactivated by right button");
      logEvent("HAZARD_LIGHTS_DEACTIVATED");
    } else {
      if (leftIndicatorOn) {
        leftIndicatorOn = false;
        rightIndicatorOn = true;
        // Serial.println("Switched from left to right indicator");
        logEvent("LEFT_INDICATOR_OFF");
        logEvent("RIGHT_INDICATOR_ON");
      } else {
        rightIndicatorOn = !rightIndicatorOn;
        // Serial.println(rightIndicatorOn ? "Right indicator ON" : "Right indicator OFF");
        if(rightIndicatorOn) logEvent("RIGHT_INDICATOR_ON");
        else logEvent("RIGHT_INDICATOR_OFF");
      }
    }
  }
}

void updateIndicators() {
  static int fadeStep = 0;
  static bool fadingOut = false;
  static unsigned long lastStepTime = 0;
  static unsigned long holdStartTime = 0;
  static bool inHoldPeriod = false;

  if (hazardOn || leftIndicatorOn || rightIndicatorOn) {
    if (inHoldPeriod) {
      if (millis() - holdStartTime >= (fadingOut ? OFF_HOLD_DURATION : HOLD_DURATION)) {
        inHoldPeriod = false;
      }
      return;
    }

    if (millis() - lastStepTime >= STEP_DELAY) {
      lastStepTime = millis();

      float progress = fadeStep / 100.0;
      float corrected = pow(fadingOut ? (1 - progress) : progress, gammaVal);
      int duty = (int)(corrected * maxDuty);

      if (hazardOn) {
        ledcWrite(LEFT_LED, duty);
        ledcWrite(RIGHT_LED, duty);
      } else if (leftIndicatorOn) {
        ledcWrite(LEFT_LED, duty);
        ledcWrite(RIGHT_LED, 0);
      } else if (rightIndicatorOn) {
        ledcWrite(RIGHT_LED, duty);
        ledcWrite(LEFT_LED, 0);
      }

      fadeStep++;
      if (fadeStep > 100) {
        fadeStep = 0;
        fadingOut = !fadingOut;
        inHoldPeriod = true;
        holdStartTime = millis();
      }
    }
  } else {
    ledcWrite(LEFT_LED, 0);
    ledcWrite(RIGHT_LED, 0);
    fadeStep = 0;
    fadingOut = false;
    inHoldPeriod = false;
  }
}

void publishStatus() {
  static unsigned long lastPublishTime = 0;
  if (millis() - lastPublishTime >= 1000) {
    lastPublishTime = millis();
    logEvent("STATUS_UPDATE");
    String status = String("STATUS:") +
                   "LEFT=" + String(leftIndicatorOn ? "ON" : "OFF") + "," +
                   "RIGHT=" + String(rightIndicatorOn ? "ON" : "OFF") + "," +
                   "HAZARD=" + String(hazardOn ? "ON" : "OFF");
    
    sendDataToPhone(status);
  }
}

void loop() {
  processButtonPresses();
  handleButtonEvents();
  updateIndicators();
  publishStatus();
}