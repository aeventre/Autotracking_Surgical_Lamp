// 
//
// TWO JOYSTICK CODE SENDING DATA
//
//

// #include <Wire.h>
// #include <Adafruit_BNO08x.h>
// #include <WiFi.h>
// #include <esp_now.h>

// Adafruit_BNO08x bno;

// // Receiver MAC Address (Replace with actual receiver MAC)
// uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX};

// // Left Joystick (Yaw & Throttle)
// #define JOYSTICK_LX 34  // Yaw Left/Right
// #define JOYSTICK_LY 35  // Throttle Up/Down

// // Right Joystick (Roll & Pitch)
// #define JOYSTICK_RX 32  // Roll Left/Right
// #define JOYSTICK_RY 33  // Pitch Forward/Back

// // Buttons
// #define IMU_SEND_BUTTON 26
// #define EMERGENCY_BUTTON 27

// // IMU Send Timer
// unsigned long buttonPressStart = 0;
// bool imuEnabled = false;
// bool emergencyState = false;
// bool lastEmergencyButtonState = HIGH;

// // Data Structure
// typedef struct {
//     float w, x, y, z;  // Quaternion
//     int yaw, throttle, roll, pitch;  // Movement values (-100 to 100)
//     bool imuSent;
//     bool emergencyStop;
// } ControlData;

// ControlData controlData;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send Success" : "Send Fail");
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();

//     // Initialize Joysticks & Buttons
//     pinMode(IMU_SEND_BUTTON, INPUT_PULLUP);
//     pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

//     if (!bno.begin()) {
//         Serial.println("Failed to find BNO08x sensor!");
//         while (1);
//     }
//     bno.enableReport(SH2_GAME_ROTATION_VECTOR);

//     WiFi.mode(WIFI_STA);
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW init failed!");
//         return;
//     }
//     esp_now_register_send_cb(OnDataSent);

//     esp_now_peer_info_t peerInfo;
//     memcpy(peerInfo.peer_addr, receiverMAC, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;

//     if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println("Failed to add peer");
//         return;
//     }
// }

// void loop() {
//     // Emergency Button Toggle
//     bool currentEmergencyButtonState = digitalRead(EMERGENCY_BUTTON);
//     if (currentEmergencyButtonState == LOW && lastEmergencyButtonState == HIGH) {  
//         emergencyState = !emergencyState;
//         Serial.print("Emergency State: ");
//         Serial.println(emergencyState ? "ON (Shutdown)" : "OFF (Normal)");
//     }
//     lastEmergencyButtonState = currentEmergencyButtonState;

//     // IMU Send Button (Hold for 2 seconds)
//     if (digitalRead(IMU_SEND_BUTTON) == LOW) {
//         if (buttonPressStart == 0) {
//             buttonPressStart = millis();
//         }
//         if (millis() - buttonPressStart >= 2000) {  
//             imuEnabled = true;
//         }
//     } else {
//         buttonPressStart = 0;
//         imuEnabled = false;
//     }

//     // Read Joystick Values (Normalize to -100 to 100)
//     int lx = analogRead(JOYSTICK_LX);
//     int ly = analogRead(JOYSTICK_LY);
//     int rx = analogRead(JOYSTICK_RX);
//     int ry = analogRead(JOYSTICK_RY);

//     controlData.yaw = map(lx, 0, 4095, -100, 100);
//     controlData.throttle = map(ly, 0, 4095, -100, 100);
//     controlData.roll = map(rx, 0, 4095, -100, 100);
//     controlData.pitch = map(ry, 0, 4095, -100, 100);

//     // IMU Data
//     controlData.imuSent = imuEnabled;
//     controlData.emergencyStop = emergencyState;

//     if (imuEnabled) {
//         sensors_event_t event;
//         bno.getEvent(&event);
//         controlData.w = event.orientation.w;
//         controlData.x = event.orientation.x;
//         controlData.y = event.orientation.y;
//         controlData.z = event.orientation.z;
//     } else {
//         controlData.w = controlData.x = controlData.y = controlData.z = 0;
//     }

//     // Send Data via ESP-NOW
//     esp_now_send(receiverMAC, (uint8_t *)&controlData, sizeof(controlData));

//     delay(50);  // Prevent excessive sending
// }

//
//
// SINGLE JOYSTICK SENDING CODE
//
//

// #include <Wire.h>
// #include <Adafruit_BNO08x.h>
// #include <WiFi.h>
// #include <esp_now.h>

// Adafruit_BNO08x bno;

// // Receiver MAC Address
// uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX};

// // Joystick Pins
// #define JOYSTICK_X 34  // Left/Right
// #define JOYSTICK_Y 35  // Up/Down
// #define JOYSTICK_BUTTON 32  // Press to switch mode

// // Buttons
// #define IMU_SEND_BUTTON 26
// #define EMERGENCY_BUTTON 27

// // Joystick Parameters
// bool isMode1 = true;  // Default: Mode 1 (Yaw + Throttle)

// // IMU Send Timer
// unsigned long buttonPressStart = 0;
// bool imuEnabled = false;
// bool emergencyState = false;
// bool lastEmergencyButtonState = HIGH;

// // Data Structure
// typedef struct {
//     float w, x, y, z;  // Quaternion
//     int yaw, throttle, roll, pitch; // Movement values (-100 to 100)
//     bool imuSent;
//     bool emergencyStop;
//     bool mode1;  // Mode indicator
// } ControlData;

// ControlData controlData;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send Success" : "Send Fail");
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();

//     // Initialize joystick & buttons
//     pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
//     pinMode(IMU_SEND_BUTTON, INPUT_PULLUP);
//     pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

//     if (!bno.begin()) {
//         Serial.println("Failed to find BNO08x sensor!");
//         while (1);
//     }
//     bno.enableReport(SH2_GAME_ROTATION_VECTOR);

//     WiFi.mode(WIFI_STA);
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW init failed!");
//         return;
//     }
//     esp_now_register_send_cb(OnDataSent);

//     esp_now_peer_info_t peerInfo;
//     memcpy(peerInfo.peer_addr, receiverMAC, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;

//     if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println("Failed to add peer");
//         return;
//     }
// }

// void loop() {
//     // Emergency Button Toggle
//     bool currentEmergencyButtonState = digitalRead(EMERGENCY_BUTTON);
//     if (currentEmergencyButtonState == LOW && lastEmergencyButtonState == HIGH) {  
//         emergencyState = !emergencyState;
//         Serial.print("Emergency State: ");
//         Serial.println(emergencyState ? "ON (Shutdown)" : "OFF (Normal)");
//     }
//     lastEmergencyButtonState = currentEmergencyButtonState;

//     // IMU Send Button (Hold for 2 seconds)
//     if (digitalRead(IMU_SEND_BUTTON) == LOW) {
//         if (buttonPressStart == 0) {
//             buttonPressStart = millis();
//         }
//         if (millis() - buttonPressStart >= 2000) {  
//             imuEnabled = true;
//         }
//     } else {
//         buttonPressStart = 0;
//         imuEnabled = false;
//     }

//     // Switch Mode When Joystick Button is Pressed
//     if (digitalRead(JOYSTICK_BUTTON) == LOW) {
//         delay(200);  // Debounce
//         isMode1 = !isMode1;
//     }

//     // Read Joystick Values (Normalize to -100 to 100)
//     int joystickX = analogRead(JOYSTICK_X);
//     int joystickY = analogRead(JOYSTICK_Y);
//     int mappedX = map(joystickX, 0, 4095, -100, 100);
//     int mappedY = map(joystickY, 0, 4095, -100, 100);

//     // Assign Values Based on Mode
//     if (isMode1) {  
//         controlData.yaw = mappedX;
//         controlData.throttle = mappedY;
//         controlData.roll = 0;
//         controlData.pitch = 0;
//     } else {  
//         controlData.yaw = 0;
//         controlData.throttle = 0;
//         controlData.roll = mappedX;
//         controlData.pitch = mappedY;
//     }

//     // IMU Data
//     controlData.imuSent = imuEnabled;
//     controlData.emergencyStop = emergencyState;
//     controlData.mode1 = isMode1;  // Send current mode

//     if (imuEnabled) {
//         sensors_event_t event;
//         bno.getEvent(&event);
//         controlData.w = event.orientation.w;
//         controlData.x = event.orientation.x;
//         controlData.y = event.orientation.y;
//         controlData.z = event.orientation.z;
//     } else {
//         controlData.w = controlData.x = controlData.y = controlData.z = 0;
//     }

//     // Send Data via ESP-NOW
//     esp_now_send(receiverMAC, (uint8_t *)&controlData, sizeof(controlData));
    
//     delay(50);  // Prevent excessive sending
// }

//
//
// BUTTON SEND CODE
//
//

// #include <Wire.h>
// #include <Adafruit_BNO08x.h>
// #include <WiFi.h>
// #include <esp_now.h>

// Adafruit_BNO08x bno;

// // Receiver ESP32 MAC Address
// uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX};

// // Button Pins
// #define YAW_LEFT 32
// #define YAW_RIGHT 33
// #define UP 34
// #define DOWN 35
// #define ROLL_LEFT 12
// #define ROLL_RIGHT 13
// #define PITCH_UP 14
// #define PITCH_DOWN 15
// #define IMU_SEND_BUTTON 26
// #define EMERGENCY_BUTTON 27

// // IMU Send Timer
// unsigned long buttonPressStart = 0;
// bool imuEnabled = false;
// bool emergencyState = false;
// bool lastEmergencyButtonState = HIGH;

// // Data Structure
// typedef struct {
//     float w, x, y, z;  // Quaternion
//     bool yawLeft, yawRight, up, down, rollLeft, rollRight, pitchUp, pitchDown;
//     bool imuSent;
//     bool emergencyStop;
// } ControlData;

// ControlData controlData;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     Serial.print("Send status: ");
//     Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();

//     // Initialize Buttons
//     pinMode(YAW_LEFT, INPUT_PULLUP);
//     pinMode(YAW_RIGHT, INPUT_PULLUP);
//     pinMode(UP, INPUT_PULLUP);
//     pinMode(DOWN, INPUT_PULLUP);
//     pinMode(ROLL_LEFT, INPUT_PULLUP);
//     pinMode(ROLL_RIGHT, INPUT_PULLUP);
//     pinMode(PITCH_UP, INPUT_PULLUP);
//     pinMode(PITCH_DOWN, INPUT_PULLUP);
//     pinMode(IMU_SEND_BUTTON, INPUT_PULLUP);
//     pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

//     if (!bno.begin()) {
//         Serial.println("Failed to find BNO08x sensor!");
//         while (1);
//     }
//     bno.enableReport(SH2_GAME_ROTATION_VECTOR);

//     WiFi.mode(WIFI_STA);
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW init failed!");
//         return;
//     }
//     esp_now_register_send_cb(OnDataSent);

//     esp_now_peer_info_t peerInfo;
//     memcpy(peerInfo.peer_addr, receiverMAC, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;

//     if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println("Failed to add peer");
//         return;
//     }
// }

// void loop() {
//     // Emergency Button Toggle
//     bool currentEmergencyButtonState = digitalRead(EMERGENCY_BUTTON);
//     if (currentEmergencyButtonState == LOW && lastEmergencyButtonState == HIGH) {  
//         emergencyState = !emergencyState; // Toggle emergency state
//         Serial.print("Emergency State: ");
//         Serial.println(emergencyState ? "ON (Shutdown)" : "OFF (Normal)");
//     }
//     lastEmergencyButtonState = currentEmergencyButtonState;

//     // IMU Send Button (Hold for 2 seconds)
//     if (digitalRead(IMU_SEND_BUTTON) == LOW) {
//         if (buttonPressStart == 0) {
//             buttonPressStart = millis();
//         }
//         if (millis() - buttonPressStart >= 2000) {  // Held for 2 seconds
//             imuEnabled = true;
//         }
//     } else {
//         buttonPressStart = 0;
//         imuEnabled = false;
//     }

//     // **Real-Time Button States (Set 0 When Released)**
//     controlData.yawLeft = (digitalRead(YAW_LEFT) == LOW) ? 1 : 0;
//     controlData.yawRight = (digitalRead(YAW_RIGHT) == LOW) ? 1 : 0;
//     controlData.up = (digitalRead(UP) == LOW) ? 1 : 0;
//     controlData.down = (digitalRead(DOWN) == LOW) ? 1 : 0;
//     controlData.rollLeft = (digitalRead(ROLL_LEFT) == LOW) ? 1 : 0;
//     controlData.rollRight = (digitalRead(ROLL_RIGHT) == LOW) ? 1 : 0;
//     controlData.pitchUp = (digitalRead(PITCH_UP) == LOW) ? 1 : 0;
//     controlData.pitchDown = (digitalRead(PITCH_DOWN) == LOW) ? 1 : 0;

//     // IMU Data
//     controlData.imuSent = imuEnabled;
//     controlData.emergencyStop = emergencyState;

//     if (imuEnabled) {
//         sensors_event_t event;
//         bno.getEvent(&event);
//         controlData.w = event.orientation.w;
//         controlData.x = event.orientation.x;
//         controlData.y = event.orientation.y;
//         controlData.z = event.orientation.z;
//     } else {
//         controlData.w = controlData.x = controlData.y = controlData.z = 0;
//     }

//     // Send Data via ESP-NOW
//     esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&controlData, sizeof(controlData));

//     if (result == ESP_OK) {
//         Serial.println("Data Sent Successfully!");
//     } else {
//         Serial.println("Failed to send data");
//     }

//     delay(50);  // Small delay to prevent spamming
// }

