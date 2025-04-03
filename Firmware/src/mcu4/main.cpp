//
//
// RECEIVER TWO JOYSTICKS
//
//

// #include <WiFi.h>
// #include <esp_now.h>

// typedef struct {
//     float w, x, y, z;
//     int yaw, throttle, roll, pitch;
//     bool imuSent;
//     bool emergencyStop;
// } ControlData;

// ControlData receivedData;

// // Callback for receiving data
// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
//     memcpy(&receivedData, incomingData, sizeof(receivedData));

//     Serial.println("Received Data:");
    
//     Serial.print("Yaw: "); Serial.print(receivedData.yaw);
//     Serial.print(" | Throttle: "); Serial.print(receivedData.throttle);
//     Serial.print(" | Roll: "); Serial.print(receivedData.roll);
//     Serial.print(" | Pitch: "); Serial.println(receivedData.pitch);

//     if (receivedData.imuSent) {
//         Serial.print("\nIMU Data -> W: "); Serial.print(receivedData.w);
//         Serial.print(" | X: "); Serial.print(receivedData.x);
//         Serial.print(" | Y: "); Serial.print(receivedData.y);
//         Serial.print(" | Z: "); Serial.println(receivedData.z);
//     }

//     if (receivedData.emergencyStop) {
//         Serial.println("⚠️ EMERGENCY SHUTDOWN TRIGGERED! ⚠️");
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA);
//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);
// }

// void loop() {}

//
//
// RECEIVER SINGLE JOYSTICK
//
//

// #include <WiFi.h>
// #include <esp_now.h>

// typedef struct {
//     float w, x, y, z;
//     int yaw, throttle, roll, pitch;
//     bool imuSent;
//     bool emergencyStop;
//     bool mode1;
// } ControlData;

// ControlData receivedData;

// // Callback for receiving data
// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
//     memcpy(&receivedData, incomingData, sizeof(receivedData));

//     Serial.println("Received Data:");
//     Serial.print("Mode: "); Serial.println(receivedData.mode1 ? "Yaw/Throttle" : "Roll/Pitch");
    
//     Serial.print("Yaw: "); Serial.print(receivedData.yaw);
//     Serial.print(" | Throttle: "); Serial.print(receivedData.throttle);
//     Serial.print(" | Roll: "); Serial.print(receivedData.roll);
//     Serial.print(" | Pitch: "); Serial.println(receivedData.pitch);

//     if (receivedData.imuSent) {
//         Serial.print("\nIMU Data -> W: "); Serial.print(receivedData.w);
//         Serial.print(" | X: "); Serial.print(receivedData.x);
//         Serial.print(" | Y: "); Serial.print(receivedData.y);
//         Serial.print(" | Z: "); Serial.println(receivedData.z);
//     }

//     if (receivedData.emergencyStop) {
//         Serial.println("⚠️ EMERGENCY SHUTDOWN TRIGGERED! ⚠️");
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA);
//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);
// }

// void loop() {}

//
//
// RECEIVER BUTTONS
//
// 

// #include <WiFi.h>
// #include <esp_now.h>

// typedef struct {
//     float w, x, y, z;
//     bool yawLeft, yawRight, up, down, rollLeft, rollRight, pitchUp, pitchDown;
//     bool imuSent;
//     bool emergencyStop;
// } ControlData;

// ControlData receivedData;

// // Callback for receiving data
// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
//     memcpy(&receivedData, incomingData, sizeof(receivedData));

//     Serial.println("Received Data:");
//     Serial.print("Yaw Left: "); Serial.print(receivedData.yawLeft);
//     Serial.print(" | Yaw Right: "); Serial.print(receivedData.yawRight);
//     Serial.print(" | Up: "); Serial.print(receivedData.up);
//     Serial.print(" | Down: "); Serial.println(receivedData.down);
    
//     Serial.print("Roll Left: "); Serial.print(receivedData.rollLeft);
//     Serial.print(" | Roll Right: "); Serial.print(receivedData.rollRight);
//     Serial.print(" | Pitch Up: "); Serial.print(receivedData.pitchUp);
//     Serial.print(" | Pitch Down: "); Serial.println(receivedData.pitchDown);

//     if (receivedData.imuSent) {
//         Serial.print("\nIMU Data -> W: "); Serial.print(receivedData.w);
//         Serial.print(" | X: "); Serial.print(receivedData.x);
//         Serial.print(" | Y: "); Serial.print(receivedData.y);
//         Serial.print(" | Z: "); Serial.println(receivedData.z);
//     }

//     if (receivedData.emergencyStop) {
//         Serial.println("⚠️ EMERGENCY SHUTDOWN TRIGGERED! ⚠️");
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA);
//     esp_now_init();
//     esp_now_register_recv_cb(OnDataRecv);
// }

// void loop() {}

