#include <esp_now.h>
#include <WiFi.h>

// Data Packet Structure (to receive joystick/button and SPI sensor data)
struct __attribute__((packed)) dataPacket {
  int xValue;
  int yValue;
  int button1State;
  int button2State;
  int button3State;
  int joyButtonState;
  float x, y, z, w;  // Quaternion values from the sensor
};

int lightMode = 1;  // Starts at 1
int lastYDir = 0;   // To avoid repeat increments
const int lightMin = 1;
const int lightMax = 5;

// Adjusted dead zone threshold and midpoint
const int DEAD_ZONE = 500;  // Increased dead zone for more stability around the center
const int MID_POINT = 2047; // Midpoint for 12-bit ADC values (0-4095)

// Map joystick values to binary for up/down (Y-axis)
int mapToBinary(int value, int axis) {
  if (axis == 1) {  // Y-axis
    if (value < (MID_POINT - DEAD_ZONE)) return 1;  // Up
    else if (value > (MID_POINT + DEAD_ZONE)) return 2;  // Down
  }
  return 0;  // Neutral
}

void OnDataRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
  dataPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  // Process data only when button1State is pressed (i.e., button1State == 1)
  if (packet.button1State == 1) {
    // Handle Y-axis to detect UP/DOWN for light mode adjustment
    int yDir = mapToBinary(packet.yValue, 1);

    // Check for movement in Y direction
    if (yDir != lastYDir) {
      if (yDir == 2 && lightMode < lightMax) {  // Moving Up
        lightMode++;
      } else if (yDir == 1 && lightMode > lightMin) {  // Moving Down
        lightMode--;
      }
      lastYDir = yDir;
    }

    // Send data (button states, light mode, and quaternion values)
    int resetState = packet.button2State;  // Button 2 is used as reset
    int buttonState = packet.button1State; // Button 1
    Serial.print("<");
    Serial.print(packet.x); // Quart: x
    Serial.print(",");
    Serial.print(packet.y); // Quart: y
    Serial.print(",");
    Serial.print(packet.y); // Quart: z
    Serial.print(",");
    Serial.print(packet.w); // Quart: w
    Serial.print(",");
    Serial.print(buttonState);  // Button 1 state
    Serial.print(",");
    Serial.print(lightMode);    // Light mode (based on Y-axis movement)
    Serial.print(",");
    Serial.print(resetState);   // Button 2 state (reset)
    Serial.println(">");
  } else {
    // If button1 isn't pressed, do nothing (no data sent)
    // Optionally, you can print a message for debugging purposes:
    // Serial.println("Button 1 not pressed, no data sent.");
  }
}

void setup() {
  Serial.begin(115200);

  // ESP-NOW setup
  WiFi.mode(WIFI_AP_STA);
  WiFi.channel(1);  // Must match sender channel

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function to handle incoming data
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver ready to receive data...");
}

void loop() {
  // Nothing to do in the loop, just waiting for data
}
