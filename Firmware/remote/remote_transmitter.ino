#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>

// Pin definitions for SPI and sensor
#define BNO_CS_PIN   5   // Chip Select (CS)
#define BNO_INT_PIN  4   // Data-ready interrupt (active LOW)
#define BNO_RST_PIN  2   // Reset (optional, active LOW)

// ESP32 SPI pins
#define SCK_PIN      18  // SPI Clock (SCK/SCL)
#define MISO_PIN     19  // Master In Slave Out (MISO/SDA)
#define MOSI_PIN     23  // Master Out Slave In (MOSI/DI)

// Button pin definitions (Ensure no overlap with SPI pins)
#define BUTTON_1_PIN 25
#define BUTTON_2_PIN 26
#define BUTTON_3_PIN 27
#define JOY_X_PIN    34  // Joystick X-axis (analog)
#define JOY_Y_PIN    35  // Joystick Y-axis (analog)

// Create the BNO object, telling it which pin drives RESET
Adafruit_BNO08x bno(BNO_RST_PIN);

// Data Packet Structure to send joystick/button and SPI sensor data
struct __attribute__((packed)) dataPacket {
  int xValue;
  int yValue;
  int button1State;
  int button2State;
  int button3State;
  int joyButtonState;
  float x, y, z, w;  // Quaternion values from the sensor
};

uint8_t broadcastAddress[] = {0xcc, 0xdb, 0xa7, 0x3e, 0xe5, 0x14};  // Receiver's MAC address
esp_now_peer_info_t peerInfo;

// Container for incoming sensor data
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Set up pin modes for joystick buttons and analog pins
  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  
  // Setup the BNO085 sensor
  bno.hardwareReset();
  delay(100);

  // Initialize SPI with custom pins
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  if (!bno.begin_SPI(BNO_CS_PIN, BNO_INT_PIN)) {
    Serial.println("ERROR: BNO085 not found over SPI!");
    while (1) delay(10);
  }
  Serial.println("✓ BNO085 detected");

  // Start the rotation vector report
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Failed to enable rotation vector");
    while (1) delay(10);
  }
  Serial.println("Streaming rotation vectors now!");

  // ESP-NOW Setup
  WiFi.mode(WIFI_AP_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW Initialized and Peer Added Successfully");
}

void loop() {
  dataPacket packet;

  // Read joystick analog values
  packet.xValue = analogRead(JOY_X_PIN);
  packet.yValue = analogRead(JOY_Y_PIN);

  // Read buttons (LOW = pressed)
  packet.button1State = digitalRead(BUTTON_1_PIN) == LOW ? 1 : 0;
  packet.button2State = digitalRead(BUTTON_2_PIN) == LOW ? 1 : 0;
  packet.button3State = digitalRead(BUTTON_3_PIN) == LOW ? 1 : 0;

  // Get the quaternion data from the sensor
  if (!bno.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    // Quaternion data
    packet.x = sensorValue.un.rotationVector.i;
    packet.y = sensorValue.un.rotationVector.j;
    packet.z = sensorValue.un.rotationVector.k;
    packet.w = sensorValue.un.rotationVector.real;
  }

  // Send the data packet
  esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet));

  // Debugging (optional)
  Serial.print("X: "); Serial.print(packet.xValue);
  Serial.print(" | Y: "); Serial.print(packet.yValue);
  Serial.print(" | Button1: "); Serial.print(packet.button1State);
  Serial.print(" | Button2: "); Serial.print(packet.button2State);
  Serial.print(" | Button3: "); Serial.print(packet.button3State);
  Serial.print(" | Quat: "); Serial.print(packet.x); Serial.print(", ");
  Serial.print(packet.y); Serial.print(", ");
  Serial.print(packet.z); Serial.print(", ");
  Serial.println(packet.w);

  delay(100);  // Adjust delay as needed
}