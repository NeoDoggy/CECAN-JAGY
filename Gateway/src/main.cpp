#include <driver/twai.h>
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(5);

MCP2515 mcp2515Out(15);

struct can_frame frame;
struct can_frame frameOut;

TaskHandle_t canTaskHandle = NULL;

// Function to compare two CAN frames
bool compareCANFrames(const can_frame &frame1, const twai_message_t &frame2) {
  if (frame1.can_id != frame2.identifier) return false;
  if (frame1.can_dlc != frame2.data_length_code) return false;
  for (int i = 0; i < frame1.can_dlc; i++) {
    if (frame1.data[i] != frame2.data[i]) return false;
  }
  return true;
}

void canTask(void* param) {
  while (true) {
    uint32_t alerts;
    // Wait for alerts indefinitely
    if (twai_read_alerts(&alerts, portMAX_DELAY) == ESP_OK) {
      if (alerts & TWAI_ALERT_RX_DATA) {
        twai_message_t rx_msg;
        // Receive the CAN message
        if (twai_receive(&rx_msg, 0) == ESP_OK) { // Non-blocking receive
          Serial.print("Message received from MCP2562 (TWAI): ID 0x");
          Serial.print(rx_msg.identifier, HEX);
          Serial.print(" Data: ");
          for (int i = 0; i < rx_msg.data_length_code; i++) {
            Serial.printf("%02X ", rx_msg.data[i]);
          }
          Serial.println();

          // Check if MCP2515 received a matching frame
          if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
            Serial.println("Comparing frames...");
            if (compareCANFrames(frame, rx_msg)) {
              Serial.println("Frames match! Sending via TWAI TX...");
              frameOut.can_id = frame.can_id;
              frameOut.can_dlc = frame.can_dlc;
              for (int i = 0; i < frame.can_dlc; i++) {
                frameOut.data[i] = frame.data[i];
              }
              digitalWrite(15, LOW);
              digitalWrite(5, HIGH);
              mcp2515Out.sendMessage(&frameOut);
              digitalWrite(15, HIGH);
              digitalWrite(5, LOW);
            } else {
              Serial.println("Frames do not match.");
            }
          }
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // General config: TX on GPIO21, RX on GPIO4
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_4, TWAI_MODE_NORMAL); // Switch to NORMAL mode for TX capability
  // Set to 500 kbps
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  // Accept all messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start the driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("CAN interface started in Normal Mode at 500 kbps");
  } else {
    Serial.println("Failed to start CAN in Normal Mode");
    return;
  }

  // Enable RX data alert
  twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, NULL);

  // Create a FreeRTOS task for handling CAN messages
  xTaskCreate(canTask, "CAN Task", 2048, NULL, 10, &canTaskHandle);

  // Initialize MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  mcp2515Out.reset();
  mcp2515Out.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515Out.setNormalMode();
}

void loop() {
  // The main loop is empty because the task handles all CAN operations
}