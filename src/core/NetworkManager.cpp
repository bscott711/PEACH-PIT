#include "core/NetworkManager.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "HardwareConfig.h"
#include "messaging.h"
#include "controller.h" // controlEvents, BIT_AUTO_RUNNING
#include "drivers/LCDDriver.h" // For draw_wifiStatus
#include "core/RemoteControl.h"
#include <esp_log.h>

// WiFi Serial Bridge
static WiFiServer wifiSerialServer(6666);
static WiFiClient wifiSerialClient;

static QueueHandle_t logQueue = NULL;

void NetworkManager::logToWiFi(const char* level, const char* tag, const char* format, ...) {
    char buf[128];
    // Format timestamp, level, and tag
    int prefixLen = snprintf(buf, sizeof(buf), "%s (%lu) %s: ", level, millis(), tag);
    
    if (prefixLen > 0 && prefixLen < sizeof(buf)) {
        va_list args;
        va_start(args, format);
        int msgLen = vsnprintf(buf + prefixLen, sizeof(buf) - prefixLen - 1, format, args);
        va_end(args);
        
        if (msgLen > 0) {
            int totalLen = prefixLen + msgLen;
            if (totalLen >= sizeof(buf) - 1) totalLen = sizeof(buf) - 2;
            buf[totalLen] = '\n';
            buf[totalLen + 1] = '\0';
            
            // Output to standard USB Serial
            Serial.print(buf);
            
            // Output to WiFi Queue
            if (logQueue != NULL) {
                if (xPortInIsrContext()) {
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    xQueueSendToBackFromISR(logQueue, buf, &xHigherPriorityTaskWoken);
                } else {
                    xQueueSendToBack(logQueue, buf, 0);
                }
            }
        }
    }
}

static volatile bool g_otaActive = false;
static volatile int g_otaProgress = 0;
static const char* g_otaStatus = "";

static const char* ssid = "sdsmtopn"; // From main.cpp

void NetworkManager::init() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Disable power save for reliable OTA (UDP/mDNS)
  WiFi.begin(ssid);

  int attempt = 0;
  Serial.println("Connecting to WiFi...");
  
  while (true) {
    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      break;
    }
    
    if (attempt >= 30) { // 15 seconds timeout
      Serial.println("Connection Failed! Rebooting...");
      draw_wifiStatus("Failed! Timeout", ssid, attempt, true);
      delay(5000);
      ESP.restart();
    }
    
    draw_wifiStatus("Connecting", ssid, attempt, false);
    delay(500);
    attempt++;
  }

  // Once connected
  Serial.println("WiFi Connected!");
  draw_wifiStatus("peach-pit.local", ssid, 0, false);
  delay(2000); // Show name for 2 seconds

  // Initialize ArduinoOTA
  ArduinoOTA.setHostname("peach-pit");

  ArduinoOTA
    .onStart([]() {
      g_otaActive = true;
      g_otaProgress = 0;
      
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
        g_otaStatus = "Updating Sketch";
      } else { // U_SPIFFS
        type = "filesystem";
        g_otaStatus = "Updating Filesystem";
      }
      Serial.println("Start updating " + type);
      
      // A running sequence would keep commanding motion during the update: abort it.
      if (controlEvents != NULL && (xEventGroupGetBits(controlEvents) & BIT_AUTO_RUNNING)) {
        xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST | BIT_AUTO_RESUME);
      }
      // Safety Interlocks
      if (motorCmdQueue != NULL) {
        MotorCommand stopMotor = { MotorCmdAction::SET_SPEED, 0.0f };
        xQueueSend(motorCmdQueue, &stopMotor, 0);
      }
      if (actuatorCmdQueue != NULL) {
        ActuatorCommand stopAct = { ActuatorCmdAction::SET_TARGET, 0 };
        ActuatorTelemetry actTel;
        if (actuatorTelQueue != NULL && xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
          stopAct.value = actTel.currentPercent;
        }
        xQueueSend(actuatorCmdQueue, &stopAct, 0);
      }
      if (armCmdQueue != NULL) {
        ArmCommand stopArm = { ArmCmdAction::SET_SPEED, 0.0f };
        xQueueSend(armCmdQueue, &stopArm, 0);
      }
    })
    .onEnd([]() {
      g_otaProgress = 100;
      g_otaStatus = "Success! Rebooting";
      Serial.println("\nEnd");
      g_otaActive = true;
      delay(1000);
      g_otaActive = false;
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      g_otaProgress = (progress * 100) / total;
      Serial.printf("Progress: %u%%\r", g_otaProgress);
    })
    .onError([](ota_error_t error) {
      g_otaActive = false;
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) g_otaStatus = "Auth Failed";
      else if (error == OTA_BEGIN_ERROR) g_otaStatus = "Begin Failed";
      else if (error == OTA_CONNECT_ERROR) g_otaStatus = "Connect Failed";
      else if (error == OTA_RECEIVE_ERROR) g_otaStatus = "Receive Failed";
      else if (error == OTA_END_ERROR) g_otaStatus = "End Failed";
    });

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin("peach-pit")) {
      Serial.println("mDNS responder started: peach-pit.local");
    }
    ArduinoOTA.setHostname("peach-pit");
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start WiFi Serial Bridge
    wifiSerialServer.begin();
    wifiSerialServer.setNoDelay(true); // Reduces latency
    if (logQueue == NULL) {
        logQueue = xQueueCreate(20, 128); // 20 messages, 128 bytes each
    }
    
    Serial.println("\n--- WiFi Serial Bridge Ready on Port 6666 ---");

    // Remote command server (port 6667) + request queues for the controller task
    RemoteControl::init();
  }
}

void NetworkManager::handle() {
    ArduinoOTA.handle();

    // --- WiFi Serial Bridge Handling ---
    
    // 1. Handle new connections from PlatformIO
    if (wifiSerialServer.hasClient()) {
        if (!wifiSerialClient || !wifiSerialClient.connected()) {
            if (wifiSerialClient) wifiSerialClient.stop(); // Kick old client
            wifiSerialClient = wifiSerialServer.available();
            wifiSerialClient.println("\n\n--- CONNECTED TO PEACH-PIT WIFI SERIAL ---");
        } else {
            wifiSerialServer.available().stop(); // Reject if already connected
        }
    }

    // 2. Forward ESP32 Serial output -> PlatformIO (Mac)
    // First, flush the logQueue to the WiFi client
    if (logQueue != NULL) {
        char logBuf[128];
        while (xQueueReceive(logQueue, logBuf, 0) == pdPASS) {
            if (wifiSerialClient && wifiSerialClient.connected()) {
                wifiSerialClient.write((uint8_t*)logBuf, strlen(logBuf));
            }
        }
    }

    // Also forward any typed USB console input to the WiFi client (as requested)
    if (Serial.available()) {
        uint8_t txBuf[128]; 
        size_t available = Serial.available();
        size_t toRead = available > sizeof(txBuf) ? sizeof(txBuf) : available;
        size_t len = Serial.readBytes(txBuf, toRead);
        if (wifiSerialClient && wifiSerialClient.connected()) {
            wifiSerialClient.write(txBuf, len);
        }
        // The same bytes are also remote commands (replies go back on USB)
        RemoteControl::feedUsb(txBuf, len);
    }

    // 3. Forward PlatformIO (Mac) keyboard input -> ESP32 Serial (TX)
    if (wifiSerialClient && wifiSerialClient.connected() && wifiSerialClient.available()) {
        uint8_t rxBuf[128];
        size_t available = wifiSerialClient.available();
        size_t toRead = available > sizeof(rxBuf) ? sizeof(rxBuf) : available;
        size_t len = wifiSerialClient.readBytes(rxBuf, toRead);
        Serial.write(rxBuf, len);
    }

    // --- Remote command port (6667) ---
    RemoteControl::handle();
}

bool NetworkManager::isOTAActive() {
    return g_otaActive;
}

int NetworkManager::getOTAProgress() {
    return g_otaProgress;
}

const char* NetworkManager::getOTAStatus() {
    return g_otaStatus;
}
