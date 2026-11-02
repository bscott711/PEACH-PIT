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
static const char* HOSTNAME = "peach-pit";
static const uint16_t SERIAL_BRIDGE_PORT = 6666;
static const uint16_t REMOTE_COMMAND_PORT = 6667; // Keep in sync with RemoteControl.cpp

// --- Console output / commands -------------------------------------------
// Replies go to both the USB serial monitor and the WiFi serial client. They
// are written directly (not via logQueue) so a multi-line reply can't be
// dropped when the queue is busy. Only call from the loop task (handle/init),
// which owns wifiSerialClient.
static void consolePrintf(const char* format, ...) {
    char buf[160];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf) - 2, format, args);
    va_end(args);
    if (len < 0) return;
    if (len > (int)sizeof(buf) - 3) len = sizeof(buf) - 3;
    buf[len++] = '\r';
    buf[len++] = '\n';

    Serial.write((const uint8_t*)buf, len);
    if (wifiSerialClient && wifiSerialClient.connected()) {
        wifiSerialClient.write((const uint8_t*)buf, len);
    }
}

static void printIP() {
    if (WiFi.status() == WL_CONNECTED) {
        consolePrintf("IP address: %s  (%s.local, monitor port %u, command port %u)",
                      WiFi.localIP().toString().c_str(), HOSTNAME, SERIAL_BRIDGE_PORT,
                      REMOTE_COMMAND_PORT);
    } else {
        consolePrintf("IP address: not connected to WiFi");
    }
}

static void printInfo() {
    if (WiFi.status() != WL_CONNECTED) {
        consolePrintf("WiFi: not connected (SSID %s)", ssid);
        consolePrintf("MAC: %s", WiFi.macAddress().c_str());
        return;
    }
    consolePrintf("Hostname: %s.local", HOSTNAME);
    consolePrintf("IP:       %s", WiFi.localIP().toString().c_str());
    consolePrintf("Gateway:  %s   Subnet: %s", WiFi.gatewayIP().toString().c_str(),
                  WiFi.subnetMask().toString().c_str());
    consolePrintf("SSID:     %s   RSSI: %d dBm", ssid, WiFi.RSSI());
    consolePrintf("MAC:      %s", WiFi.macAddress().c_str());
    consolePrintf("Monitor: %s:%u   Commands: %s:%u   OTA: %s:3232",
                  WiFi.localIP().toString().c_str(), SERIAL_BRIDGE_PORT,
                  WiFi.localIP().toString().c_str(), REMOTE_COMMAND_PORT,
                  WiFi.localIP().toString().c_str());
}

static void printHelp() {
    consolePrintf("Commands: ip (show IP address), info (full network info), help");
}

static void handleConsoleLine(char* line) {
    // Trim surrounding whitespace and lowercase for matching
    while (*line == ' ' || *line == '\t') line++;
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
    if (n == 0) return;
    if (line[0] == '#') return; // Remote protocol request (RemoteControl answers it)
    for (size_t i = 0; i < n; i++) line[i] = tolower((unsigned char)line[i]);

    if (strcmp(line, "ip") == 0) {
        printIP();
    } else if (strcmp(line, "info") == 0 || strcmp(line, "status") == 0 || strcmp(line, "net") == 0) {
        printInfo();
    } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        printHelp();
    } else {
        consolePrintf("Unknown command '%s'. Type 'help'.", line);
    }
}

// Accumulates characters from one input source until a newline, then runs the line.
struct ConsoleLineBuffer {
    char buf[32];
    size_t len = 0;
    bool overflow = false;

    void feed(const uint8_t* data, size_t count) {
        for (size_t i = 0; i < count; i++) {
            char c = (char)data[i];
            if (c == '\n' || c == '\r') {
                if (!overflow) {
                    buf[len] = '\0';
                    handleConsoleLine(buf);
                }
                len = 0;
                overflow = false;
            } else if (len < sizeof(buf) - 1) {
                buf[len++] = c;
            } else {
                overflow = true; // Too long to be a command; discard until newline
            }
        }
    }
};

static ConsoleLineBuffer usbLine;
static ConsoleLineBuffer wifiLine;

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
  draw_wifiStatus(WiFi.localIP().toString().c_str(), ssid, 0, false);
  delay(2000); // Show IP for 2 seconds (for clients that can't resolve .local)

  // Initialize ArduinoOTA
  ArduinoOTA.setHostname(HOSTNAME);

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
    if (MDNS.begin(HOSTNAME)) {
      Serial.println("mDNS responder started: peach-pit.local");
    }
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.begin();
    Serial.println("OTA Ready");

    // Start WiFi Serial Bridge
    wifiSerialServer.begin();
    wifiSerialServer.setNoDelay(true); // Reduces latency
    if (logQueue == NULL) {
        logQueue = xQueueCreate(20, 128); // 20 messages, 128 bytes each
    }

    Serial.println("\n--- WiFi Serial Bridge Ready on Port 6666 ---");
    printInfo();
    consolePrintf("Type 'ip' or 'info' at any time to show this again.");

    // DHCP can hand out a different address after a reconnect/lease change;
    // announce it so a stale IP isn't used by mistake. Registered after the
    // initial connect so it only reports changes after boot.
    WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t) {
      PEACH_LOGI("NET", "WiFi (re)connected, IP address: %s", WiFi.localIP().toString().c_str());
    }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

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
            wifiLine.len = 0; // Drop any partial command from a previous client
            wifiLine.overflow = false;
            printIP();
            printHelp();
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
        usbLine.feed(txBuf, len);
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
        wifiLine.feed(rxBuf, len);
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
