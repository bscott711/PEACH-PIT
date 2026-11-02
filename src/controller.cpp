#include "controller.h"
#include "messaging.h"
#include "tasks/ActuatorNode.h"
#include "tasks/MotorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/EncoderDriver.h"
#include "drivers/LCDDriver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "core/StorageManager.h"
#include <cstdio>
#include "core/SequenceEngine.h"
#include "core/InputManager.h"
#include "core/RemoteControl.h"
// Global queue handles (declared extern in controller.h)
QueueHandle_t armCmdQueue;
QueueHandle_t armTelQueue;
QueueHandle_t actuatorCmdQueue;
QueueHandle_t actuatorTelQueue;
QueueHandle_t motorCmdQueue;
QueueHandle_t motorTelQueue;
QueueHandle_t lcdDataQueue;

// Global Node instances (defined in main.cpp, extern here)
extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;


SemaphoreHandle_t systemStateMutex;
SemaphoreHandle_t encoderStateMutex;
SemaphoreHandle_t tmcUartMutex;
EventGroupHandle_t controlEvents;

SystemState systemState = {.mode = IDLE,
                           .s4Menu = S4_STOP,
                           .s4SubMenu = 0,
                           .s4InSubMenu = false,
                           .s4InSpeedEdit = false,
                           .armJogSpeed = 5000,
                           .armGoSpeed = 5000,
                           .actJogSpeed = 128,
                           .actGoSpeed = 128,
                           .zJogSpeed = 5000,
                           .zGoSpeed = 5000,
                           .collisionDetected = false,
                           .collisionTimestamp = 0};

void initSystemState() {
  systemStateMutex = xSemaphoreCreateMutex();
  encoderStateMutex = xSemaphoreCreateMutex();
  tmcUartMutex = xSemaphoreCreateMutex();
  controlEvents = xEventGroupCreate();

  StorageManager::init();
  
  // Initialize minimal state - subsystem state is managed by Active Nodes
  if (xSemaphoreTake(systemStateMutex, portMAX_DELAY) == pdTRUE) {
    systemState.mode = IDLE;
    systemState.s4Menu = S4_STOP;
    systemState.s4SubMenu = 0;
    systemState.s4InSubMenu = false;
    systemState.s4InSpeedEdit = false;
    
    // Load speeds from NVS with defaults
    systemState.armJogSpeed = StorageManager::loadArmJogSpeed(5000);
    systemState.armGoSpeed = StorageManager::loadArmGoSpeed(5000);
    systemState.actJogSpeed = StorageManager::loadActuatorJogSpeed(128);
    systemState.actGoSpeed = StorageManager::loadActuatorGoSpeed(128);
    systemState.zJogSpeed = StorageManager::loadZJogSpeed(5000);
    systemState.zGoSpeed = StorageManager::loadZGoSpeed(5000);
    
    systemState.collisionDetected = false;
    xSemaphoreGive(systemStateMutex);
  }
}

// Removed unused state save wrappers

// ============================================================================
// Main Controller Task
// ============================================================================

void controller_task(void *pvParameters) {
  InputManager::init();

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t CONTROLLER_INTERVAL = pdMS_TO_TICKS(10);

  while (1) {
    // Process encoder inputs and dispatch commands
    InputManager::process();

    // Execute queued remote RUN/MOVE/JOG (same task as the encoder, so starts never race)
    RemoteControl::drainPending();

    // Build and push UIData
    UIData uiData;
    InputManager::populateUIData(uiData);
    if (lcdDataQueue) {
        xQueueOverwrite(lcdDataQueue, &uiData);
    }

    vTaskDelayUntil(&lastWakeTime, CONTROLLER_INTERVAL);

#ifdef DEBUG_STACK
    static uint32_t lastLog = 0;
    if (xTaskGetTickCount() - lastLog > pdMS_TO_TICKS(5000)) {
        printf("Controller_task stack free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
        lastLog = xTaskGetTickCount();
    }
#endif
  }
}



// Removed unused motor calculators
