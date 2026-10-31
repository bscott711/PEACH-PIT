#include "core/InputManager.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/MotorNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "core/SequenceEngine.h"
#include "core/NetworkManager.h"
#include <cstdio>
#include <cmath>

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

// ============================================================================
// Jog direction tracking (file-scoped for populateUIData access)
// ============================================================================
static int g_jogDirArm = 0;       // -1, 0, +1
static int g_jogDirActuator = 0;
static int g_jogDirMotor = 0;

// Configurable speeds are stored in systemState

void InputManager::init() {
  if (xSemaphoreTake(encoderStateMutex, portMAX_DELAY) == pdTRUE) {
    g_encoderState.position[0] = 0;
    g_encoderState.position[1] = 0;
    g_encoderState.position[2] = 0;
    g_encoderState.position[3] = 0;
    xSemaphoreGive(encoderStateMutex);
  }
}

void InputManager::process() {
    // 1. Button feedback
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (g_encoderState.buttonHeld[i]) {
                uint32_t duration = now - (g_encoderState.buttonPressTime[i] * portTICK_PERIOD_MS);
                if (duration >= 2500) {
                    LCD_setMessage("Very Long Press");
                } else if (duration >= 800) {
                    LCD_setMessage("Long Press");
                } else if (duration >= 100) {
                    LCD_setMessage("Press...");
                }
            }
        }
        xSemaphoreGive(encoderStateMutex);
    }

    handleArmEncoder();
    handleActuatorEncoder();
    handleMotorEncoder();
    handleMenuEncoder();
}

void InputManager::populateUIData(UIData& data) {
    if (xSemaphoreTake(systemStateMutex, 0) == pdTRUE) {
        data.currentMode = systemState.mode;
        data.s4Menu = systemState.s4Menu;
        data.s4SubMenu = systemState.s4SubMenu;
        data.s4InSubMenu = systemState.s4InSubMenu;
        data.s4InSpeedEdit = systemState.s4InSpeedEdit;
        
        data.armJogSpeed = systemState.armJogSpeed;
        data.armGoSpeed = systemState.armGoSpeed;
        data.actJogSpeed = systemState.actJogSpeed;
        data.actGoSpeed = systemState.actGoSpeed;
        data.zJogSpeed = systemState.zJogSpeed;
        data.zGoSpeed = systemState.zGoSpeed;
        xSemaphoreGive(systemStateMutex);
    }
    
    EventBits_t events = xEventGroupGetBits(controlEvents);
    data.isAutoRunning = (events & BIT_AUTO_RUNNING) != 0;

    // Jog direction indicators
    data.armJogDir = g_jogDirArm;
    data.actJogDir = g_jogDirActuator;
    data.zJogDir = g_jogDirMotor;

    // Arm telemetry
    ArmTelemetry armTel;
    if (xQueuePeek(armTelQueue, &armTel, 0) == pdPASS) {
        data.armPosition = armTel.currentPosition;
        data.armPosOut = armTel.posOut;
        data.armPosBuffer = armTel.posBuffer;
        data.armPosIn = armTel.posIn;
    }

    // Actuator telemetry
    ActuatorTelemetry actTel;
    if (xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
        data.actuatorPercent = (int)actTel.currentPercent;
        for(int i=0; i<3; i++) {
            data.actuatorLimits[i] = actTel.limits[i];
            data.actuatorLimitSet[i] = actTel.limitSet[i];
        }
    }

    // Motor telemetry
    MotorTelemetry motTel;
    if (xQueuePeek(motorTelQueue, &motTel, 0) == pdPASS) {
        data.motorPos = motTel.currentPosition;
        for(int i=0; i<3; i++) {
            data.motorLimits[i] = motTel.limits[i];
            data.motorLimitSet[i] = motTel.limitSet[i];
        }
    }
}

// ============================================================================
// S1: Arm — Simple directional jog
// ============================================================================
void InputManager::handleArmEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[0] - lastPos;
    lastPos = g_encoderState.position[0];

    if (g_encoderState.buttonPressed[0]) {
      btnPressed = true;
      g_encoderState.buttonPressed[0] = false;
    }
    // Clear unused button events for S1
    g_encoderState.buttonLongPressed[0] = false;
    g_encoderState.buttonDoublePressed[0] = false;
    xSemaphoreGive(encoderStateMutex);
  }

  // Encoder turn: set jog direction
  if (delta != 0) {
    int speed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.armJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirArm = 1;
      g_armNode.setSpeed(speed);
    } else {
      g_jogDirArm = -1;
      g_armNode.setSpeed(-speed);
    }
  }

  // Short press: stop motor
  if (btnPressed) {
    LCD_notifyButtonPress(0);
    g_jogDirArm = 0;
    g_armNode.setSpeed(0);
    LCD_setMessage("Arm: Stopped");
  }
}

// ============================================================================
// S2: Actuator — Simple directional jog
// ============================================================================
void InputManager::handleActuatorEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[1] - lastPos;
    lastPos = g_encoderState.position[1];

    if (g_encoderState.buttonPressed[1]) {
      btnPressed = true;
      g_encoderState.buttonPressed[1] = false;
    }
    // Clear unused button events for S2
    g_encoderState.buttonLongPressed[1] = false;
    g_encoderState.buttonDoublePressed[1] = false;
    xSemaphoreGive(encoderStateMutex);
  }

  // Read current actuator position for stop-in-place
  ActuatorTelemetry actTel;
  int currentPct = 50; // fallback
  if (xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
    currentPct = (int)actTel.currentPercent;
  }

  // Encoder turn: jog actuator
  if (delta != 0) {
    int speed = 128;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.actJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirActuator = 1;
      g_actuatorNode.setTarget(100, speed); // Extend toward 100%
    } else {
      g_jogDirActuator = -1;
      g_actuatorNode.setTarget(0, speed); // Retract toward 0%
    }
  }

  // Short press: stop actuator at current position
  if (btnPressed) {
    LCD_notifyButtonPress(1);
    g_jogDirActuator = 0;
    g_actuatorNode.setTarget(currentPct, 255);
    LCD_setMessage("Act: Stopped");
  }
}

// ============================================================================
// S3: Z Motor — Simple directional jog
// ============================================================================
void InputManager::handleMotorEncoder() {
  static int32_t lastPos = 0;
  int32_t delta = 0;
  bool btnPressed = false;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    delta = g_encoderState.position[2] - lastPos;
    lastPos = g_encoderState.position[2];

    if (g_encoderState.buttonPressed[2]) {
      btnPressed = true;
      g_encoderState.buttonPressed[2] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // Encoder turn: set jog direction
  if (delta != 0) {
    int speed = 5000;
    if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed = systemState.zJogSpeed;
      xSemaphoreGive(systemStateMutex);
    }
    if (delta > 0) {
      g_jogDirMotor = 1;
      g_motorNode.setSpeed(speed);
    } else {
      g_jogDirMotor = -1;
      g_motorNode.setSpeed(-speed);
    }
  }

  // Short press: stop motor
  if (btnPressed) {
    LCD_notifyButtonPress(2);
    g_jogDirMotor = 0;
    g_motorNode.setSpeed(0);
    LCD_setMessage("Z: Stopped");
  }

  // Auto-update jog indicator when motor is stopped by limit
  if (g_jogDirMotor != 0) {
    MotorTelemetry motTel;
    if (xQueuePeek(motorTelQueue, &motTel, 0) == pdPASS) {
      if (motTel.targetSpeed == 0) {
        g_jogDirMotor = 0; // Motor hit a limit
      }
    }
  }
}

// ============================================================================
// S4: Unified Hierarchical Menu
// ============================================================================

// Helper: get the number of items in the current sub-menu
static int getSubMenuCount(S4Level0 menu) {
  switch (menu) {
    case S4_ARM: return S4_ARM_COUNT;
    case S4_ACT: return S4_POS_COUNT;
    case S4_Z:   return S4_POS_COUNT;
    default:     return 0;
  }
}

void InputManager::handleMenuEncoder() {
  bool shortPress = false;
  bool longPress = false;
  bool doublePress = false;
  int32_t delta = 0;

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    static int32_t lastPos = 0;
    delta = g_encoderState.position[3] - lastPos;
    lastPos = g_encoderState.position[3];

    if (g_encoderState.buttonPressed[3]) {
      shortPress = true;
      g_encoderState.buttonPressed[3] = false;
    }
    if (g_encoderState.buttonLongPressed[3]) {
      longPress = true;
      g_encoderState.buttonLongPressed[3] = false;
    }
    if (g_encoderState.buttonDoublePressed[3]) {
      doublePress = true;
      g_encoderState.buttonDoublePressed[3] = false;
    }
    xSemaphoreGive(encoderStateMutex);
  }

  // --- Read telemetry for position operations ---
  ArmTelemetry armTel;
  int armPosOut = -1;
  int armPosBuffer = -1;
  int armPosIn = -1;
  float armCurrentPos = 0;
  if (armTelQueue != NULL && xQueuePeek(armTelQueue, &armTel, 0) == pdPASS) {
    armPosOut = armTel.posOut;
    armPosBuffer = armTel.posBuffer;
    armPosIn = armTel.posIn;
    armCurrentPos = armTel.currentPosition;
  }

  ActuatorTelemetry actTel;
  int actLimits[3] = {0};
  bool actLimitSet[3] = {false};
  if (xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
    for (int i = 0; i < 3; i++) {
      actLimits[i] = actTel.limits[i];
      actLimitSet[i] = actTel.limitSet[i];
    }
  }

  MotorTelemetry motorTel;
  float motorLimits[3] = {0};
  bool motorLimitSet[3] = {false};
  float motorCurrentPos = 0;
  if (xQueuePeek(motorTelQueue, &motorTel, 0) == pdPASS) {
    motorCurrentPos = motorTel.currentPosition;
    for (int i = 0; i < 3; i++) {
      motorLimits[i] = motorTel.limits[i];
      motorLimitSet[i] = motorTel.limitSet[i];
    }
  }

  // Check if auto is running — block menu actions
  EventBits_t events = xEventGroupGetBits(controlEvents);
  bool autoRunning = (events & BIT_AUTO_RUNNING) != 0;

  // ===========================
  // LEVEL 0: Axis selection
  // ===========================
  if (!systemState.s4InSubMenu) {
    // Encoder turn: cycle through Level 0 options
    if (delta != 0) {
      int sel = (int)systemState.s4Menu + delta;
      while (sel < 0) sel += S4_LEVEL0_COUNT;
      sel = sel % S4_LEVEL0_COUNT;
      systemState.s4Menu = (S4Level0)sel;
    }

    // Short press: enter sub-menu or launch Auto
    if (shortPress) {
      LCD_notifyButtonPress(3);

      if (autoRunning) {
        // E-STOP if auto is running
        xEventGroupSetBits(controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST);
        LCD_setMessage("Auto: STOPPING...");
        return;
      }

      if (systemState.s4Menu == S4_AUTO) {
        // Launch autonomous sequence (check prerequisites)
        bool canRun = motorLimitSet[0] && motorLimitSet[1] && motorLimitSet[2] &&
                      actLimitSet[0] && actLimitSet[1] && actLimitSet[2] &&
                      (armPosOut != -1) && (armPosIn != -1);
        
        if (canRun) {
          if (!startSequence(cmdproto::SeqName::AUTO, 0, nullptr)) {
            LCD_setMessage("Error: Task Failed");
            PEACH_LOGE("MENU", "Failed to create autonomous_task");
          }
        } else {
          LCD_setMessage("Missing Limits");
        }
      } else if (systemState.s4Menu == S4_STOP) {
        if (!startSequence(cmdproto::SeqName::PARK, 0, nullptr)) {
          LCD_setMessage("Error: Task Failed");
          PEACH_LOGE("MENU", "Failed to create shutdown task");
        }
      } else {
        // Enter sub-menu for Arm/Act/Z
        systemState.s4InSubMenu = true;
        systemState.s4SubMenu = 0;
      }
    }
    return;
  }

  // ===========================
  // LEVEL 1: Position sub-menu
  // ===========================
  int itemCount = getSubMenuCount(systemState.s4Menu);

  S4Level0 axis = systemState.s4Menu;
  int item = systemState.s4SubMenu;

  // Encoder turn: adjust speed OR cycle through sub-menu items
  if (delta != 0) {
    if (systemState.s4InSpeedEdit) {
      if (axis == S4_ARM) {
        if (item == S4_ARM_JOG_SPD) {
          int stepSize = (systemState.armJogSpeed <= 100 && delta < 0) || (systemState.armJogSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.armJogSpeed += delta * stepSize;
          if (systemState.armJogSpeed < 10) systemState.armJogSpeed = 10;
          if (systemState.armJogSpeed > 5000) systemState.armJogSpeed = 5000;
        } else if (item == S4_ARM_GO_SPD) {
          int stepSize = (systemState.armGoSpeed <= 100 && delta < 0) || (systemState.armGoSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.armGoSpeed += delta * stepSize;
          if (systemState.armGoSpeed < 10) systemState.armGoSpeed = 10;
          if (systemState.armGoSpeed > 5000) systemState.armGoSpeed = 5000;
        }
      } else if (axis == S4_ACT) {
        if (item == S4_POS_JOG_SPD) {
          systemState.actJogSpeed += delta * 10;
          if (systemState.actJogSpeed < 10) systemState.actJogSpeed = 10;
          if (systemState.actJogSpeed > 255) systemState.actJogSpeed = 255;
        } else if (item == S4_POS_GO_SPD) {
          systemState.actGoSpeed += delta * 10;
          if (systemState.actGoSpeed < 10) systemState.actGoSpeed = 10;
          if (systemState.actGoSpeed > 255) systemState.actGoSpeed = 255;
        }
      } else if (axis == S4_Z) {
        if (item == S4_POS_JOG_SPD) {
          int stepSize = (systemState.zJogSpeed <= 100 && delta < 0) || (systemState.zJogSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.zJogSpeed += delta * stepSize;
          if (systemState.zJogSpeed < 10) systemState.zJogSpeed = 10;
          if (systemState.zJogSpeed > 8500) systemState.zJogSpeed = 8500;
        } else if (item == S4_POS_GO_SPD) {
          int stepSize = (systemState.zGoSpeed <= 100 && delta < 0) || (systemState.zGoSpeed < 100 && delta > 0) ? 10 : 100;
          systemState.zGoSpeed += delta * stepSize;
          if (systemState.zGoSpeed < 10) systemState.zGoSpeed = 10;
          if (systemState.zGoSpeed > 8500) systemState.zGoSpeed = 8500;
        }
      }
    } else {
      int sel = systemState.s4SubMenu + delta;
      while (sel < 0) sel += itemCount;
      sel = sel % itemCount;
      systemState.s4SubMenu = sel;
      item = sel; // Update local item
    }
  }

  // --- Check for "Back" item ---
  bool isBack = false;
  if (axis == S4_ARM && item == S4_ARM_BACK) isBack = true;
  if ((axis == S4_ACT || axis == S4_Z) && item == S4_POS_BACK) isBack = true;

  // ---- Short Press: GOTO or Back ----
  if (shortPress) {
    LCD_notifyButtonPress(3);

    if (autoRunning) {
      xEventGroupSetBits(controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST);
      LCD_setMessage("Auto: STOPPING...");
      return;
    }

    if (isBack) {
      systemState.s4InSubMenu = false;
      return;
    }

    // Check if we are interacting with a Speed menu item
    bool isSpeedItem = false;
    if (axis == S4_ARM && (item == S4_ARM_JOG_SPD || item == S4_ARM_GO_SPD)) isSpeedItem = true;
    if ((axis == S4_ACT || axis == S4_Z) && (item == S4_POS_JOG_SPD || item == S4_POS_GO_SPD)) isSpeedItem = true;

    if (isSpeedItem) {
      if (systemState.s4InSpeedEdit) {
        // Exit edit mode and save
        systemState.s4InSpeedEdit = false;
        if (axis == S4_ARM) {
          StorageManager::saveArmJogSpeed(systemState.armJogSpeed);
          StorageManager::saveArmGoSpeed(systemState.armGoSpeed);
        } else if (axis == S4_ACT) {
          StorageManager::saveActuatorJogSpeed(systemState.actJogSpeed);
          StorageManager::saveActuatorGoSpeed(systemState.actGoSpeed);
        } else if (axis == S4_Z) {
          StorageManager::saveZJogSpeed(systemState.zJogSpeed);
          StorageManager::saveZGoSpeed(systemState.zGoSpeed);
        }
        LCD_setMessage("Speed Saved");
      } else {
        // Enter edit mode
        systemState.s4InSpeedEdit = true;
      }
      return;
    }

    // GOTO action based on axis and item
    if (axis == S4_ARM) {
      if (armPosOut == -1 || armPosIn == -1) {
        LCD_setMessage("Arm: Not Cal'd");
        return;
      }
      if (item == S4_ARM_TIP) {
        g_armNode.setTarget(100.0f, systemState.armGoSpeed);
        LCD_setMessage("Arm: Go Tip");
      } else if (item == S4_ARM_BUFFER) {
        if (armPosBuffer == -1) {
          LCD_setMessage("Arm: Buf Not Set");
          return;
        }
        float percent = 100.0f * (float)(armPosBuffer - armPosOut) / (float)(armPosIn - armPosOut);
        g_armNode.setTarget(percent, systemState.armGoSpeed);
        LCD_setMessage("Arm: Go Buffer");
      } else if (item == S4_ARM_CLEAR) {
        g_armNode.setTarget(0.0f, systemState.armGoSpeed);
        LCD_setMessage("Arm: Go Clear");
      }
    } else if (axis == S4_ACT) {
      int limitIdx = (item == S4_POS_TOP) ? 2 : ((item == S4_POS_BOT) ? 0 : 1);
      if (!actLimitSet[limitIdx]) {
        LCD_setMessage("Act: Not Set");
        return;
      }
      g_actuatorNode.setTarget(actLimits[limitIdx], systemState.actGoSpeed);
      LCD_setMessage("Act: GOTO");
    } else if (axis == S4_Z) {
      int limitIdx = (item == S4_POS_TOP) ? 2 : ((item == S4_POS_BOT) ? 0 : 1);
      if (!motorLimitSet[limitIdx]) {
        LCD_setMessage("Z: Not Set");
        return;
      }

      if (!startSequence(cmdproto::SeqName::GOTO, limitIdx, nullptr)) {
        LCD_setMessage("Error: Task Failed");
      }
    }
  }

  // ---- Long Press: SET current position ----
  if (longPress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_ARM) {
      if (item == S4_ARM_TIP) {
        g_armNode.setPosIn();
        LCD_setMessage("Arm: Tip Set");
        printf("Arm Tip (posIn) set at current position\n");
      } else if (item == S4_ARM_BUFFER) {
        g_armNode.setPosBuffer();
        LCD_setMessage("Arm: Buffer Set");
        printf("Arm Buffer (posBuffer) set at current position\n");
      } else if (item == S4_ARM_CLEAR) {
        g_armNode.setPosOut();
        LCD_setMessage("Arm: Clear Set");
        printf("Arm Clear (posOut) set at current position\n");
      }
    } else if (axis == S4_ACT) {
      int pct = (int)actTel.currentPercent;
      if (item == S4_POS_TOP) g_actuatorNode.setLimitTop(pct);
      else if (item == S4_POS_MID) g_actuatorNode.setLimitMid(pct);
      else if (item == S4_POS_BOT) g_actuatorNode.setLimitBot(pct);
      LCD_setMessage("Act: Position Set");
      printf("Actuator limit %d set to %d%%\n", item, pct);
    } else if (axis == S4_Z) {
      if (item == S4_POS_TOP) {
          g_motorNode.setLimitTop(motorCurrentPos);
      }
      else if (item == S4_POS_MID) g_motorNode.setLimitMid(motorCurrentPos);
      else if (item == S4_POS_BOT) g_motorNode.setLimitBot(motorCurrentPos);
      LCD_setMessage("Z: Position Set");
      printf("Motor limit %d set to %.2f\n", item, motorCurrentPos);
    }
  }

  // ---- Double Press: CLEAR position ----
  if (doublePress && !isBack && systemState.s4InSubMenu) {
    if (axis == S4_ARM) {
      // Clear both arm calibration points
      g_armNode.clearCal();
      LCD_setMessage("Arm: Cal Cleared");
      printf("Arm calibration cleared\n");
    } else if (axis == S4_ACT) {
      if (item == S4_POS_TOP) g_actuatorNode.clearLimitTop();
      else if (item == S4_POS_MID) g_actuatorNode.clearLimitMid();
      else if (item == S4_POS_BOT) g_actuatorNode.clearLimitBot();
      LCD_setMessage("Act: Cleared");
      printf("Actuator limit %d cleared\n", item);
    } else if (axis == S4_Z) {
      if (item == S4_POS_TOP) g_motorNode.clearLimitTop();
      else if (item == S4_POS_MID) g_motorNode.clearLimitMid();
      else if (item == S4_POS_BOT) g_motorNode.clearLimitBot();
      LCD_setMessage("Z: Cleared");
      printf("Motor limit %d cleared\n", item);
    }
  }
}
