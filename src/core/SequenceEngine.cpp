#include "core/SequenceEngine.h"
#include "controller.h"
#include "messaging.h"
#include "tasks/MotorNode.h"
#include "tasks/ActuatorNode.h"
#include "tasks/ArmNode.h"
#include "drivers/LCDDriver.h"
#include "drivers/EncoderDriver.h"
#include "esp_log.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

// ---------------------------------------------------------------------------
// Run status record (see SequenceEngine.h). Critical sections are copies only.
// ---------------------------------------------------------------------------
static portMUX_TYPE s_statusMux = portMUX_INITIALIZER_UNLOCKED;
static SequenceStatus s_status = {0, cmdproto::SeqName::NONE, 0, 0, cmdproto::Result::NONE};

SequenceStatus sequenceStatus() {
  portENTER_CRITICAL(&s_statusMux);
  SequenceStatus copy = s_status;
  portEXIT_CRITICAL(&s_statusMux);
  return copy;
}

static uint32_t statusBegin(cmdproto::SeqName seq, int steps) {
  portENTER_CRITICAL(&s_statusMux);
  s_status.run++;
  s_status.seq = seq;
  s_status.step = 0;
  s_status.steps = steps;
  s_status.result = cmdproto::Result::NONE;
  uint32_t run = s_status.run;
  portEXIT_CRITICAL(&s_statusMux);
  return run;
}

static void statusStep(int step) {
  portENTER_CRITICAL(&s_statusMux);
  s_status.step = step;
  portEXIT_CRITICAL(&s_statusMux);
}

// Called by the run's task before it clears BIT_AUTO_RUNNING, so a new run can
// only begin after this result is in place.
static void statusFinish(cmdproto::Result result) {
  portENTER_CRITICAL(&s_statusMux);
  if (result == cmdproto::Result::DONE) s_status.step = s_status.steps;
  s_status.result = result;
  portEXIT_CRITICAL(&s_statusMux);
}

bool startSequence(cmdproto::SeqName seq, int param, uint32_t *runOut) {
  if (xEventGroupGetBits(controlEvents) & BIT_AUTO_RUNNING) return false;

  TaskFunction_t entry = nullptr;
  void *arg = nullptr;
  const char *name = nullptr;
  int steps = 0;
  switch (seq) {
  case cmdproto::SeqName::AUTO:
    entry = autonomous_task; arg = (void *)(intptr_t)0; name = "AutoTask"; steps = SEQ_STEPS_AUTO;
    break;
  case cmdproto::SeqName::PARK:
    entry = autonomous_task; arg = (void *)(intptr_t)1; name = "AutoTask"; steps = SEQ_STEPS_PARK;
    break;
  case cmdproto::SeqName::GOTO:
    if (param < 0 || param > 2) return false;
    entry = motor_goto_task; arg = (void *)(intptr_t)param; name = "GotoTask"; steps = 1;
    break;
  default:
    return false;
  }

  // A stop that raced with the end of the previous run can leave this bit set;
  // it must not abort the new run at its first check.
  xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);

  uint32_t run = statusBegin(seq, steps);
  // Set the running flag before the task exists: on the second core the task could
  // otherwise finish (and clear the flag) before we set it, leaving it stuck.
  xEventGroupSetBits(controlEvents, BIT_AUTO_RUNNING);

  TaskHandle_t handle = NULL;
  if (xTaskCreate(entry, name, 4096, arg, 2, &handle) != pdPASS) {
    statusFinish(cmdproto::Result::ABORTED);
    xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
    return false;
  }
  if (runOut) *runOut = run;
  return true;
}

void autonomous_task(void *pvParameters) {
  uint8_t slowSpeed = 128;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      slowSpeed = systemState.actGoSpeed;
      xSemaphoreGive(systemStateMutex);
  }

  int seqType = (int)(intptr_t)pvParameters;

  // Define the normal autonomous sequence steps
  const SequenceStep sequence_normal[] = {
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 100, 0, 0, "Auto: Arm Tip"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Act Mid"},
      {SEQ_MOVE_Z, 0, 1, 0, "Auto: Lower Z Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 1 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 1 Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 2 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 2 Mid"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 255, "Auto: Mix 3 Bot"},
      {SEQ_MOVE_ACTUATOR, 1, 0, 255, "Auto: Mix 3 Mid"},
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 200, 0, 0, "Auto: Arm Buffer"},
      {SEQ_MOVE_ARM_AND_Z, 0, 0, 0, "Auto: Clr & Lwr Z"},
      {SEQ_MOVE_ACTUATOR, 0, 0, 165, "Auto: Dispense"},
      {SEQ_WAIT_MS, 2000, 0, 0, "Auto: Pause"},
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ACTUATOR, 2, 0, 255, "Auto: Done"}
  };

  // Define the shutdown sequence
  const SequenceStep sequence_shutdown[] = {
      {SEQ_MOVE_Z, 0, 2, 0, "Auto: Raise Z"},
      {SEQ_MOVE_ARM, 0, 0, 0, "Auto: Arm Clear"}
  };

  const SequenceStep* sequence = (seqType == 1) ? sequence_shutdown : sequence_normal;
  const int numSteps = (seqType == 1) ? 2 : (sizeof(sequence_normal) / sizeof(sequence_normal[0]));
  static_assert(sizeof(sequence_normal) / sizeof(sequence_normal[0]) == SEQ_STEPS_AUTO,
                "SEQ_STEPS_AUTO out of date");
  static_assert(sizeof(sequence_shutdown) / sizeof(sequence_shutdown[0]) == SEQ_STEPS_PARK,
                "SEQ_STEPS_PARK out of date");
  bool aborted = false;

  LCD_setMessage("Auto: Running");
  printf("Starting Autonomous Sequence...\n");

  for (int i = 0; i < numSteps; i++) {
    const SequenceStep &step = sequence[i];

    // Check for E-STOP before each step
    EventBits_t ev = xEventGroupGetBits(controlEvents);
    if (ev & BIT_ESTOP_REQUEST) {
      aborted = true;
      break;
    }

    statusStep(i + 1);

    if (step.message != NULL) {
      LCD_setMessage(step.message);
    }

    bool stepComplete = false;
    while (!stepComplete) {
      ev = xEventGroupGetBits(controlEvents);
      if (ev & BIT_ESTOP_REQUEST) {
        aborted = true;
        break;
      }

      switch (step.action) {
      case SEQ_MOVE_Z: {
        MotorTelemetry motorTel;
        float currentPos = 0;
        float targetZ = 0;
        if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }

        // Determine direction
        bool goingUp = (targetZ > currentPos);
        
        // Only command movement if we aren't already there (prevents instant speed spikes and locking)
        bool posReached = false;
        if (std::abs(targetZ - currentPos) <= 0.1f) {
            posReached = true;
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.zGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            g_motorNode.setTarget(targetZ, goSpeed);
        }

        if (!posReached) {
            xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z);
            while (true) {
                EventBits_t uxBits = xEventGroupWaitBits(
                    controlEvents, BIT_POS_REACHED_Z | BIT_ESTOP_REQUEST,
                    pdTRUE, pdFALSE, portMAX_DELAY);
                
                if (uxBits & BIT_ESTOP_REQUEST) {
                    xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                    aborted = true;
                    break;
                }
                if (uxBits & BIT_POS_REACHED_Z) {
                    posReached = true;
                    break;
                }
                
                printf("Timeout waiting for Z-axis\n");
                aborted = true;
                break;
            }
        }

        g_motorNode.setSpeed(0);
        stepComplete = true;
        break;
      }

      case SEQ_MOVE_ARM: {
        ArmTelemetry armTel;
        if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == SEQ_TARGET_BUFFER && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            
            bool posReached = false;
            if (std::abs(armTel.currentPosition - targetAbs) < 10.0f) {
              posReached = true;
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.armGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_armNode.setTarget(step.target == SEQ_TARGET_BUFFER ? 200.0f : (float)step.target, goSpeed);
            }
            
            if (!posReached) {
                xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ARM);
                while (true) {
                    EventBits_t uxBits = xEventGroupWaitBits(
                        controlEvents, BIT_POS_REACHED_ARM | BIT_ESTOP_REQUEST,
                        pdTRUE, pdFALSE, portMAX_DELAY);
                    
                    if (uxBits & BIT_ESTOP_REQUEST) {
                        xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                        aborted = true;
                        break;
                    }
                    if (uxBits & BIT_POS_REACHED_ARM) {
                        posReached = true;
                        break;
                    }
                    
                    printf("Timeout waiting for ARM\n");
                    aborted = true;
                    break;
                }
            }
            if (posReached) stepComplete = true;
          } else {
            stepComplete = true; // Skip if uncalibrated
          }
        }
        break;
      }
      
      case SEQ_MOVE_ARM_AND_Z: {
        bool zReached = false;
        bool armReached = false;
        
        // --- Z Logic ---
        MotorTelemetry motorTel;
        float currentZPos = 0;
        float targetZ = 0;
        if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
          currentZPos = motorTel.currentPosition;
          targetZ = motorTel.limits[step.limitIdx];
        }
        bool zGoingUp = (targetZ > currentZPos);
        
        if (std::abs(targetZ - currentZPos) <= 0.1f) {
            zReached = true;
            g_motorNode.setSpeed(0);
        } else {
            int goSpeed = 5000;
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                goSpeed = systemState.zGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
            g_motorNode.setTarget(targetZ, goSpeed);
        }

        // --- Arm Logic ---
        ArmTelemetry armTel;
        if (xQueuePeek(armTelQueue, &armTel, pdMS_TO_TICKS(10)) == pdPASS) {
          int outSteps = armTel.posOut;
          int inSteps = armTel.posIn;
          int bufferSteps = armTel.posBuffer;
          if (outSteps != -1 && inSteps != -1) {
            float targetAbs = 0;
            if (step.target == SEQ_TARGET_BUFFER && bufferSteps != -1) {
                targetAbs = bufferSteps;
            } else {
                targetAbs = outSteps + (step.target / 100.0f) * (inSteps - outSteps);
            }
            if (std::abs(armTel.currentPosition - targetAbs) < 10.0f) {
              armReached = true;
            } else {
              int goSpeed = 5000;
              if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                  goSpeed = systemState.armGoSpeed;
                  xSemaphoreGive(systemStateMutex);
              }
              g_armNode.setTarget(step.target == SEQ_TARGET_BUFFER ? 200.0f : (float)step.target, goSpeed);
            }
          } else {
            armReached = true; // Skip if uncalibrated
          }
        }

        xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z | BIT_POS_REACHED_ARM);
        
        while (!zReached || !armReached) {
            EventBits_t waitBits = 0;
            if (!zReached) waitBits |= BIT_POS_REACHED_Z;
            if (!armReached) waitBits |= BIT_POS_REACHED_ARM;
            waitBits |= BIT_ESTOP_REQUEST;
            
            EventBits_t uxBits = xEventGroupWaitBits(
                controlEvents, waitBits,
                pdTRUE, pdFALSE, portMAX_DELAY);
                
            if (uxBits & BIT_ESTOP_REQUEST) {
                xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                aborted = true;
                break;
            }
            if (uxBits & BIT_POS_REACHED_Z) zReached = true;
            if (uxBits & BIT_POS_REACHED_ARM) armReached = true;
            
            if (!(uxBits & waitBits)) {
                printf("Timeout waiting for ARM_AND_Z\n");
                aborted = true;
                break;
            }
        }
        
        if (zReached && armReached) {
            stepComplete = true;
        }
        break;
      }

      case SEQ_MOVE_ACTUATOR: {
        ActuatorTelemetry actTel;
        int targetPct = 0;
        if (xQueuePeek(actuatorTelQueue, &actTel, pdMS_TO_TICKS(10)) == pdPASS) {
          targetPct = actTel.limits[step.target];
        }
        
        int speed = step.actuatorSpeed;
        if (speed == 255) { // Use configured speed if max speed is requested
            if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                speed = systemState.actGoSpeed;
                xSemaphoreGive(systemStateMutex);
            }
        }
        
        bool posReached = false;
        if (std::abs((int)actTel.currentPercent - targetPct) <= 1) {
          posReached = true;
        }
        
        g_actuatorNode.setTarget(targetPct, speed);
        
        if (!posReached) {
            xEventGroupClearBits(controlEvents, BIT_POS_REACHED_ACT);
            while (true) {
                EventBits_t uxBits = xEventGroupWaitBits(
                    controlEvents, BIT_POS_REACHED_ACT | BIT_ESTOP_REQUEST,
                    pdTRUE, pdFALSE, portMAX_DELAY);
                
                if (uxBits & BIT_ESTOP_REQUEST) {
                    xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
                    aborted = true;
                    break;
                }
                if (uxBits & BIT_POS_REACHED_ACT) {
                    posReached = true;
                    break;
                }
                
                printf("Timeout waiting for Actuator\n");
                aborted = true;
                break;
            }
        }
        
        if (posReached) stepComplete = true;
        break;
      }

      case SEQ_WAIT_MS: {
        // Interruptible delay: yield in 10ms chunks, checking for E-STOP
        int elapsed = 0;
        while (elapsed < step.target) {
          EventBits_t ev = xEventGroupGetBits(controlEvents);
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }
          vTaskDelay(pdMS_TO_TICKS(10));
          elapsed += 10;
        }
        stepComplete = true;
        break;
      }

      case SEQ_WAIT_USER: {
        // Wait for user button press OR E-STOP
        while (true) {
          EventBits_t ev = xEventGroupWaitBits(
              controlEvents, BIT_AUTO_RESUME | BIT_ESTOP_REQUEST, pdTRUE,
              pdFALSE, pdMS_TO_TICKS(100));

          if (ev & BIT_AUTO_RESUME) {
            // Clear any spurious ESTOP that arrived with the resume
            xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
            LCD_setMessage("Auto: Resuming");
            stepComplete = true;
            break;
          }
          if (ev & BIT_ESTOP_REQUEST) {
            aborted = true;
            break;
          }
        }
        break;
      }

    } // end switch
    } // end while (!stepComplete)
  }   // end for

  // ---- Cleanup ----
  if (aborted) {
    // E-STOP: halt everything immediately
    g_motorNode.setSpeed(0);
    g_actuatorNode.setTarget(0);
    LCD_setMessage("Auto: E-STOPPED");
    printf("!!! Autonomous Sequence E-STOPPED !!!\n");
    xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
  } else {
    LCD_setMessage("Auto: Complete");
    printf("Autonomous Sequence Complete.\n");
  }

  statusFinish(aborted ? cmdproto::Result::ABORTED : cmdproto::Result::DONE);

  // Clear the running flag and delete self
  xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
  vTaskDelete(NULL);
}

void motor_goto_task(void *pvParameters) {
  int limitIdx = (int)(intptr_t)pvParameters;  // 0=Bot, 1=Mid, 2=Top
  float targetZ = 0.0f;

  // Read motor limits from telemetry
  MotorTelemetry motorTel;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    targetZ = motorTel.limits[limitIdx];
  }

  LCD_setMessage("Auto: GOTO");
  printf("Starting GOTO target %.2f...\n", targetZ);

  float currentPos = 0.0f;
  if (xQueuePeek(motorTelQueue, &motorTel, pdMS_TO_TICKS(10)) == pdPASS) {
    currentPos = motorTel.currentPosition;
  }

  int autoSpeed = 5000;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    autoSpeed = systemState.zGoSpeed;
    xSemaphoreGive(systemStateMutex);
  }

  g_motorNode.setTarget(targetZ, autoSpeed);

  statusStep(1);
  xEventGroupClearBits(controlEvents, BIT_POS_REACHED_Z);
  bool aborted = false;
  while (true) {
    EventBits_t uxBits = xEventGroupWaitBits(
        controlEvents, BIT_POS_REACHED_Z | BIT_ESTOP_REQUEST,
        pdTRUE, pdFALSE, portMAX_DELAY);
        
    if (uxBits & BIT_ESTOP_REQUEST) {
        xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST);
        aborted = true;
        break;
    }
    if (uxBits & BIT_POS_REACHED_Z) {
        break;
    }
    
    printf("Timeout waiting for Z-axis GOTO\n");
    aborted = true;
    break;
  }

  g_motorNode.setSpeed(0);

  if (xSemaphoreTake(encoderStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_encoderState.position[2] = 0;
    xSemaphoreGive(encoderStateMutex);
  }

  if (aborted) {
    LCD_setMessage("GOTO E-STOPPED");
    xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
  } else {
    LCD_setMessage("GOTO Complete");
  }

  statusFinish(aborted ? cmdproto::Result::ABORTED : cmdproto::Result::DONE);

  xEventGroupClearBits(controlEvents, BIT_AUTO_RUNNING);
  vTaskDelete(NULL);
}
