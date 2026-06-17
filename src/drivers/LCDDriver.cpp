#include "drivers/LCDDriver.h"
#include "messaging.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include "core/NetworkManager.h"
#include "HardwareConfig.h"
#include "controller.h"
#include <U8g2lib.h>
#include "../ota_sprites.h"

/**
 * Mutex Lock Order Protocol (ALWAYS acquire in this order to prevent deadlock):
 * 1. lcdMutex          (LCD driver internal state message buffer)
 * 2. encoderStateMutex (g_encoderState - encoder input)
 * 3. systemStateMutex  (SystemState - UI-specific states ONLY: mode, enc selections)
 * * Rule: Never hold a "lower" mutex while waiting for a "higher" one.
 * Rule: Keep critical sections short; copy data to local vars before releasing.
 * Note: Motion subsystem state is read lock-free via telemetry queues.
 */

// Instantiation for our LCD screen
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_DC, LCD_RESET);

// ============ Encapsulated LCD Globals ============
static char lcdActionMessage[32] = "";
static bool lcdMessagePending = false;
static uint32_t lcdMessageTimestamp = 0;
static uint32_t lcdBtnPressTime[4] = {0, 0, 0, 0};

// LCD-specific mutex for thread-safe message buffer access
static SemaphoreHandle_t lcdMutex = NULL;

static void draw_peachLogo() {
  // === Half-peach graphic (far left, centered vertically) ===
  const int cx = 22, cy = 34, r = 22;

  // Draw the full peach body
  u8g2.drawDisc(cx, cy, r);

  // Cut it in half: clear the left side to create the flat cut face
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, cy - r - 1, cx - 4, r * 2 + 2);
  u8g2.setDrawColor(1);

  // Flat cut face (vertical line)
  u8g2.drawVLine(cx - 4, cy - r + 2, r * 2 - 4);

  // Inner flesh ring to give depth
  u8g2.setDrawColor(0);
  u8g2.drawCircle(cx, cy, r - 4);
  u8g2.setDrawColor(1);

  // Pit: concentric circles in the center
  u8g2.drawDisc(cx + 2, cy, 7, U8G2_DRAW_ALL);
  u8g2.setDrawColor(0);
  u8g2.drawDisc(cx + 2, cy, 4, U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);
  // Pit texture
  u8g2.drawPixel(cx + 2, cy);
  u8g2.drawPixel(cx + 3, cy - 1);
  u8g2.drawPixel(cx + 1, cy + 1);

  // Stem at top
  u8g2.drawLine(cx + 3, cy - r + 1, cx + 6, cy - r - 5);
  u8g2.drawLine(cx + 4, cy - r + 1, cx + 7, cy - r - 5);

  // Small leaf
  u8g2.drawEllipse(cx + 10, cy - r - 3, 5, 2);
  u8g2.drawLine(cx + 7, cy - r - 4, cx + 14, cy - r - 3);
}

static int fallingPeachLeafIndex = -1;
static int currentLeaf = 0;
static void drawBranch(int progress, float rootX, float x, float y, float len, float angle, int depth, int pathIndex);

static void draw_splashScreen() {
  u8g2.clearBuffer();
  u8g2.drawXBMP(24, 0, BOOT_SPRITE_W, BOOT_SPRITE_H, ota_sprite_boot_0);
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(24, 55, "Initializing...");
  u8g2.sendBuffer();
  vTaskDelay(pdMS_TO_TICKS(800));

  u8g2.clearBuffer();
  u8g2.drawXBMP(24, 0, BOOT_SPRITE_W, BOOT_SPRITE_H, ota_sprite_boot_1);
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(24, 55, "Starting modules...");
  u8g2.sendBuffer();
  vTaskDelay(pdMS_TO_TICKS(800));

  u8g2.clearBuffer();
  u8g2.drawXBMP(24, 0, BOOT_SPRITE_W, BOOT_SPRITE_H, ota_sprite_boot_2);
  u8g2.setFont(u8g2_font_tiny5_tf);
  u8g2.drawStr(24, 55, "Ready!");
  u8g2.sendBuffer();
  vTaskDelay(pdMS_TO_TICKS(900));
}

void draw_wifiStatus(const char* status, const char* ssid, int attempt, bool failed) {
  u8g2.clearBuffer();

  // Draw static boot sprite centered at the top
  u8g2.drawXBMP(24, 0, BOOT_SPRITE_W, BOOT_SPRITE_H, ota_sprite_boot_0);

  u8g2.setFont(u8g2_font_tiny5_tf);
  
  // Format SSID text
  char ssidBuf[32];
  snprintf(ssidBuf, sizeof(ssidBuf), "SSID: %s", ssid);
  u8g2.drawStr(24, 50, ssidBuf);
  
  // Format Status text + Animation
  char statusBuf[48];
  if (failed) {
      snprintf(statusBuf, sizeof(statusBuf), "%s Rebooting...", status);
  } else {
      char anim[16] = "";
      int dotCount = (attempt % 4);
      for (int i = 0; i < dotCount; i++) strcat(anim, ".");
      snprintf(statusBuf, sizeof(statusBuf), "%s%s", status, anim);
  }
  u8g2.drawStr(24, 60, statusBuf);

  u8g2.sendBuffer();
}

// Deterministic hash function for organic but stable procedural generation
static float hash_fn(int a, int b) {
  float h = sinf(a * 12.9898f + b * 78.233f) * 43758.5453123f;
  return h - floorf(h);
}

static float clamp(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

// Recursive function to draw the tree dynamically
static void drawBranch(int progress, float rootX, float x, float y, float len, float angle, int depth, int pathIndex) {
  const int maxDepth = 4; // Max depth 4 for 128x64 screen performance

  // Calculate the timing for this specific depth
  float startP, endP;
  if (depth == 0) {
    startP = 0; endP = 20;
  } else {
    float depthSlice = 30.0f / maxDepth;
    startP = 20 + (depth - 1) * depthSlice;
    endP = startP + depthSlice;
  }

  float growth = clamp((progress - startP) / (endP - startP), 0.0f, 1.0f);

  if (growth > 0) {
    float currentLen = len * growth;
    float endX = x + currentLen * sinf(angle);
    float endY = y - currentLen * cosf(angle);

    // Draw the branch line
    u8g2.drawLine((int)x, (int)y, (int)endX, (int)endY);

    // --- STAGE 2: The Branches (20% - 50%) ---
    if (depth < maxDepth && progress > endP) {
      // Generate procedural but deterministic branch angles and lengths
      float leftLenMod = 0.65f + hash_fn(pathIndex, 1) * 0.15f;
      float rightLenMod = 0.65f + hash_fn(pathIndex, 2) * 0.15f;
      // Increased branch angles to spread them out more
      float leftAngleMod = 0.45f + hash_fn(pathIndex, 3) * 0.4f;
      float rightAngleMod = 0.45f + hash_fn(pathIndex, 4) * 0.4f;

      // Spawn left and right children
      drawBranch(progress, rootX, endX, endY, len * leftLenMod, angle - leftAngleMod, depth + 1, pathIndex * 2);
      drawBranch(progress, rootX, endX, endY, len * rightLenMod, angle + rightAngleMod, depth + 1, pathIndex * 2 + 1);

      // Occasionally spawn a middle branch for organic density
      // Increased probability of middle branch for a fuller tree
      if (hash_fn(pathIndex, 5) > 0.5f) { 
         drawBranch(progress, rootX, endX, endY, len * 0.5f, angle + (hash_fn(pathIndex, 6) * 0.2f - 0.1f), depth + 1, pathIndex * 2 + 2);
      }
    }

    // --- STAGE 3: The Leaves (50% - 80%) ---
    // Increased leaf density for a fuller look (70% chance of leaf)
    if (depth >= 2 && hash_fn(pathIndex, 11) > 0.3f) {
      float leafStart = 50 + hash_fn(pathIndex, 7) * 15;
      float leafEnd = leafStart + 15;
      
      float leafGrowth = clamp((progress - leafStart) / (leafEnd - leafStart), 0.0f, 1.0f);
      
      if (leafGrowth > 0) {
        // Just draw a small circle for leaves on the OLED
        int r = (int)(2.0f * leafGrowth);
        if (r > 0) {
            u8g2.drawDisc((int)endX, (int)endY, r);
        }
      }
    }

    // --- STAGE 4: The Peaches (80% - 100%) ---
    if (depth == maxDepth) {
      currentLeaf++;
      
      // Only spawn exactly three peaches
      if (currentLeaf == 8 || currentLeaf == 25 || currentLeaf == 45) {
        if (progress > 80) {
          float peachGrowth = clamp((progress - 80.0f) / 10.0f, 0.0f, 1.0f); // Grow faster (80-90)
          
          float pY = endY;
          
          // Pick the first peach that is far enough from the center to be the falling peach
          if (fallingPeachLeafIndex == -1 && abs((int)endX - (int)rootX) > 12) {
              fallingPeachLeafIndex = currentLeaf;
          }
          
          // Peach falling animation (90% - 100%)
          if (progress > 90 && currentLeaf == fallingPeachLeafIndex) {
               float fallProgress = clamp((progress - 90.0f) / 10.0f, 0.0f, 1.0f);
               // Quadratic fall: distance = 1/2 * g * t^2
               pY += (62.0f - endY) * fallProgress * fallProgress; 
               // Stop at ground
               if (pY > 60.0f) pY = 60.0f;
          }

          if (peachGrowth > 0) {
            int radius = (int)(4.0f * peachGrowth); // slightly smaller peaches
            if (radius > 0) {
                // Draw small peach body (circle with cleft line)
                u8g2.drawDisc((int)endX, (int)pY, radius);
                u8g2.setDrawColor(0);
                u8g2.drawLine((int)endX, (int)(pY - radius), (int)endX, (int)(pY + radius - 1));
                u8g2.setDrawColor(1);
            }
          }
        }
      }
    }
  }
}

void draw_otaScreen() {
  static int lastFrame = -1;
  int percent = NetworkManager::getOTAProgress();

  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  int frame;
  if (percent >= 100) {
      frame = OTA_SPRITE_COUNT - 1; 
  } else {
      frame = (percent * (OTA_SPRITE_COUNT - 1)) / 100;
  }

  if (frame != lastFrame) {
      u8g2.clearBuffer();

      const uint8_t* spritePtr = (const uint8_t*)pgm_read_ptr(&ota_sprites[frame]);
      u8g2.drawXBMP(0, 0, OTA_SPRITE_W, OTA_SPRITE_H, spritePtr);
      
      u8g2.sendBuffer();
      lastFrame = frame;
  }
}

void LCDInit() {
  u8g2.begin();
  u8g2.setBusClock(4000000); // Lower SPI speed to 4MHz to prevent screen tearing
  u8g2.sendF("ca", 0xd5, 0xf0); // Maximize internal oscillator freq to fix camera flicker
  u8g2.setFont(u8g2_font_tiny5_tf);

  lcdMutex = xSemaphoreCreateMutex();
  if (lcdMutex == NULL) {
    PEACH_LOGE("LCD", "Failed to create LCD string mutex");
  }

  // Show splash screen for 2.5 seconds
  draw_splashScreen();

  // Restore the small UI font
  u8g2.setFont(u8g2_font_tiny5_tf);
}

void LCD_setMessage(const char *msg) {
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    strncpy(lcdActionMessage, msg, sizeof(lcdActionMessage) - 1);
    lcdActionMessage[sizeof(lcdActionMessage) - 1] = '\0';
    lcdMessageTimestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    lcdMessagePending = true;
    xSemaphoreGive(lcdMutex);
  } else {
    PEACH_LOGW("LCD", "Mutex timeout setting message");
  }
}

void LCD_notifyButtonPress(int index) {
  if (index >= 0 && index < 4) {
    if (lcdMutex != NULL &&
        xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lcdBtnPressTime[index] = xTaskGetTickCount() * portTICK_PERIOD_MS;
      xSemaphoreGive(lcdMutex);
    }
  }
}

// ============ Drawing Helpers ============

static void draw_displayTimer() {
  uint32_t t = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000;
  uint32_t m = t / 60;
  uint32_t s = t % 60;
  char timerBuffer[32];
  snprintf(timerBuffer, sizeof(timerBuffer), "RUN:%02u:%02u", (unsigned int)m,
           (unsigned int)s);
  u8g2.drawStr(0, 6, timerBuffer);
}

static void draw_buttonStatus(const UIData& data) {
  uint32_t localBtnTime[4] = {0};

  // 2. Grab LCD internal state (lower priority mutex)
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < 4; i++) {
      localBtnTime[i] = lcdBtnPressTime[i];
    }
    xSemaphoreGive(lcdMutex);
  }

  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

  // 3. Render using local copies (no mutexes held during drawing)
  for (int i = 0; i < 4; i++) {
    bool recentlyPressed = (now - localBtnTime[i] < 200);

    if (recentlyPressed) {
      u8g2.drawDisc(100 + (i * 6), 4, 2); // filled = pressed
    } else {
      u8g2.drawCircle(100 + (i * 6), 4, 2); // outline = unpressed
    }
  }
}

static const char* s4Level0Names[] = {"Shutdown", "Arm", "Act", "Z", "Auto"};
static const char* s4ArmSubNames[] = {"Tip", "Buf", "Clr", "JogSpd", "GoSpd", "Back"};
static const char* s4PosSubNames[] = {"Top", "Mid", "Bot", "JogSpd", "GoSpd", "Back"};

static void draw_encoderStatus(const UIData& data) {
  char statusBuffer[32];

  // Direction arrow character helper
  auto dirChar = [](int dir) -> char {
    if (dir > 0) return '>';
    if (dir < 0) return '<';
    return '-';
  };

  // ---- S1: Arm (y=11, text baseline y=17) ----
  {
    char dc = dirChar(data.armJogDir);
    if (data.armPosOut != -1 && data.armPosIn != -1) {
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c", dc);
    } else {
      snprintf(statusBuffer, sizeof(statusBuffer), "S1:Arm:%c NC", dc);
    }
    u8g2.drawStr(0, 17, statusBuffer);

    // Sliding dot track: x=72 to x=124, y=14
    const int trackL = 72, trackR = 124, trackY = 14;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    if (data.armPosOut != -1 && data.armPosIn != -1 && data.armPosIn != data.armPosOut) {
      u8g2.drawVLine(trackL, trackY - 2, 5);
      u8g2.drawVLine(trackR, trackY - 2, 5);
      int dotX = map((int)data.armPosition, data.armPosOut, data.armPosIn, trackL, trackR);
      dotX = constrain(dotX, trackL, trackR);
      u8g2.drawDisc(dotX, trackY, 2);
    } else {
      u8g2.drawDisc((trackL + trackR) / 2, trackY, 2);
    }
  }

  // ---- S2: Actuator (y=20, text baseline y=26) ----
  {
    char dc = dirChar(data.actJogDir);
    snprintf(statusBuffer, sizeof(statusBuffer), "S2:Act:%c %d%%", dc, data.actuatorPercent);
    u8g2.drawStr(0, 26, statusBuffer);

    // Fill bar: frame from x=72 to x=124, height 7
    const int barL = 72, barW = 52, barY = 20, barH = 7;
    u8g2.drawFrame(barL, barY, barW, barH);
    int fillW = map(constrain(data.actuatorPercent, 0, 100), 0, 100, 0, barW - 2);
    if (fillW > 0) {
      u8g2.drawBox(barL + 1, barY + 1, fillW, barH - 2);
    }
  }

  // ---- S3: Z Motor (y=29, text baseline y=35) ----
  {
    char dc = dirChar(data.zJogDir);
    snprintf(statusBuffer, sizeof(statusBuffer), "S3:Z:%c", dc);
    u8g2.drawStr(0, 35, statusBuffer);

    // Position track with limit ticks (x=72 to x=124, y=32)
    bool botSet = data.motorLimitSet[0];
    bool topSet = data.motorLimitSet[2];
    float minLim = data.motorLimits[0];
    float maxLim = data.motorLimits[2];

    const int trackL = 72, trackR = 124, trackY = 32;
    u8g2.drawHLine(trackL, trackY, trackR - trackL);

    if (botSet && topSet && maxLim > minLim) {
      float range = maxLim - minLim;
      if (range > 0.01f) {
        for (int i = 0; i < 3; i++) {
          if (data.motorLimitSet[i]) {
            float val = data.motorLimits[i];
            if (val < minLim) val = minLim;
            if (val > maxLim) val = maxLim;
            int tickX = trackL + (int)(((val - minLim) / range) * (trackR - trackL));
            u8g2.drawVLine(tickX, trackY - 2, 5);
          }
        }

        float cPos = data.motorPos;
        if (cPos < minLim) cPos = minLim;
        if (cPos > maxLim) cPos = maxLim;
        int dotX = trackL + (int)(((cPos - minLim) / range) * (trackR - trackL));
        u8g2.drawDisc(dotX, trackY, 2);
      }
    } else {
      u8g2.drawDisc((trackL + trackR) / 2, trackY, 2);
    }
  }

  // ---- S4: Menu (y=38, text baseline y=44) ----
  {
    if (!data.s4InSubMenu) {
      // Level 0: axis selection
      snprintf(statusBuffer, sizeof(statusBuffer), "S4:>%s", s4Level0Names[data.s4Menu]);
    } else {
      // Level 1: sub-menu with position values
      const char* axisName = s4Level0Names[data.s4Menu];
      const char* itemName = "";

      if (data.s4Menu == S4_ARM) {
        itemName = s4ArmSubNames[data.s4SubMenu];

        if (data.s4SubMenu == S4_ARM_BACK) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        } else if (data.s4SubMenu == S4_ARM_TIP) {
          if (data.armPosIn != -1)
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Tip:%d", axisName, data.armPosIn);
          else
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Tip:--", axisName);
        } else if (data.s4SubMenu == S4_ARM_BUFFER) {
          if (data.armPosBuffer != -1) {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Buf:%d", axisName, data.armPosBuffer);
          } else {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Buf:--", axisName);
          }
        } else if (data.s4SubMenu == S4_ARM_CLEAR) {
          if (data.armPosOut != -1) {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Clr:%d", axisName, data.armPosOut);
          } else {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Clr:--", axisName);
          }
        } else if (data.s4SubMenu == S4_ARM_JOG_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.armJogSpeed, data.s4InSpeedEdit ? "*" : "");
        } else if (data.s4SubMenu == S4_ARM_GO_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.armGoSpeed, data.s4InSpeedEdit ? "*" : "");
        }
      } else if (data.s4Menu == S4_ACT) {
        int idx = data.s4SubMenu;
        itemName = s4PosSubNames[idx];
        if (idx == S4_POS_BACK) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        } else if (idx == S4_POS_JOG_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.actJogSpeed, data.s4InSpeedEdit ? "*" : "");
        } else if (idx == S4_POS_GO_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.actGoSpeed, data.s4InSpeedEdit ? "*" : "");
        } else {
          int limitIdx = (idx == S4_POS_TOP) ? 2 : ((idx == S4_POS_BOT) ? 0 : 1);
          if (data.actuatorLimitSet[limitIdx]) {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%d%%", axisName, itemName, data.actuatorLimits[limitIdx]);
          } else {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:--", axisName, itemName);
          }
        }
      } else if (data.s4Menu == S4_Z) {
        int idx = data.s4SubMenu;
        itemName = s4PosSubNames[idx];
        if (idx == S4_POS_BACK) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Back", axisName);
        } else if (idx == S4_POS_JOG_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Jog:%d%s", axisName, data.zJogSpeed, data.s4InSpeedEdit ? "*" : "");
        } else if (idx == S4_POS_GO_SPD) {
          snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>Go:%d%s", axisName, data.zGoSpeed, data.s4InSpeedEdit ? "*" : "");
        } else {
          int limitIdx = (idx == S4_POS_TOP) ? 2 : ((idx == S4_POS_BOT) ? 0 : 1);
          if (data.motorLimitSet[limitIdx]) {
            int whole = (int)data.motorLimits[limitIdx];
            int frac = abs((int)(data.motorLimits[limitIdx] * 10) % 10);
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:%d.%d", axisName, itemName, whole, frac);
          } else {
            snprintf(statusBuffer, sizeof(statusBuffer), "S4:%s>%s:--", axisName, itemName);
          }
        }
      }
    }
    u8g2.drawStr(0, 44, statusBuffer);
  }
}




static void draw_actionMessage(const UIData& data) {
  bool pending = false;
  char localMsg[32] = "";
  uint32_t timestamp = 0;

  // Safely read the message buffer
  if (lcdMutex != NULL &&
      xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    pending = lcdMessagePending;
    timestamp = lcdMessageTimestamp;
    if (pending) {
      strncpy(localMsg, lcdActionMessage, sizeof(localMsg) - 1);
      localMsg[sizeof(localMsg) - 1] = '\0';
    }
    xSemaphoreGive(lcdMutex);
  }

  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  bool isAuto = data.isAutoRunning;

  // Show message for 2.5 seconds after being set, or infinitely if in Auto
  // sequence
  if (pending && (isAuto || (now - timestamp < 2500))) {
    char actionBuffer[48];
    snprintf(actionBuffer, sizeof(actionBuffer), "> %s", localMsg);
    u8g2.drawStr(0, 60, actionBuffer);
  } else if (pending && !isAuto) {
    // Safely auto-expire the flag
    if (lcdMutex != NULL &&
        xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      lcdMessagePending = false;
      xSemaphoreGive(lcdMutex);
    }
  }
}

// ============ Main Draw Function ============

void draw_menu(const UIData& data) {
  u8g2.clearBuffer();

  draw_displayTimer();
  draw_buttonStatus(data);
  u8g2.drawHLine(0, 10, 128);

  draw_encoderStatus(data);
  u8g2.drawHLine(0, 46, 128);

  draw_actionMessage(data);

  u8g2.sendBuffer(); // Push to OLED
}