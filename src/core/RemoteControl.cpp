#include "core/RemoteControl.h"
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>
#include "controller.h"
#include "core/CommandProtocol.h"
#include "core/NetworkManager.h"
#include "core/SequenceEngine.h"
#include "messaging.h"
#include "tasks/ActuatorNode.h"
#include "tasks/ArmNode.h"
#include "tasks/MotorNode.h"

using namespace cmdproto;

extern ArmNode g_armNode;
extern ActuatorNode g_actuatorNode;
extern MotorNode g_motorNode;

namespace {

constexpr uint16_t REMOTE_PORT = 6667;
constexpr uint32_t REPLY_TIMEOUT_MS = 250;     // loop waits this long for the controller
constexpr size_t MAX_BYTES_PER_CALL = 192;     // per source per handle() call
constexpr size_t REPLY_BUF = 256;              // longest reply (STATUS) is ~200 bytes
constexpr uint32_t DEADLINE_MARGIN_MS = 20;    // controller stops executing this long before loop gives up
constexpr int REQUEST_QUEUE_DEPTH = 4;
static_assert(SEQ_TARGET_BUFFER == ARM_TARGET_BUFFER, "arm buffer target must match the sequences");
static const char *TAG = "REMOTE";

enum class Source : uint8_t { USB, TCP };

// loop task -> controller task
struct Request {
  Command cmd;
  Source source;
  uint32_t token;      // unique per request; the reply carries it back
  TickType_t deadline; // after this the requester has answered TIMEOUT: do not execute
};

// controller task -> loop task (depth-1 queue, overwritten, never blocks the controller)
struct Reply {
  uint32_t token;
  ErrorCode error;
  bool hasRun;
  uint32_t run;
  const char *text;  // string literal or nullptr
};

QueueHandle_t s_cmdQueue = nullptr;    // "remoteCmdQueue"
QueueHandle_t s_replyQueue = nullptr;
uint32_t s_token = 0;
// A request timed out: requests arriving within STALL_WINDOW_MS fail fast instead of
// each waiting another REPLY_TIMEOUT_MS (a flood of lines must not wedge loop()).
// The window is shorter than loop()'s 50 ms period, so the next cycle tries again.
constexpr uint32_t STALL_WINDOW_MS = 40;
bool s_stalled = false;
TickType_t s_stalledAt = 0;
// A STOP hit a running run: once the run has ended, drainPending halts motor and arm once more
// (the sequence's abort cleanup does not stop the arm and a late arm target can land after STOP).
// Set by the loop task, consumed by the controller task.
volatile bool s_haltAfterRun = false;

void markStalled() {
  s_stalled = true;
  s_stalledAt = xTaskGetTickCount();
}

bool controllerStalled() {
  return s_stalled && (xTaskGetTickCount() - s_stalledAt) < pdMS_TO_TICKS(STALL_WINDOW_MS);
}

WiFiServer s_server(REMOTE_PORT);
WiFiClient s_client;
bool s_serverStarted = false;

LineAssembler s_usbAsm;
LineAssembler s_tcpAsm;

// ---------------------------------------------------------------------------
// Live state (telemetry peeks, no blocking)
// ---------------------------------------------------------------------------
struct Live {
  bool haveMotor, haveArm, haveAct;
  MotorTelemetry motor;
  ArmTelemetry arm;
  ActuatorTelemetry act;
};

void readLive(Live &l) {
  l = Live();
  l.haveMotor = motorTelQueue && xQueuePeek(motorTelQueue, &l.motor, 0) == pdPASS;
  l.haveArm = armTelQueue && xQueuePeek(armTelQueue, &l.arm, 0) == pdPASS;
  l.haveAct = actuatorTelQueue && xQueuePeek(actuatorTelQueue, &l.act, 0) == pdPASS;
}

bool armCalibrated(const Live &l) {
  return l.haveArm && l.arm.posOut != -1 && l.arm.posIn != -1 && l.arm.posOut != l.arm.posIn;
}

// Same prerequisites as the encoder's Auto launch (cmdproto::readyForAuto).
bool liveReadyForAuto(const Live &l) {
  if (!l.haveMotor || !l.haveAct || !l.haveArm) return false;
  return readyForAuto(l.motor.limitSet, l.act.limitSet, l.arm.posOut, l.arm.posIn);
}

bool collisionFlag() {
  bool c = false;
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    c = systemState.collisionDetected;
    xSemaphoreGive(systemStateMutex);
  }
  return c;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
// One write call per reply line.
void sendLine(Source src, const char *buf, size_t n) {
  if (n == 0) return;
  if (src == Source::USB) {
    Serial.write((const uint8_t *)buf, n);
  } else if (s_client && s_client.connected()) {
    s_client.write((const uint8_t *)buf, n);
  }
}

void sendErr(Source src, uint32_t id, ErrorCode code, const char *text = nullptr) {
  char buf[REPLY_BUF];
  sendLine(src, buf, formatErr(buf, sizeof buf, id, code, text));
}

void sendOk(Source src, uint32_t id, const char *fields = nullptr) {
  char buf[REPLY_BUF];
  sendLine(src, buf, formatOk(buf, sizeof buf, id, fields));
}

// ---------------------------------------------------------------------------
// Loop-task side
// ---------------------------------------------------------------------------
void sendStatus(Source src, uint32_t id) {
  Live live;
  readLive(live);
  EventBits_t bits = xEventGroupGetBits(controlEvents);
  SequenceStatus ss = sequenceStatus();

  StatusFields f;
  f.running = (bits & BIT_AUTO_RUNNING) != 0;
  f.run = ss.run;
  f.seq = ss.seq;
  f.step = ss.step;
  f.steps = ss.steps;
  f.result = ss.result;
  f.estop = f.running && (bits & BIT_ESTOP_REQUEST) != 0;
  f.ready = liveReadyForAuto(live);
  if (live.haveMotor) {
    f.z = live.motor.currentPosition;
    f.zhomed = live.motor.isHomed;
    f.zmoving = live.motor.targetSpeed != 0;
  }
  if (live.haveArm) {
    f.arm = live.arm.currentPosition;
    f.armmoving = live.arm.isMoving;
  }
  if (live.haveAct) f.act = live.act.currentPercent;
  f.collision = collisionFlag();
  f.ota = NetworkManager::isOTAActive();

  char buf[REPLY_BUF];
  sendLine(src, buf, formatStatus(buf, sizeof buf, id, f));
}

// Put a halt command at the front of a node queue. If the queue is full, drop its
// backlog: a stop must never be refused.
void sendFront(QueueHandle_t q, const void *item) {
  if (q == nullptr) return;
  if (xQueueSendToFront(q, item, 0) != pdPASS) {
    xQueueReset(q);
    xQueueSendToFront(q, item, 0);
  }
}

// Motor and arm SET_SPEED 0 at the front of their queues.
void haltMotorAndArm() {
  MotorCommand m;
  m.action = MotorCmdAction::SET_SPEED;  // also cancels a GOTO in progress
  m.value = 0.0f;
  m.targetSpeed = 0;
  sendFront(motorCmdQueue, &m);

  // SET_SPEED 0 (not STOP): ArmCmdAction::STOP keeps a tracked target alive and the
  // arm would carry on to it; SET_SPEED clears the tracking.
  ArmCommand a;
  a.action = ArmCmdAction::SET_SPEED;
  a.value = 0.0f;
  a.targetSpeed = 0;
  sendFront(armCmdQueue, &a);
}

// Runs in the loop task, never queued behind the controller.
void doStop() {
  // Only when a run exists: a stale ESTOP bit with nothing running would abort the
  // next run (startSequence also clears it, drainPending clears it when idle).
  const bool running = (xEventGroupGetBits(controlEvents) & BIT_AUTO_RUNNING) != 0;
  if (running) {
    xEventGroupSetBits(controlEvents, BIT_ESTOP_REQUEST | BIT_AUTO_RESUME);
    s_haltAfterRun = true;
  }

  // Halt the nodes directly, as the OTA interlock does (NetworkManager.cpp onStart).
  haltMotorAndArm();

  // Actuator: while a run exists, leave it to the sequence's abort cleanup (it retracts to
  // 0 %, exactly as after an encoder E-STOP). Sending a hold as well would race that
  // retract. With nothing running, hold at the current position.
  if (!running) {
    ActuatorTelemetry actTel;
    if (actuatorTelQueue && xQueuePeek(actuatorTelQueue, &actTel, 0) == pdPASS) {
      ActuatorCommand c;
      c.action = ActuatorCmdAction::SET_TARGET;
      c.value = (int)lroundf(actTel.currentPercent);
      c.pwmSpeed = 255;
      sendFront(actuatorCmdQueue, &c);
    }
  }
  PEACH_LOGI(TAG, "STOP%s", running ? " (run aborting)" : "");
}

// Post to the controller task and wait for its answer (<= REPLY_TIMEOUT_MS).
void forward(Source src, const Command &cmd) {
  if (s_cmdQueue == nullptr || s_replyQueue == nullptr) {
    sendErr(src, cmd.id, ErrorCode::TIMEOUT, "no controller");
    return;
  }
  if (controllerStalled()) {
    sendErr(src, cmd.id, ErrorCode::TIMEOUT);
    return;
  }

  Request req;
  req.cmd = cmd;
  req.source = src;
  req.token = ++s_token;
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(REPLY_TIMEOUT_MS);
  // The controller stops executing a little before we give up, so a request is never run
  // after its client has been told TIMEOUT.
  req.deadline = requestDeadline(start, timeout, pdMS_TO_TICKS(DEADLINE_MARGIN_MS));

  xQueueReset(s_replyQueue);  // a late reply from an earlier request is not ours
  if (xQueueSend(s_cmdQueue, &req, 0) != pdPASS) {
    markStalled();
    sendErr(src, cmd.id, ErrorCode::TIMEOUT, "queue full");
    return;
  }

  for (;;) {
    TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= timeout) break;
    Reply rep;
    if (xQueueReceive(s_replyQueue, &rep, timeout - elapsed) != pdTRUE) break;
    if (rep.token != req.token) continue;  // stale reply, discard

    if (rep.error != ErrorCode::NONE) {
      sendErr(src, cmd.id, rep.error, rep.text);
    } else if (rep.hasRun) {
      char fields[24];
      snprintf(fields, sizeof fields, "run=%lu", (unsigned long)rep.run);
      sendOk(src, cmd.id, fields);
    } else {
      sendOk(src, cmd.id);
    }
    return;
  }
  // The controller drops requests past their deadline, but one already executing (or blocked
  // in a node queue) may still complete: TIMEOUT means "may or may not have run".
  markStalled();
  sendErr(src, cmd.id, ErrorCode::TIMEOUT);
}

void handleLine(Source src, const char *line) {
  Command cmd;
  ParseResult pr = parseCommand(line, cmd);
  if (!pr.ok()) {
    sendErr(src, cmd.id, pr.error);
    return;
  }
  switch (cmd.verb) {
  case Verb::PING:
    sendOk(src, cmd.id, "pong proto=1");
    break;
  case Verb::SEQS:
    sendOk(src, cmd.id, "seqs=auto,park");
    break;
  case Verb::STATUS:
    sendStatus(src, cmd.id);
    break;
  case Verb::STOP:
    doStop();
    sendOk(src, cmd.id);
    break;
  case Verb::RUN:
  case Verb::MOVE:
  case Verb::JOG:
    forward(src, cmd);
    break;
  default:
    sendErr(src, cmd.id, ErrorCode::BAD_CMD);
    break;
  }
}

void feedBytes(Source src, LineAssembler &lineAsm, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    LineAssembler::Event ev = lineAsm.feed((char)data[i]);
    if (ev == LineAssembler::Event::LINE) {
      handleLine(src, lineAsm.line());
    } else if (ev == LineAssembler::Event::LINE_OVERFLOW) {
      sendErr(src, 0, ErrorCode::TOO_LONG);
    }
  }
}

// ---------------------------------------------------------------------------
// Controller-task side
// ---------------------------------------------------------------------------
struct Speeds {
  int armGo, armJog, actGo, actJog, zJog;
};

bool readSpeeds(Speeds &s) {
  if (xSemaphoreTake(systemStateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  s.armGo = systemState.armGoSpeed;
  s.armJog = systemState.armJogSpeed;
  s.actGo = systemState.actGoSpeed;
  s.actJog = systemState.actJogSpeed;
  s.zJog = systemState.zJogSpeed;
  xSemaphoreGive(systemStateMutex);
  return true;
}

void postReply(const Request &req, ErrorCode err, const char *text = nullptr, bool hasRun = false,
               uint32_t run = 0) {
  Reply rep;
  rep.token = req.token;
  rep.error = err;
  rep.hasRun = hasRun;
  rep.run = run;
  rep.text = text;
  xQueueOverwrite(s_replyQueue, &rep);
}

void execute(const Request &req) {
  const Command &cmd = req.cmd;

  Live live;
  readLive(live);
  PolicySnapshot snap;
  snap.busy = (xEventGroupGetBits(controlEvents) & BIT_AUTO_RUNNING) != 0;
  snap.ready = liveReadyForAuto(live);
  snap.otaActive = NetworkManager::isOTAActive();
  snap.armCalibrated = armCalibrated(live);
  snap.armBufferSet = live.haveArm && live.arm.posBuffer != -1;
  for (int i = 0; i < 3; i++) {
    snap.zLimitSet[i] = live.haveMotor && live.motor.limitSet[i];
    snap.actLimitSet[i] = live.haveAct && live.act.limitSet[i];
  }
  ErrorCode err = checkPolicy(cmd, snap);
  if (err != ErrorCode::NONE) {
    postReply(req, err);
    return;
  }

  Speeds sp = {0, 0, 0, 0, 0};
  if (cmd.verb != Verb::RUN && !readSpeeds(sp)) {
    postReply(req, ErrorCode::TIMEOUT, "speeds busy");
    return;
  }

  uint32_t run = 0;
  switch (cmd.verb) {
  case Verb::RUN: {
    SeqName seq = cmd.seq == SeqName::PARK ? SeqName::PARK : SeqName::AUTO;
    if (!startSequence(seq, 0, &run)) {
      postReply(req, ErrorCode::BUSY, "start failed");
      return;
    }
    s_haltAfterRun = false;
    PEACH_LOGI(TAG, "RUN %s -> run %lu", seq == SeqName::PARK ? "park" : "auto", (unsigned long)run);
    postReply(req, ErrorCode::NONE, nullptr, true, run);
    return;
  }

  case Verb::MOVE:
    if (cmd.axis == Axis::Z) {
      if (!startSequence(SeqName::GOTO, limitIndex(cmd.target), &run)) {
        postReply(req, ErrorCode::BUSY, "start failed");
        return;
      }
      s_haltAfterRun = false;
      PEACH_LOGI(TAG, "MOVE z %d -> run %lu", limitIndex(cmd.target), (unsigned long)run);
    } else if (cmd.axis == Axis::ARM) {
      float pct = armMoveTarget(cmd.target, cmd.percent);  // 0 out, 100 in, 200 buffer
      g_armNode.setTarget(pct, sp.armGo);
      PEACH_LOGI(TAG, "MOVE arm %.0f", pct);
    } else if (cmd.axis == Axis::ACT) {
      int idx = limitIndex(cmd.target);
      g_actuatorNode.setTarget(live.act.limits[idx], sp.actGo);
      PEACH_LOGI(TAG, "MOVE act %d%%", live.act.limits[idx]);
    }
    postReply(req, ErrorCode::NONE);
    return;

  case Verb::JOG:
    if (cmd.axis == Axis::Z) {
      if (!live.haveMotor) {
        postReply(req, ErrorCode::NOT_READY, "no z telemetry");
        return;
      }
      if (cmd.jogZ != 0.0f) g_motorNode.setTarget(live.motor.currentPosition + cmd.jogZ, sp.zJog);
      PEACH_LOGI(TAG, "JOG z %.3f", cmd.jogZ);
    } else if (cmd.axis == Axis::ARM) {
      // Policy guarantees a calibrated arm. ArmNode's JOG adds to a tracked target but
      // never sets the tracking speed (and is unused elsewhere), so first re-target
      // the arm's current position at the jog speed (as the encoder does), then jog
      // relative to that.
      float here = armPercentOf(live.arm.currentPosition, live.arm.posOut, live.arm.posIn);
      if (cmd.jogInt != 0) {
        g_armNode.setTarget(here, sp.armJog);
        g_armNode.jog((float)cmd.jogInt);
      }
      PEACH_LOGI(TAG, "JOG arm %d", cmd.jogInt);
    } else if (cmd.axis == Axis::ACT) {
      if (!live.haveAct) {
        postReply(req, ErrorCode::NOT_READY, "no act telemetry");
        return;
      }
      int target = actuatorJogTarget(live.act.currentPercent, cmd.jogInt);
      if (cmd.jogInt != 0) g_actuatorNode.setTarget(target, sp.actJog);
      PEACH_LOGI(TAG, "JOG act %d", cmd.jogInt);
    }
    postReply(req, ErrorCode::NONE);
    return;

  default:
    postReply(req, ErrorCode::BAD_CMD);
    return;
  }
}

}  // namespace

namespace RemoteControl {

void init() {
  s_cmdQueue = xQueueCreate(REQUEST_QUEUE_DEPTH, sizeof(Request));
  s_replyQueue = xQueueCreate(1, sizeof(Reply));
  if (s_cmdQueue == nullptr || s_replyQueue == nullptr) {
    PEACH_LOGE(TAG, "queue creation failed; RUN/MOVE/JOG will answer TIMEOUT");
  }
  s_server.begin();
  s_server.setNoDelay(true);
  s_serverStarted = true;
  PEACH_LOGI(TAG, "Remote commands on port %u (and USB serial)", (unsigned)REMOTE_PORT);
}

void handle() {
  if (s_serverStarted) {
    // A new connection replaces the old one, so a reconnect after a half-open link wins.
    if (s_server.hasClient()) {
      WiFiClient fresh = s_server.available();
      if (fresh) {
        if (s_client) s_client.stop();
        s_client = fresh;
        s_client.setNoDelay(true);
        s_tcpAsm = LineAssembler();
      }
    }
    if (s_client && s_client.connected()) {
      uint8_t buf[64];
      size_t total = 0;
      while (total < MAX_BYTES_PER_CALL && s_client.available() > 0) {
        size_t want = sizeof buf;
        if (want > MAX_BYTES_PER_CALL - total) want = MAX_BYTES_PER_CALL - total;
        int n = s_client.read(buf, want);
        if (n <= 0) break;
        total += (size_t)n;
        feedBytes(Source::TCP, s_tcpAsm, buf, (size_t)n);
      }
    } else if (s_client) {
      s_client.stop();
    }
  }
}

void feedUsb(const uint8_t *data, size_t len) {
  feedBytes(Source::USB, s_usbAsm, data, len);
}

void drainPending() {
  // Nothing consumes a stop request when no run exists; do not leave one pending.
  EventBits_t bits = xEventGroupGetBits(controlEvents);
  if (!(bits & BIT_AUTO_RUNNING) && (bits & BIT_ESTOP_REQUEST)) {
    xEventGroupClearBits(controlEvents, BIT_ESTOP_REQUEST);
  }
  // A remote STOP hit a run that has now ended: halt motor and arm once more, so an arm
  // target that landed after STOP does not carry on. The actuator is not touched.
  if (s_haltAfterRun && !(bits & BIT_AUTO_RUNNING)) {
    s_haltAfterRun = false;
    haltMotorAndArm();
  }

  if (s_cmdQueue == nullptr || s_replyQueue == nullptr) return;
  Request req;
  while (xQueueReceive(s_cmdQueue, &req, 0) == pdTRUE) {
    if (requestExpired(xTaskGetTickCount(), req.deadline)) {
      continue;  // requester already answered TIMEOUT; running it now would surprise it
    }
    execute(req);
  }
}

}  // namespace RemoteControl
