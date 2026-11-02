// Remote command protocol v1 (see docs/PROTOCOL.md).
//
// Pure, hardware-free unit: parses request lines, applies the busy/ready
// policy and formats replies. No Arduino, FreeRTOS, heap or STL, so it builds
// on the ESP32 and in the host tests (test/host/run_tests.sh).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cmdproto {

constexpr int PROTOCOL_VERSION = 1;
constexpr size_t MAX_LINE = 96;  // payload bytes, excluding newline and NUL
constexpr uint32_t MAX_ID = 999999;
constexpr int MAX_JOG_ARM_STEPS = 2000;
constexpr int MAX_JOG_ACT_PCT = 20;
constexpr float MAX_JOG_Z = 1.0f;

enum class ErrorCode : uint8_t {
  NONE = 0,  // success; never printed
  BAD_CMD,
  BAD_ARG,
  TOO_LONG,
  BUSY,
  NOT_READY,
  UNKNOWN_SEQ,
  TIMEOUT,
  OTA,
};

// Wire name of a code ("BAD_CMD", ...); "" for NONE.
const char* errorName(ErrorCode code);

enum class Verb : uint8_t { NONE = 0, PING, STATUS, SEQS, STOP, RUN, MOVE, JOG };

enum class Axis : uint8_t { NONE = 0, Z, ARM, ACT };

enum class SeqName : uint8_t { NONE = 0, AUTO, PARK, GOTO };  // GOTO only appears in STATUS

enum class Result : uint8_t { NONE = 0, DONE, ABORTED };

// MOVE target. Z and ACT use BOT/MID/TOP; ARM uses OUT/BUFFER/IN or PERCENT.
enum class Target : uint8_t { NONE = 0, BOT, MID, TOP, OUT, BUFFER, IN, PERCENT };

// Limit index for zLimitSet[] and "target limit index" in SystemState.h
// (0=Bot, 1=Mid, 2=Top). Returns -1 for non-limit targets.
int limitIndex(Target t);

struct Command {
  Verb verb = Verb::NONE;
  uint32_t id = 0;            // 0 when the request had no (valid) id
  SeqName seq = SeqName::NONE;  // RUN
  Axis axis = Axis::NONE;     // MOVE, JOG
  Target target = Target::NONE;  // MOVE
  int percent = 0;            // MOVE arm <0-100>  (target == PERCENT)
  float jogZ = 0.0f;          // JOG z
  int jogInt = 0;             // JOG arm steps / act percent
};

struct ParseResult {
  ErrorCode error = ErrorCode::NONE;  // NONE == success
  bool ok() const { return error == ErrorCode::NONE; }
};

// Parses one complete line (no trailing newline needed; '\r' tolerated).
// out.id is filled in whenever the id prefix was valid, even on error, so the
// caller can address the ERR reply. A malformed id yields BAD_ARG with id 0.
ParseResult parseCommand(const char* line, Command& out);

// Accumulates bytes into lines. One instance per input channel.
class LineAssembler {
 public:
  // (not named OVERFLOW: some libc math.h headers define that as a macro)
  enum class Event : uint8_t { NONE, LINE, LINE_OVERFLOW };

  // LINE: line() holds a complete non-empty line (valid until the next feed).
  // LINE_OVERFLOW: reported once per oversized line; the rest of it is discarded
  // up to and including the next '\n'.
  Event feed(char c);
  const char* line() const { return buf_; }
  size_t length() const { return len_; }

 private:
  char buf_[MAX_LINE + 1] = {0};
  size_t len_ = 0;
  bool done_ = false;     // previous feed returned LINE; reset on next feed
  bool discard_ = false;  // dropping the tail of an oversized line
};

// State the glue samples (in controller_task) before executing a command.
struct PolicySnapshot {
  bool busy = false;      // sequence or goto running
  bool ready = false;     // encoder prerequisites for RUN auto satisfied
  bool otaActive = false;
  bool zLimitSet[3] = {false, false, false};    // Bot, Mid, Top
  bool actLimitSet[3] = {false, false, false};  // Bot, Mid, Top
  bool armCalibrated = false;  // arm Out and In both set
  bool armBufferSet = false;   // arm Buffer set
};

// Returns ErrorCode::NONE when the command is allowed.
ErrorCode checkPolicy(const Command& cmd, const PolicySnapshot& snap);

struct StatusFields {
  bool running = false;
  uint32_t run = 0;
  SeqName seq = SeqName::NONE;
  int step = 0;
  int steps = 0;
  Result result = Result::NONE;
  bool estop = false;
  bool ready = false;
  float z = 0.0f;
  bool zhomed = false;
  bool zmoving = false;
  float arm = 0.0f;
  bool armmoving = false;
  float act = 0.0f;
  bool collision = false;
  bool ota = false;
};

// All formatters write a NUL-terminated line ending in '\n' into buf (when
// size >= 2) and return its length excluding the NUL. They never write past
// size; an over-long reply is truncated but still ends in '\n'. size 0 writes
// nothing; size 1 writes just the NUL. Returns 0 in those two cases.

// "@<id> OK[ <fields>]\n"; fields is e.g. "run=7" or nullptr/"" for none.
size_t formatOk(char* buf, size_t size, uint32_t id, const char* fields = nullptr);

// "@<id> ERR <CODE>[ <text>]\n"; text may be nullptr/"".
size_t formatErr(char* buf, size_t size, uint32_t id, ErrorCode code, const char* text = nullptr);

// "@<id> OK state=... run=... \n" in the fixed D01 key order.
size_t formatStatus(char* buf, size_t size, uint32_t id, const StatusFields& s);

// ---------------------------------------------------------------------------
// Glue decisions kept pure so the host tests cover them (used by RemoteControl).
// ---------------------------------------------------------------------------

// Arm target convention of ArmNode / the sequence tables: 0 = out, 100 = in, 200 = buffer,
// otherwise a percent between Out and In. Same value as SEQ_TARGET_BUFFER.
constexpr int ARM_TARGET_OUT = 0;
constexpr int ARM_TARGET_IN = 100;
constexpr int ARM_TARGET_BUFFER = 200;

// Arm target for MOVE arm. `percent` is used only when t == Target::PERCENT.
float armMoveTarget(Target t, int percent);

// Where the arm is now, in the same 0..100 convention (100 * (pos-out)/(in-out)).
// Returns 0 when in == out (callers only use it for a calibrated arm).
float armPercentOf(float pos, int posOut, int posIn);

// JOG act target: current percent (rounded) plus jog, clamped to 0..100.
int actuatorJogTarget(float currentPercent, int jog);

// Same prerequisites as the encoder's Auto launch (InputManager.cpp, S4_AUTO): all three Z and
// actuator limits set, arm Out and In set (-1 = unset).
bool readyForAuto(const bool zLimitSet[3], const bool actLimitSet[3], int armPosOut, int armPosIn);

// A controller-side request is dropped once now has reached its deadline. Tick arithmetic
// is modulo 2^32 (wrap-safe). The requester answers TIMEOUT at now == deadline + margin,
// so a request executed at the boundary tick can never be one the client was told timed out.
bool requestExpired(uint32_t now, uint32_t deadline);

// deadline = start + timeout - margin (the controller stops executing `margin` ticks before
// the requester gives up). margin is clamped so the deadline never precedes start.
uint32_t requestDeadline(uint32_t start, uint32_t timeout, uint32_t margin);

}  // namespace cmdproto
