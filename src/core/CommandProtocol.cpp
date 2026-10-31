#include "core/CommandProtocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace cmdproto {

namespace {

constexpr int MAX_TOKENS = 4;  // [#id] VERB ARG1 ARG2 (+1 to detect extras)

char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + ('a' - 'A')) : c; }

bool isSpace(char c) { return c == ' ' || c == '\t'; }

// Case-insensitive compare of token against a lowercase literal.
bool tokenIs(const char* tok, const char* lit) {
  while (*tok && *lit) {
    if (lower(*tok) != *lit) return false;
    ++tok;
    ++lit;
  }
  return *tok == '\0' && *lit == '\0';
}

// True when every character is one of the given set (and token non-empty).
bool onlyChars(const char* tok, const char* allowed) {
  if (*tok == '\0') return false;
  for (; *tok; ++tok) {
    if (!strchr(allowed, *tok)) return false;
  }
  return true;
}

bool parseInt(const char* tok, long& out) {
  if (!onlyChars(tok, "0123456789+-")) return false;
  char* end = nullptr;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end != '\0') return false;
  out = v;
  return true;
}

// Plain decimal floats only: rejects nan, inf, hex floats, 12abc.
bool parseFloat(const char* tok, float& out) {
  if (!onlyChars(tok, "0123456789+-.eE")) return false;
  char* end = nullptr;
  float v = strtof(tok, &end);
  if (end == tok || *end != '\0') return false;
  if (!(v - v == 0.0f)) return false;  // NaN or +-inf (overflowed)
  out = v;
  return true;
}

ErrorCode parseTarget3(const char* tok, Target& t) {
  if (tokenIs(tok, "bot")) t = Target::BOT;
  else if (tokenIs(tok, "mid")) t = Target::MID;
  else if (tokenIs(tok, "top")) t = Target::TOP;
  else return ErrorCode::BAD_ARG;
  return ErrorCode::NONE;
}

ErrorCode parseMove(char* const* tok, int n, Command& out) {
  if (n != 2) return ErrorCode::BAD_ARG;
  if (tokenIs(tok[0], "z")) out.axis = Axis::Z;
  else if (tokenIs(tok[0], "arm")) out.axis = Axis::ARM;
  else if (tokenIs(tok[0], "act")) out.axis = Axis::ACT;
  else return ErrorCode::BAD_ARG;

  if (out.axis != Axis::ARM) return parseTarget3(tok[1], out.target);

  if (tokenIs(tok[1], "out")) out.target = Target::OUT;
  else if (tokenIs(tok[1], "buffer")) out.target = Target::BUFFER;
  else if (tokenIs(tok[1], "in")) out.target = Target::IN;
  else {
    long v = 0;
    if (!parseInt(tok[1], v) || v < 0 || v > 100) return ErrorCode::BAD_ARG;
    out.target = Target::PERCENT;
    out.percent = (int)v;
  }
  return ErrorCode::NONE;
}

ErrorCode parseJog(char* const* tok, int n, Command& out) {
  if (n != 2) return ErrorCode::BAD_ARG;
  if (tokenIs(tok[0], "z")) {
    out.axis = Axis::Z;
    float v = 0.0f;
    if (!parseFloat(tok[1], v) || v > MAX_JOG_Z || v < -MAX_JOG_Z) return ErrorCode::BAD_ARG;
    out.jogZ = v;
    return ErrorCode::NONE;
  }
  int limit;
  if (tokenIs(tok[0], "arm")) {
    out.axis = Axis::ARM;
    limit = MAX_JOG_ARM_STEPS;
  } else if (tokenIs(tok[0], "act")) {
    out.axis = Axis::ACT;
    limit = MAX_JOG_ACT_PCT;
  } else {
    return ErrorCode::BAD_ARG;
  }
  long v = 0;
  if (!parseInt(tok[1], v) || v > limit || v < -limit) return ErrorCode::BAD_ARG;
  out.jogInt = (int)v;
  return ErrorCode::NONE;
}

}  // namespace

const char* errorName(ErrorCode code) {
  switch (code) {
    case ErrorCode::BAD_CMD: return "BAD_CMD";
    case ErrorCode::BAD_ARG: return "BAD_ARG";
    case ErrorCode::TOO_LONG: return "TOO_LONG";
    case ErrorCode::BUSY: return "BUSY";
    case ErrorCode::NOT_READY: return "NOT_READY";
    case ErrorCode::UNKNOWN_SEQ: return "UNKNOWN_SEQ";
    case ErrorCode::TIMEOUT: return "TIMEOUT";
    case ErrorCode::OTA: return "OTA";
    default: return "";
  }
}

int limitIndex(Target t) {
  switch (t) {
    case Target::BOT: return 0;
    case Target::MID: return 1;
    case Target::TOP: return 2;
    default: return -1;
  }
}

ParseResult parseCommand(const char* line, Command& out) {
  out = Command();
  ParseResult res;
  if (line == nullptr) {
    res.error = ErrorCode::BAD_CMD;
    return res;
  }

  size_t len = strlen(line);
  if (len > MAX_LINE) {
    res.error = ErrorCode::TOO_LONG;
    return res;
  }

  // Private mutable copy, tokenised in place on spaces/tabs/CR/LF.
  char work[MAX_LINE + 1];
  memcpy(work, line, len + 1);

  char* tok[MAX_TOKENS + 1];
  int n = 0;
  char* p = work;
  while (*p) {
    while (*p && (isSpace(*p) || *p == '\r' || *p == '\n')) ++p;
    if (!*p) break;
    if (n > MAX_TOKENS) {  // more tokens than any command takes
      res.error = ErrorCode::BAD_ARG;
      return res;
    }
    tok[n++] = p;
    while (*p && !(isSpace(*p) || *p == '\r' || *p == '\n')) ++p;
    if (*p) *p++ = '\0';
  }

  int first = 0;
  if (n > 0 && tok[0][0] == '#') {
    long v = 0;
    if (!parseInt(tok[0] + 1, v) || tok[0][1] == '+' || tok[0][1] == '-' || v < 1 ||
        v > (long)MAX_ID) {
      res.error = ErrorCode::BAD_ARG;
      return res;
    }
    out.id = (uint32_t)v;
    first = 1;
  }

  if (n <= first) {
    res.error = ErrorCode::BAD_CMD;
    return res;
  }

  const char* verb = tok[first];
  char* const* args = tok + first + 1;
  int nargs = n - first - 1;

  if (tokenIs(verb, "ping")) out.verb = Verb::PING;
  else if (tokenIs(verb, "status")) out.verb = Verb::STATUS;
  else if (tokenIs(verb, "seqs")) out.verb = Verb::SEQS;
  else if (tokenIs(verb, "stop")) out.verb = Verb::STOP;
  else if (tokenIs(verb, "run")) out.verb = Verb::RUN;
  else if (tokenIs(verb, "move")) out.verb = Verb::MOVE;
  else if (tokenIs(verb, "jog")) out.verb = Verb::JOG;
  else {
    res.error = ErrorCode::BAD_CMD;
    return res;
  }

  switch (out.verb) {
    case Verb::RUN:
      if (nargs != 1) {
        res.error = ErrorCode::BAD_ARG;
      } else if (tokenIs(args[0], "auto")) {
        out.seq = SeqName::AUTO;
      } else if (tokenIs(args[0], "park")) {
        out.seq = SeqName::PARK;
      } else {
        res.error = ErrorCode::UNKNOWN_SEQ;
      }
      break;
    case Verb::MOVE:
      res.error = parseMove(args, nargs, out);
      break;
    case Verb::JOG:
      res.error = parseJog(args, nargs, out);
      break;
    default:  // PING, STATUS, SEQS, STOP take no arguments
      if (nargs != 0) res.error = ErrorCode::BAD_ARG;
      break;
  }
  return res;
}

LineAssembler::Event LineAssembler::feed(char c) {
  if (done_) {  // previous call handed out the line; start fresh
    done_ = false;
    len_ = 0;
    buf_[0] = '\0';
  }
  if (c == '\r' || c == '\0') return Event::NONE;

  if (discard_) {
    if (c == '\n') discard_ = false;
    return Event::NONE;
  }

  if (c == '\n') {
    if (len_ == 0) return Event::NONE;
    buf_[len_] = '\0';
    done_ = true;
    return Event::LINE;
  }

  if (len_ >= MAX_LINE) {
    discard_ = true;
    len_ = 0;
    buf_[0] = '\0';
    return Event::LINE_OVERFLOW;
  }
  buf_[len_++] = c;
  buf_[len_] = '\0';
  return Event::NONE;
}

ErrorCode checkPolicy(const Command& cmd, const PolicySnapshot& snap) {
  switch (cmd.verb) {
    case Verb::RUN:
    case Verb::MOVE:
    case Verb::JOG:
      break;
    default:  // PING, STATUS, SEQS, STOP (and NONE) are always allowed
      return ErrorCode::NONE;
  }
  if (snap.otaActive) return ErrorCode::OTA;
  if (snap.busy) return ErrorCode::BUSY;
  if (cmd.verb == Verb::RUN && cmd.seq == SeqName::AUTO && !snap.ready) {
    return ErrorCode::NOT_READY;
  }
  if (cmd.verb == Verb::MOVE && cmd.axis == Axis::Z) {
    int idx = limitIndex(cmd.target);
    if (idx < 0 || !snap.zLimitSet[idx]) return ErrorCode::NOT_READY;
  }
  if (cmd.verb == Verb::MOVE && cmd.axis == Axis::ACT) {
    int idx = limitIndex(cmd.target);
    if (idx < 0 || !snap.actLimitSet[idx]) return ErrorCode::NOT_READY;
  }
  if ((cmd.verb == Verb::MOVE || cmd.verb == Verb::JOG) && cmd.axis == Axis::ARM) {
    // The arm node ignores targets without calibration and would read a buffer
    // target without a buffer point as 200 %; a JOG is a tracked target too
    // (the glue re-targets the current position first), so all need the calibration.
    if (!snap.armCalibrated) return ErrorCode::NOT_READY;
    if (cmd.verb == Verb::MOVE && cmd.target == Target::BUFFER && !snap.armBufferSet) {
      return ErrorCode::NOT_READY;
    }
  }
  return ErrorCode::NONE;
}

namespace {

// Finalise a snprintf into buf: guarantee a trailing '\n' within size.
size_t finish(char* buf, size_t size, int written) {
  if (size == 0) return 0;
  if (size == 1) {
    buf[0] = '\0';
    return 0;
  }
  size_t n;
  if (written < 0) {
    buf[0] = '\n';
    buf[1] = '\0';
    return 1;
  }
  if ((size_t)written >= size - 1) {  // truncated (no room for the newline)
    n = size - 1;
    buf[n - 1] = '\n';
  } else {
    n = (size_t)written;
  }
  buf[n] = '\0';
  return n;
}

const char* seqText(SeqName s) {
  switch (s) {
    case SeqName::AUTO: return "auto";
    case SeqName::PARK: return "park";
    case SeqName::GOTO: return "goto";
    default: return "none";
  }
}

const char* resultText(Result r) {
  switch (r) {
    case Result::DONE: return "DONE";
    case Result::ABORTED: return "ABORTED";
    default: return "NONE";
  }
}

}  // namespace

size_t formatOk(char* buf, size_t size, uint32_t id, const char* fields) {
  if (size == 0) return 0;
  int w;
  if (fields && fields[0]) {
    w = snprintf(buf, size, "@%lu OK %s\n", (unsigned long)id, fields);
  } else {
    w = snprintf(buf, size, "@%lu OK\n", (unsigned long)id);
  }
  return finish(buf, size, w);
}

size_t formatErr(char* buf, size_t size, uint32_t id, ErrorCode code, const char* text) {
  if (size == 0) return 0;
  int w;
  if (text && text[0]) {
    w = snprintf(buf, size, "@%lu ERR %s %s\n", (unsigned long)id, errorName(code), text);
  } else {
    w = snprintf(buf, size, "@%lu ERR %s\n", (unsigned long)id, errorName(code));
  }
  return finish(buf, size, w);
}

size_t formatStatus(char* buf, size_t size, uint32_t id, const StatusFields& s) {
  if (size == 0) return 0;
  int w = snprintf(buf, size,
                   "@%lu OK state=%s run=%lu seq=%s step=%d steps=%d result=%s estop=%d "
                   "ready=%d z=%.3f zhomed=%d zmoving=%d arm=%.0f armmoving=%d act=%.1f "
                   "collision=%d ota=%d\n",
                   (unsigned long)id, s.running ? "RUNNING" : "IDLE", (unsigned long)s.run,
                   seqText(s.seq), s.step, s.steps, resultText(s.result), s.estop ? 1 : 0,
                   s.ready ? 1 : 0, (double)s.z, s.zhomed ? 1 : 0, s.zmoving ? 1 : 0,
                   (double)s.arm, s.armmoving ? 1 : 0, (double)s.act, s.collision ? 1 : 0,
                   s.ota ? 1 : 0);
  return finish(buf, size, w);
}

float armMoveTarget(Target t, int percent) {
  switch (t) {
    case Target::OUT: return (float)ARM_TARGET_OUT;
    case Target::IN: return (float)ARM_TARGET_IN;
    case Target::BUFFER: return (float)ARM_TARGET_BUFFER;
    default: return (float)percent;
  }
}

float armPercentOf(float pos, int posOut, int posIn) {
  if (posIn == posOut) return 0.0f;
  return 100.0f * (pos - (float)posOut) / (float)(posIn - posOut);
}

int actuatorJogTarget(float currentPercent, int jog) {
  int target = (int)lroundf(currentPercent) + jog;
  if (target < 0) target = 0;
  if (target > 100) target = 100;
  return target;
}

bool readyForAuto(const bool zLimitSet[3], const bool actLimitSet[3], int armPosOut, int armPosIn) {
  for (int i = 0; i < 3; i++) {
    if (!zLimitSet[i] || !actLimitSet[i]) return false;
  }
  return armPosOut != -1 && armPosIn != -1;
}

bool requestExpired(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

uint32_t requestDeadline(uint32_t start, uint32_t timeout, uint32_t margin) {
  if (margin > timeout) margin = timeout;
  return start + timeout - margin;
}

}  // namespace cmdproto
