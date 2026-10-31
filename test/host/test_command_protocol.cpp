// Host tests for src/core/CommandProtocol (no hardware, no Arduino).
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "core/CommandProtocol.h"

using namespace cmdproto;

static int g_fail = 0;
static int g_checks = 0;

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_fail;                                                                  \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                     \
    }                                                                            \
  } while (0)

static ErrorCode parse(const char* line, Command& c) { return parseCommand(line, c).error; }

static ErrorCode parseOnly(const char* line) {
  Command c;
  return parse(line, c);
}

static void testSimpleVerbs() {
  struct { const char* line; Verb verb; } cases[] = {
      {"PING", Verb::PING}, {"STATUS", Verb::STATUS}, {"SEQS", Verb::SEQS}, {"STOP", Verb::STOP}};
  for (auto& k : cases) {
    Command c;
    CHECK(parse(k.line, c) == ErrorCode::NONE);
    CHECK(c.verb == k.verb);
    CHECK(c.id == 0);
    std::string withId = std::string("#42 ") + k.line;
    CHECK(parse(withId.c_str(), c) == ErrorCode::NONE);
    CHECK(c.verb == k.verb);
    CHECK(c.id == 42);
    std::string crlf = std::string("#7 ") + k.line + "\r\n";
    CHECK(parse(crlf.c_str(), c) == ErrorCode::NONE);
    CHECK(c.id == 7);
  }
  // Extra args on arg-less verbs
  CHECK(parseOnly("PING now") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("#3 STOP 1") == ErrorCode::BAD_ARG);
}

static void testCase() {
  Command c;
  CHECK(parse("pInG", c) == ErrorCode::NONE && c.verb == Verb::PING);
  CHECK(parse("run AUTO", c) == ErrorCode::NONE && c.seq == SeqName::AUTO);
  CHECK(parse("Run Park", c) == ErrorCode::NONE && c.seq == SeqName::PARK);
  CHECK(parse("move Z TOP", c) == ErrorCode::NONE && c.axis == Axis::Z && c.target == Target::TOP);
  CHECK(parse("  \t#5   status  ", c) == ErrorCode::NONE && c.id == 5);
}

static void testRun() {
  Command c;
  CHECK(parse("#9 RUN auto", c) == ErrorCode::NONE);
  CHECK(c.verb == Verb::RUN && c.seq == SeqName::AUTO && c.id == 9);
  CHECK(parse("RUN park", c) == ErrorCode::NONE && c.seq == SeqName::PARK);
  CHECK(parse("#9 RUN bogus", c) == ErrorCode::UNKNOWN_SEQ);
  CHECK(c.id == 9);
  CHECK(parseOnly("RUN") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("RUN auto park") == ErrorCode::BAD_ARG);
}

static void testMove() {
  Command c;
  CHECK(parse("MOVE z bot", c) == ErrorCode::NONE);
  CHECK(c.verb == Verb::MOVE && c.axis == Axis::Z && c.target == Target::BOT);
  CHECK(parse("MOVE z mid", c) == ErrorCode::NONE && c.target == Target::MID);
  CHECK(parse("#2 MOVE act top", c) == ErrorCode::NONE);
  CHECK(c.axis == Axis::ACT && c.target == Target::TOP && c.id == 2);
  CHECK(parse("MOVE arm out", c) == ErrorCode::NONE && c.target == Target::OUT);
  CHECK(parse("MOVE arm buffer", c) == ErrorCode::NONE && c.target == Target::BUFFER);
  CHECK(parse("MOVE arm in", c) == ErrorCode::NONE && c.target == Target::IN);
  CHECK(parse("MOVE arm 0", c) == ErrorCode::NONE && c.target == Target::PERCENT && c.percent == 0);
  CHECK(parse("MOVE arm 100", c) == ErrorCode::NONE && c.percent == 100);
  CHECK(parse("MOVE arm 55", c) == ErrorCode::NONE && c.percent == 55);

  CHECK(parseOnly("MOVE arm 101") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE arm -1") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE arm 5x") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE arm 1.5") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE z out") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE z 50") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE act in") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE x top") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE z") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("MOVE z top now") == ErrorCode::BAD_ARG);
}

static void testJog() {
  Command c;
  CHECK(parse("JOG z 0.25", c) == ErrorCode::NONE);
  CHECK(c.verb == Verb::JOG && c.axis == Axis::Z && fabsf(c.jogZ - 0.25f) < 1e-6f);
  CHECK(parse("JOG z -1.0", c) == ErrorCode::NONE && fabsf(c.jogZ + 1.0f) < 1e-6f);
  CHECK(parse("JOG z 1", c) == ErrorCode::NONE && c.jogZ == 1.0f);
  CHECK(parse("#4 JOG arm -2000", c) == ErrorCode::NONE);
  CHECK(c.axis == Axis::ARM && c.jogInt == -2000 && c.id == 4);
  CHECK(parse("JOG arm 2000", c) == ErrorCode::NONE && c.jogInt == 2000);
  CHECK(parse("JOG act 20", c) == ErrorCode::NONE && c.axis == Axis::ACT && c.jogInt == 20);
  CHECK(parse("JOG act -20", c) == ErrorCode::NONE && c.jogInt == -20);
  CHECK(parse("JOG act +5", c) == ErrorCode::NONE && c.jogInt == 5);

  // out of range
  CHECK(parseOnly("JOG z 1.01") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z -1.01") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z 1e30") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z 1e999") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm 2001") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm -2001") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG act 21") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG act 99999999999999999999") == ErrorCode::BAD_ARG);
  // non-numeric / non-finite
  CHECK(parseOnly("JOG z abc") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z 0.5abc") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z nan") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z NaN") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z inf") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z -inf") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z infinity") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z 0x1") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm 12abc") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm 1.5") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm nan") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG arm") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG z 0.1 0.2") == ErrorCode::BAD_ARG);
  CHECK(parseOnly("JOG x 1") == ErrorCode::BAD_ARG);
}

static void testMalformed() {
  Command c;
  CHECK(parse("FLY", c) == ErrorCode::BAD_CMD);
  CHECK(parse("#8 FLY now", c) == ErrorCode::BAD_CMD);
  CHECK(c.id == 8);  // id retained so the ERR reply can be addressed
  CHECK(parse("#8", c) == ErrorCode::BAD_CMD);
  CHECK(parse("", c) == ErrorCode::BAD_CMD);
  CHECK(parse("   ", c) == ErrorCode::BAD_CMD);

  CHECK(parse("#0 PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#abc PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#9999999 PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#-5 PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#+5 PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#12abc PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("# PING", c) == ErrorCode::BAD_ARG && c.id == 0);
  CHECK(parse("#999999 PING", c) == ErrorCode::NONE && c.id == 999999);
  CHECK(parse("#1 PING", c) == ErrorCode::NONE && c.id == 1);
  CHECK(parse("PING #5", c) == ErrorCode::BAD_ARG);  // id must be first
}

static void testAssembler() {
  LineAssembler a;
  // Normal line with CRLF
  const char* s = "PING\r\n";
  int lines = 0;
  for (const char* p = s; *p; ++p) {
    if (a.feed(*p) == LineAssembler::Event::LINE) {
      ++lines;
      CHECK(strcmp(a.line(), "PING") == 0);
    }
  }
  CHECK(lines == 1);

  // Empty lines produce nothing
  CHECK(a.feed('\n') == LineAssembler::Event::NONE);
  CHECK(a.feed('\r') == LineAssembler::Event::NONE);
  CHECK(a.feed('\n') == LineAssembler::Event::NONE);

  // Back-to-back lines
  const char* two = "STATUS\nSEQS\n";
  std::string got;
  for (const char* p = two; *p; ++p) {
    if (a.feed(*p) == LineAssembler::Event::LINE) got += std::string(a.line()) + "|";
  }
  CHECK(got == "STATUS|SEQS|");

  // Exactly 96 bytes accepted
  {
    LineAssembler b;
    std::string l(96, 'a');
    int ov = 0, ln = 0;
    for (char ch : l) {
      auto e = b.feed(ch);
      if (e == LineAssembler::Event::LINE_OVERFLOW) ++ov;
    }
    auto e = b.feed('\n');
    if (e == LineAssembler::Event::LINE) {
      ++ln;
      CHECK(strlen(b.line()) == 96);
    }
    CHECK(ov == 0 && ln == 1);
  }
  // 97 overflows (once), then the next line is fine
  {
    LineAssembler b;
    int ov = 0, ln = 0;
    std::string in = std::string(97, 'a') + "\n" + "PING\n";
    for (char ch : in) {
      auto e = b.feed(ch);
      if (e == LineAssembler::Event::LINE_OVERFLOW) ++ov;
      if (e == LineAssembler::Event::LINE) {
        ++ln;
        CHECK(strcmp(b.line(), "PING") == 0);
      }
    }
    CHECK(ov == 1 && ln == 1);
  }
  // 200 bytes: exactly one OVERFLOW, the tail is discarded, next line parses
  {
    LineAssembler b;
    int ov = 0, ln = 0;
    std::string in = std::string(200, 'x') + "\r\n" + "#3 PING\r\n";
    Command c;
    for (char ch : in) {
      auto e = b.feed(ch);
      if (e == LineAssembler::Event::LINE_OVERFLOW) ++ov;
      if (e == LineAssembler::Event::LINE) {
        ++ln;
        CHECK(parse(b.line(), c) == ErrorCode::NONE);
        CHECK(c.verb == Verb::PING && c.id == 3);
      }
    }
    CHECK(ov == 1 && ln == 1);
  }
  // Overflow with no terminating newline yet: still one report, then silence
  {
    LineAssembler b;
    int ov = 0;
    for (int i = 0; i < 500; ++i) {
      if (b.feed('y') == LineAssembler::Event::LINE_OVERFLOW) ++ov;
    }
    CHECK(ov == 1);
    CHECK(b.feed('\n') == LineAssembler::Event::NONE);
    CHECK(b.feed('P') == LineAssembler::Event::NONE);
  }
  // parseCommand refuses over-long input handed to it directly
  {
    std::string big(120, 'a');
    CHECK(parseOnly(big.c_str()) == ErrorCode::TOO_LONG);
  }
}

static PolicySnapshot idleReady() {
  PolicySnapshot s;
  s.ready = true;
  s.zLimitSet[0] = s.zLimitSet[1] = s.zLimitSet[2] = true;
  s.actLimitSet[0] = s.actLimitSet[1] = s.actLimitSet[2] = true;
  s.armCalibrated = true;
  s.armBufferSet = true;
  return s;
}

static ErrorCode policy(const char* line, const PolicySnapshot& s) {
  Command c;
  CHECK(parse(line, c) == ErrorCode::NONE);
  return checkPolicy(c, s);
}

static void testPolicy() {
  PolicySnapshot ok = idleReady();
  CHECK(policy("RUN auto", ok) == ErrorCode::NONE);
  CHECK(policy("RUN park", ok) == ErrorCode::NONE);
  CHECK(policy("MOVE z top", ok) == ErrorCode::NONE);
  CHECK(policy("MOVE arm 10", ok) == ErrorCode::NONE);
  CHECK(policy("JOG z 0.1", ok) == ErrorCode::NONE);

  PolicySnapshot busy = idleReady();
  busy.busy = true;
  CHECK(policy("PING", busy) == ErrorCode::NONE);
  CHECK(policy("STATUS", busy) == ErrorCode::NONE);
  CHECK(policy("SEQS", busy) == ErrorCode::NONE);
  CHECK(policy("STOP", busy) == ErrorCode::NONE);
  CHECK(policy("RUN auto", busy) == ErrorCode::BUSY);
  CHECK(policy("RUN park", busy) == ErrorCode::BUSY);
  CHECK(policy("MOVE z top", busy) == ErrorCode::BUSY);
  CHECK(policy("MOVE arm out", busy) == ErrorCode::BUSY);
  CHECK(policy("JOG arm 5", busy) == ErrorCode::BUSY);
  CHECK(policy("JOG act 5", busy) == ErrorCode::BUSY);

  PolicySnapshot ota = idleReady();
  ota.otaActive = true;
  CHECK(policy("STOP", ota) == ErrorCode::NONE);
  CHECK(policy("STATUS", ota) == ErrorCode::NONE);
  CHECK(policy("PING", ota) == ErrorCode::NONE);
  CHECK(policy("RUN park", ota) == ErrorCode::OTA);
  CHECK(policy("MOVE act mid", ota) == ErrorCode::OTA);
  CHECK(policy("JOG z 0.1", ota) == ErrorCode::OTA);
  ota.busy = true;  // OTA wins over BUSY
  CHECK(policy("RUN auto", ota) == ErrorCode::OTA);

  PolicySnapshot notReady = idleReady();
  notReady.ready = false;
  CHECK(policy("RUN auto", notReady) == ErrorCode::NOT_READY);
  CHECK(policy("RUN park", notReady) == ErrorCode::NONE);  // park needs no prerequisites
  CHECK(policy("MOVE arm in", notReady) == ErrorCode::NONE);

  PolicySnapshot lim = idleReady();
  lim.zLimitSet[1] = false;  // mid unset
  CHECK(policy("MOVE z mid", lim) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE z top", lim) == ErrorCode::NONE);
  CHECK(policy("MOVE z bot", lim) == ErrorCode::NONE);
  CHECK(policy("MOVE act mid", lim) == ErrorCode::NONE);  // only z checks z limits
  lim.zLimitSet[0] = lim.zLimitSet[2] = false;
  CHECK(policy("MOVE z bot", lim) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE z top", lim) == ErrorCode::NOT_READY);
  CHECK(policy("JOG z 0.1", lim) == ErrorCode::NONE);  // jog needs no limits

  PolicySnapshot act = idleReady();
  act.actLimitSet[2] = false;  // top unset
  CHECK(policy("MOVE act top", act) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE act mid", act) == ErrorCode::NONE);
  CHECK(policy("MOVE act bot", act) == ErrorCode::NONE);
  CHECK(policy("MOVE z top", act) == ErrorCode::NONE);  // z unaffected
  CHECK(policy("JOG act 5", act) == ErrorCode::NONE);   // jog needs no limits
  act.busy = true;
  CHECK(policy("MOVE act top", act) == ErrorCode::BUSY);  // BUSY before NOT_READY

  PolicySnapshot arm = idleReady();
  arm.armBufferSet = false;
  CHECK(policy("MOVE arm buffer", arm) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE arm out", arm) == ErrorCode::NONE);
  CHECK(policy("MOVE arm in", arm) == ErrorCode::NONE);
  CHECK(policy("MOVE arm 50", arm) == ErrorCode::NONE);
  arm.armBufferSet = true;
  CHECK(policy("MOVE arm buffer", arm) == ErrorCode::NONE);
  arm.armCalibrated = false;
  CHECK(policy("MOVE arm out", arm) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE arm in", arm) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE arm buffer", arm) == ErrorCode::NOT_READY);
  CHECK(policy("MOVE arm 0", arm) == ErrorCode::NOT_READY);
  CHECK(policy("JOG arm 50", arm) == ErrorCode::NOT_READY);  // arm jog is a tracked target
  CHECK(policy("JOG act 5", arm) == ErrorCode::NONE);
  CHECK(policy("JOG z 0.1", arm) == ErrorCode::NONE);
  CHECK(policy("RUN park", arm) == ErrorCode::NONE);
  arm.armCalibrated = true;
  CHECK(policy("JOG arm -50", arm) == ErrorCode::NONE);
  arm.otaActive = true;
  arm.armCalibrated = false;
  CHECK(policy("MOVE arm out", arm) == ErrorCode::OTA);  // OTA before NOT_READY
}

static void testFormat() {
  char buf[512];
  size_t n;

  n = formatOk(buf, sizeof buf, 0);
  CHECK(strcmp(buf, "@0 OK\n") == 0 && n == strlen(buf));
  n = formatOk(buf, sizeof buf, 12, "pong proto=1");
  CHECK(strcmp(buf, "@12 OK pong proto=1\n") == 0 && n == strlen(buf));
  n = formatOk(buf, sizeof buf, 3, "run=7");
  CHECK(strcmp(buf, "@3 OK run=7\n") == 0);
  n = formatOk(buf, sizeof buf, 3, "");
  CHECK(strcmp(buf, "@3 OK\n") == 0);

  n = formatErr(buf, sizeof buf, 5, ErrorCode::BUSY);
  CHECK(strcmp(buf, "@5 ERR BUSY\n") == 0 && n == strlen(buf));
  n = formatErr(buf, sizeof buf, 0, ErrorCode::TOO_LONG, "line exceeds 96 bytes");
  CHECK(strcmp(buf, "@0 ERR TOO_LONG line exceeds 96 bytes\n") == 0);

  const ErrorCode all[] = {ErrorCode::BAD_CMD, ErrorCode::BAD_ARG,     ErrorCode::TOO_LONG,
                           ErrorCode::BUSY,    ErrorCode::NOT_READY,   ErrorCode::UNKNOWN_SEQ,
                           ErrorCode::TIMEOUT, ErrorCode::OTA};
  const char* names[] = {"BAD_CMD", "BAD_ARG",     "TOO_LONG", "BUSY",
                         "NOT_READY", "UNKNOWN_SEQ", "TIMEOUT", "OTA"};
  for (size_t i = 0; i < 8; ++i) CHECK(strcmp(errorName(all[i]), names[i]) == 0);

  StatusFields s;
  n = formatStatus(buf, sizeof buf, 11, s);
  CHECK(strcmp(buf,
               "@11 OK state=IDLE run=0 seq=none step=0 steps=0 result=NONE estop=0 ready=0 "
               "z=0.000 zhomed=0 zmoving=0 arm=0 armmoving=0 act=0.0 collision=0 ota=0\n") == 0);
  CHECK(n == strlen(buf));

  s.running = true;
  s.run = 42;
  s.seq = SeqName::AUTO;
  s.step = 3;
  s.steps = 9;
  s.result = Result::NONE;
  s.estop = false;
  s.ready = true;
  s.z = 12.3456f;
  s.zhomed = true;
  s.zmoving = true;
  s.arm = 55.4f;
  s.armmoving = true;
  s.act = 33.33f;
  s.collision = true;
  s.ota = true;
  formatStatus(buf, sizeof buf, 99, s);
  CHECK(strcmp(buf,
               "@99 OK state=RUNNING run=42 seq=auto step=3 steps=9 result=NONE estop=0 ready=1 "
               "z=12.346 zhomed=1 zmoving=1 arm=55 armmoving=1 act=33.3 collision=1 ota=1\n") == 0);

  s.running = false;
  s.seq = SeqName::GOTO;
  s.result = Result::ABORTED;
  s.estop = true;
  formatStatus(buf, sizeof buf, 1, s);
  CHECK(strstr(buf, "state=IDLE run=42 seq=goto step=3 steps=9 result=ABORTED estop=1 ") != nullptr);
  s.seq = SeqName::PARK;
  s.result = Result::DONE;
  formatStatus(buf, sizeof buf, 1, s);
  CHECK(strstr(buf, "seq=park step=3 steps=9 result=DONE") != nullptr);
}

// Truncation: for every buffer size, never write past it, always NUL, and
// (size >= 2) always end in '\n'.
static void checkTruncation(const char* what, int kind) {
  StatusFields s;
  s.running = true;
  s.run = 123456;
  s.seq = SeqName::AUTO;
  for (size_t size = 0; size <= 140; ++size) {
    char guarded[160];
    memset(guarded, 0x5A, sizeof guarded);
    char* buf = guarded + 8;  // 8 guard bytes in front
    size_t n = 0;
    if (kind == 0) n = formatStatus(buf, size, 999999, s);
    else if (kind == 1) n = formatOk(buf, size, 999999, "pong proto=1 more=stuff");
    else n = formatErr(buf, size, 999999, ErrorCode::UNKNOWN_SEQ, "some long explanatory text");

    bool guardOk = true;
    for (size_t i = 0; i < 8; ++i) guardOk &= (guarded[i] == 0x5A);
    for (size_t i = 8 + size; i < sizeof guarded; ++i) guardOk &= (guarded[i] == 0x5A);
    if (!guardOk) printf("  overflow in %s at size %zu\n", what, size);
    CHECK(guardOk);
    if (size == 0) {
      CHECK(n == 0);
    } else {
      CHECK(buf[n] == '\0');
      CHECK(n < size);
      if (size >= 2) {
        CHECK(n >= 1 && buf[n - 1] == '\n');
        CHECK(strlen(buf) == n);
      }
    }
  }
}

static void testTruncation() {
  checkTruncation("status", 0);
  checkTruncation("ok", 1);
  checkTruncation("err", 2);
  char tiny[4];
  size_t n = formatStatus(tiny, sizeof tiny, 1, StatusFields());
  CHECK(n == 3 && strcmp(tiny, "@1\n") == 0);
  CHECK(formatOk(nullptr, 0, 1) == 0);
}

static void testGlue() {
  // MOVE arm target mapping: 0 out, 100 in, 200 buffer, else the percent itself
  CHECK(armMoveTarget(Target::OUT, 55) == 0.0f);
  CHECK(armMoveTarget(Target::IN, 55) == 100.0f);
  CHECK(armMoveTarget(Target::BUFFER, 55) == 200.0f);
  CHECK(armMoveTarget(Target::PERCENT, 0) == 0.0f);
  CHECK(armMoveTarget(Target::PERCENT, 37) == 37.0f);
  CHECK(armMoveTarget(Target::PERCENT, 100) == 100.0f);
  CHECK(ARM_TARGET_BUFFER == 200 && ARM_TARGET_OUT == 0 && ARM_TARGET_IN == 100);

  // arm position as percent between Out and In (either calibration direction)
  CHECK(fabsf(armPercentOf(500.0f, 0, 1000) - 50.0f) < 1e-4f);
  CHECK(fabsf(armPercentOf(0.0f, 0, 1000) - 0.0f) < 1e-4f);
  CHECK(fabsf(armPercentOf(1000.0f, 0, 1000) - 100.0f) < 1e-4f);
  CHECK(fabsf(armPercentOf(750.0f, 1000, 500) - 50.0f) < 1e-4f);  // In < Out
  CHECK(armPercentOf(10.0f, 5, 5) == 0.0f);                       // no divide by zero

  // actuator jog clamp
  CHECK(actuatorJogTarget(50.0f, 5) == 55);
  CHECK(actuatorJogTarget(50.4f, -5) == 45);   // rounds the current percent first
  CHECK(actuatorJogTarget(50.6f, 0) == 51);
  CHECK(actuatorJogTarget(3.0f, -20) == 0);
  CHECK(actuatorJogTarget(0.0f, -1) == 0);
  CHECK(actuatorJogTarget(98.0f, 20) == 100);
  CHECK(actuatorJogTarget(100.0f, 1) == 100);

  // ready for auto: all 3 z + act limits and arm Out/In (-1 = unset)
  const bool all[3] = {true, true, true};
  const bool miss[3] = {true, false, true};
  CHECK(readyForAuto(all, all, 10, 20));
  CHECK(readyForAuto(all, all, 0, 0));        // 0 is a set position
  CHECK(!readyForAuto(miss, all, 10, 20));
  CHECK(!readyForAuto(all, miss, 10, 20));
  CHECK(!readyForAuto(all, all, -1, 20));
  CHECK(!readyForAuto(all, all, 10, -1));

  // request expiry: dropped once now has REACHED the deadline (boundary tick included)
  CHECK(!requestExpired(99, 100));
  CHECK(requestExpired(100, 100));   // boundary tick: must not execute
  CHECK(requestExpired(101, 100));
  CHECK(!requestExpired(0xFFFFFFF0u, 5));  // deadline wrapped past 2^32, still in the future
  CHECK(!requestExpired(4, 5));
  CHECK(requestExpired(5, 5));
  CHECK(requestExpired(6, 5));
  CHECK(requestExpired(3, 0xFFFFFFF0u));   // now wrapped past a deadline just before 2^32

  // deadline = start + timeout - margin, so the controller stops before the loop gives up
  CHECK(requestDeadline(1000, 250, 20) == 1230);
  CHECK(!requestExpired(1229, requestDeadline(1000, 250, 20)));
  CHECK(requestExpired(1230, requestDeadline(1000, 250, 20)));
  CHECK(requestDeadline(1000, 250, 20) < 1000 + 250);
  CHECK(requestDeadline(1000, 10, 20) == 1000);       // margin clamped to the timeout
  CHECK(requestDeadline(0xFFFFFFF0u, 250, 20) == 0xFFFFFFF0u + 230u);  // wraps
}

int main() {
  testSimpleVerbs();
  testCase();
  testRun();
  testMove();
  testJog();
  testMalformed();
  testAssembler();
  testPolicy();
  testFormat();
  testTruncation();
  testGlue();
  if (g_fail) {
    printf("%d of %d checks FAILED\n", g_fail, g_checks);
    return 1;
  }
  printf("all %d checks passed\n", g_checks);
  return 0;
}
