# PEACH-PIT remote command protocol (v1)

Line-based ASCII request/reply for driving the PIT from a computer.

> **Status: the parser, policy and formatters are host-tested (`bash test/host/run_tests.sh`).
> The firmware wiring (port 6667, USB serial, controller-task execution, status record) is
> hardware-unverified: it has not been run on a real PIT yet. Test with the arm clear and a hand
> on the E-stop.**

## Transports

| Transport | Details |
| --- | --- |
| TCP **6667** | `nc peach-pit.local 6667`. Single client: a new connection replaces the old one (so a reconnect after a half-open link wins). |
| USB serial, 115200 | Same commands. Log lines are interleaved with replies; replies start with `@`, so clients ignore every line that does not. Anything typed by hand in the USB console now yields `@0 ERR BAD_CMD` (the same bytes still go to the 6666 bridge). |
| TCP 6666 | Unchanged: the PlatformIO log monitor. No commands, no replies. |

## Wire format

- Request: `#<id> <VERB> [args...]\n`. `id` is a decimal 1..999999 and optional (missing means 0).
  Verbs and arguments are case-insensitive. `\r` is ignored. Empty lines are ignored.
- A line holds at most **96 bytes**. A longer line is discarded up to its newline and answered
  once with `@0 ERR TOO_LONG`.
- Reply: exactly one line per request, either `@<id> OK [key=value ...]` or
  `@<id> ERR <CODE> [text]`. The id is echoed (0 when the request had none or a bad one).
- A malformed id (`#0`, `#abc`, `#9999999`) gets `@0 ERR BAD_ARG`.

## Commands

| Verb | Args | While busy | Reply |
| --- | --- | --- | --- |
| `PING` | none | allowed | `OK pong proto=1` |
| `STATUS` | none | allowed | `OK state=... ` (below) |
| `SEQS` | none | allowed | `OK seqs=auto,park` |
| `STOP` | none | allowed, always, preempts (see "STOP" below) | `OK` |
| `RUN` | `auto` or `park` | `ERR BUSY` | `OK run=<n>` |
| `MOVE` | `z bot\|mid\|top`, `act bot\|mid\|top`, `arm out\|buffer\|in\|<0-100>` | `ERR BUSY` | `OK` |
| `JOG` | `z <float>` (abs <= 1.0), `arm <int steps>` (abs <= 2000), `act <int pct>` (abs <= 20) | `ERR BUSY` | `OK` |

- `RUN auto` is the autonomous drop sequence; `RUN park` raises Z and clears the arm.
  `RUN auto` needs the same prerequisites as the encoder, else `ERR NOT_READY`.
- `MOVE z <limit>` and `MOVE act <limit>` need that limit set, else `ERR NOT_READY`.
  `MOVE arm out|in|<0-100>` needs the arm calibrated (Out and In set) and `MOVE arm buffer`
  also needs the Buffer point, else `ERR NOT_READY`. `JOG arm` needs the arm calibrated too (the
  arm node tracks a jog as a target); `JOG z` and `JOG act` need no calibration.
- `MOVE` and `JOG` are fire-and-forget: `OK` means accepted, watch `STATUS` for the motion.
- `MOVE z` (and every sequence) reports `result=DONE` when the axis **stopped**, not when the
  target was reached: an interlock can stop it early (for example Z stays at Mid while the arm is
  out). Compare `z` with the limit you asked for. The arm behaves the same way.
- Numbers must be plain decimals: `12abc`, `nan`, `inf`, hex are `BAD_ARG`. Out-of-range values
  are `BAD_ARG`. Missing or extra arguments are `BAD_ARG`. Unknown verb is `BAD_CMD`.
  `RUN` with another name is `UNKNOWN_SEQ`.
- Interlocks (Z blocked while the arm is out, collision zone) are enforced by the motion nodes
  and apply to remote moves exactly as to the encoder.

## STOP

`STOP` is handled immediately by the loop task, ahead of any queued request. It halts the Z motor
and the arm (SET_SPEED 0, which also cancels a goto or tracked target).

- **Nothing running:** the actuator is held at its current percent.
- **A run exists:** the run is aborted and STOP sends no actuator hold (it would race the run's own
  abort cleanup). What the actuator does then depends on the run: an `AUTO`/`PARK` sequence's abort
  cleanup retracts it to 0 %, exactly as after an encoder E-STOP; a goto (`MOVE z`) run leaves the
  actuator where it is. Once the run has ended the firmware halts Z and the arm once more, so an
  arm target that landed after STOP does not carry on. That post-run halt does not touch the
  actuator.

No commands are served during an OTA transfer: it runs synchronously in `loop()`, so neither
USB nor 6667 is read until it finishes (and the device then reboots). OTA halts the nodes when it
starts and aborts a running sequence. A client therefore cannot observe `ERR OTA` in practice; the
code exists only as a defensive policy result.

## Error codes

| Code | Meaning |
| --- | --- |
| `BAD_CMD` | unknown verb |
| `BAD_ARG` | missing, extra, malformed or out-of-range argument or id |
| `TOO_LONG` | line over 96 bytes |
| `BUSY` | a sequence or goto is running |
| `NOT_READY` | limits or calibration missing |
| `UNKNOWN_SEQ` | `RUN` name not in `SEQS` |
| `TIMEOUT` | the controller did not answer within 250 ms; the command may or may not have run. After `RUN` or `MOVE z`, check `STATUS` `run` (and `state`) to see whether it started. **Never blindly retry `JOG`**: it is relative and has no run id to check, so a retry can move twice. |
| `OTA` | firmware update in progress, all motion rejected (defensive; not observable in practice, no command is read during a transfer) |

Checks are applied in this order for RUN/MOVE/JOG: `OTA`, `BUSY`, then `NOT_READY`.

## STATUS keys

Fixed order, on one line:

`state=IDLE|RUNNING run=<n> seq=auto|park|goto|none step=<i> steps=<n> result=NONE|DONE|ABORTED
estop=0|1 ready=0|1 z=<f.3> zhomed=0|1 zmoving=0|1 arm=<f.0> armmoving=0|1 act=<f.1>
collision=0|1 ota=0|1`

`collision` is reserved and always `0` in v1 (no firmware source sets it; the real interlocks live in
the motion nodes). Keep the key for wire compatibility, but do not gate on it.

`run` increments each time a sequence or goto starts, whether from the encoder or a client.
`result` is the outcome of run `<n>` and stays `NONE` while it runs. Encoder-started runs update
the same record.

## Lost link

The PIT never aborts a run because a client disconnected (aborting mid-dispense with the arm out
is worse than finishing). After reconnecting, send `STATUS` and match `run` with the id from
your `RUN` reply to learn whether it finished (`result=DONE|ABORTED`) or is still going. Use
`STOP` to abort.

## Examples

```
$ nc peach-pit.local 6667
#1 PING
@1 OK pong proto=1
#2 STATUS
@2 OK state=IDLE run=3 seq=auto step=17 steps=17 result=DONE estop=0 ready=1 z=0.000 zhomed=1 zmoving=0 arm=0 armmoving=0 act=0.0 collision=0 ota=0
#3 RUN auto
@3 OK run=4
#4 RUN park
@4 ERR BUSY
#5 STOP
@5 OK
#6 JOG z 5
@6 ERR BAD_ARG
#7 MOVE arm 30
@7 OK
```

## Flashing

Build with `platform = espressif32@6.9.0` (arduino-esp32 2.x). The unpinned latest platform
(8.x, arduino-esp32 3.x) does not compile `main`: `ledcSetup` was removed and the `NetworkManager`
class clashes with a core name. This predates the remote commands; `platformio.ini` is unchanged.

## Implementation

`src/core/CommandProtocol.{h,cpp}` is the pure parser/policy/formatter (no Arduino or FreeRTOS);
tests are in `test/host/` (`bash test/host/run_tests.sh`).
