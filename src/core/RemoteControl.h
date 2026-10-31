#pragma once
// Glue between the pure command protocol (core/CommandProtocol.h, docs/PROTOCOL.md)
// and the firmware: TCP port 6667 + USB serial front ends, and the controller-side
// executor. Two halves, in two different tasks:
//
//  - loop() task (priority 1): init(), handle(), feedUsb(). Parses lines, answers
//    PING/STATUS/SEQS/STOP itself, forwards RUN/MOVE/JOG to the controller task.
//  - controller_task (priority 3): drainPending(). Runs policy against live state and
//    executes through startSequence() and the node methods, exactly like the encoder.

#include <stddef.h>
#include <stdint.h>

namespace RemoteControl {

// Creates the request/reply queues and starts the 6667 server. Call once, before the
// controller task is created, after WiFi is connected.
void init();

// Services the TCP client and answers/forwards any complete lines. Call from loop().
void handle();

// Feeds raw bytes read from the USB serial port (replies go back on USB).
void feedUsb(const uint8_t *data, size_t len);

// Executes queued RUN/MOVE/JOG requests. Call from controller_task only, after
// InputManager::process(), so remote and encoder starts never race.
void drainPending();

}  // namespace RemoteControl
