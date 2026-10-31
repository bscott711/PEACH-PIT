#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include "core/CommandProtocol.h"

#define SEQ_TARGET_BUFFER 200

// Step counts of the built-in sequences (checked against the tables in SequenceEngine.cpp).
#define SEQ_STEPS_AUTO 17
#define SEQ_STEPS_PARK 2

void autonomous_task(void *pvParameters);
void motor_goto_task(void *pvParameters);

// Record of the most recent sequence/goto run. Written by startSequence and by the
// two tasks, read by the remote STATUS command. Encoder-started runs update it too.
struct SequenceStatus {
  uint32_t run;            // id of the latest run (0 = nothing ran since boot)
  cmdproto::SeqName seq;   // AUTO, PARK or GOTO (NONE before the first run)
  int step;                // 1-based step in progress (or last step reached); steps when DONE
  int steps;               // total steps of that run
  cmdproto::Result result; // NONE while running, then DONE or ABORTED
};

// Consistent copy of the record (short critical section; safe from any task).
SequenceStatus sequenceStatus();

// Starts a sequence the way the encoder menu does: AUTO (param ignored), PARK (param
// ignored) or GOTO (param = Z limit index 0=Bot, 1=Mid, 2=Top). Writes the new run id
// to *runOut when non-null. Returns false (and leaves the record ABORTED when a task
// could not be created) if the start failed or a sequence is already running.
//
// MUST be called only from controller_task: the encoder handler and the remote
// command drain both run there, so starts never race each other.
bool startSequence(cmdproto::SeqName seq, int param, uint32_t *runOut);
