# Tracealyzer Analysis

Analysis of **FreeRTOS runtime behaviour using Percepio Tracealyzer** on the STM32G431RBT6.

Tracealyzer was used to visualise task execution, scheduling, blocking behaviour, mutex operations and CPU load during the Dining Philosophers implementation.

## Objective

The objective of the analysis was to observe the actual runtime behaviour of the FreeRTOS application and understand how multiple tasks interact with shared resources.

The analysis focused on:

- Task execution
- Task scheduling
- Context switching
- Mutex blocking
- Mutex acquisition and release
- CPU load
- Task timing

## Tools

- STM32G431RBT6
- FreeRTOS
- CMSIS-RTOS2
- Percepio Tracealyzer
- STM32CubeIDE
- Embedded C

## Trace Overview

The Trace View provides an overview of the execution of the five philosopher tasks.

```text
Philosopher01 ────────────────
Philosopher02 ────────░░─────
Philosopher03 ────────────────
Philosopher04 ─────░──────────
Philosopher05 ────────────────
                 Time →
```
The trace makes task execution and state changes visible over time.

Mutex Blocking Analysis

A philosopher can become blocked when attempting to acquire a fork that is already owned by another philosopher.

Example observed behaviour:
```
xSemaphoreTake(Fork01, -1)
          ↓
       BLOCKED
          ↓
   Fork becomes available
          ↓
       RETURNS
```
The trace can therefore be used to determine when a task requested a mutex, became blocked and subsequently acquired the resource.

## CPU Load

The CPU Load view provides an overview of processor utilisation during the recorded execution interval.

The philosophers frequently enter delayed or blocked states while waiting for resources, so the processor is not continuously executing philosopher code.

## Task Timing

Tracealyzer can be used to inspect timing information for an individual task.

For example, selecting a philosopher task provides information such as:

- Execution time
- Response time
- CPU usage
- Priority

These measurements help analyse the runtime behaviour of individual FreeRTOS tasks.

## Observed RTOS Behaviour

The trace analysis demonstrates:

- Multiple philosopher tasks executing under FreeRTOS.
- Tasks entering blocked states while waiting for mutexes.
- Mutex ownership changing between tasks.
- Tasks returning to the ready state after blocking.
- The scheduler switching execution between ready tasks.
- CPU utilisation varying according to task execution and blocking behaviour.
## Key FreeRTOS Concepts Demonstrated
- Task scheduling
- Task states
- Context switching
- Mutex synchronisation
- Blocking and unblocking
- Resource contention
- Task timing
- Runtime analysis

## Conclusion

Tracealyzer provided a visual representation of the FreeRTOS runtime behaviour and made task scheduling, mutex blocking and task timing easier to analyse.

The analysis complements the Dining Philosophers implementation by showing the actual runtime behaviour of the RTOS rather than relying only on source-code inspection.
