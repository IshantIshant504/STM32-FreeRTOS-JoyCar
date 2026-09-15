# Dining Philosophers — FreeRTOS

Implementation of the classic **Dining Philosophers problem** using **FreeRTOS on the STM32G431RBT6**.

The project demonstrates real-time task scheduling, mutex-based resource protection, task synchronisation and blocking behaviour.

## Objective

The objective is to implement five concurrent philosopher tasks that share five resources (forks) while preventing unsafe simultaneous access to the same resource.

The implementation demonstrates how an RTOS can coordinate multiple tasks competing for shared resources.

## Platform

- STM32G431RBT6
- FreeRTOS
- CMSIS-RTOS2
- STM32CubeMX
- STM32CubeIDE
- Embedded C

## System Model

Five philosophers are represented as FreeRTOS tasks.

```text
                    Philosopher 01
                         │
                    Fork 01 / 05
                         │
       Philosopher 05 ───┼─── Philosopher 02
             │           │           │
          Fork 04      Table      Fork 02
             │                       │
       Philosopher 04 ───────── Philosopher 03
                    Fork 03
```
Each philosopher requires two forks before entering the eating state.

## FreeRTOS Tasks

Five philosopher tasks are created:
```
Philosopher01
Philosopher02
Philosopher03
Philosopher04
Philosopher05
```
Each philosopher repeatedly performs the following sequence:
```
Thinking
   ↓
Request first fork
   ↓
Request second fork
   ↓
Eating
   ↓
Release forks
   ↓
Thinking
```
## Mutex-Based Synchronisation

Each fork is represented by a FreeRTOS mutex.

A philosopher must acquire the required mutexes before accessing the corresponding resources.

Conceptually:
```
Philosopher
     │
     ├── Take Fork A
     │
     ├── Take Fork B
     │
     ↓
   Eating
     │
     ├── Give Fork B
     │
     └── Give Fork A
```
The mutex prevents multiple philosophers from owning the same fork simultaneously.

## Blocking Behaviour

If a required fork is already owned by another philosopher, the requesting task becomes blocked until the resource becomes available.

For example:
```
Philosopher 05
      │
      │ xSemaphoreTake(Fork01, -1)
      ↓
   BLOCKED
      │
      │ Fork01 released
      ↓
    READY
      │
      ↓
   RUNNING
```
The -1 timeout corresponds to waiting indefinitely for the mutex.

## Task Scheduling

The philosophers execute as independent FreeRTOS tasks.

The scheduler determines which ready task receives CPU time according to the configured task priorities and scheduling behaviour.

Tasks can enter the blocked state while waiting for a fork, allowing other ready tasks to execute.

## Resource Management

The five shared resources are represented by mutexes:
```
Fork01
Fork02
Fork03
Fork04
Fork05
```
The mutexes provide exclusive ownership of each fork.

This makes the project useful for demonstrating:

- Mutual exclusion
- Task blocking
- Resource contention
- Task scheduling
- Synchronisation
## RTOS API Concepts

The implementation uses CMSIS-RTOS2 interfaces for task and synchronisation management.

Important operations include:
```
osThreadNew()
osDelay()
osMutexAcquire()
osMutexRelease()
```
These APIs are used to create tasks, introduce task delays and control access to shared resources.

## Tracealyzer Analysis

The FreeRTOS execution was analysed using Percepio Tracealyzer.

The analysis focuses on:

- Task execution
- Task scheduling
- Mutex acquisition
- Blocking behaviour
- Mutex release
- CPU load
- Task timing

Detailed trace results are documented in:
```
../03_Tracealyzer_Analysis/
```

## Key Concepts Demonstrated
- FreeRTOS task creation
- Task scheduling
- Task priorities
- Task delays
- Mutexes
- Mutual exclusion
- Resource contention
- Blocking and unblocking
- Inter-task synchronisation
- Real-time execution analysis
## Project Outcome

The Dining Philosophers implementation provides a practical demonstration of how FreeRTOS manages multiple concurrent tasks competing for shared resources.
