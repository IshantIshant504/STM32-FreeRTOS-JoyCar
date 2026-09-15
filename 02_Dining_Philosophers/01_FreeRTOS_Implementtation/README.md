# FreeRTOS Implementation

Implementation and study of **FreeRTOS on the STM32G431RBT6** using the CMSIS-RTOS2 API.

This section demonstrates the fundamentals of real-time task management, task scheduling, delays, priorities and inter-task communication that were later applied to the Dining Philosophers and JoyCar projects.

## Platform

- STM32G431RBT6
- STM32CubeMX
- STM32CubeIDE
- FreeRTOS
- CMSIS-RTOS2
- Embedded C

## RTOS Concepts

The implementation covers the following FreeRTOS concepts:

- Task creation
- Task priorities
- Task scheduling
- Task delays
- Task states
- Context switching
- Mutexes
- Message queues
- Event flags
- Inter-task communication

## Task Management

The application is divided into independent tasks.

Each task has its own:

- Task function
- Stack
- Priority
- Execution context

The FreeRTOS scheduler determines which ready task is executed based on task priority and scheduling behaviour.

### Task States

A task can move between different states during execution:

```text
                    ┌─────────┐
                    │  Ready  │
                    └────┬────┘
                         │
                         ↓
                    ┌─────────┐
                    │ Running │
                    └────┬────┘
                         │
              ┌──────────┴──────────┐
              ↓                     ↓
          Blocked                 Ready
       (Delay/Event)          (Preempted)
```
## Task Priorities

Task priorities determine the scheduling order when multiple tasks are ready to execute.

A higher-priority ready task can preempt a lower-priority task when preemptive scheduling is used.

## Task Delays

Task delays are used to periodically block a task and allow other ready tasks to execute.
Example:
```
osDelay(1000);
```
The task enters the blocked state for the specified delay period and becomes ready again afterwards.

## Inter-Task Communication

The implementation uses RTOS mechanisms to allow tasks to communicate and synchronise safely.

### Message Queues

Message queues are used to transfer data between tasks without directly sharing application data.
```
Task A
  │
  │  Message
  ↓
Queue
  │
  ↓
Task B
```
## Mutexes

Mutexes provide controlled access to shared resources.
```
Task A ──┐
         ├──→ Mutex ──→ Shared Resource
Task B ──┘
```
Only the task holding the mutex can access the protected resource.

## Event Flags

Event flags allow tasks to signal and wait for specific events.

They are useful when task execution depends on one or more system conditions.

## Scheduling and Synchronisation

The practical implementation demonstrates how multiple tasks can execute concurrently while sharing system resources.

The Dining Philosophers application builds on these concepts by using multiple FreeRTOS tasks and mutexes for resource synchronisation.

## Tracealyzer Analysis

FreeRTOS execution was analysed using Percepio Tracealyzer.

The trace analysis was used to observe:

Task execution
Task switching
Blocking behaviour
Semaphore/mutex operations
CPU load
Task timing

Detailed results are documented in:
```
../03_Tracealyzer_Analysis/
```
## Application of FreeRTOS

The FreeRTOS concepts developed in this section were subsequently applied to the JoyCar embedded system.

The JoyCar uses multiple tasks for:

- Line sensor processing
- Obstacle detection
- Vehicle control
- Motor control
- Ultrasonic scanning

Communication between these tasks is handled using RTOS mechanisms such as message queues and event flags.

## Learning Outcome

This implementation provided practical experience with:

- Real-time task management
- Priority-based scheduling
- Task synchronisation
- Inter-task communication
- Resource protection
- RTOS debugging and trace analysis
