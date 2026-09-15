# UART Communication with Tera Term

STM32 UART communication project using the **STM32G431** and **Tera Term**.

## Objective

Configure UART communication between the STM32 development board and a PC and verify bidirectional serial communication.

## Hardware

- NUCLEO-G431RB
- STM32G431RBT6
- ST-LINK Virtual COM Port

## UART Configuration

| Parameter | Configuration |
|---|---|
| UART | USART2 |
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Data Bits | 8 |
| Stop Bits | 1 |
| Parity | None |
| Flow Control | None |
| TX | PA2 |
| RX | PA3 |

## Software

- STM32CubeMX
- STM32CubeIDE
- Tera Term
- C

## Implementation

The UART interface was used for transmitting and receiving characters between the STM32 and the PC.

The project implements:

- UART character transmission
- UART character reception
- `printf`-style output
- User input through the terminal
- Serial communication testing

## Terminal Test

The communication was tested using Tera Term through the ST-LINK Virtual COM Port.

Example output:

```text
This is UART program
Please enter a character
Hello
