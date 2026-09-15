# UART Communication — Tera Term

Implementation of **UART serial communication on the STM32G431RBT6** using Embedded C and the STM32 HAL library.

The STM32 communicates with a PC through the NUCLEO-G431RB board's **ST-LINK Virtual COM Port**, with **Tera Term** used as the terminal application.

## Objective

The objective of this implementation was to establish bidirectional UART communication between the STM32 and a PC.

The implementation demonstrates:

- UART configuration
- Serial data transmission
- Serial data reception
- Character input
- Formatted output
- PC-to-STM32 communication
- STM32-to-PC communication

## Hardware

- NUCLEO-G431RB
- STM32G431RBT6
- USB connection through ST-LINK

## Software

- STM32CubeMX
- STM32CubeIDE
- Embedded C
- STM32 HAL
- Tera Term

## UART Configuration

USART2 was configured for asynchronous communication with the following settings:

| Parameter | Configuration |
|---|---|
| Peripheral | USART2 |
| Baud Rate | 115200 |
| Word Length | 8 bits |
| Stop Bits | 1 |
| Parity | None |
| Mode | TX / RX |
| Hardware Flow Control | None |

The USART2 interface uses:

```text
PA2 → TX
PA3 → RX
```
Communication Flow
```
        STM32G431RBT6
              │
              │ USART2
              ↓
     ST-LINK Virtual COM Port
              │
              │ USB
              ↓
          PC / Tera Term
```
The STM32 transmits messages through UART and receives characters entered through Tera Term.

UART Functions

The UART interface was integrated with the application using callback functions for character transmission and reception.

Character Transmission
```
void STM_putchar(uint8_t c)
{
    HAL_UART_Transmit(&huart2, &c, 1, 1000);
}
```
Character Reception
```
char STM_getchar()
{
    uint8_t ch;

    HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);

    return ch;
}
```
Formatted output was implemented using a custom my_printf() function.

Tera Term Configuration

Tera Term was configured with:
```
Port: COM8
Baud Rate: 115200
Data: 8 bit
Parity: None
Stop Bits: 1
Flow Control: None
```
Test Output

The following communication was observed during testing:
```
This is UART program
Please enter a character
Hello
```
The STM32 successfully received user input from Tera Term and transmitted output back to the terminal.
