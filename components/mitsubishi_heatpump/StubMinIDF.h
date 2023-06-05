#ifndef STUBMINIDF_H_
#define STUBMINIDF_H_

#include <stdint.h>

#include "esp_timer.h"

#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/soc_caps.h"
#include "soc/uart_struct.h"
#include "soc/uart_periph.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#define ARDUINO_ISR_FLAG ESP_INTR_FLAG_IRAM
#else
#define ARDUINO_ISR_ATTR
#define ARDUINO_ISR_FLAG (0)
#endif

BaseType_t xTaskCreateUniversal( TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint32_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask,
                        const BaseType_t xCoreID );

unsigned long millis();
void delay(uint32_t);

/*
void vPortYield(void);
void yield(void);
#define optimistic_yield(u)
*/

uint32_t getApbFrequency();     // In Hz

#ifdef __cplusplus
enum SerialConfig {
SERIAL_5N1 = 0x8000010,
SERIAL_6N1 = 0x8000014,
SERIAL_7N1 = 0x8000018,
SERIAL_8N1 = 0x800001c,
SERIAL_5N2 = 0x8000030,
SERIAL_6N2 = 0x8000034,
SERIAL_7N2 = 0x8000038,
SERIAL_8N2 = 0x800003c,
SERIAL_5E1 = 0x8000012,
SERIAL_6E1 = 0x8000016,
SERIAL_7E1 = 0x800001a,
SERIAL_8E1 = 0x800001e,
SERIAL_5E2 = 0x8000032,
SERIAL_6E2 = 0x8000036,
SERIAL_7E2 = 0x800003a,
SERIAL_8E2 = 0x800003e,
SERIAL_5O1 = 0x8000013,
SERIAL_6O1 = 0x8000017,
SERIAL_7O1 = 0x800001b,
SERIAL_8O1 = 0x800001f,
SERIAL_5O2 = 0x8000033,
SERIAL_6O2 = 0x8000037,
SERIAL_7O2 = 0x800003b,
SERIAL_8O2 = 0x800003f
};
#else
#define SERIAL_5N1 0x8000010
#define SERIAL_6N1 0x8000014
#define SERIAL_7N1 0x8000018
#define SERIAL_8N1 0x800001c
#define SERIAL_5N2 0x8000030
#define SERIAL_6N2 0x8000034
#define SERIAL_7N2 0x8000038
#define SERIAL_8N2 0x800003c
#define SERIAL_5E1 0x8000012
#define SERIAL_6E1 0x8000016
#define SERIAL_7E1 0x800001a
#define SERIAL_8E1 0x800001e
#define SERIAL_5E2 0x8000032
#define SERIAL_6E2 0x8000036
#define SERIAL_7E2 0x800003a
#define SERIAL_8E2 0x800003e
#define SERIAL_5O1 0x8000013
#define SERIAL_6O1 0x8000017
#define SERIAL_7O1 0x800001b
#define SERIAL_8O1 0x800001f
#define SERIAL_5O2 0x8000033
#define SERIAL_6O2 0x8000037
#define SERIAL_7O2 0x800003b
#define SERIAL_8O2 0x800003f
#endif // __cplusplus

// These are Hardware Flow Contol possible usage
// equivalent to UDF enum uart_hw_flowcontrol_t from 
// https://github.com/espressif/esp-idf/blob/master/components/hal/include/hal/uart_types.h#L75-L81
#define HW_FLOWCTRL_DISABLE   0x0       // disable HW Flow Control
#define HW_FLOWCTRL_RTS       0x1       // use only RTS PIN for HW Flow Control
#define HW_FLOWCTRL_CTS       0x2       // use only CTS PIN for HW Flow Control
#define HW_FLOWCTRL_CTS_RTS   0x3       // use both CTS and RTS PIN for HW Flow Control

// These are Hardware Uart Modes possible usage
// equivalent to UDF enum uart_mode_t from
// https://github.com/espressif/esp-idf/blob/master/components/hal/include/hal/uart_types.h#L34-L40
#define MODE_UART 0x00                   // mode: regular UART mode
#define MODE_RS485_HALF_DUPLEX 0x01      // mode: half duplex RS485 UART mode control by RTS pin
#define MODE_IRDA 0x02                   // mode: IRDA  UART mode
#define MODE_RS485_COLLISION_DETECT 0x03 // mode: RS485 collision detection UART mode (used for test purposes)
#define MODE_RS485_APP_CTRL 0x04

struct uart_struct_t;
typedef struct uart_struct_t uart_t;

bool uartIsDriverInstalled(uart_t* uart);
uart_t* uartBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, uint16_t rx_buffer_size, uint16_t tx_buffer_size, bool inverted, uint8_t rxfifo_full_thrhd);
void uartEnd(uart_t* uart);

void uartStartDetectBaudrate(uart_t *uart);
unsigned long uartDetectBaudrate(uart_t *uart);

bool uartSetRxTimeout(uart_t* uart, uint8_t numSymbTimeout);
bool uartSetRxFIFOFull(uart_t* uart, uint8_t numBytesFIFOFull);

bool uartSetPins(uart_t* uart, int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin);
void uartDetachPins(uart_t* uart, int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin);

void uartSetDebug(uart_t* uart);
int uartGetDebug();

uint32_t uartAvailable(uart_t* uart);
size_t uartReadBytes(uart_t* uart, uint8_t *buffer, size_t size, uint32_t timeout_ms);

void uartWrite(uart_t* uart, uint8_t c);
void uartWriteBuf(uart_t* uart, const uint8_t * data, size_t len);

bool uartSetHwFlowCtrlMode(uart_t *uart, uint8_t mode, uint8_t threshold);
bool uartSetMode(uart_t *uart, uint8_t mode);

void uartFlush(uart_t* uart);
void uartFlushTxOnly(uart_t* uart, bool txOnly );

#ifdef __cplusplus
}
#endif

#endif
