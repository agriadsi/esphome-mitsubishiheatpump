#include "StubMinIDF.h"

#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/soc_caps.h"
#include "soc/uart_struct.h"
#include "soc/uart_periph.h"

#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/apb_ctrl_reg.h"

#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_rom_gpio.h"

static int s_uart_debug_nr = 0;

unsigned long ARDUINO_ISR_ATTR millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/*
void __yield()
{
    vPortYield();
}

void yield() __attribute__ ((weak, alias("__yield")));
*/

static uint32_t calculateApb(rtc_cpu_freq_config_t * conf){
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
	return APB_CLK_FREQ;
#else
    if(conf->freq_mhz >= 80){
        return 80 * MHZ;
    }
    return (conf->source_freq_mhz * MHZ) / conf->div;
#endif
}

uint32_t getApbFrequency(){
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);
    return calculateApb(&conf);
}

struct uart_struct_t {

#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif

    uint8_t num;
    bool has_peek;
    uint8_t peek_byte;
    QueueHandle_t uart_event_queue;   // export it by some uartGetEventQueue() function
};

BaseType_t xTaskCreateUniversal( TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint32_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask,
                        const BaseType_t xCoreID ){
#ifndef CONFIG_FREERTOS_UNICORE
    if(xCoreID >= 0 && xCoreID < 2) {
        return xTaskCreatePinnedToCore(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID);
    } else {
#endif
    return xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask);
#ifndef CONFIG_FREERTOS_UNICORE
    }
#endif
}

#if CONFIG_DISABLE_HAL_LOCKS

#define UART_MUTEX_LOCK()
#define UART_MUTEX_UNLOCK()

static uart_t _uart_bus_array[] = {
    {0, false, 0, NULL},
#if SOC_UART_NUM > 1
    {1, false, 0, NULL},
#endif
#if SOC_UART_NUM > 2
    {2, false, 0, NULL},
#endif
};

#else

#define UART_MUTEX_LOCK()    do {} while (xSemaphoreTake(uart->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK()  xSemaphoreGive(uart->lock)

static uart_t _uart_bus_array[] = {
    {NULL, 0, false, 0, NULL},
#if SOC_UART_NUM > 1
    {NULL, 1, false, 0, NULL},
#endif
#if SOC_UART_NUM > 2
    {NULL, 2, false, 0, NULL},
#endif
};

#endif

uint32_t _get_effective_baudrate(uint32_t baudrate) 
{
    uint32_t Freq = getApbFrequency()/1000000;
    if (Freq < 80) {
        return 80 / Freq * baudrate;
     }
    else {
        return baudrate;
    }
}

unsigned long uartBaudrateDetect(uart_t *uart, bool flg)
{
// Baud rate detection only works for ESP32 and ESP32S2
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    if(uart == NULL) {
        return 0;
    }

    uart_dev_t *hw = UART_LL_GET_HW(uart->num);

    while(hw->rxd_cnt.edge_cnt < 30) { // UART_PULSE_NUM(uart_num)
        if(flg) return 0;
        ets_delay_us(1000);
    }

    UART_MUTEX_LOCK();
    //log_i("lowpulse_min_cnt = %d hightpulse_min_cnt = %d", hw->lowpulse.min_cnt, hw->highpulse.min_cnt);
    unsigned long ret = ((hw->lowpulse.min_cnt + hw->highpulse.min_cnt) >> 1);
    UART_MUTEX_UNLOCK();

    return ret;
#else
    return 0;
#endif
}

static void ARDUINO_ISR_ATTR uart0_write_char(char c)
{
    while (uart_ll_get_txfifo_len(&UART0) == 0);
    uart_ll_write_txfifo(&UART0, (const uint8_t *) &c, 1);
}

#if SOC_UART_NUM > 1
static void ARDUINO_ISR_ATTR uart1_write_char(char c)
{
    while (uart_ll_get_txfifo_len(&UART1) == 0);
    uart_ll_write_txfifo(&UART1, (const uint8_t *) &c, 1);
}
#endif

#if SOC_UART_NUM > 2
static void ARDUINO_ISR_ATTR uart2_write_char(char c)
{
    while (uart_ll_get_txfifo_len(&UART2) == 0);
    uart_ll_write_txfifo(&UART2, (const uint8_t *) &c, 1);
}
#endif

void uart_install_putc()
{
    switch(s_uart_debug_nr) {
    case 0:
        ets_install_putc1((void (*)(char)) &uart0_write_char);
        break;
#if SOC_UART_NUM > 1
    case 1:
        ets_install_putc1((void (*)(char)) &uart1_write_char);
        break;
#endif
#if SOC_UART_NUM > 2
    case 2:
        ets_install_putc1((void (*)(char)) &uart2_write_char);
        break;
#endif
    default:
        ets_install_putc1(NULL);
        break;
    }
}

bool uartIsDriverInstalled(uart_t* uart) 
{
    if(uart == NULL) {
        return false;
    }

    if (uart_is_driver_installed(uart->num)) {
        return true;
    }
    return false;
}

uart_t* uartBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, uint16_t rx_buffer_size, uint16_t tx_buffer_size, bool inverted, uint8_t rxfifo_full_thrhd)
{
    if(uart_nr >= SOC_UART_NUM) {
        return NULL;
    }

    uart_t* uart = &_uart_bus_array[uart_nr];

    if (uart_is_driver_installed(uart_nr)) {
        uartEnd(uart);
    }

#if !CONFIG_DISABLE_HAL_LOCKS
    if(uart->lock == NULL) {
        uart->lock = xSemaphoreCreateMutex();
        if(uart->lock == NULL) {
            return NULL;
        }
    }
#endif

    UART_MUTEX_LOCK();

    uart_config_t uart_config;
    uart_config.data_bits = (uart_word_length_t)((config & 0xc) >> 2);
    uart_config.parity = (uart_parity_t)(config & 0x3);
    uart_config.stop_bits = (uart_stop_bits_t)((config & 0x30) >> 4);
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = rxfifo_full_thrhd;
#if SOC_UART_SUPPORT_XTAL_CLK
    // works independently of APB frequency
    uart_config.source_clk = UART_SCLK_XTAL; // ESP32C3, ESP32S3
    uart_config.baud_rate = baudrate;
#else
    uart_config.source_clk = UART_SCLK_APB;  // ESP32, ESP32S2
    uart_config.baud_rate = _get_effective_baudrate(baudrate);
#endif
    ESP_ERROR_CHECK(uart_driver_install(uart_nr, rx_buffer_size, tx_buffer_size, 20, &(uart->uart_event_queue), 0));
    ESP_ERROR_CHECK(uart_param_config(uart_nr, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_nr, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Is it right or the idea is to swap rx and tx pins? 
    if (inverted) {
        // invert signal for both Rx and Tx
        ESP_ERROR_CHECK(uart_set_line_inverse(uart_nr, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));    
    }
    
    UART_MUTEX_UNLOCK();

    uartFlush(uart);
    return uart;
}

void uartEnd(uart_t* uart)
{
    if(uart == NULL) {
        return;
    }

    UART_MUTEX_LOCK();
    uart_driver_delete(uart->num);
    UART_MUTEX_UNLOCK();
}

void uartStartDetectBaudrate(uart_t *uart) {
    if(uart == NULL) {
        return;
    }

#ifdef CONFIG_IDF_TARGET_ESP32C3
    
    // ESP32-C3 requires further testing
    // Baud rate detection returns wrong values 

    log_e("ESP32-C3 baud rate detection is not supported.");
    return;

    // Code bellow for C3 kept for future recall
    //hw->rx_filt.glitch_filt = 0x08;
    //hw->rx_filt.glitch_filt_en = 1;
    //hw->conf0.autobaud_en = 0;
    //hw->conf0.autobaud_en = 1;
#elif CONFIG_IDF_TARGET_ESP32S3
    log_e("ESP32-S3 baud rate detection is not supported.");
    return;
#else
    uart_dev_t *hw = UART_LL_GET_HW(uart->num);
    hw->auto_baud.glitch_filt = 0x08;
    hw->auto_baud.en = 0;
    hw->auto_baud.en = 1;
#endif
}

unsigned long uartDetectBaudrate(uart_t *uart)
{
    if(uart == NULL) {
        return 0;
    }

// Baud rate detection only works for ESP32 and ESP32S2
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2

    static bool uartStateDetectingBaudrate = false;

    if(!uartStateDetectingBaudrate) {
        uartStartDetectBaudrate(uart);
        uartStateDetectingBaudrate = true;
    }

    unsigned long divisor = uartBaudrateDetect(uart, true);
    if (!divisor) {
        return 0;
    }

    uart_dev_t *hw = UART_LL_GET_HW(uart->num);
    hw->auto_baud.en = 0;

    uartStateDetectingBaudrate = false; // Initialize for the next round

    unsigned long baudrate = getApbFrequency() / divisor;
    
    //log_i("APB_FREQ = %d\nraw baudrate detected = %d", getApbFrequency(), baudrate);

    static const unsigned long default_rates[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 256000, 460800, 921600, 1843200, 3686400};

    size_t i;
    for (i = 1; i < sizeof(default_rates) / sizeof(default_rates[0]) - 1; i++)	// find the nearest real baudrate
    {
        if (baudrate <= default_rates[i])
        {
            if (baudrate - default_rates[i - 1] < default_rates[i] - baudrate) {
                i--;
            }
            break;
        }
    }

    return default_rates[i];
#else
#ifdef CONFIG_IDF_TARGET_ESP32C3 
    log_e("ESP32-C3 baud rate detection is not supported.");
#else
    log_e("ESP32-S3 baud rate detection is not supported.");
#endif
    return 0;
#endif
}

void uartDetachPins(uart_t* uart, int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin)
{
    if(uart == NULL) {
        return;
    }

    UART_MUTEX_LOCK();
    if (txPin >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[txPin], PIN_FUNC_GPIO);
        esp_rom_gpio_connect_out_signal(txPin, SIG_GPIO_OUT_IDX, false, false);
    }

    if (rxPin >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[rxPin], PIN_FUNC_GPIO);
        esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_LOW, UART_PERIPH_SIGNAL(uart->num, SOC_UART_RX_PIN_IDX), false);
    }

    if (rtsPin >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[rtsPin], PIN_FUNC_GPIO);
        esp_rom_gpio_connect_out_signal(rtsPin, SIG_GPIO_OUT_IDX, false, false);
    }

    if (ctsPin >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[ctsPin], PIN_FUNC_GPIO);
        esp_rom_gpio_connect_in_signal(GPIO_FUNC_IN_LOW, UART_PERIPH_SIGNAL(uart->num, SOC_UART_CTS_PIN_IDX), false);
    }
    UART_MUTEX_UNLOCK();  
}

bool uartSetRxTimeout(uart_t* uart, uint8_t numSymbTimeout)
{
    if(uart == NULL) {
        return false;
    }

    UART_MUTEX_LOCK();
    bool retCode = (ESP_OK == uart_set_rx_timeout(uart->num, numSymbTimeout));
    UART_MUTEX_UNLOCK();
    return retCode;
}

bool uartSetRxFIFOFull(uart_t* uart, uint8_t numBytesFIFOFull)
{
    if(uart == NULL) {
        return false;
    }

    UART_MUTEX_LOCK();
    bool retCode = (ESP_OK == uart_set_rx_full_threshold(uart->num, numBytesFIFOFull));
    UART_MUTEX_UNLOCK();
    return retCode;
}

void uartSetDebug(uart_t* uart)
{
    if(uart == NULL || uart->num >= SOC_UART_NUM) {
        s_uart_debug_nr = -1;
    } else {
        s_uart_debug_nr = uart->num;
    }
    uart_install_putc();
}

int uartGetDebug()
{
    return s_uart_debug_nr;
}

uint32_t uartAvailable(uart_t* uart)
{

    if(uart == NULL) {
        return 0;
    }

    UART_MUTEX_LOCK();
    size_t available;
    uart_get_buffered_data_len(uart->num, &available);
    if (uart->has_peek) available++;
    UART_MUTEX_UNLOCK();
    return available;
}

size_t uartReadBytes(uart_t* uart, uint8_t *buffer, size_t size, uint32_t timeout_ms)
{
    if(uart == NULL || size == 0 || buffer == NULL) {
        return 0;
    }

    size_t bytes_read = 0;

    UART_MUTEX_LOCK();

    if (uart->has_peek) {
        uart->has_peek = false;
        *buffer++ = uart->peek_byte;
        size--;
        bytes_read = 1;
    }

    if (size > 0) {
        int len = uart_read_bytes(uart->num, buffer, size, pdMS_TO_TICKS(timeout_ms));
        if (len < 0) len = 0;  // error reading UART
        bytes_read += len;
    }

        
    UART_MUTEX_UNLOCK();
    return bytes_read;
}

void uartWrite(uart_t* uart, uint8_t c)
{
    if(uart == NULL) {
        return;
    }
    UART_MUTEX_LOCK();
    uart_write_bytes(uart->num, &c, 1);
    UART_MUTEX_UNLOCK();
}

void uartWriteBuf(uart_t* uart, const uint8_t * data, size_t len)
{
    if(uart == NULL || data == NULL || !len) {
        return;
    }

    UART_MUTEX_LOCK();
    uart_write_bytes(uart->num, data, len);
    UART_MUTEX_UNLOCK();
}

bool uartSetPins(uart_t* uart, int8_t rxPin, int8_t txPin, int8_t ctsPin, int8_t rtsPin)
{
    if(uart == NULL) {
        return false;
    }
    UART_MUTEX_LOCK();
    // IDF uart_set_pin() will issue necessary Error Message and take care of all GPIO Number validation.
    bool retCode = uart_set_pin(uart->num, txPin, rxPin, rtsPin, ctsPin) == ESP_OK; 
    UART_MUTEX_UNLOCK();  
    return retCode;
}

// 
bool uartSetHwFlowCtrlMode(uart_t *uart, uint8_t mode, uint8_t threshold) {
    if(uart == NULL) {
        return false;
    }
    // IDF will issue corresponding error message when mode or threshold are wrong and prevent crashing
    // IDF will check (mode > HW_FLOWCTRL_CTS_RTS || threshold >= SOC_UART_FIFO_LEN)
    UART_MUTEX_LOCK();
    bool retCode = (ESP_OK == uart_set_hw_flow_ctrl(uart->num, (uart_hw_flowcontrol_t) mode, threshold));
    UART_MUTEX_UNLOCK();  
    return retCode;
}

bool uartSetMode(uart_t *uart, uint8_t mode)
{
    if (uart == NULL || uart->num >= SOC_UART_NUM)
    {
        return false;
    }
    
    UART_MUTEX_LOCK();
    bool retCode = (ESP_OK == uart_set_mode(uart->num, (uart_mode_t)mode));
    UART_MUTEX_UNLOCK();
    return retCode;
}

void uartFlush(uart_t* uart)
{
    uartFlushTxOnly(uart, true);
}

void uartFlushTxOnly(uart_t* uart, bool txOnly)
{
    if(uart == NULL) {
        return;
    }
    
    UART_MUTEX_LOCK();
    while(!uart_ll_is_tx_idle(UART_LL_GET_HW(uart->num)));

    if ( !txOnly ) {
        ESP_ERROR_CHECK(uart_flush_input(uart->num));
    }
    UART_MUTEX_UNLOCK();
}
