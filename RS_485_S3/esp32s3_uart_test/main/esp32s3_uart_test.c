#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT       UART_NUM_2

#define UART2_TX_PIN    GPIO_NUM_6
#define UART2_RX_PIN    GPIO_NUM_7

#define RS485_DE_PIN    GPIO_NUM_8
#define RS485_RE_PIN    GPIO_NUM_9

#define BAUD_RATE       115200
#define BUF_SIZE        256

static void rs485_tx_mode(void)
{
    gpio_set_level(RS485_DE_PIN, 1);
    gpio_set_level(RS485_RE_PIN, 1);
}

static void rs485_rx_mode(void)
{
    gpio_set_level(RS485_DE_PIN, 0);
    gpio_set_level(RS485_RE_PIN, 0);
}


static void rs485_gpio_init(void)
{
    gpio_reset_pin(RS485_DE_PIN);
    gpio_reset_pin(RS485_RE_PIN);

    gpio_set_direction(RS485_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS485_RE_PIN, GPIO_MODE_OUTPUT);

    rs485_rx_mode();
}


static void uart2_init(void)
{
    uart_config_t uart_config =
    {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT,
                        BUF_SIZE,
                        0,
                        0,
                        NULL,
                        0);

    uart_param_config(UART_PORT, &uart_config);

    uart_set_pin(UART_PORT,
                 UART2_TX_PIN,
                 UART2_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    uart_flush(UART_PORT);
}

void app_main(void)
{
    uart2_init();
    rs485_gpio_init();
    printf("\n");
    printf("=============================\n");
    printf("ESP32-S3 UART2 TX TEST\n");
    printf("=============================\n");

    const char *msg = "HELLO STM32\r\n";

  uint8_t tx[] = {0x01,0x02,0x03,0x04,0x05};

while (1)
{
	rs485_tx_mode();
	    printf("RS485 -> TX MODE\n");
    printf("TX : ");

    for(int i = 0; i < sizeof(tx); i++)
        printf("%02X ", tx[i]);

    printf("\n");

    int ret = uart_write_bytes(UART_PORT,
                               (const char *)tx,
                               sizeof(tx));

    printf("uart_write_bytes() returned %d\n", ret);

    uart_wait_tx_done(UART_PORT,
                      pdMS_TO_TICKS(100));
vTaskDelay(pdMS_TO_TICKS(2));   // Add this
rs485_rx_mode();
    printf("RS485 -> RX MODE\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
}
}
