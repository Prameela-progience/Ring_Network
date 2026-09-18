#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_err.h"

#define UART_PORT      UART_NUM_1

/* Change these pins if required */
#define UART_TX_PIN    GPIO_NUM_4
#define UART_RX_PIN    GPIO_NUM_5

#define UART2_PORT      UART_NUM_2
#define UART2_TX_PIN    GPIO_NUM_6
#define UART2_RX_PIN    GPIO_NUM_7

/* RS485 #1 : RPi <-> ESP32-S3 */
#define RS4851_DE_PIN   GPIO_NUM_8
#define RS4851_RE_PIN   GPIO_NUM_9

/* RS485 #2 : ESP32-S3 <-> ESP32-C6 */
#define RS4852_DE_PIN   GPIO_NUM_10
#define RS4852_RE_PIN   GPIO_NUM_11
#define BUF_SIZE       128

#define PKT_DATA    1
#define PKT_ACK     2
#define PKT_STATUS_REQ    3
#define PKT_STATUS_RESP   4

#define DEV_RPI        0x00
#define DEV_ESP32S3    0x01
#define DEV_STM32      0x02

#define IDX_TYPE       0
#define IDX_SRC        1
#define IDX_DEST       2
#define IDX_LENGTH     3
#define IDX_PAYLOAD    4

void forward_to_c6(uint8_t *data, int len);
/*--------------------------------------------------------------------------*/
static void rs485_rx_mode(gpio_num_t de_pin, gpio_num_t re_pin)
{
    gpio_set_level(de_pin, 0);
    gpio_set_level(re_pin, 0);
}

static void rs485_tx_mode(gpio_num_t de_pin, gpio_num_t re_pin)
{
    gpio_set_level(de_pin, 1);
    gpio_set_level(re_pin, 1);
}

/*--------------------------------------------------------------------------*/
void forward_to_rpi(uint8_t *data, int len)
{
    /* Switch RS485 #1 (RPi side) to TX mode */
    rs485_tx_mode(RS4851_DE_PIN, RS4851_RE_PIN);

    int ret = uart_write_bytes(UART_PORT,
                               (const char *)data,
                               len);
printf("Forwarding ACK to Raspberry Pi...\n");
/*printf("ACK Bytes : ");

for(int i = 0; i < len; i++)
{
    printf("%02X ", data[i]);
}

printf("\n");*/
    if (ret > 0)
    {
        printf("ACK forwarded to Raspberry Pi\n");
        
    }
    else
    {
        printf("ACK forwarding failed\n");
    }

    uart_wait_tx_done(UART_PORT,
                      pdMS_TO_TICKS(100));
	vTaskDelay(pdMS_TO_TICKS(1));
    rs485_rx_mode(RS4851_DE_PIN,
                  RS4851_RE_PIN);
}

/*--------------------------------------------------------------------------*/

void send_status_response_to_rpi(void)
{
    uint8_t tx[5];

    tx[IDX_TYPE]   = PKT_STATUS_RESP;
    tx[IDX_SRC]    = DEV_ESP32S3;
    tx[IDX_DEST]   = DEV_RPI;
    tx[IDX_LENGTH] = 1;
    tx[IDX_PAYLOAD]= 1;

    forward_to_rpi(tx, 5);
}

/*--------------------------------------------------------------------------*/
void handle_status_request(uint8_t *data, int len)
{
    printf("\nSTATUS REQUEST RECEIVED\n");
    printf("Pointer = %p\n", data);
printf("Length  = %d\n", len);

for(int i=0;i<len;i++)
{
    printf("data[%d] = %02X\n", i, data[i]);
}

printf("Type        = %d\n", data[IDX_TYPE]);
printf("Source      = %d\n", data[IDX_SRC]);
printf("Destination = %d\n", data[IDX_DEST]);
printf("Length      = %d\n", data[IDX_LENGTH]);

printf("=====================================\n");

    printf("Source  Dev    : %d\n", data[IDX_SRC]);
    printf("Destination Dev : %d\n", data[IDX_DEST]);

    if(data[IDX_DEST] == DEV_ESP32S3)
    {
        printf("STATUS request for ESP32\n");

        send_status_response_to_rpi();
    }
    else
    {
        printf("Forward STATUS request to STM32\n");

        forward_to_c6(data, len);
    }
}

/*--------------------------------------------------------------------------*/
void handle_status_response(uint8_t *data, int len)
{
    printf("\nSTATUS RESPONSE FROM STM32\n");
    printf("Source      : %d\n", data[IDX_SRC]);
    printf("Destination : %d\n", data[IDX_DEST]);
    printf("Status      : %d\n", data[IDX_PAYLOAD]);
    forward_to_rpi(data, len);
}
/*--------------------------------------------------------------------------*/

void app_main(void)
{
    esp_err_t ret;

    printf("\n");
    printf("=====================================\n");
    printf(" ESP32 RS485 Receiver Started\n");
    printf("=====================================\n");

  /****************************************************/
    /*        RS485 DE/RE GPIO Initialization           */
    /****************************************************/
    gpio_config_t io_conf =
    {
        .pin_bit_mask =
        (1ULL << RS4851_DE_PIN) |
        (1ULL << RS4851_RE_PIN) |
        (1ULL << RS4852_DE_PIN) |
        (1ULL << RS4852_RE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

/*                 UART1 INITIALIZATION                      */

    gpio_config(&io_conf);
        /*               UART1 Initialization               */
    uart_config_t uart_config =
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    /* Install UART driver */
    ret = uart_driver_install(UART_PORT,
                              BUF_SIZE * 2,
                              0,
                              0,
                              NULL,
                              0);

    printf("uart_driver_install() : %s\n",
            esp_err_to_name(ret));

    /* Configure UART */
    ret = uart_param_config(UART_PORT,
                            &uart_config);

    printf("uart_param_config()   : %s\n",
            esp_err_to_name(ret));

    /* Assign UART pins */
    ret = uart_set_pin(UART_PORT,
                       UART_TX_PIN,
                       UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    printf("UART2 TX = GPIO%d\n", UART2_TX_PIN);
    printf("UART2 RX = GPIO%d\n", UART2_RX_PIN);
    printf("uart_set_pin()        : %s\n",
            esp_err_to_name(ret));

/*                 UART2 INITIALIZATION                      */


uart_config_t uart2_config =
{
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT
};

ret = uart_driver_install(UART2_PORT,
                          BUF_SIZE * 2,
                          0,
                          0,
                          NULL,
                          0);

printf("uart2_driver_install(): %s\n",
       esp_err_to_name(ret));

ret = uart_param_config(UART2_PORT,
                        &uart2_config);

printf("uart2_param_config() : %s\n",
       esp_err_to_name(ret));

ret = uart_set_pin(UART2_PORT,
                   UART2_TX_PIN,
                   UART2_RX_PIN,
                   UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE);

printf("uart2_set_pin()      : %s\n",
       esp_err_to_name(ret));
       
       /* Default to Receive Mode */
   rs485_rx_mode(RS4851_DE_PIN, RS4851_RE_PIN);
   rs485_rx_mode(RS4852_DE_PIN, RS4852_RE_PIN);

/*************************************************************/

    printf("-------------------------------------\n");
    printf("Waiting for RS485 data...\n");
    printf("-------------------------------------\n");

    uint8_t uart1_data[BUF_SIZE];
	uint8_t uart2_data[BUF_SIZE];

    while (1)
{
    int len;

    /**************** UART1 : RPi -> ESP32 ****************/
    memset(uart1_data, 0, sizeof(uart1_data));

   len = uart_read_bytes(UART_PORT,
                      uart1_data,
                      BUF_SIZE - 1,
                      pdMS_TO_TICKS(100));

    if (len > 0)
    {
    printf("\n----------------------------------------------\n");
        printf("\nUART1 Received %d bytes\n", len);
        
       printf("Length = %d\n", len);
    printf("Buffer Address = %p\n", uart1_data);

    for (int i = 0; i < len; i++)
    {
        printf("uart1_data[%d] = 0x%02X\n", i, uart1_data[i]);
    }

    printf("==========================================\n");
      if (uart1_data[0] == PKT_STATUS_REQ)
	{
  	  handle_status_request(uart1_data, len);
	}
	else if (uart1_data[0] == PKT_DATA)
        {
            printf("Source      : %d\n", uart1_data[1]);
printf("Destination : %d\n", uart1_data[2]);
printf("Length      : %d\n", uart1_data[3]);
printf("Payload     : %s\n", (char *)&uart1_data[4]);
            uart1_data[len] = '\0';

            forward_to_c6(uart1_data, len);
        }
        else if (uart1_data[0] == PKT_ACK)
	{
	    printf("Unexpected ACK on UART1\n");
	}
	else
	{
	    printf("Unknown Packet : 0x%02X\n", uart1_data[0]);
	}
	}

    /**************** UART2 : C6 -> ESP32 ****************/
    memset(uart2_data, 0, sizeof(uart2_data));
    len = uart_read_bytes(UART2_PORT,
                      uart2_data,
                      BUF_SIZE - 1,
                      pdMS_TO_TICKS(100));
   if (len > 0)
{
   printf("\n       UART2    \n");
    printf("Received %d bytes\n", len);
printf("\nUART2 RAW (%d bytes)\n", len);

for(int i=0;i<len;i++)
{
    printf("[%d] = %02X\n", i, uart2_data[i]);
}

   if(uart2_data[0] == PKT_STATUS_RESP)
{
    handle_status_response(uart2_data, len);
}
else if(uart2_data[0] == PKT_ACK)
    {
        printf("ACK RECEIVED FROM STM32 : %d \n",uart2_data[0]);

        forward_to_rpi(uart2_data, len);
    }
    else if(uart2_data[0] == PKT_DATA)
    {
        printf("DATA Packet from UART2\n");
    }
    else
    {
        printf("Unknown Packet : 0x%02X\n", uart2_data[0]);
    }
}
/*else 
{
	printf("read fails\n");
}
*/

    vTaskDelay(pdMS_TO_TICKS(1));
}
}

void forward_to_c6(uint8_t *data, int len)
{
printf("\n=========== forward_to_c6 ===========\n");

printf("Pointer = %p\n", data);
printf("Length  = %d\n", len);

for(int i=0;i<len;i++)
{
    printf("TX[%d] = %02X\n", i, data[i]);
}
//	uart_flush(UART2_PORT);
	rs485_tx_mode(RS4852_DE_PIN,RS4852_RE_PIN);
	printf("\nSending to STM32 (%d bytes): ", len);

	for (int i= 0; i < len; i++)
	{
   		 printf("%02X ", data[i]);
	}
	printf("\n");
//	uart_flush(UART2_PORT);
	vTaskDelay(pdMS_TO_TICKS(2));
        int ret = uart_write_bytes(UART2_PORT,
                     (const char *)data,
                     len);
	if (ret > 0)
	{
	    printf("Successfully transfered to stm of %d bytes \n", ret);
	    printf("\nAfter uart_write_bytes()\n");

for(int i=0;i<len;i++)
{
    printf("%02X ", data[i]);
}
printf("\n");
	}
	else
	{
	    printf("DATA Forward failed! ret = %d\n", ret);
	}
	
		esp_err_t err;
	err =  uart_wait_tx_done(UART2_PORT,
		              pdMS_TO_TICKS(100));
		vTaskDelay(pdMS_TO_TICKS(3));     
		printf("uart_wait_tx_done = %s\n",
       esp_err_to_name(err));                            
	    /* Return RS485 to Receive Mode */
	    rs485_rx_mode(RS4852_DE_PIN,RS4852_RE_PIN);
}

/*void forward_to_c6(uint8_t *data, int len)
{
    int ret = uart_write_bytes(UART2_PORT,
                               (const char *)data,
                               len);

    printf("uart_write_bytes() = %d\n", ret);

    uart_wait_tx_done(UART2_PORT, pdMS_TO_TICKS(100));

    uint8_t rx[128] = {0};

    int r = uart_read_bytes(UART2_PORT,
                            rx,
                            sizeof(rx) - 1,
                            pdMS_TO_TICKS(500));

    if (r > 0)
    {
        rx[r] = '\0';
        printf("Loopback received (%d): %s\n", r, rx);
    }
    else
    {
        printf("Loopback received = %d\n", r);
    }
}
*/
