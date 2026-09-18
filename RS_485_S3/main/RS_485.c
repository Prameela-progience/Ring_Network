#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT      UART_NUM_1

#define UART_TX_PIN    GPIO_NUM_4
#define UART_RX_PIN    GPIO_NUM_5

#define UART2_PORT     UART_NUM_2

#define UART2_TX_PIN   GPIO_NUM_6
#define UART2_RX_PIN   GPIO_NUM_7

#define RS4851_DE_PIN  GPIO_NUM_8
#define RS4851_RE_PIN  GPIO_NUM_9

#define RS4852_DE_PIN  GPIO_NUM_10
#define RS4852_RE_PIN  GPIO_NUM_11

#define BUF_SIZE       256

#define BAUDRATE       115200

//--------------------------------------------
#define PKT_DATA           1
#define PKT_ACK            2
#define PKT_STATUS_REQ     3
#define PKT_STATUS_RESP    4

#define DEV_RPI            0
#define DEV_ESP32S3        1
#define DEV_STM32          2
#define DEV_ESP32C6        3

#define HEADER_SIZE        4
#define MAX_PAYLOAD_SIZE   64




// in app_main(), before creating tasks:
//static SemaphoreHandle_t uart2_mutex;



typedef struct
{
    uint8_t type;
    uint8_t src;
    uint8_t dest;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD_SIZE];

}packet_t;

static void rs4851_tx_mode(void)
{
    gpio_set_level(RS4851_DE_PIN,1);
    gpio_set_level(RS4851_RE_PIN,1);
}

static void rs4851_rx_mode(void)
{
    gpio_set_level(RS4851_DE_PIN,0);
    gpio_set_level(RS4851_RE_PIN,0);
}

static void rs4852_tx_mode(void)
{
    gpio_set_level(RS4852_DE_PIN,1);
    gpio_set_level(RS4852_RE_PIN,1);
    printf("RS4852 -> TX mode\n");   // ADD
}

static void rs4852_rx_mode(void)
{
    gpio_set_level(RS4852_DE_PIN,0);
    gpio_set_level(RS4852_RE_PIN,0);
    printf("RS4852 -> RX mode\n");   // ADD
}


static void rs485_gpio_init(void)
{
printf("gpio init started\n");
    gpio_reset_pin(RS4851_DE_PIN);
    gpio_reset_pin(RS4851_RE_PIN);

    gpio_reset_pin(RS4852_DE_PIN);
    gpio_reset_pin(RS4852_RE_PIN);

    gpio_set_direction(RS4851_DE_PIN,GPIO_MODE_OUTPUT);
    gpio_set_direction(RS4851_RE_PIN,GPIO_MODE_OUTPUT);

    gpio_set_direction(RS4852_DE_PIN,GPIO_MODE_OUTPUT);
    gpio_set_direction(RS4852_RE_PIN,GPIO_MODE_OUTPUT);

    rs4851_rx_mode();
    rs4852_rx_mode();
    printf("gpio init end\n");
}

static void uart_init_all(void)
{
    uart_config_t config =
    {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_PORT,BUF_SIZE,BUF_SIZE,0,NULL,0);

    uart_param_config(UART_PORT,&config);

    uart_set_pin(UART_PORT,
                 UART_TX_PIN,
                 UART_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
uart_flush(UART_PORT);
    uart_driver_install(UART2_PORT,BUF_SIZE,BUF_SIZE,0,NULL,0);

    uart_param_config(UART2_PORT,&config);

    uart_set_pin(UART2_PORT,
                 UART2_TX_PIN,
                 UART2_RX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_flush(UART2_PORT);
}



static int read_exact(uart_port_t uart_num,
                      uint8_t *buffer,
                      int length,
                      int timeout_ms)
{
    int received = 0;

    while(received < length)
    {
        int n = uart_read_bytes(uart_num,
                                buffer + received,
                                length - received,
                                pdMS_TO_TICKS(timeout_ms));
//printf("uart_read_bytes() returned %d\n", n);
        if(n <= 0)
        {
            return -1;
        }

printf("RX : ");

for(int i = 0; i < n; i++)
    printf("%02X ", buffer[received + i]);

printf("\n");  		
   		
        received += n;
    }

    return received;
}

//-------------------------------------
static int receive_packet(uart_port_t uart_num, packet_t *pkt)
{
    uint8_t byte;

    while (1)
    {
        memset(pkt, 0, sizeof(packet_t));


        while (1)
        {
            if (read_exact(uart_num, &byte, 1, 20) != 1)
            {
                return -1;
            }

            if (byte == PKT_DATA ||
                byte == PKT_ACK ||
                byte == PKT_STATUS_REQ ||
                byte == PKT_STATUS_RESP)
            {
                pkt->type = byte;
                printf("TYPE=%d\n", pkt->type);
                break;
            }
        }

        // Read header 
        if (read_exact(uart_num, &pkt->src, 1, 20) != 1)
            return -1;

        if (read_exact(uart_num, &pkt->dest, 1, 20) != 1)
            return -1;

        if (read_exact(uart_num, &pkt->length, 1, 20) != 1)
            return -1;

 //       printf("\nHEADER: T=%d S=%d D=%d L=%d\n",
//               pkt->type,
 //              pkt->src,
//               pkt->dest,
//               pkt->length);
//
        // Validate header 
        if (pkt->src > DEV_ESP32C6)
        {
            printf("Invalid SRC\n");
            continue;
        }

        if (pkt->dest > DEV_ESP32C6)
        {
            printf("Invalid DEST\n");
            continue;
        }

        if (pkt->length > MAX_PAYLOAD_SIZE)
        {
            printf("Invalid LEN\n");
            continue;
        }

        // Read payload 
        if (pkt->length)
        {
            if (read_exact(uart_num,
                           pkt->payload,
                           pkt->length,
                           20) != pkt->length)
            {
                printf("Payload timeout\n");
                return -1;
            }
        }

        return 0;
    }
}

/*
static int receive_packet(uart_port_t uart_num, packet_t *pkt)
{
    uint8_t byte;

    printf("\n========== receive_packet() ==========\n");

    while (1)
    {
        memset(pkt, 0, sizeof(packet_t));

        printf("Searching for packet type...\n");

        while (1)
        {
            int ret = read_exact(uart_num, &byte, 1, 100);

            printf("read_exact(TYPE) ret=%d", ret);

            if(ret == 1)
                printf(" byte=%02X\n", byte);
            else
                printf("\n");

            if(ret != 1)
            {
                printf("TYPE timeout\n");
                return -1;
            }

            if(byte == PKT_DATA ||
               byte == PKT_ACK ||
               byte == PKT_STATUS_REQ ||
               byte == PKT_STATUS_RESP)
            {
                pkt->type = byte;

                printf("VALID TYPE=%d\n", pkt->type);

                break;
            }

            printf("Ignoring byte %02X\n", byte);
        }

        int ret;

        ret = read_exact(uart_num, &pkt->src, 1, 100);
        printf("SRC ret=%d value=%02X\n", ret, pkt->src);
        if(ret != 1)
            return -1;

        ret = read_exact(uart_num, &pkt->dest, 1, 100);
        printf("DEST ret=%d value=%02X\n", ret, pkt->dest);
        if(ret != 1)
            return -1;

        ret = read_exact(uart_num, &pkt->length, 1, 100);
        printf("LEN ret=%d value=%02X\n", ret, pkt->length);
        if(ret != 1)
            return -1;

        if(pkt->length > MAX_PAYLOAD_SIZE)
        {
            printf("Invalid Length\n");
            continue;
        }

        if(pkt->length)
        {
            ret = read_exact(uart_num,
                             pkt->payload,
                             pkt->length,
                             100);

            printf("PAYLOAD ret=%d\n", ret);

            if(ret != pkt->length)
            {
                printf("Payload timeout\n");
                return -1;
            }

            printf("Payload : ");

            for(int i=0;i<pkt->length;i++)
                printf("%02X ", pkt->payload[i]);

            printf("\n");
        }

        printf("Packet Complete\n");

        return 0;
    }
}
*/
//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
/*static void send_uart1(packet_t *pkt)
{
    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    tx[0]=pkt->type;
    tx[1]=pkt->src;
    tx[2]=pkt->dest;
    tx[3]=pkt->length;

    memcpy(&tx[4],pkt->payload,pkt->length);

    rs4851_tx_mode();

//printf("\n===== FORWARD TO RPI =====\n");

for(int i=0;i<HEADER_SIZE+pkt->length;i++)
{
    printf("%02X ", tx[i]);
}

printf("\n");

    uart_write_bytes(UART_PORT,
                     (char *)tx,
                     HEADER_SIZE + pkt->length);

    uart_wait_tx_done(UART_PORT,
                      pdMS_TO_TICKS(100));
//	vTaskDelay(pdMS_TO_TICKS(2));        

    rs4851_rx_mode();

}*/

static void send_uart1(packet_t *pkt)
{
    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    tx[0] = pkt->type;
    tx[1] = pkt->src;
    tx[2] = pkt->dest;
    tx[3] = pkt->length;

    memcpy(&tx[4], pkt->payload, pkt->length);

    printf("\n========== UART1 TX ==========\n");

    printf("TYPE : %d\n", pkt->type);
    printf("SRC  : %d\n", pkt->src);
    printf("DEST : %d\n", pkt->dest);
    printf("LEN  : %d\n", pkt->length);

    printf("RAW  : ");
    for(int i=0;i<HEADER_SIZE+pkt->length;i++)
        printf("%02X ", tx[i]);
    printf("\n");

    rs4851_tx_mode();

    int ret = uart_write_bytes(UART_PORT,
                               (char *)tx,
                               HEADER_SIZE + pkt->length);

    printf("uart_write_bytes() = %d\n", ret);

    esp_err_t err = uart_wait_tx_done(UART_PORT,
                                      pdMS_TO_TICKS(100));

    printf("uart_wait_tx_done() = %d\n", err);

    rs4851_rx_mode();

    printf("========== UART1 TX DONE ==========\n");
}
//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
static void send_uart2(packet_t *pkt)
{
printf("send_uart2::RS4852 -> TX\n");
    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    tx[0]=pkt->type;
    tx[1]=pkt->src;
    tx[2]=pkt->dest;
    tx[3]=pkt->length;

    memcpy(&tx[4],pkt->payload,pkt->length);

    rs4852_tx_mode();

printf("\n===== UART2 TX RAW =====\n");

for (int i = 0; i < HEADER_SIZE + pkt->length; i++)
{
    printf("%02X ", tx[i]);
}
printf("\n");
for (int i = 0; i < HEADER_SIZE + pkt->length; i++)
{
    printf("%c ", tx[i]);
}

printf("\n");
    uart_write_bytes(UART2_PORT,
                     (char *)tx,
                     HEADER_SIZE + pkt->length);
    uart_wait_tx_done(UART2_PORT,
                      pdMS_TO_TICKS(100));
                      vTaskDelay(pdMS_TO_TICKS(2));
                      printf("send_uart2::TX COMPLETE\n");
                      printf("send_uart2::RS4852 -> RX\n");
//vTaskDelay(pdMS_TO_TICKS(2));
    rs4852_rx_mode();

}


static void process_rpi_packet(void)
{
    packet_t pkt;
    packet_t reply;


    if(receive_packet(UART_PORT, &pkt) < 0)
    {
        return;
    }

    printf("\n[STATUS/DATA] Packet received from Raspberry Pi\n");

    switch(pkt.dest)
    {
        case DEV_ESP32S3:

            printf("Packet is for ESP32\n");

            switch(pkt.type)
            {
                case PKT_STATUS_REQ:

                    memset(&reply,0,sizeof(reply));

                    reply.type   = PKT_STATUS_RESP;
                    reply.src    = DEV_ESP32S3;
                    reply.dest   = DEV_RPI;
                    reply.length = 1;
                    reply.payload[0] = 1;

                    send_uart1(&reply);

                    printf("[STATUS] Request is for ESP32\n");
		    printf("[STATUS] Sending ONLINE response to Raspberry Pi\n");

                    break;

                case PKT_DATA:

                    printf("[DATA] Packet is for ESP32\n");
		    printf("[DATA] Message : ");

/*                    printf("SRC  : %d\n", pkt.src);
                    printf("DEST : %d\n", pkt.dest);
                    printf("LEN  : %d\n", pkt.length);

                    printf("DATA : ");
*/
                    for(int i = 0; i < pkt.length; i++)
                    {
                        printf("%c", pkt.payload[i]);
                    }

                    printf("\n");

                    break;

                default:

                    printf("Unknown Packet Type\n");

                    break;
            }

            break;

        case DEV_STM32:

	    switch(pkt.type)
	    {
		case PKT_STATUS_REQ:

		    printf("[STATUS] Request is not for ESP32\n");
		    printf("[STATUS] Forwarding request to STM32\n");
		    break;

		case PKT_DATA:

		     printf("[DATA] Packet is not for ESP32\n");
	             printf("[DATA] Forwarding packet to STM32\n");
		    break;

		default:

		    break;
	    }

	    send_uart2(&pkt);
//	    vTaskDelay(pdMS_TO_TICKS(5));
	    break;
    }
}


//---------------------------------------------------------

static void process_stm_packet(void)
{
    packet_t pkt;
    
    size_t len = 0;

//	uart_get_buffered_data_len(UART2_PORT, &len);

//	printf("UART2 buffered = %d\n", len);


     int ret = receive_packet(UART2_PORT, &pkt);
//     printf("ret=%d\n",ret);
    if (ret < 0)
    {
//    printf(" ststus not received , receive packet return value = %d \n",ret);
        return;
    }
  
   
    printf("\n[STATUS/DATA] Packet received from STM32\n");

//    printf("TYPE : %d\n", pkt.type);
//    printf("SRC  : %d\n", pkt.src);
//    printf("DEST : %d\n", pkt.dest);
//    printf("LEN  : %d\n", pkt.length);

    if(pkt.length)
    {
        printf("DATA : ");

        for(int i = 0; i < pkt.length; i++)
        {
            printf("%02X ", pkt.payload[i]);
        }

        printf("\n");
    }

    if(pkt.dest == DEV_RPI)
    {
    if(pkt.type == PKT_STATUS_RESP)
    {
        printf("[STATUS] ONLINE response received from STM32\n");
    }
	printf("[STATUS] Forwarding response to Raspberry Pi\n");

        send_uart1(&pkt);
 //       vTaskDelay(pdMS_TO_TICKS(5));
    }
    else
	{
	    printf("Packet not for Raspberry Pi\n");
	}
}


/*static void process_stm_packet(void)
{
    size_t len = 0;

    uart_get_buffered_data_len(UART2_PORT, &len);

    printf("UART2 buffered = %u\n", (unsigned)len);

    uint8_t rx[64];

    int n = uart_read_bytes(UART2_PORT,
                            rx,
                            sizeof(rx),
                            pdMS_TO_TICKS(500));
    printf("uart_read_bytes() = %d\n", n);
    if (n > 0)
    {
        printf("\nSTM RAW (%d): ", n);

        for (int i = 0; i < n; i++)
            printf("%02X ", rx[i]);

        printf("\n");
    }
    else
    {
        printf("No data from STM\n");
    }
}*/
//------------------------------------------------------------------------------
static void send_status_req_to_stm(void)
{
    packet_t pkt;

    memset(&pkt, 0, sizeof(pkt));

    pkt.type   = PKT_STATUS_REQ;
    pkt.src    = DEV_ESP32S3;
    pkt.dest   = DEV_STM32;
    pkt.length = 0;

    printf("\nSending STATUS_REQ to STM32\n");

    send_uart2(&pkt);
}

static void rpi_task(void *arg)
{
    while (1)
    {
//   printf("RPI TASK\n");
        process_rpi_packet();
//        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void stm_task(void *arg)
{
    while (1)
    {
//       printf("STM TASK\n");
        process_stm_packet();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
/*
static void stm_task(void *arg)
{
    uint8_t buf[64];

    while (1)
    {
        int n = uart_read_bytes(UART2_PORT,
                                buf,
                                sizeof(buf),
                                pdMS_TO_TICKS(1000));

        printf("uart_read_bytes_stmtask() = %d\n", n);

        if (n > 0)
        {
            printf("RAW data from stm: : ");

            for (int i = 0; i < n; i++)
                printf("%02X ", buf[i]);

            printf("\n");
        }
    }
}
*/

void app_main(void)
{
    printf("\n");
    printf("=====================================\n");
    printf(" ESP32 Gateway Started\n");
    printf("=====================================\n");

    rs485_gpio_init();

    uart_init_all();



static TickType_t last_status = 0;

//while (1)
//{
//    process_stm_packet();
//    process_rpi_packet();
//    process_stm_packet();
//    vTaskDelay(pdMS_TO_TICKS(10));
//}

xTaskCreate(rpi_task,
            "rpi_task",
            6144,
            NULL,
            5,
            NULL);

xTaskCreate(stm_task,
            "stm_task",
            6144,
            NULL,
            5,
            NULL);

while (1)
{
    vTaskDelay(portMAX_DELAY);
}
}


/*
void app_main(void)
{
    uart_init_all();
    rs485_gpio_init();

    rs4852_rx_mode();

    uint8_t rx[64];

    uint8_t buf[64];

while (1)
{
    int len = uart_read_bytes(UART2_PORT,
                              buf,
                              sizeof(buf),
                              pdMS_TO_TICKS(1000));
        printf("uart_read_bytes() = %d\n", len);
    if (len > 0)
    {
        printf("Received %d bytes: ", len);

        for (int i = 0; i < len; i++)
            printf("%02X ", buf[i]);

        printf("\n");
    }
}
}
*/










