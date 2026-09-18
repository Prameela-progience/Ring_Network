/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_DE_PORT    GPIOD
#define RS485_DE_PIN     GPIO_PIN_0

#define RS485_RE_PORT    GPIOD
#define RS485_RE_PIN     GPIO_PIN_1

#define PKT_DATA    1
#define PKT_ACK     2
#define PKT_STATUS_REQ    3
#define PKT_STATUS_RESP   4

#define DEV_RPI        0x00
#define DEV_ESP32S3    0x01
#define DEV_STM32      0x02
#define DEV_ESP32C6    0x03

#define IDX_TYPE      0
#define IDX_SRC       1
#define IDX_DEST      2
#define IDX_LENGTH    3
#define IDX_PAYLOAD   4
#define HEADER_SIZE 4

#define RS4851_DE_PORT    GPIOG
#define RS4851_DE_PIN     GPIO_PIN_0

#define RS4851_RE_PORT    GPIOG
#define RS4851_RE_PIN     GPIO_PIN_1

/* Reverse RS485 - UART5 */
#define RS485_REV_DE_PORT    GPIOG
#define RS485_REV_DE_PIN     GPIO_PIN_0

#define RS485_REV_RE_PORT    GPIOG
#define RS485_REV_RE_PIN     GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ETH_HandleTypeDef heth;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t rx_data[128];
uint8_t rx_reverse[128];
volatile uint8_t rx_complete = 0;
volatile uint8_t rx_error = 0;
volatile uint32_t rx_error_code = 0;
volatile uint8_t tx_complete = 0;
volatile uint8_t tx_error = 0;

/* UART2 interrupt variables */
/* USART2 RS485 interrupt RX */
volatile uint8_t uart2_rx_byte;

volatile uint8_t uart2_packet[128];
volatile uint8_t uart2_packet_index = 0;
volatile uint8_t uart2_expected_length = 0;
volatile uint8_t uart2_packet_ready = 0;
volatile uint8_t uart2_receiving = 0;

/* USART2 TX */
volatile uint8_t uart2_tx_busy = 0;
volatile uint8_t uart2_tx_complete = 0;

uint8_t uart2_tx_buffer[128];

/* UART5 reverse routing RX */
volatile uint8_t uart5_rx_byte;
volatile uint8_t uart5_packet[128];
volatile uint8_t uart5_packet_index = 0;
volatile uint8_t uart5_expected_length = 0;
volatile uint8_t uart5_packet_ready = 0;
volatile uint8_t uart5_receiving = 0;

/* UART5 reverse routing TX */
volatile uint8_t uart5_tx_complete = 0;
volatile uint8_t uart5_tx_busy = 0;

uint8_t uart5_tx_buffer[128];

/* ======================================= */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void handle_status_request(uint8_t *rx_data);
void send_status_response(void);
uint8_t receive_packet(uint8_t *rx);
void handle_data_packet(uint8_t *rx_data);
void UART2_StartReceiveIT(void);

/* Reverse routing - UART5 */
void UART5_StartReceiveIT(void);

void RS485_REV_TX_Mode(void);
void RS485_REV_RX_Mode(void);

void handle_status_request_reverse(uint8_t *rx);
void send_status_response_reverse(void);
void handle_data_packet_reverse(uint8_t *rx);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}*/



static void UART3_Print(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), 200);
}

static void UART3_Printf(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 200);
}


//=========================



//========================
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
    	uart2_packet_ready = 1;
    }
}*/


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


    if (huart->Instance == USART2)
    {
        uint8_t byte = uart2_rx_byte;

        if (uart2_packet_ready == 0)
        {

            if (uart2_packet_index == 0)
            {
                uart2_packet[0] = byte;
                uart2_packet_index = 1;
                uart2_receiving = 1;
            }


            else if (uart2_packet_index < 4)
            {
                uart2_packet[uart2_packet_index++] = byte;


                if (uart2_packet_index == 4)
                {
                    uart2_expected_length =
                        uart2_packet[3];


                    if (uart2_expected_length == 0)
                    {
                        uart2_packet_ready = 1;
                        uart2_receiving = 0;
                    }
                }
            }


            else
            {
                uart2_packet[uart2_packet_index++] = byte;

                if (uart2_packet_index >=
                    (4 + uart2_expected_length))
                {
                    uart2_packet_ready = 1;
                    uart2_receiving = 0;
                }
            }
        }


        HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&uart2_rx_byte,
            1
        );
    }
    else if (huart->Instance == UART5)
    {
        uint8_t byte = uart5_rx_byte;

        if (uart5_packet_ready == 0)
        {
            if (uart5_packet_index == 0)
            {
                uart5_packet[0] = byte;
                uart5_packet_index = 1;
                uart5_receiving = 1;
            }

            else if (uart5_packet_index < 4)
            {
                uart5_packet[uart5_packet_index++] = byte;

                if (uart5_packet_index == 4)
                {
                    uart5_expected_length =
                        uart5_packet[IDX_LENGTH];

                    if (uart5_expected_length == 0)
                    {
                        uart5_packet_ready = 1;
                        uart5_receiving = 0;
                    }
                }
            }

            else
            {
                uart5_packet[uart5_packet_index++] = byte;

                if (uart5_packet_index >=
                    (HEADER_SIZE + uart5_expected_length))
                {
                    uart5_packet_ready = 1;
                    uart5_receiving = 0;
                }
            }
        }

        HAL_UART_Receive_IT(
            &huart5,
            (uint8_t *)&uart5_rx_byte,
            1
        );
    }

}


/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2)
    {
        uint8_t byte = uart2_rx_byte;

        if (uart2_packet_ready == 0)
        {
            if (uart2_packet_index == 0)
            {
                uart2_packet[0] = byte;
                uart2_packet_index = 1;
                uart2_receiving = 1;
            }

            else if (uart2_packet_index < HEADER_SIZE)
            {
                uart2_packet[uart2_packet_index++] = byte;

                if (uart2_packet_index == HEADER_SIZE)
                {
                    uart2_expected_length =
                        uart2_packet[IDX_LENGTH];

                    if (uart2_expected_length >
                        (sizeof(uart2_packet) - HEADER_SIZE))
                    {
                        uart2_packet_index = 0;
                        uart2_expected_length = 0;
                        uart2_receiving = 0;
                    }

                    else if (uart2_expected_length == 0)
                    {
                        uart2_packet_ready = 1;
                        uart2_receiving = 0;
                    }
                }
            }

            else
            {
                uart2_packet[uart2_packet_index++] = byte;

                if (uart2_packet_index >=
                    (HEADER_SIZE + uart2_expected_length))
                {
                    uart2_packet_ready = 1;
                    uart2_receiving = 0;
                }
            }
        }

        HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&uart2_rx_byte,
            1
        );
    }


    else if (huart->Instance == UART5)
    {
        uint8_t byte = uart5_rx_byte;

        if (uart5_packet_ready == 0)
        {
            if (uart5_packet_index == 0)
            {
                uart5_packet[0] = byte;
                uart5_packet_index = 1;
                uart5_receiving = 1;
            }

            else if (uart5_packet_index < HEADER_SIZE)
            {
                uart5_packet[uart5_packet_index++] = byte;

                if (uart5_packet_index == HEADER_SIZE)
                {
                    uart5_expected_length =
                        uart5_packet[IDX_LENGTH];

                    if (uart5_expected_length >
                        (sizeof(uart5_packet) - HEADER_SIZE))
                    {
                        uart5_packet_index = 0;
                        uart5_expected_length = 0;
                        uart5_receiving = 0;
                    }

                    else if (uart5_expected_length == 0)
                    {
                        uart5_packet_ready = 1;
                        uart5_receiving = 0;
                    }
                }
            }

            else
            {
                uart5_packet[uart5_packet_index++] = byte;

                if (uart5_packet_index >=
                    (HEADER_SIZE + uart5_expected_length))
                {
                    uart5_packet_ready = 1;
                    uart5_receiving = 0;
                }
            }
        }


        HAL_UART_Receive_IT(
            &huart5,
            (uint8_t *)&uart5_rx_byte,
            1
        );
    }
}
*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart2_tx_complete = 1;
        uart2_tx_busy = 0;
    }
    else if (huart->Instance == UART5)
    {
        uart5_tx_complete = 1;
        uart5_tx_busy = 0;
    }
}


void UART2_StartReceiveIT(void)
{
    uart2_rx_byte = 0;

    uart2_packet_index = 0;
    uart2_expected_length = 0;
    uart2_packet_ready = 0;
    uart2_receiving = 0;

    HAL_UART_Receive_IT(
        &huart2,
        (uint8_t *)&uart2_rx_byte,
        1
    );
}

//--------------------------------------------------------

void RS485_RX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT,
                      RS485_DE_PIN,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(RS485_RE_PORT,
                      RS485_RE_PIN,
                      GPIO_PIN_RESET);

 /*   UART3_Printf("RS485 RX MODE: DE=%d RE=%d\r\n",
            HAL_GPIO_ReadPin(RS485_DE_PORT, RS485_DE_PIN),
            HAL_GPIO_ReadPin(RS485_RE_PORT, RS485_RE_PIN));*/
}

void RS485_TX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT,
                      RS485_DE_PIN,
                      GPIO_PIN_SET);

    HAL_GPIO_WritePin(RS485_RE_PORT,
                      RS485_RE_PIN,
                      GPIO_PIN_SET);

/*    UART3_Printf("RS485 TX MODE: DE=%d RE=%d\r\n",
            HAL_GPIO_ReadPin(RS485_DE_PORT, RS485_DE_PIN),
            HAL_GPIO_ReadPin(RS485_RE_PORT, RS485_RE_PIN));*/
}

void RS485_REV_RX_Mode(void)
{
    HAL_GPIO_WritePin(
        RS485_REV_DE_PORT,
        RS485_REV_DE_PIN,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        RS485_REV_RE_PORT,
        RS485_REV_RE_PIN,
        GPIO_PIN_RESET
    );
}


void RS485_REV_TX_Mode(void)
{
    HAL_GPIO_WritePin(
        RS485_REV_DE_PORT,
        RS485_REV_DE_PIN,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        RS485_REV_RE_PORT,
        RS485_REV_RE_PIN,
        GPIO_PIN_SET
    );
}


//-----------------------------------------------------




void send_status_response(void)
{
    uint8_t tx_len = 5;
    uint32_t start;

    uart2_tx_buffer[IDX_TYPE]   = PKT_STATUS_RESP;
    uart2_tx_buffer[IDX_SRC]    = DEV_STM32;
    uart2_tx_buffer[IDX_DEST]   = DEV_RPI;
    uart2_tx_buffer[IDX_LENGTH] = 1;
    uart2_tx_buffer[IDX_PAYLOAD] = 1;

    UART3_Print("\r\n===== STATUS RESPONSE =====\r\n");

    UART3_Print("TX: ");

    for (uint8_t i = 0; i < tx_len; i++)
    {
        UART3_Printf(
            "%02X ",
            uart2_tx_buffer[i]
        );
    }

    UART3_Print("\r\n");

    /*
     * Enter RS485 transmit mode
     */
    RS485_TX_Mode();

    UART3_Printf(
        "RS485 TX: DE=%d RE=%d\r\n",
        HAL_GPIO_ReadPin(
            RS485_DE_PORT,
            RS485_DE_PIN
        ),
        HAL_GPIO_ReadPin(
            RS485_RE_PORT,
            RS485_RE_PIN
        )
    );

    uart2_tx_complete = 0;
    uart2_tx_busy = 1;

    /*
     * Start interrupt based transmission
     */
    HAL_StatusTypeDef ret =
        HAL_UART_Transmit_IT(
            &huart2,
            (uint8_t *)uart2_tx_buffer,
            tx_len
        );

    UART3_Printf(
        "HAL_UART_Transmit_IT = %d\r\n",
        ret
    );

    if (ret != HAL_OK)
    {
        UART3_Print("ERROR: UART2 TX IT failed\r\n");

        uart2_tx_busy = 0;

        RS485_RX_Mode();

        return;
    }

    /*
     * Wait for TX interrupt completion.
     *
     * HAL callback will set uart2_tx_complete.
     */
    start = HAL_GetTick();

    while (uart2_tx_complete == 0)
    {
        if ((HAL_GetTick() - start) > 1000)
        {
            UART3_Print(
                "ERROR: USART2 TX interrupt timeout\r\n"
            );

            uart2_tx_busy = 0;

            RS485_RX_Mode();

            return;
        }
    }

    /*
     * TX interrupt completed.
     *
     * Make absolutely sure the USART shift register
     * has finished transmitting the last stop bit.
     */
    while (__HAL_UART_GET_FLAG(
               &huart2,
               UART_FLAG_TC
           ) == RESET)
    {
    }

    UART3_Print(
        "USART2 TX COMPLETE\r\n"
    );

    UART3_Print(
        "USART2 TC = SET\r\n"
    );

    /*
     * Now it is safe to release RS485 bus.
     */
    RS485_RX_Mode();

    UART3_Printf(
        "RS485 RX: DE=%d RE=%d\r\n",
        HAL_GPIO_ReadPin(
            RS485_DE_PORT,
            RS485_DE_PIN
        ),
        HAL_GPIO_ReadPin(
            RS485_RE_PORT,
            RS485_RE_PIN
        )
    );

    UART3_Print(
        "STATUS RESPONSE SENT\r\n"
    );

    UART3_Print(
        "========== STM TX END ==========\r\n"
    );
}

void handle_status_request(uint8_t *rx_data)
{
	UART3_Printf("\n===== STATUS REQUEST =====\r\n");

 //   printf("SRC  : %d\r\n", rx_data[IDX_SRC]);
 //   printf("DEST : %d\r\n", rx_data[IDX_DEST]);

    UART3_Printf("TYPE : %d\r\n", rx_data[IDX_TYPE]);
        UART3_Printf("SRC  : %d\r\n", rx_data[IDX_SRC]);
        UART3_Printf("DEST : %d\r\n", rx_data[IDX_DEST]);
        UART3_Printf("LEN  : %d\r\n", rx_data[IDX_LENGTH]);
    if(rx_data[IDX_DEST] != DEV_STM32)
    {
        return;
    }

    send_status_response();

    /* Wait until TX is completely finished */

//    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);

}

void handle_data_packet(uint8_t *rx_data)
{
    if(rx_data[IDX_DEST] != DEV_STM32)
    {
        return;
    }

    UART3_Printf("\n===== DATA PACKET =====\r\n");

    UART3_Printf("TYPE : %d\r\n", rx_data[IDX_TYPE]);
    UART3_Printf("SRC  : %d\r\n", rx_data[IDX_SRC]);
    UART3_Printf("DEST : %d\r\n", rx_data[IDX_DEST]);
    UART3_Printf("LEN  : %d\r\n", rx_data[IDX_LENGTH]);

    UART3_Printf("PAYLOAD HEX : ");

    for(int i = 0; i < rx_data[IDX_LENGTH]; i++)
    {
    	UART3_Printf("%c ", rx_data[IDX_PAYLOAD+i]);
    }

    UART3_Printf("\r\n");

    UART3_Printf("Ready for next packet\r\n");
}


uint8_t receive_packet(uint8_t *rx)
{
    uint8_t byte;

    memset(rx, 0, 128);

    while (1)
    {
        /* Read one byte */

        if (HAL_UART_Receive(&huart2, &byte, 1, HAL_MAX_DELAY) != HAL_OK)
        {
            return 0;
        }


        /* Check for valid packet type */

        if (byte == PKT_DATA ||
            byte == PKT_ACK ||
            byte == PKT_STATUS_REQ ||
            byte == PKT_STATUS_RESP)
        {
            rx[IDX_TYPE] = byte;
            break;
        }

        /* Ignore invalid byte and keep searching */
    }

    /* Receive SRC */

    if (HAL_UART_Receive(&huart2,
                         &rx[IDX_SRC],
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }
    /* Validate SRC */



    if (rx[IDX_SRC] > DEV_ESP32C6)
    {
        return 0;
    }

    /* Receive DEST */

    if (HAL_UART_Receive(&huart2,
                         &rx[IDX_DEST],
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }

    /* Validate DEST */

    if (rx[IDX_DEST] > DEV_ESP32C6)
    {
        return 0;
    }

    /* Receive LENGTH */

    if (HAL_UART_Receive(&huart2,
                         &rx[IDX_LENGTH],
                         1,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return 0;
    }

    if (rx[IDX_LENGTH] > 64)
    {
        return 0;
    }

    /* Receive Payload */

    if (rx[IDX_LENGTH] > 0)
    {
        if (HAL_UART_Receive(&huart2,
                             &rx[IDX_PAYLOAD],
                             rx[IDX_LENGTH],
                             HAL_MAX_DELAY) != HAL_OK)
        {
            return 0;
        }
    }

    return 1;
}
//--------------------------------------------------------------------

/*void UART2_StartReceiveIT(void)
{
    uart2_rx_byte = 0;

    uart2_packet_index = 0;
    uart2_expected_length = 0;
    uart2_packet_ready = 0;
    uart2_receiving = 0;

    HAL_UART_Receive_IT(&huart2,
                       (uint8_t *)&uart2_rx_byte,
                       1);
}
*/

//---------------------REVERSE ROUTING FUNCTIONS -----------

//============================================================
// REVERSE ROUTING: RPI -> STM32
//============================================================

void handle_status_request_reverse(uint8_t *rx)
{
    UART3_Print(
        "\r\n===== REVERSE STATUS REQUEST =====\r\n"
    );

    UART3_Printf(
        "TYPE : %d\r\n",
        rx[IDX_TYPE]
    );

    UART3_Printf(
        "SRC  : %d\r\n",
        rx[IDX_SRC]
    );

    UART3_Printf(
        "DEST : %d\r\n",
        rx[IDX_DEST]
    );

    UART3_Printf(
        "LEN  : %d\r\n",
        rx[IDX_LENGTH]
    );

    if (rx[IDX_DEST] != DEV_STM32)
    {
        UART3_Print(
            "Reverse status request is not for STM32\r\n"
        );

        return;
    }

    send_status_response_reverse();
}

void handle_data_packet_reverse(uint8_t *rx)
{
    if (rx[IDX_DEST] != DEV_STM32)
    {
        return;
    }

    UART3_Print(
        "\r\n===== REVERSE DATA PACKET =====\r\n"
    );

    UART3_Printf(
        "TYPE : %d\r\n",
        rx[IDX_TYPE]
    );

    UART3_Printf(
        "SRC  : %d\r\n",
        rx[IDX_SRC]
    );

    UART3_Printf(
        "DEST : %d\r\n",
        rx[IDX_DEST]
    );

    UART3_Printf(
        "LEN  : %d\r\n",
        rx[IDX_LENGTH]
    );

    UART3_Print(
        "PAYLOAD HEX : "
    );

    for (uint8_t i = 0;
         i < rx[IDX_LENGTH];
         i++)
    {
        UART3_Printf(
            "%02X ",
            rx[IDX_PAYLOAD + i]
        );
    }

    UART3_Print("\r\n");

    UART3_Print(
        "PAYLOAD ASCII : "
    );

    for (uint8_t i = 0;
         i < rx[IDX_LENGTH];
         i++)
    {
        UART3_Printf(
            "%c",
            rx[IDX_PAYLOAD + i]
        );
    }

    UART3_Print("\r\n");

    UART3_Print(
        "Reverse data received successfully\r\n"
    );
}

void UART5_StartReceiveIT(void)
{
    uart5_rx_byte = 0;

    uart5_packet_index = 0;
    uart5_expected_length = 0;
    uart5_packet_ready = 0;
    uart5_receiving = 0;

    RS485_REV_RX_Mode();

    HAL_UART_Receive_IT(
        &huart5,
        (uint8_t *)&uart5_rx_byte,
        1
    );
}

void send_status_response_reverse(void)
{
    uint8_t tx_len = 5;
    uint32_t start;

    uart5_tx_buffer[IDX_TYPE] =
        PKT_STATUS_RESP;

    uart5_tx_buffer[IDX_SRC] =
        DEV_STM32;

    uart5_tx_buffer[IDX_DEST] =
        DEV_RPI;

    uart5_tx_buffer[IDX_LENGTH] =
        1;

    uart5_tx_buffer[IDX_PAYLOAD] =
        1;

    UART3_Print(
        "\r\n===== REVERSE STATUS RESPONSE =====\r\n"
    );

    UART3_Print("UART5 TX: ");

    for (uint8_t i = 0; i < tx_len; i++)
    {
        UART3_Printf(
            "%02X ",
            uart5_tx_buffer[i]
        );
    }

    UART3_Print("\r\n");

    RS485_REV_TX_Mode();

    UART3_Printf(
        "UART5 RS485 TX: DE=%d RE=%d\r\n",
        HAL_GPIO_ReadPin(
            RS485_REV_DE_PORT,
            RS485_REV_DE_PIN
        ),
        HAL_GPIO_ReadPin(
            RS485_REV_RE_PORT,
            RS485_REV_RE_PIN
        )
    );

    uart5_tx_complete = 0;
    uart5_tx_busy = 1;

    HAL_StatusTypeDef ret =
        HAL_UART_Transmit_IT(
            &huart5,
            uart5_tx_buffer,
            tx_len
        );

    UART3_Printf(
        "HAL_UART_Transmit_IT(UART5) = %d\r\n",
        ret
    );

    if (ret != HAL_OK)
    {
        UART3_Print(
            "ERROR: UART5 TX start failed\r\n"
        );

        uart5_tx_busy = 0;
        RS485_REV_RX_Mode();
        return;
    }

    start = HAL_GetTick();

    while (uart5_tx_complete == 0)
    {
        if ((HAL_GetTick() - start) > 1000)
        {
            UART3_Print(
                "ERROR: UART5 TX interrupt timeout\r\n"
            );

            uart5_tx_busy = 0;
            RS485_REV_RX_Mode();
            return;
        }
    }

    /*
     * HAL TX callback happened.
     * Now make sure the actual UART shift register
     * has transmitted the final stop bit.
     */
    while (__HAL_UART_GET_FLAG(
               &huart5,
               UART_FLAG_TC
           ) == RESET)
    {
    }

    UART3_Print(
        "UART5 TX COMPLETE\r\n"
    );

    UART3_Print(
        "UART5 TC = SET\r\n"
    );

    RS485_REV_RX_Mode();

    UART3_Printf(
        "UART5 RS485 RX: DE=%d RE=%d\r\n",
        HAL_GPIO_ReadPin(
            RS485_REV_DE_PORT,
            RS485_REV_DE_PIN
        ),
        HAL_GPIO_ReadPin(
            RS485_REV_RE_PORT,
            RS485_REV_RE_PIN
        )
    );

    UART3_Print(
        "REVERSE STATUS RESPONSE SENT\r\n"
    );
}

//--------------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  UART3_Print("UART2 Receive Test Started\r\n");
  RS485_RX_Mode();
  UART2_StartReceiveIT();

  /* New reverse UART5 */
  RS485_REV_RX_Mode();

  UART5_StartReceiveIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  if (uart2_packet_ready)
	      {
	          UART3_Print(">>> uart2_packet_ready = 1 <<<\r\n");

	          uint8_t packet_length;


	          packet_length = HEADER_SIZE +
	                          uart2_packet[IDX_LENGTH];

	          if (packet_length > sizeof(rx_data))
	          {
	              uart2_packet_ready = 0;
	              uart2_packet_index = 0;
	              continue;
	          }

	          memcpy(
	              rx_data,
	              (const void *)uart2_packet,
	              packet_length
	          );


	          uart2_packet_ready = 0;
	          uart2_packet_index = 0;
	          uart2_expected_length = 0;
	          uart2_receiving = 0;

	          UART3_Print("\r\n[STM] COMPLETE PACKET RECEIVED\r\n");

	          UART3_Print("RX: ");

	          for (uint8_t i = 0; i < packet_length; i++)
	          {
	              UART3_Printf("%02X ", rx_data[i]);
	          }

	          UART3_Print("\r\n");

	          switch (rx_data[IDX_TYPE])
	          {
	              case PKT_STATUS_REQ:

	                  handle_status_request(rx_data);

	                  break;

	              case PKT_DATA:

	                  handle_data_packet(rx_data);

	                  break;

	              default:

	                  UART3_Print("Unknown Packet\r\n");

	                  break;
	          }
	      }


	  /* =====================================================
	   * REVERSE ROUTING - UART5
	   * ===================================================== */

	  if (uart5_packet_ready)
	      {
	          uint8_t packet_length;

	          packet_length =
	              HEADER_SIZE + uart5_packet[IDX_LENGTH];

	          if (packet_length > sizeof(rx_reverse))
	          {
	              UART3_Print(
	                  "ERROR: UART5 packet too large\r\n"
	              );

	              uart5_packet_ready = 0;
	              uart5_packet_index = 0;
	              uart5_expected_length = 0;
	              uart5_receiving = 0;

	              continue;
	          }

	          memcpy(
	              rx_reverse,
	              (const void *)uart5_packet,
	              packet_length
	          );

	          uart5_packet_ready = 0;
	          uart5_packet_index = 0;
	          uart5_expected_length = 0;
	          uart5_receiving = 0;

	          UART3_Print(
	              "\r\n[STM] REVERSE COMPLETE PACKET RECEIVED\r\n"
	          );

	          UART3_Print("UART5 RX: ");

	          for (uint8_t i = 0; i < packet_length; i++)
	          {
	              UART3_Printf(
	                  "%02X ",
	                  rx_reverse[i]
	              );
	          }

	          UART3_Print("\r\n");

	          if (rx_reverse[IDX_SRC] != DEV_RPI)
	          {
	              UART3_Print(
	                  "UART5: Invalid source\r\n"
	              );

	              continue;
	          }

	          if (rx_reverse[IDX_DEST] != DEV_STM32)
	          {
	              UART3_Print(
	                  "UART5: Packet not for STM32\r\n"
	              );

	              continue;
	          }

	          switch (rx_reverse[IDX_TYPE])
	          {
	              case PKT_STATUS_REQ:

	                  handle_status_request_reverse(
	                      rx_reverse
	                  );

	                  break;

	              case PKT_DATA:

	                  handle_data_packet_reverse(
	                      rx_reverse
	                  );

	                  break;

	              default:

	                  UART3_Print(
	                      "UART5: Unknown packet type\r\n"
	                  );

	                  break;
	          }
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DE2_Pin|RE2_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DE_Pin|RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DE2_Pin RE2_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = DE2_Pin|RE2_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DE_Pin RE_Pin */
  GPIO_InitStruct.Pin = DE_Pin|RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
