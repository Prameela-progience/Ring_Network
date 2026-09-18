#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/select.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>

#define UART_DEVICE  "/dev/serial0"
#define UART_REVERSE "/dev/ttyAMA1"

#define GPIO_CHIP "/dev/gpiochip0"

#define RS485_DE_GPIO 17
#define RS485_RE_GPIO 27

static int gpio_chip_fd = -1;
static int gpio_line_fd = -1;

#define BAUDRATE B115200

/* Packet Types */

#define PKT_DATA           1
#define PKT_ACK            2
#define PKT_STATUS_REQ     3
#define PKT_STATUS_RESP    4

/* Device IDs */

#define DEV_RPI            0x00
#define DEV_ESP32S3        0x01
#define DEV_STM32          0x02
#define DEV_ESP32C6        0x03

/* Packet Index */

#define IDX_TYPE           0
#define IDX_SRC            1
#define IDX_DEST           2
#define IDX_LENGTH         3
#define IDX_PAYLOAD        4

#define HEADER_SIZE        4
#define MAX_PAYLOAD_SIZE   64
#define PACKET_SIZE        (HEADER_SIZE + MAX_PAYLOAD_SIZE)

#define RESPONSE_TIMEOUT_MS    1000


typedef struct                                           
{                                                         
    uint8_t type;                                
    uint8_t src;                                           
    uint8_t dest;                           
    uint8_t length;                                      
    uint8_t payload[MAX_PAYLOAD_SIZE];                                       
                                  
}packet_t;  


//-------------------------------------------------------
static void update_status(packet_t *pkt);
static void print_current_route(void);
static void monitor_data(int fd);
static void route_data(int fd_forward,int fd_reverse);
static void send_data_packet(int fd,uint8_t destination,const char *message);
static int rs485_gpio_init(void);
static void rs485_tx_mode(void);
static void rs485_rx_mode(void);
static void rs485_gpio_cleanup(void);

/* Reverse Routing */
//static void reverse_send_data(int fd,const char *message);

//static void reverse_monitor_data(int fd);

static void reverse_status_check(int fd);
static void reverse_send_data(int fd);
static void reverse_route_step(int fd_forward, int fd_reverse);
//-------------------------------------------------------


typedef enum
{
    ROUTE_FORWARD = 0,
    ROUTE_REVERSE
}route_mode_t;



typedef struct                                                               
{                                                                            
    bool online;                                                             
    time_t last_seen;                                                        
}device_status_t;                                                            
                                                                             
                                                                             
static device_status_t esp32 =                                               
{                                                                            
    false,                                                                   
    0                                                                        
};                                                                           
                                                                             
static device_status_t stm32 =                                               
{                                                                            
    false,                                                                   
    0                                                                        
}; 

static route_mode_t current_route = ROUTE_FORWARD;


static void send_test_data(int fd)
{
    uint8_t tx[] =
    {
        PKT_DATA,
        DEV_RPI,
        DEV_STM32,
        5,
        'H',
        'E',
        'L',
        'L',
        'O'
    };

    rs485_tx_mode();

    write(fd, tx, sizeof(tx));
    tcdrain(fd);

    rs485_rx_mode();

    printf("TX : ");

    for (int i = 0; i < sizeof(tx); i++)
        printf("%02X ", tx[i]);

    printf("\n");
}
//--------------------------------------------------------
// ----- REVERSE ROUTE -----------------------------

/*static void reverse_send_data(int fd,
                              const char *message)
{
    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    uint8_t len = strlen(message);

    tx[IDX_TYPE]   = PKT_DATA;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = DEV_STM32;
    tx[IDX_LENGTH] = len;

    memcpy(&tx[IDX_PAYLOAD], message, len);

    printf("\n===============================\n");
    printf("REVERSE DATA TX\n");
    printf("===============================\n");

    printf("TX : ");

    for(int i=0;i<HEADER_SIZE+len;i++)
        printf("%02X ", tx[i]);

    printf("\n");

    rs485_tx_mode();

    write(fd,
          tx,
          HEADER_SIZE + len);

    tcdrain(fd);

    rs485_rx_mode();

    printf("Message : %s\n", message);
}


static void reverse_monitor_data(int fd)
{
    reverse_send_data(fd,
                      "HELLO STM REVERSE");
}
*/


//-----------------------------------------------------------------



int rs485_gpio_init(void)
{
    struct gpio_v2_line_request req;

    gpio_chip_fd = open(GPIO_CHIP, O_RDONLY);

    if (gpio_chip_fd < 0)
    {
        perror("open gpiochip");
        return -1;
    }

    memset(&req, 0, sizeof(req));

    req.offsets[0] = RS485_DE_GPIO;
    req.offsets[1] = RS485_RE_GPIO;

    req.num_lines = 2;

    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    strcpy(req.consumer, "rs485");

    if (ioctl(gpio_chip_fd,
              GPIO_V2_GET_LINE_IOCTL,
              &req) < 0)
    {
        perror("GPIO_V2_GET_LINE_IOCTL");
        close(gpio_chip_fd);
        return -1;
    }

    gpio_line_fd = req.fd;

    rs485_rx_mode();

    return 0;
}


void rs485_tx_mode(void)
{
printf("GPIO -> TX\n");
    struct gpio_v2_line_values values;

    memset(&values, 0, sizeof(values));

    values.mask = 0x3;

    values.bits = 0x3;

    if (ioctl(gpio_line_fd,
              GPIO_V2_LINE_SET_VALUES_IOCTL,
              &values) < 0)
    {
        perror("TX mode");
    }
}

void rs485_rx_mode(void)
{
printf("GPIO -> RX\n");
    struct gpio_v2_line_values values;

    memset(&values, 0, sizeof(values));

    values.mask = 0x3;

    values.bits = 0x0;

    if (ioctl(gpio_line_fd,
              GPIO_V2_LINE_SET_VALUES_IOCTL,
              &values) < 0)
    {
        perror("RX mode");
    }
}

void rs485_gpio_cleanup(void)
{
    if (gpio_line_fd >= 0)
        close(gpio_line_fd);

    if (gpio_chip_fd >= 0)
        close(gpio_chip_fd);
}

//--------------------- Backword routing -----------------//
//--------------------------------------------------------
static void update_route(bool esp_online,
                         bool stm_online)
{
    route_mode_t previous_route = current_route;

    if (esp_online && stm_online)
    {
        current_route = ROUTE_FORWARD;
    }
    else
    {
        current_route = ROUTE_REVERSE;
    }

    if (previous_route != current_route)
    {
        print_current_route();
    }
}

//-------------------------------------------

static bool is_forward_route(void)
{
    return (current_route == ROUTE_FORWARD);
}
//------------------------------------------------
static void print_current_route(void)
{
    printf("\n==============================\n");

    if(current_route == ROUTE_FORWARD)
    {
        printf("Current Route : FORWARD\n");
    }
    else
    {
        printf("Current Route : REVERSE\n");
    }

    printf("==============================\n");
}
//-----------------------------------------------------

//-------------------------------------------------------


//---------------- END -----------------------------------//

static void send_data_packet(int fd,
                             uint8_t destination,
                             const char *message)
{
    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    uint8_t len = strlen(message);

    tx[IDX_TYPE]   = PKT_DATA;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = destination;
    tx[IDX_LENGTH] = len;

    memcpy(&tx[IDX_PAYLOAD], message, len);

    if(write(fd, tx, HEADER_SIZE + len) != (HEADER_SIZE + len))
    {
        perror("write");
        return;
    }

    tcdrain(fd);

    printf("\n---------------------------------\n");
    printf("DATA PACKET SENT\n");
    printf("---------------------------------\n");
    printf("SRC  : %d\n", DEV_RPI);
    printf("DEST : %d\n", destination);
    printf("DATA : %s\n", message);
}

static void monitor_data(int fd)
{
    send_data_packet(fd,
                     DEV_ESP32S3,
                     "HELLO ESP");

    usleep(500000);

    send_data_packet(fd,
                     DEV_STM32,
                     "HELLO STM");
}

static int uart_init_reverse(void)
{
    int fd;

    fd = open(UART_REVERSE, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror("open reverse uart");
        return -1;
    }

    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr reverse");
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CREAD | CLOCAL);

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr reverse");
        close(fd);
        return -1;
    }

    printf("Reverse UART initialized (%s)\n", UART_REVERSE);

    return fd;
}

static int uart_init(void)
{
    int fd;

    fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);

    if(fd < 0)
    {
        perror("open");
        return -1;
    }

    struct termios tty;

    if(tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        close(fd);
        return -1;
    }
	cfmakeraw(&tty);
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= (CREAD | CLOCAL);

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if(tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

static int read_exact(int fd, uint8_t *buffer, int size, int timeout_ms)
{
    int received = 0;

    while(received < size)
    {
        fd_set rfds;

        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;

        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);

        if(ret == 0)
        {
    printf("Timeout waiting for %d bytes\n", size - received);
            return -1;
        }

        if(ret < 0)
        {
	perror("select");
            return -1;
        }

        int n = read(fd,
                     buffer + received,
                     size - received);

printf("\n---------------------------------\n");
printf("read() returned : %d\n", n);

        if(n <= 0)
        {
            return -1;
        }
printf("Received Bytes : ");

    for (int i = 0; i < n; i++)
        printf("%02X ", buffer[received + i]);

    printf("\n");

        received += n;
    }

    return received;
}

static int receive_packet(int fd, packet_t *pkt)
{
    uint8_t header[HEADER_SIZE];

    memset(pkt,0,sizeof(packet_t));
printf("\nWaiting for HEADER...\n");
    if(read_exact(fd,
                  header,
                  HEADER_SIZE,
                  RESPONSE_TIMEOUT_MS) != HEADER_SIZE)
    {
        return -1;
    }
printf("HEADER RECEIVED : ");
for(int i = 0; i < HEADER_SIZE; i++)
    printf("%02X ", header[i]);

printf("\n");

    pkt->type   = header[IDX_TYPE];
    pkt->src    = header[IDX_SRC];
    pkt->dest   = header[IDX_DEST];
    pkt->length = header[IDX_LENGTH];

    if(pkt->length > MAX_PAYLOAD_SIZE)
    {
        printf("Invalid Payload Length\n");
        return -1;
    }

    if(pkt->length)
    {
        if(read_exact(fd,
                      pkt->payload,
                      pkt->length,
                      RESPONSE_TIMEOUT_MS) != pkt->length)
        {
            return -1;
        }
    }

	printf("\n===== RX Packet =====\n");
printf("TYPE : %u\n", pkt->type);
printf("SRC  : %u\n", pkt->src);
printf("DEST : %u\n", pkt->dest);
printf("LEN  : %u\n", pkt->length);

if(pkt->length)
{
    printf("DATA : ");

    for(int i = 0; i < pkt->length; i++)
        printf("%02X ", pkt->payload[i]);

    printf("\n");
}


    return 0;
}


//---------WORKING STATUS REQ FUNCTION ----------------

static void send_status_request(int fd, uint8_t destination)
{
    uint8_t tx[HEADER_SIZE];

    tx[IDX_TYPE]   = PKT_STATUS_REQ;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = destination;
    tx[IDX_LENGTH] = 0;

//    tcflush(fd, TCIFLUSH);

printf("\nTX RAW : ");

for(int i=0;i<HEADER_SIZE;i++)
    printf("%02X ", tx[i]);

printf("\n");
/*    if(write(fd, tx, HEADER_SIZE) != HEADER_SIZE)
    {
        perror("write");
        return;
    }
*/
int ret = write(fd, tx, HEADER_SIZE);

printf("write() returned %d\n", ret);

    tcdrain(fd);

    printf("\n---------------------------------\n");
    printf("STATUS REQUEST\n");
    printf("---------------------------------\n");
    printf("SRC  : %d\n", DEV_RPI);
    printf("DEST : %d\n", destination);
}

/*
static void send_status_request(int fd, uint8_t destination)
{
    uint8_t tx[HEADER_SIZE];

    tx[IDX_TYPE]   = PKT_STATUS_REQ;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = destination;
    tx[IDX_LENGTH] = 0;

//     Only enable RS485 direction control when using the reverse UART 
    if (fd != fd_forward)
        rs485_tx_mode();

    if (write(fd, tx, HEADER_SIZE) != HEADER_SIZE)
    {
        perror("write");

        if (fd != fd_forward)
            rs485_rx_mode();

        return;
    }

    tcdrain(fd);

    if (fd != fd_forward)
        rs485_rx_mode();

    printf("\n---------------------------------\n");
    printf("STATUS REQUEST\n");
    printf("---------------------------------\n");
    printf("SRC  : %d\n", DEV_RPI);
    printf("DEST : %d\n", destination);
}
*/

static void update_status(packet_t *pkt)
{
    if(pkt->type != PKT_STATUS_RESP)
        return;

    if(pkt->length != 1)
        return;

    if(pkt->payload[0] != 1)
        return;

    if(pkt->src == DEV_ESP32S3)
    {
        esp32.online = true;
        esp32.last_seen = time(NULL);
    }
    else if(pkt->src == DEV_STM32)
    {
        stm32.online = true;
        stm32.last_seen = time(NULL);
    }
}

static void print_status_table(void)
{
//    printf("\033[2J");
//    printf("\033[H");

    printf("=====================================================\n");
    printf("               NETWORK STATUS\n");
    printf("=====================================================\n");

    printf("+------------+-----------+\n");
    printf("| Device     | Status    |\n");
    printf("+------------+-----------+\n");

    printf("| ESP32-S3   | %-9s |\n",
            esp32.online ? "ONLINE" : "OFFLINE");

    printf("| STM32      | %-9s |\n",
            stm32.online ? "ONLINE" : "OFFLINE");

    printf("+------------+-----------+\n");

    printf("\n");

    if(esp32.online)
        printf("ESP Last Seen : %s", ctime(&esp32.last_seen));

    if(stm32.online)
        printf("STM Last Seen : %s", ctime(&stm32.last_seen));
	printf("\nCurrent Route : %s\n",is_forward_route() ? "FORWARD" : "REVERSE");
	fflush(stdout);
}
static void monitor_esp(int fd)
{
    packet_t pkt;

    esp32.online = false;
tcflush(fd, TCIFLUSH);
    send_status_request(fd, DEV_ESP32S3);

    if(receive_packet(fd, &pkt) < 0)
    {
//	esp32.online = false;
        printf("ESP Timeout\n");
        return;
    }

    if(pkt.type != PKT_STATUS_RESP)
    {
        printf("Invalid Packet\n");
        return;
    }

    if(pkt.src != DEV_ESP32S3)
    {
        printf("Unexpected Source\n");
        return;
    }
	
	if(pkt.dest != DEV_RPI)
    {
        printf("Packet not for RPi\n");
        return;
    }

    esp32.online = true;

    update_status(&pkt);
}

static void monitor_stm(int fd)
{
    packet_t pkt;
stm32.online = false;
tcflush(fd, TCIFLUSH);
    send_status_request(fd, DEV_STM32);

    if(receive_packet(fd, &pkt) < 0)
    {
        stm32.online = false;
        printf("STM Timeout\n");
        return;
    }

    if(pkt.type != PKT_STATUS_RESP)
    {
        printf("Invalid Packet\n");
        return;
    }

    if(pkt.src != DEV_STM32)
    {
        printf("Unexpected Source\n");
        return;
    }

    if(pkt.dest != DEV_RPI)
    {
        printf("Packet not for RPi\n");
        return;
    }

    stm32.online = true;

    update_status(&pkt);
}

//----------------------------------------------------------------
static void monitor_devices(int fd)
{
    monitor_esp(fd);

    usleep(50000);

    monitor_stm(fd);

    print_status_table();
}


/*
static void network_monitor(int fd_forward, int fd_reverse)
{
    monitor_devices(fd_forward);

    if (esp32.online && stm32.online)
    {
        current_route = ROUTE_FORWARD;
        return;
    }

    printf("\n=================================\n");
    printf(" FORWARD FAILED\n");
    printf(" TRYING REVERSE\n");
    printf("=================================\n");

    monitor_stm_reverse(fd_reverse);

    if (stm32.online)
    {
        current_route = ROUTE_REVERSE;

        printf("\nReverse Route Activated\n");
    }
//print_status_table();
}
*/
//------------------------------------------------------------
//-------------------------REVERSE ROUTING FUNCTIONS ---------


//===========================================================
// NEW REVERSE ROUTING
// RPI -> STM32 directly when ESP32 status fails
//===========================================================

static void reverse_status_check(int fd)
{
    uint8_t tx[HEADER_SIZE];
    packet_t pkt;

    memset(&pkt, 0, sizeof(pkt));

    /*
     * STATUS REQUEST:
     * TYPE = 3
     * SRC  = RPI
     * DEST = STM32
     * LEN  = 0
     */
    tx[IDX_TYPE]   = PKT_STATUS_REQ;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = DEV_STM32;
    tx[IDX_LENGTH] = 0;

    printf("\n=================================\n");
    printf("REVERSE STATUS REQUEST\n");
    printf("=================================\n");

    printf("TX RAW : ");
    for (int i = 0; i < HEADER_SIZE; i++)
    {
        printf("%02X ", tx[i]);
    }
    printf("\n");

    /*
     * Reverse UART is connected directly to STM32 RS485.
     */
    rs485_tx_mode();

    int ret = write(fd, tx, HEADER_SIZE);

    printf("write() returned %d\n", ret);

    if (ret != HEADER_SIZE)
    {
        perror("reverse status write");
        rs485_rx_mode();
        stm32.online = false;
        return;
    }

    /*
     * Wait until all bytes have physically left UART.
     */
    tcdrain(fd);

    /*
     * Give RS485 transceiver a small settling time.
     */
    usleep(2000);

    /*
     * Change to receive mode BEFORE waiting for STM response.
     */
    rs485_rx_mode();

    printf("GPIO -> RX\n");

    /*
     * STM response:
     * 04 02 00 01 01
     */
    if (receive_packet(fd, &pkt) < 0)
    {
        printf("Reverse STM status timeout\n");
        stm32.online = false;
        return;
    }

    printf("\n===== REVERSE STM STATUS RESPONSE =====\n");
    printf("TYPE : %u\n", pkt.type);
    printf("SRC  : %u\n", pkt.src);
    printf("DEST : %u\n", pkt.dest);
    printf("LEN  : %u\n", pkt.length);

    if (pkt.length)
    {
        printf("DATA : ");

        for (int i = 0; i < pkt.length; i++)
        {
            printf("%02X ", pkt.payload[i]);
        }

        printf("\n");
    }

    /*
     * Validate STM response.
     */
    if (pkt.type != PKT_STATUS_RESP)
    {
        printf("Reverse STM: invalid packet type\n");
        stm32.online = false;
        return;
    }

    if (pkt.src != DEV_STM32)
    {
        printf("Reverse STM: unexpected source\n");
        stm32.online = false;
        return;
    }

    if (pkt.dest != DEV_RPI)
    {
        printf("Reverse STM: packet not for RPI\n");
        stm32.online = false;
        return;
    }

    if (pkt.length != 1 || pkt.payload[0] != 1)
    {
        printf("Reverse STM: OFFLINE response\n");
        stm32.online = false;
        return;
    }

    stm32.online = true;
    stm32.last_seen = time(NULL);

    printf("STM32 ONLINE through REVERSE route\n");
}


//===========================================================
// SEND DATA DIRECTLY TO STM32
//===========================================================

static void reverse_send_data(int fd)
{
    const char *message = "HELLO STM";

    uint8_t len = strlen(message);

    uint8_t tx[HEADER_SIZE + MAX_PAYLOAD_SIZE];

    tx[IDX_TYPE]   = PKT_DATA;
    tx[IDX_SRC]    = DEV_RPI;
    tx[IDX_DEST]   = DEV_STM32;
    tx[IDX_LENGTH] = len;

    memcpy(&tx[IDX_PAYLOAD], message, len);

    printf("\n=================================\n");
    printf("REVERSE DATA\n");
    printf("=================================\n");

    printf("TX RAW : ");

    for (int i = 0; i < HEADER_SIZE + len; i++)
    {
        printf("%02X ", tx[i]);
    }

    printf("\n");

    rs485_tx_mode();

    int ret = write(fd,
                    tx,
                    HEADER_SIZE + len);

    printf("write() returned %d\n", ret);

    if (ret != HEADER_SIZE + len)
    {
        perror("reverse data write");
        rs485_rx_mode();
        return;
    }

    tcdrain(fd);

    /*
     * Ensure the last byte has physically gone out
     * before changing RS485 direction.
     */
    usleep(2000);

    rs485_rx_mode();

    printf("Reverse DATA sent to STM32\n");
    printf("SRC  : %d\n", DEV_RPI);
    printf("DEST : %d\n", DEV_STM32);
    printf("DATA : %s\n", message);
}


//===========================================================
// ONE COMPLETE REVERSE ROUTE STEP
//===========================================================

static void reverse_route_step(int fd_forward, int fd_reverse)
{
    reverse_status_check(fd_reverse);

    print_status_table();

    if (stm32.online)
    {
        reverse_send_data(fd_reverse);
    }
    else
    {
        printf("STM32 OFFLINE - reverse data not sent\n");
    }

    monitor_esp(fd_forward);

    if (esp32.online)
    {
        printf("\n=================================\n");
        printf("ESP32 ONLINE AGAIN\n");
        printf("Switching to FORWARD Route\n");
        printf("=================================\n");

        current_route = ROUTE_FORWARD;
    }
}


/*
static void reverse_route_step(int fd_forward, int fd_reverse)
{

    (void)fd_reverse;

    printf("\n=================================\n");
    printf("REVERSE ROUTE TEST\n");
    printf("=================================\n");

    printf("STM32 reverse communication: TEST DISABLED\n");

    monitor_esp(fd_forward);

    if (esp32.online)
    {
        printf("\n=================================\n");
        printf("ESP32 ONLINE AGAIN\n");
        printf("Switching to FORWARD Route\n");
        printf("=================================\n");

        current_route = ROUTE_FORWARD;
    }
    else
    {
        printf("ESP32 still OFFLINE\n");
        current_route = ROUTE_REVERSE;
    }
}*/
//-----------------------REVERSE END -------------------------

void test_reverse_send(int fd)
{
    uint8_t tx[] = {
        0x01,
        0x00,
        0x02,
        0x09,
        'H','E','L','L','O',' ','R','P','I'
    };

    printf("\n=====================================\n");
    printf("REVERSE UART5 TEST\n");
    printf("RPi -> STM32\n");
    printf("=====================================\n");

    printf("TX RAW: ");

    for (int i = 0; i < sizeof(tx); i++)
        printf("%02X ", tx[i]);

    printf("\n");

    /* RS485 TX mode */
    printf("GPIO -> TX\n");

rs485_tx_mode();
    usleep(10000);

    int ret = write(fd, tx, sizeof(tx));

    printf("write() returned %d\n", ret);

    if (ret != sizeof(tx))
    {
        perror("write");
        return;
    }

    /*
     * Give UART time to physically transmit
     * before changing RS485 direction.
     */
    tcdrain(fd);

    usleep(10000);

    /* RS485 RX mode */
rs485_rx_mode();
    printf("GPIO -> RX\n");

    printf("REVERSE TEST TX COMPLETE\n");
}

int main(void)
{
    int fd_forward,fd_reverse;;

    fd_forward = uart_init();

    if(fd_forward < 0)
    {
        return -1;
    }
	fd_reverse = uart_init_reverse();

	if (fd_reverse < 0)
	{
	    return -1;
	}
	
	if (rs485_gpio_init() < 0)
	{
	    return -1;
	}
    printf("\n");
    printf("=========================================\n");
    printf(" Raspberry Pi Network Monitor Started\n");
    printf("=========================================\n");

while (1)
{
    if(current_route == ROUTE_FORWARD)
    {
        monitor_devices(fd_forward);

        if(!esp32.online)
        {
            printf("\n=================================\n");
            printf("ESP OFFLINE\n");
            printf("Switching to REVERSE Route\n");
            printf("=================================\n");

            current_route = ROUTE_REVERSE;

            continue;
        }

        usleep(500000);

        monitor_data(fd_forward);
    }
    else
    {
        reverse_route_step(fd_forward, fd_reverse);
    }

    sleep(2);
}


/*
while (1)
{
	test_reverse_send(fd_reverse);                                                                 
	sleep(1);
}
*/
    close(fd_forward);
    close(fd_reverse);

rs485_gpio_cleanup();
    return 0;
}
//-----------------------------




/*
int main(void)
{
    int fd_reverse;

    fd_reverse = uart_init_reverse();

    if (fd_reverse < 0)
        return -1;

    if (rs485_gpio_init() < 0)
        return -1;

    printf("\n");
    printf("=========================================\n");
    printf("   RPi <----> STM32 Reverse UART Test\n");
    printf("=========================================\n");

while (1)
{
    send_test_data(fd_reverse);
    sleep(2);
}
    close(fd_reverse);
    rs485_gpio_cleanup();

    return 0;
}
*/
