/* EC444 Quest04
*  Electronic Voting
*  November 9, 2020
*  Author: Tony Faller */

/* Note: This code is partially sourced from https://github.com/espressif/esp-idf/tree/39f090a4f1dee4e325f8109d880bf3627034d839/examples/peripherals/adc */

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/param.h>

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Wifi Communication
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

// UDP
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// IR Communication
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

/* Define macros for ADC */
#define DEFAULT_VREF    1100        // Default ADC reference voltage in mV

// /* Define macros for timer */
// #define TIMER_DIVIDER         16                                    //  Hardware timer clock divider
// #define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)      // to seconds
// #define TIMER_INTERVAL_SEC    (0.1)                                 // Sample test interval for the first timer
// #define TEST_WITH_RELOAD      1                                     // Testing will be done with auto reload
// #define TIMER_HOURS           0                                     // Timer hours unit, change to change feeding interval
// #define TIMER_MINUTES         1                                     // Timer minutes unit, change to change feeding interval

/* RMT definitions */
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

/* UART definitions */
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

/* Hardware interrupt definitions */
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

/* LED Output pins definitions */
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15
#define ONBOARD   13

/* Define macros for timer */
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_2_SEC  (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

/* Define ID/color */
#define ID 8
#define COLOR 'R'

/* Define timeout values */
#define TIMEOUT_ELECTION      10

/* Handlers for each task */
TaskHandle_t udpServerHandle;   // UDP receive task
TaskHandle_t udpClientHandle;   // UDP send task

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char) ID;
char leaderID = (char) ID;
int isLeader = 0;
int len_out = 4;
char myColor = (char) COLOR;
char recID = '0';   // ID of received IR transmission
char recColor;      // Color of received IR transmission

// Flag to check if heartbeat is received
int recHeartbeat = 0;

// Flag to check if F -> F IR vote is received
int recVote = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Election & Vote Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client */
/* https://github.com/espressif/esp-idf/blob/master/examples/protocols/sockets/udp_server/main/udp_server.c */

#define HOST_IP_ADDR "192.168.86.44"        // FOB2
#define HOST_IP_ADDR_2 "192.168.86.25"      // FOB3

#define PORT 64209

/* Tags for log messages */
static const char *TAGTX = "electionTx";
static const char *TAGRX = "electionRx";


/* Task to send payload to server (UDP Client) */
static void udp_client_task(void *pvParameters)
{
    // First FOB
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    // Second FOB
    char addr_str_2[128];
    int addr_family_2;
    int ip_protocol_2;

    while (1) {

// #ifdef CONFIG_EXAMPLE_IPV4
        // First FOB
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        // Second FOB
        struct sockaddr_in dest_addr_2;
        dest_addr_2.sin_addr.s_addr = inet_addr(HOST_IP_ADDR_2);
        dest_addr_2.sin_family = AF_INET;
        dest_addr_2.sin_port = htons(PORT);
        addr_family_2 = AF_INET;
        ip_protocol_2 = IPPROTO_IP;
        inet_ntoa_r(dest_addr_2.sin_addr, addr_str_2, sizeof(addr_str_2) - 1);

        // First FOB
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAGTX, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAGTX, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        // Second FOB
        int sock_2 = socket(addr_family_2, SOCK_DGRAM, ip_protocol_2);
        if (sock < 0) {
            ESP_LOGE(TAGTX, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAGTX, "Socket created, sending to %s:%d", HOST_IP_ADDR_2, PORT);

        while (1) {

            // Reset payload
            char payload[200];

            // Set payload
            sprintf(payload, "%x%x%x%x", start, myID, leaderID, (char)isLeader);
            //printf("leaderID = %c\n", payload[2]);    // For debugging

            // Send payload to First FOB
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAGTX, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAGTX, "Message sent: %s", payload);

            // Send payload to Second FOB
            int err_2 = sendto(sock_2, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr_2, sizeof(dest_addr_2));
            if (err_2 < 0) {
                ESP_LOGE(TAGTX, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAGTX, "Message sent: %s", payload);

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAGTX, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        if (sock_2 != -1) {
            ESP_LOGE(TAGTX, "Shutting down socket and restarting...");
            shutdown(sock_2, 0);
            close(sock_2);
        }
    }
    vTaskDelete(NULL);
}


/* Task to receive payload (UDP Server) */
static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAGRX, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAGRX, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAGRX, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAGRX, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAGRX, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAGTX, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAGRX, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAGRX, "%s", rx_buffer);

                // Set heartbeat flag
                if(rx_buffer[3] == leaderID) {recHeartbeat = 1;}

                // printf("Received ID: %c\tMyID: %c\tLeaderID: %c\n", rx_buffer[2], myID+'0', leaderID);  // For debugging
                // if(myID == leaderID){printf("I am the leader!\n");}     // For debugging
                // printf("rx_buffer by index: %c %c %c %c %c\n", rx_buffer[0], rx_buffer[1] , rx_buffer[2] , rx_buffer[3] , rx_buffer[4]); // For debugging

                // Set new leader if leader already chosen or received ID < Fob ID
                if(strlen(rx_buffer) == 5 && rx_buffer[4] == '1') {leaderID = rx_buffer[3];}
                else if(rx_buffer[2] > myID+'0') {leaderID = rx_buffer[2];}
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAGRX, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IR Communication Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "IRComm";       // For debug logs

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Yellow is shorter
    if (myColor == 'G') {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
    }
    else {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    }

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Checksum
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Send task -- sends payload | Start | myID | Start | myID
void send_task(){   
  // while(1) {     // No longer a task so it can be called by button_task

    char *data_out = (char *) malloc(len_out);
    xSemaphoreTake(mux, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char) myColor;
    data_out[2] = (char) myID;
    data_out[3] = genCheckSum(data_out,len_out-1);
    // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

    uart_write_bytes(UART_NUM_1, data_out, len_out);
    xSemaphoreGive(mux);

    // vTaskDelay(5 / portTICK_PERIOD_MS);
  // }
}



// Button task -- rotate through myIDs
void button_task(){
  uint32_t io_num;
  while(1) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      xSemaphoreTake(mux, portMAX_DELAY);
      // if (myID == 3) {
      //   myID = 1;
      // }
      // else {
      //   myID++;
      // }
      xSemaphoreGive(mux);
      printf("Button pressed.\n");
      send_task();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}



// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      if (data_in[0] == start) {
        if (checkCheckSum(data_in,len_out)) {
          ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
          myColor = data_in[1];   // Change color on receiver
          recID = data_in[2];
          recVote = 1;
          recColor = data_in[1];
        }
      }
    }
    else{
      // printf("Nothing received.\n");
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

// LED task to light LED based on traffic state
void led_task(){
  while(1) {
    switch((int)myColor){
      case 'R' : // Red
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 1);
        gpio_set_level(BLUEPIN, 0);
        // printf("Current state: %c\n",status);
        break;
      case 'B' : // Blue
        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 0);
        gpio_set_level(BLUEPIN, 1);
        // printf("Current state: %c\n",status);
        break;
      case 'G' : // Green
        gpio_set_level(GREENPIN, 1);
        gpio_set_level(REDPIN, 0);
        gpio_set_level(BLUEPIN, 0);
        // printf("Current state: %c\n",status);
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// LED task to blink onboard LED based on ID
void id_task(){
  while(1) {
    for (int i = 0; i < (int) myID; i++) {
      gpio_set_level(ONBOARD,1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(ONBOARD,0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Timer task -- R (10 seconds), G (10 seconds), Y (2 seconds)
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1) {
            printf("Action!\n");
            if (myColor == 'R') {
              myColor = 'B';
            }
            else if (myColor == 'B') {
              myColor = 'G';
            }
            else if (myColor == 'G') {
              myColor = 'R';
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Fob -> Leader -> Server Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SERVER_IP_ADDR "192.168.86.35"      // Local RPi IP
#define PORT_2 64210                        // Port for vote communication

char *LEADER_IP_ADDR = "192.168.86.25";        // Hardcoded FOB3 IP

static const char *TAGFLS = "FLSComm";      // Tag for FLS messages

int recLVote = 0;       // Flag to check if vote is recevied by leader

/* Task for receiver fob to send vote to leader */
static void FL_send_task(void *pvParameters){
    char rx_buffer[128];
    char host_ip[14]; // = LEADER_IP_ADDR;
    strcpy(host_ip, LEADER_IP_ADDR);
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

        // Check for the current leader
        if(leaderID == '1'){
            LEADER_IP_ADDR = "192.168.86.29";
        }
        else if(leaderID == '2'){
            LEADER_IP_ADDR = "192.168.86.44";
        }
        else if(leaderID == '3'){
            LEADER_IP_ADDR = "192.168.86.25";
        }

        // printf("recVote = %d\n", recVote);      // For debugging
        if(recVote){
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(LEADER_IP_ADDR);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(PORT_2);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;

            int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAGFLS, "Unable to create socket: errno %d", errno);
                break;
            }
            ESP_LOGI(TAGFLS, "Socket created, sending to %s:%d", LEADER_IP_ADDR, PORT_2);

            // while (1) {
                // Reset payload
                char payload[200];

                // Set payload
                sprintf(payload, "%x,%x", recID, recColor);

                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAGFLS, "Error occurred during sending: errno %d", errno);
                    break;
                }
                ESP_LOGI(TAGFLS, "Message sent");

                struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                socklen_t socklen = sizeof(source_addr);
                int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

                // Error occurred during receiving
                if (len < 0) {
                    ESP_LOGE(TAGFLS, "recvfrom failed: errno %d", errno);
                    break;
                }
                // Data received
                else {
                    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                    ESP_LOGI(TAGFLS, "Received %d bytes from %s:", len, host_ip);
                    ESP_LOGI(TAGFLS, "%s", rx_buffer);
                    if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                        ESP_LOGI(TAGFLS, "Received expected message, reconnecting");
                        break;
                    }
                }

            //     vTaskDelay(2000 / portTICK_PERIOD_MS);
            // }

            if (sock != -1) {
                ESP_LOGE(TAGFLS, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }

        // Reset recLVote flag
        recVote = 0;
        }

        else{
             vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        
    }
    vTaskDelete(NULL);
}


/* Task for leader to receive vote from fob */
static void FL_rec_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT_2);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT_2);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAGFLS, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAGFLS, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAGFLS, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAGFLS, "Socket bound, port %d", PORT_2);

        while (1) {

            ESP_LOGI(TAGFLS, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAGFLS, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAGFLS, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAGFLS, "%s", rx_buffer);

                // printf("%c %c %c %c\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]); // For debugging

                // Set recID and recColor
                recID = (char)rx_buffer[0];

                if(rx_buffer[2]=='5' && rx_buffer[3]=='2')
                    recColor = 'R';
                else if(rx_buffer[2]=='4' && rx_buffer[3]=='2')
                    recColor = 'B';
                else if(rx_buffer[2]=='4' && rx_buffer[3]=='7')
                    recColor = 'G';
                else
                    recColor = 'B';


                printf("Leader recColor value: %c\n", recColor);  // for debugging

                // Set recL flag
                recLVote = 1;

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAGFLS, "Error occurred during sending: errno %d", errno);
                    break;
                }

                // // Set heartbeat flag
                // if(rx_buffer[3] == leaderID) {recHeartbeat = 1;}

                // // printf("Received ID: %c\tMyID: %c\tLeaderID: %c\n", rx_buffer[2], myID+'0', leaderID);  // For debugging
                // // if(myID == leaderID){printf("I am the leader!\n");}     // For debugging
                // // printf("rx_buffer by index: %c %c %c %c %c\n", rx_buffer[0], rx_buffer[1] , rx_buffer[2] , rx_buffer[3] , rx_buffer[4]); // For debugging

                // // Set new leader if leader already chosen or received ID < Fob ID
                // if(strlen(rx_buffer) == 5 && rx_buffer[4] == '1') {leaderID = rx_buffer[3];}
                // else if(rx_buffer[3] > myID+'0') {leaderID = rx_buffer[3];}
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAGFLS, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

/* Task for leader to send vote to server */
static void LS_send_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = SERVER_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
        // printf("recVote = %d\n", recVote);      // For debugging
        if(isLeader && (recLVote || recVote)){
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP_ADDR);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;

            int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAGFLS, "Unable to create socket: errno %d", errno);
                break;
            }
            ESP_LOGI(TAGFLS, "Socket created, sending to %s:%d", SERVER_IP_ADDR, PORT);

            // while (1) {
                // Reset payload
                char payload[200];

                // Set payload
                sprintf(payload, "%x,%x", recID, recColor);

                int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAGFLS, "Error occurred during sending: errno %d", errno);
                    break;
                }
                ESP_LOGI(TAGFLS, "Message sent");

                struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
                socklen_t socklen = sizeof(source_addr);
                int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

                // Error occurred during receiving
                if (len < 0) {
                    ESP_LOGE(TAGFLS, "recvfrom failed: errno %d", errno);
                    break;
                }
                // Data received
                else {
                    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                    ESP_LOGI(TAGFLS, "Received %d bytes from %s:", len, host_ip);
                    ESP_LOGI(TAGFLS, "%s", rx_buffer);
                    if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                        ESP_LOGI(TAGFLS, "Received expected message, reconnecting");
                        break;
                    }
                }

            //     vTaskDelay(2000 / portTICK_PERIOD_MS);
            // }

            if (sock != -1) {
                ESP_LOGE(TAGFLS, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }

        // Reset recLVote and recVote flags
        recLVote = 0;
        recVote = 0;
        }

        else{
             vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        
    }
    vTaskDelete(NULL);
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Function to initialize Wifi */
void init_wifi(){
    // Initialize Wifi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
}

static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

// Configure timer
static void alarm_init() {
    // Select and initialize basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Button interrupt init
static void button_init() {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Election Functions //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Main election function */
void election(){
    printf(">> Running election...\n");

    // Wait for election to timeout
    for(int i=0; i<TIMEOUT_ELECTION;i++){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Pause election tasks
    vTaskSuspend(udpServerHandle);
    vTaskSuspend(udpClientHandle);
    printf(" - Election tasks paused\n");

    // Change leaderID
    if(myID == leaderID)
        {isLeader = 1;}
    else
        {isLeader = 0;}

    printf(" - Election complete. LeaderID:");
    if(leaderID < 48) {printf("%c", leaderID+'0');}
    else if(leaderID > 57) {printf("%c", leaderID-'0');}
    else {printf("%c", leaderID);}
    printf("\n\n");
}

/* Function to restart the election */
void election_restart(){
    printf(">> Restarting election...\n");

    // Reset payload values
    myID = (char) ID;
    leaderID = (char) ID;
    isLeader = 0;

    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Resume election tasks
    vTaskResume(udpServerHandle);
    vTaskResume(udpClientHandle);

    // Call election
    election();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void){

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize wifi
    init_wifi();

    // Initialize IR Stuff
    rmt_tx_init();
    uart_init();
    led_init();
    alarm_init();
    button_init();

    // Create FLS Communication tasks
    xTaskCreate(FL_send_task, "FL_send", 4096, NULL, 9, NULL);
    xTaskCreate(FL_rec_task, "FL_rec", 4096, (void*)AF_INET, 8, NULL);
    xTaskCreate(LS_send_task, "LS_send", 4096, NULL, 7, NULL);

    // Create Election tasks
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 6, &udpServerHandle);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, &udpClientHandle);

    // Create IR Tasks
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, 4, NULL);
    // xTaskCreate(send_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024*2, NULL, 3, NULL);
    xTaskCreate(id_task, "set_id_task", 1024*2, NULL, 2, NULL);
    xTaskCreate(button_task, "button_task", 1024*2, NULL, 1, NULL);

    // Initial election
    election();

    // If you are the leader, send out your heartbeat. Else open your ears to hear heartbeat
    if(isLeader) {vTaskResume(udpClientHandle);}
    else {vTaskResume(udpServerHandle);}

    while(1){

        // If you're not the leader, check for heartbeat
        if(!isLeader){
            if(recHeartbeat){
                printf(">> Heartbeat received\n");
                recHeartbeat = 0;   // Reset heartbeat flag
            }
            else{
                printf(">> No heartbeat received in time\n");
                election_restart();
                if(isLeader) {vTaskResume(udpClientHandle);}
                else {vTaskResume(udpServerHandle);}
            }
        }

        else{
            printf("I am the leader, hear my heartbeat.\n");
        }
        

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

}