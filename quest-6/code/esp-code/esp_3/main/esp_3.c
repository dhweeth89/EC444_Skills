/* EC444 Quest06 
*  ESP_3 Code 
*  December 6, 2020
*  Authors: Shaivya Gupta, Tony Faller */

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

// i2c
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_attr.h"
#include "sdkconfig.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "./ADXL343.h"

// PWM
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Define esp ID */
#define ID 3

#define GPIO_ONBOARD_LED  13  // Blinks to indicate ID

/* Define start signal for UDP */
#define UDP_start "Acceleration"        // This string determines which CSV file the paylaod is appended to

/* Define macros for servo pwm */
#define SERVO_MIN_PULSEWIDTH 550    // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2550   // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180        // Maximum angle in degree upto which servo can rotate (-90 to +90)

/* Define macros for timer */
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload
#define TIMER_HOURS           0     // Timer hours unit, change to change feeding interval
#define TIMER_MINUTES         1    // Timer minutes unit, change to change feeding interval

/* Define macros for alphanumeric display */ 
// 14-Segment Display
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// Accelerometer stuff
//If abs value of pitch or roll is greater than 30, set alarm
float xp;
float yp;
float zp;
float x;
float y;
float z;
float roll;
float pitch;
float negateX;
float mv_v;
uint32_t accel_counter;
uint32_t rp_counter;

//Boolean to determine if button toggled to turn on fan
bool masterFan = 0;

//Boolean to determine if thermistors say fan must turn on
bool tempFan = 0;

//Boolean to determine if accelerometer reads an alert
bool alert = 0;

//Boolean to determine if we need to reset timer after masterFan and/or tempFan are done
bool reset_timer = 0;

//Flag to show whether or not flag is currently running
uint8_t runningFan = 0;

//Not used but don't feel like removing.
bool safeToAlnum = 1;

//Made slave_addr an int so that it can keep switching between accelerometer and display addresses
uint32_t SLAVE_ADDR; 

/* Global timer and display variables */
// ASCII Table taken from https://github.com/adafruit/Adafruit_LED_Backpack/blob/master/Adafruit_LEDBackpack.cpp
static const uint16_t alphafonttable[] = {

  0b0000000000000001, 0b0000000000000010, 0b0000000000000100,
  0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
  0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
  0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
  0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
  0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
  0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
  0b0011101000000000, 0b0001011100000000,
  0b0000000000000000, //
  0b0000000000000110, // !
  0b0000001000100000, // "
  0b0001001011001110, // #
  0b0001001011101101, // $
  0b0000110000100100, // %
  0b0010001101011101, // &
  0b0000010000000000, // '
  0b0010010000000000, // (
  0b0000100100000000, // )
  0b0011111111000000, // *
  0b0001001011000000, // +
  0b0000100000000000, // ,
  0b0000000011000000, // -
  0b0000000000000000, // .
  0b0000110000000000, // /
  0b0000110000111111, // 0
  0b0000000000000110, // 1
  0b0000000011011011, // 2
  0b0000000010001111, // 3
  0b0000000011100110, // 4
  0b0010000001101001, // 5
  0b0000000011111101, // 6
  0b0000000000000111, // 7
  0b0000000011111111, // 8
  0b0000000011101111, // 9
  0b0001001000000000, // :
  0b0000101000000000, // ;
  0b0010010000000000, // <
  0b0000000011001000, // =
  0b0000100100000000, // >
  0b0001000010000011, // ?
  0b0000001010111011, // @
  0b0000000011110111, // A
  0b0001001010001111, // B
  0b0000000000111001, // C
  0b0001001000001111, // D
  0b0000000011111001, // E
  0b0000000001110001, // F
  0b0000000010111101, // G
  0b0000000011110110, // H
  0b0001001000000000, // I
  0b0000000000011110, // J
  0b0010010001110000, // K
  0b0000000000111000, // L
  0b0000010100110110, // M
  0b0010000100110110, // N
  0b0000000000111111, // O
  0b0000000011110011, // P
  0b0010000000111111, // Q
  0b0010000011110011, // R
  0b0000000011101101, // S
  0b0001001000000001, // T
  0b0000000000111110, // U
  0b0000110000110000, // V
  0b0010100000110110, // W
  0b0010110100000000, // X
  0b0001010100000000, // Y
  0b0000110000001001, // Z
  0b0000000000111001, // [
  0b0010000100000000, //
  0b0000000000001111, // ]
  0b0000110000000011, // ^
  0b0000000000001000, // _
  0b0000000100000000, // `
  0b0001000001011000, // a
  0b0010000001111000, // b
  0b0000000011011000, // c
  0b0000100010001110, // d
  0b0000100001011000, // e
  0b0000000001110001, // f
  0b0000010010001110, // g
  0b0001000001110000, // h
  0b0001000000000000, // i
  0b0000000000001110, // j
  0b0011011000000000, // k
  0b0000000000110000, // l
  0b0001000011010100, // m
  0b0001000001010000, // n
  0b0000000011011100, // o
  0b0000000101110000, // p
  0b0000010010000110, // q
  0b0000000001010000, // r
  0b0010000010001000, // s
  0b0000000001111000, // t
  0b0000000000011100, // u
  0b0010000000000100, // v
  0b0010100000010100, // w
  0b0010100011000000, // x
  0b0010000000001100, // y
  0b0000100001001000, // z
  0b0000100101001001, // {
  0b0001001000000000, // |
  0b0010010010001001, // }
  0b0000010100100000, // ~
  0b0011111111111111,
};


// Time in seconds
int sec = TIMER_HOURS * 60 * 60 + TIMER_MINUTES * 60 + 1;

//References to the various tasks. Used to suspend and resume tasks.
TaskHandle_t timerHandle;
TaskHandle_t alnumHandle;
TaskHandle_t servoHandle;
TaskHandle_t accelHandle;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UDP Communication Stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client */

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT    // Port to communicate with server
#define PORT2 64208                 // Port to communicate between ESPs

static const char *TAG = "ESP_3";
static const char *TAGRX = "ESP2->ESP3";


/* Task to send sensor readings to server */
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        // Setting scope_id to the connecting interface for correct routing if IPv6 Local Link supplied
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        // int counter = 0;
        // float thermistor = 0;

        while (1) {
            // Reset payload
            char payload[200];
            // memset(payload, '\0', sizeof(payload));

            // Set payload
            sprintf(payload, "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d", UDP_start, xp, yp, zp, roll, pitch, runningFan, alert);
            // thermistor = 0;


            // Send payload
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent: %s", payload);

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string

                // // Toggle LED if HTML button is pressed
                // if(len == 7){
                //   masterStop ^= 1; // Toggle stop bit
                //   printf(">> masterStop = %d\n", masterStop);
                //   // toggle_led(ledON);
                // }

                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                // Toggle masterFan boolean if command received from server
                if(len == 7){
                  masterFan ^= 1;
                }

                // Clear alert if command received form server
                if(len == 11){
                  alert = 0;
                }
            }

            

            vTaskDelay(2000 / portTICK_PERIOD_MS);          // Adjust this to change rate of transmission
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        // counter++;

        vTaskDelete(NULL);
    }
}

/* Task to receive thermistor readings from esp_2 */
static void temp_receive_task(void *pvParameters)
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
            dest_addr_ip4->sin_port = htons(PORT2);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT2);
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
        ESP_LOGI(TAGRX, "Socket bound, port %d", PORT2);

        while (1) {

            ESP_LOGI(TAGRX, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAGRX, "recvfrom failed: errno %d", errno);
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

                // Set tempFan flag
                if( strcmp(rx_buffer,"Temperature,1") == 0 ) {tempFan = 1;}
                else {tempFan = 0;}

                printf("\n~~~~~~~~~~~~~~~~~~~~~\n");
                printf("rx_buffer[len-1] = %c\n", rx_buffer[len-1]);
                printf("tempFan = %d\n", tempFan);
                printf("~~~~~~~~~~~~~~~~~~~~~\n\n");

                // printf("Received ID: %c\tMyID: %c\tLeaderID: %c\n", rx_buffer[2], myID+'0', leaderID);  // For debugging
                // if(myID == leaderID){printf("I am the leader!\n");}     // For debugging
                // printf("rx_buffer by index: %c %c %c %c %c\n", rx_buffer[0], rx_buffer[1] , rx_buffer[2] , rx_buffer[3] , rx_buffer[4]); // For debugging

                // Set new leader if leader already chosen or received ID < Fob ID
                // if(strlen(rx_buffer) == 5 && rx_buffer[4] == '1') {leaderID = rx_buffer[3];}
                // else if(rx_buffer[2] > myID+'0') {leaderID = rx_buffer[2];}
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
//  Servo Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Modified from https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control

/* This function calculates pulse width for per-degree rotation */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation){
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

/* This function establishes the base conditions of the servo as it is initialized. It resets it as well. */
void servo_init(){
    /* Initialize MCPWN GPIO */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    // Set GPIO 18 as PWM0A, to which servo is connected

    /* Configure MCPWN */
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(0));   // Reset servo position
    runningFan = 0;
    vTaskDelay(50);
}

//The task that causes the servo to turn back and forth 3 times
void task_servo_wiggle(){

  while(1){

      // printf("Time to feed!\n");

      // printf("Wiggling...\n");

      // runningFan = 1;
      // printf("\n~~~~~~~~~~~~~~~~~~~~\n\n RUNNINGFAN = %d\n\n~~~~~~~~~~~~~~~~~~~~\n", runningFan);

      //Runs the servo back and forth 3 times whenever called.
      for (int i = 0; i < 3; i++)
      {
      	  // printf("Swing %d\n", i);
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(180));  // Wiggle
          vTaskDelay(100);     
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(0));  // Wiggle
          vTaskDelay(100);
          vTaskResume(accelHandle);
      }
      
      if(masterFan || tempFan){
        runningFan = 1;
      }
      else{
        runningFan = 0;
      }
      printf("\n~~~~~~~~~~~~~~~~~~~~\n\n RUNNINGFAN = %d\n\n~~~~~~~~~~~~~~~~~~~~\n", runningFan);

      // printf("Yum!\n");
      
      //Let the display start running again now that servo is done acting and let the timer start working again to decrement timer.
      //Make servo not run again while it isn't needed
      vTaskResume(alnumHandle);
      timer_start(TIMER_GROUP_0, TIMER_0);

      vTaskSuspend(servoHandle);  
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  I2C Functions //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Taken from https://github.com/BU-EC444/code-examples/tree/master/i2c-display

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
  if (err == ESP_OK) {printf("- initialized: yes\n\n");}

  // Dat in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Display Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Taken from https://github.com/BU-EC444/code-examples/tree/master/i2c-display

// Turn on oscillator for alpha display
int alpha_oscillator() {
  SLAVE_ADDR = 0x70; 
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Alnum Display (Note: No longer used as a task for purpose of synchonicity)
static void task_Alnum_Disp(){
  while (1){
    int ret;
    
    char disp[20];

    /* Set up routines */
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to disp in HHMM format if hr>0
    
    if((sec-(sec%(60*60)))/(60*60))
      sprintf(disp, "%02d%02d", (sec-(sec%(60*60)))/(60*60), (sec%(60*60))/60);
    else 
      sprintf(disp, "%02d%02d", (sec%(60*60)/60), (sec%(60*60))%60);

    
    // Write characters to displaybuffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = alphafonttable[ (int)disp[0] ];
    displaybuffer[1] = alphafonttable[ (int)disp[1] ];
    displaybuffer[2] = alphafonttable[ (int)disp[2] ];
    displaybuffer[3] = alphafonttable[ (int)disp[3] ];

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
      i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);

    printf("- second: %d\n", sec);
    printf("- display: %c%c%c%c\n\n", disp[0], disp[1], disp[2], disp[3]);

    //Make sure alnumHandle doesn't run UNLESS it is right after sec is decremented by timer event
    vTaskResume(accelHandle);
    vTaskSuspend(alnumHandle);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADXL343 Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
            
    int ret=0;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
     return ret;


     
}

// Read register
uint8_t readRegister(uint8_t reg) {
     uint8_t data=0;

      // Send commands characters to display over I2C
 

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;

     }


// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    uint8_t data[2]={0,0};
    uint16_t data16=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_read_byte(cmd, data+1, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
        
    data16=data[1];
    data16 = data16 << 8;
    data16 |= data[0];
   // printf("%x \n",data16);
    return (int16_t) data16;

 }

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

/* Function to initialize accelerometer */
void init_accel(void){

    SLAVE_ADDR = ADXL343_ADDRESS; // 0x53
    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        // printf("\n>> Found ADAXL343, Device ID  is %x\n",deviceID);
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);

    // Display range
    // printf  ("- Range:         +/- ");
    // switch(getRange()) {
    // case ADXL343_RANGE_16_G:
    //     printf  ("16 ");
    //     break;
    // case ADXL343_RANGE_8_G:
    //     printf  ("8 ");
    //     break;
    // case ADXL343_RANGE_4_G:
    //     printf  ("4 ");
    //     break;
    // case ADXL343_RANGE_2_G:
    //     printf  ("2 ");
    //     break;
    // default:
    //     printf  ("?? ");
    //     break;
    // }
    // printf(" g\n");

    // // Display data rate
    // printf ("- Data Rate:    ");
    // switch(getDataRate()) {
    //     case ADXL343_DATARATE_3200_HZ:
    //         //printf  ("3200 ");
    //         break;
    //     case ADXL343_DATARATE_1600_HZ:
    //         //printf  ("1600 ");
    //         break;
    //     case ADXL343_DATARATE_800_HZ:
    //         //printf  ("800 ");
    //         break;
    //     case ADXL343_DATARATE_400_HZ:
    //         //printf  ("400 ");
    //         break;
    //     case ADXL343_DATARATE_200_HZ:
    //         //printf  ("200 ");
    //         break;
    //     case ADXL343_DATARATE_100_HZ:
    //         //printf  ("100 ");
    //         break;
    //     case ADXL343_DATARATE_50_HZ:
    //         //printf  ("50 ");
    //         break;
    //     case ADXL343_DATARATE_25_HZ:
    //         //printf  ("25 ");
    //         break;
    //     case ADXL343_DATARATE_12_5_HZ:
    //         printf  ("12.5 ");
    //         break;
    //     case ADXL343_DATARATE_6_25HZ:
    //         printf  ("6.25 ");
    //         break;
    //     case ADXL343_DATARATE_3_13_HZ:
    //         printf  ("3.13 ");
    //         break;
    //     case ADXL343_DATARATE_1_56_HZ:
    //         printf  ("1.56 ");
    //         break;
    //     case ADXL343_DATARATE_0_78_HZ:
    //         printf  ("0.78 ");
    //         break;
    //     case ADXL343_DATARATE_0_39_HZ:
    //         printf  ("0.39 ");
    //         break;
    //     case ADXL343_DATARATE_0_20_HZ:
    //         printf  ("0.20 ");
    //         break;
    //     case ADXL343_DATARATE_0_10_HZ:
    //         printf  ("0.10 ");
    //         break;
    //     default:
    //         printf  ("???? ");
    //         break;
    // }
    // printf(" Hz\n\n");

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);

}


// Alnum Display (Note: No longer used as a task for purpose of synchonicity)
static void task_Accel(){
  while (1){ 

    safeToAlnum = 0;

    /* Set up routines */
    // Turn on alpha oscillator
    init_accel();

    // Calculate all values for acceleration, which will then be used in payload task
    xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

    safeToAlnum = 1;

    roll = atan2(yp,zp)*57.3;
    negateX = -1 * xp;
    pitch = atan2(negateX,sqrt(yp*yp+zp*zp)) * 57.3;

    // printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", xp, yp, zp);
    // printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);


    if (abs(pitch) > 30 || abs(roll) > 30)
    {
      alert = 1;
      printf("Alert triggered\n");
    }

    if (alert == 1)
    {
      printf("Alert not yet addressed\n");
    }

    vTaskSuspend(accelHandle);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer Stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Taken from https://github.com/BU-EC444/code-examples/tree/master/timer-example

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
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
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Timer task. For every second, the queue receives the flag and then decrements the timer for the servo/display to do their things.
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);    // Timer

        // Decrement / Wraparound when triggered
        if (evt.flag == 1) {
            //When time is 0

            if (masterFan)
            {
              runningFan = 1;
              vTaskSuspend(alnumHandle);
              vTaskResume(servoHandle);
              reset_timer = 1;
              timer_pause(TIMER_GROUP_0, TIMER_0);
            }
            else if (tempFan)
            {
              runningFan = 1;
              vTaskSuspend(alnumHandle);
              vTaskResume(servoHandle);
              reset_timer = 1;
              timer_pause(TIMER_GROUP_0, TIMER_0);
            }
            else
            {            
              if (reset_timer)
              {
                reset_timer = 0;
                sec = TIMER_HOURS * 60 * 60 + TIMER_MINUTES * 60;
                vTaskSuspend(servoHandle);
                vTaskResume(alnumHandle);
              }
              else if(sec == 0){
                  //Run the servo
                  runningFan = 1;
                  vTaskResume(servoHandle);

                  //Stop running the display so it doesn't update after sec is reset
                  vTaskSuspend(alnumHandle);
                  sec = TIMER_HOURS * 60 * 60 + TIMER_MINUTES * 60;

                  //Pause the timer while the servo runs
                  timer_pause(TIMER_GROUP_0, TIMER_0);
              }
              else
              {
                  sec--;
                  //Make sure servo doesn't run while it isn't needed
                  vTaskSuspend(servoHandle);
                  vTaskResume(alnumHandle);
                  // //Let servo run ONLY after sec is decremented
                  // if (safeToAlnum == 1)
                  // {
                  //   vTaskResume(alnumHandle);
                  // }
                  // else
                  // {
                  //   printf("Not safe to alnum");
                  // }    
              }
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Function to initialize LEDs */
void init_LED(){
    gpio_pad_select_gpio(GPIO_ONBOARD_LED);
    gpio_set_direction(GPIO_ONBOARD_LED,GPIO_MODE_OUTPUT);
}

/* LED task to blink onboard LED based on ID */
void id_task(){
  while(1) {
    for (int i = 0; i < (int) ID; i++) {
      gpio_set_level(GPIO_ONBOARD_LED,1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(GPIO_ONBOARD_LED,0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Initializes all values and then creates persistent tasks for each piece of hardware.
void app_main(void) {

    /********** Initialization **********/
    // Initialize Wifi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Initialize PWM
    servo_init();

    // Initialize i2c 
    i2c_example_master_init();
    i2c_scanner();

    // Initialize LED
    init_LED();

    // Create a FIFO queue for timer-based
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));


    /********** Tasks **********/

    // Create tasks to handle timer-based events. Timer itself. Servo task. Display updating.
    xTaskCreate(task_Alnum_Disp, "task_Alnum_disp", 2048, NULL, 6, &alnumHandle);
    xTaskCreate(task_servo_wiggle, "task_servo_wiggle", 2048, NULL, 4, &servoHandle);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 7, &timerHandle);
    xTaskCreate(task_Accel, "task_Accel", 2048, NULL, 5, &accelHandle);

    // Enable Wifi
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 16, NULL);

    // Receive temperature status from esp_2
    xTaskCreate(temp_receive_task, "temp_receive", 4096, (void*)AF_INET, 15, NULL);

    // Power onboard LED
    xTaskCreate(id_task, "set_id_task", 1024*2, NULL, 1, NULL);

    // Initiate alarm using timer API
    alarm_init();
}
