/* EC444 Quest05 Cruise Control
*  Novermber 25, 2020
*  Author: Tony Faller  */

// Standard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <sys/param.h>

// Motor control 
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// Ultrasonic 
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "sdkconfig.h"

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
//#include "addr_from_stdin.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Define and Initialize stuff for encoder (ADC) */
#define DEFAULT_VREF    1100        // Default ADC reference voltage in mV

static esp_adc_cal_characteristics_t *adc_chars;        // Pointer to empty structure used to store ADC characteristics
static const adc_channel_t channel = ADC_CHANNEL_3;     // GPI39 (A3) to use ADC1
static const adc_atten_t atten = ADC_ATTEN_DB_11;       // Attenuation to characterize, changed from 0 to 11 to allow for a higher range
static const adc_unit_t unit = ADC_UNIT_1;              // ADC to characterize (ADC_UNIT_1 or ADC_UNIT_2)
float speed;    // Speed of car from encoder


/* Define stuff for motor control */
#define GPIO_PWM0A_OUT 26   //Set GPIO 26 (A0) as PWM0A (EN1-2)
#define GPIO_PWM0B_OUT 14   //Set GPIO 14 as PWM0B (EN3-4)

#define GPIO_IN1	25		// Set GPIO 25 (A1) as IN1
#define GPIO_IN2	21		// Set GPIO 21 as IN2
#define GPIO_IN3	12		// Set GPIO 12 as IN3
#define GPIO_IN4	32		// Set GPIO 32 as IN4


/* Define stuff for ultrasonic sensor */
#define RMT_TX_CHANNEL    RMT_CHANNEL_1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  17                 /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    RMT_CHANNEL_0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  16                 /*!< GPIO number for receiver */

#define RMT_CLK_DIV      100                /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define HCSR04_MAX_TIMEOUT_US  25000        /*!< RMT receiver timeout value(us) */

#define US2TICKS(us)    (us / 10 * RMT_TICK_10_US)
#define TICKS2US(ticks) (ticks * 10 / RMT_TICK_10_US)


/* Stuff for PID LED control */
#define GPIO_RED_LED        27
#define GPIO_GREEN_LED      33
#define GPIO_BLUE_LED       15
#define GPIO_ONBOARD_LED    13

/* Setpoints */
#define SETPOINT_DISTANCE 20.0    // 20cm forward setpoint
uint32_t SETPOINT_WALL_MIN = -1;
uint32_t SETPOINT_WALL_MAX = 2147483647;

/* Stuff for 14-Segment Display */
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

/* Stuff for Master I2C */
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

/* Define macros for timer */
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (0.1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload
#define TIMER_HOURS           0     // Timer hours unit, change to change feeding interval
#define TIMER_MINUTES         1    // Timer minutes unit, change to change feeding interval

/* Handlers for each task */
TaskHandle_t timerHandle;
TaskHandle_t Collect_handle;

// //Index variable for csv (time in s from server start)
// uint32_t i=0;

/* Handler for encoder task */
TaskHandle_t encoder_read_handle;


/* Global state variables */
uint32_t direction = 1;     // 1 forward, 0 backwards
uint32_t stop = 0;          // 1 stop, 0 go
uint32_t masterStop = 0;    // Active-low enable bit for motors
uint32_t turning = 0;
float duty = 65.0;          // Initial duty cycle

float distance = 696969;

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
  0b0010010010001001, // }I
  0b0000010100100000, // ~
  0b0011111111111111,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UDP Communication Stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client */

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "purpleCar";


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
            char payload[] = "Hello!";
            // memset(payload, '\0', sizeof(payload));

            // Calcualte accelerometer values
            // xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            // yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            // zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            // roll = atan2(yp,zp)*57.3;
            // negateX = -1 * xp;
            // pitch = atan2(negateX,sqrt(yp*yp+zp*zp)) * 57.3;

            // Load payload with sensor values      https://www.techonthenet.com/c_language/standard_library_functions/string_h/strcat.php
            // char load_therm[20], load_volt[20], load_xp[20], load_yp[20], load_zp[20], load_roll[20], load_pitch[20];

            // for (int i =0; i < 5; i++)
            // {
            //     thermistor += read_thermistor();
            //     usleep(200000);
            // }

            //ftoa(thermistor/5, load_therm, 2);
            //intToStr(read_voltmeter(), load_volt, 4);
            //ftoa(xp, load_xp, 2);
            //ftoa(yp, load_yp, 2);
            //ftoa(zp, load_zp, 2);
            //ftoa(roll, load_roll, 2);
            //ftoa(pitch, load_pitch, 2);

            // sprintf(payload, "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f", thermistor/5, read_voltmeter(), xp, yp, zp, roll, pitch);
            // thermistor = 0;

            // strcpy(payload, load_therm);    
            // strcat(payload, ",");
            // strcat(payload, load_volt);
            // strcat(payload, ",");
            // strcat(payload, load_xp);
            // strcat(payload, ",");
            // strcat(payload, load_yp);
            // strcat(payload, ",");
            // strcat(payload, load_zp);
            // strcat(payload, ",");
            // strcat(payload, load_roll);
            // strcat(payload, ",");
            // strcat(payload, load_pitch);


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

                // Toggle LED if HTML button is pressed
                if(len == 7){
                  masterStop ^= 1; // Toggle stop bit
                  printf(">> masterStop = %d\n", masterStop);
                  // toggle_led(ledON);
                }

                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        // counter++;
    }
    vTaskDelete(NULL);
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IR Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for IR reading */
float dist_ir_init = -1;
float dist_ir = 0;
uint32_t reading_ir = 0;
static esp_adc_cal_characteristics_t *adc_chars_ir;			// Pointer to empty structure used to store IR ADC characteristics
static const adc_channel_t channel_ir = ADC_CHANNEL_0; 	// IR sensor uses GPIO36 (A4)
static const adc_atten_t atten_ir = ADC_ATTEN_DB_11;		// IR sensor uses attenuation 11
static const adc_unit_t unit_ir = ADC_UNIT_1;				    // Characterize ADC1

//Counter for infrared readings
uint32_t ir_read_counter = 0;

/* Function to initialize IR ADC pin */
void init_ir(void){
	/* Configure for ADC */
	adc1_config_width(ADC_WIDTH_BIT_11);
	adc1_config_channel_atten(channel_ir, atten_ir);	

	/* Characterize ADC */
	adc_chars_ir = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit_ir, atten_ir, ADC_WIDTH_BIT_11, DEFAULT_VREF, adc_chars_ir);
}
void read_collect_task(void){

    float voltage;

    while(1){
      // Reset stuff
      voltage = 0;

      //init_ir();
        /* Configure for ADC */
	    adc1_config_width(ADC_WIDTH_BIT_11);
	    adc1_config_channel_atten(channel_ir, atten_ir);	

	/* Characterize ADC */
	adc_chars_ir = calloc(1, sizeof(esp_adc_cal_characteristics_t));

  		esp_adc_cal_characterize(unit_ir, atten_ir, ADC_WIDTH_BIT_11, DEFAULT_VREF, adc_chars_ir);
  		
      for(int i=0; i<50; i++){
      		reading_ir = adc1_get_raw((adc1_channel_t)channel_ir);
      		voltage += esp_adc_cal_raw_to_voltage(reading_ir,adc_chars_ir);
            //printf("Reading_IR: %d\n", reading_ir);
          vTaskDelay(10 / portTICK_PERIOD_MS);
  		}

      voltage /= 50;

  		//Convert voltage to distance in cm
  		dist_ir = 61.175 * pow((float)voltage/1000, -1.092);

      // Get initial distance, set setpoint
      if(dist_ir_init < 0){
          dist_ir_init = dist_ir;

          printf(">> Initial distance: %.2f\n", dist_ir_init);

          if(dist_ir_init < 25){
          SETPOINT_WALL_MIN = (int)floor(dist_ir_init / 2.0);
          SETPOINT_WALL_MAX = (int)floor(dist_ir_init + dist_ir_init / 2.0);
          }

          else{
          SETPOINT_WALL_MIN = (int)dist_ir_init - 10;
          SETPOINT_WALL_MAX = (int)dist_ir_init + 40;
          }

          printf(" - SETPOINT_WALL_MIN: %d\n", SETPOINT_WALL_MIN);
          printf(" - SETPOINT_WALL_MAX: %d\n\n", SETPOINT_WALL_MAX);

      }

  		//Increase counter for how many times IR has been read
  		// ir_read_counter++;

      // Print distance to console (for debugging)
      printf(">> IR Distance: %.2f cm\n", dist_ir);

      // Reset dist_ir
      //dist_ir = 0;

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// ////////////////////////////////////////////////////////////////////////////////
// // Timer Stuff /////////////////////////////////////////////////////////////////
// //   Taken from https://github.com/BU-EC444/code-examples/tree/master/timer-example

// // A simple structure to pass "events" to main task
// typedef struct {
//     int flag;     // flag for enabling stuff in main code
// } timer_event_t;

// // Initialize queue handler for timer-based events
// xQueueHandle timer_queue;

// // ISR handler
// void IRAM_ATTR timer_group0_isr(void *para) {

//     // Prepare basic event data, aka set flag
//     timer_event_t evt;
//     evt.flag = 1;

//     // Clear the interrupt, Timer 0 in group 0
//     TIMERG0.int_clr_timers.t0 = 1;

//     // After the alarm triggers, we need to re-enable it to trigger it next time
//     TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

//     // Send the event data back to the main program task
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }

// // Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
// static void alarm_init() {
//     /* Select and initialize basic parameters of the timer */
//     timer_config_t config;
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = TEST_WITH_RELOAD;
//     timer_init(TIMER_GROUP_0, TIMER_0, &config);

//     // Timer's counter will initially start from value below
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

//     // Configure the alarm value and the interrupt on alarm
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
//         (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

//     // Start timer
//     timer_start(TIMER_GROUP_0, TIMER_0);
// }

// // Timer task. For every second, the queue receives the flag and then decrements the timer for the servo/display to do their things.
// uint32_t counter = 0; //Keeps track of 1/10 of a second, everytime 

// static void timer_evt_task(void *arg) {
//     while (1) {
//         // Create dummy structure to store structure from queue
//         timer_event_t evt;

// 		// Transfer from queue
//         xQueueReceive(timer_queue, &evt, portMAX_DELAY);    // Timer
		
// 		//Start with each task suspended
// 		//vTaskSuspend(IR_handle);
// 		//vTaskSuspend(Thermistor_handle);
		
// 		vTaskSuspend(Collect_handle);

// 		//Once we count 10 readings of the hardware, display the results in a CSV format
// 		if (counter == 10)
// 		{
// 			//Divide by number of runs for each task. Divide by 1000 (mm to m) and by 100 (cm to m)
// 			printf("%d,%.2f\n", i++,  (dist_ir/(float)ir_read_counter) / 100.0);	// Convert distance to m
// 			//Reset distances and temperatures to be re-accumulated
// 			dist_ir = 0;
			
// 			//Reset counters for each task to 0 to be re-accumulated
// 			ir_read_counter = 0;
// 			counter = 0;
// 		}


//         //If timer event is not triggered, do nothing
// 		//If it is, run each of the hardware tasks
//         if (evt.flag == 1) {
// 				//vTaskResume(IR_handle);
// 				//vTaskResume(Thermistor_handle);
// 				vTaskResume(Collect_handle);
// 				counter++;
//         }
//         else
//         {
			
// 			//vTaskSuspend(IR_handle);
// 			//vTaskSuspend(Thermistor_handle);
// 			vTaskSuspend(Collect_handle);
//         }
        
//     }
// }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const char* NEC_TAG = "HCSR04";

static inline void set_item_edge(rmt_item32_t* item, int low_us, int high_us){
    item->level0 = 0;
    item->duration0 = US2TICKS(low_us);
    item->level1 = 1;
    item->duration1 = US2TICKS(high_us);
}

/* Function to initialize transmitter */
static void nec_tx_init(){
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;

    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 0;    // off

    rmt_tx.tx_config.idle_level = 0;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    ESP_ERROR_CHECK(rmt_config(&rmt_tx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));
}


/*Function to initialize receiver */
static void nec_rx_init(){
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = false;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = US2TICKS(HCSR04_MAX_TIMEOUT_US);
    ESP_ERROR_CHECK(rmt_config(&rmt_rx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, 1000, 0));
}


/* Function to receive data */
static void rx_task(){
    printf(">> Beginning rx task\n");

    printf(" - Initializing rx...\n");
    int channel = RMT_RX_CHANNEL;

    nec_rx_init();
    printf(" - rx initialization complete.\n");

    uint32_t rx_size = 0;
    RingbufHandle_t rb = NULL;
    rmt_item32_t *item = NULL;

    printf(">> Beginning rx reception...\n");
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);


    while (rb){
        // printf(">> RB condition met\n");

        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        // printf(" - Something has been received from ring buffer\n");

        if (item){
            // printf(" - Item condition met\n");
            for (int i=0; i<rx_size / sizeof(rmt_item32_t); ++i){
                ESP_LOGI(NEC_TAG, "RMT RCV -- %d:%d | %d:%d : %.1fcm",   // Log data to console
                        item[i].level0, TICKS2US(item[i].duration0),
                        item[i].level1, TICKS2US(item[i].duration1),
                        (float)TICKS2US(item[i].duration0) / 58.2);

                //Stop if sensor reads <20 cm
                distance = (float)TICKS2US(item[i].duration0)/58.2;

                if( distance < SETPOINT_DISTANCE){
                    stop = 1;
                    // printf(">> STOP\n");
                }
                else
                    stop = 0;
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void*) item);
        } 
        else {
            printf(">> Item condition failed\n");
            vTaskDelay(10);
//            break;
        }
    }

    vTaskDelete(NULL);
}

/* Structure containing transmission payload */
static const rmt_item32_t morse_esp[] = {
    // E : dot
    {{{ US2TICKS(20), 0, US2TICKS(180), 1 }}}, // dot
    {{{ 16, 0, 144, 1 }}}, // dot
    // // S : dot, dot, dot
    // {{{ 32767, 0, 32767, 1 }}}, // dot
    // {{{ 32767, 0, 32767, 1 }}}, // dot
    // {{{ 32767, 0, 32767, 1 }}}, // dot
    // {{{ 32767, 0, 32767, 1 }}}, // SPACE
    // // P : dot, dash, dash, dot
    // {{{ 32767, 0, 32767, 1 }}}, // dot
    // {{{ 32767, 0, 32767, 1 }}},
    // {{{ 32767, 0, 32767, 1 }}}, // dash
    // {{{ 32767, 0, 32767, 1 }}},
    // {{{ 32767, 0, 32767, 1 }}}, // dash
    // {{{ 32767, 0, 32767, 1 }}}, // dot
    // RMT end marker
    {{{ 0, 1, 0, 0 }}}
};

/* Function to send pulses */
static void tx_task(){
    vTaskDelay(10);
    printf(">> Beginning tx task\n");

    printf(" - Initializing tx...\n");
    nec_tx_init();

    int channel = RMT_TX_CHANNEL;
    printf(" - tx initialization complete.\n");

    int item_num = 1;
    rmt_item32_t item[item_num];
    for (int i=0; i<item_num; ++i)
        set_item_edge(&item[i], 20, 180);
//  set_item_edge(&item[1], factor * 70, factor * 30);
    printf(" - Item creation complete.\n");
    printf(">> Sending tx data...\n");

    for (;;)
    {
        ESP_LOGI(NEC_TAG, "RMT TX DATA");

        // To send data according to the waveform items.
        rmt_write_items(channel, morse_esp, sizeof(morse_esp) / sizeof(morse_esp[0]), true);
        // Wait until sending is done.
        rmt_wait_tx_done(channel, portMAX_DELAY);
        // before we free the data, make sure sending is already done.

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/* Function to create read/send tasks */
void hcsr04_init(){
    xTaskCreate(rx_task, "rx_task", 2048, NULL, 10, NULL);
    xTaskCreate(tx_task, "tx_task", 2048, NULL, 11, NULL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Optical Encoder Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


esp_adc_cal_value_t val_type;

/* Function to initialize ADC for optical encoder */
void encoder_init(){
	    /* Configure for ADC1 */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    /* Characterize ADC */
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

}

/* Set of states (black/white) */
typedef enum {
	STATE_BLACK,
	STATE_WHITE
} state_e;

/* Task to read optical encoder & calculate speed*/
static void encoder_read_task(){

	// Initial states of the wheels
	state_e currState = STATE_BLACK;
	state_e oldState = currState;
    
    // Revolution counter
	uint32_t revolutionCount = 0;
    uint32_t adc_reading = 0;
    uint32_t voltage = 0;

	while(1){
        
        	    /* Configure for ADC1 */
                adc1_config_width(ADC_WIDTH_BIT_12);
                adc1_config_channel_atten(channel, atten);

                /* Characterize ADC */
                adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
                val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        revolutionCount = 0;

        if(voltage < 240)
        {
	       	currState = STATE_WHITE;
            oldState = currState;
        }
        else
        {
	    	currState = STATE_BLACK;
            oldState = currState;
        }

		// Sample for 1 second
		for(int i=0; i<100; i++){
	        
	        adc_reading = adc1_get_raw((adc1_channel_t)channel);

	        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

	        // Print to console (for debugging)
	        // printf("Raw: %d\tVoltage: %d\tColor: ", adc_reading, voltage);
	       	// if(voltage < 240) printf("white\n");
	       	// else printf("black\n");

	       	// Change state based on voltage
	       	if(voltage < 240)
	       		currState = STATE_WHITE;
	       	else
	       		currState = STATE_BLACK;

	       	// Increment rotation counter if state changed
	       	if(currState != oldState){
	       		revolutionCount++;
	       		oldState = currState;
	       	}

	        // Wait 10ms before next sample
	        vTaskDelay(10 / portTICK_PERIOD_MS);
	    }

	    // Calculate speed in m/s based on revolution counter
        speed = ( (float)revolutionCount * 3.14 * .03) / (1*6); // 2 * pi * r / t; t = 0.5 s, pi = 3.14, r = .03 m

      // Calibration
        if(speed < 0.1) {speed = 0.1;}
        if(speed > 0.4) {speed = 0.4;}

        //PID for speed
        if (speed > 0.35)
        {
            duty -= 5;
        }
        if (speed < 0.15)
        {
            duty += 5;
        }

	    // Calibration
	    // if(speed < 0.4) {speed = 0;}

	    // Print speed to console
	    // printf("revolutionCount: %d\tCurrent speed: %.2f m/s\n", revolutionCount, speed);

	    // Print velocity to console
	    // printf(" - Current velocity: ");
	    // if(direction) {printf("+");}
	    // else {printf("-");}
	    // printf("%.2f m/s\n", speed);

    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor control stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Function to initialize MCPWM and H-Bridge GPIO */
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);

    gpio_pad_select_gpio(GPIO_IN1);
    gpio_pad_select_gpio(GPIO_IN2);
    gpio_pad_select_gpio(GPIO_IN3);
    gpio_pad_select_gpio(GPIO_IN4);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN4, GPIO_MODE_OUTPUT);
}

/* Function to move motors forward with duty cycle = duty % */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
	gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
    gpio_set_level(GPIO_IN3, 1);
    gpio_set_level(GPIO_IN4, 0);

    // Set direction
    direction = 1;

    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state

}

/* Function to turn left with duty cycle = duty % */
static void brushed_motor_forward_right(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
	gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
    gpio_set_level(GPIO_IN3, 1);
    gpio_set_level(GPIO_IN4, 0);

    // Set direction
    direction = 1;

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    // mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    // mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state

}

/* Function to turn right with duty cycle = duty % */
static void brushed_motor_forward_left(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
	gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 1);
    gpio_set_level(GPIO_IN3, 1);
    gpio_set_level(GPIO_IN4, 0);

    // Set direction
    direction = 1;

    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    // mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    // mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state

}

/* Function to move backward with duty cycle = duty % */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    gpio_set_level(GPIO_IN1, 1);
    gpio_set_level(GPIO_IN2, 0);
    gpio_set_level(GPIO_IN3, 0);
    gpio_set_level(GPIO_IN4, 1);

    // Set direction
    direction = 0;

    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state


    // mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/* Function to stop */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{

	gpio_set_level(GPIO_IN1, 0);
    gpio_set_level(GPIO_IN2, 0);
    gpio_set_level(GPIO_IN3, 0);
    gpio_set_level(GPIO_IN4, 0);

    // Set direction
    direction = 1;

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}



/* Example task to run through all motions */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    printf(">> Stop...\n\n");
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    vTaskDelay(500 / portTICK_RATE_MS);

    printf(">> Forward...\n");
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 60.0);
    vTaskDelay(500 / portTICK_RATE_MS);

    // printf(">> Stop...\n\n");
    // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // vTaskDelay(100 / portTICK_RATE_MS);
              	
    while (1) {
    	// Resume speed readings
    	// vTaskResume(encoder_read_handle);
      if(!masterStop){
          if(!stop){

              if(dist_ir < SETPOINT_WALL_MIN){
                  turning = 1;

                  vTaskSuspend(encoder_read_handle);
                  
                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(500 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(500 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(350 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  vTaskResume(encoder_read_handle);
                  //vTaskDelay(500 / portTICK_RATE_MS);
              }
              else if(dist_ir > SETPOINT_WALL_MAX){
                  turning = 1;

                  vTaskSuspend(encoder_read_handle);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(500 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(500 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);

                  brushed_motor_forward_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
                  vTaskDelay(350 / portTICK_RATE_MS);

                  brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                  vTaskDelay(300 / portTICK_RATE_MS);
                  
                  vTaskResume(encoder_read_handle);
                  //vTaskDelay(500 / portTICK_RATE_MS);
              }
              else{
                  turning = 0;

                  vTaskResume(encoder_read_handle);

                  printf(">> Forward...\n");
                  brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, duty);
                  vTaskDelay(400 / portTICK_RATE_MS);
              }
              // printf(">> Backward...\n");
              // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
              // vTaskDelay(2000 / portTICK_RATE_MS);
          }

          // printf(">> Right...\n");
          // brushed_motor_forward_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
          // vTaskDelay(1000 / portTICK_RATE_MS);

          // printf(">> Left...\n");
          // brushed_motor_forward_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
          // vTaskDelay(1000 / portTICK_RATE_MS);

          // printf(">> Backward...\n");
          // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 75.0);
          // vTaskDelay(2000 / portTICK_RATE_MS);

          // printf(">> Forward faster...\n");
          // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 85.0);
          // vTaskDelay(2000 / portTICK_RATE_MS);

          else{
              turning = 0;
              printf(">> Stop...\n\n");
              brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
              vTaskDelay(2000 / portTICK_RATE_MS);
          }
      }

      else{
        turning = 0;
        printf(">> MASTER STOP\n\n");
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
      }

        //Suspend speed readings
        // vTaskSuspend(encoder_read_handle);

    }
}

int offbit = 0;
/* Function to change motor state */
void stop_motor(int offbit){
  	gpio_set_level(GPIO_IN1, offbit);
    gpio_set_level(GPIO_IN2, offbit);
    gpio_set_level(GPIO_IN3, offbit);
    gpio_set_level(GPIO_IN4, offbit);

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    // if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    // if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    // if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to disp in HHMM format if hr>0
    
    // sprintf(disp, "%.2f", speed);

    sprintf(disp, "%.2f", dist_ir);

    
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

  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Initialization function for LEDs */
static void LED_init(){
    // Select GPIO
    gpio_pad_select_gpio(GPIO_RED_LED);
    gpio_pad_select_gpio(GPIO_GREEN_LED);
    gpio_pad_select_gpio(GPIO_BLUE_LED);
    gpio_pad_select_gpio(GPIO_ONBOARD_LED);

    // Set as outputs
    gpio_set_direction(GPIO_RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BLUE_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ONBOARD_LED, GPIO_MODE_OUTPUT);

}

/* Task to change LEDs based on distance */
static void LED_task(){
    while(1){
        if(stop || masterStop){
            gpio_set_level(GPIO_RED_LED, 1);
            gpio_set_level(GPIO_GREEN_LED, 0);
            gpio_set_level(GPIO_BLUE_LED, turning);
        }

        else{
            gpio_set_level(GPIO_RED_LED, 0);
            gpio_set_level(GPIO_GREEN_LED, 1);
            gpio_set_level(GPIO_BLUE_LED, turning);
        }

        gpio_set_level(GPIO_ONBOARD_LED, masterStop);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void){

    /********** Initialization **********/
    // Initialize Wifi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

  	// Initialize encoder and pcnt
  	encoder_init();

    // Initialize IR sensor
    init_ir();

    // Initialize ultrasonic sensor
    hcsr04_init();

    // Initialize LEDs
    LED_init();

    // Initialize i2c
    i2c_example_master_init();
    i2c_scanner();


    /********** Tasks **********/
    // Continuously sample ADC1 for encoder
    xTaskCreate(encoder_read_task, "encoder_read", 4096, NULL, 14, &encoder_read_handle);

    // Continuously sample ADC1 for IR
    xTaskCreate(read_collect_task, "read_collect", 4096, NULL, 15, NULL);

    // Enable Wifi
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 16, NULL);

    // Power LEDs
    xTaskCreate(LED_task, "LED_task", 2048, NULL, 12, NULL);

    // Power display
    xTaskCreate(task_Alnum_Disp, "task_Alnum_Disp", 2048, NULL, 13, NULL);

    // Run motors
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 9, NULL);

}