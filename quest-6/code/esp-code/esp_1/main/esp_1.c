/* EC444 Quest06 
*  ESP_1 Code
*  December 3, 2020
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

/* Define esp ID */
#define ID 1

/* Define start signal for UDP */
#define UDP_start "Presence"        // This string determines which CSV file the paylaod is appended to


/* Define and Initialize stuff for encoder (ADC) */
#define DEFAULT_VREF    1100        // Default ADC reference voltage in mV


/* Global Values for IR Reading */
// float dist_ir_init = -1;
float dist_ir = 0;
uint32_t reading_ir = 0;
static esp_adc_cal_characteristics_t *adc_chars_ir;				// Pointer to empty structure used to store IR ADC characteristics
static const adc_channel_t channel_ir = ADC_CHANNEL_0; 			// IR sensor uses GPIO36 (A4)
static const adc_atten_t atten_ir = ADC_ATTEN_DB_11;			// IR sensor uses attenuation 11
static const adc_unit_t unit_ir = ADC_UNIT_1;				    // Characterize ADC1

// /* Global values for thermistor reading */
// float temp_therm = 0;												// Temperature reading (C) of thermistor
// uint32_t reading_therm = 0;											// ADC reading of thermistor
// static esp_adc_cal_characteristics_t *adc_chars_thermistor;			// Pointer to empty structure used to store thermistor ADC characteristics
// static const adc_channel_t channel_thermistor = ADC_CHANNEL_3; 		// Thermistor uses GPIO39 (A3)
// static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_11;		// Thermistor uses attenuation 11
// static const adc_unit_t unit_thermistor = ADC_UNIT_1;				// Characterize ADC1


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

float dist_us = 696969;


/* Stuff for PID LED control */
#define GPIO_IR_RED_LED        	33
#define GPIO_IR_GREEN_LED      	27
#define GPIO_US_RED_LED		   	32
#define GPIO_US_GREEN_LED	   	15
#define GPIO_ONBOARD_LED    	13
uint8_t IR_goodD = 0;		// Good distance (green)
uint8_t IR_badD = 0;		// Bad Distance  (red)
uint8_t US_goodD = 0;
uint8_t US_badD = 0;


/* Setpoints */
#define SETPOINT_DISTANCE_MAX 45.0    	// 45cm max setpoint
#define SETPOINT_DISTANCE_MIN 35.0		// 35cm min setpoint
// uint32_t SETPOINT_WALL_MIN = -1;
// uint32_t SETPOINT_WALL_MAX = 2147483647;


/* Handler for IR task */
TaskHandle_t IR_handle;
TaskHandle_t Therm_handle;


/* Set of object proximity states */
typedef enum {
	STATE_PRESENT,
	STATE_ABSENT
} state_e;

// Initial states
state_e currState = STATE_ABSENT;
state_e oldState = STATE_ABSENT;

// Flag for state change
uint8_t stateChange = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UDP Communication Stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client */

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "ESP_1";


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
            // Only send data if object present
            if(IR_goodD && US_goodD && stateChange){
                // Reset payload
                char payload[200];
                // memset(payload, '\0', sizeof(payload));

                sprintf(payload, "%s,%.2f,%.2f", UDP_start, dist_ir, dist_us);
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
                }

                // Reset state change flag
                stateChange = 0;

                vTaskDelay(2000 / portTICK_PERIOD_MS);          // Adjust this to change rate of transmission
            }

            else{
                vTaskDelay(250 / portTICK_PERIOD_MS);
            }
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
// ADC Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Counter for infrared readings */
uint32_t ir_read_counter = 0;

/* Calculating temperature terms */
float B = 3435;
float T0 = 298.15;
float R0 = 10000;
float R;
float R2 = 10000;
float Vin_therm = 5000;

/* Counter for temperature readings */
uint32_t therm_read_counter = 0;


/* Function to initialize IR ADC pin */
void init_ir(void){
	/* Configure for ADC */
	adc1_config_width(ADC_WIDTH_BIT_11);
	adc1_config_channel_atten(channel_ir, atten_ir);	

	/* Characterize ADC */
	adc_chars_ir = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit_ir, atten_ir, ADC_WIDTH_BIT_11, DEFAULT_VREF, adc_chars_ir);
}

// /* Function to initialize thermistor ADC pin */
// void init_thermistor(void){
// 	/* Configure for ADC */
// 	//adc1_config_width(ADC_WIDTH_BIT_12);
// 	adc1_config_channel_atten(channel_thermistor, atten_thermistor);	

//	/* Characterize ADC */
// 	adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
// 	esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);
// }

/* Task to read ADC1 */
void read_ADC_task(){

    float voltage;
    // float temp_therm_test;

    while(1){

  //   	//////////////////////////////////////////////////////////////////////////////////////////////////
  //   	// Thermistor ////////////////////////////////////////////////////////////////////////////////////

  //   	// Reset temp_therm_test
		// temp_therm_test = 0;

		// // Configure ADC Channel
		// adc1_config_channel_atten(channel_thermistor, atten_thermistor);
		// esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);

		// for(int i=0; i<50; i++){
		// 	//Get the reading
		// 	reading_therm = adc1_get_raw((adc1_channel_t)channel_thermistor);
		// 	voltage = esp_adc_cal_raw_to_voltage(reading_therm,adc_chars_thermistor);
		// 	//printf("Voltage is: %.2f\n\n", voltage);

		// 	//Calculate Resistance of thermistor
		// 	R = R2 * ( (Vin_therm/(float)reading_therm) - 1);

		// 	//Calculate the temperature
		// 	temp_therm_test += pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );

		// 	// Wait between readings
		// 	vTaskDelay(10 / portTICK_PERIOD_MS);
		// }

		// //Add to runsum of temperature and ensure it's Celsius, not Kelvin
		// temp_therm = temp_therm_test / 50;
		// temp_therm -= 273.15; 

		// // Calibration
		// temp_therm = 0 - temp_therm + 10;

		// // Print to console
		// printf(">> Temperature: %.2f c\n", temp_therm);

		//////////////////////////////////////////////////////////////////////////////////////////////////
    	// IR ////////////////////////////////////////////////////////////////////////////////////////////

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
            // printf("Reading_IR: %d\n", reading_ir);
          vTaskDelay(10 / portTICK_PERIOD_MS);
  		}

      	voltage /= 50;

  		//Convert voltage to distance in cm
  		dist_ir = 61.175 * pow((float)voltage/1000, -1.092);

  		// Print distance to console (for debugging)
      	printf(">> IR Distance: %.2f cm\n", dist_ir);

  		// Set D values
  		if(dist_ir < SETPOINT_DISTANCE_MIN || dist_ir > SETPOINT_DISTANCE_MAX){
  			printf(" - Object not in good range\n\n");
  			IR_goodD = 0;
  			IR_badD = 1;
  		}
  		else{
  			printf(" - Object in good range\n\n");
  			IR_goodD = 1;
  			IR_badD = 0;
  		}

    }

}

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

                // Get distance in cm
                dist_us = (float)TICKS2US(item[i].duration0)/58.2;

                // Set D
                if(dist_us < SETPOINT_DISTANCE_MIN || dist_us > SETPOINT_DISTANCE_MAX){
                	US_goodD = 0;
                	US_badD = 1;
                }
                else{
                	US_goodD = 1;
                	US_badD = 0;
                }

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
    xTaskCreate(rx_task, "rx_task", 2048, NULL, 12, NULL);
    xTaskCreate(tx_task, "tx_task", 2048, NULL, 11, NULL);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Initialization function for LEDs */
static void LED_init(){
    // Select GPIO
    gpio_pad_select_gpio(GPIO_IR_RED_LED);
    gpio_pad_select_gpio(GPIO_IR_GREEN_LED);
    gpio_pad_select_gpio(GPIO_US_RED_LED);
    gpio_pad_select_gpio(GPIO_US_GREEN_LED);
    gpio_pad_select_gpio(GPIO_ONBOARD_LED);

    // Set as outputs
    gpio_set_direction(GPIO_IR_RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IR_GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_US_RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_US_GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ONBOARD_LED, GPIO_MODE_OUTPUT);

}

/* Task to change LEDs based on distance */
static void LED_task(){
    while(1){
        gpio_set_level(GPIO_IR_RED_LED, IR_badD);
        gpio_set_level(GPIO_IR_GREEN_LED, IR_goodD);
        gpio_set_level(GPIO_US_RED_LED, US_badD);
        gpio_set_level(GPIO_US_GREEN_LED, US_goodD);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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

/* Task to handle present vs absent states */
void state_task(){
    while(1){

        // Change state
        if(IR_goodD && US_goodD){
            currState = STATE_PRESENT;
        }
        else{
            currState = STATE_ABSENT;
        }

        // Raise flag if state changed
        if(currState != oldState){
            stateChange = 1;
            oldState = currState;
        }

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

    // Initialize IR sensor
    init_ir();

    // // Initialize thermistor
    // init_thermistor();

    // Initialize LEDs
    LED_init();


    /********** Tasks **********/

    // Enable Wifi
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 16, NULL);

    // Continuously sample ADC1 for IR and thermistor
    xTaskCreate(read_ADC_task, "read_ADC_task", 4096, NULL, 13, NULL);

    // Continuously sample ultrasonic sensor (priorities 12 and 11)
    hcsr04_init();

    // Detect if object in place or not
    xTaskCreate(state_task, "state_task", 4096, NULL, 10, NULL);

    // Power LEDs
    xTaskCreate(LED_task, "LED_task", 2048, NULL, 12, NULL);

    // Power onboard LED
    xTaskCreate(id_task, "set_id_task", 1024*2, NULL, 1, NULL);

}