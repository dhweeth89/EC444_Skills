/* EC444 Quest06 
*  ESP_2 Code
*  December 4, 2020
*  Authors: Tony Faller, Roger Ramesh */

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
#define ID 2

/* Define start signal for UDP */
#define UDP_start "Temperature"        // This string determines which CSV file the paylaod is appended to

/* Define esp_3 IP */
#define ESP3_IP_ADDR "192.168.86.248"

/* Define stuff for ADC */
#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          // Multisampling

/* Global variables for calculating temperature */
float B = 3435;
float T0 = 298.15;
float R0 = 10000;
float R;
float R2 = 10000;
float voltage;
uint32_t reading_probe = 0;	
float count = 1;		

/* Stuff for PID LED control */
#define GPIO_RED_LED 12     // RED
#define GPIO_GREEN_LED 27   // Green
#define GPIO_ONBOARD_LED  13

/* Define stuff for thermal probe */
static esp_adc_cal_characteristics_t *adc_chars_probe;
static const adc_channel_t channel_probe = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten_probe = ADC_ATTEN_DB_11;
static const adc_unit_t unit_probe = ADC_UNIT_1;
float temp_probe;
float temp_probe_sum = 0;
float temp_probe_avg = 0;               // ADC reading of probe

/* Define stuff for thermistor */
static esp_adc_cal_characteristics_t *adc_chars_thermistor;			// Pointer to empty structure used to store thermistor ADC characteristics
static const adc_channel_t channel_thermistor = ADC_CHANNEL_3; 		// Thermistor uses GPIO39 (A3)
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_11;		// Thermistor uses attenuation 11
static const adc_unit_t unit_thermistor = ADC_UNIT_1;				// Characterize ADC1
float Vin_therm = 5000;
uint32_t reading_therm = 0;                     // ADC reading of thermistor
float temp_therm;
float temp_therm_sum = 0;
float temp_therm_avg = 0;

/* Setpoints */
#define SETPOINT_TEMPERATURE_MAX 20.0    // 20 C max setpoint
#define SETPOINT_TEMPERATURE_MIN 10.0    // 10 C min setpoint
uint8_t badTemp = 0;                    // Flag to show if temperature is good or not


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

static const char *TAG = "ESP_2";
static const char *TAG2 = "ESP2->ESP3";


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

            sprintf(payload, "%s,%.2f,%.2f", UDP_start, temp_therm_avg, temp_probe_avg);
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

            // // Reset state change flag
            // stateChange = 0;

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


/* Task to send thermistor values to esp_3 */ 
static void temperature_send_task(void *pvParameters)
{
    // First FOB
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

// #ifdef CONFIG_EXAMPLE_IPV4
        // First FOB
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(ESP3_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT2);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        // First FOB
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG2, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG2, "Socket created, sending to %s:%d", ESP3_IP_ADDR, PORT2);

        while (1) {

            // Reset payload
            char payload[200];

            // Set payload
            sprintf(payload, "%s,%d", UDP_start, badTemp);
            //printf("leaderID = %c\n", payload[2]);    // For debugging

            // Send payload to esp_3
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG2, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG2, "Message sent: %s", payload);

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG2, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Temperature Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Function to initialize thermistor */
void init_thermistor(void){
  // Configure for ADC
  adc1_config_channel_atten(channel_thermistor, atten_thermistor);  

  // Characterize ADC 
  adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);
}

/* Function to intialize thermal probe */
void init_ad(void){
    //adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel_probe, atten_probe);
    adc_chars_probe = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    //adc2_config_channel_atten((adc2_channel_t)channel, atten);
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit_probe, atten_probe, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_probe);
}

/* Task to read ADC1 */
void read_ADC_task(){

    while(1){

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Probe /////////////////////////////////////////////////////////////////////////////////////////
    esp_adc_cal_characterize(unit_probe, atten_probe, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_probe);
    reading_probe = adc1_get_raw((adc1_channel_t)channel_probe);
    voltage = esp_adc_cal_raw_to_voltage(reading_probe,adc_chars_probe);

    R = R2 * ( (Vin_therm/voltage) - 1);

    temp_probe = pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );

    temp_probe -=273.15;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Thermistor ////////////////////////////////////////////////////////////////////////////////////
    esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);
    reading_therm = adc1_get_raw((adc1_channel_t)channel_thermistor);
    voltage = esp_adc_cal_raw_to_voltage(reading_therm,adc_chars_thermistor);

    R = R2 * ( (Vin_therm/voltage) - 1);

    temp_therm = pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );

    temp_therm -= 273.15; 

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Average ///////////////////////////////////////////////////////////////////////////////////////
    
    // Add current values to running sum of averages
    temp_therm_sum = temp_therm_sum + temp_therm;
    temp_probe_sum = temp_probe_sum + temp_probe;

    // Contiguous aggregate averaging
    temp_therm_avg = temp_therm_sum/count;
    temp_probe_avg = temp_probe_sum/count++;    // Continuous aggregate averaging

    // Calibration
    temp_therm_avg += 9;
    temp_probe_avg += 9;
    
    // Print to console
    // printf("count %f \n",count);
    printf("Probe: %.2fC\n",temp_probe_avg);
    printf("Therm: %.2fC\n\n",temp_therm_avg);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

  }


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Function to initialize LEDs */
void init_LED(){
    gpio_pad_select_gpio(GPIO_RED_LED);
    gpio_pad_select_gpio(GPIO_GREEN_LED);
    gpio_pad_select_gpio(GPIO_ONBOARD_LED);

    gpio_set_direction(GPIO_RED_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_GREEN_LED,GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ONBOARD_LED,GPIO_MODE_OUTPUT);
}

/* Function to change LEDs based on temperature */
void LED_task(){
  while(1){
    if (temp_probe_avg < SETPOINT_TEMPERATURE_MAX && temp_probe_avg >= SETPOINT_TEMPERATURE_MIN && temp_therm_avg < SETPOINT_TEMPERATURE_MAX && temp_probe_avg >= SETPOINT_TEMPERATURE_MIN){
      badTemp = 0;
      gpio_set_level(GPIO_GREEN_LED,1);
      gpio_set_level(GPIO_RED_LED,0);
    }
    else{
      badTemp = 1;
      gpio_set_level(GPIO_GREEN_LED,0);
      gpio_set_level(GPIO_RED_LED,1);
    }

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{

  /********** Initialization **********/
  // Initialize Wifi
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());

  // Configure bit width of ADC channel
  adc1_config_width(ADC_WIDTH_BIT_10);

  // Initialize LEDs
  init_LED();

  // Initialize thermistor
	init_thermistor();

  // Initialize thermal probe
  init_ad();
    

  /********** Tasks **********/

  // Enable Wifi
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 16, NULL);

  // Continuously sample ADC1 for thermistor and probe
  xTaskCreate(read_ADC_task, "read_ADC", 4096, NULL, 11, NULL);

  // Send temperature to esp_3
  xTaskCreate(temperature_send_task, "temperature_send", 4096, NULL, 10, NULL);

  // Power LEDs
  xTaskCreate(LED_task, "LED_task", 2048, NULL, 9, NULL);

  // Power onboard LED
  xTaskCreate(id_task, "set_id_task", 1024*2, NULL, 1, NULL);
    
}

