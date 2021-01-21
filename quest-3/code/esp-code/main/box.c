/* EC444 Quest03
*  Tactile Internet
*  October 19, 2020
*  Authors: Tony Faller, Roger Ramesh, Shaivya Gupta */

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

/* Define macros for ADC */
#define DEFAULT_VREF    1100        // Default ADC reference voltage in mV

/* Define macros for timer */
#define TIMER_DIVIDER         16                                    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)      // to seconds
#define TIMER_INTERVAL_SEC    (0.1)                                 // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1                                     // Testing will be done with auto reload
#define TIMER_HOURS           0                                     // Timer hours unit, change to change feeding interval
#define TIMER_MINUTES         1                                     // Timer minutes unit, change to change feeding interval

/* Define macros for i2c */
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
// ADXL343 Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53
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
    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        printf("\n>> Found ADAXL343, Device ID  is %x\n",deviceID);
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    // Set range
    setRange(ADXL343_RANGE_16_G);

    // Display range
    printf  ("- Range:         +/- ");
    switch(getRange()) {
    case ADXL343_RANGE_16_G:
        printf  ("16 ");
        break;
    case ADXL343_RANGE_8_G:
        printf  ("8 ");
        break;
    case ADXL343_RANGE_4_G:
        printf  ("4 ");
        break;
    case ADXL343_RANGE_2_G:
        printf  ("2 ");
        break;
    default:
        printf  ("?? ");
        break;
    }
    printf(" g\n");

    // Display data rate
    printf ("- Data Rate:    ");
    switch(getDataRate()) {
        case ADXL343_DATARATE_3200_HZ:
            printf  ("3200 ");
            break;
        case ADXL343_DATARATE_1600_HZ:
            printf  ("1600 ");
            break;
        case ADXL343_DATARATE_800_HZ:
            printf  ("800 ");
            break;
        case ADXL343_DATARATE_400_HZ:
            printf  ("400 ");
            break;
        case ADXL343_DATARATE_200_HZ:
            printf  ("200 ");
            break;
        case ADXL343_DATARATE_100_HZ:
            printf  ("100 ");
            break;
        case ADXL343_DATARATE_50_HZ:
            printf  ("50 ");
            break;
        case ADXL343_DATARATE_25_HZ:
            printf  ("25 ");
            break;
        case ADXL343_DATARATE_12_5_HZ:
            printf  ("12.5 ");
            break;
        case ADXL343_DATARATE_6_25HZ:
            printf  ("6.25 ");
            break;
        case ADXL343_DATARATE_3_13_HZ:
            printf  ("3.13 ");
            break;
        case ADXL343_DATARATE_1_56_HZ:
            printf  ("1.56 ");
            break;
        case ADXL343_DATARATE_0_78_HZ:
            printf  ("0.78 ");
            break;
        case ADXL343_DATARATE_0_39_HZ:
            printf  ("0.39 ");
            break;
        case ADXL343_DATARATE_0_20_HZ:
            printf  ("0.20 ");
            break;
        case ADXL343_DATARATE_0_10_HZ:
            printf  ("0.10 ");
            break;
        default:
            printf  ("???? ");
            break;
    }
    printf(" Hz\n\n");

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thermistor Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for thermistor reading */
static esp_adc_cal_characteristics_t *adc_chars_thermistor;         // Pointer to empty structure used to store thermistor ADC characteristics
static const adc_channel_t channel_thermistor = ADC_CHANNEL_6;      // Thermistor uses GPIO34 (A2)
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_11;        // Thermistor uses attenuation 11
static const adc_unit_t unit_thermistor = ADC_UNIT_1;               // Characterize ADC1

/* Calculating temperature terms */ 
float B = 3435;
float T0 = 298.15;
float R0 = 10000;
float R;
float R2 = 10000;
float Vin_therm = 3300;

/* Function to initialize thermistor ADC pin */
void init_thermistor(void){
    // Configure for ADC 
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(channel_thermistor, atten_thermistor);    

    // Characterize ADC 
    adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);
}

 /* Function to read sensor */
float read_thermistor(){
    // Reconfigure ADC 
    esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);

    // Read from ADC pin 
    uint32_t reading_therm = adc1_get_raw((adc1_channel_t)channel_thermistor);
    float voltage_therm = esp_adc_cal_raw_to_voltage(reading_therm,adc_chars_thermistor);

    // Calculate resistance of thermistor 
    R = R2 * ( (Vin_therm/voltage_therm) - 1);

    // Calculate temperature without multisampling 
    float temp_therm = pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );
    temp_therm -= 273.15;

    // Return
    return temp_therm;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Battery Voltage Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for voltage reading */
static esp_adc_cal_characteristics_t *adc_chars_voltmeter;          // Pointer to empty structure used to store voltmeter ADC characteristics
static const adc_channel_t channel_voltmeter = ADC_CHANNEL_3;       // Voltmeter uses GPIO39 (A3)
static const adc_atten_t atten_voltmeter = ADC_ATTEN_DB_11;         // Voltmeter uses attenuation 11
static const adc_unit_t unit_voltmeter = ADC_UNIT_1;                // Characterize ADC1

/* Function to initialize voltmeter ADC pin */
void init_voltmeter(void){
    // Configure for ADC 
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(channel_voltmeter, atten_voltmeter);  

    // Characterize ADC 
    adc_chars_voltmeter = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit_voltmeter, atten_voltmeter, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_voltmeter);
}

/* Function to read voltmeter */
int read_voltmeter(){
    // Reconfigure ADC
    esp_adc_cal_characterize(unit_voltmeter, atten_voltmeter, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_voltmeter);

    // Read from ADC pin
    uint32_t reading_voltmeter = adc1_get_raw((adc1_channel_t)channel_voltmeter);
    uint32_t voltage_voltmeter = esp_adc_cal_raw_to_voltage(reading_voltmeter,adc_chars_voltmeter);

    // Return double voltage read assuming divider circuit is two 10k resistors
    return voltage_voltmeter * 2;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility functions to convert readings to string  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://www.geeksforgeeks.org/convert-floating-point-number-string/ */

// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 

// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating-point/double number to a string. 
void ftoa(float n, char* res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i] = '.'; // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LED Control Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Define macro for LED control */
#define LED0_GPIO 32

/* ON bit */
int ledON = 0;

/* Function to initialize LED GPIO */  
void init_led(){
  // Configure GPIO
  gpio_pad_select_gpio(LED0_GPIO);
  gpio_set_direction(LED0_GPIO, GPIO_MODE_OUTPUT);
}

/* Function to change LED state */
void toggle_led(int ONbit){
  gpio_set_level(LED0_GPIO, ONbit);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UDP Communication Stuff /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client */

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "hurricane box";


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

        int counter = 0;
        float thermistor = 0;

        while (1) {

            // Reset payload
            char payload[200];
            // memset(payload, '\0', sizeof(payload));

            // Calcualte accelerometer values
            xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
            roll = atan2(yp,zp)*57.3;
            negateX = -1 * xp;
            pitch = atan2(negateX,sqrt(yp*yp+zp*zp)) * 57.3;

            // Load payload with sensor values      https://www.techonthenet.com/c_language/standard_library_functions/string_h/strcat.php
            // char load_therm[20], load_volt[20], load_xp[20], load_yp[20], load_zp[20], load_roll[20], load_pitch[20];

            for (int i =0; i < 5; i++)
            {
                thermistor += read_thermistor();
                usleep(200000);
            }

            //ftoa(thermistor/5, load_therm, 2);
            //intToStr(read_voltmeter(), load_volt, 4);
            //ftoa(xp, load_xp, 2);
            //ftoa(yp, load_yp, 2);
            //ftoa(zp, load_zp, 2);
            //ftoa(roll, load_roll, 2);
            //ftoa(pitch, load_pitch, 2);

            sprintf(payload, "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f", thermistor/5, read_voltmeter(), xp, yp, zp, roll, pitch);
            thermistor = 0;

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
                  ledON ^= 1; // Toggle ON bit
                  toggle_led(ledON);
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

        counter++;
    }
    vTaskDelete(NULL);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void app_main(void){



    // Initialize ADC pins
    init_thermistor();
    init_voltmeter();

    // Initialize i2c
    i2c_example_master_init();
    i2c_scanner();

    // Initialize acccelerometer
    init_accel();

    // Initialize Wifi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize LED GPIO
    init_led();

    

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // // Test LED toggle
    // while(1){
    //   ledON ^= 1;
    //   printf("Toggling with ON bit = %d\n", ledON);
    //   toggle_led(ledON);
    //   vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }


    // Create UDP server
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

}