/* EC444 Quest02
*  Tactile Internet
*  October 4, 2020
*  Author: Tony Faller, Shaivya Gupta, Roger Ramesh  */

/* Note: This code is a modified version of https://github.com/espressif/esp-idf/tree/39f090a4f1dee4e325f8109d880bf3627034d839/examples/peripherals/adc */

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
#include "esp_attr.h"
#include "sdkconfig.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define DEFAULT_VREF    1100        // Default ADC reference voltage in mV

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

//Index variable for csv (time in s from server start)
uint32_t i=0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IR Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for IR reading */
float dist_ir = 0;
uint32_t reading_ir = 0;
static esp_adc_cal_characteristics_t *adc_chars_ir;			// Pointer to empty structure used to store IR ADC characteristics
static const adc_channel_t channel_ir = ADC_CHANNEL_0; 		// IR sensor uses GPIO36 (A4)
static const adc_atten_t atten_ir = ADC_ATTEN_DB_11;		// IR sensor uses attenuation 11
static const adc_unit_t unit_ir = ADC_UNIT_1;				// Characterize ADC1

//Counter for infrared readings
uint32_t ir_read_counter = 0;

/* Function to initialize IR ADC pin */
void init_ir(void){
	/* Configure for ADC */
	//adc1_config_width(ADC_WIDTH_BIT_11);
	adc1_config_channel_atten(channel_ir, atten_ir);	

	/* Characterize ADC */
	adc_chars_ir = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit_ir, atten_ir, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_ir);
}

/* Function to read IR sensor */
// void read_ir_task(void){

// 	while (1) {
// 		init_ir();
		
// 		reading_ir = adc1_get_raw((adc1_channel_t)channel_ir);
// 		float voltage = esp_adc_cal_raw_to_voltage(reading_ir,adc_chars_ir);
		
// 		//Each time you sample IR reading, add to a running sum
// 		dist_ir += 61.175 * pow((float)voltage/1000, -1.092);

// 		//Increase counter for how many times IR has been read
// 		ir_read_counter++;

// 		//Shut task down till next timer event
// 		vTaskSuspend(IR_handle);
// 	}
// }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thermistor Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for thermistor reading */
float temp_therm = 0;												// Temperature reading (C) of thermistor
uint32_t reading_therm = 0;											// ADC reading of thermistor
static esp_adc_cal_characteristics_t *adc_chars_thermistor;			// Pointer to empty structure used to store thermistor ADC characteristics
static const adc_channel_t channel_thermistor = ADC_CHANNEL_3; 		// Thermistor uses GPIO39 (A3)
static const adc_atten_t atten_thermistor = ADC_ATTEN_DB_11;		// Thermistor uses attenuation 11
static const adc_unit_t unit_thermistor = ADC_UNIT_1;				// Characterize ADC1

/* Calculating temperature terms */
float B = 3435;
float T0 = 298.15;
float R0 = 10000;
float R;
float R2 = 10000;
float Vin_therm = 5000;

//Counter for temperature readings
uint32_t therm_read_counter = 0;


/* Function to initialize thermistor ADC pin */
void init_thermistor(void){
	/* Configure for ADC */
	//adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(channel_thermistor, atten_thermistor);	

	/* Characterize ADC */
	adc_chars_thermistor = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);
}

// /* Function to read sensor */
// void read_thermistor_task(void){
// 	//Added variable as intermediate for temperature calculation
// 	float temp_therm_test;

// 	while (1){
		
// 		init_thermistor();

// 		//Get the reading
// 		reading_therm = adc1_get_raw((adc1_channel_t)channel_thermistor);
// 		float voltage = esp_adc_cal_raw_to_voltage(reading_therm,adc_chars_thermistor);

// 		//printf("Voltage is: %.2f\n\n", voltage);
// 		//Calculate Resistance of thermistor
		
// 		R = R2 * ( (Vin_therm/(float)reading_therm) - 1);
		
// 		//R = R2 * ( (Vin_therm/voltage) - 1);

// 		//Calculate the temperature
// 		temp_therm_test = pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );

// 		//Add to runsum of temperature and ensure it's Celsius, not Kelvin
// 		temp_therm += temp_therm_test;
// 		temp_therm -= 273.15; 

// 		//Increase the counter that tracks how many times thermistor is read
// 		therm_read_counter++;

// 		vTaskSuspend(Thermistor_handle);
// 	}
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic Stuff ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Global values for ultrasonic reading */
float dist_us = 0;												// Calculates overall distance 
uint32_t reading_us = 0;											// Current ADC reading of Ultrasonic
static esp_adc_cal_characteristics_t *adc_chars_ultrasonic;			// Pointer to empty structure used to store ultrasonic ADC characteristics
static const adc_channel_t channel_ultrasound = ADC_CHANNEL_6; 		// Ultrasonic sensor uses GPIO34 (A2)
static const adc_atten_t atten_ultrasound = ADC_ATTEN_DB_0;			// Ultrasonic sensor uses attenuation 0
static const adc_unit_t unit_ultrasound = ADC_UNIT_1;				// Characterize ADC1

//Counter that tracks how many times ultrasonic sampled in a second
uint32_t us_read_counter = 0;

/* Function to initialize ultrasonic ADC pin */
void init_ultrasonic(void){
	/* Configure for ADC */
	adc1_config_channel_atten(channel_ultrasound, atten_ultrasound);	

	/* Characterize ADC */
	adc_chars_ultrasonic = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(unit_ultrasound, atten_ultrasound, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_ultrasonic);
}

/* Function to read sensor */
void read_collect_task(void){

	float voltage;
	float temp_therm_test;

	while (1)
	{
		//Ultrasonic Section
		//////////////////////////////////////////////////////////////////////////////////////////
		//init_ultrasonic();
		esp_adc_cal_characterize(unit_ultrasound, atten_ultrasound, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_ultrasonic);

		reading_us = adc1_get_raw((adc1_channel_t)channel_ultrasound);

		//printf("Raw readings ultrasonic: %d\n", reading_us);
		//float voltage = esp_adc_cal_raw_to_voltage(reading_us,adc_chars_ultrasonic);

		//Increase the running sum for ultrasound distance
		dist_us += (reading_us * 10);

		//Increase counter for how many times ultrasonic task run
		us_read_counter++;

		//Infrared Section
		//////////////////////////////////////////////////////////////////////////////////////////
		//init_ir();
		esp_adc_cal_characterize(unit_ir, atten_ir, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_ir);
		
		reading_ir = adc1_get_raw((adc1_channel_t)channel_ir);
		voltage = esp_adc_cal_raw_to_voltage(reading_ir,adc_chars_ir);
		
		//Each time you sample IR reading, add to a running sum
		dist_ir += 61.175 * pow((float)voltage/1000, -1.092);

		//Increase counter for how many times IR has been read
		ir_read_counter++;

		//Thermistor Section
		//////////////////////////////////////////////////////////////////////////////////////////
		//init_thermistor();
		esp_adc_cal_characterize(unit_thermistor, atten_thermistor, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars_thermistor);

		//Get the reading
		reading_therm = adc1_get_raw((adc1_channel_t)channel_thermistor);
		voltage = esp_adc_cal_raw_to_voltage(reading_therm,adc_chars_thermistor);

		//printf("Voltage is: %.2f\n\n", voltage);
		//Calculate Resistance of thermistor
		
		//R = R2 * ( (Vin_therm/(float)reading_therm) - 1);
		R = R2 * ( (Vin_therm/voltage) - 1);

		//float temp_therm_test;
		//Calculate the temperature
		temp_therm_test = pow( 1/T0 + ( (1/(float)B) * log( R / R0 )), -1 );

		//Add to runsum of temperature and ensure it's Celsius, not Kelvin
		temp_therm += temp_therm_test;
		temp_therm -= 273.15; 

		//Increase the counter that tracks how many times thermistor is read
		therm_read_counter++;


		vTaskSuspend(Collect_handle);
	}
}


////////////////////////////////////////////////////////////////////////////////
// Timer Stuff /////////////////////////////////////////////////////////////////
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
uint32_t counter = 0; //Keeps track of 1/10 of a second, everytime 

static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

		// Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);    // Timer
		
		//Start with each task suspended
		//vTaskSuspend(IR_handle);
		//vTaskSuspend(Thermistor_handle);
		
		vTaskSuspend(Collect_handle);

		//Once we count 10 readings of the hardware, display the results in a CSV format
		if (counter == 10)
		{
			//Divide by number of runs for each task. Divide by 1000 (mm to m) and by 100 (cm to m)
			printf("%d,%.2f,%.2f,%.2f\n", i++, (dist_us/ (float)us_read_counter) / 1000.0, temp_therm / (float)therm_read_counter, (dist_ir/(float)ir_read_counter) / 100.0);	// Convert distance to m
			//Reset distances and temperatures to be re-accumulated
			dist_us = 0;
			temp_therm = 0;
			dist_ir = 0;
			
			//Reset counters for each task to 0 to be re-accumulated
			us_read_counter = 0;
			ir_read_counter = 0;
			therm_read_counter = 0;
			counter = 0;
		}


        //If timer event is not triggered, do nothing
		//If it is, run each of the hardware tasks
        if (evt.flag == 1) {
				//vTaskResume(IR_handle);
				//vTaskResume(Thermistor_handle);
				vTaskResume(Collect_handle);
				counter++;
        }
        else
        {
			
			//vTaskSuspend(IR_handle);
			//vTaskSuspend(Thermistor_handle);
			vTaskSuspend(Collect_handle);
        }
        
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void app_main(void){

	timer_queue = xQueueCreate(10, sizeof(timer_event_t));
	adc1_config_width(ADC_WIDTH_BIT_10);
	/* Initialize ADC pins */
	init_ultrasonic();
	init_thermistor();
	init_ir();


	alarm_init();

	/* Create each of the tasks for hardware and timer */
	xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 10, &timerHandle);
    //xTaskCreate(read_ir_task, "read_ir_task", 2048, NULL, 7, &IR_handle);
	//xTaskCreate(read_thermistor_task, "read_thermistor_task", 2048, NULL, 6, &Thermistor_handle);
	xTaskCreate(read_collect_task, "read_collect_task", 2048, NULL, 5, &Collect_handle);
}