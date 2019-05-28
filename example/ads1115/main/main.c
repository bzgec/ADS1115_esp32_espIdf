
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

#include "main.h"
#include "movingAverage.h"
#include "ADS1115.h"
#include "customPrintf.h"

#define TAG_EXTERNAL_ADC  "ADS1115"

float g_fVoltageA0;
float g_fVoltageA1;

void app_main()
{
	init_gpio_LEDBUILTIN();  // turn OFF the LEDBUILTIN

	//rtc_cpu_freq_config_t frequency;
	//rtc_clk_cpu_freq_get_config(&frequency);
	//printf("ESP32 clock frequency: %dMHz\n", frequency.freq_mhz);

	ads1115_i2c_master_init(PIN_SDA, PIN_SCL);
	init_gpio_ADS1115_alertPin();
	init_ADS1115();

	xTaskCreate(&task_read_ads1115, "ads1115_task", TASK_STACK_SIZE_ADS1115, NULL, TASK_PRIORITY_ADS1115, NULL);
	xTaskCreate(&task_printf_voltages, "ads1115_print_values", TASK_STACK_SIZE_ADS1115, NULL, TASK_PRIORITY_ADS1115, NULL);

  /*while(1)
  {
  }*/
}

void init_gpio_LEDBUILTIN()
{
	// turn OFF the led that is built in...
	gpio_pad_select_gpio(LED_BUILTIN);
  gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_BUILTIN, 1);
}

void init_gpio_ADS1115_alertPin()
{
	gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1UL<<PIN_ADS1115_ALRT);
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

void init_ADS1115()
{
	esp_err_t espRc;
	printf("Testing device connections...\n");
  
	ads1115_set_alert_pin(PIN_ADS1115_ALRT);
	ads1115_set_address_default();

	espRc = ads1115_testConnection();
	if (espRc == ESP_OK) 
	{
		ESP_LOGI("ADS1115", "Connection successful");
	} 
	else 
	{
		ESP_LOGE("ADS1115", "Connection  failed. code: 0x%X", espRc);
	}

	// basically the same
	espRc = ads1115_reset();
  //ads1115_setToDefault(); // initialize ADS1115 16 bit A/D chip
  if(espRc != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "reset error: 0x%.2X", espRc);

  // no need to set because in this case it is set before every read (using more than just one pin...)
	//ads1115_setMultiplexer(ADS1115_MUX_P0_NG);

  // We're going to do single shot sampling
  espRc = ads1115_setMode(ADS1115_MODE);
  if(espRc != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setMode error: 0x%.2X", espRc);

  // Slow things down so that we can see that the "poll for conversion" code works
  espRc = ads1115_setRate(ADS1115_DATA_RATE);
  if(espRc != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setRate error: 0x%.2X", espRc);
      
  // Set the gain (PGA) +/- 6.144v
  // Note that any analog input must be higher than –0.3V and less than VDD +0.3
  espRc = ads1115_setGain(ADS1115_PGA);
  if(espRc != ESP_OK) ESP_LOGI(TAG_EXTERNAL_ADC, "setGain error: 0x%.2X", espRc);
  // ALERT/RDY pin will indicate when conversion is ready
    
  ads1115_setConversionReadyPinMode();

	  // To get output from this method, you'll need to turn on the 
  //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
  #ifdef ADS1115_SERIAL_DEBUG
  ads1115_showConfigRegister();
  printf("HighThreshold=%X\n", ads1115_getHighThreshold());
  printf("LowThreshold=%X\n", ads1115_getLowThreshold());
	#endif
}

void task_read_ads1115(void *pvParameter)
{
	esp_err_t espErr;
  float fDummy;

  while(1) 
  {
		// The below method sets the mux and gets a reading.
    espErr = ads1115_setMultiplexer(ADS1115_MUX_P0_NG);
		if(espErr != ESP_OK) printf("ERROR setting multiplexer A0: 0x%.2X\n", espErr);
    espErr = ads1115_triggerConversion();
    if(espErr != ESP_OK) printf("ERROR triggering conversion: 0x%.2X\n", espErr);
		vTaskDelay(TASK_INTERVAL_ADS1115_SINGLE_READ / portTICK_PERIOD_MS);
    ads1115_pollAlertReadyPin();
		// check movingAverage.h for "strength" of the filter
    espErr = ads1115_getMilliVolts(GET_CONVERSION_READ_ONLY, &fDummy);
    if(espErr != ESP_OK) 
    {
      ESP_LOGE(TAG_EXTERNAL_ADC, "Getting value from ADC - A0: 0x%.2X", espErr);
    }
    else
    {
      g_fVoltageA0 = movingAvg_floatA0(fDummy);
    }
    
  
    espErr = ads1115_setMultiplexer(ADS1115_MUX_P1_NG);
    if(espErr != ESP_OK) printf("ERROR setting multiplexer A1: 0x%.2X\n", espErr);
		espErr = ads1115_triggerConversion();
    if(espErr != ESP_OK) printf("ERROR triggering conversion: 0x%.2X\n", espErr);
		vTaskDelay(TASK_INTERVAL_ADS1115_SINGLE_READ / portTICK_PERIOD_MS);
    ads1115_pollAlertReadyPin();
		// check movingAverage.h for "strength" of the filter
    espErr = ads1115_getMilliVolts(GET_CONVERSION_READ_ONLY, &fDummy);
    if(espErr != ESP_OK) 
    {
      ESP_LOGE(TAG_EXTERNAL_ADC, "Getting value from ADC - A1: 0x%.2X", espErr);
    }
    else
    {
      g_fVoltageA1 = movingAvg_floatA1(fDummy);
    }
    
    
    vTaskDelay(TASK_INTERVAL_ADS1115 / portTICK_PERIOD_MS);
  }
}

void task_printf_voltages(void *pvParameters)
{
	printf("\n\n");
	while(1)
	{
		printf_cursorUpLines(2);
		printf("Voltage A0: %f\n", g_fVoltageA0);
		printf("Voltage A1: %f\n", g_fVoltageA1);

    vTaskDelay(TASK_INTERVAL_ADS1115_PRINTF / portTICK_PERIOD_MS);
	}
}