#ifndef __main_h
#define __main_h

#include "Types.h"
#include "bitInRegOperations.h"


#define PIN_SDA           GPIO_NUM_21
#define PIN_SCL           GPIO_NUM_22
#define LED_BUILTIN       GPIO_NUM_2
#define PIN_ADS1115_ALRT  GPIO_NUM_23

// enable what you use
#define ENABLE_ADS1115

// intervals at which task is run (in milli seconds)
#define TASK_INTERVAL_ADS1115_SINGLE_READ  20  // depends on ADS1115_DATA_RATE of ADS1115!!!!
#define TASK_INTERVAL_ADS1115  100                // depends on ADS1115_DATA_RATE of ADS1115!!!!
#define TASK_INTERVAL_ADS1115_PRINTF 100
// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 )
// Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY). 
#define TASK_PRIORITY_ADS1115   8

#define TASK_STACK_SIZE_ADS1115 configMINIMAL_STACK_SIZE*10


#define ADS1115_MODE      ADS1115_MODE_SINGLESHOT
#define ADS1115_DATA_RATE ADS1115_DATA_RATE_64  // carefully because of task interval
#define ADS1115_PGA       ADS1115_PGA_6P144

void app_main();
void init_gpio_LEDBUILTIN();
void init_gpio_ADS1115_alertPin();
void init_ADS1115();
void task_read_ads1115(void* pvParameter);
void task_printf_voltages(void *pvParameters);
void printf_clearLine();
void printf_cursorUpOneLine();
void printf_cursorUpLines(BYTE byN);

#define GPIO_REG_ADDR 0x3FF44000
#define GPIO_REG_IN_ADDR_OFFSET 0x003c
#define GPIO_REG_IN_ADDR (GPIO_REG_ADDR + GPIO_REG_IN_ADDR_OFFSET)
#define GPIO_REG_IN (*((DWORD*)GPIO_REG_IN_ADDR))


#ifndef INTERRUPTS_DISABLE
#define INTERRUPTS_DISABLE()  portDISABLE_INTERRUPTS()
#endif  // INTERRUPTS_DISABLE

#ifndef INTERRUPTS_ENABLE
#define INTERRUPTS_ENABLE()   portENABLE_INTERRUPTS()
#endif  // INTERRUPTS_ENABLE

#endif  // __main_h

