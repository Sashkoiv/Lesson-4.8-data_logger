#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "at24c.h"

#define EEPROM_I2C_ADDR   (0x50 << 1)   // 24C32 base address
#define EEPROM_SIZE       4096          // bytes
#define EEPROM_PAGE_SIZE  32
#define EEPROM_TIMEOUT    100

#define TAG "DataLogger"

/*
Code Plan
* Initialize I2C (OLED, RTC, EEPROM, SENSOR)
* Initialize OLED SSD1306 and display test image/string/etc
* Initialize BME280 and get id - display on OLED
* Request datetime from RTC - display on OLED
* Retrieve data function for BME280
* Retrieve data function for DS1307
* Function to update data on OLED
    * Date and time
    * Temp, Hum, Pres
* Cycle business logic in superloop
    * Change to using timers later
* Init memory
* Prepare formatted buffer for the memory (data structure)
* START/STOP logging button
* UART commands handler
    * START_LOG
    * STOP_LOG
    * RETRIEVE_DATA
    * CLEAR EEPROM
    * REBOOT
* Auto write to memory every X s
* Button interrupt and handler
* Handler for UART commands
* START logging function
* STOP loggig function
* RETRIEVE data function
* CLEAR memory function
* REBOOT function
*

*/





void app_main(void)
{
}
