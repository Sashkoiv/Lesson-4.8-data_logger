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


void app_main(void)
{
}
