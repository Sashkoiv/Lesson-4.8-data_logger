#include <assert.h>
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

#define I2C_PORT        I2C_NUM_0
#define I2C_SCL         4
#define I2C_SDA         5
// i2c object
static i2c_master_bus_handle_t i2c_handle = NULL;

#define OLED_SIZE I2C_SSD1306_128x32_CONFIG_DEFAULT
#define OLED_CONTRAST   255
// i2c object
ssd1306_handle_t dev_hdl = NULL;

#define PIN_NUM_MOSI    11
#define PIN_NUM_MISO    13
#define PIN_NUM_CLK     12
#define PIN_NUM_CS      10
// spi object
static spi_device_handle_t bme280_spi;

#define TAG "DataLogger"

static void i2c_init(){
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_handle));
}

static void spi_init(){
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &bme280_spi));
}

static void oled_init(){
    ssd1306_config_t dev_cfg = OLED_SIZE;

    ESP_ERROR_CHECK(ssd1306_init(i2c_handle, &dev_cfg, &dev_hdl));

    if (dev_hdl == NULL) {
        ESP_LOGI(TAG, "ssd1306 init failed");
        assert(dev_hdl);
    }

    ssd1306_clear_display(dev_hdl, false);
    ESP_ERROR_CHECK(ssd1306_set_contrast(dev_hdl, OLED_CONTRAST));
}


/*
Code Plan
+ Initialize I2C (OLED, RTC, EEPROM, SENSOR)
+ Initialize OLED SSD1306 and display test image/string/etc
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





void app_main(void){
    i2c_init();
    oled_init();

    //Test string on display
    ssd1306_display_textbox_ticker(dev_hdl, 0, 0, "putin - XY..0!", 15, false, 0);

    while(1){
        vTaskDelay(1000);
    }


}
