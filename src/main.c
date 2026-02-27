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

// BME280 infrastructure
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;


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

static void bme280_write(uint8_t reg, uint8_t value){
    uint8_t tx[2] = { reg & 0x7F, value };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };
    ESP_ERROR_CHECK(spi_device_transmit(bme280_spi, &t));
}

static void bme280_read(uint8_t reg, uint8_t *data, size_t len){
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    tx[0] = reg | 0x80;
    memset(&tx[1], 0xFF, len);
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    ESP_ERROR_CHECK(spi_device_transmit(bme280_spi, &t));
    memcpy(data, &rx[1], len);
}

static void bme280_read_calibration(){
    uint8_t buf1[26];
    uint8_t buf2[7];
    bme280_read(0x88, buf1, 26);
    bme280_read(0xE1, buf2, 7);
    dig_T1 = (buf1[1] << 8) | buf1[0];
    dig_T2 = (buf1[3] << 8) | buf1[2];
    dig_T3 = (buf1[5] << 8) | buf1[4];
    dig_P1 = (buf1[7] << 8) | buf1[6];
    dig_P2 = (buf1[9] << 8) | buf1[8];
    dig_P3 = (buf1[11] << 8) | buf1[10];
    dig_P4 = (buf1[13] << 8) | buf1[12];
    dig_P5 = (buf1[15] << 8) | buf1[14];
    dig_P6 = (buf1[17] << 8) | buf1[16];
    dig_P7 = (buf1[19] << 8) | buf1[18];
    dig_P8 = (buf1[21] << 8) | buf1[20];
    dig_P9 = (buf1[23] << 8) | buf1[22];
    dig_H1 = buf1[25];
    dig_H2 = (buf2[1] << 8) | buf2[0];
    dig_H3 = buf2[2];
    dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F);
    dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
    dig_H6 = (int8_t)buf2[6];
}

static void bme280_init(){
    bme280_write(0xE0, 0xB6);
    // TODO: functional delay. Investigate that.
    vTaskDelay(pdMS_TO_TICKS(10));
    bme280_write(0xF2, 0x01);
    bme280_write(0xF4, 0x27);
    bme280_write(0xF5, 0xA0);

    // Read calibration data
    bme280_read_calibration();
}

static float compensate_temperature(int32_t adc_T){
    float var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
    float var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) *
                  (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) *
                 ((float)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

static float compensate_pressure(int32_t adc_P){
    float var1 = ((float)t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)dig_P4) * 65536.0f);
    var1 = (((float)dig_P3) * var1 * var1 / 524288.0f +
            ((float)dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1);
    if (var1 == 0.0f) {
        return 0;
    }
    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)dig_P8) / 32768.0f;
    return (p + (var1 + var2 + ((float)dig_P7)) / 16.0f) / 100.0f;
}

static float compensate_humidity(int32_t adc_H){
    float h = ((float)t_fine) - 76800.0f;
    h = (adc_H - (((float)dig_H4) * 64.0f + ((float)dig_H5) / 16384.0f * h)) *
        (((float)dig_H2) / 65536.0f *
         (1.0f + ((float)dig_H6) / 67108864.0f * h *
          (1.0f + ((float)dig_H3) / 67108864.0f * h)));
    h = h * (1.0f - ((float)dig_H1) * h / 524288.0f);
    if (h > 100.0f) h = 100.0f;
    if (h < 0.0f) h = 0.0f;
    return h;
}

static void bme280_get_data(float *temp, float *hum, float *press){
    uint8_t raw[8];
    bme280_read(0xF7, raw, 8);
    int32_t adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = (raw[6] << 8)  | raw[7];

    *temp = compensate_temperature(adc_T);
    *press = compensate_pressure(adc_P);
    *hum = compensate_humidity(adc_H);
}

/*
Code Plan
+ Initialize I2C (OLED, RTC, EEPROM, SENSOR)
+ Initialize OLED SSD1306 and display test image/string/etc
+ Initialize BME280 and get id - display on UART
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

    spi_init();
    bme280_init();
    //Test string on display
    ssd1306_display_textbox_ticker(dev_hdl, 0, 0, "putin - XY..0!", 15, false, 0);

    float temp = 0.0f;
    float press = 0.0f;
    float hum = 0.0f;

    while(1){
        bme280_get_data(&temp, &hum, &press);
        ESP_LOGI(TAG, "T=%.2f °C  P=%.2f hPa  H=%.2f %%", temp, press, hum);
        vTaskDelay(pdMS_TO_TICKS(2000));

    }


}
