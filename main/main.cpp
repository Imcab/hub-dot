#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30102.hpp"

static const char *TAG = "main";

static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_6; // D4
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_7; // D5
static constexpr i2c_port_num_t I2C_PORT = I2C_NUM_0; // the C3 only has one I2C controller

static constexpr int READ_PERIOD_MS = 100; // FIFO holds 32 samples, at 25 effective sps it never overflows
static constexpr int MAX_CONSECUTIVE_ERRORS = 10; // 10 x 100 ms = 1 s failing in a row -> restart the sensor
static constexpr int RETRY_DELAY_MS = 1000; // wait between restart attempts if begin() keeps failing

extern "C" void app_main(void)
{
    // A. Create the I2C bus
    i2c_master_bus_config_t bus_cfg{};
    bus_cfg.i2c_port = I2C_PORT;
    bus_cfg.sda_io_num = I2C_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not create the I2C bus: %s", esp_err_to_name(err));
        return;
    }

    static MAX30102 sensor;

    MAX30102::Config cfg = {};
    cfg.smp_ave = MAX30102_SMP_AVE_4;
    cfg.adc_range = MAX30102_ADC_RGE_16384NA;
    cfg.sample_rate = MAX30102_SR_100SPS;
    cfg.pulse_width = MAX30102_PW_411US;
    cfg.led_red_pa = MAX30102_LED_PA_FROM_MA(6.2);
    cfg.led_ir_pa = MAX30102_LED_PA_FROM_MA(6.2);
    cfg.fifo_a_full = 15;
    cfg.fifo_rollover = true;

    err = sensor.begin(bus, cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 begin failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus);
        return;
    }

    MAX30102::Sample samples[MAX30102_FIFO_DEPTH];
    int consecutive_errors = 0;
    while (true) {
        int n = sensor.readSamples(samples, MAX30102_FIFO_DEPTH);
        if (n < 0) {
            consecutive_errors++;
            if (consecutive_errors == 1) {
                ESP_LOGW(TAG, "readSamples failed"); // only the first one, so the monitor is not flooded
            }
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                ESP_LOGE(TAG, "%d errors in a row, restarting the sensor", consecutive_errors);
                sensor.end();
                // Keep retrying until it comes back (e.g. a loose wire that gets reconnected)
                while ((err = sensor.begin(bus, cfg)) != ESP_OK) {
                    ESP_LOGE(TAG, "Restart failed: %s, retrying in %d ms", esp_err_to_name(err), RETRY_DELAY_MS);
                    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                }
                ESP_LOGI(TAG, "Sensor restarted");
                consecutive_errors = 0;
            }
        } else {
            if (consecutive_errors > 0) {
                ESP_LOGI(TAG, "Recovered after %d failed reads", consecutive_errors);
            }
            consecutive_errors = 0;
        }
        for (int i = 0; i < n; i++) {
            printf("%lu,%lu\n", (unsigned long)samples[i].red, (unsigned long)samples[i].ir);
        }
        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
    }
}
