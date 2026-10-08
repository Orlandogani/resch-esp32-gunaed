#include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "board";

esp_err_t board_api_i2c_bus(const board_desc_t *desc, i2c_master_bus_handle_t *out)
{
    static i2c_master_bus_handle_t s_bus;
    static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
    static SemaphoreHandle_t s_mutex;

    if (desc == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (desc->i2c.sda == BOARD_PIN_NONE || desc->i2c.scl == BOARD_PIN_NONE) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Create the mutex exactly once, then serialise bus creation on it (DES-BRD-003). */
    if (s_mutex == NULL) {
        SemaphoreHandle_t m = xSemaphoreCreateMutex();
        if (m == NULL) {
            return ESP_ERR_NO_MEM;
        }
        bool mine = false;
        portENTER_CRITICAL(&s_lock);
        if (s_mutex == NULL) {
            s_mutex = m;
            mine = true;
        }
        portEXIT_CRITICAL(&s_lock);
        if (!mine) {
            vSemaphoreDelete(m);
        }
    }

    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_bus == NULL) {
        const i2c_master_bus_config_t cfg = {
            .i2c_port = -1,
            .sda_io_num = desc->i2c.sda,
            .scl_io_num = desc->i2c.scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = { .enable_internal_pullup = desc->i2c.internal_pullup },
        };
        err = i2c_new_master_bus(&cfg, &s_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus SDA%d SCL%d: %s", desc->i2c.sda, desc->i2c.scl, esp_err_to_name(err));
            s_bus = NULL;
        } else {
            ESP_LOGI(TAG, "%s: I2C bus SDA%d SCL%d", desc->name, desc->i2c.sda, desc->i2c.scl);
        }
    }
    *out = s_bus;
    xSemaphoreGive(s_mutex);
    return err;
}
