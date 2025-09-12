#include "i2c.h"
#include "safe_printf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

bool s_i2c_initialized = false;

esp_err_t i2c_master_init()
{
    if (s_i2c_initialized) return ESP_OK;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        },
        .clk_flags = 0
    };
    
    // Uninstall driver first if it was previously installed
    i2c_driver_delete(I2C_MASTER_NUM);
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        LOG("Failed to configure I2C parameters: %s\n", esp_err_to_name(err));
        return err;
    }
    
    // Install driver with 0 buffer sizes for master mode
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        LOG("Failed to install I2C driver: %s\n", esp_err_to_name(err));
        return err;
    }

    s_i2c_initialized = true;
    LOG("I2C Master initialized - SDA:%d SCL:%d Freq:%dHz\n", I2C_SDA_PIN, I2C_SCL_PIN, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

void initialize_i2c()
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        LOG("Failed to initialize I2C master: %s\n", esp_err_to_name(ret));
    }
}

void send_i2c_command(uint8_t command, int8_t val1, int8_t val2)
{
    if (!s_i2c_initialized) {
        initialize_i2c();
        if (!s_i2c_initialized) {
            LOG("Failed to initialize I2C\n");
            return;
        }
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);

    if (command == I2C_CMD_JOYSTICK_MOVE) {
        i2c_master_write_byte(cmd, (uint8_t)val1, true);
        i2c_master_write_byte(cmd, (uint8_t)val2, true);
    }
    // Other commands might send 1 byte (val1) or no additional bytes.
    // For now, we assume only CMD_JOYSTICK_MOVE sends two additional bytes.
    // If other commands need val1, they can be added here.
    // For example:
    // else if (command == SOME_OTHER_COMMAND_WITH_ONE_VAL) {
    //     i2c_master_write_byte(cmd, (uint8_t)val1, true);
    // }

    i2c_master_stop(cmd);
    
    // Use a longer timeout to ensure reliable communication
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        if (command == I2C_CMD_JOYSTICK_MOVE) {
            LOG("I2C command %d (x:%d, y:%d) sent successfully\n", command, val1, val2);
        } else {
            LOG("I2C command %d sent successfully\n", command);
        }
    } else {
        LOG("Failed to send I2C command %d, error: %s\n", command, esp_err_to_name(ret));
        // Try to reinitialize I2C on failure
        s_i2c_initialized = false;
        initialize_i2c();
    }
}
