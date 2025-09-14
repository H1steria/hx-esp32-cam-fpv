#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h" // For PWM control
#include "esp_log.h"
#include <cmath> // For std::abs

static const char *TAG = "MOTOR";

// LEDC (PWM) settings
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY          5000 // Hz
#define LEDC_RESOLUTION         LEDC_TIMER_10_BIT // 0-1023

#define LEDC_CHANNEL_0          LEDC_CHANNEL_0
#define LEDC_CHANNEL_1          LEDC_CHANNEL_1

// Constructor
Motor::Motor(gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4, 
             gpio_num_t enA, gpio_num_t enB) {
    // Store pin assignments
    this->in1 = in1;
    this->in2 = in2;
    this->in3 = in3;
    this->in4 = in4;
    this->enA = enA;
    this->enB = enB;
    
    // Initialize GPIO and LEDC pins
    init_pins();
}

// Initialize GPIO and LEDC pins
void Motor::init_pins() {
    // Configure motor direction pins as outputs
    gpio_config_t io_conf_dir;
    io_conf_dir.intr_type = GPIO_INTR_DISABLE;
    io_conf_dir.mode = GPIO_MODE_OUTPUT;
    io_conf_dir.pin_bit_mask = (1ULL << in1) | (1ULL << in2) | (1ULL << in3) | (1ULL << in4);
    io_conf_dir.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_dir.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_dir);
    
    // Initialize direction pins to low
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 0);
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 0);

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_RESOLUTION,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel for enA (Motor A speed control)
    ledc_channel_config_t ledc_channel_0 = {
        .gpio_num       = (int)enA,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));

    // Configure LEDC channel for enB (Motor B speed control)
    ledc_channel_config_t ledc_channel_1 = {
        .gpio_num       = (int)enB,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
    
    ESP_LOGI(TAG, "Motor pins and LEDC initialized");
}

// Stop
void Motor::stop() {
    ESP_LOGI(TAG, "Stopping");
    
    // Set duty to 0 for both channels
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    
    // Set all direction pins to low
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 0);
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 0);
}

// Set motor movement based on joystick X and Y values (-100 to 100)
void Motor::set_movement(int8_t joystick_x, int8_t joystick_y) {
    // Scale joystick values from -100 to 100 to PWM duty cycle (0-1023 for 10-bit resolution)
    // Max speed is 100, so duty cycle will be (value / 100.0) * 1023
    int32_t left_motor_speed;
    int32_t right_motor_speed;

    // Basic differential drive logic
    // joystick_y controls forward/backward speed
    // joystick_x controls turning (differential speed)

    // Calculate motor speeds based on joystick X and Y values
    // joystick_y: -100 (forward) to 100 (backward)
    // joystick_x: -100 (left) to 100 (right)

    // For forward movement (joystick_y negative or zero), -joystick_y will be positive or zero.
    // For backward movement (joystick_y positive), -joystick_y will be negative.
    // This ensures that the base speed for motors has the correct sign for forward/backward.
    left_motor_speed = -joystick_y + joystick_x;
    right_motor_speed = -joystick_y - joystick_x;

    // Clamp speeds to -100 to 100
    left_motor_speed = std::max((int32_t)-100, std::min((int32_t)100, left_motor_speed));
    right_motor_speed = std::max((int32_t)-100, std::min((int32_t)100, right_motor_speed));

    // Convert speeds to PWM duty cycles
    uint32_t left_duty = (uint32_t)(std::abs(left_motor_speed) / 100.0 * (1 << LEDC_RESOLUTION) - 1);
    uint32_t right_duty = (uint32_t)(std::abs(right_motor_speed) / 100.0 * (1 << LEDC_RESOLUTION) - 1);

    // Set directions
    if (left_motor_speed > 0) { // Left motor forward
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (left_motor_speed < 0) { // Left motor backward
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else { // Left motor stop
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    }

    if (right_motor_speed > 0) { // Right motor forward
        gpio_set_level(in3, 1);
        gpio_set_level(in4, 0);
    } else if (right_motor_speed < 0) { // Right motor backward
        gpio_set_level(in3, 0);
        gpio_set_level(in4, 1);
    } else { // Right motor stop
        gpio_set_level(in3, 0);
        gpio_set_level(in4, 0);
    }

    // Set PWM duty cycles
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, left_duty);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, right_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);

    ESP_LOGI(TAG, "Set movement: Raw X=%d, Raw Y=%d -> Left Speed=%d (Duty=%lu), Right Speed=%d (Duty=%lu)", 
             joystick_x, joystick_y, left_motor_speed, left_duty, right_motor_speed, right_duty);
}
