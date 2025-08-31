#include "motor.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

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
    
    // Initialize GPIO pins
    init_pins();
}

// Initialize GPIO pins
void Motor::init_pins() {
    // Configure motor control pins as outputs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << in1) | (1ULL << in2) | (1ULL << in3) | 
                          (1ULL << in4) | (1ULL << enA) | (1ULL << enB);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    gpio_config(&io_conf);
    
    // Initialize all pins to low
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 0);
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 0);
    gpio_set_level(enA, 0);
    gpio_set_level(enB, 0);
    
    ESP_LOGI(TAG, "Motor pins initialized");
}

// Move forward
void Motor::forward() {
    ESP_LOGI(TAG, "Moving forward");
    
    // Left motor forward
    gpio_set_level(in1, 1);
    gpio_set_level(in2, 0);
    
    // Right motor forward
    gpio_set_level(in3, 1);
    gpio_set_level(in4, 0);
    
    // Enable both motors
    gpio_set_level(enA, 1);
    gpio_set_level(enB, 1);
}

// Move backward
void Motor::backward() {
    ESP_LOGI(TAG, "Moving backward");
    
    // Left motor backward
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 1);
    
    // Right motor backward
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 1);
    
    // Enable both motors
    gpio_set_level(enA, 1);
    gpio_set_level(enB, 1);
}

// Turn right
void Motor::right() {
    ESP_LOGI(TAG, "Turning right");
    
    // Left motor forward
    gpio_set_level(in1, 1);
    gpio_set_level(in2, 0);
    
    // Right motor backward
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 1);
    
    // Enable both motors
    gpio_set_level(enA, 1);
    gpio_set_level(enB, 1);
}

// Turn left
void Motor::left() {
    ESP_LOGI(TAG, "Turning left");
    
    // Left motor backward
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 1);
    
    // Right motor forward
    gpio_set_level(in3, 1);
    gpio_set_level(in4, 0);
    
    // Enable both motors
    gpio_set_level(enA, 1);
    gpio_set_level(enB, 1);
}

// Stop
void Motor::stop() {
    ESP_LOGI(TAG, "Stopping");
    
    // Disable both motors
    gpio_set_level(enA, 0);
    gpio_set_level(enB, 0);
    
    // Set all control pins to low
    gpio_set_level(in1, 0);
    gpio_set_level(in2, 0);
    gpio_set_level(in3, 0);
    gpio_set_level(in4, 0);
}
