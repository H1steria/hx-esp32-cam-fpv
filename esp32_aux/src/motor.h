#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"

class Motor {
private:
    // Motor control pins
    gpio_num_t in1, in2, in3, in4, enA, enB;
    
    // Initialize GPIO pins
    void init_pins();

public:
    // Constructor
    Motor(gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4, 
          gpio_num_t enA, gpio_num_t enB);
    
    // Motor control methods
    void forward();
    void backward();
    void right();
    void left();
    void stop();
};

#endif // MOTOR_H
