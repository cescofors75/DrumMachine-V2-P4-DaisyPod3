#pragma once

#include <stdint.h>

// Four DFRobot SEN0502 rotaries and one direct-ADC M5Stack fader. Hardware
// polling uses a dedicated Wire1 task;
// process() applies changes from Arduino loop().
void i2c_rotaries_init();
void i2c_rotaries_task_start();
void i2c_rotaries_poll();
void i2c_rotaries_process();

bool i2c_rotaries_owns_function(uint8_t function);
bool i2c_rotaries_is_applying();
uint8_t i2c_rotaries_detected_mask();
uint8_t i2c_rotaries_address_ack_mask();
uint16_t i2c_rotaries_value(uint8_t index);
uint8_t i2c_rotaries_gain(uint8_t index);
uint32_t i2c_rotaries_button_press_count();
uint32_t i2c_rotaries_button_press_count(uint8_t index);
uint8_t i2c_rotaries_mux_address();
bool p4_fader_detected();
bool p4_fader_led_driver_ready();
uint16_t p4_fader_raw();
uint16_t p4_fader_value();
