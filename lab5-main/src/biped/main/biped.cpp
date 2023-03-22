/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 *  Project headers.
 */
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "task/interrupt.h"
#include "common/pin.h"
#include "common/parameter.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "task/task.h"
#include "platform/neopixel.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

void IRAM_ATTR test() {
    biped::Serial(LogLevel::info) << "test";
}

void IRAM_ATTR button() {
    biped::Serial(LogLevel::info) << "btn";
}

int add_int_handler(const uint8_t pin, void (*f)(void), gpio_int_type_t type) {
    int ret;
    ret = gpio_isr_handler_add((gpio_num_t) pin, (void (*)(void*)) f, nullptr);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_set_intr_type((gpio_num_t) pin, type);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

/**
 *  @brief  Arduino setup function.
 *
 *  This function creates, configures, and launches
 *  objects. The function also attaches the interrupt
 *  handlers to the corresponding pins.
 */
void
setup()
{
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    Serial::setLogLevelMax(SerialParameter::log_level_max);
    serial_number_ = static_cast<unsigned>(EEPROM.read(AddressParameter::eeprom_serial_number));
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    Display::initialize();
    Serial::initialize();

    neopixel_ = std::make_shared<NeoPixel>();

    /*
     *  Instantiate all objects.
     *  See the defined object pointers at the top.
     *
     *  Note that the order of instantiation matters!
     *  An object must be instantiated first before
     *  its pointer can be used.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Set periods of Controller and Sensor.
     *  See the corresponding class for details.
     *
     *  Remember to set both the fast and slow domain
     *  period for each class, if applicable.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Attach the corresponding interrupt handler to the left
     *  motor encoder pin. Make sure to set the interrupt to CHANGE mode
     *  for maximum resolution.
     *
     *  See the Pin enum for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Enable timer thus starting the real-time tasks.
     */
    // TODO LAB 5 YOUR CODE HERE.


    if (biped::Serial::getLogLevelWorst() <= LogLevel::error)
    {
        biped::Serial(LogLevel::warn) << "Initialized with error(s).";
    }
    else
    {
        biped::Serial(LogLevel::info) << "Initialized.";
    }
}

/**
 *  @brief  Arduino loop function.
 *
 *  This function is periodically called by the
 *  Arduino framework. The function calls the
 *  best-effort task function.
 */
void
loop()
{
    /*
     *  Perform best-effort tasks.
     */
    bestEffortTask();
}
