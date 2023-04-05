/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Biped main program source.
 *
 *  This file implements the Biped main program.
 */

/*
 *  External headers.
 */
#include <EEPROM.h>
#include <esp_intr_alloc.h>
#include <ESP32TimerInterrupt.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "task/interrupt.h"
#include "planner/maneuver_planner.h"
#include "common/pin.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

/**
 *  @brief  Main program setup function.
 *
 *  This function creates, configures, and launched drivers, objects,
 *  and tasks. The function also sets pin modes and attaches interrupt
 *  handlers to their corresponding pins.
 */
void
setup()
{

    /*
     *  Set pin mode for the I/O expander interrupt pins using
     *  the Arduino pin mode function. Use pull-up if the pin
     *  mode is input.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Set I2C driver object (Wire) SDA and SCL pins and set the
     *  serial object maximum log level.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Initialize I2C driver (Wire), EEPROM driver (EEPROM),
     *  display, and serial objects.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Instantiate all objects and store their shared pointers.
     *
     *  Note that the order of instantiation matters! The camera
     *  object has to be instantiated first, then the I/O expanders,
     *  and then the rest of the objects.
     *
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Read and store the serial number from the EEPROM.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Set controller periods.
     *  See the controller class for details.
     *
     *  Remember to set both the fast and slow domain
     *  periods, if applicable.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Create I/O expander interrupt tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function. Set
     *  the task descriptive names to be their task
     *  function names. The tasks have the highest
     *  priority. Pin both tasks to core 1.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Attach the I/O expander and encoder interrupt handlers.
     *  Attach the I/O expander interrupt handlers first in rising mode.
     *  Then, attach the encoder interrupt handlers in change mode.
     *
     *  See the interrupt and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Set pin mode for the push button pins using
     *  the I/O expander pin mode functions. Use
     *  pull-up if the pin mode is input.
     *  See the parameter header for details.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Attach the push button interrupt handlers using
     *  the I/O expander functions in falling mode.
     *  See the interrupt and parameter header for details.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Create real-time, Wi-Fi, and camera tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function, in that
     *  order. Set the task descriptive names to be their task
     *  function names. The real-time task has the second
     *  highest priority and the other two tasks have the
     *  lowest priority. Pin all tasks to core 1.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Attach the timer interrupt handler to the interrupt timer.
     *  See the interrupt header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Print initialization status to serial based on
     *  the current worst log level.
     */
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
 *  @brief  Main program loop task function.
 *
 *  This function is called by the loop task created and launched by
 *  the ESP-IDF framework. The loop task has a low priority and calls
 *  the best-effort task function.
 */
void
loop()
{
    /*
     *  Perform best-effort tasks.
     */
    // TODO LAB 6 YOUR CODE HERE.
}
