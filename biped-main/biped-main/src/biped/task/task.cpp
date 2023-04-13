/**
 *  @file   task.cpp
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Task function source.
 *
 *  This file implements the task functions.
 */

/*
 *  External headers.
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "planner/maneuver_planner.h"
#include "platform/neopixel.h"
#include "sensor/sensor.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
void
ioExpanderAInterruptTask(void* pvParameters)
{
    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Validate I/O expander A object pointer and call the I/O
         *  expander interrupt callback function.
         *  See the I/O expander class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        if (io_expander_a_)
        {
            io_expander_a_->onInterrupt();
        }
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
ioExpanderBInterruptTask(void* pvParameters)
{
    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Validate I/O expander B object pointer and call the I/O
         *  expander interrupt callback function.
         *  See the I/O expander class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        if (io_expander_b_)
        {
            io_expander_b_->onInterrupt();
        }
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
cameraTask(void* pvParameters)
{
    /*
     *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
     *  function. Set clear count on exit to true and maximum
     *  task wait time to be maximum delay.
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Validate camera object pointer and perform streaming.
     *  See the camera class for details.
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
wiFiTask(void* pvParameters)
{
    /*
     *  Initialize the Wi-Fi driver object and disable sleep.
     *  See parameter header for details
     */
    // TODO LAB 9 YOUR CODE HERE.

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
realTimeTask(void* pvParameters)
{
    /*
     *  Declare start time point and set to 0.
     */
    unsigned long time_point_start = 0;

    for (;;)
    {
        /*
         *  Sleep until woken using the FreeRTOS ulTaskNotifyTake
         *  function. Set clear count on exit to true and maximum
         *  task wait time to be maximum delay.
         */
        // TODO LAB 6 YOUR CODE HERE.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         *  Calculate real-time task interval and update start
         *  time point.
         */
        interval_real_time_task_ = micros() - time_point_start;
        time_point_start = micros();

        /*
         *  Perform fast domain sensing.
         *  See the sensor class for details.
         */
        // TODO LAB 6 YOUR CODE HERE.
        sensor_->sense(true);

        /*
         *  Perform fast domain control.
         *  See the controller class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.
        controller_->control(true);

        /*
         *  Slow domain tasks.
         */
        if (timer_domain_ >= PeriodParameter::slow)
        {
            /*
             *  Perform slow domain sensing.
             *  See the sensor class for details.
             */
            // TODO LAB 6 YOUR CODE HERE.
            sensor_->sense(false);

            /*
             *  Perform slow domain control.
             *  See the controller class for details.
             */
            // TODO LAB 7 YOUR CODE HERE.
            controller_->control(false);

            /*
             *  Reset timing domain timer.
             */
            timer_domain_ = 0;
        }

        /*
         *  Perform actuation using the actuation
         *  command struct from the controller object.
         *  See the actuator class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.
        actuator_->actuate(controller_->getActuationCommand());

        /*
         *  Update timing domain timer.
         */
        timer_domain_ += PeriodParameter::fast;

        /*
         *  Calculate real-time task execution time.
         */
        execution_time_real_time_task_ = micros() - time_point_start;
    }

    /*
     *  Delete task upon exit using the FreeRTOS
     *  vTaskDelete function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    vTaskDelete(nullptr);
}

void
bestEffortTask()
{
    /*
     *  Declare camera task woken flag and set to false.
     */
    static bool camera_task_woken = false;

    /*
     *  Print serial number to display.
     */
    Display(0) << "Biped: #" << serial_number_;

    /*
     *  Print real-time task timings to display.
     */
    Display(1) << "Real-time: " << execution_time_real_time_task_ << " "
            << interval_real_time_task_;

    /*
     *  Print controller active status to display.
     */
    if (controller_->getActiveStatus())
    {
        Display(2) << "Controller: active";
    }
    else
    {
        Display(2) << "Controller: inactive";
    }

    /*
     *  Execute plan and store the planner stage.
     *  See the planner class for details.
     */
    // TODO LAB 8 YOUR CODE HERE.
    const int stage = -1;

    /*
     *  Print planner status to display.
     */
    if (stage < 0)
    {
        Display(3) << "Planner: inactive";
    }
    else
    {
        Display(3) << "Planner: stage " << stage;
    }

    /*
     *  Print Wi-Fi status to display.
     */
    if (WiFi.status() == WL_CONNECTED)
    {
        Display(4) << "Wi-Fi: " << WiFi.localIP().toString().c_str();

        /*
         *  If the Wi-Fi is connected, validate the camera task
         *  handle and wake camera task using the FreeRTOS
         *  xTaskNotifyGive function if the camera task woken flag
         *  is false. Then, set camera task woken flag to true.
         */
        // TODO LAB 9 YOUR CODE HERE.
    }
    else
    {
        Display(4) << "Wi-Fi: disconnected";
    }
    Display::display();

    /*
     *  Show the NeoPixel frame.
     *  See the NeoPixel class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    neopixel_->show();

    /*
     *  Flush the display driver buffer to the display.
     *  See the display class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    // EncoderData encoderData = sensor_->getEncoderData();
    // IMUData imudata = sensor_->getIMUDataBMX160();
    // TimeOfFlightData tofdata = sensor_->getTimeOfFlightData();

    // biped::Serial(LogLevel::info) << "LinearAcc" << imudata.acceleration_x << " " << imudata.acceleration_y << " " << imudata.acceleration_z;
    // biped::Serial(LogLevel::info) << "AngV" << imudata.angular_velocity_x << " " << imudata.angular_velocity_y << " " << imudata.angular_velocity_z;
    // biped::Serial(LogLevel::info) << "c" << imudata.compass_x << " " << imudata.compass_y << " " << imudata.compass_z;
    // // print the yaw
    // biped::Serial(LogLevel::info) << "Yaw: " << imudata.attitude_z;
    // biped::Serial(LogLevel::info) << "Pitch: " << imudata.attitude_y;
    // biped::Serial(LogLevel::info) <<"TOF"<<tofdata.range_left << " " << tofdata.range_right << " " << tofdata.range_middle;
    // biped::Serial(LogLevel::info) <<"Encoder"<<encoderData.position_x << " " << encoderData.velocity_x << " " << encoderData.steps;
    // //
    // ActuationCommand command = ActuationCommand();
    // command.motor_enable= true;
    // command.motor_left_forward =true;
    // command.motor_right_forward=true;
    // command.motor_left_pwm = 100;
    // command.motor_right_pwm = 100;

    // actuator_->actuate(command);

    // if (Serial.read()==1) {
    //     controller_->
    //     ControllerParameter::attitude_y_gain_proportional += 5;
    //     biped::Serial(LogLevel::info) << "Cur Controller Param Value: " << ControllerParameter::attitude_y_gain_proportional;
    // } else if (Serial.read() == 2) {
    //     ControllerParameter::attitude_y_gain_proportional -= 5
    //     biped::Serial(LogLevel::info) << "Cur Controller Param Value: " << ControllerParameter::attitude_y_gain_proportional;
    // }

    IMUData imudata = sensor_->getIMUDataBMX160();
    Compass::Calibration calibration_data = sensor_->getCompassCalibrationBMX160();
    biped::Serial(LogLevel::info) << "Compass Calibration offset x: " << calibration_data.offset_x;
    biped::Serial(LogLevel::info) << "Compass Calibration offset y: " << calibration_data.offset_y;
    biped::Serial(LogLevel::info) << "Compass Calibration offset z: " << calibration_data.offset_z;

    biped::Serial(LogLevel::info) << "Compass Calibration scaler x: " << calibration_data.scaler_x;
    biped::Serial(LogLevel::info) << "Compass Calibration scaler y: " << calibration_data.scaler_y;
    biped::Serial(LogLevel::info) << "Compass Calibration scaler z: " << calibration_data.scaler_z;

    biped::Serial(LogLevel::info) << "Compass Calibration sign x: " << calibration_data.sign_x;
    biped::Serial(LogLevel::info) << "Compass Calibration sign y: " << calibration_data.sign_y;
    biped::Serial(LogLevel::info) << "Compass Calibration sign z: " << calibration_data.sign_z;
    // biped::Serial(LogLevel::info) << "compass_x"<<  imudata.compass_x;
    // biped::Serial(LogLevel::info) << "compass_y"<<  imudata.compass_y;
    // biped::Serial(LogLevel::info) << "Yaw"<<  imudata.attitude_z;

}
}   // namespace biped