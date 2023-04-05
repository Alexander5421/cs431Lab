/**
 *  @file   actuator.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Actuator class source.
 *
 *  This file implements the actuator class.
 */

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "controller/controller.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
Actuator::Actuator()
{
    /*
     *  Set pin mode for the motor PWM pins using
     *  the Arduino pin mode function.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    int level = 0;
    analogWrite(ESP32Pin::motor_left_pwm,level);
    analogWrite(ESP32Pin::motor_right_pwm,level);

    /*
     *  Set pin mode for the motor direction and enable pins using
     *  the I/O expander pin mode functions.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (io_expander_a_) {
        // TODO no such function
        io_expander_a_->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_left_direction,true);
        io_expander_a_->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_right_direction,true);
    }
}

ActuationCommand
Actuator::getActuationCommand() const
{
    /*
     *  Return the member actuation command struct.
     */
    return actuation_command_;
    // TODO LAB 6 YOUR CODE HERE.
}

void
Actuator::actuate(const ActuationCommand& actuation_command)
{
    /*
     *  Store the given actuation command struct in the function
     *  parameter to the member actuation command struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
    actuation_command_ = actuation_command;
    /*
     *  Write motor enable from the member actuation
     *  command struct to the motor enable pin using the I/O expander
     *  digitalWrite* function.
     */
    // TODO LAB 6 YOUR CODE HERE.
    //todo no such function
    io_expander_a_->digitalWrite(actuation_command_.motor_enable);
    /*
     *  Write motor directions from the member actuation
     *  command struct to the motor direction pins using the I/O expander
     *  digitalWrite* function.
     *
     *  Note that Biped is expected to move forward
     *  with both motor_left_forward and motor_right_forward set to true.
     */
    // TODO LAB 6 YOUR CODE HERE.
    //todo no such function
    io_expander_a_->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_left_direction,actuation_command_.motor_left_forward);
    io_expander_a_->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_right_direction,actuation_command_.motor_left_forward);
    /*
     *  Clamp the motor PWM values from the member actuation
     *  command struct to be in between the minimum and the
     *  maximum PWM values defined in the motor parameter
     *  name space using the clamp function in the math header.
     *  Then, write the clamped values to the motor PWM pins.
     *  Remember to static_cast the motor parameter entries to
     *  doubles before passing them to the clamp function and
     *  static_cast the clamped values back to integers before
     *  passing them to the write function.
     *
     *  Note that the PWM values are analog values.
     *  Thus, one should use the Arduino analogWrite function
     *  instead of digitalWrite.
     */
    // TODO LAB 6 YOUR CODE HERE.
    int left_pwm = clamp(actuation_command_.motor_left_pwm,MotorParameter::pwm_min,MotorParameter::pwm_max);
    int right_pwm = clamp(actuation_command_.motor_right_pwm,MotorParameter::pwm_min,MotorParameter::pwm_max);
    analogWrite(ESP32Pin::motor_left_pwm,left_pwm);
    analogWrite(ESP32Pin::motor_right_pwm,right_pwm);
}
}   // namespace biped
