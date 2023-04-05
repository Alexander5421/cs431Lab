/**
 *  @file   sensor.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class source.
 *
 *  This file implements the sensor class.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "platform/io_expander.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "sensor/sensor.h"

/*
 *  Biped namespace.
 */
namespace biped
{
Sensor::Sensor()
{
    /*
     *  Set pin mode for the time-of-flight shutdown pins using
     *  the I/O expander pin mode functions.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Instantiate all time-of-flight objects and store
     *  their unique pointers.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

Compass::Calibration
Sensor::getCompassCalibrationBMX160() const
{
    /*
     *  Get BMX160 compass calibration struct from the
     *  member IMU object and return the struct.
     */
    // TODO LAB 7 YOUR CODE HERE.
}

EncoderData
Sensor::getEncoderData() const
{
    /*
     *  Get encoder data struct from the member encoder
     *  object and return the struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

IMUData
Sensor::getIMUDataBMX160() const
{
    /*
     *  Get BMX160 IMU data struct from the member IMU
     *  object and return the struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

IMUData
Sensor::getIMUDataMPU6050() const
{
    /*
     *  Get MPU6050 IMU data struct from the member IMU
     *  object and return the struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

TimeOfFlightData
Sensor::getTimeOfFlightData() const
{
    /*
     *  Return the class member time-of-flight data struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void
Sensor::sense(const bool& fast_domain)
{
    /*
     *  The fast domain tasks are the ones that requires
     *  faster execution frequency, such as the reading of
     *  the IMU, the attitude calculation, and etc. If such
     *  tasks were performed too infrequently, the delay might
     *  cause inaccurate or delayed data sampling. For example, if
     *  the attitude was calculated too infrequently, then it
     *  is possible that the plant has already moved to another
     *  pose during the gap between the calculations, which could
     *  cause, for instance, Biped failing to perfectly
     *  balance itself due to the delayed sample.
     *
     *  The slow domain tasks, on the other hand, are the ones
     *  that are better executed at lower frequency due to sensor noise
     *  or performance issues. For example, the X velocity is
     *  calculated as the number of encoder steps per unit time.
     *  The smaller the period, the noisier the velocity.
     *  Additionally, it is unnecessary to read non-essential sensors
     *  such as the time-of-flight sensors at higher frequency. Thus, it's
     *  more performance-friendly to process such tasks in slow domain.
     */
    if (fast_domain)
    {
        /*
         *  Read encoders.
         */
        // TODO LAB 6 YOUR CODE HERE.

        /*
         *  Read BMX160 IMU.
         */
        // TODO LAB 6 YOUR CODE HERE.

        /*
         *  Read MPU6050 IMU.
         */
        // TODO LAB 6 YOUR CODE HERE.
    }
    else
    {
        /*
         *  Calculate velocity.
         */
        // TODO LAB 6 YOUR CODE HERE.

        /*
         *  Read time-of-flight sensors.
         */
        // TODO LAB 6 YOUR CODE HERE.
    }
}

void IRAM_ATTR
Sensor::onEncoderLeftA()
{
    /*
     *  Call left encoder A callback function.
     *  See the encoder class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
Sensor::onEncoderLeftB()
{
    /*
     *  Call left encoder B callback function.
     *  See the encoder class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
Sensor::onEncoderRightA()
{
    /*
     *  Call right encoder A callback function.
     *  See the encoder class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
Sensor::onEncoderRightB()
{
    /*
     *  Call right encoder B callback function.
     *  See the encoder class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
Sensor::onPushButtonB()
{
    /*
     *  Call push button B callback function.
     *  See the IMU class for details.
     */
    // TODO LAB 7 YOUR CODE HERE.
}
}   // namespace biped
