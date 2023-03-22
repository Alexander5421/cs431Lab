/*
 * imu.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#include "platform/imu.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "platform/serial.h"

namespace biped
{
IMU::IMU() : bmx160_(&Wire)
{
    initializeBMX160();
    initializeMPU6050();
}

IMUData
IMU::getDataBMX160() const
{
    return bmx160_data_;
}

IMUData
IMU::getDataMPU6050() const
{
    return mpu6050_data_;
}

void
IMU::readBMX160()
{
    // Assuming the sensor event structs are similar to MPU6050
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    // Read from BMX160 and populate sensor event structs
    if (!bmx160_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        Serial(LogLevel::error) << "Failed to read from BMX160.";
        return;
    }

    // TODO: Convert raw data into the standard body reference frame if needed

    // Populate the member sensor data struct
    bmx160_data_.acceleration_x = acceleration.acceleration.x;
    bmx160_data_.acceleration_y = acceleration.acceleration.y;
    bmx160_data_.acceleration_z = acceleration.acceleration.z;
    bmx160_data_.angular_velocity_x = angular_velocity.gyro.x;
    bmx160_data_.angular_velocity_y = angular_velocity.gyro.y;
    bmx160_data_.angular_velocity_z = angular_velocity.gyro.z;
    bmx160_data_.temperature = temperature.temperature;

    calculateAttitudeBMX160();
}

void
IMU::readMPU6050()
{
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    if (!mpu6050_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        Serial(LogLevel::error) << "Failed to read from MPU6050.";
        return;
    }

    // TODO: Convert raw data into the standard body reference frame if needed

    mpu6050_data_.acceleration_x = acceleration.acceleration.x;
    mpu6050_data_.acceleration_y = acceleration.acceleration.y;
    mpu6050_data_.acceleration_z = acceleration.acceleration.z;
    mpu6050_data_.angular_velocity_x = angular_velocity.gyro.x;
    mpu6050_data_.angular_velocity_y = angular_velocity.gyro.y;
    mpu6050_data_.angular_velocity_z = angular_velocity.gyro.z;
    mpu6050_data_.temperature = temperature.temperature;

    calculateAttitudeMPU6050();
}

void
IMU::initializeBMX160()
{
    if (!bmx160_.begin())
    {
        Serial(LogLevel::error) << "Failed to initialize BMX160.";
        return;
    }

    bmx160_.setAccelRange(eAccelRange_2G);
    bmx160_.setGyroRange(eGyroRange_125DPS);

    // Perform initial BMX160 read
    readBMX160();

    // Perform initial attitude calculation
    calculateAttitudeBMX160();

    bmx160_kalman_filter_attitude_y_.setAngle(radiansToDegrees(bmx160_data_.attitude_y));
    bmx160_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    bmx160_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    bmx160_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::initializeMPU6050()
{
    if (!mpu6050_.begin(AddressParameter::imu_mpu6050, &Wire, 0))
    {
        Serial(LogLevel::error) << "Failed to initialize MPU6050.";
        return;
    }

    mpu6050_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050_.setGyroRange(MPU6050_RANGE_250_DEG);

    // Perform initial MPU6050 read
    readMPU6050();

    // Perform initial attitude calculation
    calculateAttitudeMPU6050();

    mpu6050_kalman_filter_attitude_y_.setAngle(radiansToDegrees(mpu6050_data_.attitude_y));
    mpu6050_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    mpu6050_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    mpu6050_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::calculateAttitudeBMX160()
{
    // TODO: Implement attitude calculation for BMX160 similar to MPU6050
    // Refer to calculateAttitudeMPU6050() for guidance
}

void
IMU::calculateAttitudeMPU6050()
{
    if (PeriodParameter::fast <= 0)
    {
        Serial(LogLevel::error) << "Invalid fast period.";
        return;
    }

    // Calculate raw Y attitude (pitch) using linear accelerations
    double attitude_y_raw = atan2(mpu6050_data_.acceleration_x, sqrt(pow(mpu6050_data_.acceleration_y, 2) + pow(mpu6050_data_.acceleration_z, 2)));

    // Filter the raw Y attitude data using the Kalman filter
    const double attitude_y_kalman_filter = mpu6050_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(mpu6050_data_.angular_velocity_y),
            PeriodParameter::fast);

    // Convert the filtered Y attitude data back to radians and populate the member sensor data struct
    mpu6050_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
}
}
