/**
 *  @file   imu.cpp
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  IMU class source.
 *
 *  This file implements the IMU class.
 */

/*
 *  Project headers.
 */
#include "platform/imu.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
IMU::IMU() : bmx160_(&Wire)
{
    /*
     *  Initialize BMX160 and MPU6050 IMUs.
     */
    // TODO LAB 6 YOUR CODE HERE.
    initializeBMX160();
    initializeMPU6050();
}

Compass::Calibration
IMU::getCompassCalibrationBMX160() const
{
    /*
     *  Get calibration data struct from the member
     *  BMX160 compass object and return the struct.
     */
    // TODO LAB 7 YOUR CODE HERE.
}

IMUData
IMU::getDataBMX160() const
{
    /*
     *  Return the class member BMX160 IMU data struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
    return bmx160_data_;
}

IMUData
IMU::getDataMPU6050() const
{
    /*
     *  Return the class member MPU6050 IMU data struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
    return mpu6050_data_;
}

void
IMU::readBMX160()
{
    /*
     *  Declare BMX160 sensor data structs.
     */
    sBmx160SensorData_t compass;
    sBmx160SensorData_t angular_velocity;
    sBmx160SensorData_t acceleration;

    /*
     *  Read from BMX160 IMU and populate BMX160 sensor data structs.
     */
    bmx160_.getAllData(&compass, &angular_velocity, &acceleration);

    /*
     *  Using the populated BMX160 sensor data structs, populate
     *  the corresponding entries in the member BMX160 IMU
     *  data struct.
     *
     *  Note that the raw data read from the BMX160 might not be in the
     *  standard body reference frame. Refer to the following materials
     *  to correctly convert the raw data into the standard body
     *  reference frame. Assign all compass raw data to the member BMX160
     *  IMU data struct as is. Assign 0 to the temperature entry in the member
     *  BMX160 IMU data struct since BMX160 does not have a temperature sensor.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 6 YOUR CODE HERE.
    bmx160_data_.compass_x = compass.magnetic.x;
    bmx160_data_.compass_y = compass.magnetic.y;
    bmx160_data_.compass_z = compass.magnetic.z;

    bmx160_data_.angular_velocity_x = angular_velocity.gyro.y;
    bmx160_data_.angular_velocity_y = angular_velocity.gyro.x;
    bmx160_data_.angular_velocity_z = -angular_velocity.gyro.z;

    bmx160_data_.acceleration_x = -acceleration.acceleration.y;
    bmx160_data_.acceleration_y = -acceleration.acceleration.x;
    bmx160_data_.acceleration_z = acceleration.acceleration.z;

    /*
     *  Execute BMX160 compass calibration using the populated
     *  member BMX160 IMU data struct.
     *  See the compass class for details.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Perform attitude calculations for BMX160 IMU.
     */
    // TODO LAB 6 YOUR CODE HERE.
     calculateAttitudeBMX160();
}

void
IMU::readMPU6050()
{
    /*
     *  Declare MPU6050 sensor event structs.
     */
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    /*
     *  Read from MPU6050 IMU and populate MPU6050 sensor event structs.
     */
    if (!mpu6050_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        Serial(LogLevel::error) << "Failed to read from MPU6050.";
        return;
    }

    /*
     *  Using the populated MPU6050 sensor event structs, populate
     *  the corresponding entries in the member MPU6050 IMU data
     *  data struct.
     *
     *  Note that the raw data read from the MPU6050 might not be in the
     *  standard body reference frame. Refer to the following materials
     *  to correctly convert the raw data into the standard body
     *  reference frame. Assign 0 to compass entries in the member
     *  MPU6050 IMU data struct since MPU6050 does not have a compass.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 6 YOUR CODE HERE.
    mpu6050_data_.acceleration_x = acceleration.acceleration.x;
    mpu6050_data_.acceleration_y = acceleration.acceleration.y;
    mpu6050_data_.acceleration_z = acceleration.acceleration.z;

    mpu6050_data_.angular_velocity_x = angular_velocity.gyro.x;
    mpu6050_data_.angular_velocity_y = angular_velocity.gyro.y;
    mpu6050_data_.angular_velocity_z = angular_velocity.gyro.z;

    mpu6050_data_.temperature = temperature.temperature;

    /*
     *  Perform attitude calculations for MPU6050 IMU.
     */
    // TODO LAB 6 YOUR CODE HERE.
    calculateAttitudeMPU6050();
}

void
IMU::initializeBMX160()
{
    /*
     *  Initialize BMX160 IMU driver object and validate
     *  the initialization.
     */
    if (!bmx160_.begin())
    {
        Serial(LogLevel::error) << "Failed to initialize BMX160.";
        return;
    }

    /*
     *  Configure BMX160 IMU.
     */
    bmx160_.setAccelRange(eAccelRange_2G);
    bmx160_.setGyroRange(eGyroRange_250DPS);

    /*
     *  Perform initial BMX160 IMU read.
     */
    // TODO LAB 6 YOUR CODE HERE.
    readBMX160();

    /*
     *  Initialize BMX160 compass with the member BMX160
     *  IMU data struct.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Perform initial attitude calculations.
     *  Do not call the calculateAttitude* function here.
     *  See the comments in the calculateAttitude* function
     *  for further instructions.
     */
    // TODO LAB 6 YOUR CODE HERE.


    /*
     *  Configure Y attitude Kalman filter.
     */
    bmx160_kalman_filter_attitude_y_.setAngle(radiansToDegrees(bmx160_data_.attitude_y));
    bmx160_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    bmx160_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    bmx160_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::initializeMPU6050()
{
    /*
     *  Initialize MPU6050 IMU driver object and validate the initialization.
     */
    if (!mpu6050_.begin(AddressParameter::imu_mpu6050, &Wire, 0))
    {
        Serial(LogLevel::error) << "Failed to initialize MPU6050.";
        return;
    }

    /*
     *  Configure MPU6050 IMU.
     */
    mpu6050_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050_.setGyroRange(MPU6050_RANGE_250_DEG);

    /*
     *  Perform initial MPU6050 IMU read.
     */
    // TODO LAB 6 YOUR CODE HERE.
     readMPU6050();

    /*
     *  Perform initial attitude calculations.
     *  Do not call the calculateAttitude* function here.
     *  See the comments in the calculateAttitude* function
     *  for further instructions.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Configure Y attitude Kalman filter.
     */
    mpu6050_kalman_filter_attitude_y_.setAngle(radiansToDegrees(mpu6050_data_.attitude_y));
    mpu6050_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    mpu6050_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    mpu6050_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::calculateAttitudeBMX160()
{
    /*
     *  Validate fast period.
     */
    if (PeriodParameter::fast <= 0)
    {
        Serial(LogLevel::error) << "Invalid fast period.";
        return;
    }

    /*
     *  Calculate the raw Y attitude (pitch) data using
     *  the populated linear accelerations in the member
     *  BMX160 IMU data struct. Refer to the following
     *  materials to correctly convert the calculated
     *  data into the standard body reference frame.
     *
     *  Note that the attitudes (roll, pitch, and yaw) are
     *  angles between pairs of acceleration vectors. Use
     *  atan2 function instead of atan for correct signedness.
     *
     *  Remember to perform the same calculation in the
     *  initialize* function for initialization but populate the
     *  member BMX160 IMU data struct using the raw data
     *  (unfiltered) directly, since the Kalman filter
     *  had not been initialized at that point in the
     *  initialize* function.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 6 YOUR CODE HERE.
    const double attitude_y_raw = atan2(bmx160_data_.compass_y, bmx160_data_.compass_x );
    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    const double attitude_y_kalman_filter = bmx160_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(bmx160_data_.angular_velocity_y),
            PeriodParameter::fast);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member BMX160 IMU data
     *  struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
    bmx160_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);

    /*
     *  Perform attitude calculations for BMX160 compass
     *  with the member BMX160 IMU data struct.
     *  See the compass class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void
IMU::calculateAttitudeMPU6050()
{
    /*
     *  Validate fast period.
     */
    if (PeriodParameter::fast <= 0)
    {
        Serial(LogLevel::error) << "Invalid fast period.";
        return;
    }

    /*
     *  Calculate the raw Y attitude (pitch) data using
     *  the populated linear accelerations in the member
     *  MPU6050 IMU data struct. Refer to the following
     *  materials to correctly convert the calculated
     *  data into the standard body reference frame.
     *
     *  Note that the attitudes (roll, pitch, and yaw) are
     *  angles between pairs of acceleration vectors. Use
     *  atan2 function instead of atan for correct signedness.
     *
     *  Remember to perform the same calculation in the
     *  initialize* function for initialization but populate the
     *  member MPU6050 IMU data struct using the raw data
     *  (unfiltered) directly, since the Kalman filter
     *  had not been initialized at that point in the
     *  initialize* function.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 6 YOUR CODE HERE.
    const double attitude_y_raw = 1;

    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    const double attitude_y_kalman_filter = mpu6050_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(mpu6050_data_.angular_velocity_y),
            PeriodParameter::fast);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member MPU6050 IMU data
     *  data struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
     mpu6050_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
}

void IRAM_ATTR
IMU::onPushButtonB()
{
    /*
     *  Start BMX160 compass calibration process.
     *  See the compass class for details.
     */
    // TODO LAB 7 YOUR CODE HERE.
}
}   // namespace biped
