/*!
 *  @file BNO055.h
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __BNO055_H__
#define __BNO055_H__

#include <Arduino.h>
#include <Wire.h>

#include "utility/imumaths.h"

class I2CDevice;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          BNO055 Sensor
 */
class BNO055 {
public:
  enum class Address {
    ToLow = 0x28,
    ToHigh = 0x29,
  };

  enum class TemperatureSource {
    Accelerometer = 0,
    Gyroscope = 1,
  };

  typedef struct {
    int16_t accOffsetX;
    int16_t accOffsetY;
    int16_t accOffsetZ;
    int16_t magOffsetX;
    int16_t magOffsetY;
    int16_t magOffsetZ;
    int16_t gyrOffsetX;
    uint16_t gyrOffsetY;
    uint16_t gyrOffsetZ;
    uint16_t accRadiusOffsetX;
    uint16_t magRadiusOffsetY;
  } OffsetValues;

  /** Operation mode settings **/
  enum class OperationMode {
    Config = 0x00,
    AccelerometerOnly = 0x01,
    MagnetometerOnly = 0x02,
    GyroscopeOnly = 0x03,
    AccelerometerMagnetometer = 0x04,
    AccelerometerGyroscope = 0x05,
    MagnetometerGyroscope = 0x06,
    AccelerometerMagnetometerGyroscope = 0x07,
    InertialMeasurementUnit = 0x08,
    Compass = 0x09,
    MagnetometerForGyroscope = 0x0A,
    NineDegreesOfFreedomFastMagnetometerCalibrationOff = 0x0B,
    NineDegreesOfFreedom = 0x0C,
  };

  /** Remap settings **/
  enum class AxisRemapConfig {
    P0 = 0x21,
    P1 = 0x24, // default
    P2 = 0x24,
    P3 = 0x21,
    P4 = 0x24,
    P5 = 0x21,
    P6 = 0x21,
    P7 = 0x24
  };

  /** Remap Signs **/
  enum class AxisRemapSign {
    P0 = 0x04,
    P1 = 0x00, // default
    P2 = 0x06,
    P3 = 0x02,
    P4 = 0x03,
    P5 = 0x01,
    P6 = 0x07,
    P7 = 0x05
  };

  /** A structure to represent revisions **/
  typedef struct {
    uint8_t accel_rev; /**< acceleration rev */
    uint8_t mag_rev;   /**< magnetometer rev */
    uint8_t gyro_rev;  /**< gyroscrope rev */
    uint16_t sw_rev;   /**< SW rev */
    uint8_t bl_rev;    /**< bootloader rev */
  } adafruit_bno055_rev_info_t;

  /** Vector Mappings **/
  enum class Vector {
    Accelerometer,
    Magnetometer,
    Gyroscope,
    Euler,
    LinearAccelerometer,
    Gravity,
  };

  BNO055(Address address = Address::ToLow, TwoWire *wire = &Wire);

  bool begin(OperationMode mode = OperationMode::NineDegreesOfFreedom);
  bool checkChipID();
  bool setMode(OperationMode mode);
  bool setAxisRemap(AxisRemapConfig remapcode);
  bool setAxisSign(AxisRemapSign remapsign);
  bool getRevInfo(adafruit_bno055_rev_info_t *);
  bool setExtCrystalUse(boolean usextal);
  bool getSystemStatus(uint8_t *system_status, uint8_t *self_test_result,
                       uint8_t *system_error);
  bool getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel,
                      uint8_t *mag);

  bool getVector(Vector vector, imu::Vector<3> *vectorData);
  bool getQuat(imu::Quaternion *quat);

  bool getTemperature(int8_t *temp);
  bool setTemperatureSource(TemperatureSource source);

  /* Functions to deal with raw calibration data */
  bool getSensorOffsets(OffsetValues *offsets_type);
  bool setSensorOffsets(OffsetValues offsets_type);
  bool isFullyCalibrated();

  /* Power managments functions */
  bool enterSuspendMode();
  bool enterNormalMode();

private:
  I2CDevice *_busDevice;
  OperationMode _mode;
};

#endif
