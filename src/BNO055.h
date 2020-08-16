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
  enum class Axis {
    XAxis = 0,
    YAxis = 1,
    ZAxis = 2,
  };

  /** Remap Signs **/
  enum class AxisSign {
    // 0: Positive X, Positive Y, Positive Z
    PosXPosYPosZ = 0b000,
    // 1: Positive X, Positive Y, Negative Z
    PosXPosYNegZ = 0b001,
    // 2: Positive X, Negative Y, Positive Z
    PosXNegYPosZ = 0b010,
    // 3: Positive X, Negative Y, Negative Z
    PosXNegYNegZ = 0b011,
    // 4: Negative X, Positive Y, Positive Z
    NegXPosYPosZ = 0b100,
    // 5: Negative X, Positive Y, Negative Z
    NegXPosYNegZ = 0b101,
    // 6: Negative X, Negative Y, Positive Z
    NegXNegYPosZ = 0b110,
    // 7: Negative X, Negative Y, Negative Z
    NegXNegYNegZ = 0b111,
  };

  enum class PlacementConfig {
    // Top view:
    // NegXPosYPosZ, 0x21 (2,0,1), 0x04
    // Z: +Z, Y: +X, X: -Y
    P0,
    // PosXPosYPosZ, 0x24 (2,1,0), 0x00
    // Z: +Z, Y: +Y, X: +X
    P1,
    // NegXNegYPosZ, 0x24 (2,1,0), 0x06
    // Z: +Z, Y: -Y, X: -X
    P2,
    // PosXNegYPosZ, 0x21 (2,0,1), 0x02
    // Z: +Z, Y: -X, X: +Y
    P3,
    // Bottom view:
    // PosXNegYNegZ, 0x24 (2,1,0), 0x03
    // Z: -Z, Y: -Y, X: +X
    P4,
    // PosXPosYNegZ, 0x21 (2,0,1), 0x01
    // Z: -Z, Y: +X, X: +Y
    P5,
    // NegXNegYNegZ, 0x21 (2,0,1), 0x07
    // Z: -Z, Y: -X, X: -Y
    P6,
    // NegXNegYNegZ, 0x24 (2,1,0), 0x05
    // Z: -Z, Y: -Y, X: -X
    P7,
  };

  /** Vector Mappings **/
  enum class Vector {
    Accelerometer,
    Magnetometer,
    Gyroscope,
    Euler,
    LinearAccelerometer,
    Gravity,
  };

  /** A structure to represent revisions **/
  typedef struct {
    uint8_t accelerometerChipID;
    uint8_t magnetometerChipID;
    uint8_t gyroscopeChipID;
    uint16_t softwareRevision;
    uint8_t bootloadRevision;
  } DeviceInfo;

  BNO055(Address address = Address::ToLow, TwoWire *wire = &Wire);

  bool begin(OperationMode mode = OperationMode::NineDegreesOfFreedom);
  bool checkChipID();

  bool setMode(OperationMode mode);
  bool setPlacementConfig(PlacementConfig placementConfig);
  bool setAxisRemap(Axis xAxis, Axis yAxis, Axis zAxis);
  bool setAxisSignRemap(AxisSign remapsign);

  bool getDeviceInfo(DeviceInfo *deviceInfo);
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
