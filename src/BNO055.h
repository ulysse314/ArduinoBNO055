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
  enum class OperationMode : uint8_t {
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
  enum class Axis : uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
  };

  typedef struct {
    Axis axis;
    bool inverted;
  } AxisConfiguration;

  typedef struct {
    AxisConfiguration x;
    AxisConfiguration y;
    AxisConfiguration z;
  } AxesConfiguration;

  enum class DevicePlacement : uint8_t {
    // Top view:
    // P0: 0x21 (2,0,1), 0x04 (-X, +Y, +Z)
    //     Z: +Z, Y: +X, X: -Y
    P0,
    // P1: 0x24 (2,1,0), 0x00 (+X, +Y, +Z)
    //     Z: +Z, Y: +Y, X: +X
    P1,
    // P2: 0x24 (2,1,0), 0x06 (-X, -Y, +Z)
    //     Z: +Z, Y: -Y, X: -X
    P2,
    // P3: 0x21 (2,0,1), 0x02 (+X, -Y, +Z)
    //     Z: +Z, Y: -X, X: +Y
    P3,
    // Bottom view:
    // P4: 0x24 (2,1,0), 0x03 (+X, -Y, -Z)
    //     Z: -Z, Y: -Y, X: +X
    P4,
    // P5: 0x21 (2,0,1), 0x01 (+X, +Y, -Z)
    //     Z: -Z, Y: +X, X: +Y
    P5,
    // P6: 0x21 (2,0,1), 0x07 (-X, -Y, -Z)
    //     Z: -Z, Y: -X, X: -Y
    P6,
    // P7: 0x24 (2,1,0), 0x05 (-X, -Y, +Z)
    //     Z: -Z, Y: -Y, X: -X
    P7,
  };

  /** Vector Mappings **/
  enum class Vector : uint8_t {
    Accelerometer,
    Magnetometer,
    Gyroscope,
    Euler,
    LinearAccelerometer,
    Gravity,
  };

  enum class SystemStatus : uint8_t {
    Idle = 0,
    Error = 1,
    InitializingPeripherals = 2,
    SystemInitialization = 3,
    ExecutingSelfTest = 4,
    RunningWithFusionAlgorithm = 5,
    RunningWihtoutFusionAlgorithm = 6,
  };

  enum class SystemError : uint8_t {
    NoError = 0x00,
    PeripheralInitializationError = 0x01,
    SystemInitializationError = 0x02,
    SelfTestResultFailed = 0x03,
    RegisterMapValueOutOfRange = 0x04,
    RegisterMapAddressOutOfRange = 0x05,
    RegisterMapWriteError = 0x06,
    LowPowerNotAvailableForSelectedOperationMode = 0x07,
    AccelerometerPowerModeNotAvailable = 0x08,
    FusionAlgorithmConfigurationError = 0x09,
    SensorConfigurationError = 0x0A,
  };

  enum class Calibration : uint8_t {
    None = 0,
    Low = 1,
    Medium = 2,
    Fully = 3,
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
  static AxesConfiguration getAxesConfigurationForDevicePlacement(DevicePlacement devicePlacement);
  bool setAxesRemap(AxesConfiguration configuration);

  bool getDeviceInfo(DeviceInfo *deviceInfo);
  bool setExtCrystalUse(boolean usextal);
  bool getSystemStatus(SystemStatus *systemStatus);
  bool getSystemError(SystemError *systemError);
  bool getSelfTestResult(bool *accelerometerSelfTest, bool *magnetometerSelfTest,
                         bool *gyroscopeSelfTest, bool *microcontrollerSelfTest);
  bool getCalibration(Calibration *system, Calibration *gyro, Calibration *accel,
                      Calibration *mag);

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
