/*!
 * @file BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
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
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "BNO055.h"

#include <Arduino.h>

#include <limits.h>
#include <math.h>
#include <I2CDevice.h>

/** CHIP ID **/
#define CHIP_ID (0xA0)

/** BNO055 Registers **/
enum class BNO055RegisterAddress {
  /* PAGE0 REGISTER DEFINITION START*/
  ChipID = 0x00,
  AccelerometerRevisionID = 0x01,
  MagnetometerRevisionID = 0x02,
  GyroscopeRevisionID = 0x03,
  SoftwareRevisionID = 0x04, // 2 bytes
  BootloaderRevisionID = 0x06,

  /* Page id register definition */
  PageID = 0x07,

  /* Accel data register */
  AccelerometerDataX = 0x08, // 2 bytes
  AccelerometerDataY = 0x0A, // 2 bytes
  AccelerometerDataZ = 0x0C, // 2 bytes

  /* Mag data register */
  MagnetometerDataX = 0x0E, // 2 bytes
  MagnetometerDataY = 0x10, // 2 bytes
  MagnetometerDataZ = 0x12, // 2 bytes

  /* Gyro data registers */
  GyroscopeDataX = 0x14, // 2 bytes
  GyroscopeDataY = 0x16, // 2 bytes
  GyroscopeDataZ = 0x18, // 2 bytes

  /* Euler data registers */
  EulerDataH = 0x1A, // 2 bytes
  EulerDataR = 0x1C, // 2 bytes
  EulerDataP = 0x1E, // 2 bytes

  /* Quaternion data registers */
  QuaternionDataW = 0x20, // 2 bytes
  QuaternionDataX = 0x23, // 2 bytes
  QuaternionDataY = 0x24, // 2 bytes
  QuaternionDataZ = 0x26, // 2 bytes

  /* Linear acceleration data registers */
  LinearAccelerometerDataX = 0x28, // 2 bytes
  LinearAccelerometerDataY = 0x2A, // 2 bytes
  LinearAccelerometerDataZ = 0x2C, // 2 bytes

  /* Gravity data registers */
  GravityDataX = 0x2E, // 2 bytes
  GravityDataY = 0x30, // 2 bytes
  GravityDataZ = 0x32, // 2 bytes

  /* Temperature data register */
  Temperature = 0x34,

  /* Status registers */
  CalibrationStatus = 0x35,
  SelfTestResult = 0x36,
  InterruptStatus = 0x37,

  SystemClockStatus = 0x38,
  SystemStatus = 0x39,
  SystemError = 0x3A,

  /* Unit selection register */
  UnitSelection = 0x3B,

  /* Mode registers */
  OperationMode = 0x3D,
  PowerMode = 0x3E,

  SystemTrigger = 0x3F,
  TemperatureSource = 0x40,

  /* Axis remap registers */
  AxisMapConfig = 0x41,
  AxisMapSign = 0x42,

  /* Accelerometer Offset registers */
  AccelerometerOffsetX = 0x55, // 2 bytes
  AccelerometerOffsetY = 0x57, // 2 bytes
  AccelerometerOffsetZ = 0x59, // 2 bytes

  /* Magnetometer Offset registers */
  MagnetometerOffsetX = 0x5B, // 2 bytes
  MagnetometerOffsetY = 0x5D, // 2 bytes
  MagnetometerOffsetZ = 0x5F, // 2 bytes

  /* Gyroscope Offset register s*/
  GyroscopeOffsetX = 0x61, // 2 bytes
  GyroscopeOffsetY = 0x63, // 2 bytes
  GyroscopeOffsetZ = 0x65, // 2 bytes

  /* Radius registers */
  AccelerometerRadius = 0x67, // 2 bytes
  MagnetometerRadius = 0x69, // 2 bytes
};

/** BNO055 power settings */
enum class BNO055PowerMode : uint8_t {
  Normal = 0x00,
  LowPower = 0x01,
  Suspend = 0x02,
};

namespace {

BNO055RegisterAddress RegisterAddressForVector(BNO055::Vector vector) {
  switch (vector) {
  case BNO055::Vector::Accelerometer:
    return BNO055RegisterAddress::AccelerometerDataX;
  case BNO055::Vector::Magnetometer:
    return BNO055RegisterAddress::AccelerometerDataZ;
  case BNO055::Vector::Gyroscope:
    return BNO055RegisterAddress::GyroscopeDataX;
  case BNO055::Vector::Euler:
    return BNO055RegisterAddress::EulerDataH;
  case BNO055::Vector::LinearAccelerometer:
    return BNO055RegisterAddress::LinearAccelerometerDataX;
  case BNO055::Vector::Gravity:
    return BNO055RegisterAddress::GravityDataX;
  }
  return BNO055RegisterAddress::AccelerometerDataX;
}

}

/*!
 *  @brief  Instantiates a new BNO055 class
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
BNO055::BNO055(Address address, TwoWire *wire) :
    _busDevice(new I2CDevice((int)address, wire))  {
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OperationMode::Config,
 *            OperationMode::AccelerometerOnly,
 *            OperationMode::MagnetometerOnly,
 *            OperationMode::GyroscopeOnly,
 *            OperationMode::AccelerometerMagnetometer,
 *            OperationMode::AccelerometerGyroscope,
 *            OperationMode::MagnetometerGyroscope,
 *            OperationMode::AccelerometerMagnetometerGyroscope,
 *            OperationMode::InertialMeasurementUnit,
 *            OperationMode::Compass,
 *            OperationMode::MagnetometerForGyroscope,
 *            OperationMode::NineDegreesOfFreedomFastMagnetometerCalibrationOff,
 *            OperationMode::NineDegreesOfFreedom]
 *  @return true if process is successful
 */
bool BNO055::begin(OperationMode mode) {
  // BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  _busDevice->getWire()->setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  /* Make sure we have the right device */
  if (!checkChipID()) {
    delay(1000); // hold on for boot
    if (!checkChipID()) {
      return false; // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OperationMode::Config)) {
    return false;
  }

  /* Reset */
  if (_busDevice->write8ToRegister(0x20, (uint8_t)BNO055RegisterAddress::SystemTrigger) != 1) {
    return false;
  }
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(30);
  while (!checkChipID()) {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  if (_busDevice->write8ToRegister((uint8_t)BNO055PowerMode::Normal, (uint8_t)BNO055RegisterAddress::PowerMode) != 1) {
    return false;
  }
  delay(10);

  if (_busDevice->write8ToRegister(0, (uint8_t)BNO055RegisterAddress::PageID) != 1) {
    return false;
  }

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8((uint8_t)BNO055RegisterAddress::UnitSelection, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8((uint8_t)BNO055RegisterAddress::AxisMapConfig, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8((uint8_t)BNO055RegisterAddress::AxisMapSign, AxisSign::P2); // P0-P7, Default is P1
  delay(10);
  */

  if (_busDevice->write8ToRegister(0, (uint8_t)BNO055RegisterAddress::SystemTrigger) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  return setMode(mode);
}

bool BNO055::checkChipID() {
  uint8_t value;
  if (_busDevice->read8FromRegister(&value, (uint8_t)BNO055RegisterAddress::ChipID) != 1) {
    return false;
  }
  return CHIP_ID == value;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OperationMode::Config,
 *            OperationMode::AccelerometerOnly,
 *            OperationMode::MagnetometerOnly,
 *            OperationMode::GyroscopeOnly,
 *            OperationMode::AccelerometerMagnetometer,
 *            OperationMode::AccelerometerGyroscope,
 *            OperationMode::MagnetometerGyroscope,
 *            OperationMode::AccelerometerMagnetometerGyroscope,
 *            OperationMode::InertialMeasurementUnit,
 *            OperationMode::Compass,
 *            OperationMode::MagnetometerForGyroscope,
 *            OperationMode::NineDegreesOfFreedomFastMagnetometerCalibrationOff,
 *            OperationMode::NineDegreesOfFreedom]
 */
bool BNO055::setMode(OperationMode mode) {
  _mode = mode;
  bool result = _busDevice->write8ToRegister((uint8_t)_mode, (uint8_t)BNO055RegisterAddress::OperationMode) == 1;
  delay(30);
  return result;
}

bool BNO055::setPlacementConfig(PlacementConfig placementConfig) {
  AxisSign axisSign = AxisSign::PosXPosYPosZ;
  Axis xAxis = Axis::XAxis;
  Axis yAxis = Axis::YAxis;
  Axis zAxis = Axis::ZAxis;
  switch (placementConfig) {
  case PlacementConfig::P0:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::YAxis;
    yAxis = Axis::XAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P1:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::XAxis;
    yAxis = Axis::YAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P2:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::XAxis;
    yAxis = Axis::YAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P3:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::YAxis;
    yAxis = Axis::XAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P4:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::XAxis;
    yAxis = Axis::YAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P5:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::YAxis;
    yAxis = Axis::XAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P6:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::YAxis;
    yAxis = Axis::XAxis;
    zAxis = Axis::ZAxis;
    break;
  case PlacementConfig::P7:
    axisSign = AxisSign::PosXPosYPosZ;
    xAxis = Axis::XAxis;
    yAxis = Axis::YAxis;
    zAxis = Axis::ZAxis;
    break;
  };
  bool result = setAxisRemap(xAxis, yAxis, zAxis);
  if (!result) {
    return result;
  }
  return setAxisSignRemap(axisSign);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [AxisRemapConfig::P0
 *           AxisRemapConfig::P1 (default)
 *           AxisRemapConfig::P2
 *           AxisRemapConfig::P3
 *           AxisRemapConfig::P4
 *           AxisRemapConfig::P5
 *           AxisRemapConfig::P6
 *           AxisRemapConfig::P7]
 */
bool BNO055::setAxisRemap(Axis xAxis, Axis yAxis, Axis zAxis) {
  OperationMode modeback = _mode;

  if (!setMode(OperationMode::Config)) {
    return false;
  }
  uint8_t remapConfig = ((uint8_t)zAxis << 4) + ((uint8_t)yAxis << 2) + (uint8_t)xAxis;
  if (_busDevice->write8ToRegister(remapConfig, (uint8_t)BNO055RegisterAddress::AxisMapConfig) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  return setMode(modeback);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [AxisSign::P0
 *           AxisSign::P1 (default)
 *           AxisSign::P2
 *           AxisSign::P3
 *           AxisSign::P4
 *           AxisSign::P5
 *           AxisSign::P6
 *           AxisSign::P7]
 */
bool BNO055::setAxisSignRemap(AxisSign remapSign) {
  OperationMode modeback = _mode;

  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->write8ToRegister((uint8_t)remapSign, (uint8_t)BNO055RegisterAddress::AxisMapSign) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  return setMode(modeback);
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
bool BNO055::setExtCrystalUse(boolean usextal) {
  OperationMode modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->write8ToRegister(0, (uint8_t)BNO055RegisterAddress::PageID) != 1) {
    return false;
  }
  if (_busDevice->write8ToRegister(usextal ? 0x80 : 0x00, (uint8_t)BNO055RegisterAddress::SystemTrigger) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  return setMode(modeback);
}

/*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
bool BNO055::getSystemStatus(uint8_t *system_status,
                                      uint8_t *self_test_result,
                                      uint8_t *system_error) {
  if (_busDevice->write8ToRegister(0, (uint8_t)BNO055RegisterAddress::PageID) != 1) {
    return false;
  }

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0) {
    size_t count = _busDevice->read8FromRegister(system_status, (uint8_t)BNO055RegisterAddress::SystemStatus);
    if (count != 1) {
      return false;
    }
  }

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0) {
    size_t count = _busDevice->read8FromRegister(self_test_result, (uint8_t)BNO055RegisterAddress::SelfTestResult);
    if (count != 1) {
      return false;
    }
  }

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0) {
    size_t count = _busDevice->read8FromRegister(self_test_result, (uint8_t)BNO055RegisterAddress::SystemError);
    if (count != 1) {
      return false;
    }
  }

  delay(200);
  return true;
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
bool BNO055::getRevInfo(adafruit_bno055_rev_info_t *info) {
  memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

  /* Check the accelerometer revision */
  if (_busDevice->read8FromRegister(&(info->accel_rev), (uint8_t)BNO055RegisterAddress::AccelerometerRevisionID) != 1) {
    return false;
  }

  /* Check the magnetometer revision */
  if (_busDevice->read8FromRegister(&(info->mag_rev), (uint8_t)BNO055RegisterAddress::MagnetometerRevisionID) != 1) {
    return false;
  }

  /* Check the gyroscope revision */
  if (_busDevice->read8FromRegister(&(info->gyro_rev), (uint8_t)BNO055RegisterAddress::GyroscopeRevisionID) != 1) {
    return false;
  }

  /* Check the SW revision */
  if (_busDevice->read8FromRegister(&(info->bl_rev), (uint8_t)BNO055RegisterAddress::BootloaderRevisionID) != 1) {
    return false;
  }

  return _busDevice->read16FromRegister(&(info->sw_rev), (uint8_t)BNO055RegisterAddress::SoftwareRevisionID) != 1;
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
bool BNO055::getCalibration(uint8_t *sys, uint8_t *gyro,
                                     uint8_t *accel, uint8_t *mag) {
  uint8_t calData;
  if (_busDevice->read8FromRegister(&calData, (uint8_t)BNO055RegisterAddress::CalibrationStatus) != 1) {
    return false;
  }
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
  return true;
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
bool BNO055::getTemperature(int8_t *temp) {
  return _busDevice->read8FromRegister((uint8_t *)temp, (uint8_t)BNO055RegisterAddress::Temperature) == 1;
}

bool BNO055::setTemperatureSource(TemperatureSource source) {
  return _busDevice->write8ToRegister((uint8_t)source, (uint8_t)BNO055RegisterAddress::TemperatureSource) != 1;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
bool BNO055::getVector(Vector vector, imu::Vector<3> *vectorData) {
  int16_t buffer[3];

  /* Read vector data */
  BNO055RegisterAddress registerAddress = RegisterAddressForVector(vector);
  if (_busDevice->readArray16FromRegister((uint16_t *)buffer, sizeof(buffer) / sizeof(buffer[0]), (uint8_t)registerAddress) != sizeof(buffer)) {
    return false;
  }
  if (!vectorData) {
    return true;
  }

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  double coef = 1.0;
  switch (vector) {
  case Vector::Magnetometer:
    /* 1uT = 16 LSB */
    coef = 16.0;
    break;
  case Vector::Gyroscope:
    /* 1dps = 16 LSB */
    coef = 16.0;
    break;
  case Vector::Euler:
    /* 1 degree = 16 LSB */
    coef = 16.0;
    break;
  case Vector::Accelerometer:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  case Vector::LinearAccelerometer:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  case Vector::Gravity:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  }
  (*vectorData)[0] = ((double)buffer[0]) / coef;
  (*vectorData)[1] = ((double)buffer[1]) / coef;
  (*vectorData)[2] = ((double)buffer[2]) / coef;

  return true;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
bool BNO055::getQuat(imu::Quaternion *quat) {
  uint16_t buffer[4];

  /* Read quat data (8 bytes) */
  if (_busDevice->readArray16FromRegister((uint16_t *)buffer, sizeof(buffer) / sizeof(buffer[0]), (uint8_t)BNO055RegisterAddress::QuaternionDataW) != sizeof(buffer)) {
    return false;
  }
  if (!quat) {
    return true;
  }

  /*!
   * Assign to Quaternion
   * See
   * http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  *quat = imu::Quaternion(scale * buffer[0], scale * buffer[1], scale * buffer[2], scale * buffer[3]);
  return true;
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool BNO055::getSensorOffsets(
    OffsetValues *offsets_type) {
  if (!isFullyCalibrated()) {
    return false;
  }
  OperationMode lastMode = _mode;
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->readArray16FromRegister((uint16_t *)offsets_type, sizeof(*offsets_type) / sizeof(offsets_type->accOffsetX), (uint8_t)BNO055RegisterAddress::AccelerometerOffsetX) != sizeof(*offsets_type)) {
    return false;
  }
  return setMode(lastMode);
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
bool BNO055::setSensorOffsets(
    OffsetValues offsets_type) {
  OperationMode lastMode = _mode;
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->writeArray16ToRegister((uint16_t *)&offsets_type, sizeof(offsets_type) / sizeof(offsets_type.accOffsetX), (uint8_t)BNO055RegisterAddress::AccelerometerOffsetX) != sizeof(offsets_type)) {
    return false;
  }
  return setMode(lastMode);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool BNO055::isFullyCalibrated() {
  uint8_t system, gyro, accel, mag;
  if (!getCalibration(&system, &gyro, &accel, &mag)) {
    return false;
  }

  switch (_mode) {
  case OperationMode::AccelerometerOnly:
    return (accel == 3);
  case OperationMode::MagnetometerOnly:
    return (mag == 3);
  case OperationMode::GyroscopeOnly:
  case OperationMode::MagnetometerForGyroscope: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OperationMode::AccelerometerMagnetometer:
  case OperationMode::Compass:
    return (accel == 3 && mag == 3);
  case OperationMode::AccelerometerGyroscope:
  case OperationMode::InertialMeasurementUnit:
    return (accel == 3 && gyro == 3);
  case OperationMode::MagnetometerGyroscope:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
bool BNO055::enterSuspendMode() {
  OperationMode modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->write8ToRegister(0x02, (uint8_t)BNO055RegisterAddress::PowerMode) != 1) {
    return false;
  }
  /* Set the requested operating mode (see section 3.3) */
  return setMode(modeback);
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
bool BNO055::enterNormalMode() {
  OperationMode modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  if (_busDevice->write8ToRegister(0, (uint8_t)BNO055RegisterAddress::PowerMode) != 1) {
    return false;
  }
  /* Set the requested operating mode (see section 3.3) */
  return setMode(modeback);
}
