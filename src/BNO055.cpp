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
enum class BNO055RegisterAddress : uint8_t {
  /* PAGE0 REGISTER DEFINITION START*/
  ChipID = 0x00,
  AccelerometerChipID = 0x01,
  MagnetometerChipID = 0x02,
  GyroscopeChipID = 0x03,
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

enum class BNO055SelfTestResultMask : uint8_t {
  Microcontroller = 0b1000,
  Gyroscope =       0b0100,
  Magnetometer =    0x0010,
  Accelerometer =   0x0001,
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

BNO055::AxesConfiguration BNO055::getAxesConfigurationForDevicePlacement(DevicePlacement devicePlacement) {
  AxesConfiguration result = { { Axis::X, false}, { Axis::Y, false }, { Axis::Z, false} };
  switch (devicePlacement) {
  case DevicePlacement::P0:
    result.x.inverted = true;
    result.x.axis = Axis::Y;
    result.y.axis = Axis::X;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P1:
    result.x.axis = Axis::X;
    result.y.axis = Axis::Y;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P2:
    result.x.inverted = true;
    result.x.axis = Axis::X;
    result.y.axis = Axis::Y;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P3:
    result.y.inverted = true;
    result.x.axis = Axis::Y;
    result.y.axis = Axis::X;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P4:
    result.y.inverted = true;
    result.z.inverted = true;
    result.x.axis = Axis::X;
    result.y.axis = Axis::Y;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P5:
    result.z.inverted = true;
    result.x.axis = Axis::Y;
    result.y.axis = Axis::X;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P6:
    result.x.inverted = true;
    result.y.inverted = true;
    result.z.inverted = true;
    result.x.axis = Axis::Y;
    result.y.axis = Axis::X;
    result.z.axis = Axis::Z;
    break;
  case DevicePlacement::P7:
    result.x.inverted = true;
    result.z.inverted = true;
    result.x.axis = Axis::X;
    result.y.axis = Axis::Y;
    result.z.axis = Axis::Z;
    break;
  };
  return result;
}

bool BNO055::setAxesRemap(AxesConfiguration configuration) {
  OperationMode modeback = _mode;
  if (!setMode(OperationMode::Config)) {
    return false;
  }
  uint8_t axisMapSign = ((configuration.x.inverted ? 1 : 0) << 2) | ((configuration.y.inverted ? 1 : 0) << 1) | ((configuration.z.inverted ? 1 : 0) << 0);
  if (_busDevice->write8ToRegister((uint8_t)axisMapSign, (uint8_t)BNO055RegisterAddress::AxisMapSign) != 1) {
    return false;
  }
  uint8_t remapConfig = ((uint8_t)configuration.z.axis << 4) | ((uint8_t)configuration.y.axis << 2) | ((uint8_t)configuration.x.axis);
  if (_busDevice->write8ToRegister(remapConfig, (uint8_t)BNO055RegisterAddress::AxisMapConfig) != 1) {
    return false;
  }
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
 *   @param  systemStatus
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
bool BNO055::getSystemStatus(SystemStatus *systemStatus) {
  if (!systemStatus) {
    return true;
  }
  return _busDevice->read8FromRegister((uint8_t*)systemStatus, (uint8_t)BNO055RegisterAddress::SystemStatus) == 1;
}

bool BNO055::getSystemError(SystemError *systemError) {
  if (!systemError) {
    return true;
  }
  return _busDevice->read8FromRegister((uint8_t*)systemError, (uint8_t)BNO055RegisterAddress::SystemError) == 1;
}

bool BNO055::getSelfTestResult(bool *accelerometerSelfTest, bool *magnetometerSelfTest,
                               bool *gyroscopeSelfTest, bool *microcontrollerSelfTest) {
  uint8_t selfTestResult = 0;
  if (_busDevice->read8FromRegister(&selfTestResult, (uint8_t)BNO055RegisterAddress::SelfTestResult) != 1) {
    return false;
  }
  if (accelerometerSelfTest) {
    *accelerometerSelfTest = (selfTestResult & (uint8_t)BNO055SelfTestResultMask::Accelerometer) != 0;
  }
  if (magnetometerSelfTest) {
    *magnetometerSelfTest = (selfTestResult & (uint8_t)BNO055SelfTestResultMask::Magnetometer) != 0;
  }
  if (gyroscopeSelfTest) {
    *gyroscopeSelfTest = (selfTestResult & (uint8_t)BNO055SelfTestResultMask::Gyroscope) != 0;
  }
  if (microcontrollerSelfTest) {
    *microcontrollerSelfTest = (selfTestResult & (uint8_t)BNO055SelfTestResultMask::Microcontroller) != 0;
  }
  return true;
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
bool BNO055::getDeviceInfo(DeviceInfo *deviceInfo) {
  memset(deviceInfo, 0, sizeof(DeviceInfo));
  /* Fetch the accelerometer revision */
  if (_busDevice->read8FromRegister(&(deviceInfo->accelerometerChipID), (uint8_t)BNO055RegisterAddress::AccelerometerChipID) != 1) {
    Serial.println(" acc");
    return false;
  }
  /* Fetch the magnetometer revision */
  if (_busDevice->read8FromRegister(&(deviceInfo->magnetometerChipID), (uint8_t)BNO055RegisterAddress::MagnetometerChipID) != 1) {
    Serial.println(" mag");
    return false;
  }
  /* Fetch the gyroscope revision */
  if (_busDevice->read8FromRegister(&(deviceInfo->gyroscopeChipID), (uint8_t)BNO055RegisterAddress::GyroscopeChipID) != 1) {
    Serial.println(" gyr");
    return false;
  }
  /* Fetch the SW revision */
  if (_busDevice->read16FromRegister(&(deviceInfo->softwareRevision), (uint8_t)BNO055RegisterAddress::SoftwareRevisionID) != 2) {
    Serial.println(" sw");
    return false;
  }
  // Fetch the bootloader revision */
  return _busDevice->read8FromRegister(&(deviceInfo->bootloadRevision), (uint8_t)BNO055RegisterAddress::BootloaderRevisionID) == 1;
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
