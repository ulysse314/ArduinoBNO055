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

/** BNO055 CHIP ID **/
#define BNO055_CHIP_ID (0xA0)

/*!
 *  @brief  Instantiates a new BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
BNO055::BNO055(int32_t sensorID, Address address, TwoWire *wire) :
    _busDevice(new I2CDevice((int)address, wire)),
    _sensorID(sensorID) {
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool BNO055::begin(adafruit_bno055_opmode_t mode) {
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
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }

  /* Reset */
  if (_busDevice->write8ToRegister(0x20, BNO055_SYS_TRIGGER_ADDR) != 1) {
    return false;
  }
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(30);
  while (!checkChipID()) {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  if (_busDevice->write8ToRegister(POWER_MODE_NORMAL, BNO055_PWR_MODE_ADDR) != 1) {
    return false;
  }
  delay(10);

  if (_busDevice->write8ToRegister(0, BNO055_PAGE_ID_ADDR) != 1) {
    return false;
  }

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  if (_busDevice->write8ToRegister(0, BNO055_SYS_TRIGGER_ADDR) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  bool result = setMode(mode);
  delay(20);

  return result;
}

bool BNO055::checkChipID() {
  uint8_t value;
  if (_busDevice->read8FromRegister(&value, BNO055_CHIP_ID_ADDR) != 1) {
    return false;
  }
  return BNO055_CHIP_ID == value;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
bool BNO055::setMode(adafruit_bno055_opmode_t mode) {
  _mode = mode;
  bool result = _busDevice->write8ToRegister(_mode, BNO055_OPR_MODE_ADDR) == 1;
  delay(30);
  return result;
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
bool BNO055::setAxisRemap(
    adafruit_bno055_axis_remap_config_t remapcode) {
  adafruit_bno055_opmode_t modeback = _mode;

  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);
  if (_busDevice->write8ToRegister(remapcode, BNO055_AXIS_MAP_CONFIG_ADDR) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  bool result = setMode(modeback);
  delay(20);
  return result;
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
bool BNO055::setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign) {
  adafruit_bno055_opmode_t modeback = _mode;

  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);
  if (_busDevice->write8ToRegister(remapsign, BNO055_AXIS_MAP_SIGN_ADDR) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  bool result = setMode(modeback);
  delay(20);
  return result;
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
bool BNO055::setExtCrystalUse(boolean usextal) {
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);
  if (_busDevice->write8ToRegister(0, BNO055_PAGE_ID_ADDR) != 1) {
    return false;
  }
  if (_busDevice->write8ToRegister(usextal ? 0x80 : 0x00, BNO055_SYS_TRIGGER_ADDR) != 1) {
    return false;
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
  return true;
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
  if (_busDevice->write8ToRegister(0, BNO055_PAGE_ID_ADDR) != 1) {
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
    size_t count = _busDevice->read8FromRegister(system_status, BNO055_SYS_STAT_ADDR);
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
    size_t count = _busDevice->read8FromRegister(self_test_result, BNO055_SELFTEST_RESULT_ADDR);
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
    size_t count = _busDevice->read8FromRegister(self_test_result, BNO055_SYS_ERR_ADDR);
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
  if (_busDevice->read8FromRegister(&(info->accel_rev), BNO055_ACCEL_REV_ID_ADDR) != 1) {
    return false;
  }

  /* Check the magnetometer revision */
  if (_busDevice->read8FromRegister(&(info->mag_rev), BNO055_MAG_REV_ID_ADDR) != 1) {
    return false;
  }

  /* Check the gyroscope revision */
  if (_busDevice->read8FromRegister(&(info->gyro_rev), BNO055_GYRO_REV_ID_ADDR) != 1) {
    return false;
  }

  /* Check the SW revision */
  if (_busDevice->read8FromRegister(&(info->bl_rev), BNO055_BL_REV_ID_ADDR) != 1) {
    return false;
  }

  return _busDevice->read16FromRegister(&(info->sw_rev), BNO055_SW_REV_ID_MSB_ADDR) != 1;
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
  if (_busDevice->read8FromRegister(&calData, BNO055_CALIB_STAT_ADDR) != 1) {
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
bool BNO055::getTemp(int8_t *temp) {
  return _busDevice->read8FromRegister((uint8_t *)temp, BNO055_CALIB_STAT_ADDR) != 1;
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
bool BNO055::getVector(adafruit_vector_type_t vector_type, imu::Vector<3> *xyz) {
  int16_t buffer[3];

  /* Read vector data */
  if (_busDevice->readArray16FromRegister((uint16_t *)buffer, sizeof(buffer) / sizeof(buffer[0]), (uint8_t)vector_type) != sizeof(buffer)) {
    return false;
  }
  if (!xyz) {
    return true;
  }

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  double coef = 1.0;
  switch (vector_type) {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    coef = 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    coef = 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    coef = 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    coef = 100.0;
    break;
  }
  (*xyz)[0] = ((double)buffer[0]) / coef;
  (*xyz)[1] = ((double)buffer[1]) / coef;
  (*xyz)[2] = ((double)buffer[2]) / coef;

  return true;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
bool BNO055::getQuat(imu::Quaternion *quat) {
  uint16_t buffer[4];

  /* Read quat data (8 bytes) */
  if (_busDevice->readArray16FromRegister((uint16_t *)buffer, sizeof(buffer) / sizeof(buffer[0]), BNO055_QUATERNION_DATA_W_LSB_ADDR) != sizeof(buffer)) {
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
  adafruit_bno055_opmode_t lastMode = _mode;
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }

  if (_busDevice->readArray16FromRegister((uint16_t *)offsets_type, sizeof(*offsets_type) / sizeof(offsets_type->accOffsetX), ACCEL_OFFSET_X_LSB_ADDR) != sizeof(*offsets_type)) {
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
  adafruit_bno055_opmode_t lastMode = _mode;
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);

  if (_busDevice->writeArray16ToRegister((uint16_t *)&offsets_type, sizeof(offsets_type) / sizeof(offsets_type.accOffsetX), ACCEL_OFFSET_X_LSB_ADDR) != sizeof(offsets_type)) {
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
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
bool BNO055::enterSuspendMode() {
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);
  if (_busDevice->write8ToRegister(0x02, BNO055_PWR_MODE_ADDR) != 1) {
    return false;
  }
  /* Set the requested operating mode (see section 3.3) */
  bool result = setMode(modeback);
  delay(20);
  return result;
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
bool BNO055::enterNormalMode() {
  adafruit_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OPERATION_MODE_CONFIG)) {
    return false;
  }
  delay(25);
  if (_busDevice->write8ToRegister(0, BNO055_PWR_MODE_ADDR) != 1) {
    return false;
  }
  /* Set the requested operating mode (see section 3.3) */
  bool result = setMode(modeback);
  delay(20);
  return result;
}
