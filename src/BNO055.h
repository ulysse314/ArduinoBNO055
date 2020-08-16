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

  /** BNO055 power settings */
  typedef enum {
    POWER_MODE_NORMAL = 0x00,
    POWER_MODE_LOWPOWER = 0x01,
    POWER_MODE_SUSPEND = 0x02
  } adafruit_bno055_powermode_t;

  /** Operation mode settings **/
  typedef enum {
    OPERATION_MODE_CONFIG = 0x00,
    OPERATION_MODE_ACCONLY = 0x01,
    OPERATION_MODE_MAGONLY = 0x02,
    OPERATION_MODE_GYRONLY = 0x03,
    OPERATION_MODE_ACCMAG = 0x04,
    OPERATION_MODE_ACCGYRO = 0x05,
    OPERATION_MODE_MAGGYRO = 0x06,
    OPERATION_MODE_AMG = 0x07,
    OPERATION_MODE_IMUPLUS = 0x08,
    OPERATION_MODE_COMPASS = 0x09,
    OPERATION_MODE_M4G = 0x0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    OPERATION_MODE_NDOF = 0x0C
  } adafruit_bno055_opmode_t;

  /** Remap settings **/
  typedef enum {
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
  } adafruit_bno055_axis_remap_config_t;

  /** Remap Signs **/
  typedef enum {
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
  } adafruit_bno055_axis_remap_sign_t;

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

  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  bool checkChipID();
  bool setMode(adafruit_bno055_opmode_t mode);
  bool setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode);
  bool setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign);
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
  adafruit_bno055_opmode_t _mode;
};

#endif
