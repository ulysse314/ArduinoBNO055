#include <Wire.h>
#include <BNO055.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 500;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
BNO055 bno;

void setup(void)
{
  Serial.begin(115200);

  /* Enable I2C */
  Wire.begin();
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  BNO055::AxesConfiguration axesConfiguration;
  axesConfiguration = BNO055::getAxesConfigurationForDevicePlacement(BNO055::DevicePlacement::P5);
  bno.setAxesRemap(axesConfiguration);
}

void loop(void)
{
  struct {
    BNO055::Vector vector;
    const char *name;
  } vectorTypes[] = {
    { BNO055::Vector::Euler,                 "Euler " },
    { BNO055::Vector::Gyroscope,             "Gyro  " },
    { BNO055::Vector::LinearAccelerometer,   "Linear" },
    { BNO055::Vector::Accelerometer,         "Accel " },
    { BNO055::Vector::Magnetometer,          "Mag   " },
    { BNO055::Vector::Gravity,               "Grav  " },
  };
  for (size_t ii = 0; ii < sizeof(vectorTypes) / sizeof(vectorTypes[0]); ++ii) {
    imu::Vector<3> vector;
    Serial.print(vectorTypes[ii].name);
    Serial.print(": ");
    if (bno.getVector(vectorTypes[ii].vector, &vector)) {
      Serial.print("x= ");
      Serial.print(vector[0]);
      Serial.print(" |\ty= ");
      Serial.print(vector[1]);
      Serial.print(" |\tz= ");
      Serial.println(vector[2]);
    } else {
      Serial.println("failure");
    }
  }

  Serial.print("temperature: ");
  int8_t boardTemp;
  if (bno.getTemperature(&boardTemp)) {
    Serial.println(boardTemp);
  } else {
    Serial.println("failure");
  }

  BNO055::DeviceInfo deviceInfo;
  if (bno.getDeviceInfo(&deviceInfo)) {
    Serial.print("Accelerometer chip ID:\t");
    Serial.println(deviceInfo.accelerometerChipID, HEX);
    Serial.print("Magnetometer chip ID:\t");
    Serial.println(deviceInfo.magnetometerChipID, HEX);
    Serial.print("Gyroscope chip ID:\t");
    Serial.println(deviceInfo.gyroscopeChipID, HEX);
    Serial.print("Software revision:\t");
    Serial.println(deviceInfo.softwareRevision);
    Serial.print("Bootloader revision:\t");
    Serial.println(deviceInfo.bootloadRevision);
  } else {
    Serial.println("Device info failure");
  }

  BNO055::SystemStatus systemStatus;
  if (bno.getSystemStatus(&systemStatus)) {
    Serial.print("System status: ");
    switch (systemStatus) {
    case BNO055::SystemStatus::Idle:
      Serial.println("idle");
      break;
    case BNO055::SystemStatus::Error:
      Serial.println("error");
      break;
    case BNO055::SystemStatus::InitializingPeripherals:
      Serial.println("initializing peripherals");
      break;
    case BNO055::SystemStatus::SystemInitialization:
      Serial.println("system initialization");
      break;
    case BNO055::SystemStatus::ExecutingSelfTest:
      Serial.println("executing self test");
      break;
    case BNO055::SystemStatus::RunningWithFusionAlgorithm:
      Serial.println("running with fusion algorithm");
      break;
    case BNO055::SystemStatus::RunningWihtoutFusionAlgorithm:
      Serial.println("running without fusion algorithm");
      break;
    }
  } else {
    Serial.println("System status failed");
  }
  BNO055::SystemError systemError;
  if (bno.getSystemError(&systemError)) {
    Serial.print("System error: ");
    switch (systemError) {
    case BNO055::SystemError::NoError:
      Serial.println("no error");
      break;
    case BNO055::SystemError::PeripheralInitializationError:
      Serial.println("peripheral initialiazation error");
      break;
    case BNO055::SystemError::SystemInitializationError:
      Serial.println("system initialiazation error");
      break;
    case BNO055::SystemError::SelfTestResultFailed:
      Serial.println("self test result failed");
      break;
    case BNO055::SystemError::RegisterMapValueOutOfRange:
      Serial.println("register map value out of range");
      break;
    case BNO055::SystemError::RegisterMapAddressOutOfRange:
      Serial.println("register map address out of range");
      break;
    case BNO055::SystemError::RegisterMapWriteError:
      Serial.println("register map write error");
      break;
    case BNO055::SystemError::LowPowerNotAvailableForSelectedOperationMode:
      Serial.println("low power not available for selected operation mode");
      break;
    case BNO055::SystemError::AccelerometerPowerModeNotAvailable:
      Serial.println("accelerometer power mode not available");
      break;
    case BNO055::SystemError::FusionAlgorithmConfigurationError:
      Serial.println("fusion algorithm configuraton error");
      break;
    case BNO055::SystemError::SensorConfigurationError:
      Serial.println("sensor configuration error");
      break;
    }
  } else {
    Serial.println("System error failed");
  }
  bool accelerometerSelfTest, magnetometerSelfTest, gyroscopeSelfTest, microcontrollerSelfTest;
  if (bno.getSelfTestResult(&accelerometerSelfTest, &magnetometerSelfTest, &gyroscopeSelfTest, &microcontrollerSelfTest)) {
    Serial.print("Self test, acc: ");
    Serial.print(accelerometerSelfTest);
    Serial.print(", mag: ");
    Serial.print(magnetometerSelfTest);
    Serial.print(", gyr: ");
    Serial.print(gyroscopeSelfTest);
    Serial.print(", mcu: ");
    Serial.println(microcontrollerSelfTest);
  } else {
    Serial.println("Self test result failed");
  }
  uint8_t system, gyro, accel, mag = 0;
  if (!bno.getCalibration(&system, &gyro, &accel, &mag)) {
    Serial.println("Failed to get calibration");
  } else {
    Serial.print("Calibration: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
  }
  Serial.println("--");

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
