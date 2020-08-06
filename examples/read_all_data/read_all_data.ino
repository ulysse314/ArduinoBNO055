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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
BNO055 bno;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Enable I2C */
  Wire.begin();
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  struct {
    BNO055::adafruit_vector_type_t type;
    const char *name;
  } vectorTypes[] = {
    { BNO055::VECTOR_EULER, "Euler" },
    { BNO055::VECTOR_GYROSCOPE, "Gyro" },
    { BNO055::VECTOR_LINEARACCEL, "Linear" },
    { BNO055::VECTOR_ACCELEROMETER, "Accel" },
    { BNO055::VECTOR_MAGNETOMETER, "Mag" },
    { BNO055::VECTOR_GRAVITY, "Grav" },
  };
  for (size_t ii = 0; ii < sizeof(vectorTypes) / sizeof(vectorTypes[0]); ++ii) {
    imu::Vector<3> vector;
    Serial.print(vectorTypes[ii].name);
    Serial.print(": ");
    if (bno.getVector(vectorTypes[ii].type, &vector)) {
      Serial.print("x= ");
      Serial.print(vector[0]);
      Serial.print(" | y= ");
      Serial.print(vector[1]);
      Serial.print(" | z= ");
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

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
