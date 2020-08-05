#include <Wire.h>
#include <BNO055.h>

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
BNO055 bno = BNO055(BNO055::Address::ToLow);

void setup(void)
{
  Serial.begin(115200);
  /* Enable I2C */
  Wire.begin();
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }


  delay(1000);
}

void loop(void)
{
  //
  unsigned long tStart = micros();
  imu::Vector<3> orientationData;
  bno.getVector(BNO055::VECTOR_EULER, &orientationData);
  //  bno.getEvent(&angVelData, BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linearAccelData;
  bno.getVector(BNO055::VECTOR_LINEARACCEL, &linearAccelData);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData[0];
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData[1];

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData[0] / cos(DEG_2_RAD * orientationData[0]);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(orientationData[0]);
    Serial.print("Position: ");
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
    Serial.print("Speed: ");
    Serial.println(headingVel);
    Serial.println("-------");

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }



  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}
