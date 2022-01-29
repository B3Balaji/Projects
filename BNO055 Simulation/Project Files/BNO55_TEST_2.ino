#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v2.h>

/*
 * Last Edited: 29/01/22
 * Author: Bobby Balaji
 * Reference: https://adafruit.github.io/Adafruit_BNO055/html/_adafruit___b_n_o055_8h.html
 */

/*
 * Const Defines
 */
#define SAMPLE_RATE_MS  100

/*
 * Global Variables
 */
Adafruit_BNO055 myIMU = Adafruit_BNO055(-1, 0x28);
unsigned long currentTime;
unsigned long previousTime;
double differentialTime;
double platformVelocity[3];
double platformDisplacement[3];
/*
 * Run Once
 */
void setup() 
{
  Wire.begin();
  Serial.begin(9600);
  while(myIMU.begin() == false);
  Serial.print("IMU Setup");

  previousTime = 0;
  currentTime = 0;   
}

/*
 * Run Continuously
 */
void loop() 
{
  
  /* deg */
  imu::Vector<3> eulerAngle = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* quaternion */
  imu::Quaternion quaternionData = myIMU.getQuat();
  
  /* rad/s */
  imu::Vector<3> angularVelocity = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* m/s/s w/o gravity */
  imu::Vector<3> isolatedLinearAcceleration = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* m/s/s gravity only */
  imu::Vector<3> gravityAcceleration = myIMU.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

//  calibrate_IMU();
//    stabilize_system(float targetYaw, float targetPitch, float targetRoll, imu::Vector<3> eul);
      position_estimation( isolatedLinearAcceleration );
      
//  serial_communication(quaternionData, isolatedLinearAcceleration, eulerAngle);
  
  
}

void calibrate_IMU()
{
 
  uint8_t calStatus = 0;
  uint8_t gyroCal = 0;
  uint8_t accelCal = 0;
  uint8_t magCal = 0;
  
  myIMU.getCalibration(&calStatus, &gyroCal, &accelCal, &magCal);
  Serial.println();
  Serial.print("Calibration: Overall=");
  Serial.print(calStatus);
  Serial.print(" Gyro=");
  Serial.print(gyroCal);
  Serial.print(" Accel=");
  Serial.print(accelCal);
  Serial.print(" Mag=");
  Serial.println(magCal);
}

void stabilize_system(float targetYaw, float targetPitch, float targetRoll, imu::Vector<3> eul)
{
//  static long previousTime;
//  static double errorSum[3];
//  if(previousTime == NULL)
//  {
//    previousTime = 0;
//    errorSum[0] = 0;
//    errorSum[1] = 0;
//    errorSum[2] = 0;
//  }
//  unsigned long currentTime = millis();
//  unsigned long deltaTime = currentTime - previousTime;
//  if(deltaTime >= 50)
//  {
//    double errorYaw = targetYaw - eul.z();
//    double errorPitch = targetPitch - eul.y();
//    double errorRoll = targetRoll - eul.x();
//
//    errorSum[0] += errorYaw;
//    errorSum[1] += errorPitch;
//    errorSum[2] += errorRoll;
//    
//    /* Control Equation */
//  }
  
 
}

void serial_communication(imu::Quaternion quat, imu::Vector<3> linacc, imu::Vector<3> eul)
{
  Serial.print(quat.w(), 4);
  Serial.print(", ");
  Serial.print(quat.x(), 4);
  Serial.print(", ");
  Serial.print(quat.y(), 4);
  Serial.print(", ");
  Serial.print(quat.z(), 4);
  
  Serial.print(", ");
  
  Serial.print(eul.x());
  Serial.print(", ");
  Serial.print(eul.y());
  Serial.print(", ");
  Serial.println(eul.z());
  
}

void position_estimation(imu::Vector<3> linacc)
{
  previousTime = currentTime;
  currentTime = millis();
  differentialTime = (currentTime-previousTime)/1000;
   Serial.println(differentialTime);
  
  platformVelocity[0] += linacc.x() * differentialTime;
  platformVelocity[1] += linacc.y() * differentialTime;
  platformVelocity[2] += linacc.z() * differentialTime;

  platformDisplacement[0] += platformVelocity[0] * differentialTime;
  platformDisplacement[1] += platformVelocity[1] * differentialTime;
  platformDisplacement[2] += platformVelocity[2] * differentialTime;

//  Serial.println(linacc.x());
}
