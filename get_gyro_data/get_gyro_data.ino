#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

// Estimated current angle around the X axis
float angleAroundXAxis;

// Acceleration of the MPU in direction of the given axis
int16_t accelerationAlongXAxis, accelerationAlongYAxis, accelerationAlongZAxis;

// Sample rate
float dt = 0.01;

// Weight
float weight_factor = 0.05;

#define MPU_ADDR 0x68  // Default I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  mpu.initialize();

  delay(1000);
  Serial.println(String(mpu.getFullScaleGyroRange()));
}

void loop() {
  // Rotational velocity around X axis, divide by 131 to account for FS_SEL = 0 to convert from rotational units to degrees/sec
  float rotationalVelocityAroundXAxis = mpu.getRotationX() / 131.0; // Units: Degrees/Sec

  // Get acceleration along the X, Y, and Z axis. These are given in accelerometer units.
  mpu.getAcceleration(&accelerationAlongXAxis, &accelerationAlongYAxis, &accelerationAlongZAxis);

  // Determine 'angle of gravity' using acceleration values along Y and Z axis
  // Use this in tandem with the read rotation to determine best guess rotation value
  float angleOfGravityFromZAxis = atan2(accelerationAlongYAxis, accelerationAlongZAxis) * (180.0 / PI); // Angle of Gravity WRT Y anx Z axis

  // To determine the actual final angle, use simplified complimentary filter
  angleAroundXAxis = (1 - weight_factor) * (angleAroundXAxis + rotationalVelocityAroundXAxis * dt) + (weight_factor * angleOfGravityFromZAxis);
  
  Serial.println("angleAroundXAxis: " + String(angleAroundXAxis));

  delay(1000 * dt);
}
