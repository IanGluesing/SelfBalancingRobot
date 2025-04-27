#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

// Estimated current angle around a given axis
float angleAroundXAxis, angleAroundYAxis, angleAroundZAxis;

// Truth samples read from the MPU
int16_t rotationalVelocityAroundXAxis, rotationalVelocityAroundYAxis, rotationalVelocityAroundZAxis;
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
  // Rotational velocity around axis
  mpu.getRotation(&rotationalVelocityAroundXAxis, &rotationalVelocityAroundYAxis, &rotationalVelocityAroundZAxis);

  // Add change in position to overall position, pos += velocity * change in time
  // Divide by 131 to get degrees/sec from rotational units from mpu
  angleAroundXAxis += rotationalVelocityAroundXAxis / 131.0 * dt;
  angleAroundYAxis += rotationalVelocityAroundYAxis / 131.0 * dt;
  angleAroundZAxis += rotationalVelocityAroundZAxis / 131.0 * dt;
  
  // Accelerations along an axis, raw accelerometer Units, to convert to Gs, divide by 16384
  // seems to be, how vertical are these axis, closer to vertical, closer to 16384
  // this makes sense, as if you divide by 16384 to get Gs, it would be 1G, which is straight down
  mpu.getAcceleration(&accelerationAlongXAxis, &accelerationAlongYAxis, &accelerationAlongZAxis);

  // Convert to float to allow for values to fit in more than 16 bits
  float temp_accelerationAlongXAxis = accelerationAlongXAxis;
  float temp_accelerationAlongYAxis = accelerationAlongYAxis;
  float temp_accelerationAlongZAxis = accelerationAlongZAxis;

  // Determine 'angle of gravity' using acceleration values along Y and Z axis
  // Use this in tandem with the read rotation to determine best guess rotation value
  float angleOfGravityFromZAxis = atan2(temp_accelerationAlongYAxis, temp_accelerationAlongZAxis) * (180.0 / PI); // Angle of Gravity WRT Y anx Z axis

  // To determine the actual final angle, 
  angleAroundXAxis += weight_factor * (angleOfGravityFromZAxis - angleAroundXAxis);
  
  Serial.println("angleAroundXAxis: " + String(angleAroundXAxis));

  delay(1000 * dt);
}
