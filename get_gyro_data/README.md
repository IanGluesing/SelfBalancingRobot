# Interacting with MPU6050 for X-axis rotational value


## Read in initial rotation velocities around XYZ axis'

There are a few different ways to read in rotational values using the MPU6050 api. This can be done either through getRotation(&x, &y, &z) or for a specific axis, getRotationX(), getRotationY(), and getRotationZ(). These functions will return a 16 bit signed int corresponding to raw rotational values directly from the MPU6050. To convert these values to degrees, divide by the corresponding Sensitivity Scale Factor your MPU6050 is configured to. 

To get this value, run MPU6050_Object.getFullScaleGyroRange(). This will return the Full-Scale Range value your MPU6050 is configured for. Based on the spec, a Gyroscopic FS_SEL value of 0 corresponds to a Sensitivity Scale Factor of 131, so to convert a raw gyroscope value to degrees/sec, divide by 131.

## Read in acceleration values along XYZ axis'

To read in acceleration data from the MPU6050, use the getAcceleration(&x, &y, &z) interface call. Like with rotational data, if you need to convert to G's or m/s/s, you will need to get the AFS_SEL value using the getFullScaleAccelRange() function call, and divide by the corresponding Sensitivity Scale Factor value in the Accelerometer Sensitivity section of the MPU6050 spec

## Determining rotational angle from acceleration data

Specifically for this self balancing robot problem, we only care about the 'angle of gravity' in the YZ-plane. This angle can be used to determine current rotation around the X-axis. 

We will define this angle as 

![Theta equation](https://latex.codecogs.com/png.image?\dpi{110}&space;\color{White}\Theta=\text{atan2}(a_y,a_z))

Where ![a_y](https://latex.codecogs.com/png.image?\dpi{110}&space;\color{White}a_y) is the acceleration along the Y-axis, and ![a_z](https://latex.codecogs.com/png.image?\dpi{110}&space;\color{White}a_z) is the acceleration along the Z-axis

## Simplified Complimentary Filter for noise and error reduction

Up to this point we now have two different measurements for the rotational angle around the X-axis. These can be combined using a simplified complimentary filter to allow for noise and error reduction

angle_estimate = (1 - alpha) * (previous_angle + gyroscope_reading_x * dt) + (alpha * angle_of_gravity)

Where alpha is the weight factor for gyroscope readings