## DARTV2 drivers v2

### Magnetic heading sensor

Here we use a fast and dirty 2D calibration procedure. The robot will turn in place for a given duration at a given speed.

```
python fast_fast_heading_calibration 100 60.0
```
Here the motor speed (PWM command) is 100 and the duration is 60 seconds. You can modify these parameters if needed.
The idea is to make at least two revolutions (720 degrees).
At the end of the execution, you will get the minimum and maximum values of the magnetic measurements along x and y.
These values are displayed as follows :

```
min,max : 459 5485 -7046 -2233
```
In your program, you can calibrate the heading sensor with the function *fast_heading_calibration* from the *imu* driver :
```
import drivers_v2 as drv2
mybot = drv2.DartV2DriverV2()
magx_min = 459
magx_max = 5485
magy_min = -7046
magy_max = -2233
mybot.imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)
```
This has to be done before using the heading sensor.
Then, in your program, you can get the calibrated heading in radians with :
```
mag = mybot.imu.read_mag_raw() # get raw magnetic heading sensor values
heading = mybot.imu.heading(mag[0],mag[1])
```
If you prefer to work with degrees, you can do :
```
mag = mybot.imu.read_mag_raw() # get raw magnetic heading sensor values
heading = mybot.imu.heading_deg(mag[0],mag[1])
```