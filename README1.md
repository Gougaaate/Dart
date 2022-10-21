# DartV2 Challenge 2021

## DartV2 - Lab1 - Part 3 - Low Level Control

### Several small programs are given in the **py** folder :

robot_cmd_1_qx.py : (x=1..9) examples of programs to solve questions 1 to 9 of the low level control lab.

fast_heading_calibration.py : a very ... very very simple 2D calibration of the compass by making the robot turning in places to get min and max values of the
horizontal magnetic field (these values, different for each dart, can be entered in the imu9 driver)

stop.py just stops the robot and indicate the battery voltage

manual_control.py  controls manually the robot with the laptop keyboard (ESC to exit, all other keys are defined in the program)

### Executing a program on the real robot 

Asuming that your group is grp-dum, the program to test is stop.py, the number of the dart is 7, the user and passwd are uv27 :

``` bash
$ scp -r py uv27@172.20.25.107:grp-dum
```

Then in another terminal, log in the dart using ssh with user uv27 (passwd uv27)

``` bash
$ ssh uv27@172.20.25.107
$ cd grp-dum/py
$ python3 stop.py
```


## DartV2 - Lab1 - Part 3 - Motion Control

**Now describe your work here ...**
