# DARTV2 Challenge 2022 - Group xXDartSasukeXx




### Authors

- Louis Gillard - [louis.gillard@ensta-bretagne.org](https://outlook.office.com/mail) (Promotion ENSTA Bretagne 2024 Speciality Autonomous Robotics)
- Adam Goux--Gateau - [adam.goux--gateau@ensta-bretagne.org](https://outlook.office.com/mail) (Promotion ENSTA Bretagne 2024 Speciality Autonomous Robotics)

### Description

This project consists in the automation of a ground robot, the "DART". It will have to travel a path in a maze using only its ultrasonic sensors and odometers to guide itself. The programming will be done in Python 3 language.

### Note(s)

Project finished on 06/01/2023, Robotics 2nd year ENSTA Bretagne.

## Contents
- [Git structure](#structure-du-git)
- [General information](#general-information)
  - [Project Status](#project-status)
  - [Work done](#work-done)
  - [Work in Progress](#work-in-progress)
- [User Guide](#user-guide)

## Git structure
The codes are in the folder py
  
## General Information  
  
  ### Project Status
    
100%  
Functional simulations, real robot meets specifications 

  ### Work done
This README was written, and we got familiar with V-REP and git.
Set up a log display system.

Validation of our simulation, switch to real DART. Straight line mastered, as well as detection of walls in front of the robot. 
Use of the compass for the turns. 
Robot effective, it manages to correct its course and makes its turns using the compass.
  
  ### Work in progress
None
## User guide


Connect to the robot with a SSH command in a Linux shell : 
```console 
$ ssh username@hostname
```
Then use a file transfer software like Filezilla to send the files to the robot. You can also clone the git inside it, however, it could take time to commit/pull/push.
Finally, once you are connected to the robot, move to the py directory, and execute the python programm you want to run :
```console
$ python3 my_code.py
```
