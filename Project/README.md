# Robotics_I:_Analysis_Control_Laboratory
Kinematic analysis of robot manipulator, forward and inverse geometric modeling for "Robotics I" course NTUA 2019-2020


## Abstract

The main purpose of this exercise was the kinematic alanysis of a Robotic Manipulator  so as to perform a straigh movement between two points

## Robotic Manipulator

This robotic manipulator has 3 Rotational DOF (as a common human-like arm) and shows up in the below image:

![1](https://user-images.githubusercontent.com/50829499/111982512-bb6b2c00-8b11-11eb-9c74-63441d88ece5.png)

## Theorytical Analysis 

### Task Space

First of all we check if it possible for arm to reach A and B points (these points must be inside of workspace without a danger of singular configurations).

#### Path planing

After we define a 3rd polynomial that ensures velocity smoothness:

![2](https://user-images.githubusercontent.com/50829499/111986837-1ce1c980-8b17-11eb-844d-e9489521de9a.png)

About position the variables pe_x, pe_y and pe_z are during time:

![3](https://user-images.githubusercontent.com/50829499/111988139-c6758a80-8b18-11eb-827b-ac7cf588e979.png)

About velocity the variables pe_x, pe_y and pe_z are during time:

![3](https://user-images.githubusercontent.com/50829499/111988815-a5616980-8b19-11eb-8b09-e6deb2590044.png)

As we see, there is no discontinuity in these curves and no "conrner points",as desired.

### Joint Space

After finding desired position/velocity of end effector (each time), we should identify which must be joint's angular positions to ensure these. We broke this task into two subtasks 

#### Subtask 1: From 2nd rotational joint to end effector 

The equations that describe poistion of end - effector about 2nd joint's coordinates system are:
![5](https://user-images.githubusercontent.com/50829499/111989972-1d7c5f00-8b1b-11eb-82c0-cf9307ecc18c.png)


#### Subtask 2: From start to 2nd rotational joint 
 
Finally after reduction below equations to base coordinates' system we take 

![5](https://user-images.githubusercontent.com/50829499/111991459-076f9e00-8b1d-11eb-92ed-102271e8e79c.png)

that are final equations from base to end -effector
 
## Simulation 

The above equations, have beenn included to matlab code in order to simulate a straight forward movement. As we see the robotic arm
performs desired movement through points.

![6](https://user-images.githubusercontent.com/50829499/111991587-266e3000-8b1d-11eb-9ec0-195cfaf5a193.png)





