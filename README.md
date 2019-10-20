# AutomationSensationABJ
Warehouse simulation by using particle filtering, VREP, and ROS

# Instructions
In the simulation, we have two objects: the YouBot and an object. The user is able to use q, w, e, a, s, d, and W to control the YouBot. As for the object, its purpose is for the user to test the proximity sensor, LIDAR, that is attached on the front of the robot. When the user is moving the YouBot around the freespace, the console would continuously output the reading values of the proximity sensor. 

## Key controller:
w: Forward
a: Left
s: Back
d: Right
q: Left rotation
e: Right rotation
W: Forward with faster speed
 

# YouBot
The child script of the YouBot functions primarily through a keylistener. When a key is hit, the velocities of the front left, front right, rear left, and rear right wheels are set to specific values, depending on the intended direction. This causes the YouBot to move in said direction. The robot doesn’t coast, but rather, it brakes once the key is lifted.


# Proximity_sensor
	The robot uses a Vrep proximity_sensor object of type ray to simulate a lidar sensor. Using the object script, we can output the measurements to the user. 
Moving forward, our goal is to output the measurements to ROS to minimize roll of simulator and use ROS packages for motion planning.

# Forward Kinematics
This robot uses forward kinematics to calculate the (X,Y,Z) coordinates of the end effector of its arm. The data for the calculations is located in transformation_data.py. 

# Using VREP remotely through Python
Vrep Python API:
http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
Only edit roboProj.py, and/or helper files.
