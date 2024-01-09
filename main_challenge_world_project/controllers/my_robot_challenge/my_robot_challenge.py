from controller import Robot, Lidar, Camera, DistanceSensor
import time
import cv2  
import numpy as np

# Create the Robot instance.
robot = Robot()

# Initialize motors
fl_motor = robot.getDevice('fl_motor')
fr_motor = robot.getDevice('fr_motor')
bl_motor = robot.getDevice('bl_motor')
br_motor = robot.getDevice('br_motor')

# Set motors to infinite position and zero velocity
for motor in [fl_motor, fr_motor, bl_motor, br_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    
# Initialize left and right distance sensors
left_dis = robot.getDevice("left_side")
left_dis.enable(64)

right_dis = robot.getDevice("right_side")
right_dis.enable(64)

left_infra = robot.getDevice('left_infra')
left_infra.enable(64)

right_infra = robot.getDevice('right_infra')
right_infra.enable(64)

middle_infra = robot.getDevice('middle_infra')
middle_infra.enable(64)

    
# Initialize Lidar
lidar = Lidar("lidar")
lidar.enable(64)
lidar.enablePointCloud()

# Main loop:
# Perform simulation steps until Webots stops the controller
while robot.step(64) != -1:
    left = 10  # Default velocity for left motor
    right = 10  # Default velocity for right motor
    
    left_infra_value = left_infra.getValue()
    print ('left infra value : ',left_infra_value)
    
    right_infra_value = right_infra.getValue()
    print ('right infra value : ',right_infra_value)
    
    middle_infra_value = middle_infra.getValue()
    print ('middle infra value : ',middle_infra_value)
        
    # Get range data from Lidar
    distances = lidar.getRangeImage()
    threshold = 0.13
    
    # Process left and right distance data separately
    left_distances = distances[:10]
    right_distances = distances[10:]
    right_min = min(right_distances)
    left_min = min(left_distances)    
    distance_side_left = left_dis.getValue()
    distance_side_right = right_dis.getValue()   
     
    # Replace 'inf' values with a high value for comparison
    for i in range(10):
        if left_distances[i] == float('inf'):
            left_distances[i] = 999    
    
    for i in range(10):    
        if right_distances[i] == float('inf'):
            right_distances[i] = 999    
    
    min_left = min(left_distances)
    min_right = min(right_distances)
    
    print("right min is:", right_min)
    print("left min is:", left_min)    
    
    # Obstacle avoidance logic
    if min_left < min_right and min_left < threshold :
        print("Obstacle in left side, avoiding...")
        right = -3
        left = 3
        
    elif min_right < min_left and min_right < threshold:
        print("Obstacle in right side, avoiding...")
        right = 3
        left = -3        
        
    else:
        pass
    
    # Circle around if an obstacle is detected on the right side
    if distance_side_right < 250 and not min_right < threshold :
        print("Circle around")
        right = -3
        left = 3
    
    print("Left Side Distances:", distance_side_left)
    print("Right Side Distances:", distance_side_right)

    # Set motor velocities
    fl_motor.setVelocity(left)
    fr_motor.setVelocity(right)
    bl_motor.setVelocity(left)
    br_motor.setVelocity(right)

    print("-----------------------------------------------------------------------")
