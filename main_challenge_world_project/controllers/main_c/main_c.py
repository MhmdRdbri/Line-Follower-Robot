"""ta_make_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Lidar, Motor
import cv2
import numpy as np
import time

# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())

p_camera = robot.getDevice("p_camera")
p_camera.enable(timestep)

g_camera = robot.getDevice("g_camera")
g_camera.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

p_distance = robot.getDevice("p_distance")
p_distance.enable(timestep)

li_middle = robot.getDevice("li_m")
li_middle.enable(timestep)

li_left = robot.getDevice("li_l")
li_left.enable(timestep)

li_right = robot.getDevice("li_r")
li_right.enable(timestep)

left_middle = robot.getDevice("left_middle_dist")
left_middle.enable(timestep)

right_middle = robot.getDevice("right_middle_dist")
right_middle.enable(timestep)

up_right_li = robot.getDevice("up_right_li")
up_right_li.enable(timestep)

left_obstacle = robot.getDevice('left_ob')
left_obstacle.enable(timestep)

right_obstacle = robot.getDevice('right_ob')
right_obstacle.enable(timestep)

l_motor = robot.getDevice('left_motor')
l_motor.setPosition(float('inf'))
l_motor.setVelocity(0)

r_motor = robot.getDevice('right_motor')
r_motor.setPosition(float('inf'))
r_motor.setVelocity(0)

lidar = Lidar('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

lidar1 = Lidar('lidar(1)')
lidar1.enable(timestep)
lidar1.enablePointCloud()

lidar2 = Lidar('lidar(2)')
lidar2.enable(timestep)
lidar2.enablePointCloud()


flag = 0
flag2 = 0
while robot.step(timestep) != -1:
    threshold = 0.16
    threshold1 = 0.2
    color = 0
    obs = 0
    
    if flag == 1:
        right = 3
        left = -1
    else:
        right = 10
        left = 10
    
    dist_val = p_distance.getValue()
    print(dist_val)
    
    if dist_val < 950:
        lidar.disable()
        left_obstacle.disable()
        right_obstacle.disable()

    if dist_val < 20 and min(left_distances1) < 40:
        # flag = 1
        l_motor.setVelocity(0)
        r_motor.setVelocity(0)
        
        left_turn_speed = -3.0
        right_turn_speed = 3.0
        
        l_motor.setVelocity(left_turn_speed)
        r_motor.setVelocity(right_turn_speed)
        
        robot.step(int(1.90 * 1000))
        
            
        p_distance.disable()
        lidar.enable(timestep)
        left_obstacle.enable(timestep)
        right_obstacle.enable(timestep)
        
        l_motor.setVelocity(left)
        r_motor.setVelocity(right)



    distances = lidar.getRangeImage()
    left_distances = distances[:10]
    right_distances = distances[10:]
    print('left distances are : ', left_distances)
    print('right distances are: ', right_distances)
        
    gyroscope = gyro.getValues()
    print(gyroscope[1])

    image2 = g_camera.getImageArray()
    image2 = np.array(image2, dtype=np.uint8)
    hsv2 = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)
    
    image = p_camera.getImageArray()
    image = np.array(image, dtype=np.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])

    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])
    
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    
    blue_detected = cv2.bitwise_and(image, image, mask=mask_blue)
    red_detected = cv2.bitwise_and(image, image, mask=mask_red)
    green_detected = cv2.bitwise_and(image, image, mask=mask_green)
    
    mask_green2 = cv2.inRange(hsv2, lower_green, upper_green)
    green_detected2 = cv2.bitwise_and(image2, image2, mask=mask_green2)
    
    black_detected = cv2.bitwise_and(image, image, mask=mask_black)

    if np.any(blue_detected):
        print('Red')
        color = 1
        p_distance.enable(timestep)

    elif np.any(red_detected):
        print("Blue")
        color = 2

    elif np.any(green_detected):
        print("Green")   
        color = 3
        
    elif np.any(black_detected):
        print("Black")   
        color = 4
        
    if np.any(green_detected2):
        print("Green2")   
        color = 5
    
    print(color)
    
    
    
    left_value = li_left.getValue()
    right_value = li_right.getValue()
    middle_value = li_middle.getValue()
    
    left_middle_value = left_middle.getValue()
    right_middle_value = right_middle.getValue()
    
    print('left_middle_value value is : ' , left_middle_value)
    print('right_middle_value value is : ' , right_middle_value)
    
    right_ob_value = right_obstacle.getValue()
    left_ob_value = left_obstacle.getValue()
    print('right obstacle value is : ' , right_ob_value)
    print('left obstacle value is : ' , left_ob_value)
    
    #Up distance sensors
    up_right_value = up_right_li.getValue()
    # print('up right value is : ' , up_right_value)
    #-------------------
    
    print('right value is : ' , right_value)
    print('left value is : ' , left_value)
    print('middle value is : ' , middle_value)
    
    
    if middle_value > 300 and left_value < 300 and right_value < 300 and color == 0:
        right = 6
        left = 6
        
    if middle_value > 300 and left_value < 300 and right_value < 300 and color == 1:
        right = 6
        left = -3
        
    elif middle_value > 300 and left_value < 300 and right_value > 300:
        right = -3
        left = 6 
        
    elif middle_value < 300 and left_value < 300 and right_value > 300 and color == 0:
        right = -3
        left = 6
        
    elif middle_value < 300 and left_value < 300 and right_value > 300 and color == 1:
        right = 5
        left = -3
        
    elif middle_value < 300 and left_value < 300 and right_value > 300 and color == 4:
        right = 5
        left = -3
        
    elif middle_value < 300 and left_value > 300 and right_value < 300:
        right = 6
        left = -3
        
        
    elif middle_value < 300 and left_value > 300 and right_value > 300:
        right = 6
        left = -3
        
    # elif middle_value > 300 and left_value < 300 and right_value > 300:
        # right = 3
        # left = -1
        
    elif middle_value > 300 and left_value > 300 and right_value < 300 and color == 2:
        right = 1
        left = 5        

    elif middle_value > 300 and left_value > 300 and right_value < 300:
        right = 6
        left = -3

    elif middle_value > 300 and left_value > 300 and right_value < 300 and left_middle_value > 300:
        right = 6
        left = -3
        
    elif middle_value > 300 and left_value < 300 and right_value < 300 and left_middle_value > 300:
        right = 6
        left = -3
        
        #-------------------
    elif middle_value < 300 and left_value > 300 and right_value < 300 and left_middle_value > 300:
        right = -3
        left = 6
        
    elif middle_value < 300 and left_value < 300 and right_value < 300 and color == 1:
        right = 6
        left = -3

    # elif middle_value > 460 and left_value < 300 and right_value > 460:
        # right = -1
        # left = 3 
        
    # print('left distances are : ', left_distances)
    # print('right distances are: ', right_distances)
    
    distances1 = lidar1.getRangeImage()
    left_distances1 = distances1[10:]
    print('Left Goal', min(left_distances1))
    
    # distances2 = lidar2.getRangeImage()
    # print('Right Goal', min(distances2))
        
    min_right = min(right_distances)
    min_left = min(left_distances)
    print('min_right :' , min_right)
    print('min_left :' , min_left )
    
    if min_right < min_left and min_right < threshold :
        print("obstacle in right side, avoiding ...")
        right = 3
        left = 0
        
    elif min_left < min_right and min_left < threshold :
        print("obstacle in left side, avoiding ...")
        right = 0
        left = 3
        
    if left_ob_value < 510:
        right = 3
        left = 1.5
        
    if right_ob_value < 510:
        right = 1.5
        left = 3
    
    l_motor.setVelocity(left)
    r_motor.setVelocity(right)
    print("-------------------------------------------")