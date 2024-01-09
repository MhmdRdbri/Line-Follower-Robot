from controller import Robot, Lidar, Camera, DistanceSensor
import time
import cv2  
import numpy as np


robot = Robot()


fl_motor = robot.getDevice('fl_motor')
fr_motor = robot.getDevice('fr_motor')
bl_motor = robot.getDevice('bl_motor')
br_motor = robot.getDevice('br_motor')

infra_dis = robot.getDevice('disinfra')
infra_dis.enable(64)

for motor in [fl_motor, fr_motor, bl_motor, br_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)


lidar = Lidar("lidar")
lidar.enable(64)
lidar.enablePointCloud()

camera = Camera("cam")
camera.enable(64)  


left_dis = robot.getDevice("left_side")
left_dis.enable(64)

right_dis = robot.getDevice("right_side")
right_dis.enable(64)


image_width = camera.getWidth()
image_height = camera.getHeight()
image = None

while robot.step(64) != -1:


    infra_value= infra_dis.getValue()
    print ('infra value is : ',infra_value)



    
    left = 4
    right = 4


    distances = lidar.getRangeImage()
    threshold = 0.13
    
    left_distances = distances[:10]
    right_distances = distances[10:]

    right_min = min(right_distances)
    left_min = min(left_distances)

    distance_side_left = left_dis.getValue()
    distance_side_right = right_dis.getValue()

    
    for i in range(10):
        if left_distances[i] == float('inf'):
            left_distances[i] = 999    
    
    for i in range(10) :    
        if right_distances[i] == float('inf'):
           right_distances[i] = 999
    
    
    min_left = min(left_distances)
    min_right = min(right_distances)
    print("right min is:", right_min)
    print("left min is:", left_min)
    
    if min_left < min_right and min_left < threshold :
        print("obstacle in left side , avoiding...")
        right = -3
        left = 3
        
    elif min_right < min_left and min_right < threshold:
        print("obstacle in right side , avoiding...")
        right = 3
        left = -3        
        
    else :pass
   
    if distance_side_right < 250 and not min_right < threshold :
        print("circle around")
        right = -3
        left = 3
    
    
    print("Left Side Distances:", distance_side_left)
    print("Right Side Distances:", distance_side_right)




        
    # image = camera.getImage()
    # image_array = np.frombuffer(image, dtype=np.uint8).reshape((image_height, image_width, 4))
    
    # image_rgb = camera.getImageArray()
    # if image_rgb:
       
        # for x in range(0,camera.getWidth()):
            # for y in range(0,camera.getHeight()):
                # red   = image_rgb[x][y][0]
                # green = image_rgb[x][y][1]
                # blue  = image_rgb[x][y][2]
                # gray  = (red + green + blue) / 3
                # print('r='+str(red)+' g='+str(green)+' b='+str(blue))

    # Save images with a timestamped name
    # timestamp = time.strftime("%H-%M-%S-") + str(int(time.time() * 1000) % 1000)  # Milliseconds added
    # image_filename = f'ImageDataset/{timestamp}.png'
    # cv2.imwrite(image_filename, image_array)

    
    fl_motor.setVelocity(left)
    fr_motor.setVelocity(right)
    bl_motor.setVelocity(left)
    br_motor.setVelocity(right)

    print("-----------------------------------------------------------------------")
    
    
    
