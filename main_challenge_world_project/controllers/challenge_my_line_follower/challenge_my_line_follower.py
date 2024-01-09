from controller import Robot, Lidar, Camera, DistanceSensor
import time
import cv2  
import numpy as np


robot = Robot()


############# motors ###############

fl_motor = robot.getDevice('fl_motor')
fr_motor = robot.getDevice('fr_motor')
bl_motor = robot.getDevice('bl_motor')
br_motor = robot.getDevice('br_motor')

############# line infrareds ###############

line_mid = robot.getDevice('line_middle')
line_mid.enable(64)


line_re = robot.getDevice('line_right')
line_re.enable(64)

line_le = robot.getDevice('line_left')
line_le.enable(64)


############# activate motors ###############

for motor in [fl_motor, fr_motor, bl_motor, br_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)


############# activate lidar and cam  ###############
lidar = Lidar("lidar")
lidar.enable(64)
lidar.enablePointCloud()

camera = Camera("cam")
camera.enable(64)  


############# side distance sensores ###############
left_dis = robot.getDevice("left_side")
left_dis.enable(64)

right_dis = robot.getDevice("right_side")
right_dis.enable(64)


############# image width and height ###############

image_width = camera.getWidth()
image_height = camera.getHeight()
image = None

while robot.step(64) != -1:
    left = 4
    right = 4

################# line infra values #############################################

    line_mid_values = line_mid.getValue()
    print ('mid infra value is : ',line_mid_values)
    
    
    line_re_values = line_re.getValue()
    print ('right infra value is : ',line_re_values)

    line_le_value = line_le.getValue()
    print ('left infra value is : ',line_le_value)    


############### lidar left and right values #################

    distances = lidar.getRangeImage()
    threshold = 0.17
    
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
    
    
####################### logic ############################   





        
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
    
    
    
