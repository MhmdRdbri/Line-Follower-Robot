from controller import Robot, Camera
import cv2
import numpy as np

# Create a robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Get the camera device
camera = robot.getDevice("right_camera")
camera.enable(timestep)

# Define color thresholds (adjust these values based on your camera and lighting conditions)
color_thresholds = {
    'red': ([0, 0, 100], [50, 50, 255]),
    'blue': ([100, 0, 0], [255, 50, 50]),
    'green': ([0, 100, 0], [50, 255, 50]),
    'black': ([0, 0, 0], [50, 50, 50])
}

# Initialize ball counts
ball_counts = {'red': 0, 'blue': 0, 'green': 0, 'black': 0}

# Main control loop
while robot.step(timestep) != -1:
    # Get the image from the camera
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

    # Extract RGB channels (discard alpha channel)
    image_rgb = image[:, :, :3]

    # Check for each color
    for color, thresholds in color_thresholds.items():
        # Extract color channels
        lower_bound, upper_bound = thresholds
        lower_bound = np.array(lower_bound, dtype=np.uint8)
        upper_bound = np.array(upper_bound, dtype=np.uint8)

        # Create a mask using inRange function from OpenCV
        color_mask = cv2.inRange(image_rgb, lower_bound, upper_bound)

        # Count non-zero pixels in the mask and update the count
        ball_counts[color] += np.sum(color_mask)
    print("Final Ball Counts: ", ball_counts)
# Print the final ball counts after the while loop

