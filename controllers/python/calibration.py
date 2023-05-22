#!/usr/bin/env python3     
import time
import numpy as np
from controller import *
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from tf import TransformBroadcaster

def send_image(bridge, imglr_pub, imgr):
    # Concatenate images L and R
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(imgr, 'bgr8')
    msglr.header.stamp = rospy.Time.now();
    msglr.header.frame_id = "camera_link"
    imglr_pub.publish(msglr)
    return None

def camera_system(robot, name, timestep):
    # System configuration for camera
    camera = robot.getDevice(name)
    camera.enable(timestep)
    return camera

def get_image(camera):
    # Adquisition of camera information
    data = camera.getImage()

    # Decoding image more faster that others metods
    img = np.frombuffer(data, np.uint8)

    # Resize the image to the respective dimesions
    aux = img.reshape(camera.getHeight(), camera.getWidth(), 4)

    # Convert image to the respective type of open cv
    frame = cv2.cvtColor(aux, cv2.COLOR_BGRA2BGR)
    return frame


def main(robot, image_pu_rgb_l, image_pu_rgb_r):
    # Get time Step
    time_step = int(robot.getBasicTimeStep()) 

    # Sample Time Defintion
    sample_time = 0.01

    # Frequency of the simulation
    hz = int(1/sample_time)

    # Rate Ros Node
    loop_rate = rospy.Rate(100)

    # Robot get Node
    supervisor_node = robot.getFromDef('matrice')

    # Camera Definitions
    camera_rgb_r = camera_system(robot, "camera_r", time_step)
    camera_rgb_l = camera_system(robot, "camera_l", time_step)

    # Time defintion
    t = 0
    # Configuration Bridge
    bridge = CvBridge()
    
    # Initial Rotation system
    while robot.step(time_step) != -1:
        tic = time.time()
        # Position and anles from the callback
        
        # Get image
        img_rgb_r = get_image(camera_rgb_r)
        img_rgb_l = get_image(camera_rgb_l)


        # Wait Ros Node and update times
        loop_rate.sleep()
        delta = time.time()- tic
        t = t + delta
        # Send Images
        send_image(bridge, image_pu_rgb_l, img_rgb_l)
        send_image(bridge, image_pu_rgb_r, img_rgb_r)
        
    return None

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("vision_system",disable_signals=True, anonymous=True)

        # Conection Webots External
        robot_a = Supervisor()

        # Vision Topic 
        image_topic_rbg_l = "/camera_l/color/image_raw"
        image_topic_rgb_r = "/camera_r/color/image_raw"
        image_publisher_rgb_l = rospy.Publisher(image_topic_rbg_l, Image, queue_size=1)
        image_publisher_rgb_r = rospy.Publisher(image_topic_rgb_r, Image, queue_size=1)


        # Simulation 
        main(robot_a, image_publisher_rgb_l, image_publisher_rgb_r)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass