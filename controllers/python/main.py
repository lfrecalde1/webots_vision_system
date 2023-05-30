#!/usr/bin/env python3     
import time
import numpy as np
from controller import *
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
from tf import TransformBroadcaster


xd = 3.0
yd = -4.6
zd = 5.16
vxd = 0.0
vyd = 0.0
vzd = 0.0

# Angular velocities
qx = 0.0005
qy = 0.0
qz = 0.0
qw = 1.0
wxd = 0.0
wyd = 0.0
wzd = 0.0

vx_c = 0
vy_c = 0
vz_c = 0

# Angular velocities
wx_c = 0
wy_c = 0
wz_c = 0


def velocity_call_back(velocity_message):
    global vx_c, vy_c, vz_c, wx_c, wy_c, wz_c
    # Read desired linear velocities from node
    vx_c = velocity_message.linear.x
    vy_c = velocity_message.linear.y
    vz_c = velocity_message.linear.z

    # Read desired angular velocities from node
    wx_c = velocity_message.angular.x
    wy_c = velocity_message.angular.y
    wz_c = velocity_message.angular.z
    return None
# Time message Header in the topic
time_message = 0.0
def odometry_call_back(odom_msg):
    global xd, yd, zd, qx, qy, qz, qw, time_message, vxd, vyd, vzd, wxd, wyd, wzd

    # Read desired linear velocities from node
    time_message = odom_msg.header.stamp
    xd = odom_msg.pose.pose.position.x 
    yd = odom_msg.pose.pose.position.y
    zd = odom_msg.pose.pose.position.z
    vxd = odom_msg.twist.twist.linear.x
    vyd = odom_msg.twist.twist.linear.y
    vzd = odom_msg.twist.twist.linear.z


    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w

    wxd = odom_msg.twist.twist.angular.x
    wyd = odom_msg.twist.twist.angular.y
    wzd = odom_msg.twist.twist.angular.z
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

def send_image(bridge, imglr_pub, imgr):
    # Concatenate images L and R
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(imgr, 'bgr8')
    msglr.header.stamp = rospy.Time.now();
    msglr.header.frame_id = "camera_link"
    imglr_pub.publish(msglr)
    return None

def send_image_depth(bridge, imglr_pub, imgr):
    # Concatenate images L and R
    # Decode Image left
    msglr = bridge.cv2_to_imgmsg(imgr, '16UC1')
    msglr.header.stamp = rospy.Time.now();
    msglr.header.frame_id = "camera_link"

    imglr_pub.publish(msglr)
    return None

def get_range_image(camera):
    # Adquisition of camera information
    data = camera.getRangeImageArray()
    #import pdb; pdb.set_trace()
    img = np.array(data)
    img[img == np.inf] = camera.getMaxRange()
    img_aux = img*(65536/8.0)
    img_normalized = np.array(img_aux, np.uint16)
    return img_normalized

def get_odometry(translation, rotation, supervisor, odom_msg):
        # Get system states position, rotation, velocity
        position_traslation = translation.getSFVec3f()
        angles_rotation = rotation.getSFRotation()
        velocity_system =supervisor.getVelocity()

        # Get axix representation
        r = R.from_rotvec(angles_rotation[3] * np.array([angles_rotation[0], angles_rotation[1], angles_rotation[2]]))
        quaternion = r.as_quat()


        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "drone_link"


        odom_msg.pose.pose.position.x = position_traslation[0]
        odom_msg.pose.pose.position.y = position_traslation[1]
        odom_msg.pose.pose.position.z = position_traslation[2]

        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = velocity_system[0]
        odom_msg.twist.twist.linear.y = velocity_system[1]
        odom_msg.twist.twist.linear.z = velocity_system[2]

        odom_msg.twist.twist.angular.x = velocity_system[3]
        odom_msg.twist.twist.angular.y = velocity_system[4]
        odom_msg.twist.twist.angular.z = velocity_system[5]
        return odom_msg

def send_drone_tf(drone_tf, odom_msg):
    position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)   
    drone_tf.sendTransform(position, quaternion, rospy.Time.now(), "drone_link", "world")
    return None

def move_velocity(supervisor, rotation):

    angles_rotation = rotation.getSFRotation()
    velocity_system =supervisor.getVelocity()

    # Get axix representation
    r = R.from_rotvec(angles_rotation[3] * np.array([angles_rotation[0], angles_rotation[1], angles_rotation[2]]))
    quaternion = r.as_quat()
    Rotational_matrix = r.as_matrix()
    velocity_linear = np.array([[vx_c], [vy_c], [vz_c]])
    velocity_angular = np.array([[wx_c], [wy_c], [wz_c]])

    velocity_linear_world = Rotational_matrix@velocity_linear
    velocity_angular_world = Rotational_matrix@velocity_angular

    velocity = [velocity_linear_world[0], velocity_linear_world[1], velocity_linear_world[2], velocity_angular_world[0], velocity_angular_world[1], velocity_angular_world[2]]
    #velocity = [velocity_linear[0], velocity_linear[1], velocity_linear[2], velocity_angular[0], velocity_angular[1], velocity_angular[2]]

    supervisor.setVelocity(velocity)
    return None

def send_camera_tf(camera_tf):
    # Parameters camera traslation and rotation
    position = (0.0, 0.0, 0.18)
    Rotation_matrix = R.from_matrix([[0, 0, 1],
                                    [-1, 0, 0],
                                    [0, -1, 0]])
    
    quaterni_aux = Rotation_matrix.as_quat()
    quaternion = (quaterni_aux[0], quaterni_aux[1], quaterni_aux[2], quaterni_aux[3])
    
    camera_tf.sendTransform(position, quaternion, rospy.Time.now(), "camera_link", "drone_link")
    return None

def set_robot(translation, rotation, h, angle):
    translation.setSFVec3f(h)
    rotation.setSFRotation(angle)
    return None

def send_odometry(odom_msg, odom_pu):
    odom_pu.publish(odom_msg)
    return None

def main(robot, image_pu_rgb, image_pu_d, odometry_pu):
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
    translation_field = supervisor_node.getField('translation')
    rotation_field = supervisor_node.getField('rotation')

    # Camera Definitions
    camera_d = camera_system(robot, "range-finder", time_step)
    camera_rgb = camera_system(robot, "camera_l", time_step)

    # Bridge Openc Cv
    bridge = CvBridge()

    # Time defintion
    t = 0

    # Odometry Message
    odom_drone = Odometry()

    # Tf Drone
    drone_tf = TransformBroadcaster()
    camera_tf = TransformBroadcaster()

    displacement = [xd, yd, zd]
    quaternion_d = [qx, qy, qz, qw] # x y z w
    r_quat = R.from_quat(quaternion_d)
    r_axis = r_quat.as_rotvec()
    r_axis_angle = np.linalg.norm([r_axis]) 
    r_axis_normalized = r_axis/r_axis_angle
    angles = [r_axis_normalized[0], r_axis_normalized[1], r_axis_normalized[2], r_axis_angle] 

    # Set Displacement and rotation
    set_robot(translation_field, rotation_field, displacement, angles)
    
    # Initial Rotation system
    while robot.step(time_step) != -1:
        tic = time.time()
        # Position and anles from the callback
        displacement = [xd, yd, zd]
        quaternion_d = [qx, qy, qz, qw] # x y z w
        r_quat = R.from_quat(quaternion_d)
        r_axis = r_quat.as_rotvec()
        r_axis_angle = np.linalg.norm([r_axis]) 
        r_axis_normalized = r_axis/r_axis_angle
        angles = [r_axis_normalized[0], r_axis_normalized[1], r_axis_normalized[2], r_axis_angle] 

        # Set Displacement and rotation
        set_robot(translation_field, rotation_field, displacement, angles)
        #move_velocity(supervisor_node, rotation_field)
        

        # Get Odometry
        odom_drone = get_odometry(translation_field, rotation_field, supervisor_node, odom_drone)

        # Get image
        img_rgb = get_image(camera_rgb)
        img_d = get_range_image(camera_d)


        # Wait Ros Node and update times
        loop_rate.sleep()
        delta = time.time()- tic
        t = t + delta
        # Send Images
        send_image(bridge, image_pu_rgb, img_rgb)
        send_image_depth(bridge, image_pu_d, img_d)
        send_drone_tf(drone_tf, odom_drone)
        send_camera_tf(camera_tf)
        send_odometry(odom_drone, odometry_pu)
        
    return None

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("vision_system",disable_signals=True, anonymous=True)

        # Conection Webots External
        robot_a = Supervisor()

        # Vision Topic 
        image_topic_rbg = "/camera/color/image_raw"
        image_topic_d = "/camera/aligned_depth_to_color/image_raw"
        image_publisher_rgb = rospy.Publisher(image_topic_rbg, Image, queue_size=1)
        image_publisher_d = rospy.Publisher(image_topic_d, Image, queue_size=1)

        odometry_topic = "/dji_sdk/odometry"
        velocity_subscriber = rospy.Subscriber(odometry_topic, Odometry, odometry_call_back)

        velocity_topic = "/cmd_vel"
        velocity_subscriber = rospy.Subscriber(velocity_topic, Twist, velocity_call_back)

        odometry_webots = "/drone/odometry"
        odometry_publisher = rospy.Publisher(odometry_webots, Odometry, queue_size=10)

        # Simulation 
        main(robot_a, image_publisher_rgb, image_publisher_d, odometry_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass