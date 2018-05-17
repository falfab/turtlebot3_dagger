#!/usr/bin/env python
import random
import os
import rospy
from rospkg import RosPack

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import agent

import tensorflow as tf
import numpy as np

from cv_bridge import CvBridge
import cv_bridge
import cv2 as cv

# Setting current working directory to the directory containing the file
os.chdir(RosPack().get_path('turtlebot3_dagger'))

# conf = ConfigHandler()

global LAST_IMAGE
LAST_IMAGE = None

cv_Bridge = CvBridge()


def image_callback(image):
    global LAST_IMAGE
    img = cv_Bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    res = cv.resize(img, (64, 64))
    LAST_IMAGE = res


def trained_controller():

    sess = tf.InteractiveSession()
    model = agent.Agent(name='model', sess=sess)
    model.load_model()

    rospy.init_node('dagger_test_node')

    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    pub_cmd_controller = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    img_pub = rospy.Publisher("/img", Image, queue_size=1)

    rospy.sleep(3)
    init_ring = True
    init_panda = True

    bridge = cv_bridge.CvBridge()
    for i in range(5):
        print "Iteration: ", i
        
        #dataset_handler = DatasetHandler(i)        
        iteration = 0
        init_ring = True
        init_panda = True
        while not rospy.is_shutdown():
            # if init_panda:
            #     print "moving to home position"
            #     pub_joint_controller.publish(moveit_handler.target_joint_states)
            #     init_panda = False
            #     moveit_handler.wait(moveit_handler.target_joint_states)
            #     continue
            # if init_ring:
            #     print "setting ring to random pose"
            #     ring_handler.set_random_valid_pose()
            #     ring_pose = ring_handler.get_ring_pose()
            #     pub_ring.publish(ring_pose)
            #     # delta between vrep and rviz on x of 0.5!!
            #     ring_handler.ring_coordinate.x += 0.5
            #     init_ring = False
            #     rospy.sleep(1)
            #     continue

            # if moveit_handler.get_step_size(ring_handler.ring_coordinate) < conf.getfloat('Goal', 'MinStep'):
            #     # succesfull trained
            #     print "grasp successfull"
            #     rospy.sleep(1)
            #     iteration = 0
            #     break

            # if not ring_handler.is_ring_visible(moveit_handler.current_pose):
            #     # failed trained
            #     print "ring no more visible"
            #     rospy.sleep(1)
            #     break

            # if iteration > conf.getint('Dagger', 'MaxActions'):
            #     # failed trained
            #     print "Too many iterations for a single grasp"
            #     rospy.sleep(1)
            #     break

            # if moveit_handler.current_pose.pose.position.z < conf.getfloat('Goal','GoalHeight'):
            #     # failed trained
            #     print "robot too close to the ground"
            #     rospy.sleep(1)
            #     break

            mat = LAST_IMAGE
            img_pub.publish(bridge.cv2_to_imgmsg(mat, encoding="bgr8"))

            # print "compute trained policy"
            # moveit_handler.compute_trained_policy(model,mat)
            act = model.predict(np.reshape(mat,(1, 64, 64, 3)))

            print "trained policy: ", act[0][0],act[0][1]
            cmd_vel = Twist()
            cmd_vel.linear.x = act[0][0]
            cmd_vel.angular.z = act[0][1]
            pub_cmd_controller.publish(cmd_vel)
            rospy.sleep(1)



            # moveit_handler.update_target_pose()
            # print "wait robot moving..."
            # moveit_handler.wait(moveit_handler.target_pose)
            # print "done."

if __name__ == '__main__':
    trained_controller()
