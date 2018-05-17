import pickle
import rospy
import os
import tensorflow as tf
import numpy as np
from rospkg import RosPack
from agent import Agent
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2 as cv

class Dagger(object):
    def __init__(self):
        sess = tf.InteractiveSession()
        self.model = Agent(name='model', sess=sess)
        self.images = []
        self.actions = []
        self.cv_bridge = CvBridge()
        self.last_image = []
        self.last_cmd = None
        self.idx = 0
        self.request_to_stop = False
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.gazebo_model_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    def load_dataset(self):
        with open('./dataset/dataset.pkl', 'rb') as fd:
            dataset = pickle.load(fd)
            self.idx = len(dataset)
            # array preallocation
            self.images = np.zeros((50000, 64,64,3))
            self.actions = np.zeros((50000, 2))
            for index, d in enumerate(dataset):
                self.images[index] = d[0]
                self.actions[index] = d[1]
            del dataset

    def train(self):
        self.model.train(self.images[:self.idx], self.actions[:self.idx], print_freq=1)
        self.model.save_model()

    def aggregate(self):
        self.images[self.idx] = self.last_image
        self.actions[self.idx] = (self.last_cmd.linear.x, self.last_cmd.angular.z)
        print "M: ", self.last_cmd.linear.x, self.last_cmd.angular.z
        self.idx += 1

    def move(self, cmd_msg):
        self.cmd_pub.publish(cmd_msg)

    def load_model(self):
        self.model.load_model()

    def cbImage(self, image):
        # print "-- image received" 
        img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        res = cv.resize(img, (64,64))
        self.last_image = res

    def cbTwist(self, twist):
        # print "-- cmd   received"
        self.last_cmd = twist

    def cbStop(self, bool_msg):
        self.request_to_stop = bool_msg.data

    def turtle_in_lane(self):
        if self.request_to_stop == True:
            self.request_to_stop = False
            return False
        return True

    def set_turtle_to_home_pose(self):
        home_pose = ModelState()
        home_pose.model_name = 'turtlebot3_burger_pi'
        stop_cmd = Twist()
        self.gazebo_model_pub.publish(home_pose)
        self.cmd_pub.publish(stop_cmd)


if __name__ == '__main__':
    os.chdir(RosPack().get_path('turtlebot3_dagger'))
    rospy.init_node('dagger')
    
    dagger = Dagger()
    img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, dagger.cbImage)
    cmd_sub = rospy.Subscriber('/dagger/cmd_vel', Twist, dagger.cbTwist)
    stop_sub = rospy.Subscriber('/dagger/stop_policy', Bool, dagger.cbStop)

    dagger.load_dataset()
    dagger.load_model()

    # dagger.train()

    # wait until the first image and command is received
    while len(dagger.last_image) == 0 and dagger.last_cmd == None:
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        for i in range(3):
            while dagger.turtle_in_lane():
                dagger.aggregate()

                action = dagger.model.predict(np.reshape(dagger.last_image, (1, 64,64,3)))

                cmd_msg = Twist()
                cmd_msg.linear.x = action[0][0]
                cmd_msg.angular.z = action[0][1]

                print "T: ", cmd_msg.linear.x, cmd_msg.angular.z

                dagger.move(cmd_msg)
                rospy.sleep(0.5)
                if rospy.is_shutdown():
                    exit()
            dagger.set_turtle_to_home_pose()
            print "STOP, i should retrain"
            dagger.train()
    
    exit()


            


    
