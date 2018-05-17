#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import pickle


class DatasetGenerator:
    def __init__(self):
        self.dataset = list()
        self.last_image = None
        self.cvBridge = CvBridge()
        self.imgPub = rospy.Publisher('/img_compressed', Image, queue_size=10)
        self.cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def cbCommandReceived(self, twist):
        rospy.loginfo("Received /dagger/cmd_vel")
        data = (self.last_image, (twist.linear.x, twist.angular.z))
        self.dataset.append(data)
        self.cmdPub.publish(twist)

    def cbImageReceived(self, image):
        """received a image_msg, it resize it to 64x64 and store it in last_image"""
        img = self.cvBridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        res = cv.resize(img, (64, 64))
        self.last_image = res
        img_msg = self.cvBridge.cv2_to_imgmsg(res, encoding='bgr8')
        self.imgPub.publish(img_msg)

    def cbSaveDataset(self):
        rospy.loginfo("Saving dataset to dataset.pkl")
        with open('dataset.pkl', mode="wb") as fd:
            pickle.dump(self.dataset, fd)

if __name__ == '__main__':
    rospy.init_node('dataset_generator')
    dg = DatasetGenerator()
    cmdSub = rospy.Subscriber('/dagger/cmd_vel', Twist, dg.cbCommandReceived)
    imgSub = rospy.Subscriber(
        '/camera/rgb/image_raw', Image, dg.cbImageReceived)
    rospy.on_shutdown(dg.cbSaveDataset)
    rospy.spin()
