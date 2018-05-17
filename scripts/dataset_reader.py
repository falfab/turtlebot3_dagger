import rospy
from geometry_msgs.msg import Twist
import pickle

if __name__ == '__main__':
    rospy.init_node('dataset_reader')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    data = []
    with open('./dataset.pkl', 'rb') as fd:
        data = pickle.load(fd)

    for val in data:
        pub.publish(val[1])
        rospy.sleep(0.1)
