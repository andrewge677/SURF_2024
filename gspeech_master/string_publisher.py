#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # Initialize the ROS node named 'talker'
    rospy.init_node('talker', anonymous=True)
    # Create a publisher that will publish on the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        hello_str = "Hello ROS World %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
