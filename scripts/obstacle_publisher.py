#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('obstacle_detector', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        obstacles = [0, 1.5, 0, 3, 0, 4.5, 
                    1.5, 0, 1.5, 1.5, 1.5, 3, 1.5, 4.5,
                    3, 0, 3,1.5, 3,3, 3, 4.5,
                    4.5, 0, 4.5, 1.5, 4.5, 3, 4.5, 4.5
                    ]
        obstacle_publishing = Float32MultiArray(data = obstacles)
        print(type(obstacle_publishing))
        rospy.loginfo(obstacle_publishing)
        pub.publish(obstacle_publishing)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
