#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
from geometry_msgs.msg import Twist
import numpy as np

def calc_distance(p_1, p_2):
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def calculate_angle(p_1, p_2, p_3):        
    '''
    p_1 is the point it is coming from 

    p_2 is the point it is currently at

    p_3 is the point it is going to
    '''
    
    angle_1 = math.atan((p_2[1]-p_1[1])/(p_2[0]-p_1[0]))
    angle_2 = math.atan((p_3[1]-p_2[1])/(p_3[0]-p_2[0]))

    # return (angle_1 - angle_2)*180/math.pi
    return math.pi/2 - (angle_1 - angle_2)

def get_sign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

class controller():

    def __init__(self, path) -> None:
        
        self.path = path
        self.max_speed = 1
        self.translating = False
        self.rotating = False
        self.num_of_points = len(path)

        self.current_pos = None
        self.trans_speed = 0.1
        self.rot_speed = 1

    def translate(self, p_a, p_b):
        self.translating = True
        time_required_to_translate = calc_distance(p_a, p_b)/self.trans_speed

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        move_cmd = Twist()
        move_cmd.linear.x = self.trans_speed
        now = rospy.Time.now()
        rate = rospy.Rate(10)

        #Move bot with for required amount of time
        while rospy.Time.now() < now + rospy.Duration.from_sec(time_required_to_translate):
            pub.publish(move_cmd)
            rate.sleep()
        
        print('translated distance: ', calc_distance(p_a, p_b))

    def rotate(self, angle):
        self.rotating = True
        time_required_to_rotate = angle/self.rot_speed

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        move_cmd = Twist()
        move_cmd.angular.z = self.rot_speed*get_sign(angle)
        now = rospy.Time.now()
        rate = rospy.Rate(10)

        #Rotate bot for required amount of time
        while rospy.Time.now() < now + rospy.Duration.from_sec(time_required_to_rotate):
            pub.publish(move_cmd)
            rate.sleep()
        
        print('rotated angle: ', angle)

    def position_getter(self):
        pass

    def mover(self):

        #To move from (0,0) to first point 

        first_angle = math.atan(self.path[1][1]/self.path[1][0])
        print(first_angle)
        self.rotate(first_angle)
        self.translate((0,0), self.path[1])
        
        #Traversing rest of the path
        reached = False
        point_idx = 1

        while not reached:
            current_node = self.path[point_idx]
            to_move_node = self.path[point_idx + 1]
            prev_node = self.path[point_idx - 1]

            self.rotate(calculate_angle(prev_node, current_node, to_move_node))
            
            self.translate(current_node, to_move_node)
            
            point_idx = point_idx + 1

            # if self.current_pos == (6,6):
            #     print('Bot has reached the goal')
            #     reached = True

            if point_idx == self.num_of_points - 1:
                print('Bot has reached the goal')
                reached = True
            else:
                continue


def callback(data):
    published_data = data.data
    path_nodes = []
    for i in range(len(published_data)):
        if i%2 == 0:
            path_nodes.append((published_data[i], published_data[i+1]))
        else:
            continue
    print(path_nodes)
    control = controller(path_nodes)
    control.mover()




def listener():
    rospy.init_node('controller', anonymous=True)
    data = rospy.wait_for_message('path', Float32MultiArray, timeout=None)
    callback(data)
    rospy.spin()

if __name__ == '__main__':
    listener()