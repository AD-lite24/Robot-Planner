#!/usr/bin/env python
from planner import point_to_line
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

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
    return angle_2 - angle_1

def quat_2_euler(ox, oy, oz, ow):
    t3 = +2.0 * (ow * oz + ox * oy)
    t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def get_sign(current, goal, dir):
    
    vect1 = np.array([goal[0] - current[0], goal[1] - current[1]])/ np.linalg.norm([goal[0] - current[0], goal[1] - current[1]])
    vect2 = np.array(dir)/np.linalg.norm(dir)

    if round(np.dot(vect1, vect2), 2) < 0:
            return -1
    else:
        return 1


    


class controller():

    def __init__(self, path) -> None:
        
        self.path = path
        self.max_speed = 1
        self.translating = False
        self.rotating = False
        self.num_of_points = len(path)

        self.vec = 0

        self.prev_orient = 0
        self.current_pos = (0,0)
        self.current_orient = 0
        self.trans_speed = 0.1
        self.rot_speed = 1
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.get_pos)

        self.kp_lin = 0.05
        self.ki_lin = 0
        self.kd_lin = 0.0

        self.kp_ang = 0.01
        self.ki_ang = 0
        self.kd_ang = 0

        self.error = 0
        self.error_last = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def get_pos(self, data):
        self.current_pos = (data.pose.pose.position.x, data.pose.pose.position.y)
        ox, oy, oz, ow = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        self.current_orient = quat_2_euler(ox, oy, oz, ow)*180/math.pi

    def mover(self):
        print('number of nodes: ', len(self.path))
        #To move from (0,0) to first point         
        #Traversing rest of the path
        reached = False
        point_idx = 0
        
        self.rotate = True
        self.translate = False
        self.correction_protocol = False
        
        

        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        move_cmd = Twist()
        
        rate = rospy.Rate(100)

        while not reached:

            #When final goal is reached
            if calc_distance(self.current_pos, (6,6)) <= 0.1:
                reached = True
                print("Goal reached!")
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0
                pub.publish(move_cmd)
                break

            goal = self.path[point_idx + 1]
            
            #for rotation
            if abs((math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient)> 0.5 and self.rotate:
                
                rot_flag = 0
                expected_rotation = (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.prev_orient
                if expected_rotation < 0:
                    if goal[0] < self.current_pos[0]:
                        expected_rotation = 180 + expected_rotation
                        rot_flag = 1
                
                # possible = (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient 
                if rot_flag == 1:
                    self.error = 180 + (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient
                else:
                    self.error = (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient
                self.integral_error += self.error
                self.derivative_error = self.error - self.error_last

                output = self.kp_ang*self.error + self.ki_ang*self.integral_error + self.kd_ang*self.derivative_error
                # print('rotating')
                move_cmd.angular.z = output
                pub.publish(move_cmd)
                rate.sleep()

                if abs(self.error) <= 0.75:
                    self.rotate = False
                    self.translate = True
                    move_cmd.angular.z = 0
                    move_cmd.linear.x = 0
                    pub.publish(move_cmd)
                    
                    if point_idx == 0:
                        print('expected rotation: ', math.atan(goal[1]/goal[0])*180/math.pi, ' actual rotation: ', self.current_orient)
                        print('error: ', self.error)
                        print('stopped rotating, starting transalation')
                        print('**************************************')
                        

                    else:
                        print('expected rotation: ', expected_rotation
                        , ' actual rotation: ', self.current_orient - self.prev_orient)
                        print('error:', self.error)
                        print('stopped rotating, starting transalation')
                        print('**************************************')


            
            #for translating
            elif self.translate:
                self.rotate = False
                self.translate = True
                
                self.error = calc_distance(self.current_pos, goal)
                self.integral_error += self.error
                self.derivative_error = self.error - self.error_last
                self.error_last = self.error

                output = self.kp_lin*self.error + self.ki_lin*self.integral_error + self.kd_lin*self.derivative_error
                sign = get_sign(self.current_pos, goal, [goal[0] - self.path[point_idx][0], goal[1] - self.path[point_idx][1]])
                move_cmd.linear.x = output*sign
            
                # print('translating')
                pub.publish(move_cmd)
                rate.sleep()

                if abs(math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0]))*180/math.pi - self.current_orient) > 3:
                    print('Too much deviation, initiating correction protocol')
                    print('deviation: ', abs(math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0]))*180/math.pi - self.current_orient))
                    self.translate = False
                    self.rotate = False
                    self.correction_protocol = True
                    #stop the bot
                    move_cmd.angular.z = 0
                    move_cmd.linear.x = 0
                    pub.publish(move_cmd)

                if self.error <= 0.075:
                    
                    print('expected coord ', goal, ' actual coord: ', self.current_pos)
                    print('error: ', self.error)
                    print('reached index: ', point_idx + 1)
                    print('stopped translating, starting rotation')
                    
                    print('******************************************')
                    self.rotate = True
                    self.translate = False
                    self.correction_protocol = False
                    point_idx += 1
                    move_cmd.angular.z = 0
                    move_cmd.linear.x = 0
                    self.prev_orient =  self.current_orient
                    pub.publish(move_cmd)
            
            elif self.correction_protocol:


                rot_flag = 0
                expected_rotation = (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.prev_orient
                if expected_rotation < 0:
                    if goal[0] < self.current_pos[0]:
                        expected_rotation = 180 + expected_rotation
                        rot_flag = 1
                
                if rot_flag == 1:
                    self.error = 180 + (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient
                else:
                    self.error = (math.atan((goal[1] - self.current_pos[1])/(goal[0] - self.current_pos[0])))*180/math.pi - self.current_orient
                self.integral_error += self.error
                self.derivative_error = self.error - self.error_last

                output = 0.01*self.error + 0.0001*self.integral_error + 0.1*self.derivative_error
                print(output)
                move_cmd.angular.z = output
                pub.publish(move_cmd)
                rate.sleep()
                
                if abs(self.error) <= 0.75:
                    self.rotate = False
                    self.translate = True
                    move_cmd.angular.z = 0
                    move_cmd.linear.x = 0
                    pub.publish(move_cmd)
                    print('correction complete')
                    print('*************************')
                    self.translate = True
                    self.correction_protocol = False
                    self.rotate = False


def callback(data):
    path_nodes = []
    published_data = data.data
    
    for i in range(len(published_data)):
        if i%2 == 0:
            path_nodes.append((round(published_data[i], 3), round(published_data[i+1], 3)))
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
