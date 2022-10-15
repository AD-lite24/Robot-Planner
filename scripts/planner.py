#!/usr/bin/env python
from tkinter.messagebox import NO
import rospy
from std_msgs.msg import Float32MultiArray
import random
import math
import matplotlib.pyplot as plt
import matplotlib as mpl

def calc_distance(p_1, p_2):
    return math.sqrt((p_2[0]-p_1[0])**2 + (p_2[1]-p_1[1])**2)

def point_to_line(p_1, p_2, p_3):
    dist = math.sqrt((p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2)

    # determine intersection ratio u
    # for three points A, B with a line between them and a third point C, the tangent to the line AB
    # passing through C intersects the line AB a distance along its length equal to u*|AB|
    r_u = ((p_3[0] - p_1[0])*(p_2[0] - p_1[0]) + (p_3[1] - p_1[1])*(p_2[1] - p_1[1]))/(dist**2)

    # intersection point
    p_i = (p_1[0] + r_u*(p_2[0] - p_1[0]), p_1[1] + r_u*(p_2[1] - p_1[1]))

    # distance from P3 to intersection point
    tan_len = calc_distance(p_i, p_3)

    return r_u, tan_len

class planner():
    
    def __init__(self, obstacles):
        self.num_of_obs = 15
        self.obstacles = obstacles
        self.side_len = 6

        self.start, self.end = (0,0), (6,6)

        self.nodes_list = [['q0', self.start, 'None']]
        self.segments_list = []
        self.path_nodes, self.path_segments = [], []
        self.publishing_data = []

    def obstacle_check(self, current):
        for pos in self.obstacles:
            if calc_distance(current, pos) <= 0.35:
                return True
            else:
                continue

        return False

    def find_closest(self, current):
        d_list = [calc_distance(current, node[1]) for node in self.nodes_list]
        return min(range(len(d_list)), key=d_list.__getitem__)

    def obstacle_in_path_cheker(self, new_node, parent):            #checks if the path crosses an obstacle
        for obstalce in self.obstacles:
            _, tan_dist = point_to_line(new_node, parent, obstalce)
            if tan_dist <= 0.35:
                return True
            else:
                continue
        return False
    

    def node_generator(self):

        valid_point = False
        node_name = "q{}".format(len(self.nodes_list))

        while not valid_point:
            #rand_coords = (self.side_len*random.random(), self.side_len*random.random())
            rand_coords = (round(random.uniform(0, 6.01), 3), round(random.uniform(0, 6.01), 3))
            parent = self.nodes_list[self.find_closest(rand_coords)]

            x_dist = rand_coords[0] - parent[1][0]
            y_dist = rand_coords[1] - parent[1][1]

            
            dist = calc_distance(rand_coords, parent[1])
            
            new_node = (parent[1][0] + x_dist/dist, parent[1][1] + y_dist/dist)
            new_node = (round(new_node[0], 2), round(new_node[1], 2))
            path_crossing_obstacle = self.obstacle_in_path_cheker(new_node, parent[1])
            if self.obstacle_check(new_node) or path_crossing_obstacle:
                continue
            else:
                valid_point = True
                
        
        self.nodes_list.append([node_name, new_node, parent[0]])
        self.segments_list.append([parent[1], new_node])

    def path_check(self, current):

        path_collisions = []
        for obstacle in self.obstacles:
            colliding = False
            too_close = False
            r_u, d_obs = point_to_line(current, self.end, obstacle)

            if 0 <= r_u <= 1:
                colliding = True

            if d_obs <= 0.35:
                too_close = True
            
            if colliding and too_close:
                path_collisions.append(True)
                
            else:
                path_collisions.append(False)
        return any(path_collisions)
    
    def final_path_generator(self):
        self.segments_list.append([self.nodes_list[-1][1], self.end])  #creating segment for the last node and end point
    
    def tree_generator(self):

        done = False
        while not done:
            self.node_generator()
            if not self.path_check(self.nodes_list[-1][1]):    #getting coordinate of parent node
                done = True
        self.final_path_generator()
        self.nodes_list.append(["q{}".format(len(self.nodes_list)), self.end, self.nodes_list[-1][0]])
    
    def path_finder(self):

        current = self.nodes_list[-1]
        self.path_nodes.append(current[1])

        for _, j in reversed(list(enumerate(self.nodes_list))):
            if current[2] == j[0]:
                self.path_nodes.insert(0, j[1])
                self.path_segments.insert(0, (j[1], current[1]))
                current = j


    
def callback(data):
    #rospy.loginfo(data.data)
    

    published_data = data.data
    obstacles = []
    for i in range(len(published_data)):
        if i%2 == 0:
            obstacles.append((published_data[i], published_data[i+1]))
        else:
            continue
    

    plan = planner(obstacles)
    plan.tree_generator()
    plan.path_finder()
    
    path_nodes = plan.path_nodes
    path_segments = plan.path_segments

    COLORS = ['#6AB71F', '#FF5733', '#4DAAEA', '#C0120A']
    FIG, AX = plt.subplots(nrows=1, ncols=1, sharex=True, sharey=True, figsize=(6, 6))

    plt.xlim(0, plan.side_len)
    plt.ylim(0, plan.side_len)

    OBSTACLES = [plt.Circle(center, 0.35) for center in obstacles]
    OBS_PATCHES = mpl.collections.PatchCollection(OBSTACLES, facecolors='black')
    AX.add_collection(OBS_PATCHES)

    plt.scatter(plan.start[0], plan.start[1], s=200, c = COLORS[0], marker='1')
    plt.scatter(plan.end[0], plan.end[1], s=200, c= COLORS[1], marker = '2')

    for k in enumerate(plan.nodes_list):
        plt.scatter(k[1][1][0], k[1][1][1], s=10, c= COLORS[2])
        if k[0] > 0:
            node_seg = mpl.collections.LineCollection(plan.segments_list[k[0]-1:k[0]], colors = COLORS[3])
            AX.add_collection(node_seg)
        plt.pause(0.25)
    
    for m in enumerate(plan.path_nodes):
        plt.scatter(m[1][0], m[1][1], s=10, c=COLORS[3])
        if m[0] > 0:
            path_seg = mpl.collections.LineCollection(plan.path_segments[m[0]-1:m[0]], colors=COLORS[3])
            AX.add_collection(path_seg)
        plt.pause(0.1)
    
    plt.show
    plt.pause(5)
    
    for point in path_nodes:
        plan.publishing_data.append(point[0])
        plan.publishing_data.append(point[1])
    
    pub = rospy.Publisher('path', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        path = plan.publishing_data
        path_publishing = Float32MultiArray(data = path)
        pub.publish(path_publishing)
        rate.sleep()

def listener():
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('obstacle_detector',Float32MultiArray, callback)
    data = rospy.wait_for_message('obstacle_detector', Float32MultiArray, timeout=None)
    callback(data)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
    
