import heapq
import numpy as np
import sys

import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
    Duration,
)
import math

class Node():
    def __init__(self, loc):
        self.loc = loc
        self.g = sys.float_info.max
        self.rhs = sys.float_info.max
        self.neighbors = []
        self.blocked = False

    def heuristic(self, node):
        return int(math.sqrt((self.loc[1] - node.loc[1])**2 + (self.loc[0] - node.loc[0])**2) * 1000)

    def __lt__(self, other):
        return self.loc < other.loc
    
    def cost(self, other):
        if(self.blocked or other.blocked):
            return float('inf')
        else:
            dx = abs(self.loc[0] - other.loc[0])
            dy = abs(self.loc[1] - other.loc[1])

            if dx == 1 and dy == 1:
                return 1414
            return 1000
    
    
class Map():
    def __init__(self, start, target):
        self.start = start
        self.target = target
        self.map = {}
        self.init()
        self.find_neighbors()
        self.start = self.map[start]
        self.target = self.map[target]
        
        self.k_m = 0
        self.p_queue = []
        heapq.heapify(self.p_queue)

        self.target.rhs = 0
        key = self.calculate_key(self.target)
        heapq.heappush(self.p_queue, (key[0], key[1], self.target))


    def init(self):
        for x in range(self.start[0] - 5, self.target[0] + 6):
            for y in range(self.start[1] - 5, self.target[1] + 6):
                self.map[(x, y)] = Node((x, y))
    
    def set_occupied(self, loc):
        self.map[loc].g = sys.float_info.max
        self.map[loc].rhs = sys.float_info.max


    def calculate_key(self, node):
        return [min(node.g, node.rhs) + node.heuristic(self.start) + self.k_m, min(node.g, node.rhs)]
    
    def find_neighbors(self):
        for node in self.map.values():
            for x in self.map.values():
                if x != node and (x.loc[0] in (node.loc[0] - 1, node.loc[0], node.loc[0] + 1) and x.loc[1] in (node.loc[1] - 1, node.loc[1], node.loc[1] + 1)):
                    node.neighbors.append(x)


    def update_vertex(self, node):
        if(self.target != node):
            node.rhs = min([(node.cost(x) + x.g) for x in node.neighbors])
            
            if node.g != node.rhs:
                key = self.calculate_key(node)
                heapq.heappush(self.p_queue, (key[0], key[1], node))
    
    def compute_shorted_path(self):
        while(self.p_queue and (self.p_queue[0][:2] < tuple(self.calculate_key(self.start)) or self.start.rhs != self.start.g)):
            k_old = (self.p_queue[0][0], self.p_queue[0][1])
            node = heapq.heappop(self.p_queue)[2]

            if k_old < tuple(self.calculate_key(node)):
                heapq.heappush(self.p_queue,  (*self.calculate_key(node), node))
            elif node.g > node.rhs:
                node.g = node.rhs
                for x in node.neighbors:
                    self.update_vertex(x)
            else:
                g_old = node.g
                node.g = sys.float_info.max
                for x in node.neighbors:
                    self.update_vertex(x)

    def print_map(self):
        for y in range(self.target.loc[1] + 5, self.start.loc[1] - 5, -1):
            for x in range(self.start.loc[0] - 5, self.target.loc[0] + 5):
                if(self.map[(x, y)] == self.start):
                    print("S ", end="")
                elif(self.map[(x, y)] == self.target):
                    print("T ", end="")
                else:
                    if self.map[(x, y)].g == sys.float_info.max:
                        if self.map[(x, y)].blocked:
                            print("B ", end="")
                        else:
                            print("∞ ", end="")
                    else:
                        print(self.map[(x, y)].g//100, end=" ")
            print()
        


        
    

class ObstacleDetectionNode(RosNode):

    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.start = (0, 0)
        self.target = (10, 10)
        self.map = Map(self.start, self.target)
        self.map.compute_shorted_path()
        zed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            lifespan=Duration(seconds=0.1, nanoseconds=0),
        )
        # self._cloud_subscription = self.create_subscription(
        #     PointCloud,
        #     "/zed/zed_node/point_cloud/cloud_registered",
        #     self.update_map,
        #     zed_qos
        # )
        self._pos_subscriber = self.create_subscription(
            Odometry,
            "/zed/zed_node/odom",
            self.update_pos,
            zed_qos
        )
        self._next_point_publisher = self.create_publisher(
            Twist,
            "next_point",
            pub_qos,
        )
        self.timer = self.create_timer(5, self.publish_map)

    def publish_map(self):
        self.map.print_map()

    def update_pos(self, msg):
        pos = (int(msg.pose.pose.position.x), int(msg.pose.pose.position.y))

        if(self.start != pos):
            self.map.k_m += self.map.map[self.start].heuristic(self.map.map[pos])
            self.start = pos
            self.map.start = self.map.map[self.start]
            self.map.compute_shorted_path()
            if self.map.start.rhs != sys.float_info.max:
                next_node = min(self.map.start.neighbors, key=lambda x: x.cost(self.map.start) + x.g)
                next_point_msg = Twist()
                next_point_msg.linear.x = next_node.loc[0]
                next_point_msg.linear.y = next_node.loc[1]
                self._next_point_publisher.publish(next_point_msg)
    
    def update_map(self, msg):
        for point in msg.points:
            x = int(point.x)
            y = int(point.y)
            if (x, y) in self.map.map.keys():
                self.map.map[(x, y)].blocked = True
                self.map.update_vertex(self.map.map[(x, y)])
        self.map.compute_shorted_path()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
