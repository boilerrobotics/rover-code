import heapq
import numpy as np
import sys

from enum import Enum
import sys
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
        


        
    



                
start = (0, 0)
target = (10, 10)
map = Map(start, target)
map.map[(5, 5)].blocked = True 
map.map[(4, 5)].blocked = True 
map.map[(6, 5)].blocked = True 

map.map[(3, 5)].blocked = True 
map.compute_shorted_path()
map.print_map()

