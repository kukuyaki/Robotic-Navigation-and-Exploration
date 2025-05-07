import cv2
import sys
sys.path.append("..")
import heapq
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()
        

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None
        self.open_set = set()

    def get_neighbors(self, node,inter):
        # ��^??������?�~
        neighbors = []
        directions = [(-inter, 0), (inter, 0), (0, -inter), (0, inter), 
                      (-inter, -inter), (inter, -inter), (-inter, inter), (inter, inter)]  # �K?��V
        
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if self.is_valid(neighbor):
                neighbors.append(neighbor)
        return neighbors
    
    def is_valid(self, node):
        # ?�d??�O�_�b�I��?�M�O�_�i�H�q?
        return self.map[int(node[1]),int(node[0])]>0.5

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        
        # Initialize 
        self.initialize()
        self.goal_node = (int(goal[0]), int(goal[1]))

        self.queue.clear()
        self.parent.clear()
        self.g.clear()
        self.h.clear()
        self.open_set.clear()

        
        
        self.g[start] = 0
        self.h[start] = utils.distance(start, self.goal_node)

        f_score = self.g[start] + self.h[start]
        heapq.heappush(self.queue, (f_score, start))
        self.open_set.add(start)
        self.parent[start] = None

        while(self.queue):
            current_f_score, current = heapq.heappop(self.queue)
            self.open_set.remove(current)  # ??���X�������w?�z��??

            if current == self.goal_node:
                break
            
            for neighbor in self.get_neighbors(current,inter):
                if neighbor not in self.g:  # �̫O?�~�b g �r�夤
                    self.g[neighbor] = float('inf')  # ��l��???�j
                
                tentative_g_score = self.g[current] + utils.distance(current, neighbor)

                if tentative_g_score < self.g[neighbor]:
                    self.parent[neighbor] = current
                    self.g[neighbor] = tentative_g_score
                    self.h[neighbor] = utils.distance(neighbor, self.goal_node)
                    f_score = self.g[neighbor] + self.h[neighbor]

                    if neighbor not in self.open_set:
                        self.open_set.add(neighbor)
                        heapq.heappush(self.queue, (f_score, neighbor))

        # print(f"Parent map: {self.parent}")
        # print(f"g values: {self.g}")
        # print(f"h values: {self.h}")
        # print(f"Open set: {self.open_set}")
        print(self.goal_node)
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
