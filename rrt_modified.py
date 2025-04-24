import random
import math
import pygame

from typing import List, Tuple

class RRTGraph: 
    SAFETY_DISTANCE: int = 0
    ENEMY_RADIUS = 175/4

    def __init__(self, start, goal, mapDimensions, my_dim, obsdim, obstacles, adv_obs, prohibited_zones):
        self.reset(start, goal, mapDimensions, my_dim, obsdim, obstacles, adv_obs, prohibited_zones)

    def reset(self, start, goal, mapDimensions, my_dim, obsdim, obstacles, adv_obs, prohibited_zones):
        self.start = start
        self.goal = goal
        self.found_goal = False
        self.mapw, self.maph = mapDimensions

        self.x = [self.start[0]]
        self.y = [self.start[1]]
        self.parent = [0]

        self.my_dim = my_dim
        self.obstacles = obstacles
        self.adv_obs = adv_obs
        self.obsDim = obsdim 
        self.prohibited_zones = prohibited_zones  # Now a list of zones

        self.goalstate = None 
        self.path = []
        self.refined_path = []
    def add_node(self, n, x, y) -> None:
        """
        Add a node to the graph with given coordinates.

        Parameters:
        - n (int): Index of the new node.
        - x (float): X-coordinate of the new node.
        - y (float): Y-coordinate of the new node.
        """        
        self.x.insert(n, x)
        self.y.insert(n, y)

    def remove_node(self,n): 
        """
        Remove a node from the graph.

        Parameters:
        - n (int): Index of the node to be removed.
        """        
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        """
        Add an edge between two nodes in the graph.

        Parameters:
        - parent (int): Index of the parent node.
        - child (int): Index of the child node.
        """        
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        """
        Remove an edge from the graph.

        Parameters:
        - n (int): Index of the edge to be removed.
        """
        self.parent.pop(n)

    def number_of_nodes(self): 
        """
        Return the number of nodes in the graph.

        Returns:
        - count (int): Number of nodes in the graph.
        """        
        return len(self.x)
    
    def distance(self, n1: int, n2: int) -> float: 
        """
        Calculate Euclidean distance between two nodes in the graph.

        Parameters:
        - n1 (int): Index of the first node.
        - n2 (int): Index of the second node.

        Returns:
        - dist (float): Euclidean distance between the nodes.
        """        
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return math.sqrt(px + py)
    
    def sample_envir(self):
        """Sample a point while avoiding all prohibited zones."""
        while True:
            x = int(random.uniform(self.my_dim, self.mapw - self.my_dim))
            y = int(random.uniform(self.my_dim, self.maph - self.my_dim))
            
            # Check against all prohibited zones
            in_prohibited = False
            for zone in self.prohibited_zones:
                x_min, y_min, x_max, y_max = zone
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    in_prohibited = True
                    break
            if not in_prohibited:
                return (x, y)

    def nearest(self, n: int) -> int:
        """
        Find the nearest node to a given node in the graph.

        Parameters:
        - n (int): Index of the reference node.

        Returns:
        - nnear (int): Index of the nearest node.
        """
        nnear = 0
        dmin = self.distance(nnear, n)
        # TODO[RS]: shouldn't this go through all the nodes, and not the nodes up to that given index ?
        #for i in range(1, self.number_of_nodes()):
        for i in range(1, n):
            d = self.distance(i, n)
            if d < dmin:
                dmin = d
                nnear = i
        return nnear

    def isFree(self):
        """
        Check if the last added node is in collision with any obstacles.

        Returns:
        - free (bool): True if the node is collision-free, False otherwise.
        """    
        n = self.number_of_nodes() - 1
        x = self.x[n]
        y = self.y[n]
        distance_to_enemy = math.sqrt((x - self.adv_obs[0]) ** 2 + (y - self.adv_obs[1] ) ** 2)
        if distance_to_enemy < (self.ENEMY_RADIUS + self.SAFETY_DISTANCE + self.my_dim):
            self.remove_node(n)
            return False  
        obs = self.obstacles.copy()
        while len(obs) > 0:
            temp = obs.pop(0)
            distance = math.sqrt((x - temp[0]) ** 2 + (y - temp[1]) ** 2)
            limit = self.obsDim + self.SAFETY_DISTANCE + self.my_dim
            if distance < limit:
                # TODO[RS]: why not check at the time of insertion, why insert it then check if it's free and remove it
                self.remove_node(n)
                return False 
        return True
    
    def crossObstacle(self, x1: float, x2: float, y1: float, y2: float) -> bool:
        """Check if path crosses any obstacle or prohibited zone."""
        # Check enemy proximity and prohibited zones
        for i in range(0, 101):
            u = i / 100
            x = x1 * u + x2 * (1 - u)
            y = y1 * u + y2 * (1 - u)
            # Enemy check
            distance_to_enemy = math.sqrt((x - self.adv_obs[0])**2 + (y - self.adv_obs[1])**2)
            """if distance_to_enemy < (100/4):       #(self.ENEMY_RADIUS + self.SAFETY_DISTANCE + self.my_dim):
                return True"""
            if distance_to_enemy < (self.ENEMY_RADIUS + self.SAFETY_DISTANCE + self.my_dim):
                #print("crossObstacle: enemy detected")
                return True#adem : commented the last line and added this one
            
            # Prohibited zones check
            for zone in self.prohibited_zones:
                x_min, y_min, x_max, y_max = zone
                if x_min < x < x_max and y_min < y < y_max:
                    return True
        
        # Check obstacles and prohibited zones
        obs = self.obstacles.copy()
        for obstacle in obs:
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                # Obstacle proximity
                distance = math.sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2)
                if distance < (self.obsDim + self.SAFETY_DISTANCE + self.my_dim):
                    return True
                # Prohibited zones check
                for zone in self.prohibited_zones:
                    x_min, y_min, x_max, y_max = zone
                    if x_min < x < x_max and y_min < y < y_max:
                        return True
        return False
    
    def connect(self, n1: int, n2: int) -> bool:
        """
        Attempt to connect two nodes with a straight line, avoiding obstacles.

        Parameters:
        - n1 (int): Index of the first node.
        - n2 (int): Index of the second node.
        - SAFETY_DISTANCE (float): Safety distance to consider around obstacles.

        Returns:
        - connected (bool): True if the nodes are successfully connected, False otherwise.
        """    
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        if self.crossObstacle(x1, x2, y1, y2):
            # TODO[RS]: again, if we check that the nodes are connectable at the time of inserting the node, no need to remove it afterwards
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear: int, nrand: int, dmax: int = 20):
        """
        Take a step towards a random sample, ensuring a maximum step distance and avoiding obstacles.

        Parameters:
        - nnear: Index of the nearest node.
        - nrand: Index of the random sample node.
        - dmax: Maximum step distance.
        """
        d = self.distance(nnear, nrand)

        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + (dmax - self.SAFETY_DISTANCE) * math.cos(theta)), int(ynear + (dmax - self.SAFETY_DISTANCE) * math.sin(theta)))

            self.remove_node(nrand) 

            if math.sqrt(float(x - self.goal[0])**2 + float(y - self.goal[1])**2)< dmax : 
            #if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.found_goal = True
            else:
                self.add_node(nrand, x, y)
                

    def path_to_goal(self) -> bool:
        """
        Generate a path from the start to the goal using the parent relationships.

        Returns:
        - success (bool): True if a path to the goal is successfully generated, False otherwise.
        """
        if not self.found_goal:
            return False

        if not (0 <= self.goalstate < len(self.parent)):
            return False

        newpos = self.parent[self.goalstate]

        self.path = []
        while newpos != 0 and newpos in self.parent:
            self.path.append(newpos)
            newpos = self.parent[newpos]

        if not self.path:
            return False

        self.path.append(0)  
        self.path.reverse()
        return True


    def getPathCoords(self) -> List[List[int]]:
        """
        Get the coordinates of nodes in the original path.

        Returns:
        - pathCoords: List of node coordinates in the original path.
        """
        pathCoords = [[self.x[node], self.y[node]] for node in self.path]
        pathCoords.append([self.goal[0], self.goal[1]])
        return pathCoords


    def bias(self, ngoal: int) -> Tuple[List[float], List[float], List[int]]:
        """
        Bias the exploration towards the goal.

        Parameters:
        - ngoal (tuple): Coordinates of the goal node.

        Returns:
        - x (list): List of x-coordinates of nodes.
        - y (list): List of y-coordinates of nodes.
        - parent (list): List of parent indices for each node.
        """
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent


    def expand(self): 
        """
        Expand the RRT graph by adding a new node.
        Returns:
        - x (list): List of x-coordinates of nodes.
        - y (list): List of y-coordinates of nodes.
        - parent (list): List of parent indices for each node.
        """
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent


    def optimize_path(self):
        """
        Smooth the path by removing unnecessary waypoints.

        Returns:
        - refined_path (list): List of indices representing the optimized (smoothed) RRT path.
        """
        temporary_path = self.getPathCoords()
        self.refined_path.append(temporary_path[0])

        i = 0  
        n = len(temporary_path)
        #print(n)

        while i < n-1:
            #print("n= ", n)
            #print("i= ", i)
            last_valid_node = temporary_path[i + 1]

            j = n - 1
            while j > i:
                #print("j= ", j)
                if not self.crossObstacle(temporary_path[i][0], temporary_path[j][0], temporary_path[i][1], temporary_path[j][1]):
                    last_valid_node = temporary_path[j]
                    self.refined_path.append(last_valid_node)
                    #n = len(temporary_path) - (j - i)
                    break
                else:
                    j -= 1
            # # j == 0 -> all points cross obstacles
            # if not j:
            #     break
            if j == n - 1 and i == n - 1:
                break
            i = j
        """
        if not (self.refined_path[-1] == self.goal):   
            x = self.goal[0] 
            y = self.goal[1]    
            self.refined_path.append([x, y])
        """    
        print ("original path" ,self.getPathCoords()) 
        print ("refined path ",self.refined_path)     
        return self.refined_path
