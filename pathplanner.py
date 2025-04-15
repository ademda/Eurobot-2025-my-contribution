# path_planner_module.py
from path_planning.rrt_modified import RRTGraph
from geometry_msgs.msg import Pose2D
import math
from typing import List
import threading
import time


# Parameters for obstacle and robot dimensions
obstacle_radius = 60
robot_radius = 175
safe_distance = 50 # adem:maybe i need to make this 0 or 10 only
enemy_radius=175


class PathPlanner:
    def __init__(self, prohibited_zones):
        # Initialize prohibited zones (scaled down by 4)
        self.prohibited_zones = [
            [zone[0] / 4, zone[1] / 4, zone[2] / 4, zone[3] / 4]
            for zone in prohibited_zones
        ]

    def find_path_coordinates(self, start, goal, dimensions, my_dim, obsdim, obstacles, adv_obs):
        # Use self.prohibited_zones instead of passing it as a parameter
        graph = RRTGraph(start, goal, dimensions, my_dim, obsdim, obstacles, adv_obs, self.prohibited_zones)
        
        iteration = 0
        result = [None]  # Shared list to store the result 

        def worker(iteration):
            start_time = time.time()
            while not graph.path_to_goal():
                if time.time() - start_time > 2:  # Check if 1 seconds have passed
                    result[0] = None  # Set the result to None
                    return
                if iteration > 100000:
                    result[0] = None
                    return
                    
                if iteration % 10 == 0:
                    X, Y, Parent = graph.bias(goal)
                else:
                    X, Y, Parent = graph.expand()

                iteration += 1
                #print(iteration)

            smoothed_path_coords = graph.optimize_path()
            result[0] = smoothed_path_coords  # Store the result

        # Create and start the worker thread
        thread = threading.Thread(target=worker, args=(iteration,))
        thread.start()

        # Wait for the worker thread to finish or for timeout
        thread.join(timeout=1)

        if thread.is_alive():  # If the thread is still alive, it means timeout occurred
            return None  # Return None if timeout occurs

        return result[0]  # Return the result obtained by the worker thread
        
    def can_go_directly_to_target(self, current_position : Pose2D, task_position : Pose2D, map_obstacles : List[Pose2D], enemy_robot_position : Pose2D) -> bool:
        for obstacle in map_obstacles:
            # Pour chaque obstacle, vérifie si le chemin direct est valide et si la position de la tâche n'est pas égale à celle de l'obstacle
            if ( (not self.is_valid_path(current_position, task_position,obstacle, enemy_radius)) and 
                (task_position.x != obstacle.x and task_position.y != obstacle.y) ):

                return False
        
        # Vérifie également si le chemin direct vers le robot ennemi est valide
        if not self.is_valid_path(current_position, task_position, enemy_robot_position, enemy_raduis):         
            return False
        
        return True

    
    def get_path_rrt(self, current_position: Pose2D, target_position: Pose2D, obstacles_position: List[Pose2D], enemy_robot_position: Pose2D):
        global robot_radius
        robot_radius_reduced = (robot_radius / 4)
        start = (int(current_position.x / 4), int(current_position.y / 4))
        goal = (int(target_position.x / 4), int(target_position.y / 4))

        obsdim = obstacle_radius / 4
        dimensions = (3000 / 4, 2000 / 4)

        dx = enemy_robot_position.x - target_position.x
        dy = enemy_robot_position.y - target_position.y
        distance_target_enemy = math.sqrt(dx**2 + dy**2)
        if distance_target_enemy <= obstacle_radius + robot_radius :
            pose = Pose2D()
            pose.x = -1.0
            pose.y = -1.0
            pose.theta = 0.0
            return [pose]

        # In get_path_rrt():
        obstacles = []
        for obstacle in obstacles_position:
            if obstacle != target_position:
                dx_current = obstacle.x - current_position.x
                dy_current = obstacle.y - current_position.y
                distance_obstacle_current = math.sqrt(dx_current**2 + dy_current**2)

                dx_target = obstacle.x - target_position.x
                dy_target = obstacle.y - target_position.y
                distance_obstacle_target = math.sqrt(dx_target**2 + dy_target**2)

                buffer_current = obstacle_radius + robot_radius + safe_distance  # 450 units
                buffer_target = obstacle_radius + robot_radius  

                # Include obstacle if it's not too close to the robot or target
                if (distance_obstacle_current > buffer_current) and (distance_obstacle_target > buffer_target):
                    obstacles.append([int(obstacle.x/4), int(obstacle.y/4)])
        enemy_robot_position_rrt = [int(enemy_robot_position.x/4), int(enemy_robot_position.y/4)]

        print("start:", start)
        print("goal:", goal)
        print("obstacles:", obstacles)

        # Use self.prohibited_zones here
        final_path=[]

        #if (self.can_go_directly_to_target(current_position,target_position,obstacles_position,enemy_robot_position)):

         #  final_path.append(target_position)
        #else: 
        path_result = self.find_path_coordinates(start, goal, dimensions, robot_radius_reduced, obsdim, obstacles, enemy_robot_position_rrt)

        if path_result is None:
            pose = Pose2D()
            pose.x = -1.0
            pose.y = -1.0
            pose.theta = 0.0
            return [pose]

        for i in range(len(path_result) - 1):  # Parcours jusqu'à l'avant-dernière case
            path_segment = path_result[i]
            pose = Pose2D()
            pose.x = float(path_segment[0]) * 4
            pose.y = float(path_segment[1]) * 4
            pose.theta = 655.0
            print(pose)
            final_path.append(pose)
        final_path.append(target_position)
        print(target_position)
            
        return final_path

    # Function to estimate time required to traverse a given path
    """def estimate_time(self, path):
        speed = 1.0
         # If the path is impossible (contains invalid points), return a large time value
        if self.impossible_path(path):
            return 50000
        
        # Calculate the total distance of the path
        distance = sum(self.calculate_distance(path_segment) for path_segment in zip(path, path[1:]))

        # Calculate estimated time based on speed
        estimated_time = distance / speed
        return estimated_time """
    
    # Check if a path is impossible (contains invalid points)
    def impossible_path(self, path):
        for point in path:
            if point.x == -1 or point.y == -1:
                return True
        return False

    # Calculate distance between two points in a path
    def calculate_distance(self, path_segment : List[Pose2D]):
        return math.sqrt((path_segment[1].x - path_segment[0].x)**2 + (path_segment[1].y - path_segment[0].y)**2)
    

    # Check if a given path is valid and collision-free
    def is_valid_path(self, current_position: Pose2D, target_position: Pose2D, obstacle_position: Pose2D, obstacle_radius : int):
        
        path_vector = [target_position.x - current_position.x, target_position.y - current_position.y]
        path_vector_length = math.sqrt(path_vector[0] ** 2 + path_vector[1] ** 2)
        obstacle_vector = [obstacle_position.x - current_position.x, obstacle_position.y - current_position.y]
        obstacle_vector_length = math.sqrt(obstacle_vector[0] ** 2 + obstacle_vector[1] ** 2)
        
        dot_product = path_vector[0] * obstacle_vector[0] + path_vector[1] * obstacle_vector[1]
        determinate = path_vector[0] * obstacle_vector[1] - path_vector[1] * obstacle_vector[0] 
        
        # Calculate the Angle between the Path and the obstacle vector
        theta = math.atan2(determinate, dot_product)
        
        # Check if the theta angle between -90 and 90 degrees
        if (not (-90 < math.degrees(theta) < 90) ):
            return True
        
        # Check if the Projection of the Obstacle Exisist outside the Path, then return True
        projection_distance = obstacle_vector_length * math.cos(abs(theta))
        
        if ( projection_distance > path_vector_length + robot_radius + obstacle_radius + safe_distance ):
            return True
        
        # Calculate the distance of the projection line
        projection_line_distance = obstacle_vector_length * math.sin(abs(theta))
        
        # Check if the path is valid without collision
        if (projection_line_distance > robot_radius + obstacle_radius + safe_distance):
            return True
        
        # if the path is not valide, return False
        return False
    
    # Find intersection points of an avoidance circle with the path vector
    def find_intersection(self, avoidance_circle_center: Pose2D, avoidance_circle_radius: float, path_vector):
        alpha_rad = math.atan2(path_vector[1], path_vector[0])
        
        # Calculate the intersection points
        x_intersect_1 = avoidance_circle_center.x + avoidance_circle_radius * math.cos(alpha_rad + math.pi/2)
        y_intersect_1 = avoidance_circle_center.y + avoidance_circle_radius * math.sin(alpha_rad + math.pi/2)

        x_intersect_2 = avoidance_circle_center.x + avoidance_circle_radius * math.cos(alpha_rad - math.pi/2)
        y_intersect_2 = avoidance_circle_center.y + avoidance_circle_radius * math.sin(alpha_rad - math.pi/2)
        
        return [Pose2D(x_intersect_1, y_intersect_1, 655), Pose2D(x_intersect_2, y_intersect_2, 655)]
    
    # Get an intermediate point for path planning considering obstacles
    def get_intermediate_point(self, current_position: Pose2D, target_position: Pose2D, obstacle_position: Pose2D):
        # Calculate the vector from the current position to the target position
        path_vector = [target_position.x - current_position.x, target_position.y - current_position.y]
        
        # Declare avoidance circle parameteres
        avoidance_circle_center = obstacle_position
        avoidance_circle_radius = obstacle_radius + robot_radius + safe_distance
        
        intersection_points = self.find_intersection(avoidance_circle_center, avoidance_circle_radius, path_vector)
        
        intermediate_point_1 = intersection_points[0]
        intermediate_point_2 = intersection_points[1]
        
        if(not(0 + robot_radius < intermediate_point_1.x < 3000 - robot_radius) or not(0 + robot_radius < intermediate_point_1.y < 2000 - robot_radius)):
            if(self.is_valid_path( current_position, intermediate_point_2, obstacle_position )
               and self.is_valid_path( intermediate_point_2, target_position, obstacle_position )):
                return intermediate_point_2
            else:
                return Pose2D(-1, -1, 0)
            
        if(not(0 + robot_radius < intermediate_point_2.x < 3000 - robot_radius) or not(0 + robot_radius < intermediate_point_2.y < 2000 - robot_radius)):
            if(self.is_valid_path( current_position, intermediate_point_1, obstacle_position )
               and self.is_valid_path( intermediate_point_1, target_position, obstacle_position )):
                return intermediate_point_1
            else:
                return Pose2D(-1, -1, 0)
            
        if (self.is_valid_path( current_position, intermediate_point_1, obstacle_position)
            and self.is_valid_path( intermediate_point_1, target_position, obstacle_position)
            and self.is_valid_path(current_position, intermediate_point_2, obstacle_position)
            and self.is_valid_path( intermediate_point_2, target_position, obstacle_position)):
            
                path_1_distance = self.calculate_distance([current_position, intermediate_point_1]) + self.calculate_distance([intermediate_point_1, target_position])
                path_2_distance = self.calculate_distance([current_position, intermediate_point_2]) + self.calculate_distance([intermediate_point_2, target_position])  
                          
                if(path_1_distance <= path_2_distance):
                    return intermediate_point_1
                else:
                    return intermediate_point_2
        
        if ( not ( self.is_valid_path( current_position, intermediate_point_1, obstacle_position ))
            or not ( self.is_valid_path( intermediate_point_1, target_position, obstacle_position))):
            
                if(self.is_valid_path( current_position, intermediate_point_2, obstacle_position )
                   and self.is_valid_path( intermediate_point_2, target_position, obstacle_position)):
                    return intermediate_point_2
                else:
                    return Pose2D(-1, -1, 0)
        
        if ( not ( self.is_valid_path( current_position, intermediate_point_2, obstacle_position ))
            or not ( self.is_valid_path( intermediate_point_2, target_position, obstacle_position))):
    
                if(self.is_valid_path( current_position, intermediate_point_1, obstacle_position )
                    and self.is_valid_path( intermediate_point_1, target_position, obstacle_position)):
                    return intermediate_point_1
                else:
                    return Pose2D(-1, -1, 0)
                
    def is_valid_path_all_obstacles(self, current_position: Pose2D, target_position: Pose2D, obstacles_position: List[Pose2D]):
        for obstacle in obstacles_position:
            if(not self.is_valid_path(current_position, target_position, obstacle, 100)):
                return False
        return True
                
    # Get a valid path considering obstacles   
    def getPath(self, current_position: Pose2D, target_position: Pose2D, obstacle_position: Pose2D):
        if (self.is_valid_path(current_position, target_position, obstacle_position)):
            return [current_position, target_position]
        else:
            intermediate_point = self.get_intermediate_point(current_position, target_position, obstacle_position)
            return [current_position, intermediate_point , target_position]
            
        


        





        
