from path_planning.pathplanner import PathPlanner
from geometry_msgs.msg import Pose2D

prohibited_zones = [
   [0, 1450, 1950, 2000],  
   [1950,1790,2360,2000],
    [2550, 650, 3000, 1100],     
   [550, 0, 1000, 150]  ,
    [1000, 0, 1450, 450]  ,
    [2550, 0, 3000, 150]  ,
    			    
]
"""
blue_prohibited_zones = [
   [0, 1550, 1950, 2000],   
    [2550, 650, 3000, 1100],     
   [550, 0, 1000, 150]  ,
    [1000, 0, 1450, 450]  ,
    [2550, 0, 3000, 150]  ,
    			    
]

yellow_prohibited_zones = [
    [1050, 1550, 3000, 2000],   
    [0, 650, 460, 1100],     
    [0, 0, 450, 150]  ,
    [1560, 0, 2000, 450]  ,
    [2005, 0, 2450, 150]  ,
    			    
]

"""

if __name__ == '__main__':
	p=PathPlanner(prohibited_zones)
		

	current_position = Pose2D()
	current_position.x = 1111.0
	current_position.y = 73.0
	current_position.theta = 0.0

	target_position = Pose2D()
	target_position.x = 2200.0
	target_position.y = 540.0
	target_position.theta = 0.0

	enemy_robot_position = Pose2D()
	enemy_robot_position.x = 2770.0
	enemy_robot_position.y = 770.0
	enemy_robot_position.theta = 0.0

	# Obstacles as a list of Pose2D
	obstacles = [
    [77, 393], [77, 1315], [777, 246], [2230, 243], 
    [2923, 393], [2923, 1320], [1904, 940], [1110, 940],[1780,225]
]

	obstacles_position = [Pose2D(x=float(o[0]), y=float(o[1]), theta=0.0) for o in obstacles]
	# Call the function
	path = p.get_path_rrt(current_position, target_position, obstacles_position, enemy_robot_position)

