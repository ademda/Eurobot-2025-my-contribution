import rclpy
from rclpy.node import Node
import pygame
from path_planning.pathplanner import PathPlanner
from geometry_msgs.msg import Pose2D

# Map and robot dimensions
MAP_LENGTH_MM = 3000
MAP_WIDTH_MM = 2000
ROBOT_LENGTH_MM = 230
ROBOT_WIDTH_MM = 370

WINDOW_LENGTH_PX = 1080
WINDOW_WIDTH_PX = 720
SCALE_X = WINDOW_LENGTH_PX / MAP_LENGTH_MM
SCALE_Y = WINDOW_WIDTH_PX / MAP_WIDTH_MM

ROBOT_LENGTH_PX = int(ROBOT_LENGTH_MM * SCALE_X)
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_MM * SCALE_Y)

# Start‑circle radius in mm and px
START_RADIUS_MM = 185
START_RADIUS_PX = int(START_RADIUS_MM * SCALE_X)

# Pre‑create the start‑circle surface
start_surface = pygame.Surface((START_RADIUS_PX * 2, START_RADIUS_PX * 2), pygame.SRCALPHA)
pygame.draw.circle(
    start_surface,
    (238, 130, 238, 200),               # Violet (R=238, G=130, B=238) with alpha=200
    (START_RADIUS_PX, START_RADIUS_PX), # center of the surface
    START_RADIUS_PX                     # radius
)

# Obstacles list (in mm)
obstacles = [
    [80, 542],[80, 442],[80, 342],[80, 242],[80, 1165],[80,1265],[80,1365],[80, 1465],
    [2080, 250],[2180, 250],[2280, 250],[2380, 250],
    [2930, 542],[2930, 442],[2930, 342],[2930, 242],[2930, 1165],[2930, 1265],
    [2930, 1365],[2930, 1465],[960, 945],[1060, 945],[1160, 945],[1260, 945],
    [1760,945],[1860,945],[1960,945],[2060,945]
]

# Prohibited zones [x1, y1, x2, y2] in mm
prohibited_zones = [
    [1050, 1550, 3000, 2000],
    [0, 650, 460, 1100],
    [650, 1790, 1050, 2000],
    [0, 0, 450, 150],
    [1560, 0, 2000, 450],
    [2005, 0, 2450, 150],
]

class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    p = PathPlanner(prohibited_zones)
    robot_simulation_node = RobotSimulation()
    screen = pygame.display.set_mode((WINDOW_LENGTH_PX, WINDOW_WIDTH_PX + 50))

    map_image = pygame.image.load("map.png")
    map_image = pygame.transform.scale(map_image, (WINDOW_LENGTH_PX, WINDOW_WIDTH_PX))

    robot_image = pygame.image.load("robot.png")
    robot_image = pygame.transform.scale(robot_image, (ROBOT_LENGTH_PX, ROBOT_WIDTH_PX))

    clock = pygame.time.Clock()
    selecting_start = selecting_goal = False
    start_pos = goal_pos = path = None

    # Obstacles as Pose2D
    obstacles_position = [
        Pose2D(x=float(o[0]), y=float(o[1]), theta=0.0)
        for o in obstacles
    ]

    # Enemy Robot Initialization
    enemy_robot_position = Pose2D(x=2770.0, y=770.0, theta=0.0)
    ENEMY_SIZE_PX = int(250 * SCALE_X)  # Enemy robot size in px
    MOVE_STEP = 50  # mm per keypress

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                if selecting_start:
                    start_pos = Pose2D(
                        x=event.pos[0] / SCALE_X,
                        y=(WINDOW_WIDTH_PX - event.pos[1]) / SCALE_Y,
                        theta=0.0
                    )
                    selecting_start = False
                elif selecting_goal:
                    goal_pos = Pose2D(
                        x=event.pos[0] / SCALE_X,
                        y=(WINDOW_WIDTH_PX - event.pos[1]) / SCALE_Y,
                        theta=0.0
                    )
                    selecting_goal = False
                elif start_pos and goal_pos:
                    path = p.get_path_rrt(start_pos, goal_pos,
                                          obstacles_position,
                                          enemy_robot_position)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    selecting_start = True
                if event.key == pygame.K_g:
                    selecting_goal = True

                # Enemy Robot Movement
                if event.key == pygame.K_UP:
                    enemy_robot_position.y = min(MAP_WIDTH_MM,
                                                 enemy_robot_position.y + MOVE_STEP)
                if event.key == pygame.K_DOWN:
                    enemy_robot_position.y = max(0,
                                                 enemy_robot_position.y - MOVE_STEP)
                if event.key == pygame.K_LEFT:
                    enemy_robot_position.x = max(0,
                                                 enemy_robot_position.x - MOVE_STEP)
                if event.key == pygame.K_RIGHT:
                    enemy_robot_position.x = min(MAP_LENGTH_MM,
                                                 enemy_robot_position.x + MOVE_STEP)

                print(f"Enemy moved to: ({enemy_robot_position.x}, {enemy_robot_position.y})")

        rclpy.spin_once(robot_simulation_node, timeout_sec=0.01)

        # Draw the map
        screen.blit(map_image, (0, 0))

        # Draw Start Point with semi‑transparent circle
        if start_pos:
            sx = int(start_pos.x * SCALE_X)
            sy = int((MAP_WIDTH_MM - start_pos.y) * SCALE_Y)
            screen.blit(start_surface, (sx - START_RADIUS_PX,
                                        sy - START_RADIUS_PX))

        # Draw Goal Point as before
        if goal_pos:
            gx = int(goal_pos.x * SCALE_X)
            gy = int((MAP_WIDTH_MM - goal_pos.y) * SCALE_Y)
            pygame.draw.circle(screen, (255, 0, 0), (gx, gy), 5)

        # Draw Obstacles
        for ox, oy in obstacles:
            px = int(ox * SCALE_X)
            py = int((MAP_WIDTH_MM - oy) * SCALE_Y)
            pygame.draw.circle(screen, (255, 255, 0),
                               (px, py),
                               int(60 * SCALE_X))

        # Draw Prohibited Zones
        for x1, y1, x2, y2 in prohibited_zones:
            x1_px = int(x1 * SCALE_X)
            y1_px = int((MAP_WIDTH_MM - y2) * SCALE_Y)
            width_px = int((x2 - x1) * SCALE_X)
            height_px = int((y2 - y1) * SCALE_Y)
            pygame.draw.rect(screen, (255, 0, 0),
                             (x1_px, y1_px, width_px, height_px), 2)

        # Draw Path
        try:
            if path and len(path) >= 2:
                pts = [
                    (int(p_.x * SCALE_X),
                     int((MAP_WIDTH_MM - p_.y) * SCALE_Y))
                    for p_ in path
                ]
                pygame.draw.lines(screen, (0, 0, 255), False, pts, 2)
        except Exception as e:
            print("Error drawing path:", e)

        # Draw Enemy Robot
        ex = int(enemy_robot_position.x * SCALE_X)
        ey = int((MAP_WIDTH_MM - enemy_robot_position.y) * SCALE_Y)
        pygame.draw.rect(screen, (255, 0, 0),
                         (ex - ENEMY_SIZE_PX // 2,
                          ey - ENEMY_SIZE_PX // 2,
                          ENEMY_SIZE_PX,
                          ENEMY_SIZE_PX))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    robot_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
