import pygame
import numpy as np
import time
import math
import os
import cv2
import random
from collections import deque

class Node:
    def __init__(self, x, y, parent, theta, rpm_left, rpm_right, c2c, c2g, cst):
        self.x = x
        self.y = y
        self.parent = parent
        self.theta = theta
        self.rpm_left = rpm_left
        self.rpm_right = rpm_right
        self.c2c = c2c
        self.c2g = c2g
        self.cst = cst
        
    def __lt__(self, other):
        return self.cst < other.cst

def steer(nearest, towards, step_size, robot_radius, clearance):
    theta = math.atan2(towards.y - nearest.y, towards.x - nearest.x)
    distance = min(step_size, euclidean_distance((nearest.x, nearest.y), (towards.x, towards.y)))
    new_x = nearest.x + distance * math.cos(theta)
    new_y = nearest.y + distance * math.sin(theta)
    if is_valid(new_x, new_y, robot_radius, clearance):
        new_node = Node(new_x, new_y, nearest, theta, 0, 0, nearest.c2c + distance, 0, nearest.c2c + distance)
        return new_node
    return None

def connect_trees(node_from_start, node_from_goal, step_size):
    if node_from_goal and node_from_start:
        if euclidean_distance((node_from_start.x, node_from_start.y), (node_from_goal.x, node_from_goal.y)) <= 2*step_size:
            original_parent_goal = node_from_goal.parent
            node_from_goal.parent = node_from_start
            return True, original_parent_goal
    return False, None

def find_nearby_nodes(tree, new_node, radius):
    nearby_nodes = []
    for node in tree:
        if euclidean_distance((node.x, node.y), (new_node.x, new_node.y)) < radius:
            nearby_nodes.append(node)
    return nearby_nodes

def choose_optimal_parent(tree, new_node, radius):
    nearby_nodes = find_nearby_nodes(tree, new_node, radius)
    if not nearby_nodes:
        return None, float('inf')
    optimal_parent = min(nearby_nodes, key=lambda node: node.c2c + euclidean_distance((node.x, node.y), (new_node.x, new_node.y)))
    optimal_cost = optimal_parent.c2c + euclidean_distance((optimal_parent.x, optimal_parent.y), (new_node.x, new_node.y))
    return optimal_parent, optimal_cost

def rewire(tree, new_node, radius):
    for node in find_nearby_nodes(tree, new_node, radius):
        potential_cost = new_node.c2c + euclidean_distance((node.x, node.y), (new_node.x, new_node.y))
        if potential_cost < node.c2c:
            node.parent = new_node
            node.c2c = potential_cost
            node.cst = node.c2c + node.c2g

def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


def bi_rrt_star(start_position, goal_position, rpm1, rpm2, clearance, robot_radius):
    iterations = 5000
    step_size = 15
    radius = 20  # The radius within which to consider rewiring
    start_tree = [start_position]
    goal_tree = [goal_position]
    c_best = float('inf')
    c_min = euclidean_distance((start_position.x, start_position.y), (goal_position.x, goal_position.y))

    for i in range(iterations):
        if c_best < float('inf'):
            random_point = sample_in_ellipse(start_position, goal_position, c_best, c_min)
        else:
            random_point = Node(random.uniform(0, 600), random.uniform(0, 200), None, 0, rpm1, rpm2, 0, 0, 0)

        tree = start_tree if i % 2 == 0 else goal_tree
        other_tree = goal_tree if i % 2 == 0 else start_tree

        nearest_node = min(tree, key=lambda node: euclidean_distance((node.x, node.y), (random_point.x, random_point.y)))
        new_node = steer(nearest_node, random_point, step_size, robot_radius, clearance)

        if new_node:
            optimal_parent, optimal_cost = choose_optimal_parent(tree, new_node, radius)
            if optimal_parent:
                new_node.parent = optimal_parent
                new_node.c2c = optimal_cost
                new_node.cst = optimal_cost
            tree.append(new_node)
            rewire(tree, new_node, radius)

            nearest_other = min(other_tree, key=lambda node: euclidean_distance((node.x, node.y), (new_node.x, new_node.y)))

            connected, original_parent_goal = connect_trees(new_node, nearest_other, step_size)
            if connected:
                c_current = nearest_other.c2c + euclidean_distance((nearest_other.x, nearest_other.y), (new_node.x, new_node.y))
                c_best = min(c_best, c_current)
                if i % 2 == 0:
                    return 1, start_tree, goal_tree, new_node, nearest_other, original_parent_goal
                else:
                    return 1, start_tree, goal_tree, nearest_other, new_node, original_parent_goal

    return 0, start_tree, goal_tree, None, None, None

def sample_in_ellipse(start, goal, c_best, c_min):
    # Ellipse focal points
    f1 = np.array([start.x, start.y])
    f2 = np.array([goal.x, goal.y])
    center = (f1 + f2) / 2
    d = np.linalg.norm(f1 - f2)
    
    # Semi-major axis
    a = c_best / 2
    # Semi-minor axis
    b = math.sqrt(a**2 - (d / 2)**2)

    while True:
        # Generate a random point in a unit circle
        angle = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1))
        x_unit = r * math.cos(angle)
        y_unit = r * math.sin(angle)

        # Scale and rotate to the ellipse
        x_ellipse = a * x_unit
        y_ellipse = b * y_unit
        theta = math.atan2(goal.y - start.y, goal.x - start.x)
        rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        sample = np.dot(rotation, np.array([x_ellipse, y_ellipse])) + center
        
        if is_valid(sample[0], sample[1], 0, 0):
            return Node(sample[0], sample[1], None, 0, 0, 0, 0, 0, 0)

def backtrack(connection_node_start, connection_node_goal, original_parent_goal):  
    # Backtrack from connection node to start node
    current_node = connection_node_start
    path_start = []
    while current_node is not None:
        path_start.append((current_node.x, current_node.y))
        current_node = current_node.parent
        
    
    # Backtrack from connection node to goal node
    current_node = original_parent_goal
    path_goal = []
    while current_node is not None:
        path_goal.append((current_node.x, current_node.y))
        current_node = current_node.parent
    
    # Reverse the paths to get the correct order of nodes
    path_start.reverse()
    # path_goal.reverse()

    # Combine the paths
    x_path = []
    y_path = []

    # Add path from start node to connection node
    for node in path_start:
        x_path.append(node[0])
        y_path.append(node[1])

    # Add path from connection node to goal node
    for node in path_goal[1:]:  # Exclude the connection node
        x_path.append(node[0])
        y_path.append(node[1])
    
    return x_path, y_path

# Optimized path pruning with obstacle checking
def prune_path(x_path, y_path, robot_radius, clearance):
    pruned_x_path = [x_path[0]]
    pruned_y_path = [y_path[0]]
    queue = deque([(0, len(x_path) - 1)])  # Start and goal indices

    while queue:
        left_index, goal_index = queue.popleft()
        right_index = goal_index

        # Find the farthest reachable point directly from left_index
        while right_index > left_index and not is_path_obstructed(x_path[left_index], y_path[left_index], x_path[right_index], y_path[right_index], robot_radius, clearance):
            right_index -= 1

        # Adjust to stay within list bounds
        next_index = min(right_index + 1, goal_index)

        # The farthest point that can be reached without obstruction
        pruned_x_path.append(x_path[next_index])
        pruned_y_path.append(y_path[next_index])

        if next_index < goal_index:
            queue.append((next_index, goal_index))

    return pruned_x_path, pruned_y_path

# Improved function to check if a path between two points is obstructed
def is_path_obstructed(x1, y1, x2, y2, robot_radius, clearance):
    # Increase the number of steps for finer collision detection
    steps = max(abs(x2 - x1), abs(y2 - y1)) * 2  # Multiplying to increase granularity
    steps = int(steps)
    for step in range(steps + 1):
        t = step / steps
        x = int(round(x1 + t * (x2 - x1)))
        y = int(round(y1 + t * (y2 - y1)))
        if not is_valid(x, y, robot_radius, clearance):
            return False
    return True


# Function to create configuration space with obstacles
def is_valid(x, y, robot_radius, clearance):

    # Creating buffer space for obstacles    
    rectangle1_buffer_vts = [(150 - (robot_radius + clearance), 200), (175 + (robot_radius + clearance), 200), (175 + (robot_radius + clearance), 100 - (robot_radius + clearance)), (150 - (robot_radius + clearance), 100 - (robot_radius + clearance))]
    rectangle2_buffer_vts = [(250 - (robot_radius + clearance), 100 + (robot_radius + clearance)), (275 + (robot_radius + clearance), 100 - (robot_radius + clearance)), (275 + (robot_radius + clearance), 0), (250 - (robot_radius + clearance), 0)]

    rect1_buffer = is_point_inside_rectangle(x,y, rectangle1_buffer_vts)
    rect2_buffer = is_point_inside_rectangle(x, y, rectangle2_buffer_vts)
    circ_buffer = is_point_inside_circle(x, y, 420, 120, 120 + 2*(robot_radius + clearance))
    
    # Setting buffer space constraints to obtain obstacle space
    if rect1_buffer or rect2_buffer or circ_buffer:
        return False
    
    # Adding check if obstacle is in walls
    if x <= (robot_radius + clearance) or y >= 200 - (robot_radius + clearance) or x >= 600 - (robot_radius + clearance) or y <= (robot_radius + clearance):
        return False

    return True
                                               

def is_point_inside_rectangle(x, y, vertices):
    x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    return x_min <= x <= x_max and y_min <= y <= y_max

def is_point_inside_circle(x, y, center_x, center_y, diameter):
    radius = diameter / 2.0
    distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
    return distance <= radius

# clearance = 1.5
frame_rate = 30
def plot_path(start_node, goal_node, tree_a, tree_b, x_path, y_path, clearance, frame_rate):
    pygame.init()
    screen = pygame.display.set_mode((600, 200))
    clock = pygame.time.Clock()
    BLUE = (0, 0, 255)
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)
    LIGHT_GREY = (190, 190, 190)
    DARK_GREY = (100, 100, 100)
    padding = clearance
    center_x, center_y = 420, 120
    if not os.path.exists("bi_rrt_star_frames"):
        os.makedirs("bi_rrt_star_frames")

    frame_count = 0
    count = 0
    max_len = max(len(tree_a), len(tree_b))
    running = True
    # print(len(present_nodes))
    while running and frame_count < len(x_path)/2:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(LIGHT_GREY)
        padding_rect = pygame.Rect(padding, padding, 600 - 2 * padding, 200 - 2 * padding) # Background Canvas
        pygame.draw.rect(screen, WHITE, padding_rect) # Original Canvas
        pygame.draw.circle(screen, LIGHT_GREY, (center_x, 200-center_y), 60 + padding)
        pygame.draw.circle(screen, DARK_GREY, (center_x, 200-center_y), 60)
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(150 - clearance, 0, 25 + 2 * clearance,
                                                         100 + clearance))  # Rectangle1 Clearance
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(250 - clearance, 100 - clearance, 25 + 2 * clearance,
                                                         100 + clearance))  # Rectangle2 Clearance
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(150, 0, 25, 100))  # Rectangle1 Obstacle
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(250, 100, 25, 100))  # Rectangle2 Obstacle

        # Draw start and goal points
        pygame.draw.rect(screen, RED, (start_node.x, 200 - start_node.y, 10, 10))  # Invert y-axis for start node
        pygame.draw.rect(screen, RED, (goal_node.x, 200 - goal_node.y, 10, 10))  # Invert y-axis for goal node

        for i in range(max_len):
            if i < len(tree_a):
                node = tree_a[i]
                if node.parent is not None:
                    pygame.draw.line(screen, RED, (node.parent.x, height - node.parent.y - 1), (node.x, height - node.y - 1), 1)
                pygame.draw.circle(screen, RED, (node.x, height - node.y - 1), 2)
            if i < len(tree_b):
                node = tree_b[i]
                if node.parent is not None:
                    pygame.draw.line(screen, (190, 190, 0), (node.parent.x, height - node.parent.y - 1), (node.x, height - node.y - 1), 1)
                pygame.draw.circle(screen, (190, 190, 0), (node.x, height - node.y - 1), 2)
            count+=1		
            pygame.image.save(screen, os.path.join("bi_rrt_star_frames", f"frame_{count}.png"))

        for i in range(len(x_path) - 1):
            pygame.draw.line(screen, BLUE, (x_path[i], 200 - y_path[i]), (x_path[i + 1], 200 - y_path[i + 1]), width=4)
            pygame.image.save(screen, os.path.join("bi_rrt_star_frames", f"frame_{count}.png"))

            count +=1
            frame_count += 1
            pygame.display.update()

        clock.tick(frame_rate)

    pygame.quit()


def frames_to_video(frames_dir, output_video):
    frames = [img for img in os.listdir(frames_dir) if img.endswith(".png")]
    frames.sort(key=lambda x: int(x.split("_")[1].split(".")[0]))  # Sort frames by frame number
    frame = cv2.imread(os.path.join(frames_dir, frames[0]))
    height, width, layers = frame.shape
    print("Creating Videowriter")
    video = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'mp4v'), 60, (width, height))
    print("Writing Video")
    for frame in frames:
        video.write(cv2.imread(os.path.join(frames_dir, frame)))

    cv2.destroyAllWindows()
    video.release()

if __name__ == '__main__':
    start_node = Node(50, 100, None, 0, 0, 0, 0, 0, 0)
    goal_node = Node(575, 100, None, 0, 0, 0, 0, 0, 0)
    width = 600
    height = 200
    clearance = 8
    frame_rate = 30
    save_dir = "bi_rrt_star_frames"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    timer_begin = time.time()   
    # flag, present_nodes, parent_nodes, connection_node_start, connection_node_goal = bi_rrt(start_node, goal_node, 5, 5, 10, 10)
    flag, start_tree, goal_tree, connection_node_start, connection_node_goal, original_parent_goal = bi_rrt_star(start_node, goal_node, 5, 5, clearance, 11)
    timer_end = time.time()
    print("Time taken to explore:", timer_end - timer_begin, "seconds")

    if flag:
        print("Path found")
        x_path, y_path = backtrack(connection_node_start, connection_node_goal, original_parent_goal)
        x_path, y_path = prune_path(x_path, y_path, 11, clearance)
        print("path:", x_path, y_path)
        plot_path(start_node, goal_node, start_tree, goal_tree, x_path, y_path, clearance, frame_rate)
        output_video = "bi_rrt_star.mp4"
        print("Generating Video")
        frames_to_video(save_dir, output_video)
        print("Video created successfully!")
    else:
        print("No path found")
