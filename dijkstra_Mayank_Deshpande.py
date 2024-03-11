import pygame
import numpy as np
import time
import heapq
import math
import os
import cv2

########## DEFINING A NODE CLASS TO STORE NODES AS OBJECTS ###############

class Node:
    def __init__(self, x, y, cost, parent_id):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_id = parent_id
    
    def __lt__(self, other):
        return self.cost < other.cost

########### DEFINING ACTIONS TO BE PERFORMED ##############
########### CALCULATING COST TO COME FOR ALL ACTIONS ########

def up(x, y, cost):
    x = x
    y = y + 1
    cost = cost + 1
    return x, y, cost

def down(x, y, cost):
    x = x
    y = y - 1
    cost = cost + 1
    return x, y, cost

def left(x, y, cost):
    x = x - 1
    y = y
    cost = cost + 1
    return x, y, cost

def right(x, y, cost):
    x = x + 1
    y = y
    cost = cost + 1
    return x, y, cost

def bottom_left(x, y, cost):
    x = x - 1
    y = y - 1
    cost = cost + 1.4
    return x, y, cost

def bottom_right(x, y, cost):
    x = x + 1
    y = y - 1
    cost = cost + 1.4
    return x, y, cost

def up_left(x, y, cost):
    x = x - 1
    y = y + 1
    cost = cost + 1.4
    return x, y, cost

def up_right(x, y, cost):
    x = x + 1
    y = y + 1
    cost = cost + 1.4
    return x, y, cost

############ CONFIGURATION SPACE CONSTRUCTION WITH OBSTACLES ############
#### Check if point is inside Rectangle ####
def is_point_inside_rectangle(x, y, vertices):
    x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
    y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
    return x_min <= x <= x_max and y_min <= y <= y_max

#### Check if point is inside hexagon ####
def is_point_inside_hexagon(x, y , center_x, center_y, side_length):
    cx, cy = center_x, center_y
    vertices = []
    angle_deg = 60
    angle_rad = math.radians(angle_deg)
    for i in range(6):
        px = cx + side_length * math.cos(angle_rad * i + math.radians(30))
        py = cy + side_length * math.sin(angle_rad * i + math.radians(30))
        vertices.append((px, py))
    odd_nodes = False
    j = 5
    for i in range(6):
        if (vertices[i][1] < y and vertices[j][1] >= y) or (vertices[j][1] < y and vertices[i][1] >= y):
            if (vertices[i][0] + (y - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) * (vertices[j][0] - vertices[i][0])) < x:
                odd_nodes = not odd_nodes
        j = i
    return odd_nodes
#### Check if point is inside C-Block ####
def is_point_inside_block(point, vertices):
    odd_nodes = False
    j = len(vertices) - 1
    for i in range(len(vertices)):
        if (vertices[i][1] < point[1] and vertices[j][1] >= point[1]) or (vertices[j][1] < point[1] and vertices[i][1] >= point[1]):
            if (vertices[i][0] + (point[1] - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) * (vertices[j][0] - vertices[i][0])) < point[0]:
                odd_nodes = not odd_nodes
        j = i
    return odd_nodes

#### Define the COnfiguration Space ####
def Configuration_space(width,height):
    obs_space = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
            
            ####### CLEARANCE FOR THE OBSTACLES #######
            point = [x, y]
            # Plotting Buffer Space for the Obstacles    
            rectangle1_buffer_vts = [(95, 500), (180, 500), (180, 95), (95, 95)]
            rectangle2_buffer_vts = [(270, 405), (355, 405), (355, 0), (270, 0)]
            cblock_buffer_vts = [(895, 455), (895, 370), (1015, 370), (1015, 130), (895, 130), (895, 45), (1105, 45), (1105, 455)]

            rect1_buffer = is_point_inside_rectangle(x, y, rectangle1_buffer_vts)
            rect2_buffer = is_point_inside_rectangle(x, y, rectangle2_buffer_vts)
            hexa_buffer = is_point_inside_hexagon(x, y, 650, 250, 155)
            cblock_buffer = is_point_inside_block(point, cblock_buffer_vts)
            
            # Setting the line constrain to obtain the obstacle space with buffer
            if(cblock_buffer or rect1_buffer or rect2_buffer or hexa_buffer):
                obs_space[y, x] = 1
             
            # Plotting Actual Object Space Half Plane Equations
            rectangle1_vts = [(100, 500), (175, 500), (175, 100), (100, 100)]
            rectangle2_vts = [(275, 400), (350, 400), (350, 0), (275, 0)]
            cblock_vertices = [(900, 450), (900, 375), (1020, 375), (1020, 125), (900, 125), (900, 50), (1100, 50), (1100, 450)]

            rect1 = is_point_inside_rectangle(x, y, rectangle1_vts)
            rect2 = is_point_inside_rectangle(x, y, rectangle2_vts)
            hexa = is_point_inside_hexagon(x, y, 650, 250, 150)
            cblock = is_point_inside_block(point, cblock_vertices)

            # Setting the line constrain to obtain the obstacle space with buffer
            if(cblock or rect1 or rect2 or hexa):
                obs_space[y, x] = 2
                
    ####### CLEARANCE FOR THE WALLS ########
    for i in range(height):
        for j in range(6):
            obs_space[i][j] = 1
            obs_space[i][width - j - 1] = 1
    
    for i in range(width):
        for j in range(6):  # Mark the first 5 columns of the top and bottom boundaries as unreachable
            obs_space[j][i] = 1
            obs_space[height - j - 1][i] = 1  # Also mark the first 5 columns of the bottom boundary as unreachable

    return obs_space

############## CHECK IF THE GIVEN MOVE IS VALID OR NOT ###############

def is_valid(x, y, obs_space):
    height, width = obs_space.shape
    
    # Check if coordinates are within the boundaries of the obstacle space and if the cell is occupied by an obstacle (value 1 or 2)
    if x < 0 or x >= width or y < 0 or y >= height or obs_space[y][x] == 1  or obs_space[y][x]==2:
        return False
    
    return obs_space[y, x] == 0

############## CHECK IF THE GOAL NODE IS REACHED ###############

def is_goal(present, goal):

    if (present.x == goal.x) and (present.y == goal.y):
        return True
    else:
        return False

############# DIJKSTRA ALGORITHM ###############

def dijkstra(start_node, goal_node, obs_space):
    if is_goal(start_node, goal_node):
        return None, 1
    
    possible_moves = [up, down, left, right, bottom_left, bottom_right, up_left, up_right]
    open_nodes = {}  # Dictionary of all open nodes with coordinates as keys
    open_nodes[(start_node.x, start_node.y)] = start_node
    
    closed_nodes = {}  # Dictionary of all closed nodes with coordinates as keys
    priority_queue = []  # List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_queue, [start_node.cost, start_node])  # Prioritize nodes with less cost
    
    traversed_nodes = []  # Store all traversed nodes for visualization
    
    while priority_queue:
        current_node = heapq.heappop(priority_queue)[1]
        traversed_nodes.append([current_node.x, current_node.y])
        current_node_coords = (current_node.x, current_node.y)
        
        if is_goal(current_node, goal_node):
            goal_node.parent_id = current_node.parent_id
            goal_node.cost = current_node.cost
            print("Goal Node found")
            return traversed_nodes, 1

        if current_node_coords in closed_nodes:  
            continue
        else:
            closed_nodes[current_node_coords] = current_node
        
        del open_nodes[current_node_coords]
        
        for move in possible_moves:
            x, y, cost = move(current_node.x, current_node.y, current_node.cost)
            new_node = Node(x, y, cost, current_node)  
            new_node_coords = (new_node.x, new_node.y) 
            
            if not is_valid(new_node.x, new_node.y, obs_space):
                continue
            elif new_node_coords in closed_nodes:
                continue

            if new_node_coords in open_nodes:
                if new_node.cost < open_nodes[new_node_coords].cost: 
                    open_nodes[new_node_coords].cost = new_node.cost
                    open_nodes[new_node_coords].parent_id = new_node.parent_id
            else:
                open_nodes[new_node_coords] = new_node
            
            heapq.heappush(priority_queue, [new_node.cost, new_node])
   
    return traversed_nodes, 0


########### BACKTRACK AND GENERATE SHORTEST PATH ############

def backtrack(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.parent_id
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.parent_id
        
    x_path.reverse()
    y_path.reverse()
    
    return x_path,y_path

#########  PLOT OBSTACLES SPACE, EXPLORED NODES, SHORTEST PATH  #######

# Function to draw a hexagon
def draw_hexagon(screen, color, center_x, center_y, side_length):
    vertices = []
    angle_deg = 60
    angle_rad = math.radians(angle_deg)
    for i in range(6):
        x = center_x + side_length * math.cos(angle_rad * i + math.radians(30))  # Adding 30 degrees to start with vertex up
        y = center_y + side_length * math.sin(angle_rad * i + math.radians(30))
        vertices.append((x, y))
    pygame.draw.polygon(screen, color, vertices)

# Function to draw a hexagon with padding
def draw_padded_hexagon(screen, color, center_x, center_y, side_length, padding):
    enlarged_side_length = side_length + padding
    draw_hexagon(screen, color, center_x, center_y, enlarged_side_length)

# Function to draw C obstacle
def draw_C(screen, color):
    vertices = [(900, 450), (900, 375), (1020, 375), (1020, 125), (900, 125), (900, 50), (1100, 50), (1100, 450)]
    pygame.draw.polygon(screen, color, vertices)

def plot_path(start_node, goal_node, x_path, y_path, all_nodes, frame_rate):
    BLUE = (0, 0, 255)
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)
    LIGHT_GREY = (190, 190, 190)
    DARK_GREY = (100, 100, 100)

    padding = 5
    center_x, center_y = 650, 250
    side_length = 150

    # Initialize Pygame and plot the map
    pygame.init()
    screen = pygame.display.set_mode((1200, 500))
    clock = pygame.time.Clock()
    if not os.path.exists("frames"):
        os.makedirs("frames")
    
    frame_count = 0
    # Counter to keep track of frames

    running = True
    while running and frame_count < len(all_nodes) + len(x_path) - 1:  # Terminate loop once all frames are saved
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill(LIGHT_GREY)
        padding_rect = pygame.Rect(padding, padding, 1200 - 2 * padding, 500 - 2 * padding)
        pygame.draw.rect(screen, WHITE, padding_rect)
        draw_padded_hexagon(screen, LIGHT_GREY, center_x, center_y, side_length, padding)
        draw_hexagon(screen, DARK_GREY, center_x, center_y, side_length)
        cblock_vertices = [(895, 455), (895, 370), (1015, 370), (1015, 130), (895, 130), (895, 45), (1105, 45), (1105, 455)]
        pygame.draw.polygon(screen, LIGHT_GREY, cblock_vertices)
        draw_C(screen, DARK_GREY)
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(95, 0, 85, 405)) # Rectangle1 Clearance 
        pygame.draw.rect(screen, LIGHT_GREY, pygame.Rect(270, 95, 85, 405)) # Rectangle1 Obstacle 
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(100, 0, 75, 400)) # Rectangle2 Clearance
        pygame.draw.rect(screen, DARK_GREY, pygame.Rect(275, 100, 75, 400)) # Rectangle2 Obstacle

        pygame.draw.rect(screen, RED, (start_node.x, 500 - start_node.y, 10, 10))  # Invert y-axis for start node
        pygame.draw.rect(screen, RED, (goal_node.x, 500 - goal_node.y, 10, 10))  # Invert y-axis for goal node

        for node in all_nodes:
            pygame.draw.rect(screen, (190, 190, 0), (node[0], 500 - node[1], 1, 1))  # Invert y-axis for explored nodes
            frame_count += 1
            if frame_count % 250 == 0:  # Save frame every 100th frame
                pygame.image.save(screen, os.path.join("frames", f"frame_{frame_count}.png"))
            pygame.display.update()

        for i in range(len(x_path) - 1):
            pygame.draw.line(screen, BLUE, (x_path[i], 500 - y_path[i]), (x_path[i + 1], 500 - y_path[i + 1]), width=4)
            pygame.image.save(screen, os.path.join("frames", f"frame_{frame_count}.png"))
            frame_count += 1
            pygame.display.update()
        

        clock.tick(frame_rate)  # Ensure frame rate

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
    width = 1200
    height = 500
    print("Wait few seconds for the input prompt...")
    obs_space = Configuration_space(width, height)
    
    # Taking start and end node coordinates as input from the user
    start_input_x = input("Enter the Start X: ")
    start_input_y = input("Enter the Start Y: ")

    start_x = int(start_input_x)
    start_y = int(start_input_y)

    end_input_x = input("Enter the End X: ")
    end_input_y = input("Enter the End Y: ")

    end_x = int(end_input_x)
    end_y = int(end_input_y)

    # Define start and goal nodes
    start_point = Node(start_x, start_y, 0, -1)  # Start node with cost 0 and no parent
    goal_point = Node(end_x, end_y, 0, -1)  # You can adjust the goal node coordinates as needed

    save_dir = "frames"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    timer_begin = time.time()
    traversed_nodes, goal_found = dijkstra(start_point, goal_point, obs_space)
    timer_end = time.time()
    print("Time taken to explore:", timer_end - timer_begin, "seconds")

    if goal_found:
        x_path, y_path = backtrack(goal_point)
        optimal_cost = goal_point.cost  # Cost of the optimal path
        print("Optimal path cost:", optimal_cost)
        plot_path(start_point, goal_point, x_path, y_path, traversed_nodes, frame_rate=30)
        output_video = "output_video.mp4"
        print("Generating Video")
        frames_to_video(save_dir, output_video)
        print("Video created successfully!")
    else:
        print("Goal not found!")

