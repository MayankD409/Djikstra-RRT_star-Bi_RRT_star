#!/usr/bin/env python3

import pygame
import numpy as np
import time
import heapq
import math

########## DEFINING A NODE CLASS TO STORE NODES AS OBJECTS ###############

class Node:

    def __init__(self, x, y, cost, parent_id):

        self.x = x
        self.y = y
        self.cost = cost
        self.parent_id = parent_id
    
    def __lt__(self,other):
        return self.cost < other.cost

########### DEFINING ACTIONS TO BE PERFORMED ##############
########### CALCULATING COST TO COME FOR ALL ACTIONS ########

def move_East(x,y,cost):
    x = x + 1
    cost = 1 + cost
    return x,y,cost

def move_West(x,y,cost):
    x = x - 1
    cost = 1 + cost
    return x,y,cost

def move_North(x,y,cost):
    y = y + 1
    cost = 1 + cost
    return x,y,cost

def move_South(x,y,cost):
    y = y - 1
    cost = 1 + cost
    return x,y,cost

def move_NorthEast(x,y,cost):
    x = x + 1
    y = y + 1
    cost = 1 + cost
    return x,y,cost

def move_NorthWest(x,y,cost):  #Traversing Diagonally will cost more because of the distance covered
    x = x - 1
    y = y + 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def move_SouthEast(x,y,cost):
    x = x + 1
    y = y - 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def move_SouthWest(x,y,cost):
    x = x -1
    y = y - 1
    cost = np.sqrt(2) + cost
    return x,y,cost

############ DEFINING A FUNCTION TO PERFORM ACTIONS THAT ARE DEFINED #########

def Action_set(move,x,y,cost):

    if move == 'West':
        return move_West(x,y,cost)
    elif move == 'East':
        return move_East(x,y,cost)
    elif move == 'North':
        return move_North(x,y,cost)
    elif move == 'South':
        return move_South(x,y,cost)
    elif move == 'NorthEast':
        return move_NorthEast(x,y,cost)
    elif move == 'NorthWest':
        return move_NorthWest(x,y,cost)
    elif move == 'SouthEast':
        return move_SouthEast(x,y,cost)
    elif move == 'SouthWest':
        return move_SouthWest(x,y,cost)
    else:
        return None

############ CONFIGURATION SPACE CONSTRUCTION WITH OBSTACLES ############

def is_point_inside_rectangle(x, y, vertices):
                x_min = min(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
                x_max = max(vertices[0][0], vertices[1][0], vertices[2][0], vertices[3][0])
                y_min = min(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
                y_max = max(vertices[0][1], vertices[1][1], vertices[2][1], vertices[3][1])
                return x_min <= x <= x_max and y_min <= y <= y_max

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

def is_point_inside_block(point, vertices):
    odd_nodes = False
    j = len(vertices) - 1
    for i in range(len(vertices)):
        if (vertices[i][1] < point[1] and vertices[j][1] >= point[1]) or (vertices[j][1] < point[1] and vertices[i][1] >= point[1]):
            if (vertices[i][0] + (point[1] - vertices[i][1]) / (vertices[j][1] - vertices[i][1]) * (vertices[j][0] - vertices[i][0])) < point[0]:
                odd_nodes = not odd_nodes
        j = i
    return odd_nodes

def C_obs_space(width,height):

    obs_space = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
            
        ####### CLEARANCE FOR THE OBSTACLES #######
            
            # Plotting Buffer Space for the Obstacles    
            rectangle1_vts = [(95, 500), (180, 500), (180, 95), (95, 95)]
            rectangle2_vts = [(270, 405), (355, 405), (355, 0), (270, 0)]
            rect1_buffer = is_point_inside_rectangle(x, y, rectangle1_vts)
            rect2_buffer = is_point_inside_rectangle(x, y, rectangle2_vts)

            hexa_buffer = is_point_inside_hexagon(x, y, 650, 250, 155)
            point = [x, y]
            v2_buffer = [(895, 455), (895, 370), (1015, 370), (1015, 130), (895, 130), (895, 45), (1105, 45), (1105, 455)]
            cblock_buffer = is_point_inside_block(point, v2_buffer)
            
            # Setting the line constrain to obatain the obstacle space with buffer
            if(cblock_buffer or rect1_buffer or rect2_buffer or hexa_buffer):
                obs_space[y, x] = 1
             
            # Plotting Actual Object Space Half Plane Equations
            rectangle1_vts = [(100, 500), (175, 500), (175, 100), (100, 100)]
            rectangle2_vts = [(275, 400), (350, 400), (350, 0), (275, 0)]
            rect1 = is_point_inside_rectangle(x, y, rectangle1_vts)
            rect2 = is_point_inside_rectangle(x, y, rectangle2_vts)

            hexa = is_point_inside_hexagon(x, y, 650, 250, 150)
            point = [x, y]
            v2 = [(900, 450), (900, 375), (1020, 375), (1020, 125), (900, 125), (900, 50), (1100, 50), (1100, 450)]
            cblock = is_point_inside_block(point, v2)

            # Setting the line constrain to obatain the obstacle space with buffer
            if(cblock or rect1 or rect2 or hexa):
                obs_space[y, x] = 2
                
                
####### DEFINING THE BOUNDARIES FOR CONFIGURATION SPACE ########
    for i in range(height):
        for j in range(6):
            obs_space[i][j] = 1
            obs_space[i][width - j - 1] = 1
    
    for i in range(width):
        for j in range(6):  # Mark the first 5 columns of the top and bottom boundaries as unreachable
            obs_space[j][i] = 1
            obs_space[height - j - 1][i] = 1  # Also mark the first 5 columns of the bottom boundary as unreachable

    return obs_space

########## TO SEE IF THE MOVE IS VALID OR NOT #########

def ValidMove(x, y, obs_space):

    e = obs_space.shape

    if( x > e[1] or x < 0 or y > e[0] or y < 0 ):
        return False
    
    else:
        try:
            if(obs_space[y][x] == 1  or obs_space[y][x]==2):
                return False
        except:
            pass
    return True

########## DEFINING A FUNCTION TO CHECK IF THE PRESENT NODE IS GOAL NODE ##########

def Check_goal(present, goal):

    if (present.x == goal.x) and (present.y == goal.y):
        return True
    else:
        return False

######### GENERATE UNIQUE KEY ##########

def key(node):
    key = 1022*node.x + 111*node.y 
    return key


########## DIJKSTRA ALGORITHM ###########

def dijkstra(start, goal,obs_space):

    if Check_goal(start, goal):
        return None,1
    goal_node = goal
    start_node = start
    
    moves = ['West','East','North','South','NorthEast','NorthWest','SouthEast','SouthWest']
    unexplored_nodes = {}  #List of all open nodes
    
    start_key = key(start_node) #Generating a unique key for identifying the node
    unexplored_nodes[(start_key)] = start_node
    
    explored_nodes = {} #List of all closed nodes
    priority_list = []  #List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_list, [start_node.cost, start_node]) #This Data structure will prioritize the node to be explored which has less cost.
    
    all_nodes = [] #stores all nodes that have been traversed, for visualization purposes.
    

    while (len(priority_list) != 0):
        
        #popping the first element in the priority list to create child nodes for exploration
        present_node = (heapq.heappop(priority_list))[1]
        #appending all child nodes so that the explored region of the map can be plotted.
        all_nodes.append([present_node.x, present_node.y])
        #creating a dict key for identfication of node individually
        present_id = key(present_node)
        #The program will exist if the present node is the goal node
        if Check_goal(present_node, goal_node):
            goal_node.parent_id = present_node.parent_id
            goal_node.cost = present_node.cost
            print("Goal Node found")
            return all_nodes,1

        if present_id in explored_nodes:  
            continue
        else:
            explored_nodes[present_id] = present_node
    #deleting the node from the open nodes list because it has been explored and further its child nodes will be generated   
        del unexplored_nodes[present_id]
    #For all actions in action set, a new child node has to be formed if it is not already explored
        for move in moves:
            x,y,cost = Action_set(move,present_node.x,present_node.y,present_node.cost)
   #Creating a node class object for all coordinates being explored
            new_node = Node(x,y,cost,present_node)  
   
            new_node_id = key(new_node) 
   
            if not ValidMove(new_node.x, new_node.y, obs_space):
                continue
            elif new_node_id in explored_nodes:
                continue
   
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_id = new_node.parent_id
            else:
                unexplored_nodes[new_node_id] = new_node
            
            heapq.heappush(priority_list, [ new_node.cost, new_node])
   
    return  all_nodes,0

########### BACKTRACK AND GENERATE SHORTEST PATH ############

def Backtrack(goal_node):  
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
    # vertices = [(575, 400), (704.9, 325), (704.9, 174), (575, 100), (445.1, 175), (445.1, 325)]
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

def plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space):
    pygame.init()
    screen = pygame.display.set_mode((1200, 500))
    screen.fill((190, 190, 190))
    pygame.display.set_caption("Path Planning")

    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    white = (255, 255, 255)
    light_grey = (190, 190, 190)
    dark_grey = (100, 100, 100)
    padding = 5
    center_x = 650
    side_length = 150
    center_y = 250
    # Draw obstacles
    padding_rect = pygame.Rect(padding, padding, 1200 - 2 * padding, 500 - 2 * padding)
    pygame.draw.rect(screen, white, padding_rect)
    draw_padded_hexagon(screen, light_grey, center_x, center_y, side_length, padding)
    draw_hexagon(screen, dark_grey, center_x, center_y, side_length)
    v2 = [(895, 455), (895, 370), (1015, 370), (1015, 130), (895, 130), (895, 45), (1105, 45), (1105, 455)]
    pygame.draw.polygon(screen, light_grey, v2)
    draw_C(screen, dark_grey)
    pygame.draw.rect(screen, light_grey, pygame.Rect(95, 0, 85, 405))
    pygame.draw.rect(screen, light_grey, pygame.Rect(270, 95, 85, 405))
    pygame.draw.rect(screen, dark_grey, pygame.Rect(100, 0, 75, 400))
    pygame.draw.rect(screen, dark_grey, pygame.Rect(275, 100, 75, 400))

    # Draw start and goal nodes
    pygame.draw.rect(screen, GREEN, (start_node.x, 500 - start_node.y, 3, 3))  # Invert y-axis for start node
    pygame.draw.rect(screen, RED, (goal_node.x, 500 - goal_node.y, 3, 3))  # Invert y-axis for goal node

    # Draw explored nodes
    for node in all_nodes:
        pygame.draw.rect(screen, (0, 255, 0), (node[0], 500 - node[1], 1, 1))  # Invert y-axis for explored nodes

    # Draw shortest path
    for i in range(len(x_path) - 1):
        pygame.draw.line(screen, RED, (x_path[i], 500 - y_path[i]), (x_path[i + 1], 500 - y_path[i + 1]))  # Invert y-axis for shortest path

    pygame.display.flip()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

######### CALLING ALL MY FUNCTIONS TO IMPLEMENT DIJKSTRA ALGORITHM ON A POINT ROBOT ###########

if __name__ == '__main__':

    width = 1200
    height = 500
    obs_space = C_obs_space(width, height)
    
    #### Taking start node coordinates as input from user #####
    start_coordinates = input("Enter coordinates for Start Node: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    end_coordinates = input("Enter coordinates for Start Node: ")
    e_x, e_y = end_coordinates.split()
    e_x = int(e_x)
    e_y = int(e_y)
	# Define start and goal nodes
    start_node = Node(s_x, s_y, 0, -1)  # Start node with cost 0 and no parent
    goal_node = Node(e_x, e_y, 0, -1)  # You can adjust the goal node coordinates as needed

    # Run Dijkstra algorithm
    all_nodes, found_goal = dijkstra(start_node, goal_node, obs_space)

    if found_goal:
        # Generate shortest path
        x_path, y_path = Backtrack(goal_node)

        # Plot the result using Pygame
        plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space)
    else:
        print("Goal not found.")