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

def C_obs_space(width,height):

    obs_space = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
            
        ####### CLEARANCE FOR THE OBSTACLES #######
            
            # Plotting Buffer Space for the Obstacles using Half Plane Equations
            
            # Rectangle 1 Obastacle
            r11_buffer = (x + 5) - 100  
            r12_buffer = (y + 5) - 100
            r13_buffer = (x - 5) - 175
            # r14_buffer = (y - 5) - 500    # No need to define lower most line at boundry
            
            # Rectangle 2 Obastacle
            r21_buffer = (x + 5) - 275  
            # r22_buffer = (y - 5) - 0  # No need to define lower most line at boundry
            r23_buffer = (x - 5) - 350
            r24_buffer = (y - 5) - 400 
            
            # Hexagon Obstacle
            h6_buffer = (y + 5) +  0.58*(x + 5) - 431.82
            h5_buffer = (y + 5) + 0.58*(x - 5) - 231.72
            h4_buffer = (x - 6.5) - 704.9
            h3_buffer = (y - 5) + 0.58*(x - 5) - 731.78
            h2_buffer = (y - 5) - 0.58*(x + 5) - 68.23
            h1_buffer = (x + 6.5) - 445.1
            
            # Block Obstacle
            t1_buffer = (x + 5) - 900
            t2_buffer = (x + 5) - 1020
            t3_buffer = (x - 5) - 1100
            t4_buffer = (y + 5) - 50
            t5_buffer = (y - 5) - 125
            t6_buffer = (y + 5) - 375
            t7_buffer = (y - 5) - 450
            
            # Setting the line constrain to obatain the obstacle space with buffer
            if((t1_buffer>0 and t2_buffer<0 and t4_buffer>0 and t5_buffer<0) or(t2_buffer>0 and t3_buffer<0 and t4_buffer>0 and t7_buffer<0) or (t6_buffer>0 and t7_buffer<0 and t1_buffer>0 and t2_buffer<0) or (r11_buffer>0 and r12_buffer>0 and r13_buffer<0) or (r21_buffer>0 and r23_buffer<0 and r24_buffer<0) or (h6_buffer<0 and h5_buffer>0 and h4_buffer>0 and h3_buffer>0 and h2_buffer>0 and h1_buffer<0)):
                obs_space[y, x] = 1
             
             
            # Plotting Actual Object Space Half Plane Equations
            
            # Rectangle 1 Obastacle
            r11 = (x) - 100  
            r12 = (y) - 100
            r13 = (x) - 175
            # r14 = (y) - 500
            
            # Rectangle 2 Obastacle
            r21 = (x) - 275  
            # r22 = (y) - 0
            r24 = (x) - 350
            r23 = (y) - 400 
            
            # Hexagon Obstacle
            h6 = (y) +  0.58*(x) - 431.82
            h5 = (y) - 0.58*(x) + 231.72
            h4 = (x) - 704.9
            h3 = (y) + 0.58*(x) - 731.78
            h2 = (y) - 0.58*(x) - 68.23
            h1 = (x) - 445.1 
            
            # Block Obstacle
            t1 = (x) - 900
            t2 = (x) - 1020
            t3 = (x) - 1100
            t4 = (y) - 50
            t5 = (y) - 125
            t6 = (y) - 375
            t7 = (y) - 450

            # Setting the line constrain to obatain the obstacle space with buffer
            if((h6>0 and h5>0 and h4<0 and h3<0 and h2<0 and h1>0) or (r11>0 and r12>0 and r13<0 ) or (r21>0  and r23<0 and r24<0) or (t1>0 and t2<0 and t4>0 and t5<0) or (t2>0 and t3<0 and t4>0 and t7<0) or (t6>0 and t7<0 and t1>0 and t2<0)):
                obs_space[y, x] = 2
                
                
####### DEFINING THE BOUNDARIES FOR CONFIGURATION SPACE ########

    for i in range(1200):
        obs_space[0][i] = 1
        
    for i in range(1200):
        obs_space[499][i] = 1
        
    for i in range(500):
        obs_space[i][1] = 1
        
    for i in range(500):
        obs_space[i][1199] = 1
       
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
    screen.fill((255, 255, 255))
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