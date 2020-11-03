import numpy as np
import pygame
import math
import time

class RRT:

    # class constructor
    def __init__(self, start, goal, obstacle_list, max_iterations=250, max_distance=15):
        self.start = (start[0], start[1])   # starting point
        self.goal = (goal[0], goal[1])      # destination area
        self.obstacles = obstacle_list      # obstacles
        self.max_iterations = max_iterations    # maximum number of iterations
        self.path_tree = {self.start : -1}                 # the tree to be built from the nodes
        self.iter_count = 0                 # counter to keep track of the number of iterations
        self.goal_found = False             # this flag will be set as 'True' if the goal is found
        self.max_distance = max_distance    # maximum distance between nodes
        self.obstacle_hit = False

        # initialize display
        pygame.init()
        self.surface = pygame.display.set_mode((300, 300))
        self.surface.fill((255, 255, 255))

        # draws all obstacles, if there are any
        if self.obstacles is not None:
            for obstacle in self.obstacles:
                pygame.draw.circle(self.surface, (0, 0, 0), (obstacle[0], obstacle[1]), obstacle[2])

        # draw start and goal areas
        pygame.draw.circle(self.surface, (255, 0, 0), self.start, 3)
        pygame.draw.circle(self.surface, (0, 0, 255), self.goal, 10)
        pygame.display.update()

    # randomly samples a point in space
    def random_sample(self):
        sample_point = (np.random.randint(0, 300), np.random.randint(0, 300))
        if self.iter_count <= self.max_iterations:
            pygame.draw.circle(self.surface, (0, 255, 0), sample_point, 2)
            pygame.display.update()
            return sample_point

    # returns the euclidean distance between two points
    def distance(self, point_1, point_2):
        return math.sqrt(((point_1[0] - point_2[0])**2) + ((point_1[1] - point_2[1])**2))

    # chains the valid sample point to tree
    def chain_to_tree(self, point, tree):
        self.obstacle_hit = False
        min = float('infinity')
        if tree is not True:
            chain_destination = self.start

        for node in tree:
            if self.distance(point, node) < min:
                min = self.distance(point, node)
                chain_destination = node

        try:
            slope = (point[1] - float(chain_destination[1])) / (point[0] - float(chain_destination[0]))
            theta = math.atan(slope)
            theta_deg = math.degrees(theta)
            print("Iteration: " + str(self.iter_count) + " | Angle: " + str(theta_deg) + " | Coords: " + str(
                chain_destination) + " | Coords2: " + str(point))
            if point[0] > chain_destination[0] and point[1] < chain_destination[1]:
                new_point = (chain_destination[0] + (self.max_distance * math.cos(theta)),
                             chain_destination[1] + (self.max_distance * math.sin(theta)))
            elif point[0] < chain_destination[0] and point[1] < chain_destination[1]:
                new_point = (chain_destination[0] - (self.max_distance * math.cos(theta)),
                             chain_destination[1] - (self.max_distance * math.sin(theta)))
            elif point[0] < chain_destination[0] and point[1] > chain_destination[1]:
                new_point = (chain_destination[0] - (self.max_distance * math.cos(theta)),
                             chain_destination[1] - (self.max_distance * math.sin(theta)))
            else:
                new_point = (chain_destination[0] + (self.max_distance * math.cos(theta)),
                             chain_destination[1] + (self.max_distance * math.sin(theta)))
        except ZeroDivisionError:
            self.obstacle_hit = True

        if self.distance(new_point, self.goal) > 10:
            if self.obstacles is not None:
                for obst in self.obstacles:
                    if self.distance(new_point, (obst[0], obst[1])) < obst[2]:
                        self.obstacle_hit = True
        else:
            self.goal_found = True

        if self.obstacle_hit == False:
            self.path_tree[new_point] = chain_destination
            pygame.draw.line(self.surface, (0, 0, 255), chain_destination, new_point)
            pygame.display.update()
            time.sleep(0.001)

    # the driver function that performs the search
    def driver(self):
        while self.iter_count <= self.max_iterations:
            new_point = self.random_sample()
            self.chain_to_tree(new_point, self.path_tree)
            self.iter_count += 1
            if self.goal_found == True:
                print("==== GOAL FOUND ====")
                time.sleep(5)
                exit()
def distance(point_1, point_2):
        return math.sqrt(((point_1[0] - point_2[0])**2) + ((point_1[1] - point_2[1])**2))

def start_goal_obstacle_randomizer():
    temp_goal_collide = True
    obstacle_collide = True
    obstacles = []
    obstacle_limit = np.random.randint(1, 5)
    obstacle_count = 0
    start = (np.random.randint(0, 300), np.random.randint(0, 300))
    while temp_goal_collide:
        temp_goal = (np.random.randint(0, 300), np.random.randint(0, 300))
        if temp_goal != start and distance(start, temp_goal) > 10:
            goal = temp_goal
            break

    while obstacle_collide and obstacle_count <= obstacle_limit:
        temp_obstacle = (np.random.randint(0, 300), np.random.randint(0, 300), np.random.randint(5, 30))
        if (temp_obstacle[0], temp_obstacle[1]) != start and (temp_obstacle[0], temp_obstacle[1]) != goal and distance((temp_obstacle[0], temp_obstacle[1]), start) > temp_obstacle[2] and distance((temp_obstacle[0], temp_obstacle[1]), goal) > temp_obstacle[2]:
            obstacles.append(temp_obstacle)
            obstacle_count += 1

    return (start, goal, obstacles)

def main():
    start, goal, obstacles = start_goal_obstacle_randomizer()
    print(start, goal)
    print(obstacles)
    max_iterations = 500
    rrt_object = RRT(start, goal, obstacles, max_iterations)
    rrt_object.driver()

if __name__ == '__main__':
    main()












