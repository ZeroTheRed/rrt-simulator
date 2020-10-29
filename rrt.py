import numpy as np
import pygame
import math
import time

class RRT:

    # class constructor
    def __init__(self, start, goal, obstacle_list, max_iterations=250):
        self.start = (start[0], start[1])   # starting point
        self.goal = (goal[0], goal[1])      # destination area
        self.obstacles = obstacle_list      # obstacles
        self.max_iterations = max_iterations    # maximum number of iterations
        self.path_tree = {}                 # the tree to be built from the nodes
        self.iter_count = 0                 # counter to keep track of the number of iterations
        self.goal_found = False             # this flag will be set as 'True' if the goal is found

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
        if self.distance(sample_point, self.goal) > 10 and self.iter_count <= self.max_iterations:
            if self.obstacles is not None:
                obstacle_collide_check = tuple(pygame.Surface.get_at(sample_point)[:3])
                if obstacle_collide_check != (0, 0, 0):
                    return sample_point

            else:
                return sample_point

        elif self.distance(sample_point, self.goal) < 10:
            print('===== GOAL FOUND =====')
            self.goal_found = True
            return sample_point

    # returns the euclidean distance between two points
    def distance(self, point_1, point_2):
        return math.sqrt( ((point_1[0] - point_2[0])**2) + ((point_1[1] - point_2[1])**2) )

    # chains the valid sample point to tree
    def chain_to_tree(self, point, tree):
        min = float('infinity')
        if tree is not True:
            chain_destination = self.start

        for node in tree:
            if self.distance(point, node) < min:
                min = self.distance(point, node)
                chain_destination = node
        self.path_tree[point] = chain_destination
        pygame.draw.line(self.surface, (0, 0, 255), chain_destination, point)
        pygame.display.update()

    # the driver function that performs the search
    def driver(self):
        while self.iter_count <= self.max_iterations:
            new_point = self.random_sample()
            self.chain_to_tree(new_point, self.path_tree)
            self.iter_count += 1
            if self.goal_found == True:
                time.sleep(5)
                exit()

def main():
    start = (50, 250)
    goal = (250, 50)
    obstacle_list = None
    max_iterations = 500
    rrt_object = RRT(start, goal, obstacle_list, max_iterations)
    rrt_object.driver()

if __name__ == '__main__':
    main()
