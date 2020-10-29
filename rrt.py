import numpy as np
import pygame
import math
import time

class RRT:

    def __init__(self, start, goal, obstacle_list, max_iterations=250):
        self.start = (start[0], start[1])
        self.goal = (goal[0], goal[1])
        self.obstacles = obstacle_list
        self.max_iterations = max_iterations
        self.path_tree = {}
        self.iter_count = 0

        pygame.init()
        self.surface = pygame.display.set_mode((300, 300))
        self.surface.fill((255, 255, 255))

        if self.obstacles is not None:
            pygame.draw.circle(self.surface, (0, 0, 0), (self.obstacles[0], self.obstacles[1]), self.obstacles[2])
        pygame.draw.circle(self.surface, (255, 0, 0), self.start, 3)
        pygame.draw.circle(self.surface, (0, 0, 255), self.goal, 10)
        pygame.display.update()

    def random_sample(self):
        point_x = np.random.randint(0, 300)
        point_y = np.random.randint(0, 300)
        if self.distance(point_x, point_y, self.goal) > 10 and self.iter_count <= self.max_iterations:
            if self.obstacles is not None:
                for obst in self.obstacles:
                    if self.distance(point_x, point_y, (obst[0], obst[1])) > obst[2]:
                        return (point_x, point_y)

            else:
                return (point_x, point_y)

        elif self.distance(point_x, point_y, self.goal) < 10:
            print('===== GOAL FOUND =====')
            return True

    def distance(self, x, y, center):
        return math.sqrt( ((x - center[0])**2) + ((y - center[1])**2) )

    def chain_to_tree(self, point, tree):
        min = float('infinity')
        if tree is not True:
            chain_destination = self.start

        if point == True:
            time.sleep(5)

        for node in tree:
            if self.distance(point[0], point[1], (node[0], node[1])) < min:
                min = self.distance(point[0], point[1], (node[0], node[1]))
                chain_destination = node
        self.path_tree[point] = chain_destination
        pygame.draw.line(self.surface, (0, 0, 255), chain_destination, point)
        pygame.display.update()

    def driver(self):
        while self.iter_count <= self.max_iterations:
            new_point = self.random_sample()
            self.chain_to_tree(new_point, self.path_tree)
            self.iter_count += 1
            if new_point == True:
                pass

def main():
    start = (50, 250)
    goal = (250, 50)
    obstacle_list = None
    max_iterations = 500
    rrt_object = RRT(start, goal, obstacle_list, max_iterations)
    rrt_object.driver()

if __name__ == '__main__':
    main()












