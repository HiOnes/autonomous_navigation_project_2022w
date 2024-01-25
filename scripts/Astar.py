"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
from PIL import Image
import numpy as np

import plotting, env

forward_step = 5

def get_dwa_goal(global_path, curr_pos):
    global_path = np.array(global_path)
    curr_pos = np.array(curr_pos)
    dist = np.sum((global_path - curr_pos)**2, axis=1)
    min_idx = np.argmin(dist)
    print("min_idx: ", min_idx)
    print("global_path[min_idx]: ", global_path[min_idx])
    return global_path[min_idx - forward_step] if min_idx - forward_step >= 0 else global_path[0]

class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type, map_path):
        self.Env = env.Env(map_path)  # class Env

        self.s_start = (round(s_start[0]*self.Env.factor), round(s_start[1]*self.Env.factor))
        self.s_goal = (round(s_goal[0]*self.Env.factor), round(s_goal[1]*self.Env.factor))
        self.heuristic_type = heuristic_type

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.obs_x_average = self.Env.obs_x_average

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))
        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        path, normalized_path = self.extract_path(self.PARENT)
        return path, normalized_path, self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        normalized_path, path, visited = [], [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            normalized_p_k = (p_k[0]/self.Env.factor, p_k[1]/self.Env.factor)
            path.append(p_k)
            normalized_path.append(normalized_p_k)
            visited.append(v_k)
            e -= 0.5

        return path, normalized_path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        path, normalized_path = self.extract_path(self.PARENT)
        return path, normalized_path, CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        # path = [(self.s_goal[0]/self.Env.factor, self.s_goal[1]/self.Env.factor)]
        path = [self.s_goal]
        normalized_path = [(self.s_goal[0]/self.Env.factor, self.s_goal[1]/self.Env.factor)]
        s = self.s_goal

        while True:
            s = PARENT[s]
            normalized_s = (s[0]/self.Env.factor, s[1]/self.Env.factor)
            path.append(s)
            normalized_path.append(normalized_s)

            if s == self.s_start:
                break

        return list(path), list(normalized_path)

    def obstacle_punish(self, s):
        """
        Calculate obstacle punishment.
        :param s: current node (state)
        :return: obstacle punishment
        """
        manhattan_punish_factor = 1e-2
        euler_punish_factor = 1e-4
        if self.heuristic_type == "manhattan":
            return abs(s[0] - self.obs_x_average)*manhattan_punish_factor
        else:
            return math.pow(s[0] - self.obs_x_average, 2)*euler_punish_factor

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1]) - self.obstacle_punish(s)
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1]) - self.obstacle_punish(s)


def main():
    INIT_POSITION = [-2, 3, 1.57]  
    GOAL_POSITION = [INIT_POSITION[0], INIT_POSITION[1] + 10]  
    s_start = (INIT_POSITION[0], INIT_POSITION[1])
    s_goal = (GOAL_POSITION[0], GOAL_POSITION[1])
    map_path = "src/jackal_helper/worlds/BARN/map_files/map_pgm_50.pgm"
    astar = AStar(s_start, s_goal, "euclidean", map_path)
    plot = plotting.Plotting(astar.s_start, astar.s_goal, map_path)

    path, normalized_path, visited = astar.searching()
    plot.animation(path, visited, "A*")  # animation
    print(normalized_path)

    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
