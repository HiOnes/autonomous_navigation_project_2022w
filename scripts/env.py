"""
Env 2D
@author: huiming zhou
"""
from PIL import Image
import numpy as np

class Env:
    def __init__(self, map_path):
        self.map = np.array(Image.open(map_path))
        self.flation_radius = 0.2
        self.factor = 5
        self.flation_grid = round(self.flation_radius * self.factor)
        # self.x_range = 51  # size of background
        # self.y_range = 31
        self.x_range = self.map.shape[1]*self.factor
        self.y_range = self.map.shape[0]*self.factor
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs_x_average = -self.map.shape[1]*self.factor / 2
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        obs_indices = np.where(self.map < 112.5)  
        ob = np.array(list(zip(obs_indices[1]*0.15-4.5, (66-obs_indices[0])*0.15)))
        ob = np.rint(ob*self.factor)
        self.obs_x_average = np.mean(ob[:, 0])   
        obs = set()
        for i in range(len(ob)):
            obs.add((ob[i][0], ob[i][1]))
            for j in range(1, self.flation_grid + 1):
                obs.add((ob[i][0]+j, ob[i][1]))
                obs.add((ob[i][0], ob[i][1]+j))
                obs.add((ob[i][0]+j, ob[i][1]+j))
                obs.add((ob[i][0]-j, ob[i][1]))
                obs.add((ob[i][0], ob[i][1]-j))
                obs.add((ob[i][0]-j, ob[i][1]-j))
                obs.add((ob[i][0]+j, ob[i][1]-j))
                obs.add((ob[i][0]-j, ob[i][1]+j))

        return obs
