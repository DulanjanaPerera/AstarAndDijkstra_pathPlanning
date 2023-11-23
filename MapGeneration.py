import numpy as np


class PerceptionMapper:
    def __init__(self, image, resolution):
        self.map = self.initialiseMap(image)
        # height, width
        self.size = self.map.shape
        self.defaultResolution = resolution

    def initialiseMap(self, testImage):
        env = np.ones(testImage.shape)  # keep 1 cost for empty area.
        for i in range(testImage.shape[0]):
            for j in range(testImage.shape[1]):
                if testImage[i][j] < 125:  # here I add 1000 for the obstacles
                    env[i][j] = 100

        return env

