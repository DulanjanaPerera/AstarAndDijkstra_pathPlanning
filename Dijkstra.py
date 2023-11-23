import sys
import typing


class Dijkstra:
    def __init__(self, start: list, goal: list, env: typing.Any) -> None:
        self.env = env
        self.r = self.env.shape[0]
        self.c = self.env.shape[1]
        self.start = start[0] * self.c + start[1] # convert in to index
        self.goal = goal[0] * self.c + goal[1] # convert in to index
        self.vertices = self.r * self.c
        self.distance = {}
        self.nodes = {}
        self.best_previous = {}
        self.best_path = None
        self.__initialization()

    def __initialization(self) -> None:
        for v in range(self.vertices):
            self.distance[v] = sys.maxsize
            self.best_previous[v] = None
            self.nodes[v] = v
        self.distance[self.start] = 0

    def search(self) -> None:
        while len(self.nodes) > 0:
            u = self.__minDistance(self.distance)
            if u == self.goal:
                break
            self.nodes.pop(u)

            neb = self.__neighbor(u)
            for n in neb:
                r = n // self.c  # convert index to row
                c = n % self.c  # convert index to column

                temp_dist = self.distance[u] + self.env[r][c]
                if temp_dist < self.distance[n]:
                    self.distance[n] = temp_dist
                    self.best_previous[n] = u
        self.best_path = self.__path()

    def __path(self) -> list:
        node = self.goal
        path = [node]
        while node != self.start:
            node = self.best_previous[node]
            path.append(node)
        path. reverse()
        return path

    def __minDistance(self, dist) -> int:

        """
        search through the distance dictionary excluding the visited nodes

        :param dist:
        :return index:
        """
        val = sys.maxsize
        index = None
        for ind in range(self.vertices):
            if (val > self.distance[ind]) and (ind in self.nodes):
                val = self.distance[ind]
                index = ind

        return index

    def __neighbor(self, u: int) -> list:
        r, c = self.env.shape[0], self.env.shape[1]
        top = u - c
        left = u - 1
        right = u + 1
        bottom = u + c

        neighbor = []

        # check the indeces are valid in terms of grid location and non-visited condition
        if top >= 0 and top in self.nodes:
            neighbor.append(top)
        if left >= (u // c) * c and left in self.nodes:
            neighbor.append(left)
        if bottom < r * c and bottom in self.nodes:
            neighbor.append(bottom)
        if right < (u // c + 1) * c and right in self.nodes:
            neighbor.append(right)
        return neighbor
