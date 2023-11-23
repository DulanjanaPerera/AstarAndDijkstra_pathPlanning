import numpy as np
from Dijkstra import Dijkstra
from Astar import Astar
import matplotlib.pyplot as plt
import matplotlib.image as img
from MapGeneration import PerceptionMapper
import sys
import os
import matplotlib.patches as patches
import time
import math

testImage0 = img.imread('pgmimg_1.000000.pgm')
env0 = PerceptionMapper(testImage0, 1)
map0 = env0.map

# Map of Rellis Campus with resolution of 1pixel = 4mt^2 (2x2)
testImage1 = img.imread('pgmimg_2.000000.pgm')
env1 = PerceptionMapper(testImage1, 2)
map1 = env1.map

# Map of Rellis Campus with resolution of 1pixel = 25mt^2 (5x5)
testImage2 = img.imread('pgmimg_5.000000.pgm')
env2 = PerceptionMapper(testImage2, 5)
map2 = env2.map

# Map of Rellis Campus with resolution of 1pixel = 100mt^2 (10x10)
testImage3 = img.imread('pgmimg_10.000000.pgm')
env3 = PerceptionMapper(testImage3, 10)
map3 = env3.map

environment = {0: env3, 1: env2, 2: env1, 3: env0}

nstart = [[224, 158], [436, 892]]
ngoal = [[232, 1468], [964, 870], [304, 72], [274, 840]]
count = 0
print("A* algorithm:\n")
for e_i in range(0, 1):
    env_ast = environment[e_i]
    map_ast = env_ast.map
    print("Map resolution: ", env_ast.defaultResolution, "\n")
    for s_i in range(0, 2):

        print("Source: ", nstart[s_i], "\n")
        start = [math.floor(x / env_ast.defaultResolution) for x in nstart[s_i]]

        for g_i in range(4):
            goal = [math.floor(x / env_ast.defaultResolution) for x in ngoal[g_i]]

            print("Start point: ", start, "\n\rEnd point: ", goal)

            st = time.time()
            ast = Astar(start, goal, env=map_ast)
            ast.search()
            et = time.time()
            elapsed_ast = et - st

            print(f"Elapsed time: {elapsed_ast} sec\n\n")

            path_ast = ast.best_path
            coordinates_ast = []
            for i in path_ast:
                # coordinates are changed to (column, row)
                coordinates_ast.append((i % env_ast.size[1], i // env_ast.size[1]))

            plt.imshow(map_ast)
            title_str = "A* | Res: " + str(env_ast.defaultResolution) + " | SP: " + str(start) + " | EP: " + str(
                goal) + " | t: " + str(round(elapsed_ast, 5))
            img_name = "A_R" + str(env_ast.defaultResolution) + "_" + str(count) + ".png"
            plt.title(title_str)
            line_s = coordinates_ast[0]
            for i_tup in coordinates_ast[1:]:
                line_e = i_tup
                line = patches.Polygon([line_s, line_e], fill=False, edgecolor='red', linewidth=1)
                plt.gca().add_patch(line)
                line_s = i_tup
            plt.savefig(os.path.join(os.getcwd(), 'figures', img_name), dpi=600)
            plt.show()
            count = count + 1
