import numpy as np
import os
from PIL import Image  # apt-get install imagemagick
import matplotlib.pyplot as plt

def bresenham(sx, sy, ex, ey):
    dx = int(abs(ex-sx))
    dy = int(abs(ey-sy))

    x = sx
    y = sy
    
    line = []

    if (dx > dy):
        d = 2 * dy - dx
        for i in range(0, dx):
            if (d < 0):
                d = d + 2 * dy
            else:
                if (ey > sy):
                    y = y + 1
                else: 
                    y = y - 1
                d = d + 2 * (dy - dx)
            
            line.append((int(x),int(y)))

            if (ex > sx):
                x = x + 1
            else:
                x = x - 1
    else:
        d = 2 * dx - dy
        for i in range(0, dy):
            if (d < 0):
                d = d + 2*dx
            else:
                if (ex > sx):
                    x = x +1
                else:
                    x = x -1
                d = d + 2 * (dx - dy)
            
            line.append((int(x),int(y)))

            if (ey > sy):
                y = y +1
            else:
                y = y -1
    
    return line


if __name__ == "__main__":
    WIDTH = 50
    HEIGHT = 50
    RESOLUTION = 0.5

    sizeX = int(WIDTH / RESOLUTION)
    sizeY = int(HEIGHT / RESOLUTION)
    centerX = sizeX / 2
    centerY = sizeY / 2

    costmap = np.zeros((sizeX, sizeY))

    test_vec = [[15, 10],  [20, 20],  [10, 15],   [0, 20],    [-10, 15],  [-20, 20],
                [-15, 10], [-20, 0],  [-15, -10], [-20, -20], [-10, -15], [0, -20],
                [10, -15], [20, -20], [15, -10],  [20, 0]]
    
    x = int(0/RESOLUTION) + centerX
    y = int(0/RESOLUTION) + centerY

    for vec in test_vec:
        vecX = int(vec[0]/RESOLUTION) + centerX
        vecY = int(vec[1]/RESOLUTION) + centerY
        line = bresenham(x, y, vecX, vecY)
        for p in line:
            costmap[p[0], p[1]] = 255

    img = Image.fromarray(costmap.astype(np.uint8))
    img.save("bresenham_test.jpeg")

