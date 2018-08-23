"""

A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import matplotlib.pyplot as plt
import math
from collections import OrderedDict
import timeit

show_animation = True
motion_model_2 = True


class Node:

    def __init__(self, x, y, cost, pind, prev, direction):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        self.prev = prev
        self.direction = direction

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr, start):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1, None, None)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1, None, None)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, ".c")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        print([current.x,current.y, ngoal.x, ngoal.y])
        if current.x == ngoal.x and current.y == ngoal.y:
            stop = timeit.default_timer()
            print('Time: ', stop - start)  
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break
        
        if abs(current.x - ngoal.x) <= math.sqrt(2) and abs(current.y - ngoal.y) <= math.sqrt(2):
            motion = [[1, 0, 0, "E"],
                [0, 1, 0, "N"],
                [-1, 0, 0, "W"],
                [0, -1, 0, "S"],
                [-1, -1, math.sqrt(2), "SW"],
                [-1, 1, math.sqrt(2), "NW"],
                [1, -1, math.sqrt(2), "SE"],
                [1, 1, math.sqrt(2), "NE"]]

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id, current, motion[i][3])
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            # did_hit = hit_obstacle(node, obmap)
            # if not did_hit [0]: #collision logic
            #     print(did_hit)
            #     continue 

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node

            if abs(nstart.x - node.x) < abs(nstart.x - current.x) and abs(nstart.y - node.y) < abs(nstart.y - current.y):
               node.cost += 10 

            tcost = current.cost + calc_heuristic(current, node)

            if tcost >= node.cost:
                continue  # this is not a better path

            node.cost = tcost
            openset[n_id] = node  # This path is the best unitl now. record it!
        
    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1  # weight of distance

    x = abs(n1.x - n2.x)
    y = abs(n1.y - n2.y) 

    d = w * math.hypot(x, y)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True

# def hit_obstacle(node, obmap):
#     if obmap[node.x][node.y]:
#         if node.direction == "N" or node.direction == "S":
#             return [False, node.direction, node.x]
#         else:
#             return [False, node.direction, node.y]

#     return [True]


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break
    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    if motion_model_2:
        return [[2, 0, 0, "E"],
                [0, 2, 0, "N"],
                [-2, 0, 0, "W"],
                [0, -2, 0, "S"],
                [-2, -2, math.sqrt(4), "SW"],
                [-2, 2, math.sqrt(4), "NW"],
                [2, -2, math.sqrt(4), "SE"],
                [2, 2, math.sqrt(4), "NE"]]
    else:
        return [[1, 0, 0, "E"],
                [0, 1, 0, "N"],
                [-1, 0, 0, "W"],
                [0, -1, 0, "S"],
                [-1, -1, math.sqrt(2), "SW"],
                [-1, 1, math.sqrt(2), "NW"],
                [1, -1, math.sqrt(2), "SE"],
                [1, 1, math.sqrt(2), "NE"]]


    return motion


def main():
    start = timeit.default_timer()
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 51.0  # [m]
    gy = 51.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size, start)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()