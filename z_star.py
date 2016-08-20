from collections import namedtuple
from copy import deepcopy
from pprint import pprint
from roboCV.maps import grid
from roboCV.robot import Robot


class Node:
    def __init__(self, y, x, z=0):
        self.y = y
        self.x = x
        self.z = z
        self.g = 0
        self.h = 0
        self.score = 0
        self.parent = None

    def __repr__(self):
        return '(%d %d %d)' % (self.y, self.x, self.z)
        # return '(%d %d %d %d %d %d)' % (self.y, self.x, self.z, self.g, self.h, self.score)


def calc_heuristic(a, b):
    return abs(b.y - a.y) + abs(b.x - a.x)
    # return sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def get_neighbor_cells(_map, current_pos, closed_list, robot):
    return [target_pos for target_pos in surround_area(current_pos)
            if (is_not_wall(target_pos, _map) and
                not is_in_list(target_pos, closed_list) and
                is_not_crossed(current_pos, target_pos, robot))]


def is_not_wall(_position, _map):
    height = len(_map)
    width = len(_map[0])
    y = _position.y
    x = _position.x
    return 0 <= y < height and 0 <= x < width and not _map[y][x]


def is_in_list(pos, list):
    for item in list:
        if is_same_pos(item, pos):
            return True
    return False


def is_same_pos(a, b):
    if a.y == b.y and a.x == b.x and a.z == b.z:
        return True
    return False


def is_not_crossed(current_pos, target_pos, this_robot):
    z = target_pos.z

    for robot in robots:

        if robot == this_robot:
            continue

        point = robot.path[z] if z < len(robot.path) else robot.path[-1]
        # print(z, point, current_pos, target_pos)

        if is_same_pos(Coords(point.y, point.x, z), target_pos):
            # print('alarm', point, target_pos)
            return False

        if z < len(robot.path):
            if is_x_crosed(target_pos, current_pos, point):
                return False
    return True


# def is_not_crossed2(target_pos, current_pos):
#     z = target_pos.z
#
#     for robot in robots:
#         for point in robot.path:
#             if is_same_pos(point, target_pos) or is_x_crosed(target_pos, current_pos, point):
#                 return False
#
#     return True


def is_x_crosed(target_pos, current_pos, point):
    return (is_same_pos(point.parent, Coords(target_pos.y, target_pos.x, target_pos.z - 1)) and
            is_same_pos(point, Coords(current_pos.y, current_pos.x, current_pos.z + 1)))


Coords = namedtuple('Coords', 'y x z')


# TBC: 8 neighbors is tricky because diagonal movement is x1.4 times longer, but we should stay synchronized
# So, there are 5 cells, including stay on the same place
def surround_area(current_pos):
    y, x, z = current_pos.y, current_pos.x, current_pos.z
    return [Coords(y - 1, x, z + 1),
            Coords(y, x + 1, z + 1),
            Coords(y + 1, x, z + 1),
            Coords(y, x - 1, z + 1),
            Coords(y, x, z + 1)]


def reconstruct_path_(current_pos):
    path = []
    while current_pos.parent:
        path.append(current_pos)
        current_pos = current_pos.parent
    return path[::-1]


def find_node(cell, list):
    for item in list:
        if is_same_pos(item, cell):
            return item
    return None


def search(grid, robot):
    start = robot.path[0]
    end = robot.dst
    openlist = []
    closedlist = []
    openlist.append(start)

    while openlist:
        current_pos = min(openlist, key=lambda o: (o.score, o.h))

        if current_pos.y == end.y and current_pos.x == end.x:
            return reconstruct_path_(current_pos)

        openlist.remove(current_pos)
        closedlist.append(current_pos)

        for cell in get_neighbor_cells(grid, current_pos, closedlist, robot):
            if cell.y == current_pos.y and cell.x == current_pos.x:
                move_cost = current_pos.g + 0.5
            else:
                move_cost = current_pos.g + 1  # current.move_cost(node)

            if is_in_list(cell, openlist):
                node = find_node(cell, openlist)
                if node.g > move_cost:
                    node.g = move_cost
                    node.parent = current_pos
            else:
                node = Node(*cell)
                node.g = move_cost
                node.h = calc_heuristic(node, end)
                node.score = node.g + node.h
                node.parent = current_pos
                openlist.append(node)
    return []


def visualize(grid, robots):
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig2 = plt.figure()
    plt.grid(True)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))

    frames = []
    n = max([len(robot.path) for robot in robots])
    for j in np.arange(n):
        frame = deepcopy(grid)
        for i, robot in enumerate(robots):
            if j < len(robot.path):
                frame[robot.path[j].y][robot.path[j].x] = i + 2
            else:
                frame[robot.path[-1].y][robot.path[-1].x] = i + 2

        frames.append((plt.imshow(frame, interpolation='nearest', cmap='jet'),))

    ani = animation.ArtistAnimation(fig2, frames, interval=500, repeat_delay=0, blit=False)
    plt.rcParams['animation.ffmpeg_path'] = 'D:\\SOFT\\ffmpeg\\bin\\ffmpeg'
    FFwriter = animation.FFMpegWriter()
    ani.save('im.mp4', writer=FFwriter, fps=30)
    plt.show()


if __name__ == '__main__':
    m = len(grid) - 1
    n = len(grid[0]) - 1
    # TODO: Check for border and obstacles
    robots = []
    bots = []
    # bots.append(Robot('Wall-e', Node(0, 0), Node(m, n)))
    # bots.append(Robot('Eva', Node(m, n), Node(0, 0)))
    # robots.append(Robot('Zumo', Node(m, 0), Node(0, n)))
    # robots.append(Robot('R2D2', Node(0, n), Node(m, 0)))

    for i in range(10 + 1):
        bots.append(Robot('a' + str(i), Node(m - i, 0), Node(i, n)))
        bots.append(Robot('b' + str(i), Node(i, n), Node(m - i, 0)))
        # bots.append(Robot('a' + str(i), Node(m - i, 1), Node(i, n-1)))
        # bots.append(Robot('b' + str(i), Node(i, n-1), Node(m - i, 1)))

    for robot in bots:
        print(robot.name)
        path = search(grid, robot)
        [robot.path.append(point) for point in path]
        pprint(robot.path)
        robots.append(robot)
    visualize(grid, robots)
