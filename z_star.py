from collections import namedtuple
from copy import deepcopy
from pprint import pprint
from roboCV.maps import grid
from roboCV.robot import Robot


class Node:
    def __init__(self, y, x, z):
        self.y = y
        self.x = x
        self.z = z
        self.g = 0
        self.h = 0
        self.score = 0
        self.parent = None

    def __repr__(self):
        return '(%d %d %d %d %d %d)' % (self.y, self.x, self.z, self.g, self.h, self.score)


def calc_heuristic(a, b):
    return abs(b.y - a.y) + abs(b.x - a.x)
    # return sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def get_neighbor_cells(_map, current_pos, closed_list):
    return [target_pos for target_pos in surround_area(current_pos)
            if (is_not_wall(target_pos, _map) and
                not is_in_list(target_pos, closed_list) and
                is_not_crossed(target_pos, current_pos))]


def is_not_wall(_position, _map):
    height = len(_map)
    width = len(_map[0])
    y = _position[0]
    x = _position[1]
    return 0 <= y < height and 0 <= x < width and not _map[y][x]


def is_in_list(pos, list):
    for item in list:
        if is_same_pos(item, pos):
            return True
    return False


def is_same_pos(a, b):
    if a.y == b[0] and a.x == b[1] and a.z == b[2]:
        return True
    return False


def is_not_crossed(target_pos, current_pos):
    z = target_pos[2]

    for robot in robots:
        for point in robot.path:
            if is_same_pos(point, target_pos) or is_x_crosed(target_pos, current_pos, point):
                return False

    return True


def is_x_crosed(target_pos, current_pos, point):
    return (is_same_pos(point.parent, (target_pos[0], target_pos[1], target_pos[2] - 1)) and
            is_same_pos(point, (current_pos.y, current_pos.x, current_pos.z + 1)))


Coords = namedtuple('Coords', 'y x z')


# TBC: 8 neighbors is tricky because diagonal movement is x1.4 times longer, but we should stay synchronized
# So, there are 5 cells, including stay on the same place
def surround_area(current_pos):
    y, x, z = current_pos.y, current_pos.x, current_pos.z
    return [(y - 1, x, z + 1), (y, x + 1, z + 1), (y + 1, x, z + 1), (y, x - 1, z + 1), (y, x, z + 1)]


def reconstruct_path_(current):
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    return path[::-1]


def find_node(cell, list):
    for item in list:
        if is_same_pos(item, cell):
            return item
    return None


def search(grid, start, end):
    openlist = []
    closedlist = []
    openlist.append(start)

    while openlist:
        current_pos = min(openlist, key=lambda o: (o.score, o.h))

        if current_pos.y == end.y and current_pos.x == end.x:
            return reconstruct_path_(current_pos)

        openlist.remove(current_pos)
        closedlist.append(current_pos)

        for cell in get_neighbor_cells(grid, current_pos, closedlist):
            if cell[0] == current_pos.y and cell[1] == current_pos.x:
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
                # print(node)
    return []


def visualize2(robots):
    map_path = deepcopy(grid)

    for robot in robots:

        start = Node(*robot.src, 0)
        end = Node(*robot.dst, 0)
        robot.path = search(grid, start, end)
        print(robot.name)
        pprint(robot.path)

        for point in robot.path:
            map_path[point.y][point.x] += 1

    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.imshow(map_path, interpolation='nearest', cmap='Blues')
    plt.grid(True)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))
    plt.show()


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
    ani.save('im.mp4', writer = FFwriter, fps=30)
    plt.show()


if __name__ == '__main__':
    robots = []
    m = len(grid)-1
    n = len(grid[0])-1
    # TODO: Check for border and obstacles
    robots.append(Robot('0a', (0, 0), (m, n)))
    robots.append(Robot('0b', (0, n), (m, 0)))
    robots.append(Robot('1a', (1, 0), (1, n)))
    robots.append(Robot('1b', (1, n), (1, 0)))
    robots.append(Robot('2a', (2, 0), (2, n)))
    robots.append(Robot('2b', (2, n), (2, 0)))
    robots.append(Robot('3a', (3, 0), (3, n)))
    robots.append(Robot('3b', (3, n), (3, 0)))
    robots.append(Robot('4a', (4, 0), (4, n)))
    robots.append(Robot('4b', (4, n), (4, 0)))
    robots.append(Robot('5a', (5, 0), (5, n)))
    robots.append(Robot('5b', (5, n), (5, 0)))
    robots.append(Robot('6a', (6, 0), (6, n)))
    robots.append(Robot('6b', (6, n), (6, 0)))
    robots.append(Robot('7a', (7, 0), (7, n)))
    robots.append(Robot('7b', (7, n), (7, 0)))
    robots.append(Robot('8a', (8, 0), (8, n)))
    robots.append(Robot('8b', (8, n), (8, 0)))
    robots.append(Robot('9a', (9, 0), (9, n)))
    robots.append(Robot('9b', (9, n), (9, 0)))
    robots.append(Robot('Wall-e', (m, 0), (0, n)))
    robots.append(Robot('Eva', (m, n), (0, 0)))

    # robots.append(Robot('Wall-e', (0, 0), (19, 29)))
    # robots.append(Robot('R2D2', (0, 29), (19, 0)))
    # robots.append(Robot('Zumo', (19, 0), (0, 29)))
    # robots.append(Robot('Eva', (19, 29), (0, 0)))
    for robot in robots:
        start = Node(*robot.src, 0)
        end = Node(*robot.dst, 0)
        robot.path = search(grid, start, end)
        print(robot.name)
        pprint(robot.path)

    visualize(grid, robots)
