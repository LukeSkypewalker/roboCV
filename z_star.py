from collections import namedtuple
from pprint import pprint

from roboCV.maps import grid
from roboCV.robot import Robot
from roboCV.visualization import visualize


class Node:
    def __init__(self, y, x, z=0):
        self.y = y
        self.x = x
        self.z = z
        self.g_cost = 0
        self.h_cost = 0
        self.score = 0
        self.parent = None

    def __repr__(self):
        return '(%d %d %d)' % (self.y, self.x, self.z)
        # return '(%d %d %d %d %d %d)' % (self.y, self.x, self.z, self.g_cost, self.h_cost, self.score)


Coords = namedtuple('Coords', 'y x z')


# TBC: 8 neighbors is tricky because diagonal movement is x1.4 times longer, but we should stay synchronized
# So, there are 5 cells, including stay on the same place
def surround_area(current_pos):
    y, x, z = current_pos.y, current_pos.x, current_pos.z
    return [Coords(y - 1, x, z + 1),
            Coords(y, x + 1, z + 1),
            Coords(y + 1, x, z + 1),
            Coords(y, x - 1, z + 1),
            Coords(y, x, z + 1)]  # stay on the same place


def neighbor_cells(_map, current_pos, closed_list, robot):
    return [target_pos for target_pos in surround_area(current_pos)
            if (is_not_wall(target_pos, _map) and
                not is_in_list(closed_list, target_pos) and
                is_not_crossed(current_pos, target_pos, robot))]


def is_not_wall(_position, _map):
    height = len(_map)
    width = len(_map[0])
    y = _position.y
    x = _position.x
    return 0 <= y < height and 0 <= x < width and not _map[y][x]


def is_in_list(list, pos):
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

        if is_same_pos(Coords(point.y, point.x, z), target_pos):
            return False

        if z < len(robot.path):
            if is_x_crosed(target_pos, current_pos, point):
                return False
    return True


def is_x_crosed(target_pos, current_pos, point):
    return (is_same_pos(point.parent, Coords(target_pos.y, target_pos.x, target_pos.z - 1)) and
            is_same_pos(point, Coords(current_pos.y, current_pos.x, current_pos.z + 1)))


def calc_heuristic(a, b):
    return abs(b.y - a.y) + abs(b.x - a.x)
    # return sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def reconstruct_path(current_pos):
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


def calc_heuristic(a, b):
    return abs(b.y - a.y) + abs(b.x - a.x)
    # return sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def search(grid, robot):
    openlist = []
    closedlist = []
    openlist.append(robot.path[0])

    # duct tape
    max_path_len_list = [len(robo.path) for robo in robots]
    max_path_len_list.append(0)  # otherwise empty list error
    z_max = max(max_path_len_list)-1  # index offset

    while openlist:
        current_pos = min(openlist, key=lambda o: (o.score, o.h_cost))
        openlist.remove(current_pos)
        closedlist.append(current_pos)

        for target_pos in neighbor_cells(grid, current_pos, closedlist, robot):

            if target_pos.z >= z_max and target_pos.y == robot.dst.y and target_pos.x == robot.dst.x:
                end = Node(*target_pos)
                end.parent = current_pos
                return reconstruct_path(end)

            new_g_cost = current_pos.g_cost + (0.5
                if target_pos.y == current_pos.y and target_pos.x == current_pos.x
                else 1)

            if is_in_list(openlist, target_pos):
                node = find_node(target_pos, openlist)
                if new_g_cost >= node.g_cost:
                    continue
            else:
                node = Node(*target_pos)
                openlist.append(node)
                node.h_cost = calc_heuristic(node, robot.dst)

            node.g_cost = new_g_cost
            node.score = new_g_cost + node.h_cost
            node.parent = current_pos
    return []


if __name__ == '__main__':
    m = len(grid) - 1
    n = len(grid[0]) - 1
    # TODO: Check for borders and obstacles before creating the robot
    robots = []
    bots = []
    # bots.append(Robot('Wall-e', Node(0, 0), Node(m, n)))
    # bots.append(Robot('Eva', Node(m, n), Node(0, 0)))
    # robots.append(Robot('Zumo', Node(m, 0), Node(0, n)))
    # robots.append(Robot('R2D2', Node(0, n), Node(m, 0)))

    for i in range(10 + 1):
        bots.append(Robot('a' + str(i), Node(m - i, 0),     Node(i,     n)))
        bots.append(Robot('b' + str(i), Node(i,     n),     Node(m - i, 0)))
        # bots.append(Robot('c' + str(i), Node(m - i, 2),     Node(i,     n - 2)))
        # bots.append(Robot('d' + str(i), Node(i,     n - 2), Node(m - i, 2)))
        # bots.append(Robot('e' + str(i), Node(m - i, 4),     Node(i,     n-4)))
        # bots.append(Robot('f' + str(i), Node(i,     n-4),   Node(m - i, 4)))
        # bots.append(Robot('g' + str(i), Node(m - i, 6),     Node(i,     n-6)))
        # bots.append(Robot('h' + str(i), Node(i,     n-6),   Node(m - i, 6)))
    for robot in bots:
        print(robot.name)
        path = search(grid, robot)
        [robot.path.append(point) for point in path]
        pprint(robot.path)
        robots.append(robot)

    visualize(grid, robots)
