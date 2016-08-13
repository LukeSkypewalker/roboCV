from collections import namedtuple
from pprint import pprint
from roboCV.maps import grid


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


class Robot:
    def __init__(self, path):
        self.path = path


def calc_heuristic(node, end):
    return abs(end.y - node.y) + abs(end.x - node.x)
    # return sqrt((end.x - node.x)**2 + (end.y - node.y)**2)


def get_neighbors_nodes(_map, current_pos, closedset):
    return [Node(*target_pos) for target_pos in surround_area(current_pos)
            if is_free(target_pos, _map, current_pos, closedset)]


def is_free(cell, _map, current_pos, visited_set):
    return is_not_wall(cell, _map) and is_not_visited(cell, visited_set) and is_not_crossed(cell, current_pos)


def is_not_wall(_position, _map):
    height = len(_map)
    width = len(_map[0])
    y = _position[0]
    x = _position[1]
    return 0 <= x < width and 0 <= y < height and not _map[y][x]


def is_not_visited(cell, closedset):
    for i in closedset:
        if i.y == cell[0] and i.x == cell[1]:
            return False
    return True


robots = []


def is_not_crossed(target_pos, current_pos):
    for robot in robots:
        for point in robot.path:
            if is_same_pos(point, target_pos) or is_x_crosed(target_pos, current_pos, point):
                return False
    return True


def is_same_pos(a, b):
    if a.y == b[0] and a.x == b[1] and a.z == b[2]:
        return True
    return False


def is_x_crosed(target_pos, current_pos, point):
    return (is_same_pos(point.parent, (target_pos[0], target_pos[1], target_pos[2] - 1)) and
            is_same_pos(point, (current_pos.y, current_pos.x, current_pos.z + 1)))


Coords = namedtuple('Coords', 'y x z')


# TBC: 8 neighbors is tricky because diagonal movement is x1.4 times longer, but we should stay synchronized
# So, there are 5 cells, including stay on the same place
def surround_area(current_pos):
    y, x, z = current_pos.y, current_pos.x, current_pos.z
    return [(y - 1, x, z + 1), (y, x + 1, z + 1), (y + 1, x, z + 1), (y, x - 1, z + 1),
            (y, x, z + 1)]  # , (y, x, z + 1)]stay and wait


def reconstruct_path_(current):
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    # path.append(current)
    return path[::-1]


def search(grid, start, end):
    openset = set()
    closedset = set()
    openset.add(start)

    while openset:
        current_pos = min(openset, key=lambda o: (o.score, o.h))

        if current_pos.x == end.x and current_pos.y == end.y:
            return reconstruct_path_(current_pos)

        openset.remove(current_pos)
        closedset.add(current_pos)

        for node in get_neighbors_nodes(grid, current_pos, closedset):
            new_g = current_pos.g + 1  # current.move_cost(node)
            if node in openset:
                if node.g > new_g:
                    node.g = new_g
                    node.parent = current_pos
            else:
                node.g = new_g
                node.h = calc_heuristic(node, end)
                node.score = node.g + node.h
                node.parent = current_pos
                openset.add(node)
    return []


if __name__ == '__main__':
    start = Node(1, 3, 0)
    end = Node(6, 3, 0)
    path = search(grid, start, end)
    print('path1')
    pprint(path)

    robots.append(Robot(path))

    start = Node(6, 3, 0)
    end = Node(1, 3, 0)
    path2 = search(grid, start, end)
    print('path2')
    pprint(path2)

    map_path = grid.copy()
    for coords in path:
        map_path[coords.y][coords.x] = 2
    for coords in path2:
        map_path[coords.y][coords.x] += 3

    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.imshow(map_path, interpolation='nearest', cmap='Blues')
    plt.grid(True)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))

    plt.show()
