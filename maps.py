from pprint import pprint

map0 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

map_str = '''
0000000000
0000000000
0111110011
0111110011
0111110011
0111110011
0111110011
0000000000
0000000000
'''

def get_map(s):
    return [[int(i) for i in list(row)] for row in s.split()]

# grid = get_map(map_str)
grid = map0

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    pprint(grid)
    plt.figure(1)
    plt.imshow(grid, interpolation='nearest', cmap='Blues')
    plt.grid(True)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))

    plt.show()