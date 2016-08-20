from pprint import pprint

map_0 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

map_1 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

map_1 = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 1, 1, 1, 1, 0, 1, 1],
         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

map_y = [[0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0],
         [0, 0, 0, 1, 1],
         [0, 0, 0, 1, 1],
         [0, 0, 0, 1, 1],
         [0, 0, 0, 1, 1],
         [0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0]]

map_5 = [[0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0],
         [5, 5, 0, 5, 5],
         [5, 5, 0, 5, 5],
         [5, 5, 0, 5, 5],
         [5, 5, 0, 5, 5],
         [0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0]]

map_str10x10 = '''
0000000000
0000000000
1111111100
0000000000
1111101111
1111001111
1111011111
0000000000
1111111000
0000000000
'''

map_str76 = '''
000000
000000
111010
110010
110110
000000
000000
'''

map_str = '''
00001110010001001110010000000
00010000010010010001010000000
00011111011100010001010000000
00000001010010010001010000000
00001110010001001110011111000
00000000000000000000000000000
00011111011111001111010001000
00000100010000010000010001000
00000100011110010000011111000
00000100010000010000010001000
00000100011111001111010001000
'''


def get_map(s):
    return [[int(i) for i in list(row)] for row in s.split()]


grid = get_map(map_str)
# grid = map_1

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    pprint(grid)
    plt.figure(1)
    plt.imshow(grid, interpolation='nearest', cmap='Blues')
    plt.grid(True)
    plt.xticks(range(len(grid[0])))
    plt.yticks(range(len(grid)))

    plt.show()
