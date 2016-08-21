from copy import deepcopy


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
    # ani.save('im.mp4', writer=FFwriter, fps=30)
    plt.show()