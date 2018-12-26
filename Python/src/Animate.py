import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

ax = plt.axes(xlim=(-1, 1), ylim=(-1, 1))
elevator = plt.Rectangle((0.1, 0.2), 0.2, 0.4)

def init():
    elevator.set_x(0.1)
    elevator.set_y(0.2)
    ax.add_patch(elevator)
    return elevator

def animate(i):
    elevator.set_y(position)
    return elevator

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=254,
                               interval=1,
                               blit=True)

plt.show()