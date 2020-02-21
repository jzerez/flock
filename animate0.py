from drone import *
from boid import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time
v0 = np.array([0.0,0.0])
drones = []
for i in range(15):
    r = 5
    pos = np.random.rand(2,) * 5 - r/2
    vel = unit(np.random.rand(2,)) * 0.1
    drones.append(Boid(pos, vel))
carrot = Carrot(np.array([0.0,0.0]))
# d1 = Drone(None, None, np.array([1.0,1.0]), v0)
# d2 = Drone(None, None, np.array([3.0,4.0]), v0)
fig = plt.figure()
# drones = [d1, d2]

plt.axis('equal')
plt.title('thing')

plt.xlabel('x')
plt.ylabel('y')
plt.grid(True)
colors = ['r', 'b', 'g', 'y', 'k', 'm', 'orange', 'navy', 'aqua', 'violet', 'salmon', 'lime', 'indigo', 'pink', 'plum', 'teal']
scat = plt.scatter([drone.pos[0] for drone in drones], [drone.pos[1] for drone in drones])
plt.xlim([-r, r])
plt.ylim([-r, r])


def update(f_num):
    goals = []
    plt.hold(False)
    plt.scatter(0,0,color='black', marker='*', s=20)
    plt.hold(True)

    plt.axis('equal')
    plt.grid('on')
    for i, drone in enumerate(drones):
        g = drone.calc_goal_vec(drones, carrot)
        goals.append(g)
        plt.scatter(drone.pos[0], drone.pos[1], color=colors[i])
        # plt.quiver(*drone.pos, g[0], g[1], color='black', width=0.003, headwidth=1, headlength=1)
        plt.quiver(*drone.pos, drone.vel[0], drone.vel[1], color='grey')
        # ns = drone.get_neighbor_vecs(drones, carrot=None, thresh_dist=(0,2), thresh_angle = 20, debug=False)
        # for n in ns:
        #     plt.plot([drone.pos[0], drone.pos[0]+n[0][0]], [drone.pos[1], drone.pos[1]+n[0][1]], color='black')
        plt.xlim([-r, r])
        plt.ylim([-r, r])
    for goal, drone in zip(goals, drones):
        drone.step(goal)
    scat.set_offsets([drone.pos for drone in drones])

ani = animation.FuncAnimation(fig, update, frames=500, interval=10, repeat=False)
plt.show()
