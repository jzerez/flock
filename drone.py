import numpy as np

class Drone:
    def __init__(self, repulsion, attraction, pos, vel):
        self.repulsion = repulsion
        self.attraction = attraction
        self.pos = pos
        self.vel = vel

    def step(self, force):
        """
        updates position based on velocity
        """
        self.pos += self.vel
        self.vel += force

    def calc_force(self, drone):
        """
        calculates the force that a neighboring drone exerts
        """
        vec, dist = self.calc_dist(self, drone.pos)
        return vec * (self.repulsion(dist) + self.attraction(dist))

    def calc_dist(self, neighboring_pos):
        """
        calculates the distance between the current drone and a neighboring position
        """
        vec = np.array([i[0] - i[1] for i in zip(self.pos,neighboring_pos)])
        dist = np.linalg.norm(dist)
        print(vec)
        return vec/dist, dist

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib import animation
    import time

    grid = (-5, 5)
    def r(dist):
        return 0.1/(dist**3)
    def a(dist):
        return 0
    d = Drone(r, a, np.array([0.0,0.0]), np.array([0.1, 0.1]))
    def calc_ft(d):
        v1 = [i[0] - i[1] for i in zip(grid, d.pos)]
        v2 = [i[0] - i[1] for i in zip(grid[::-1], d.pos)]
        f1 = np.array([r(v1[0]), 0])
        f2 = np.array([0, -r(v1[1])])
        f3 = np.array([-r(v2[0]), 0])
        f4 = np.array([0, r(v2[1])])
        ft = f1+f2+f3+f4
        print(ft)
        return ft
    plt.figure()
    ax = plt.axes(xlim=grid, ylim=grid)
    point, = ax.scatter([], [])
    def init():
        point.set_data(d.pos[0], d.pos[1])
        return point
    def animate(i):
        ft = calc_ft(d)
        point.set_data(d.pos[0], d.pos[1])
    for i in range(120):

        plt.scatter(d.pos[0], d.pos[1])
        plt.pause(0.01)
        d.step(ft)
        plt.draw()

    plt.show()
