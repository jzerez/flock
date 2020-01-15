import numpy as np
import pdb
class Drone:
    def __init__(self, repulsion, attraction, pos, vel):
        self.repulsion = repulsion
        self.attraction = attraction
        self.pos = pos
        self.vel = vel
        self.vec_mag_limit = 1
        self.null_vector = np.array([0,0])

    def in_threshold(self, target, threshold):
        return target < threshold[1] and target > threshold[0]

    def get_neighbor_vecs(self, drones, threshold=(0,6), debug=False):
        """
        return the neighbors within a certain distance of the drone
        """
        neighbor_vecs = []
        for drone in drones:
            if drone is not self:
                vec, dist = self.calc_dist(drone.pos)
                if dist < 0.5 and debug: print('CRASH')
                if self.in_threshold(dist, threshold):
                    neighbor_vecs.append((vec, dist))
        return neighbor_vecs

    def calc_center_vector(self, drones, threshold=(0,6)):
        neighbor_vecs = self.get_neighbor_vecs(drones, threshold, debug=True)
        if not neighbor_vecs:
            return self.null_vector
        center = sum([vec[0] for vec in neighbor_vecs])/len(neighbor_vecs)
        center_vector = center
        return self.limit_vec(center_vector)

    def calc_dist(self, neighboring_pos):
        """
        calculates the distance between the current drone and a neighboring
        position

        returns each of the vectors and their magnitudes as a tuple of arrays
        """
        vec = np.array([i[1] - i[0] for i in zip(self.pos, neighboring_pos)])
        dist = np.linalg.norm(vec)
        return vec, dist

    def limit_vec(self, vec, limit=None):
        """
        truncates the magnitude of a given vector if above specfified magnitude

        returns truncated vector
        """
        mag = np.linalg.norm(vec)
        if not limit:
            limit = self.vec_mag_limit

        if mag < limit:
            return vec
        return vec * (limit / mag)
    def unit(self, vec):
        mag = np.linalg.norm(vec)
        return vec / mag

    def calc_goal_vec(self, drones):
        avoid_weight = -5
        attract_weight = 3
        center_weight = 3

        avoid_vec = avoid_weight * (self.calc_center_vector(drones, (0, 1)))
        if not np.array_equal(self.null_vector, avoid_vec):
            print('avoid')
            # plt.quiver(*self.pos, avoid_vec[0], avoid_vec[1])

        center_vec = center_weight * (self.calc_center_vector(drones, (0, 6)))
        print('avoid vec: ', avoid_vec)
        print('center vec: ', center_vec)
        return self.limit_vec(avoid_vec + center_vec)

    def step(self, goal_vec):
        mu = 0.3
        self.pos += self.vel
        self.vel = self.limit_vec(self.vel * (1-mu) + goal_vec * mu, 0.3)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib import animation
    import time
    v0 = (0,0)
    drones = []
    # d1 = Drone(None, None, np.array([1,1]), v0)
    # d2 = Drone(None, None, np.array([2,2]), v0)
    # d3 = Drone(None, None, np.array([3,3]), v0)
    # d4 = Drone(None, None, np.array([6,6]), v0)
    # d5 = Drone(None, None, np.array([10,10]), v0)
    # drones = [d1, d2, d3, d4, d5]
    # print(d1.calc_center_vector(drones, ))
    for i in range(3):
        r = 5
        pos = np.random.rand(2,) * 5 - r/2
        drones.append(Drone(None, None, pos, v0))
    # d1 = Drone(None, None, np.array([1.0,1.0]), v0)
    # d2 = Drone(None, None, np.array([3.0,4.0]), v0)
    # drones = [d1, d2]
    plt.figure()
    plt.axis('equal')
    colors = ['r', 'b', 'g', 'y', 'k', 'm', 'orange', 'navy', 'aqua', 'violet']
    for i, drone in enumerate(drones):
        # plt.scatter(drone.pos[0], drone.pos[1])
        g = drone.calc_goal_vec(drones)
        drone.vel = g
        # cen = drone.calc_center_vector(drones)
    #     plt.quiver(*drone.pos, g[0], g[1])
    #     plt.scatter(cen[0], cen[1], marker='*', color = colors[i])
    # plt.show()
    print('initializtion done')
    for i in range(20):
        goals = []
        for i, drone in enumerate(drones):
            g = drone.calc_goal_vec(drones)
            goals.append(g)
            print('goal: ', g)
            plt.scatter(drone.pos[0], drone.pos[1], color=colors[i])
            # plt.quiver(*drone.pos, g[0], g[1], color='black', width=0.01, headwidth=1, headlength=2)
            plt.quiver(*drone.pos, drone.vel[0], drone.vel[1], color='grey')

        for goal, drone in zip(goals, drones):
            drone.step(goal)
            print('taking step')
        plt.draw()
        plt.pause(0.1)
    plt.show()
