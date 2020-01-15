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

    def get_neighbor_vecs(self, drones, carrot, threshold=(0,6), debug=False):
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
        vec, dist = self.calc_dist(carrot)
        neighbor_vecs.append((vec,dist))
        return neighbor_vecs

    def calc_center_vector(self, drones, carrot, threshold=(0,6)):
        neighbor_vecs = self.get_neighbor_vecs(drones, carrot, threshold, debug=True)
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

    def calc_goal_vec(self, drones, carrot):
        avoid_weight = -20
        attract_weight = 8
        center_weight = 0.2

        avoid_vec = avoid_weight * (self.calc_center_vector(drones, carrot, (0, .5)))
        # if not np.array_equal(self.null_vector, avoid_vec):
            # print('avoid')
            # plt.quiver(*self.pos, avoid_vec[0], avoid_vec[1])
        attract_vec = attract_weight * (carrot - self.pos)
        center_vec = center_weight * (self.calc_center_vector(drones, carrot, (0, 1.5)))
        print('avoid vec: ', avoid_vec)
        print('center vec: ', center_vec)
        print('attract vec: ', attract_vec)
        return self.limit_vec(avoid_vec + center_vec + attract_vec)

    def step(self, goal_vec):
        mu = 0.1
        self.pos += self.vel
        self.vel = self.limit_vec(self.vel * (1-mu) + goal_vec * mu, 0.1)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib import animation
    import time
    v0 = np.array([0.0,0.0])
    drones = []
    for i in range(15):
        r = 5
        pos = np.random.rand(2,) * 5 - r/2
        drones.append(Drone(None, None, pos, v0))
    carrot = np.array([0.0,0.0])
    # d1 = Drone(None, None, np.array([1.0,1.0]), v0)
    # d2 = Drone(None, None, np.array([3.0,4.0]), v0)
    # drones = [d1, d2]
    fig = plt.figure()

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
            plt.quiver(*drone.pos, g[0], g[1], color='black', width=0.003, headwidth=1, headlength=1)
            # plt.quiver(*drone.pos, drone.vel[0], drone.vel[1], color='grey')
            plt.xlim([-r, r])
            plt.ylim([-r, r])
        for goal, drone in zip(goals, drones):
            drone.step(goal)
        scat.set_offsets([drone.pos for drone in drones])

    ani = animation.FuncAnimation(fig, update, frames=500, interval=10, repeat=False)
    plt.show()
