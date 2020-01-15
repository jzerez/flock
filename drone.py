import numpy as np
import pdb
import matplotlib.pyplot as plt
def unit(vec):
    mag = np.linalg.norm(vec)
    return vec / mag

def calc_angle(vec1, vec2):
    theta = np.arccos(np.dot(vec1,vec2) / np.linalg.norm(vec1) / np.linalg.norm(vec2))
    return np.degrees(theta)

class Drone:
    def __init__(self, pos, vel):
        self.pos = pos
        self.vel = vel
        self.vec_mag_limit = 1
        self.null_vector = np.array([0,0])

    def in_threshold(self, target, threshold):
        return target < threshold[1] and target > threshold[0]

    def get_neighbor_vecs(self, drones, carrot, thresh_dist=(0,6), thresh_angle = 15, debug=False):
        """
        return the neighbors within a certain distance of the drone
        """
        neighbor_vecs = []
        for drone in drones:
            if drone is not self:
                vec, dist = self.calc_dist(drone.pos)
                angle = calc_angle(self.vel, vec)
                if dist < 0.5 and debug: print('CRASH')
                if self.in_threshold(dist, thresh_dist) and thresh_angle > angle:
                    neighbor_vecs.append((vec, dist, angle))
        vec, dist = self.calc_dist(carrot)
        neighbor_vecs.append((vec,dist, -1))
        return neighbor_vecs

    def calc_center_vector(self, drones, carrot, threshold=(0,6), thresh_angle=15):
        neighbor_vecs = self.get_neighbor_vecs(drones, carrot, threshold, thresh_angle, debug=True)
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



    def calc_goal_vec(self, drones, carrot):
        avoid_weight = -10
        attract_weight = 3
        center_weight = 5

        avoid_vec = avoid_weight * (self.calc_center_vector(drones, carrot, (0, 1), 120))
        attract_vec = attract_weight * (carrot - self.pos)
        center_vec = center_weight * (self.calc_center_vector(drones, carrot, (0, 2)))
        print('avoid vec: ', avoid_vec)
        print('center vec: ', center_vec)
        print('attract vec: ', attract_vec)
        return self.limit_vec(avoid_vec + center_vec + attract_vec)

    def step(self, goal_vec):
        mu = 0.02
        self.pos += self.vel
        self.vel = self.limit_vec(self.vel * (1-mu) + goal_vec * mu, 0.1)


if __name__ == "__main__":
    carrot = np.array([0.0, 0.0])
    drones = []
    for i in range(15):
        r = 5
        pos = np.random.rand(2,) * 5 - r/2
        vel = unit(np.random.rand(2,)) * 0.1
        drones.append(Drone(pos, vel))
    plt.figure()
    plt.axis('equal')
    p1 = drones[0].pos[0]
    p2 = drones[0].pos[1]
    for drone in drones:
        plt.scatter(drone.pos[0], drone.pos[1], color='grey')
    plt.scatter(p1, p2, s=30, color='black')
    plt.quiver(*drones[0].pos, drones[0].vel[0], drones[0].vel[1])
    ns = drones[0].get_neighbor_vecs(drones, carrot)
    print(ns)
    vs = [vec[0] for vec in ns]
    for i, v in enumerate(vs):
        plt.scatter(v[0] + p1, v[1] + p2, color='red')
        print(ns[i][2])
    # pdb.trace()
    plt.show()
