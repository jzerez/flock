import numpy as np
import pdb
import matplotlib.pyplot as plt
def unit(vec):
    mag = np.linalg.norm(vec)
    return vec / mag

def calc_angle(vec1, vec2):
    theta = np.arccos(np.dot(unit(vec1),unit(vec2)))
    return np.degrees(theta)

class Boid:
    def __init__(self, pos, vel):
        self.pos = pos
        self.vel = vel
        self.vec_mag_limit = 0.1
        self.null_vector = np.array([0.0,0.0])
        self.c0 = np.copy(self.null_vector)
        self.c1 = None

    def in_threshold(self, target, threshold):
        return target < threshold[1] and target > threshold[0]

    def get_neighbors(self, objects, thresh_dist=(0,6), thresh_angle = 180, debug=True):
        """
        return the neighbors within a certain distance of the drone
        """
        neighbors = []
        for object in objects:
            if object is not self:
                vec, dist = self.calc_dist(object.pos)
                angle = calc_angle(self.vel, vec)
                if dist < 0.05 and debug: print('CRASH')
                if self.in_threshold(dist, thresh_dist):
                    if thresh_angle > angle:
                        neighbors.append(object)

        return neighbors

    def calc_center_vector(self, neighbors, type):
        # type argument is either 'pos' or 'vel'
        if not neighbors:
            return self.null_vector
        center = sum([getattr(drone, type) for drone in neighbors])/len(neighbors)
        center_vector = center - getattr(self, type)
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

    def center(self, drones, thresh_dist=(0,1), thresh_angle=35):
        ns = self.get_neighbors(drones, thresh_dist=thresh_dist, thresh_angle=thresh_angle)
        vec = self.calc_center_vector(ns, 'pos')
        return self.limit_vec(vec)

    def avoid(self, drones, carrot, thresh_dist=(0,0.4), thresh_angle=180):
        objects = drones + [carrot]
        ns = self.get_neighbors(drones, thresh_dist=thresh_dist, thresh_angle=thresh_angle)
        vec = self.calc_center_vector(ns, 'pos')
        return self.limit_vec(vec)

    def attract(self, carrot, thresh_dist=(0,8)):
        return self.limit_vec(carrot.pos - self.pos)

    def align(self, drones, thresh_dist=(0,0.7), thresh_angle=35):
        ns = self.get_neighbors(drones, thresh_dist=thresh_dist, thresh_angle=thresh_angle)
        vec = self.calc_center_vector(ns, 'vel')+self.vel
        return self.limit_vec(vec)

    def calc_goal_vec(self, drones, carrot):
        avoid_weight = -3.5
        attract_weight = 1
        center_weight = 2
        align_weight = 4

        goal_vec = (avoid_weight * self.avoid(drones, carrot) +
                    attract_weight * self.attract(carrot) +
                    center_weight * self.center(drones)+
                    align_weight * self.align(drones))
        return unit(goal_vec)

    def step(self, goal_vec, wrap=False):
        mu = 0.3
        self.pos += self.vel
        if wrap:
            for i, coor in enumerate(self.pos):
                if coor < -2.5:
                    # pdb.set_trace()
                    self.pos[i] = coor % 2.5
                elif coor > 2.5:
                    self.pos[i] = -2.5 + (coor % 2.5)
        self.vel = self.limit_vec(unit(self.vel) * (1-mu) + goal_vec * mu, 0.1)

class Carrot:
    def __init__(self, pos):
        self.pos = pos


if __name__ == "__main__":
    carrot = Carrot(np.array([0.0, 0.0]))
    drones = []
    for i in range(15):
        r = 5
        pos = np.random.rand(2,) * 5 - r/2
        vel = unit(np.random.rand(2,)) * 0.1
        drones.append(Boid(pos, vel))
    for i in range(100):
        plt.figure()
        plt.axis('equal')
        plt.grid('on')
        p1 = drones[0].pos[0]
        p2 = drones[0].pos[1]
        goals = []
        goal_vecs = []
        for drone in drones:
            # pdb.set_trace()
            goal = drone.calc_goal_vec(drones, carrot)
            goal_vecs.append(goal)
            total_goal = drone.limit_vec(sum(goal), 0.1)
            goals.append(total_goal)
            plt.scatter(drone.pos[0], drone.pos[1], color='grey')
        plt.scatter(p1, p2, s=30, color='black')
        plt.quiver(*drones[0].pos, drones[0].vel[0], drones[0].vel[1])
        # ns = drones[0].get_neighbor_vecs(drones, carrot)
        # print(ns)
        # vs = [vec[0] for vec in ns]
        # for i, v in enumerate(vs):
        #     plt.scatter(v[0] + p1, v[1] + p2, color='red')
        #     print(ns[i][2])
        for i, goal in enumerate(goals):

            # plt.quiver(*drones[i].pos, goal_vecs[i][0][0],  goal_vecs[i][0][1], color='red')   #avoid_vec,
            # plt.quiver(*drones[i].pos, goal_vecs[i][1][0],  goal_vecs[i][1][1], color='blue')    #attract_vec,
            # plt.quiver(*drones[i].pos, goal_vecs[i][2][0],  goal_vecs[i][2][1], color='green')  #center_vec,
            # plt.quiver(*drones[i].pos, goal_vecs[i][3][0],  goal_vecs[i][3][1], color='yellow')  #align_vec
            # plt.quiver(*drones[i].pos, goals[i][0],  goals[i][1], color='black')  #align_vec
            plt.draw()
            plt.pause(0.2)
            drones[i].step(goal)
        # pdb.set_trace()
        plt.show()
