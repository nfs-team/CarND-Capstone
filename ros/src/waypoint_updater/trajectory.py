from cost_functions import *

WEIGHTED_COST_FUNCTIONS = [
    (min_velocity_cost, 10000),
    (max_velocity_cost, 10000),
    (efficiency_cost,   1),
    (total_efficiency_cost, 15),
    (max_jerk_cost,     1000),
    (total_jerk_cost,   1),
    (max_accel_cost,    1000),
    (total_accel_cost,  1),
]

class Trajectory(object):
    def __init__(self, start, end, time, max_v, full_distance):
        self.start = start
        self.end = end
        self.time = time
        self.max_v = max_v
        self.full_distance = full_distance
        self.a = self.JMT(start, end, time)
        self._cost = None

    def cost(self):
        if self._cost is None:
            self._cost = self.calculate_cost(WEIGHTED_COST_FUNCTIONS)

        return self._cost

    def position(self, t):
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        return self.a[0] + self.a[1] * t + self.a[2] * t2 + self.a[3] * t3 + self.a[4] * t4 + self.a[5] * t5

    def velocity(self, t):
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        return self.a[1] + 2. * self.a[2] * t + 3. * self.a[3] * t2 + 4. * self.a[4] * t3 + 5. * self.a[5] * t4

    def acceleration(self, t):
        t2 = t * t
        t3 = t2 * t
        return 2 * self.a[2] + 6 * self.a[3] * t + 12 * self.a[4] * t2 + 20 * self.a[5] * t3

    def jerk(self, t):
        return 6 * self.a[3] + 24 * self.a[4] * t + 60 * self.a[5] * t * t

    def JMT(self, start, end, T):
        """
        Calculates Jerk Minimizing Trajectory for start, end and T.
        """
        a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
        c_0 = a_0 + a_1 * T + a_2 * T ** 2
        c_1 = a_1 + 2 * a_2 * T
        c_2 = 2 * a_2

        A = np.array([
            [T ** 3, T ** 4, T ** 5],
            [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [6 * T, 12 * T ** 2, 20 * T ** 3],
        ])
        B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])

        a_3_4_5 = np.linalg.solve(A, B)
        alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
        return alphas

    def calculate_cost(self, cost_functions_with_weights, verbose=False):
        cost = 0
        for cf, weight in cost_functions_with_weights:
            new_cost = weight * cf(self.a, self.max_v, self.time, self)
            cost += new_cost
            if verbose:
                print "cost for {} is \t {}".format(cf.func_name, new_cost)
        return cost
