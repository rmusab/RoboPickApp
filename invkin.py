__author__ = 'ravil'
import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from threading import Thread

class InvKin(object):

    JOINTS_COUNT = 3
    LINKS = [100.0, 145.0, 123.0]
    BASE_HEIGHT = 65.0
    PLATE_HEIGHT = 20.0
    ALPHA = 1.0  # Learning rate

    SINGULAR_MATRIX_ERR = 1
    DIVERGENT_ERR = 2
    OUT_OF_REACH_ERR = 3
    ERR_LIMIT = 10

    def __init__(self, (X, Y), (arm_X, arm_Y)):
        self.world = []
        self.arm = []
        self.goal = []

        self.world.append(X)
        self.world.append(Y)

        self.arm.append(arm_X)
        self.arm.append(arm_Y)

        self.errors = 0

    # Returns base rotation angle as well as path length
    def base_rotation(self, plate):
        result = {}

        dy = self.world[1] - self.arm[1]
        dx = self.world[0] - self.arm[0]
        length = np.sqrt(dx ** 2 + dy ** 2)
        alpha = np.arccos(dx / length) * 180.0 / np.pi
        result['alpha'] = alpha
        result['length'] = length

        if dy < 0 or length > sum(self.LINKS):
            result['error'] = self.OUT_OF_REACH_ERR
            print "Object is out of reach"
        else:
            self.goal.append(length)
            if plate is True:
                self.goal.append(-self.BASE_HEIGHT + self.PLATE_HEIGHT)
            else:
                self.goal.append(-self.BASE_HEIGHT)
            result['error'] = None

        return result

    # Sin function with argument in degrees
    @staticmethod
    def sin(alpha):
        return np.sin(np.radians(alpha))

    # Cos function with argument in degrees
    @staticmethod
    def cos(alpha):
        return np.cos(np.radians(alpha))

    @staticmethod
    def sign(x):
        if x > 0:
            return 1
        if x < 0:
            return -1
        else:
            return 0

    # Forward kinematics equation
    def f(self, theta):
        e = np.array([], np.float32)

        link = self.LINKS
        e = np.append(e, link[0] * self.sin(theta[0]) +
                      link[1] * self.sin(theta[0] + theta[1]) +
                      link[2] * self.sin(theta[0] + theta[1] + theta[2]))
        e = np.append(e, link[0] * self.cos(theta[0]) +
                      link[1] * self.cos(theta[0] + theta[1]) +
                      link[2] * self.cos(theta[0] + theta[1] + theta[2]))

        return e

    # Jacobian matrix for specific theta vector
    def J(self, theta):
        jacmtx = np.zeros((2, self.JOINTS_COUNT), np.float32)

        link = self.LINKS
        jacmtx[0, 2] = link[2] * self.cos(theta[0] + theta[1] + theta[2])
        jacmtx[0, 1] = jacmtx[0, 2] + link[1] * self.cos(theta[0] + theta[1])
        jacmtx[0, 0] = jacmtx[0, 1] + link[0] * self.cos(theta[0])

        jacmtx[1, 2] = -link[2] * self.sin(theta[0] + theta[1] + theta[2])
        jacmtx[1, 1] = jacmtx[1, 2] - link[1] * self.sin(theta[0] + theta[1])
        jacmtx[1, 0] = jacmtx[1, 1] - link[0] * self.sin(theta[0])

        return jacmtx

    # Pseudo inverse of Jacobian matrix
    @staticmethod
    def pseudo(jacmtx):
        return np.dot(inv(np.dot(jacmtx.T, jacmtx)), jacmtx.T)

    # de - a small step vector in the direction from the end-effector to the goal
    def de(self, theta):
        e = self.f(theta)
        de = np.array(self.goal) - np.array(e)

        return de

    # Shows how far end-effector is from the goal position
    def shift(self, theta):
        e = self.f(theta)
        shift = np.array(self.goal) - np.array(e)
        shift_norm = norm(shift)

        return shift_norm

    # Properly main iterative algorithm
    def solve_jacobian(self, theta):
        result = {}
        while self.shift(theta) > 1.0:
            try:
                inc = self.ALPHA * np.dot(self.pseudo(self.J(theta)), self.de(theta))
                if np.count_nonzero(inc) == 0:
                    result['theta'] = None
                    result['error'] = self.DIVERGENT_ERR
                    return result
                theta = np.array(theta) + inc
            except np.linalg.linalg.LinAlgError as err:
                if 'Singular matrix' in err.message:
                    print "Singular matrix"
                    result['theta'] = None
                    result['error'] = self.SINGULAR_MATRIX_ERR
                    return result
                else:
                    raise

        result['theta'] = theta
        result['error'] = None
        return result

    # Solves IK task for 1000 samples in order to get an appropriate one
    def solve_1000_jacobians(self):
        counter = 0
        self.errors = 0
        for j1 in np.arange(10) * 10:
            for j2 in np.arange(10) * 10:
                for j3 in np.arange(10) * 10:
                    counter += 1

                    theta = np.float32([j1, j2, j3])
                    print counter
                    print theta

                    res = self.solve_jacobian(theta)

                    if res['error'] is None:
                        theta = self.adjust_solution(res['theta'])
                        if self.in_reach(theta):
                            return theta
                    elif res['error'] == self.SINGULAR_MATRIX_ERR:
                        continue
                    elif res['error'] == self.DIVERGENT_ERR:
                        if self.errors > self.ERR_LIMIT:
                            print "Divergent iterative process: object is out of reach"
                            return None
                        else:
                            self.errors += 1
                            continue

        return None

    # Adjust obtained solution to be in range (-180, 180), that is in range of joint constraints
    @staticmethod
    def adjust_solution(theta):
        for i in range(3):
            theta[i] -= 360.0 * int(theta[i] / 360.0)
            if not (180 >= theta[i] >= -180):
                if theta[i] > 0:
                    theta[i] -= 360
                else:
                    theta[i] += 360

        return theta

    @staticmethod
    def in_reach(theta):
        result = True
        for i in range(3):
            if theta[i] > 90 or theta[i] < 0:
                result = False
                break

        return result

class ComputingThread(Thread):

    def __init__(self, (X, Y), (arm_X, arm_Y), plate):
        super(ComputingThread, self).__init__()
        self.X = X
        self.Y = Y

        self.arm_X = arm_X
        self.arm_Y = arm_Y

        self.alpha = None
        self.length = None
        self.theta = None
        self.end_eff = None
        self.goal = None
        self.plate = plate

    def run(self):
        invkin = InvKin((self.X, self.Y), (self.arm_X, self.arm_Y))
        result = invkin.base_rotation(self.plate)
        self.alpha = result['alpha']
        self.length = result['length']
        if result['error'] is None:
            self.theta = invkin.solve_1000_jacobians()
            if self.theta is not None:
                self.end_eff = invkin.f(self.theta)
                self.goal = invkin.goal
