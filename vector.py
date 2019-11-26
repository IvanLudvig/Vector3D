import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setZ(self, z):
        self.z = z

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getZ(self):
        return self.z

    def abs(self):
        return math.sqrt((self.x ** 2) + (self.y ** 2) + (self.z ** 2))

    def __add__(self, other):
        return Vector3D(self.getX() + other.getX(),
                        self.getY() + other.getY(),
                        self.getZ() + other.getZ())

    def __sub__(self, other):
        return Vector3D(self.getX() - other.getX(),
                        self.getY() - other.getY(),
                        self.getZ() - other.getZ())

    def __mul__(self, other):
        return Vector3D(self.getX() * other, self.getY() * other, self.getZ() * other)

    def __truediv__(self, other):
        return Vector3D(self.getX() / other, self.getY() / other, self.getZ() / other)

    def dot(self, other):
        return (self.getX() * other.getX()) + (self.getY() * other.getY()) + (self.getZ() * other.getZ())

    def cross(self, other):
        return Vector3D((self.getY() * other.getZ()) - (self.getZ() * other.getY()),
                        (self.getZ() * other.getX()) - (self.getX() * other.getZ()),
                        (self.getX() * other.getY()) - (self.getY() * other.getX()))

    def isCollinear(self, other):
        if (self.getX() / other.getX()) == (self.getY() / other.getY()) == (self.getZ() / other.getZ()):
            return True
        else:
            return False

    def isNull(self):
        if (self.getX() == 0) & (self.getY() == 0) & (self.getZ() == 0):
            return True
        else:
            return False


class Line:
    def __init__(self, r, a):
        self.r = r
        if not a.isNull():
            self.a = a
        else:
            print('Invalid direction vector')

    def setR(self, r):
        self.r = r

    def setA(self, a):
        self.a = a

    def getR(self):
        return self.r

    def getA(self):
        return self.a

    def getN(self):
        return Vector3D(-self.getA().y, self.getA().x)

    def dist(self, other):
        if not self.getA().isCollinear(other.getA()):
            return math.fabs((self.getR() - other.getR()).dot(self.getA().cross(other.getA()))
                             / self.getA().cross(other.getA()).abs())
        else:
            return math.fabs((self.getR() - other.getR()).cross(self.getA()).abs() / self.getA().abs())


def commonN(l1, l2):
    return l1.getA().cross(l2.getA())


v1 = Vector3D(1, 1, 3)
v2 = Vector3D(-1, 3, -2)
v3 = v1.cross(v2)
l1 = Line(Vector3D(-4, -1, 4), v1)
l2 = Line(Vector3D(5, 4, 1), v2)
n = commonN(l1, l2)
print('distance: ', l1.dist(l2))
origin = Vector3D(0, 0, 0)
vectors = np.array([v1, v2, v3, n])
lines = np.array([l1, l2])
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
for vector in vectors:
    ax.quiver(origin.getX(), origin.getY(), origin.getZ(),
              vector.getX(), vector.getY(), vector.getZ(),
              normalize=True, pivot='tail', length=vector.abs(), arrow_length_ratio=0.3 / vector.abs())
t = 5
for line in lines:
    ax.plot([line.getR().getX() - t * line.getA().getX(), line.getR().getX() + t * line.getA().getX()],
            [line.getR().getY() - t * line.getA().getY(), line.getR().getY() + t * line.getA().getY()],
            [line.getR().getZ() - t * line.getA().getZ(), line.getR().getZ() + t * line.getA().getZ()],
            color='red')

ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([0, 10])
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
