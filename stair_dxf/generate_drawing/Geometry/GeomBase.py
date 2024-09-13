"""
  点、向量、矩阵
"""

from dataclasses import dataclass
from typing import List, Any, Tuple, Dict
import math

epsilon = 1e-6  # 定义极小值，10^(-6)
epsilonSquare = epsilon * epsilon  # 极小值平方


@dataclass
class Point3D(object):
    """
    3维坐标下用齐次坐标表示点
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __str__(self):
        """
        点在print下的显示行为
        :return:
        """
        return "Point3D: %s, %s, %s" % (self.x, self.y, self.z)

    def clone(self):
        """
        复制当前点，得到新点
        :return:
        """
        return Point3D(self.x, self.y, self.z, self.w)

    def pointTo(self, other):
        """
        返回当前点与另外一个点的向量
        :param other: 另外一个点
        :return:
        """
        return Vector3D(other.x - self.x, other.y - self.y, other.z - self.z)

    def translate(self, vec):
        """
        根据向量对当前点进行平移
        :param vec:平移向量
        :return:
        """
        self.x = self.x + vec.dx
        self.y = self.y + vec.dy
        self.z = self.z + vec.dz

    def translated(self, vec):
        """
        对当前点平移后返回新点
        :param vec: 平移向量
        :return:
        """
        return Point3D(self.x + vec.dx, self.y + vec.dy, self.z + vec.dz)

    def multiplied(self, m):
        """
        点右乘一个矩阵返回新点
        :param m: 矩阵
        :return:
        """
        x = (
            self.x * m.a[0][0]
            + self.y * m.a[1][0]
            + self.z * m.a[2][0]
            + self.w * m.a[3][0]
        )
        y = (
            self.x * m.a[0][1]
            + self.y * m.a[1][1]
            + self.z * m.a[2][1]
            + self.w * m.a[3][1]
        )
        z = (
            self.x * m.a[0][2]
            + self.y * m.a[1][2]
            + self.z * m.a[2][2]
            + self.w * m.a[3][2]
        )
        return Point3D(x, y, z)

    def distance(self, other):
        """
        计算两点间的距离
        :param other: 另一个点
        :return:
        """
        return math.sqrt(self.distanceSquare(other))

    def distanceSquare(self, other):
        """
        计算两点距离的平方
        :param other: 点
        :return:
        """
        return self.pointTo(other).lengthSquare()

    def middle(self, other):
        """
        计算两个点的中点
        :param other: 点
        :return:
        """
        return Point3D(
            (self.x + other.x) / 2.0, (self.y + other.y) / 2.0, (self.z + other.z) / 2.0
        )

    def isCoincide(self, other):
        """
        根据距离判断两点是否重合
        :param other: 点
        :return:
        """
        return self.distanceSquare(other) < epsilonSquare

    def isIdentical(self, other):
        """
        判断两点是否一致
        :param other:
        :return:
        """
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __add__(self, vec):  # +
        """
        加号，对点进行平移
        :param vec:
        :return:
        """
        return self.translated(vec)

    def __sub__(self, other):  # -
        """
        减号，输入向量返回点；输入点返回向量
        :param other:
        :return:
        """
        if isinstance(other, Point3D):
            return other.pointTo(self)  # 向量
        else:
            return self.translated(other.reversed())  # 点

    def __mul__(self, m):  # *
        """
        乘号
        :param m:
        :return:
        """
        return self.multiplied(m)


@dataclass
class Vector3D(object):
    """
    向量
    """

    def __init__(self, dx=0.0, dy=0.0, dz=0.0, dw=0.0):
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.dw = dw

    def __str__(self):
        """
        向量在print时的显示行为
        :return:
        """
        return "Vector3D: %s, %s, %s" % (self.dx, self.dy, self.dz)

    def clone(self):
        """
        克隆当前向量，得到新的向量
        :return:
        """
        return Vector3D(self.dx, self.dy, self.dz, self.dw)

    def reverse(self):
        """
        对当前向量取反
        :return:
        """
        self.dx = -self.dx
        self.dy = -self.dy
        self.dz = -self.dz

    def reversed(self):
        """
        对向量取反，并返回新向量
        :return:
        """
        return Vector3D(-self.dx, -self.dy, -self.dz)

    def dotProduct(self, other):
        """
        向量点乘
        :param other:
        :return:
        """
        return self.dx * other.dx + self.dy * other.dy + self.dz * other.dz

    def crossProduct(self, other):
        """
        向量叉乘
        :param other:
        :return:
        """
        dx = self.dy * other.dz - self.dz * other.dy
        dy = self.dz * other.dx - self.dx * other.dz
        dz = self.dx * other.dy - self.dy * other.dx
        return Vector3D(dx, dy, dz)

    def amplify(self, f):
        """
        对当前向量乘以实数，放大
        :param f:
        :return:
        """
        self.dx = self.dx * f
        self.dy = self.dy * f
        self.dz = self.dz * f

    def amplified(self, f):
        """
        对当前向量放大，并返回新向量
        :param f:
        :return:
        """
        return Vector3D(self.dx * f, self.dy * f, self.dz * f)

    def length(self):
        """
        向量长度
        :return:
        """
        return math.sqrt(self.lengthSquare())

    def lengthSquare(self):
        """
        向量长度的平方
        :return:
        """
        return self.dx * self.dx + self.dy * self.dy + self.dz * self.dz

    def normalize(self):
        """
        向量标准化
        :return:
        """
        len = self.length()
        self.dx = self.dx / len
        self.dy = self.dy / len
        self.dz = self.dz / len

    def normalized(self):
        """
        向量标准化，返回新的向量
        :return:
        """
        len = self.length()
        return Vector3D(self.dx / len, self.dy / len, self.dz / len)

    def isZeroVector(self):
        """
        判断是否为零向量，即长度为0
        :return:
        """
        return self.lengthSquare() == 0

    def multiplied(self, m):
        """
        向量乘以矩阵，返回点
        :param m:
        :return:
        """
        dx = (
            self.dx * m.a[0][0]
            + self.dy * m.a[1][0]
            + self.dz * m.a[2][0]
            + self.dw
            + m.a[3][0]
        )
        dy = (
            self.dx * m.a[0][1]
            + self.dy * m.a[1][1]
            + self.dz * m.a[2][1]
            + self.dw
            + m.a[3][1]
        )
        dz = (
            self.dx * m.a[0][2]
            + self.dy * m.a[1][2]
            + self.dz * m.a[2][2]
            + self.dw
            + m.a[3][2]
        )
        return Vector3D(dx, dy, dz)

    def isParallel(self, other):
        """
        判断两个向量是否平行
        :param other:
        :return:
        """
        return self.crossProduct(other).isZeroVector()

    def getAngle(self, vec):  # 0 ~PI
        """
        计算两个向量夹角，返回值范围0~pi
        :param vec:
        :return:弧度值
        """
        v1 = self.normalized()
        v2 = vec.normalized()
        dotPro = v1.dotProduct(v2)
        if dotPro < -1.0:
            dotPro = -1.0
        if dotPro > 1.0:
            dotPro = 1.0
        return math.acos(dotPro)  # 0 ~PI

    def getAngle2D(self):  # 0 ~ 2PI, On XY Plane
        """
        在XY平面上计算当前向量和X轴夹角，返回值范围0~2PI
        :return:
        """
        rad = self.getAngle(Vector3D(1.0, 0.0, 0.0))
        if self.dy < 0:
            rad = math.pi * 2 - rad
        return rad

    def getOrthoVector2D(self):  # On XY Plane
        """
        在XY平面生成当前向量的正交向量---点积为零的向量
        :return:
        """
        if self.dx == 0:
            return Vector3D(1.0, 0.0, 0.0)
        else:
            return Vector3D(-self.dy / self.dx, 1.0, 0.0).normalized()

    def __add__(self, other):
        """
        向量相加
        :param other:
        :return:
        """
        return Vector3D(self.dx + other.dx, self.dy + other.dy, self.dz + other.dz)

    def __sub__(self, other):
        """
        向量相减
        :param other:
        :return:
        """
        return Vector3D(self.dx - other.dx, self.dy - other.dy, self.dz - other.dz)

    def __mul__(self, m):
        """
        向量与矩阵相乘
        :param m:
        :return:
        """
        return self.multiplied(m)


@dataclass
class Matrix3D(object):
    """
    三维矩阵
    """

    def __init__(self):
        self.a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def __str__(self):
        """
        矩阵在print时的显示行为
        :return:
        """
        return "Matrix3D: \n%s\n%s\n%s\n%s" % (
            self.a[0],
            self.a[1],
            self.a[2],
            self.a[3],
        )

    def makeIdentical(self):
        """
        矩阵单位化
        :return:
        """
        self.a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def mutiplied(self, other):
        """
        当前矩阵乘以另一个矩阵
        :param other:
        :return:
        """
        m = Matrix3D()
        for i in range(4):
            for j in range(4):
                m.a[i][j] = (
                    self.a[i][0] * other.a[0][j]
                    + self.a[i][1] * other.a[1][j]
                    + self.a[i][2] * other.a[2][j]
                    + self.a[i][3] * other.a[3][j]
                )
        return m

    def getDeterminant(self):
        """
        获取矩阵的行列式
        :return:
        """
        pass

    def getReverseMatrix(self):
        """
        获取当前矩阵的逆矩阵
        :return:
        """
        pass

    @staticmethod
    def createTranslateMatrix(dx, dy, dz):
        """
        静态函数，生成平移矩阵
        :param dx: x方向平移量
        :param dy: y方向平移量
        :param dz: z方向平移量
        :return:
        """
        m = Matrix3D()
        m.a[3][0] = dx
        m.a[3][1] = dy
        m.a[3][2] = dz
        return m

    @staticmethod
    def createScaleMatrix(sx, sy, sz):
        """
        静态函数---静态方法中无法使用实例属性和方法，即不能用self.，生成缩放矩阵
        :param sx:
        :param sy:
        :param sz:
        :return:
        """
        m = Matrix3D()
        m.a[0][0] = sx
        m.a[1][1] = sy
        m.a[2][2] = sz
        return m

    @staticmethod
    def createRotateMatrix(axis, angle):
        """
        静态函数，旋转矩阵
        :param axis: 旋转轴
        :param angle: 旋转角度
        :return:
        """
        m = Matrix3D()
        sin = math.sin(angle)
        cos = math.cos(angle)
        if axis == "X" or axis == "x":
            m.a[1][1], m.a[1][2], m.a[2][1], m.a[2][2] = cos, sin, -sin, cos
        elif axis == "Y" or axis == "y":
            m.a[0][0], m.a[0][2], m.a[2][0], m.a[2][2] = cos, -sin, sin, cos
        elif axis == "Z" or axis == "z":
            m.a[0][0], m.a[0][1], m.a[1][0], m.a[1][1] = cos, sin, -sin, cos
        return m

    @staticmethod
    def createMirrorMatrix(normal):
        """
        静态函数，生成镜像矩阵
        镜像变换，根据反射成像的概念，它是缩放变换的一个特例，当缩放银子k<0时会导致镜像变换。
        :param normal: 镜像轴
        :return:
        """
        m = Matrix3D()  # 初始镜像矩阵
        scale_coeff = [1, 1, 1]  # 缩放因子
        if normal == "X" or normal == "x":
            scale_coeff[1] = -1
            scale_coeff[2] = -1
        elif normal == "Y" or normal == "y":
            scale_coeff[0] = -1
            scale_coeff[2] = -1
        elif normal == "Z" or normal == "z":
            scale_coeff[0] = -1
            scale_coeff[1] = -1
        elif normal == "XY" or normal == "xy":
            scale_coeff[2] = -1
        elif normal == "XZ" or normal == "xz":
            scale_coeff[1] = -1
        elif normal == "YZ" or normal == "yz":
            scale_coeff[0] = -1
        elif normal == "XYZ" or normal == "xyz":
            scale_coeff[0] = -1
            scale_coeff[1] = -1
            scale_coeff[2] = -1
        m.a[0][0], m.a[1][1], m.a[2][2] = scale_coeff[0], scale_coeff[1], scale_coeff[2]
        return m

    def __mul__(self, other):
        """
        矩阵相乘，调用multiplied函数
        :param other: 4*4矩阵
        :return:
        """
        return self.mutiplied(other)

    def __add__(self, other):
        """
        矩阵相加
        :param other: 矩阵
        :return:
        """
        m = Matrix3D()
        for i in range(4):
            for j in range(4):
                m.a[i][j] = self.a[i][j] + other.a[i][j]
        return m

    def __sub__(self, other):
        """
        矩阵相减
        :param other:矩阵
        :return:
        """
        m = Matrix3D()
        for i in range(4):
            for j in range(4):
                m.a[i][j] = self.a[i][j] - other.a[i][j]
        return m
