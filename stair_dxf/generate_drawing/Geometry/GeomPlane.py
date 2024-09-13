"""
  描述有限平面和无限平面

"""

from .GeomBase import Point3D, Vector3D
from .GeomLines import Line, Segment
from dataclasses import dataclass
import logging


@dataclass
class InfinitePlane(object):
    """
    平面的表示方法:点和法线
    """

    def __init__(self, P: Point3D, N: Vector3D):
        self.P = P.clone()  # 平面上的点
        self.N = N.clone().normalized()  # 平面法向量，单位向量

    def __str__(self):
        """
        平面显示的print方法
        :return:
        """
        return "Plane\n%s\n%s\n" % (str(self.P), str(self.N))

    def toFormula(self):
        """
        获取标准直线方程的系数
        :return:
        """
        A, B, C = self.N.dx, self.N.dy, self.N.dz
        D = -self.N.dx * self.P.x - self.N.dy * self.P.y - self.N.dz * self.P.z
        return A, B, C, D

    @staticmethod
    def xPlane(x: float):
        """
        返回x平面
        :param x:
        :return:
        """
        return InfinitePlane(Point3D(x, 0, 0), Vector3D(1, 0, 0))

    @staticmethod
    def yPlane(y: float):
        """
        返回y平面
        :param y:
        :return:
        """
        return InfinitePlane(Point3D(0, y, 0), Vector3D(0, 1.0, 0))

    @staticmethod
    def zPlane(z: float):
        """
        返回z平面
        :param z:
        :return:
        """
        return InfinitePlane(Point3D(0, 0, z), Vector3D(0, 0, 1.0))

    def intersect(self, other):
        """
        两平面求交，返回Line或None
        :param other:
        :return:
        """
        dir = self.N.crossProduct(other.N)  # dir为交线的方向向量
        if dir.isZeroVector():
            return None
        else:
            x, y, z = 0, 0, 0  # x,y,z为直线经过点的坐标
            A1, B1, C1, D1 = self.toFormula()  # 通过平面标准方程联立求解
            A2, B2, C2, D2 = other.toFormula()
            if B2 * C1 - B1 * C2 != 0:
                y = -(C2 * D1 + C1 * D2) / (B2 * C1 - B1 * C2)
                z = -(B2 * D1 - B1 * D2) / (B2 * C1 - B1 * C2)
            elif A2 * C1 - A1 * C2 != 0:
                x = -(-C2 * D1 + C1 * D2) / (A2 * C1 - A1 * C2)
                z = -(A2 * D1 - A1 * D2) / (A2 * C1 - A1 * C2)
            elif A2 * B1 - A1 * B2 != 0:
                x = -(-B2 * D1 + B1 * D2) / (A2 * B1 - A1 * B2)
                y = -(A2 * D1 - A1 * D2) / (A2 * B1 - A1 * B2)
            else:
                return None
            return Line(Point3D(x, y, z), dir.normalized())


@dataclass
class FinitePlane(object):
    """
    有限平面：首尾相连的多点组成
    """

    def __init__(self):
        self.points = []
        self.segments = []
        self.N = []

    def __str__(self):
        """
        定义Polyline被print时的显示行为
        :return:
        """
        if self.count() > 0:
            return "FinitePlane\nPoint number: %s\nStart %s\nEnd %s\n" % (
                self.count(),
                str(self.startPoint()),
                str(self.endPoint()),
            )
        else:
            return "Polyline\nPoint number: 0\n"
        pass

    def clone(self):
        """
        克隆一条线
        :return:
        """
        fP = FinitePlane()
        for pt in self.points:
            fP.points.append(pt.clone())
        return fP

    def count(self):
        """
        统计多段线的点数
        :return:
        """
        return len(self.points)

    def addPoint(self, pt: Point3D):
        """
        在多段线的末尾添加一个点
        :param pt:
        :return:
        """
        self.points.append(pt)

    def formNormal(self):
        """
        形成有限平面的法向量
        :return:
        """
        if self.count() > 3:
            p1 = self.points[0]
            p2 = self.points[1]
            p3 = self.points[2]
            v1 = p1.pointTo(p2)
            v2 = p2.pointTo(p3)
            N = v1.crossProduct(v2)  # 两向量叉乘
        else:
            return
        return N

    def raddPoint(self, pt: Point3D):
        """
        在多段线起点前添加一个点，r表示reverse
        :param pt:
        :return:
        """
        self.points.insert(0, pt)

    def removePoint(self, index: int):
        """
        根据点的序号移除一个点
        :param index:
        :return:
        """
        return self.points.pop(index)

    def point(self, index: int):
        """
        根据点的序号获取一个点
        :param index:
        :return:
        """
        return self.points[index]

    def startPoint(self):
        """
        获取起点
        :return:
        """
        return self.points[0]

    def endPoint(self):
        """
        获取终点
        :return:
        """
        return self.points[-1]

    def isClosed(self):
        """
        判断多边形平面是否闭合（起点和终点同）
        :return:
        """
        if self.count() <= 2:
            return False
        else:
            return self.startPoint().isCoincide(self.endPoint())

    def reverse(self):
        """
        对多封闭多边形的点序进行反向操作
        :return:
        """
        sz = self.count()
        for i in range(int(sz / 2)):
            self.points[i], self.points[sz - 1 - i] = (
                self.points[sz - 1 - i],
                self.points[i],
            )

    def getArea(self):
        """
        当封闭多边形平面上投影曲线闭合时，获取多边形面积,向量为右手定则
        :return:
        """
        area = 0.0
        for i in range(self.count() - 1):
            area += 0.5 * (
                self.points[i].x * self.points[i + 1].y
                - self.points[i + 1].x * self.points[i].y
            )
        return area

    def makeCCW(self):
        """
        将XY平面上的多边形调整成逆时针方向
        :return:
        """
        if self.getArea() < 0:
            self.reverse()

    def isCCW(self):
        """
        判断多边形是否为逆时针方向
        :return:
        """
        return self.getArea() > 0

    def clearSegemnt(self):
        """
        清空列表中所有线段
        :return:
        """
        return self.segments.clear()

    def formSegment(self):
        """
        形成线段
        :return:
        """
        try:
            for i in range(len(self.points) - 1):
                A1: Point3D = self.points[i]
                B1: Point3D = self.points[i + 1]
                seg = Segment(A=A1, B=B1)
                self.segments.append(seg)  # 添加线段
        except IndexError as e:
            print("点数量不足，无法形成线段！")
            return

    def multiply(self, m):
        """
        相乘,???有问题
        :param m:
        :return:
        """
        for pt in self.points:
            pt.multiply(m)

    def multiplied(self, m):
        """
        平面点相乘
        :param m:
        :return:
        """
        fP = FinitePlane()
        for pt in self.points:
            fP.addPoint(pt * m)
        return fP

    def toFormula(self):
        """
        获取标准直线方程的系数
        :return:
        """
        if self.count() > 0:
            P = self.points[0]
            N = self.formNormal()
            A, B, C = N.dx, N.dy, N.dz
            D = -N.dx * P.x - N.dy * P.y - N.dz * P.z
            return A, B, C, D
        else:
            return

    @staticmethod
    def xPlane(x: float):
        """
        返回x平面
        :param x:
        :return:
        """
        return InfinitePlane(Point3D(x, 0, 0), Vector3D(1, 0, 0))

    @staticmethod
    def yPlane(y: float):
        """
        返回y平面
        :param y:
        :return:
        """
        return InfinitePlane(Point3D(0, y, 0), Vector3D(0, 1.0, 0))

    @staticmethod
    def zPlane(z: float):
        """
        返回z平面
        :param z:
        :return:
        """
        return InfinitePlane(Point3D(0, 0, z), Vector3D(0, 0, 1.0))

    def intersect(self, other):
        """
        两平面求交，返回True或False，对于有限平面，存在问题
        :param other:
        :return:
        """
        other_segs = other.formSegment()  # 形成多线段
        self.segments = self.formSegment()  # 形成多线段
        if other_segs is not None and self.segments is not None:
            for seg_one in self.segments:
                for seg_two in self.segments:
                    A1 = seg_one.A
                    B1 = seg_one.B
                    A2 = seg_two.A
                    B2 = seg_two.B
                    if (A1.isIdentical(A2) or A1.isIdentical(B2)) and (
                        B1.isIdentical(A2) or B1.isIdentical(B2)
                    ):  # 两多边形平面右共线段
                        return True
        return False
