"""
  几何线系列
"""
from .GeomBase import *
from dataclasses import dataclass
from typing import List, Any, Tuple, Dict
import math


@dataclass
class Line(object):
    """
    线定义方式：点和方向
    """

    def __init__(self, P: Point3D, V: Vector3D) -> None:
        self.P = P.clone()  # 防止该值在外部被修改
        self.V = V.clone().normalized()

    def __str__(self):
        return "Line\nP %s\nV %s\n" % (str(self.P), str(self.V))


@dataclass
class Segment(object):
    """
    线段：起点和终点
    """

    def __init__(self, A: Point3D, B: Point3D):
        self.A = A.clone()
        self.B = B.clone()

    def multiply(self, m):
        """
        点右乘矩阵返回新点
        :param m:
        :return:
        """
        self.A = self.A.multiplied(m)
        self.B = self.B.multiplied(m)

    def multiplied(self, m):
        """
        线段整体进行变换，得到线段本身
        :param m:
        :return:
        """
        seg = Segment(self.A, self.B)
        seg.multiply(m)
        return seg

    def __str__(self):
        return "Segment\nA %s\nB %s\n" % (str(self.A), str(self.B))

    def length(self):
        """
        线段长度
        :return:
        """
        return self.A.distance(self.B)

    def direction(self):
        """
        线段方向
        :return:
        """
        return self.A.pointTo(self.B)

    def swap(self):
        """
        交换线段的顺序
        :return:
        """
        self.A, self.B = self.B, self.A


@dataclass
class Ray(object):
    def __init__(self, P, V):
        self.P = P.clone()
        self.V = V.clone().normalized()

    def __str__(self):
        return "Ray\nP %s\nV %s\n" % (str(self.P), str(self.V))


@dataclass
class Polyline(object):
    def __init__(self):
        self.points = []

    def __str__(self):
        """
        定义Polyline被print时的显示行为
        :return:
        """
        if self.count() > 0:
            return "Polyline\nPoint number: %s\nStart %s\nEnd %s\n" % (
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
        poly = Polyline()
        for pt in self.points:
            poly.points.append(pt.clone())
        return poly

    def count(self):
        """
        统计多段线的点数
        :return:
        """
        return len(self.points)

    def addPoint(self, pt):
        """
        在多段线的末尾添加一个点
        :param pt:
        :return:
        """
        self.points.append(pt)

    def addTuple(self, tuple):
        """
        在多段线末尾添加一个点(元组形式)
        :param tuple:
        :return:
        """
        self.points.append(Point3D(tuple[0], tuple[1], tuple[2]))

    def raddPoint(self, pt):
        """
        在多段线起点前添加一个点，r表示reverse
        :param pt:
        :return:
        """
        self.points.insert(0, pt)

    def removePoint(self, index):
        """
        根据点的序号移除一个点
        :param index:
        :return:
        """
        return self.points.pop(index)

    def point(self, index):
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
        判断多段线是否闭合
        :return:
        """
        if self.count() <= 2:
            return False
        else:
            return self.startPoint().isCoincide(self.endPoint())

    def reverse(self):
        """
        对多段线的点序进行反向操作
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
        当多段线在XY平面上投影曲线闭合时，获取多边形面积,向量为右手定则
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

    def makeCW(self):
        """
        将XY平面上的多边形调整成顺时针方向
        :return:
        """
        if self.getArea() > 0:
            self.reverse()

    def isCCW(self):
        """
        判断多边形是否为逆时针方向
        :return:
        """
        return self.getArea() > 0

    def translate(self, vec):
        """
        根据一个向量平移多线段
        :param vec:
        :return:
        """
        for i in range(len(self.points)):
            self.points[i].translate(vec)

    def appendSegment(self, seg):
        """
        在多线段的开头和结尾添加一条多段线
        四种拼接情况
        :param seg:多段线
        :return:
        """
        if self.count() == 0:
            self.points.append(seg.A)
            self.points.append(seg.B)
        else:
            if seg.A.isCoincide(self.endPoint()):  # 新增的起点与多线段末点冲突
                self.addPoint(seg.B)
            elif seg.B.isCoincide(self.endPoint()):  # 新增的末点与多线段的末点冲突
                self.addPoint(seg.A)
            elif seg.A.isCoincide(self.startPoint()):  # 新增的起点与多线段的起点冲突
                self.raddPoint(seg.B)
            elif seg.B.isCoincide(self.startPoint()):  # 新增的末点与多线段的起点冲突
                self.raddPoint(seg.A)
            else:
                return False
        return True

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

        :param m:
        :return:
        """
        poly = Polyline()
        for pt in self.points:
            poly.addPoint(pt * m)
        return poly


# 将多边形保存到文本中
def writePolyline(path, polyline: Polyline):
    """
    将多边形保存到本文件中
    :param path:
    :param polyline:
    :return:
    """
    f = None
    try:
        f = open(path, "w")  # 打开文件，w表示write，写文件
        f.write("%s\n" % polyline.count())
        for pt in polyline.points:
            txt = "%s, %s, %s\n" % (pt.x, pt.y, pt.z)
            f.write(txt)
    except Exception as ex:
        print(ex)
    finally:
        if f:
            f.close()


# 从文本中阅读多边形
def readPolyline(path):
    """
    从文本文件读取多边形
    :param path:
    :return:
    """
    f = None
    try:
        f = open(path, "r")  # 打开文件,r表示read，读文件
        poly = Polyline()
        number = int(f.readline())
        for i in range(number):
            txt = f.readline()
            txts = txt.split(",")
            x, y, z = float(txts[0]), float(txts[1]), float(txts[2])
            poly.addPoint(Point3D(x, y, z))
        return poly
    except Exception as ex:
        print(ex)
    finally:
        if f:
            f.close()
