"""
Date:2022/9/19
Author:super zhang
Aim:
   几何算法(Geometry algorithm)，距离和求交计算
   如点线距离、点面距离、线线距离、线面距离、点点距离
"""

from .GeomBase import *
import math
from .GeomLines import *
from .GeomPlane import *


def nearZero(x):
    """
    判断该数是否接近0
    :param x:
    :return:
    """
    return math.fabs(x) < epsilon  # 判断该数和给定极小数的关系


def distance(obj1, obj2):
    """
    实体距离计算
    :param obj1:
    :param obj2:
    :return:
    """
    if isinstance(obj1, Point3D) and isinstance(obj2, Line):  # Point --- Line
        # 点到直线的距离
        P, Q, V = obj2.P, obj1, obj2.V
        t = P.pointTo(Q).dotProduct(V)
        R = P + V.amplified(t)
        d = Q.distance(R)
        return d
    elif isinstance(obj1, Point3D) and isinstance(obj2, Ray):  # Point --- Ray
        # 点到射线的距离
        P, Q, V = obj2.P, obj1, obj2.V
        t = P.pointTo(Q).dotProduct(V)
        if t >= 0:  # 点投影在射线上
            R = P + V.amplified(t)
            return Q.distance(R)
        else:
            return Q.distance(P)  # 点投影在射线外
    elif isinstance(obj1, Point3D) and isinstance(obj2, Segment):  # Point --- Segment线段
        # 点到线段的距离
        Q, P, P1, V = obj1, obj2.A, obj2.B, obj2.direction().normalized()
        L = obj2.length()
        t = P.pointTo(Q).dotProduct(V)
        if t <= 0:
            return Q.distance(P)
        elif t >= L:
            return Q.distance(P1)
        else:
            R = P + V.amplified(t)
            return Q.distance(R)
    elif isinstance(obj1, Point3D) and isinstance(obj2, InfinitePlane):  # Point---Plane
        # 点到平面的距离
        P, Q, N = obj2.P, obj1, obj2.N
        angle = N.getAngle(P.pointTo(Q))
        return P.distance(Q) * math.cos(angle)
    elif isinstance(obj1, Line) and isinstance(obj2, Line):  # Line--Line
        # 线到线的距离
        P1, V1, P2, V2 = obj1.P, obj1.V, obj2.P, obj2.V
        N = V1.crossProduct(V2)
        if N.isZeroVector():
            return distance(P1, obj2)
        else:
            return distance(P1, InfinitePlane(P2, N))
    elif isinstance(obj1, Line) and isinstance(obj2, InfinitePlane):  # Line--Plane
        # 线到平面的距离
        if obj1.V.dotProduct(obj2.N) == 0:
            return distance(obj1.P, obj2)
        else:
            return 0
    elif isinstance(obj1, Ray) and isinstance(obj2, InfinitePlane):  # Ray --- Plane
        # 射线到平面的距离
        P1, V = obj1.P, obj1.V
        P2, N = obj2.P, obj2.N
        P3 = P1 + V.amplified(2)
        angle_1 = N.getAngle(P2.pointTo(P1))
        angle_2 = N.getAngle(P2.pointTo(P3))
        distance_1 = P2.distance(P1) * math.cos(angle_1)
        distance_2 = P2.distance(P3) * math.cos(angle_2)
        if V.dotProduct(N) != 0:  # 射线与平面不平行,看射线走势
            if distance_1 < distance_2:  # TODO
                return distance_1
            else:
                return 0
        else:  # 射线与直线平行，射线上点到直线的距离
            return distance_1

    elif isinstance(obj1, Segment) and isinstance(
        obj2, InfinitePlane
    ):  # Segment --- Plane
        # 线段到平面的距离,判断线段的两端点在平面同侧还是异侧
        P1, P2 = obj1.A, obj1.B
        Q, N = obj2.P, obj2.N
        V1 = Q.pointTo(P1)
        V2 = Q.pointTo(P2)
        f1 = V1.dotProduct(N)
        f2 = V2.dotProduct(N)
        if f1 * f2 <= 0:
            return 0
        else:
            angle_1 = N.getAngle(V1)
            angle_2 = N.getAngle(V2)
            distance_1 = Q.distance(P1) * math.cos(angle_1)
            distance_2 = Q.distance(P2) * math.cos(angle_2)
            return min(distance_1, distance_2)


def intersectLineLine(line1: Line, line2: Line):
    """
    两平面直线求交点
    :param line1:
    :param line2:
    :return:
    """
    P1, V1, P2, V2 = line1.P, line1.V, line2.P, line2.V
    P1P2 = P1.pointTo(P2)  # 向量
    deno = V1.dy * V2.dx - V1.dx * V2.dy  # x、y前面的系数成比例
    if deno != 0:
        t1 = -(-P1P2.dy * V2.dx + P1P2.dx * V2.dy) / deno
        t2 = -(-P1P2.dy * V1.dx + P1P2.dx * V1.dy) / deno
        return P1 + V1.amplified(t1), t1, t2  # 返回交点和偏移值
    else:
        deno = V1.dz * V2.dy - V1.dy * V2.dz  # y、z前面的系数成比例
        if deno != 0:
            t1 = -(-P1P2.dz * V2.dy + P1P2.dy * V2.dz) / deno
            t2 = -(-P1P2.dz * V1.dy + P1P2.dy * V1.dz) / deno
            return P1 + V1.amplified(t1), t1, t2
        else:
            deno = V1.dx * V2.dz - V1.dz * V2.dx  # x、z前面的系数成比例
            if deno != 0:
                t1 = -(-P1P2.dx * V2.dz + P1P2.dz * V2.dx) / deno
                t2 = -(-P1P2.dx * V1.dz + P1P2.dz * V1.dx) / deno
                return P1 + V1.amplified(t1), t1, t2
            else:
                return None, 0, 0


def intersectSegmentPlane(seg: Segment, plane: InfinitePlane):
    """
    线段和平面形成的交点
    :param seg:
    :param plane:
    :return:
    """
    A, B, P, N = seg.A, seg.B, plane.P, plane.N
    V = A.pointTo(B)
    PA = P.pointTo(A)
    if V.dotProduct(N) == 0:
        return None
    else:
        t = -(PA.dotProduct(N)) / V.dotProduct(N)
        if t >= 0 and t <= 1:
            return A + (V.amplified(t))
        else:
            return None


def intersect(obj1, obj2):
    """
    求交操作
    :param obj1:
    :param obj2:
    :return:
    """
    if isinstance(obj1, Line) and isinstance(obj2, Line):
        # 两条直线求交操作
        P, t1, t2 = intersectLineLine(obj1, obj2)
        return P
    elif isinstance(obj1, Segment) and isinstance(obj2, Segment):
        # 两线段求交操作
        line1 = Line(obj1.A, obj1.direction())
        line2 = Line(obj2.A, obj2.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if t1 >= 0 and t1 <= obj1.length() and t2 >= 0 and t2 <= obj2.length():
                return P
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, Segment):
        # 直线和线段求交操作
        line1 = obj1
        line2 = Line(obj2.A, obj2.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None and t2 >= 0 and t2 <= obj2.length():
            return P
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, Ray):
        # 直线与射线求交操作
        P1, V1 = obj1.P, obj1.V
        Q, V2 = obj2.P, obj2.V
        V21 = P1.pointTo(Q)
        if (
            V1.dx * V2.dy - V1.dy * V2.dx != 0 and V1.dx * V2.dz - V1.dz * V2.dx != 0
        ):  # 判断是否平行
            t1 = (V21.dx * V2.dy - V21.dy * V2.dx) / (V1.dx * V2.dy - V1.dy * V2.dx)
            t2 = (V1.dy * V21.dx - V1.dx * V21.dy) / (V1.dx * V2.dy - V2.dx * V1.dy)
            if t1 is not None and t2 is not None:
                if t2 >= 0:
                    P = Point3D()
                    P.x = P1.x + V1.amplified(t1).dx
                    P.y = P1.y + V1.amplified(t1).dy
                    P.z = P1.z + V1.amplified(t1).dz
                    return P

        return None
    elif isinstance(obj1, Ray) and isinstance(obj2, Segment):
        # 射线与线段求交操作
        P1, V1 = obj1.P, obj1.V
        Q1, Q2, V2, length_2 = obj2.A, obj2.B, obj2.direction(), obj2.length()
        V21 = P1.pointTo(Q1)
        if (
            V1.dx * V2.dy - V1.dy * V2.dx != 0 and V1.dx * V2.dz - V1.dz * V2.dx != 0
        ):  # 判断是否平行
            t1 = (V21.dx * V2.dy - V21.dy * V2.dx) / (V1.dx * V2.dy - V1.dy * V2.dx)
            t2 = (V1.dy * V21.dx - V1.dx * V21.dy) / (V1.dx * V2.dy - V2.dx * V1.dy)
            if t1 is not None and t2 is not None:  # 判断是否有解
                if t1 >= 0 and (t2 >= 0 and t2 <= length_2):  # 判断解是否满足要求
                    P = Point3D()
                    P.x = P1.x + V1.amplified(t1).dx
                    P.y = P1.y + V1.amplified(t1).dy
                    P.z = P1.z + V1.amplified(t1).dz
                    return P
        return None
    elif isinstance(obj1, Ray) and isinstance(obj2, Ray):
        # 射线与射线求交操作
        P1, V1 = obj1.P, obj1.V
        P2, V2 = obj2.P, obj2.V
        V21 = P1.pointTo(P2)
        if (
            V1.dx * V2.dy - V1.dy * V2.dx != 0 and V1.dx * V2.dz - V1.dz * V2.dx != 0
        ):  # 判断是否平行
            t1 = (V21.dx * V2.dy - V21.dy * V2.dx) / (V1.dx * V2.dy - V1.dy * V2.dx)
            t2 = (V1.dy * V21.dx - V1.dx * V21.dy) / (V1.dx * V2.dy - V2.dx * V1.dy)
            if t1 is not None and t2 is not None:  # 判断是否有解
                if t1 >= 0 and t2 >= 0:  # 判断解是否满足要求
                    P = Point3D()
                    P.x = P1.x + V1.amplified(t1).dx
                    P.y = P1.y + V1.amplified(t1).dy
                    P.z = P1.z + V1.amplified(t1).dz
                    return P
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, InfinitePlane):
        # 直线与平面求交操作
        P0, V, P1, N = obj1.P, obj1.V, obj2.P, obj2.N
        dotPro = V.dotProduct(N)
        if dotPro != 0:
            t = P0.pointTo(P1).dotProduct(N) / dotPro
            return P0 + V.amplified(t)
        return None
    elif isinstance(obj1, Ray) and isinstance(obj2, InfinitePlane):
        # 射线与平面求交操作
        P1, V1 = obj1.P, obj1.V
        Q, N = obj2.P, obj2.N
        distace_value = distance(obj1, obj2)
        if distace_value == 0:  # 有交点
            angle = N.getAngle(V1)
            t = N.length() / abs(math.cos(angle))
            P = P1 + V1.amplified(t)
            return P
        return None
    elif isinstance(obj1, Segment) and isinstance(obj2, InfinitePlane):
        # 线段与平面求交操作
        return intersectSegmentPlane(obj1, obj2)
    pass


def pointOnRay(p: Point3D, ray: Ray):
    """
    点在射线上
    :param p:
    :param ray:
    :return:
    """
    v = ray.P.pointTo(p)
    if v.dotProduct(ray.V) >= 0 and v.crossProduct(ray.V).isZeroVector():
        return True
    return False


# -1 : on polygon
# 1 : inside polygon
# 0 : outside polygon
def pointInPolygon(p: Point3D, polygon: Polyline):  # 判断点是否在多边形内部
    """
    点与封闭多边形的关系
    :param p:
    :param polygon:
    :return:
    """
    passCount = 0  # 返回1：点在多边形内部；返回0：点在多边形外部；返回-1：点在多边形边上
    ray = Ray(p, Vector3D(0, 1, 0))  # 射线  TODO---修改此部分，在指定平面内进行判断
    segments = []
    for i in range(polygon.count() - 1):
        seg = Segment(polygon.point(i), polygon.point(i + 1))
        segments.append(seg)
    # 线段
    for seg in segments:  # 计算线段（不包含端点）和射线交点个数
        line1, line2 = Line(ray.P, ray.V), Line(seg.A, seg.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if nearZero(t1):  # 在端点上
                return -1
            elif (
                seg.A.y != p.y
                and seg.B.y != p.y
                and t1 > 0
                and t2 > 0
                and t2 < seg.length()
            ):
                passCount += 1
    upSegments, downSegments = [], []  # 存放射线上方和下方的线段
    for seg in segments:  # 计算多边形顶点和射线交点的个数，排除共线的可能。
        if seg.A.isIdentical(ray.P) or seg.B.isIdentical(ray.P):
            return -1  # 点在多边形上
        elif pointOnRay(seg.A, ray) ^ pointOnRay(seg.B, ray):
            if seg.A.y >= p.y and seg.B.y >= p.y:
                upSegments.append(seg)
            elif seg.A.y <= p.y and seg.B.y <= p.y:
                downSegments.append(seg)
    passCount += min(len(upSegments), len(downSegments))  # 顶点配对
    if passCount % 2 == 1:  # 射线穿过多边形次数为奇数
        return 1  # 则点在多边形内部
    return 0  # 否则点在多边形外部


def pointInPolygon_xy(p: Point3D, polygon: Polyline):  # 判断点是否在多边形内部
    """
    点与封闭多边形的关系:多边形在xy平面内
    :param p:
    :param polygon:
    :return:
    """
    passCount = 0  # 返回1：点在多边形内部；返回0：点在多边形外部；返回-1：点在多边形边上
    ray = Ray(p, Vector3D(0, 1, 0))  # 射线  TODO---修改此部分，在指定平面内进行判断
    segments = []
    for i in range(polygon.count() - 1):
        seg = Segment(polygon.point(i), polygon.point(i + 1))
        segments.append(seg)
    # 线段
    for seg in segments:  # 计算线段（不包含端点）和射线交点个数
        line1, line2 = Line(ray.P, ray.V), Line(seg.A, seg.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if nearZero(t1):  # 在端点上
                return -1
            elif (
                seg.A.y != p.y
                and seg.B.y != p.y
                and t1 > 0
                and t2 > 0
                and t2 < seg.length()
            ):
                passCount += 1
    upSegments, downSegments = [], []  # 存放射线上方和下方的线段
    for seg in segments:  # 计算多边形顶点和射线交点的个数，排除共线的可能。
        if seg.A.isIdentical(ray.P) or seg.B.isIdentical(ray.P):
            return -1  # 点在多边形上
        elif pointOnRay(seg.A, ray) ^ pointOnRay(seg.B, ray):
            if seg.A.y >= p.y and seg.B.y >= p.y:
                upSegments.append(seg)
            elif seg.A.y <= p.y and seg.B.y <= p.y:
                downSegments.append(seg)
    passCount += min(len(upSegments), len(downSegments))  # 顶点配对
    if passCount % 2 == 1:  # 射线穿过多边形次数为奇数
        return 1  # 则点在多边形内部
    return 0  # 否则点在多边形外部


def pointInPolygon_xz(p: Point3D, polygon: Polyline):  # 判断点是否在多边形内部
    """
    点与封闭多边形的关系
    :param p:
    :param polygon:
    :return:
    """
    passCount = 0  # 返回1：点在多边形内部；返回0：点在多边形外部；返回-1：点在多边形边上
    ray = Ray(p, Vector3D(1, 0, 0))  # 射线  TODO---修改此部分，在指定平面内进行判断
    segments = []
    for i in range(polygon.count() - 1):
        seg = Segment(polygon.point(i), polygon.point(i + 1))
        segments.append(seg)
    # 线段
    for seg in segments:  # 计算线段（不包含端点）和射线交点个数
        line1, line2 = Line(ray.P, ray.V), Line(seg.A, seg.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if nearZero(t1):  # 在端点上
                return -1
            elif (
                seg.A.x != p.x
                and seg.B.x != p.x
                and t1 > 0
                and t2 > 0
                and t2 < seg.length()
            ):
                passCount += 1
    upSegments, downSegments = [], []  # 存放射线上方和下方的线段
    for seg in segments:  # 计算多边形顶点和射线交点的个数，排除共线的可能。
        if seg.A.isIdentical(ray.P) or seg.B.isIdentical(ray.P):
            return -1  # 点在多边形上
        elif pointOnRay(seg.A, ray) ^ pointOnRay(seg.B, ray):
            if seg.A.x >= p.x and seg.B.x >= p.x:
                upSegments.append(seg)
            elif seg.A.x <= p.x and seg.B.x <= p.x:
                downSegments.append(seg)
    passCount += min(len(upSegments), len(downSegments))  # 顶点配对
    if passCount % 2 == 1:  # 射线穿过多边形次数为奇数
        return 1  # 则点在多边形内部
    return 0  # 否则点在多边形外部


def segmentInPolygonPoint(points: List[List[float]], polygon: Polyline):
    """
    线段在封闭多边形上的交点
    :param points:
    :param polygon:
    :return:
    """
    # 创建线段点
    point_1 = Point3D()
    point_2 = Point3D()
    point_1.x = points[0][0]
    point_1.y = points[0][1]
    point_1.z = points[0][2]
    point_2.x = points[1][0]
    point_2.y = points[1][1]
    point_2.z = points[1][2]
    segment = Segment(point_1, point_2)
    segments = []
    for i in range(polygon.count() - 1):
        seg = Segment(polygon.point(i), polygon.point(i + 1))
        segments.append(seg)
    # 线段
    points = []  # 线段相交点
    for seg in segments:  # 计算线段（不包含端点）和射线交点个数
        line1, line2 = Line(segment.A, segment.direction()), Line(
            seg.A, seg.direction()
        )
        P, t1, t2 = intersectLineLine(line1, line2)
        if (
            P is not None
            and min(t1, t2) >= 0
            and t1 <= segment.length()
            and t2 <= seg.length()
        ):  # t1和t2值有上下限，表明为线段
            points.append(P)
    return points


def intersectTrianglePlane(triangle, plane):  # 输入Triangle和Plane对象
    """
    三角形与平面的相交情况
    :param triangle:
    :param plane:
    :return:
    """
    AB = Segment(triangle.A, triangle.B)  # 三角形三条边
    AC = Segment(triangle.A, triangle.C)
    BC = Segment(triangle.B, triangle.C)
    c1 = intersectSegmentPlane(AB, plane)  # 三条边和平面交点
    c2 = intersectSegmentPlane(AC, plane)
    c3 = intersectSegmentPlane(BC, plane)
    if c1 is None:
        if c2 is not None and c3 is not None:  # 存在2个交点的情况
            if c2.distance(c3) != 0.0:  # 不重合
                return Segment(c2, c3)
    elif c2 is None:
        if c1 is not None and c3 is not None:  # 存在2个交点的情况
            if c1.distance(c3) != 0.0:  # 不重合
                return Segment(c1, c3)
    elif c3 is None:
        if c1 is not None and c2 is not None:  # 存在2个交点的情况
            if c1.distance(c2) != 0.0:  # 不重合
                return Segment(c1, c2)
    elif c1 is not None and c2 is not None and c3 is not None:  # 存在3个交点
        return Segment(c1, c3) if c1.isIdentical(c2) else Segment(c1, c2)
    return None


def intersectTriangleZPlane(triangle, z):
    """
    三角形与z平面的交点
    :param triangle:
    :param z:模型生长方向
    :return:
    """
    if triangle.zMinPnt().z > z:  # 无交线
        return None
    if triangle.zMaxPnt().z < z:  # 无交线
        return None
    return intersectTrianglePlane(triangle, InfinitePlane.zPlane(z))


def intersectTriangleXPlane(triangle, x):
    """
    三角形与x平面的交点
    :param triangle:
    :param x:模型生长方向
    :return:
    """
    if triangle.xMinPnt().x > x:  # 无交线
        return None
    if triangle.xMaxPnt().x < x:  # 无交线
        return None
    return intersectTrianglePlane(triangle, InfinitePlane.xPlane(x))


def intersectTriangleYPlane(triangle, y):
    """
    三角形与y平面的交点
    :param triangle:
    :param y:模型生长方向
    :return:
    """
    if triangle.xMinPnt().y > y:  # 无交线
        return None
    if triangle.xMaxPnt().y < y:  # 无交线
        return None
    return intersectTrianglePlane(triangle, InfinitePlane.yPlane(y))


def adjustPolygonDirs(polygons):
    """
    识别轮廓线顺逆时针方向
    :param polygons:封闭轮廓线
    :return:
    """
    for i in range(len(polygons)):  # 第1个循环
        pt = polygons[i].startPoint()  # 取出待检测轮廓的起点
        insideCount = 0  # 点在几个多边形内部计数
        for j in range(len(polygons)):  # 第2个循环
            if j == i:  # 两个多边形一样则跳过
                continue
            restPoly = polygons[j]
            if 1 == pointInPolygon(pt, restPoly):  # 点在另一多边形内部，则加1
                insideCount += 1
            if insideCount % 2 == 0:  # 判断点在内部次数是否为偶数
                polygons[i].makeCCW()  # 调整多边形方向为逆时针
            else:
                polygons[i].makeCW()  # 调整多边形方向为顺时针


def adjustPolygonDirsByRay(polygons):
    """
    用封闭多边形上一顶点与任意方向形成的射线与其余多边形交点数量判断封闭多边形是顺时针还是逆时针
    交点个数为偶数为逆时针，交点个数为奇数则为顺时针。
    :param polygons: 封闭线段集合
    :return:
    """
    for i in range(len(polygons)):  # 第1层循环
        point_s = polygons[i].startPoint()  # 取出待检测轮廓的起点
        point_e = polygons[i].endPoint()  # 结束点
        v1 = point_s.PointTo(point_e)
        v1.x = v1.x + 2
        ray = Ray(point_e, v1)
        count_1 = 0
        count_2 = 0
        for j in range(len(polygons)):  # 第2个循环
            if j == i:  # 两个多边形一样则跳过
                continue
            restPoly = polygons[j]
            for k in range(len(restPoly) - 1):
                seg = Segment()
                seg.A = restPoly[i]
                seg.B = restPoly[i + 1]
                v2 = seg.direction()
                P = intersect(ray, seg)
                if P != None and P != seg.A and P != seg.B:  # 排除端点相交的情况
                    count_1 += 1
                if P == seg.A and v1.isParallel(v2):  # 增加端点相交的情况，射线与线段平行，计数不增加
                    count_2 += 1
            count = count_1 + count_2
            if count % 2 == 0:  # 判断点在内部次数是否为偶数
                polygons[i].makeCCW()  # 调整多边形方向为逆时针
            else:
                polygons[i].makeCW()  # 调整多边形方向为顺时针


def rotatePolygons(polygons, angle, center=None):
    """
    旋转多边形，旋转中心和旋转角
    :param polygons:
    :param angle:
    :param center:
    :return:
    """
    dx = 0 if center is None else center.x
    dy = 0 if center is None else center.y
    mt = Matrix3D.createTranslateMatrix(-dx, -dy, 0)
    mr = Matrix3D.createRotateMatrix("Z", angle)
    mb = Matrix3D.createTranslateMatrix(dx, dy, 0)
    m = mt * mr * mb
    newPolys = []
    for poly in polygons:
        newPolys.append(poly.multiplied(m))
    return newPolys
