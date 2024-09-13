"""
  几何实体的表示方式

"""

from .GeomBase import Point3D, Vector3D
from .GeomLines import Line, Segment
from .GeomPlane import FinitePlane
from dataclasses import dataclass


@dataclass
class Entity:
    """
    几何实体：由不同的面组成的封闭三维图形
    """

    def __init__(self):
        self.surfaces = []

    def __str__(self):
        """
        定义实体print的显示:显示有限平面的个数
        :return:
        """
        if self.count() > 0:
            return "Entity\nSurface number: %s\n" % (self.count())
        else:
            return "Entity\nSurface number: 0\n"
        pass

    def count(self):
        """
        统计实体中有限平面的个数
        :return:
        """
        return len(self.surfaces)

    def addSurface(self, face):
        """
        添加有限平面
        :param face:
        :return:
        """
        self.surfaces.append(face)
