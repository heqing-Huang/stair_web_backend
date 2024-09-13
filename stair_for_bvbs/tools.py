"""
# File       : tools.py
# Time       ：2022/12/09 11:27
# Author     ：CR_X
# version    ：python 3.8
# Description：
"""


def rebar_mandrel_diameter(radius=5, steel_grade=1):
    if steel_grade == 1:
        mandrel_diameter = 1.25 * radius * 2
    elif steel_grade == 2:
        mandrel_diameter = 2.0 * radius * 2
    elif steel_grade == 3:
        mandrel_diameter = 2.5 * radius * 2
    else:
        mandrel_diameter = 2.5 * radius * 2
    return int(mandrel_diameter)


def get_z(slope: float, y_0: float, z_0: float, y: float) -> float:
    """
    已知斜率和一点坐标，根据y值求解该直线上z坐标
    :param slope: 斜率
    :param y_0: 固定y坐标
    :param z_0: 固定z坐标
    :param y: 动态y坐标
    :return:
    """
    z = z_0 + slope * (y - y_0)
    return z


def get_y(slope: float, y_0: float, z_0: float, z) -> float:
    """
    已知斜率和一点坐标，根据z值求解该直线上y坐标
    :param slope: 斜率
    :param y_0: 固定y坐标
    :param z_0: 固定z坐标
    :param z: 动态z坐标
    :return:
    """
    y = y_0 + (z - z_0) / slope
    return y
