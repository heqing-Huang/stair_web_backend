"""
# File       : Fcl_models.py
# Time       ：2022/9/21 12:16
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Rebar_fcl:
    # 钢筋的fcl数据类型
    type = "Rebar"
    diameter: float
    length: float
    transformation: object
    position: object


@dataclass
class Box_fcl:
    """
    position: 包围盒的中心点坐标
    """

    # 竖向长方体包围盒
    type = "Box"
    x: float
    y: float
    z: float
    transformation = np.eye(3)
    position: object


@dataclass
class Diagonal_fcl:
    # 斜向长方体包围盒
    type = "Box"
    x: float
    y: float
    z: float
    transformation: object
    position: object


@dataclass
class Cylinder_fcl:
    # 圆柱体包围盒 包围盒是中心点坐标 默认法线方向是平行于Z轴
    type = "Cylinder"
    radius: float
    length: float
    position: object


@dataclass
class Agent:
    # 智能体的fcl数据
    size: float
    position: list
