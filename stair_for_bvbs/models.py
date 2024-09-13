"""
# File       : models.py
# Time       ：2022/9/19 18:45
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class RebarGeoBVBS:
    length: float
    angle: float


@dataclass
class RebarBVBS:
    """
    为bvbs准备的钢筋属性
    """

    project_ID: str = None
    stair_ID: str = None
    mark: int = None  # 钢筋序号
    rebar_length: int = None  # 钢筋长度
    rebar_quantity: int = None  # 钢筋数量
    rebar_diameter: int = None  # 钢筋直径
    rebar_grade: str = None  # 钢筋等级 使用name
    mandrel_diameter: int = None  # 弯曲直径
    geometric: List[RebarGeoBVBS] = field(default_factory=list)  # 钢筋形状


@dataclass
class RebarforBVBS:
    """
    为生成bvbs准备的钢筋集合的类
    """

    hole_rebar: RebarBVBS = None
    lifting_longitudinal_rebar: RebarBVBS = None
    lifting_point_rebar: RebarBVBS = None
    bottom_edge_rein_rebar: RebarBVBS = None
    top_edge_rein_rebar: RebarBVBS = None
    bottom_rebar: RebarBVBS = None
    top_rebar: RebarBVBS = None
    bottom_edge_stirrup_rebar: RebarBVBS = None
    top_edge_stirrup_rebar: RebarBVBS = None
    bottom_rein_rebar: RebarBVBS = None
    top_rein_rebar: RebarBVBS = None
    mid_rebar: RebarBVBS = None
