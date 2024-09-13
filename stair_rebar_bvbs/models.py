from typing import (
    List,
)
from dataclasses import dataclass


@dataclass
class GEO(object):
    """S
    钢筋数据
    """

    length: float
    angle: float
    lengthCompensation: float  # 长度补偿
    angleCompensation: float  # 角度补偿
    direction: bool
    arc: bool
    retract: bool


@dataclass
class Subs(object):
    """
    钢筋数据
    """

    geos: List[GEO]


@dataclass
class WriteRebarJSON(object):
    """
    钢筋数据
    """

    billcode: int  # 任务编号
    plan: int  # 计划产量
    finish: int  # 完成量
    diameter: int  # 直径
    subs: Subs


@dataclass
class ReadRebarJSON(object):
    """
    钢筋数据
    """

    billcode: int  # 任务编号
    plan: int  # 计划产量
    finish: int  # 完成量
    diameter: int  # 直径
    subs: Subs
