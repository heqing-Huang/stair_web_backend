"""
# File       : models.py
# Time       ：2022/9/19 18:45
# Author     ：CR_X
# version    ：python 3.8
# Description：
"""
from dataclasses import dataclass, field
from typing import List
from dc_rebar import Rebar


@dataclass
class Point(object):
    """
    点的信息：x、y、z
    """

    x: float
    y: float
    z: float


@dataclass
class RebarBIM:
    """
    为BIM准备的钢筋属性
    """

    project_ID: str = None
    stair_ID: str = None
    mark: int = None  # 钢筋序号
    rebar_length: int = None  # 钢筋长度
    rebar_quantity: int = None  # 钢筋数量
    rebar_diameter: int = None  # 钢筋直径
    rebar_grade: str = None  # 钢筋等级 使用name
    mandrel_diameter: float = None  # 弯曲直径


@dataclass
class RebarforBIM:
    """
    为生成BIM模型准备的钢筋集合的类
    """

    hole_rebar: List[Rebar] = field(default_factory=list)  # 孔洞钢筋
    lifting_point_rebar: List[Rebar] = field(default_factory=list)  # 吊点钢筋(横向)
    lifting_longitudinal_rebar: List[Rebar] = field(default_factory=list)  # 吊点纵筋
    bottom_edge_rein_rebar: List[Rebar] = field(default_factory=list)  # 底/顶部边缘加强筋
    top_edge_rein_rebar: List[Rebar] = field(default_factory=list)
    bottom_rebar: List[Rebar] = field(default_factory=list)
    top_rebar: List[Rebar] = field(default_factory=list)
    bottom_edge_stirrup_rebar: List[Rebar] = field(default_factory=list)  # 底/顶部边缘箍筋
    top_edge_stirrup_rebar: List[Rebar] = field(default_factory=list)
    bottom_rein_rebar: List[Rebar] = field(default_factory=list)
    top_rein_rebar: List[Rebar] = field(default_factory=list)
    mid_rebar: List[Rebar] = field(default_factory=list)  # 中部分布筋

    def _converter_hole_rebar(self):

        _hole_rebar = []
        for rebar in self.hole_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)  # Rebar传入参数**rebar: 钢筋半径和轨迹
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _hole_rebar.append(rebar)
        self.hole_rebar = _hole_rebar

        _lifting_point_rebar = []
        for rebar in self.lifting_point_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _lifting_point_rebar.append(rebar)
        self.lifting_point_rebar = _lifting_point_rebar

        _lifting_longitudinal_rebar = []
        for rebar in self.lifting_longitudinal_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _lifting_longitudinal_rebar.append(rebar)
        self.lifting_longitudinal_rebar = _lifting_longitudinal_rebar

        _bottom_edge_rein_rebar = []
        for rebar in self.bottom_edge_rein_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _bottom_edge_rein_rebar.append(rebar)
        self.bottom_edge_rein_rebar = _bottom_edge_rein_rebar

        _top_edge_rein_rebar = []
        for rebar in self.top_edge_rein_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _top_edge_rein_rebar.append(rebar)
        self.top_edge_rein_rebar = _top_edge_rein_rebar

        _bottom_rebar = []
        for rebar in self.bottom_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _bottom_rebar.append(rebar)
        self.bottom_rebar = _bottom_rebar

        _top_rebar = []
        for rebar in self.top_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _top_rebar.append(rebar)
        self.top_rebar = _top_rebar

        _bottom_edge_stirrup_rebar = []
        for rebar in self.bottom_edge_stirrup_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _bottom_edge_stirrup_rebar.append(rebar)
        self.bottom_edge_stirrup_rebar = _bottom_edge_stirrup_rebar
        _top_edge_stirrup_rebar = []
        for rebar in self.top_edge_stirrup_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _top_edge_stirrup_rebar.append(rebar)
        self.top_edge_stirrup_rebar = _top_edge_stirrup_rebar

        _bottom_rein_rebar = []
        for rebar in self.bottom_rein_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _bottom_rein_rebar.append(rebar)
        self.bottom_rein_rebar = _bottom_rein_rebar
        _top_rein_rebar = []
        for rebar in self.top_rein_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _top_rein_rebar.append(rebar)
        self.top_rein_rebar = _top_rein_rebar
        _mid_rebar = []
        for rebar in self.mid_rebar:
            if not isinstance(rebar, Rebar):
                if isinstance(rebar, dict):
                    rebar = Rebar(**rebar)
                else:
                    raise Exception(f"意外的rebar 类型:{rebar}")
            _mid_rebar.append(rebar)
        self.mid_rebar = _mid_rebar

    def __post_init__(self):
        self._converter_hole_rebar()
