"""

约定钢筋表达形式

"""
import logging
from functools import partial
from typing import List, Optional

# from dataclasses import dataclass, field, Field, MISSING
from converter_dataclass import (
    field_with_converter as field,
    dataclass_with_converter as dataclass,
    iter_convert,
)

_LOGGER = logging.getLogger(__name__)

__version__ = "0.0.6"


@dataclass
class Point3D:
    """
    定义三维空间中一个点
    """

    x: float = 0
    y: float = 0
    z: float = 0


@dataclass
class IndexedPolyCurve:
    """

    参照 http://www.vfkjsd.cn/ifc/ifc4_1/buildingsmart/ifcindexedpolycurve.htm 表示钢筋

    此格式兼容此前两种钢筋表示方式[两种具体方式请联系作者]

    此处segments,通过嵌套列表内的length 判断是弧段还是线段

    该钢筋表示在局部坐标中的轨迹,局部坐标 xyz 为标准 xyz 方向向量

    """

    points: List[Point3D] = field(
        converter=partial(iter_convert, func=Point3D.converter), default_factory=list
    )
    segments: List[List[int]] = field(default_factory=list)


@dataclass
class Rebar:
    """
    钢筋定义需要的:
        - 半径
        - 轨迹
        - 材质 [暂不需要]

    """

    radius: int = 4
    poly: Optional[IndexedPolyCurve] = field(
        converter=IndexedPolyCurve.converter, default=None
    )


@dataclass
class Direction(Point3D):
    """
    定义向量,默认是x 为方向的向量
    """

    x: float = 1


@dataclass
class Coordinate:
    """
    定义坐标系
    """

    x: Direction = field(converter=Direction.converter)
    y: Direction = field(converter=Direction.converter)
    z: Direction = field(converter=Direction.converter)


@dataclass
class GroupTheSameRebar:
    """

    形状相同的钢筋进行分组,如果不便于从分散的、相同的钢筋中汇总成该格式,则仅使用List[rebar]表示分组

    """

    # 多个坐标系,该种做法参照与IFC,如果是Revit,亦可通过转换得到新数据
    coordinates: List[Coordinate] = field(
        converter=partial(iter_convert, func=Coordinate.converter), default=list
    )
    # 局部坐标系定义的单根钢筋性质
    rebar: Rebar = field(converter=Rebar.converter, default=None)
