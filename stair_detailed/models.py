"""
# File       : models.py
# Time       ：2022/9/19 18:45
# Author     ：CR_X
# version    ：python 3.6
# Description：
"""
from enum import Enum
from functools import partial
from typing import Union, Optional

from converter_dataclass import dataclass_with_converter as dataclass
from converter_dataclass import field_with_converter as field
from converter_dataclass import iter_convert
from stair_structure.model import StructuralDesign, StructuralDesignResult


class RebarDesignMode(Enum):
    """
    钢筋设计模式枚举
    """

    AUTOMATIC = 0
    MANUAL = 1


class HoleDesignMode(Enum):
    """
    孔洞设计模式枚举
    """

    AUTOMATIC = 0
    MANUAL = 1


class JointDesignMode(Enum):
    """
    节点设计模式枚举
    """

    AUTOMATIC = 0
    MANUAL = 1


class StepSlotDesignMode(Enum):
    """
    防滑槽设计模式枚举
    """

    AUTOMATIC = 0
    MANUAL = 1
    NO = 2


class WaterDripDesignMode(Enum):
    """
    滴水槽设计模式枚举
    """

    MANUAL = 1
    NO = 2


class HoleType(Enum):
    """
    顶端孔洞类型
    """

    FIXED_HINGE = 0
    SLIDING_HINGE = 1


class WaterDripLayout(Enum):
    """
    滴水槽布局
    """

    ONLY_TOP = 0
    ONLY_BOTTOM = 1
    BOTH = 2


class WaterDripShape(Enum):
    """
    滴水槽形状类型
    """

    TRAPEZOID = 0
    SEMICIRCLE = 1


class LiftingDesignMode(Enum):
    """
    吊装预埋件设计模式枚举
    """

    AUTOMATIC = 0
    MANUAL = 1


class DemoldingDesignMode(Enum):
    """
    脱模预埋件设计模式
    """

    AUTOMATIC = 0
    MANUAL = 1


class RailDesignMode(Enum):
    """
    栏杆预埋件设计模式
    """

    MANUAL = 1
    NO = 2


class LiftingType(Enum):
    """
    吊装预埋件类型
    """

    ROUNDING_HEAD = 0
    ANCHOR = 1


class PouringWay(Enum):
    """
    脱模方式
    """

    VERTICAL_HORIZONTAL = 0  # 立式浇筑卧式脱模，
    VERTICAL_VERTICAL = 1  # 立式浇筑立式脱模，
    HORIZONTAL_HORIZONTAL = 2  # 卧式浇筑卧式脱模


class DemoldingType(Enum):
    """
    脱模预埋件类型
    """

    ROUNDING_HEAD = 0
    ANCHOR = 1

    # LIFT_HOOK = 2


class RailLayout(Enum):
    """
    栏杆布局
    """

    ONLY_RIGHT = 0
    ONLY_LEFT = 1
    BOTH = 2


@dataclass
class GeometricDetailed:
    """
    深化设计几何参数，补充结构设计阶段几何尺寸缺失的部分
    长度单位均为毫米
    """

    # 梯段板宽度   即b0， 深化设计输入的值会小
    width: int
    # 顶端上边长 深化设计输入的值可能会略小
    top_top_length: int
    # 顶端板厚
    top_thickness: int
    # 顶端挑耳宽度
    top_b: int
    # 底端上边长 深化设计输入的值可能会略小
    bottom_top_length: int
    # 底端板厚
    bottom_thickness: int
    # 底端挑耳宽度
    bottom_b: int

    @classmethod
    def converter(cls, datas):
        # 将字典形式的datas 转换成为dataclass 类型
        return cls(**datas)


@dataclass
class FixedHinge:
    """
    固定铰支座 一般在顶端
    """

    # 固定铰支座孔洞尺寸
    fix_hinge_c2: int
    fix_hinge_d2: int


@dataclass
class SlidingHinge:
    """
    滑动铰支座 一般在底端
    """

    # 滑动铰支座孔洞尺寸
    sliding_hinge_c1: int
    sliding_hinge_d1: int
    sliding_hinge_e1: int
    sliding_hinge_f1: int
    sliding_hinge_h1: int


@dataclass(frozen=True)
class RoundHeadParameter(object):
    """
    圆头吊钉形状数据
    """

    type: str  # 类型
    factory: str  # 系列名
    name: str  # 名称
    abbreviation: str  # 编号前缀
    capacity: float  # 承载力
    length: float  # 吊钉长度
    top_diameter: float  # 顶部直径
    top_height: float  # 顶部高度
    top_adjacent_height: float  # 顶部连接高度
    middle_diameter: float  # 中间直径
    middle_height: float  # 中间高度
    bottom_adjacent_height: float  # 底部连接高度
    bottom_diameter: float  # 底部半径
    bottom_height: float  # 底部高度
    radius: float  # 埋入半径


@dataclass(frozen=True)
class AnchorParameter(object):
    """
    预埋锚栓：
    """

    type: str  # 类型
    factory: str  # 系列名
    name: str  # 名称
    abbreviation: str  # 编号前缀
    capacity: float  # 承载力
    m_diameter: float  # 接口直径
    m_length: float  # 埋入深度
    anchor_name: str  # 锚栓螺纹
    e_diameter: float  # 嵌入直径
    g: float  # 嵌入长度
    b: float  # 嵌入箭头长度
    o_diameter: float  # 锚栓直径
    length: float  # 锚栓长度
    s_diameter: float  # 卡槽直径
    l_p: float  # 卡槽长度
    a: float  # 卡槽边距


# @dataclass
# class LiftHookParameter:
#     """
#     吊钩预埋件数据
#     """
#     type: str  # 类型
#     name: str  # 名称
#     abbreviation: str  # 编号前缀
#     material: str  # 材质
#     diameter: float  # 钢筋直径
#     top_radius: float  # 顶部半径
#     top_height: float  # 顶部高度
#     bottom_radius: float  # 底部半径
#     bottom_height: float  # 底部高度
#     m_height: float  # 弯钩高度
#     a1: int  # 企口长边内边长
#     b1: int  # 企口长边外边长
#     a2: int  # 企口短边内边长
#     b2: int  # 企口短边外边长
#     d: int  # 企口深度


@dataclass
class TopHolePosition:
    """
    孔洞定位 边距大小
    """

    a1: int
    a2: int
    b1: int
    b2: int


@dataclass
class BottomHolePosition:
    """
    孔洞定位 边距大小
    """

    a3: int
    a4: int
    b3: int
    b4: int


@dataclass
class StepSlot:
    """
    防滑槽信息
    """

    # 防滑槽尺寸参数
    a: int
    b: int
    c: int
    d: int
    e: int


@dataclass
class StepSlotPosition:
    """
    防滑槽信息
    """

    # 防滑槽的定位参数
    c1: int
    c2: int
    c3: int


@dataclass
class WaterDripTrapezoid:
    """
    梯形线槽
    """

    a: float
    b: float
    c: float


@dataclass
class WaterDripSemicircle:
    """
    半圆形滴水线槽
    """

    a: float
    b: float


@dataclass
class WaterDripPosition:
    """
    滴水线槽定位
    """

    a1: float
    a2: float
    a3: float


@dataclass
class JointShape:
    """
    节点形状尺寸：a、b、b
    """

    a: int
    b: int
    c: int


@dataclass
class ConstructionDetailed:
    """
    深化设计连接孔洞、节点、防滑槽、滴水槽的参数设计
    """

    # 0 代表自动计算,1 代表手动输入
    hole_design_mode: HoleDesignMode = HoleDesignMode.AUTOMATIC
    joint_design_mode: JointDesignMode = JointDesignMode.AUTOMATIC
    step_slot_design_mode: StepSlotDesignMode = StepSlotDesignMode.AUTOMATIC
    water_drip_design_mode: WaterDripDesignMode = WaterDripDesignMode.NO
    # 孔洞设计
    top_hole_type: Optional[HoleType] = None  # 顶端孔洞类型
    top_hole: Union[FixedHinge, SlidingHinge] = None  # 顶端孔洞参数
    top_hole_position: Optional[TopHolePosition] = None  # 底端孔洞位置参数
    bottom_hole_type: Optional[HoleType] = None  # 底端孔洞类型
    bottom_hole: Union[FixedHinge, SlidingHinge] = None  # 底端孔洞参数
    bottom_hole_position: Optional[BottomHolePosition] = None  # 底端孔洞位置参数
    # 节点设计
    top_joint: Optional[JointShape] = None  # 上部节点参数
    bottom_joint: Optional[JointShape] = None  # 下部节点参数
    # 防滑槽
    step_slot: Optional[StepSlot] = None  # 防滑槽参数
    step_slot_position: Optional[StepSlotPosition] = None  # 防滑槽位置参数
    # 滴水槽
    water_drip_layout: Optional[
        WaterDripLayout
    ] = None  # 布局位置 only_top仅上侧，only_bottom仅下侧，both两侧
    water_drip_shape: WaterDripShape = None  # trapezoid梯形截面，semicircle半圆形截面
    water_drip: Union[WaterDripSemicircle, WaterDripTrapezoid] = None  # 参数
    water_drip_position: WaterDripPosition = None  # 位置参数

    @classmethod
    def converter(cls, data):
        return cls(**data)

    def __post_converter__(self):
        if isinstance(self.hole_design_mode, int):
            self.hole_design_mode = HoleDesignMode(self.hole_design_mode)
        assert isinstance(self.hole_design_mode, HoleDesignMode), Exception(
            f"hole_design_mode 传入类型错误"
        )

        if self.hole_design_mode == HoleDesignMode.AUTOMATIC:
            self.top_hole_type: HoleType = HoleType.FIXED_HINGE  # 顶端孔洞类型
            self.top_hole: Union[FixedHinge, SlidingHinge] = FixedHinge(
                fix_hinge_c2=60, fix_hinge_d2=50
            )  # 顶端孔洞参数
            self.top_hole_position: TopHolePosition = TopHolePosition(
                a1=100, a2=100, b1=300, b2=300
            )  # 底端孔洞位置参数
            self.bottom_hole_type: HoleType = HoleType.SLIDING_HINGE  # 底端孔洞类型
            self.bottom_hole: Union[FixedHinge, SlidingHinge] = SlidingHinge(
                sliding_hinge_c1=70,
                sliding_hinge_d1=55,
                sliding_hinge_e1=65,
                sliding_hinge_f1=50,
                sliding_hinge_h1=50,
            )  # 底端孔洞参数
            # 底端孔洞位置参数
            self.bottom_hole_position: BottomHolePosition = BottomHolePosition(
                a3=100, a4=100, b3=300, b4=300
            )
        elif self.hole_design_mode == HoleDesignMode.MANUAL:
            assert self.top_hole_type is not None, Exception(f"top_hole_type 不能为空")
            if isinstance(self.top_hole_type, int):
                self.top_hole_type = HoleType(self.top_hole_type)
            if not isinstance(self.top_hole_type, HoleType):
                raise Exception(f"top_hole_type 数据异常:{self.top_hole_type}")

            assert self.top_hole is not None, Exception(f"top_hole 不能为空")
            if isinstance(self.top_hole, dict):  # 兼容
                if self.top_hole_type == HoleType.FIXED_HINGE:
                    self.top_hole = FixedHinge(**self.top_hole)
                elif self.top_hole_type == HoleType.SLIDING_HINGE:
                    self.top_hole = SlidingHinge(**self.top_hole)
            if not isinstance(self.top_hole, FixedHinge) and not isinstance(
                self.top_hole, SlidingHinge
            ):
                raise Exception(f"top_hole 数据异常:{self.top_hole}")

            assert self.top_hole_position is not None, Exception(
                f"top_hole_position 不能为空"
            )
            if isinstance(self.top_hole_position, dict):
                self.top_hole_position = TopHolePosition(**self.top_hole_position)
            if not isinstance(self.top_hole_position, TopHolePosition):
                raise Exception(f"top_hole_position 数据异常:{self.top_hole_position}")

            assert self.bottom_hole_type is not None, Exception(
                f"bottom_hole_type 不能为空"
            )
            if isinstance(self.bottom_hole_type, int):
                self.bottom_hole_type = HoleType(self.bottom_hole_type)
            if not isinstance(self.bottom_hole_type, HoleType):
                raise Exception(f"bottom_hole_type 数据异常:{self.bottom_hole_type}")

            assert self.bottom_hole is not None, Exception(f"bottom_hole 不能为空")
            if isinstance(self.bottom_hole, dict):  # 兼容
                if self.bottom_hole_type == HoleType.FIXED_HINGE:
                    self.bottom_hole = FixedHinge(**self.bottom_hole)
                elif self.bottom_hole_type == HoleType.SLIDING_HINGE:
                    self.bottom_hole = SlidingHinge(**self.bottom_hole)
            if not isinstance(self.bottom_hole, FixedHinge) and not isinstance(
                self.bottom_hole, SlidingHinge
            ):
                raise Exception(f"bottom_hole 数据异常:{self.bottom_hole}")

            assert self.bottom_hole_position is not None, Exception(
                f"bottom_hole_position 不能为空"
            )
            if isinstance(self.bottom_hole_position, dict):
                self.bottom_hole_position = BottomHolePosition(
                    **self.bottom_hole_position
                )
            if not isinstance(self.bottom_hole_position, BottomHolePosition):
                raise Exception(
                    f"bottom_hole_position 数据异常:{self.bottom_hole_position}"
                )
        # 节点设计
        if isinstance(self.joint_design_mode, int):
            self.joint_design_mode = JointDesignMode(self.joint_design_mode)
        assert isinstance(self.joint_design_mode, JointDesignMode), Exception(
            f"joint_design_mode 传入类型错误"
        )

        if (
            self.joint_design_mode == JointDesignMode.AUTOMATIC
        ):  # automatic为自动计算，manual为手动输入
            self.top_joint: JointShape = JointShape(a=30, b=50, c=20)  # 上部节点参数
            self.bottom_joint: JointShape = JointShape(a=30, b=50, c=20)  # 下部节点参数
        elif self.joint_design_mode == JointDesignMode.MANUAL:
            assert self.top_joint is not None, Exception(f"top_joint 不能为空")
            if isinstance(self.top_joint, dict):
                self.top_joint = JointShape(**self.top_joint)
            if not isinstance(self.top_joint, JointShape):
                raise Exception(f"top_joint 数据异常:{self.top_joint}")

            assert self.bottom_joint is not None, Exception(f"bottom_joint 不能为空")
            if isinstance(self.bottom_joint, dict):
                self.bottom_joint = JointShape(**self.bottom_joint)
            if not isinstance(self.bottom_joint, JointShape):
                raise Exception(f"bottom_joint 数据异常:{self.bottom_joint}")

        # 防滑槽参数
        if isinstance(self.step_slot_design_mode, int):
            self.step_slot_design_mode = StepSlotDesignMode(self.step_slot_design_mode)
        assert isinstance(self.step_slot_design_mode, StepSlotDesignMode), Exception(
            f"step_slot_design_mode 传入类型错误"
        )

        if self.step_slot_design_mode == StepSlotDesignMode.AUTOMATIC:
            self.step_slot: Optional[StepSlot] = StepSlot(
                a=9, b=6, c=15, d=8, e=6
            )  # 防滑槽参数
            self.step_slot_position: StepSlotPosition = StepSlotPosition(
                c1=50, c2=50, c3=21
            )  # 防滑槽位置参数
        elif self.step_slot_design_mode == StepSlotDesignMode.MANUAL:
            assert self.step_slot is not None, Exception(f"step_slot 不能为空")  # 防滑槽参数
            if isinstance(self.step_slot, dict):
                self.step_slot = StepSlot(**self.step_slot)
            if not isinstance(self.step_slot, StepSlot):
                raise Exception(f"step_slot 数据异常:{self.step_slot}")

            assert self.step_slot_position is not None, Exception(
                f"step_slot_position 不能为空"
            )  # 防滑槽位置参数
            if isinstance(self.step_slot_position, dict):
                self.step_slot_position = StepSlotPosition(**self.step_slot_position)
            if not isinstance(self.step_slot_position, StepSlotPosition):
                raise Exception(f"step_slot_position 数据异常:{self.step_slot_position}")
        elif self.step_slot_design_mode == StepSlotDesignMode.NO:
            pass

        # 滴水槽参数
        if isinstance(self.water_drip_design_mode, int):
            self.water_drip_design_mode = WaterDripDesignMode(
                self.water_drip_design_mode
            )
        assert isinstance(self.water_drip_design_mode, WaterDripDesignMode), Exception(
            f"water_drip_design_mode 传入类型错误"
        )

        if self.water_drip_design_mode == WaterDripDesignMode.MANUAL:
            assert self.water_drip_layout is not None, Exception(
                f"water_drip_design_layout 不能为空"
            )  # 防滑槽参数
            if isinstance(self.water_drip_layout, int):
                self.water_drip_layout = WaterDripLayout(self.water_drip_layout)
            if not isinstance(self.water_drip_layout, WaterDripLayout):
                raise Exception(
                    f"water_drip_design_layout 数据异常:{self.water_drip_layout}"
                )

            assert self.water_drip_shape is not None, Exception(
                f"water_drip_shape 不能为空"
            )  # 防滑槽位置参数
            if isinstance(self.water_drip_shape, int):
                self.water_drip_shape = WaterDripShape(self.water_drip_shape)
            if not isinstance(self.water_drip_shape, WaterDripShape):
                raise Exception(f"water_drip_shape 数据异常:{self.water_drip_shape}")

            assert self.water_drip is not None, Exception(f"water_drip 不能为空")
            if isinstance(self.water_drip, dict):  # 兼容
                if self.water_drip_shape == WaterDripShape.TRAPEZOID:
                    self.water_drip = WaterDripTrapezoid(**self.water_drip)
                elif self.water_drip_shape == WaterDripShape.SEMICIRCLE:
                    self.water_drip = WaterDripSemicircle(**self.water_drip)
            if (not isinstance(self.water_drip, WaterDripTrapezoid)) and (
                not isinstance(self.water_drip, WaterDripSemicircle)
            ):
                raise Exception(f"water_drip 数据异常:{self.water_drip}")

            assert self.water_drip_position is not None, Exception(
                f"water_drip_position 不能为空"
            )  # 防滑槽位置参数
            if isinstance(self.water_drip_position, dict):
                self.water_drip_position = WaterDripPosition(**self.water_drip_position)
            if not isinstance(self.water_drip_position, WaterDripPosition):
                raise Exception(f"water_drip_position 数据异常:{self.water_drip_position}")
        elif self.water_drip_design_mode == WaterDripDesignMode.NO:
            pass


@dataclass
class RebarDiamSpac(object):
    """
    钢筋基本信息：直径，间距
    """

    diameter: int  # 钢筋的直径
    spacing: int  # 钢筋的间距


@dataclass
class RebarDiam(object):
    """
    钢筋基本信息：直径
    """

    diameter: int  # 钢筋的直径


@dataclass
class RebarDetailed:
    """
    深化设计钢筋设计各类钢筋的直径和间距计算类，后期可在此类中增加判断逻辑,也可以增加钢筋边距要求。
    深化钢筋名称及对应编号：
    """

    # 0 代表自动计算,1 代表手动输入
    rebar_design_mode: RebarDesignMode = RebarDesignMode.AUTOMATIC
    # 手动输入需要的参数
    bottom_edge_longitudinal_rebar: RebarDiamSpac = None  # 4号底端边缘纵筋
    top_edge_longitudinal_rebar: RebarDiamSpac = None  # 5号顶端边缘纵筋
    bottom_edge_stirrup: RebarDiamSpac = None  # 6号底端边缘箍筋
    top_edge_stirrup: RebarDiamSpac = None  # 9号顶端边缘箍筋
    hole_reinforce_rebar: RebarDiam = None  # 7号销键加强筋
    lifting_reinforce_rebar: RebarDiam = None  # 8号吊点加强筋
    top_edge_reinforce_rebar: RebarDiam = None  # 10号上部边缘加强筋
    bottom_edge_reinforce_rebar: RebarDiam = None  # 11号下部边缘加强筋

    @classmethod
    def converter(cls, data):
        """
        定义强制类型转换函数
        :param data:
        :return:
        """
        return cls(**data)

    def __post_converter__(self):
        if isinstance(self.rebar_design_mode, int):
            self.rebar_design_mode: int
            self.rebar_design_mode = RebarDesignMode(self.rebar_design_mode)
        assert isinstance(self.rebar_design_mode, RebarDesignMode), Exception(
            f"rebar_design_mode 传入类型错误"
        )

        if self.rebar_design_mode == RebarDesignMode.AUTOMATIC:
            self.bottom_edge_longitudinal_rebar = RebarDiamSpac(
                diameter=12, spacing=210
            )  # 4号底端边缘纵筋
            self.top_edge_longitudinal_rebar = RebarDiamSpac(
                diameter=12, spacing=210
            )  # 5号顶端边缘纵筋
            self.bottom_edge_stirrup = RebarDiamSpac(
                diameter=8, spacing=130
            )  # 6号底端边缘箍筋
            self.top_edge_stirrup = RebarDiamSpac(diameter=8, spacing=130)  # 9号顶端边缘箍筋
            self.hole_reinforce_rebar = RebarDiam(diameter=10)  # 7号销键加强筋
            self.lifting_reinforce_rebar = RebarDiam(diameter=12)  # 8号吊点加强筋
            self.top_edge_reinforce_rebar = RebarDiam(diameter=12)  # 10号上部边缘加强筋
            self.bottom_edge_reinforce_rebar = RebarDiam(diameter=12)  # 11号下部边缘加强筋
        else:
            assert self.bottom_edge_longitudinal_rebar is not None, Exception(
                f"bottom_edge_longitudinal_rebar 不能为空"
            )
            if isinstance(self.bottom_edge_longitudinal_rebar, dict):
                self.bottom_edge_longitudinal_rebar: dict
                self.bottom_edge_longitudinal_rebar = RebarDiamSpac(
                    **self.bottom_edge_longitudinal_rebar
                )
            if not isinstance(self.bottom_edge_longitudinal_rebar, RebarDiamSpac):
                raise Exception(
                    f"bottom_edge_longitudinal_rebar 数据异常:{self.bottom_edge_longitudinal_rebar}"
                )

            assert self.top_edge_longitudinal_rebar is not None, Exception(
                f"top_edge_longitudinal_rebar 不能为空"
            )
            if isinstance(self.top_edge_longitudinal_rebar, dict):
                self.top_edge_longitudinal_rebar: dict
                self.top_edge_longitudinal_rebar = RebarDiamSpac(
                    **self.top_edge_longitudinal_rebar
                )
            if not isinstance(self.top_edge_longitudinal_rebar, RebarDiamSpac):
                raise Exception(
                    f"top_edge_longitudinal_rebar 数据异常:{self.top_edge_longitudinal_rebar}"
                )

            assert self.bottom_edge_stirrup is not None, Exception(
                f"bottom_edge_stirrup 不能为空"
            )
            if isinstance(self.bottom_edge_stirrup, dict):
                self.bottom_edge_stirrup: dict
                self.bottom_edge_stirrup = RebarDiamSpac(**self.bottom_edge_stirrup)
            if not isinstance(self.bottom_edge_stirrup, RebarDiamSpac):
                raise Exception(f"bottom_edge_stirrup 数据异常:{self.bottom_edge_stirrup}")

            assert self.top_edge_stirrup is not None, Exception(
                f"top_edge_stirrup 不能为空"
            )
            if isinstance(self.top_edge_stirrup, dict):
                self.top_edge_stirrup: dict
                self.top_edge_stirrup = RebarDiamSpac(**self.top_edge_stirrup)
            if not isinstance(self.top_edge_stirrup, RebarDiamSpac):
                raise Exception(f"top_edge_stirrup 数据异常:{self.top_edge_stirrup}")

            assert self.hole_reinforce_rebar is not None, Exception(
                f"hole_reinforce_rebar 不能为空"
            )
            if isinstance(self.hole_reinforce_rebar, dict):
                self.hole_reinforce_rebar: dict
                self.hole_reinforce_rebar = RebarDiam(**self.hole_reinforce_rebar)
            if not isinstance(self.hole_reinforce_rebar, RebarDiam):
                raise Exception(
                    f"hole_reinforce_rebar 数据异常:{self.hole_reinforce_rebar}"
                )

            assert self.lifting_reinforce_rebar is not None, Exception(
                f"lifting_reinforce_rebar 不能为空"
            )
            if isinstance(self.lifting_reinforce_rebar, dict):
                self.lifting_reinforce_rebar: dict
                self.lifting_reinforce_rebar = RebarDiam(**self.lifting_reinforce_rebar)
            if not isinstance(self.lifting_reinforce_rebar, RebarDiam):
                raise Exception(
                    f"lifting_reinforce_rebar 数据异常:{self.lifting_reinforce_rebar}"
                )

            assert self.top_edge_reinforce_rebar is not None, Exception(
                f"top_edge_reinforce_rebar 不能为空"
            )
            if isinstance(self.top_edge_reinforce_rebar, dict):
                self.top_edge_reinforce_rebar: dict
                self.top_edge_reinforce_rebar = RebarDiam(
                    **self.top_edge_reinforce_rebar
                )
            if not isinstance(self.top_edge_reinforce_rebar, RebarDiam):
                raise Exception(
                    f"top_edge_reinforce_rebar 数据异常:{self.top_edge_reinforce_rebar}"
                )

            assert self.bottom_edge_reinforce_rebar is not None, Exception(
                f"bottom_edge_reinforce_rebar 不能为空"
            )
            if isinstance(self.bottom_edge_reinforce_rebar, dict):
                self.bottom_edge_reinforce_rebar: dict
                self.bottom_edge_reinforce_rebar = RebarDiam(
                    **self.bottom_edge_reinforce_rebar
                )
            if not isinstance(self.bottom_edge_reinforce_rebar, RebarDiam):
                raise Exception(
                    f"bottom_edge_reinforce_rebar 数据异常:{self.bottom_edge_reinforce_rebar}"
                )


@dataclass
class LiftingPosition:
    """
    吊装预埋件定位信息：a、b、c、d
    """

    a: int  # 吊装预埋件顶端踏步阶数
    b: int  # 吊装预埋件底端踏步阶数
    c: float  # 吊装预埋件左侧横向边距
    d: float  # 吊装预埋件右侧横向边距


@dataclass
class DemoldingPosition:
    """
    脱模预埋件定位信息：a、b、c、d、t
    """

    a: float  # 脱模埋件顶端纵向边距
    b: float  # 脱模埋件底端纵向边距
    c: float  # 脱模埋件左侧横向边距
    d: float  # 脱模埋件右侧横向边距
    t: float  # 脱模埋件厚度方向边距


@dataclass(frozen=True)
class RailParameter(object):
    """
    栏杆预埋件数据:name、a、b、c、d、t、fi，注：栏杆埋入企口的深度和边长在此明确
    """

    name: str
    a: int
    b: int
    c: int
    d: int
    t: int
    fi: int  # 直径
    depth: int  # 栏杆埋入深度
    length: int  # 底边长度


@dataclass
class RailPosition:
    """
    栏杆预埋件定位信息：a、b、c、d

    """

    a: float  # 栏杆埋件横向边距
    b: float  # 栏杆埋件踏步上的纵向边距


@dataclass
class InsertsDetailed:
    """
    预埋件设计的参数输入
    """

    lifting_design_mode: LiftingDesignMode = (
        LiftingDesignMode.AUTOMATIC
    )  # automatic为孔洞设计自动计算，manual为销键孔洞手动输入，默认为自动计算
    demolding_design_mode: DemoldingDesignMode = (
        DemoldingDesignMode.AUTOMATIC
    )  # automatic自动计算，manual手动输入
    rail_design_mode: RailDesignMode = RailDesignMode.NO  # no为无，manual为手动输入
    # 吊装预埋件
    lifting_type: Optional[LiftingType] = None  # 吊装预埋件类型
    lifting_position: Optional[LiftingPosition] = field(
        converter=LiftingPosition.converter, default=None
    )  # 吊装预位置参数
    lifting_name: Optional[str] = None  # 吊装预埋件规格
    # 脱模预埋件
    # vertical_horizontal立式浇筑卧式脱模，vertical_vertical立式浇筑立式脱模， horizontal_horizontal卧式浇筑卧式脱模
    pouring_way: Optional[PouringWay] = None  # 脱模方式
    demolding_type: Optional[DemoldingType] = None  # 脱模类型
    demolding_position: Optional[DemoldingPosition] = field(
        converter=DemoldingPosition.converter, default=None
    )  # 脱模预位置参数
    demolding_name: Optional[str] = None  # 脱模预埋件规格

    # 栏杆预埋件
    rail_layout: Optional[RailLayout] = None  # 栏杆预埋件布局
    rail_number: Optional[list] = None  # 栏杆预埋件所在的阶数，以底端板为第0阶开始计数
    rail_position: Optional[RailPosition] = None  # 栏杆位置
    rail_name: Optional[str] = None  # 栏杆预埋件型号

    @classmethod
    def converter(cls, data):
        return cls(**data)

    def __post_converter__(self):
        # 埋件设计--吊装预埋件
        if isinstance(self.lifting_design_mode, int):
            self.lifting_design_mode = LiftingDesignMode(self.lifting_design_mode)
        assert isinstance(self.lifting_design_mode, LiftingDesignMode), Exception(
            f"lifting_design_mode 传入类型错误"
        )

        if self.lifting_design_mode == LiftingDesignMode.AUTOMATIC:
            self.lifting_type: Optional[
                LiftingType
            ] = LiftingType.ROUNDING_HEAD  # 吊装预埋件类型
        elif self.lifting_design_mode == LiftingDesignMode.MANUAL:
            assert self.lifting_type is not None, Exception(f"lifting_type 不能为空")
            if isinstance(self.lifting_type, int):
                self.lifting_type = LiftingType(self.lifting_type)
            if not isinstance(self.lifting_type, LiftingType):
                raise Exception(f"lifting_type 数据异常:{self.lifting_type}")

            assert self.lifting_position is not None, Exception(
                f"lifting_position 不能为空"
            )
            if isinstance(self.lifting_position, dict):
                self.lifting_position = LiftingPosition(**self.lifting_position)
            if not isinstance(self.lifting_position, LiftingPosition):
                raise Exception(f"lifting_position 数据异常:{self.lifting_position}")

            assert self.lifting_name is not None, Exception(f"lifting_name 不能为空")
            if self.lifting_type == LiftingType.ROUNDING_HEAD:
                assert self.lifting_name in [
                    "DJ-13-120",
                    "DJ-20-140",
                    "DJ-25-170",
                    "DJ-30-210",
                    "DJ-75-300",
                ], Exception(f"吊装预埋件名称异常:{self.lifting_name}")
            elif self.lifting_type == LiftingType.ANCHOR:
                assert self.lifting_name in [
                    "MS-6-60",
                    "MS-9-60",
                    "MS-12-70",
                    "MS-15-75",
                    "MS-24-80",
                    "MS-36-90",
                    "MS-60-115",
                    "MS-75-120",
                    "MS-120-172",
                ], Exception(f"吊装预埋件名称异常:{self.lifting_name}")
            if not isinstance(self.lifting_name, str):
                raise Exception(f"lifting_name 数据异常:{self.lifting_name}")

        # 埋件设计--脱模预埋件
        if isinstance(self.demolding_design_mode, int):
            self.demolding_design_mode = DemoldingDesignMode(self.demolding_design_mode)
        assert isinstance(self.demolding_design_mode, DemoldingDesignMode), Exception(
            f"demolding_design_mode 传入类型错误"
        )

        if self.demolding_design_mode == DemoldingDesignMode.AUTOMATIC:
            self.pouring_way: Optional[
                PouringWay
            ] = PouringWay.VERTICAL_HORIZONTAL  # 脱模方式
            self.demolding_type: Optional[
                DemoldingType
            ] = DemoldingType.ROUNDING_HEAD  # 脱模类型
        elif self.demolding_design_mode == DemoldingDesignMode.MANUAL:
            assert self.pouring_way is not None, Exception(f"pouring_way 不能为空")
            if isinstance(self.pouring_way, int):
                self.pouring_way = PouringWay(self.pouring_way)
            if not isinstance(self.pouring_way, PouringWay):
                raise Exception(f"pouring_way 数据异常:{self.pouring_way}")

            assert self.demolding_type is not None, Exception(f"demolding_type 不能为空")
            if isinstance(self.demolding_type, int):
                self.demolding_type = DemoldingType(self.demolding_type)
            if not isinstance(self.demolding_type, DemoldingType):
                raise Exception(f"demolding_type 数据异常:{self.demolding_type}")

            assert self.demolding_position is not None, Exception(
                f"demolding_position 不能为空"
            )
            if isinstance(self.demolding_position, dict):
                self.demolding_position = DemoldingPosition(**self.demolding_position)
            if not isinstance(self.demolding_position, DemoldingPosition):
                raise Exception(f"demolding_position 数据异常:{self.demolding_position}")

            assert self.demolding_name is not None, Exception(f"demolding_name 不能为空")

            if self.demolding_type == DemoldingType.ROUNDING_HEAD:
                assert self.demolding_name in [
                    "DJ-13-120",
                    "DJ-20-140",
                    "DJ-25-170",
                    "DJ-30-210",
                    "DJ-75-300",
                ], Exception(f"脱模预埋件名称异常:{self.demolding_name}")
            elif self.demolding_type == DemoldingType.ANCHOR:
                assert self.demolding_name in [
                    "MS-6-60",
                    "MS-9-60",
                    "MS-12-70",
                    "MS-15-75",
                    "MS-24-80",
                    "MS-36-90",
                    "MS-60-115",
                    "MS-75-120",
                    "MS-120-172",
                ], Exception(f"脱模预埋件名称异常:{self.demolding_name}")
            # elif self.demolding_type == DemoldingType.LIFT_HOOK:
            #     assert self.demolding_name in ["WDG-6", "WDG-14"], Exception(
            #         f"脱模预埋件名称异常:{self.demolding_name}")
            if not isinstance(self.demolding_name, str):
                raise Exception(f"demolding_name 数据异常:{self.demolding_name}")

        # 埋件设计--栏杆预埋件
        if isinstance(self.rail_design_mode, int):
            self.rail_design_mode = RailDesignMode(self.rail_design_mode)
        assert isinstance(self.rail_design_mode, RailDesignMode), Exception(
            f"rail_design_mode 传入类型错误"
        )

        if self.rail_design_mode == RailDesignMode.MANUAL:
            assert self.rail_layout is not None, Exception(f"rail_layout 不能为空")
            if isinstance(self.rail_layout, int):
                self.rail_layout = RailLayout(self.rail_layout)
            if not isinstance(self.rail_layout, RailLayout):
                raise Exception(f"rail_layout 数据异常:{self.rail_layout}")

            assert self.rail_name is not None, Exception(f"rail_name 不能为空")
            assert self.rail_name in [
                "M1",
                "M2",
                "M3",
                "M4",
                "M5",
                "M6",
                "M7",
            ], Exception(f"栏杆预埋件名称异常:{self.rail_name}")
            if not isinstance(self.rail_name, str):
                raise Exception(f"rail_name 数据异常:{self.rail_name}")

            assert self.rail_number is not None, Exception(
                f"rail_number 不能为空{self.rail_number}"
            )
            if not isinstance(self.rail_number, list):
                raise Exception(f"rail_number 数据异常:{self.rail_number}")

            assert self.rail_number is not None, Exception(f"rail_number 不能为空")
            if isinstance(self.rail_position, dict):
                self.rail_position = RailPosition(**self.rail_position)
            if not isinstance(self.rail_position, RailPosition):
                raise Exception(f"rail_position 数据异常:{self.rail_position}")
        elif self.rail_design_mode == RailDesignMode.NO:
            pass


@dataclass
class DetailedDesign:
    """
    整个深化计算需要传递的参数
    """

    rebar_detailed: RebarDetailed = field(
        converter=RebarDetailed.converter,
    )

    construction_detailed: ConstructionDetailed = field(
        converter=ConstructionDetailed.converter
    )
    inserts_detailed: InsertsDetailed = field(converter=InsertsDetailed.converter)
    geometric_detailed: GeometricDetailed = field(converter=GeometricDetailed.converter)

    def __post_init__(self):
        if isinstance(self.construction_detailed, dict):
            self.construction_detailed = ConstructionDetailed(
                **self.construction_detailed
            )
        if isinstance(self.geometric_detailed, dict):
            self.geometric_detailed = GeometricDetailed(**self.geometric_detailed)
        if isinstance(self.rebar_detailed, dict):
            self.rebar_detailed = RebarDetailed(**self.rebar_detailed)
        if isinstance(self.inserts_detailed, dict):
            self.inserts_detailed = InsertsDetailed(**self.inserts_detailed)


def _enum_converter(data, /, *, cls):
    """
    将数据,往枚举类强制类型转换
    :param data:
    :param cls:
    :return:
    """
    return cls(data)


@dataclass
class DetailedDesignResult:
    """
    定义深化设计返回的计算结果
    """

    detailed_design: DetailedDesign = field(converter=DetailedDesign.converter)
    top_bottom_length: float  # 顶端下边长 mm
    bottom_bottom_length: float  # 底端下边长 mm
    l_total: int  # "计算跨度 mm
    h_total: int  # 楼梯总高度
    v: int  # 计算体积
    cos: float  # 梯段板与水平方向夹角余弦值
    sin: float  # 梯段板与水平方向夹角正弦值
    tan: float  # 梯段板与水平方向夹角正切值
    # 荷载计算
    gkt: float  # 自重 kN/m
    gk: float  # 恒荷标准值 kN/m
    # # 吊装预埋件
    gdk: float  # 吊装预埋件自重标准值乘以动力系数 kN/m
    gek: float  # 脱模预埋件自重标准值乘以动力系数 kN/m
    single_capacity_lifting: float  # 受压区高度 /mm
    lifting_name: str  # 吊装件型号
    lifting_parameter: Union[AnchorParameter, RoundHeadParameter]  # 吊装预埋件参数
    lifting_edge_xa: float
    lifting_edge_xb: float
    lifting_edge_xc: float
    m_lifting_ka: float
    m_lifting_kb: float
    w_lifting_a: float
    w_lifting_b: float
    sigm_lifting_cka: float
    sigm_lifting_ckb: float
    max_sigm: float
    lifting_f_tk: float
    q_k1: float
    single_capacity_demolding: float  # 脱模预埋件单承载
    demolding_name: str  # 吊装件型号
    demolding_parameter: Union[
        AnchorParameter, RoundHeadParameter
    ]  # 栏杆预埋件参数  # 脱模预埋件参数

    lifting_type: Optional[LiftingType] = field(
        converter=partial(_enum_converter, cls=LiftingType)
    )  # 吊装预埋件类型
    pouring_way: Optional[PouringWay] = field(
        converter=partial(_enum_converter, cls=PouringWay)
    )  # 浇筑形式
    demolding_type: Optional[DemoldingType] = field(
        converter=partial(_enum_converter, cls=DemoldingType)
    )  # 吊装预埋件类型
    rail_parameter: Optional[RailParameter] = field(
        converter=RailParameter.converter
    )  # 栏杆预埋件参数

    def __post_converter__(self):
        if self.lifting_type == LiftingType.ROUNDING_HEAD:
            if not isinstance(self.lifting_parameter, RoundHeadParameter):
                self.lifting_parameter = RoundHeadParameter(**self.lifting_parameter)
        elif self.lifting_type == LiftingType.ANCHOR:
            if not isinstance(self.lifting_parameter, AnchorParameter):
                self.lifting_parameter = AnchorParameter(**self.lifting_parameter)
        else:
            raise Exception(f"{self.lifting_type=} 强制类型转换后只会有两种类型")

        if self.demolding_type == DemoldingType.ROUNDING_HEAD:
            if not isinstance(self.demolding_parameter, RoundHeadParameter):
                self.demolding_parameter = RoundHeadParameter(
                    **self.demolding_parameter
                )
        elif self.demolding_type == DemoldingType.ANCHOR:
            if not isinstance(self.demolding_parameter, AnchorParameter):
                self.demolding_parameter = AnchorParameter(**self.demolding_parameter)
        else:
            raise Exception(f"{self.demolding_type=} 强制类型转换后只会有两种类型")


@dataclass
class DetailedCalculationBook:
    """
    约定楼梯计算书能够拥有的参数
    """

    structural_design_result: StructuralDesignResult = field(
        converter=StructuralDesignResult.converter
    )
    detailed_design_result: DetailedDesignResult = field(
        converter=DetailedDesignResult.converter
    )

    detailed_design: DetailedDesign = field(converter=DetailedDesign.converter)
    structural_design: StructuralDesign = field(converter=StructuralDesign.converter)

    def __post_init__(self):
        if isinstance(self.structural_design_result, dict):
            self.structural_design_result = StructuralDesignResult(
                **self.structural_design_result
            )
        if isinstance(self.structural_design, dict):
            self.structural_design = StructuralDesign(**self.structural_design)
        if isinstance(self.detailed_design_result, dict):
            self.detailed_design_result = DetailedDesignResult(
                **self.detailed_design_result
            )
        if isinstance(self.detailed_design, dict):
            self.detailed_design = DetailedDesign(**self.detailed_design)
