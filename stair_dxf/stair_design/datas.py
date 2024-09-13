"""
深化设计程序钢筋设计、构造设计、埋件设计、节点设计自动计算默认参数
Date:2022/8/19
"""
from dataclasses import dataclass
from typing import List, Any, Tuple, Dict
from pydantic import BaseModel
import json


@dataclass
class Concrete(object):
    """
    混凝土信息基类
    """

    name: str = "C40"  # 混凝土名称
    f_c: float = 19.1  # 混凝土轴心抗拉强度设计值
    f_t: float = 1.71  # 混凝土抗拉强度设计值
    a_erf: float = 1.0  # 受压区混凝土矩形应力图的应力值与混凝土轴心抗压强度设计值的比值，参考《混规》受弯构件正截面承载力
    b_ta: float = 0.8  # 矩形应力图受压区高度x月中和轴高度xc的比值，参考《混规》受弯构件正截面承载力
    f_tk: float = 2.39  # 混凝土抗拉强度标准值
    f_ck: float = 26.8  # 混凝土抗压强度标准值
    ec: float = 32500  # 混凝土轴心抗拉强度标准值


class ConcreteConfig(object):
    """
    混凝土等级相关配置信息
    """

    def __init__(self):
        self.concret_sets = {}
        concrete_30 = Concrete(
            name="C30",
            f_c=14.3,
            f_t=1.43,
            a_erf=1.0,
            b_ta=0.8,
            f_ck=20.1,
            f_tk=2.01,
            ec=30000,
        )
        concrete_35 = Concrete(
            name="C35",
            f_c=16.7,
            f_t=1.57,
            a_erf=1.0,
            b_ta=0.8,
            f_ck=23.4,
            f_tk=2.2,
            ec=31500,
        )
        concrete_40 = Concrete(
            name="C40",
            f_c=19.1,
            f_t=1.71,
            a_erf=1.0,
            b_ta=0.8,
            f_ck=26.8,
            f_tk=2.39,
            ec=32500,
        )
        concrete_45 = Concrete(
            name="C45",
            f_c=21.2,
            f_t=1.8,
            a_erf=1.0,
            b_ta=0.8,
            f_ck=29.6,
            f_tk=2.51,
            ec=33500,
        )
        concrete_50 = Concrete(
            name="C50",
            f_c=23.1,
            f_t=1.89,
            a_erf=1.0,
            b_ta=0.8,
            f_ck=32.4,
            f_tk=2.64,
            ec=34500,
        )
        concrete_55 = Concrete(
            name="C55",
            f_c=25.3,
            f_t=1.96,
            a_erf=0.99,
            b_ta=0.79,
            f_ck=35.5,
            f_tk=2.74,
            ec=35500,
        )
        concrete_60 = Concrete(
            name="C60",
            f_c=27.5,
            f_t=2.04,
            a_erf=0.98,
            b_ta=0.78,
            f_ck=38.5,
            f_tk=2.85,
            ec=36000,
        )
        concrete_65 = Concrete(
            name="C65",
            f_c=29.7,
            f_t=2.09,
            a_erf=0.97,
            b_ta=0.77,
            f_ck=41.5,
            f_tk=2.93,
            ec=36500,
        )
        concrete_70 = Concrete(
            name="C70",
            f_c=31.8,
            f_t=2.14,
            a_erf=0.96,
            b_ta=0.76,
            f_ck=44.5,
            f_tk=2.99,
            ec=37000,
        )
        concrete_75 = Concrete(
            name="C75",
            f_c=33.8,
            f_t=2.18,
            a_erf=0.95,
            b_ta=0.75,
            f_ck=47.4,
            f_tk=3.05,
            ec=37500,
        )
        concrete_80 = Concrete(
            name="C80",
            f_c=35.9,
            f_t=2.22,
            a_erf=0.94,
            b_ta=0.74,
            f_ck=50.2,
            f_tk=3.11,
            ec=38000,
        )
        self.concret_sets[concrete_30.name] = concrete_30
        self.concret_sets[concrete_35.name] = concrete_35
        self.concret_sets[concrete_40.name] = concrete_40
        self.concret_sets[concrete_45.name] = concrete_45
        self.concret_sets[concrete_50.name] = concrete_50
        self.concret_sets[concrete_55.name] = concrete_55
        self.concret_sets[concrete_60.name] = concrete_60
        self.concret_sets[concrete_65.name] = concrete_65
        self.concret_sets[concrete_70.name] = concrete_70
        self.concret_sets[concrete_75.name] = concrete_75
        self.concret_sets[concrete_80.name] = concrete_80

    def choose(self, name: str) -> Concrete:
        """
        根据混凝土类型选择相应数据
        :param name:
        :return:
        """
        return self.concret_sets[name]


@dataclass
class RebarStrength(object):
    name: str = "HRB400"  # 钢筋牌号
    symbol: str = "f"  # 几级钢符号表示，d、D、f、F分别表示符号圈1、圈2、圈3、圈4，即一级钢、二级钢、三级钢、四级钢
    f_y: float = 360.0  # 抗拉强度设计值 N/mm2
    f1_y: float = 360.0  # 抗压强度设计值 N/mm2
    d_min: float = 6  # 该型号钢筋最小直径 mm
    d_max: float = 50  # 该型号钢筋最大直径 mm
    fyk: float = 400  # 屈服强度标准值  N/mm2
    fstk: float = 540  # 极限强度标准值  N/mm2
    es: float = 200000  # 钢筋弹性模量  N/mm2
    level: int = 3  # 钢筋强度等级


class RebarConfig(object):
    """
    钢筋等级配置
    """

    def __init__(self):
        self.rebar_sets = {}
        rebar_hpb300 = RebarStrength(
            name="HPB300",
            symbol="d",
            f_y=270.0,
            f1_y=270.0,
            d_min=6.0,
            d_max=22.0,
            fyk=300.0,
            fstk=420.0,
            es=210000.0,
            level=1,
        )
        rebar_hrb335 = RebarStrength(
            name="HRB335",
            symbol="D",
            f_y=300.0,
            f1_y=300.0,
            d_min=6.0,
            d_max=50.0,
            fyk=335.0,
            fstk=455.0,
            es=200000.0,
            level=2,
        )
        rebar_hrbf335 = RebarStrength(
            name="HRBF335",
            symbol="D",
            f_y=300.0,
            f1_y=300.0,
            d_min=6.0,
            d_max=50.0,
            fyk=335.0,
            fstk=455.0,
            es=200000.0,
            level=2,
        )
        rebar_hrb400 = RebarStrength(
            name="HRB400",
            symbol="f",
            f_y=360.0,
            f1_y=360.0,
            d_min=6.0,
            d_max=50.0,
            fyk=400.0,
            fstk=540.0,
            es=200000.0,
            level=3,
        )
        rebar_hrbf400 = RebarStrength(
            name="HRBF400",
            symbol="f",
            f_y=360.0,
            f1_y=360.0,
            d_min=6.0,
            d_max=50.0,
            fyk=400.0,
            fstk=540.0,
            es=200000.0,
            level=3,
        )
        rebar_hrb500 = RebarStrength(
            name="HRB500",
            symbol="F",
            f_y=435.0,
            f1_y=410.0,
            d_min=6.0,
            d_max=50.0,
            fyk=500.0,
            fstk=630.0,
            es=200000.0,
            level=4,
        )
        rebar_hrbf500 = RebarStrength(
            name="HRBF500",
            symbol="F",
            f_y=435.0,
            f1_y=410.0,
            d_min=6.0,
            d_max=50.0,
            fyk=500.0,
            fstk=630.0,
            es=200000.0,
            level=4,
        )
        self.rebar_sets[rebar_hpb300.name] = rebar_hpb300
        self.rebar_sets[rebar_hrb335.name] = rebar_hrb335
        self.rebar_sets[rebar_hrbf335.name] = rebar_hrbf335
        self.rebar_sets[rebar_hrb400.name] = rebar_hrb400
        self.rebar_sets[rebar_hrbf400.name] = rebar_hrbf400
        self.rebar_sets[rebar_hrb500.name] = rebar_hrb500
        self.rebar_sets[rebar_hrbf500.name] = rebar_hrbf500

    def choose(self, name: str) -> RebarStrength:
        """
        根据钢筋强度等级选择相应钢筋配置信息
        :param name: 钢筋强度等级
        :return:
        """
        return self.rebar_sets[name]


@dataclass
class RelativeLimitZone(object):
    """
    相对受压区高度
    """

    ksi_b: float
    alph_s_max: float


class RelativeLimitZoneDataSets(object):
    def __init__(self) -> None:
        self.datasets = []

    def choose(self, rebar_name: str, concrete_name: str) -> RelativeLimitZone:
        """
        根据输入的钢筋名称和混凝土强度等级名称选择相应的相对界限受压区高度ksi_b和截面最大抵抗矩系数alph_s_max
        :param rebar_name: 钢筋强度等级名称
        :param concrete_name: 混凝土强度等级名称
        :return:
        """
        ksi_b = 0.576  # 随机初始化
        alph_s_max = 0.410  # 随机初始化
        if (
            concrete_name == "C30"
            or concrete_name == "C35"
            or concrete_name == "C40"
            or concrete_name == "C45"
            or concrete_name == "C50"
        ):
            if rebar_name == "HPB300":
                ksi_b = 0.576
                alph_s_max = 0.410
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.550
                alph_s_max = 0.399
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.518
                alph_s_max = 0.384
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.482
                alph_s_max = 0.366
            else:
                print("输入有误！")
        elif concrete_name == "C55":
            if rebar_name == "HPB300":
                ksi_b = 0.566
                alph_s_max = 0.406
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.541
                alph_s_max = 0.394
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.508
                alph_s_max = 0.379
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.473
                alph_s_max = 0.361
            else:
                print("输入有误！")
        elif concrete_name == "C60":
            if rebar_name == "HPB300":
                ksi_b = 0.557
                alph_s_max = 0.402
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.531
                alph_s_max = 0.39
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.499
                alph_s_max = 0.375
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.464
                alph_s_max = 0.356
            else:
                print("输入有误！")
        elif concrete_name == "C65":
            if rebar_name == "HPB300":
                ksi_b = 0.547
                alph_s_max = 0.397
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.522
                alph_s_max = 0.386
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.49
                alph_s_max = 0.37
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.456
                alph_s_max = 0.352
            else:
                print("输入有误！")
        elif concrete_name == "C70":
            if rebar_name == "HPB300":
                ksi_b = 0.537
                alph_s_max = 0.393
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.512
                alph_s_max = 0.381
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.481
                alph_s_max = 0.365
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.447
                alph_s_max = 0.347
            else:
                print("输入有误！")
        elif concrete_name == "C75":
            if rebar_name == "HPB300":
                ksi_b = 0.528
                alph_s_max = 0.388
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.503
                alph_s_max = 0.376
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.472
                alph_s_max = 0.36
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.438
                alph_s_max = 0.342
            else:
                print("输入有误！")
        elif concrete_name == "C80":
            if rebar_name == "HPB300":
                ksi_b = 0.518
                alph_s_max = 0.384
            elif rebar_name == "HRB335" or rebar_name == "HRBF335":
                ksi_b = 0.493
                alph_s_max = 0.371
            elif rebar_name == "HRB400" or rebar_name == "HRBF400":
                ksi_b = 0.463
                alph_s_max = 0.356
            elif rebar_name == "HRB500" or rebar_name == "HRBF500":
                ksi_b = 0.429
                alph_s_max = 0.337
            else:
                print("输入有误！")

        return RelativeLimitZone(ksi_b=ksi_b, alph_s_max=alph_s_max)


@dataclass
class Rebar(object):
    """
    钢筋基本信息：直径，间距
    """

    diameter: int  # 钢筋的直径
    spacing: int  # 钢筋的间距
    edge: int  # 钢筋的边距


@dataclass
class Point(object):
    """
    点的信息：x、y、z
    """

    x: float
    y: float
    z: float


class Vertex(BaseModel):
    x: float
    y: float
    z: float


@dataclass
class HoleReinRebar(object):
    """
    销键或孔洞加强筋的长度和半径
    """

    length: int = 280
    radius: int = 60


class StructRebarLimit(object):
    """
    结构设计三类钢筋的直径和间距限值：下部纵筋的最小直径和最大间距，上部纵筋的最小直径和最大间距，中部分布筋的最小直径和最大间距
    """

    # 结构设计阶段三类钢筋直径和间距限值
    min_dia_bottom = 10  # mm 下部纵筋最小直径
    max_spac_bottom = 200  # mm 下部纵筋最大间距
    min_dia_top = 8  # mm 上部纵筋最小直径
    max_spac_top = 200  # mm 上部纵筋最大间距
    min_dia_dist = 8  # mm 中部分布筋最小直径
    max_spac_dist = 250  # mm 中部分布筋最大间距


@dataclass
class HoistRebar(object):
    """
    吊点加强纵筋边距
    """

    edge: int = 150  # 钢筋两端边距
    top_edge: int = 10  # 钢筋距离顶端企口下边缘的距离,防腐


class RebarData(object):
    """
    深化设计钢筋设计各类钢筋的直径和间距计算类，后期可在此类中增加判断逻辑,也可以增加钢筋边距要求。
    深化钢筋名称及对应编号：
        1号下部纵筋，钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        2号上部纵筋，钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        3号中部分布筋，钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        4号底端边缘纵筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        5号顶端边缘纵筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        6号底端边缘箍筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[[d,l,e]
        7号销键加强筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,None,e],为了数据格式的统一，None表示该类钢筋不需要该属性
        8a号吊点加强纵筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,None,e]
        8b号吊点加强点筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,None,e]
        9号顶端边缘箍筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,l,e]
        10号上部边缘加强筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,None,e]
        11号下部边缘加强筋：钢筋直径d、钢筋间距l、钢筋的边距e；表示方式：[d,None,e]
    """

    bottom_longitudinal_rebar = Rebar(diameter=10, spacing=100, edge=40)
    top_longitudinal_rebar = Rebar(diameter=10, spacing=100, edge=35)
    mid_distribution_rebar = Rebar(diameter=10, spacing=100, edge=40)
    bottom_edge_longitudinal_rebar = Rebar(diameter=12, spacing=210, edge=40)  # 底端边缘纵筋
    top_edge_longitudinal_rebar = Rebar(diameter=12, spacing=210, edge=40)  # 顶端边缘纵筋
    bottom_edge_stirrup = Rebar(diameter=8, spacing=130, edge=60)  # 底端边缘箍筋
    top_edge_stirrup = Rebar(diameter=8, spacing=130, edge=60)  # 顶端边缘箍筋
    hole_reinforce_rebar = Rebar(
        diameter=10, spacing=0, edge=50
    )  # 销键加强筋或孔洞加强筋  zc09/5修改了孔洞加强筋
    hoisting_reinforce_longitudinal_rebar = Rebar(
        diameter=12, spacing=0, edge=0
    )  # 吊点加强纵筋
    hoisting_reinforce_point_rebar = Rebar(diameter=12, spacing=0, edge=0)  # 吊点加强点筋
    top_edge_reinforce_rebar = Rebar(diameter=12, spacing=0, edge=65)  # 上部边缘加强筋
    bottom_edge_reinforce_rebar = Rebar(diameter=12, spacing=0, edge=65)  # 下部边缘加强筋


@dataclass
class SlidingHinge(object):
    """
    滑动铰支座孔洞几何参数:
        c1--顶部孔洞直径，d1--中部内侧孔洞直径，e1---中部外侧孔洞直径，
        f1--底部孔洞直径，h1--顶部到中部变截面高度
    """

    c1: float
    d1: float
    e1: float
    f1: float
    h1: float


@dataclass
class FixedHinge(object):
    """
    固定铰支座孔洞几何参数:c2--顶部孔洞直径，d2--底部孔洞直径
    """

    c2: float
    d2: float


@dataclass
class ConnectionHoleLocation(object):
    """
    四个连接孔洞定位参数：a1,b1,a2,b2,a3,b3,a4,b4
    """

    a1: float
    b1: float
    a2: float
    b2: float
    a3: float
    b3: float
    a4: float
    b4: float


class ConnectionHoleInformation(object):
    """
    四个连接孔洞的形状和定位信息
    """

    top_connection_type = FixedHinge(c2=60, d2=50)  # 固定铰支座
    bottom_connection_type = SlidingHinge(c1=70, d1=55, e1=65, f1=50, h1=50)  # 滑动铰支座
    hole_location = ConnectionHoleLocation(
        a1=100, b1=300, a2=100, b2=300, a3=100, b3=300, a4=100, b4=300
    )


@dataclass
class StepSlotShape(object):
    """
    防滑槽几何参数
    """

    a: float
    b: float
    c: float
    d: float
    e: float


@dataclass
class StepSlotLocation(object):
    """
    防滑槽定位参数
    """

    c1: float
    c2: float
    c3: float


class StepSlot(object):
    """
    防滑槽数据：几何参数和定位参数
    """

    step_slot_shape = StepSlotShape(a=9, b=6, c=15, d=8, e=6)  # 防滑槽数据
    step_slot_edge = 10  # 防滑槽边距
    step_slot_location = StepSlotLocation(c1=50, c2=50, c3=21)  # 防滑槽定位信息


@dataclass
class TrapezoidWaterDripShape(object):
    """
    梯形线槽
    """

    a: float
    b: float
    c: float


@dataclass
class SemicircleWaterDripShape(object):
    """
    半圆形滴水线槽
    """

    a: float
    b: float


@dataclass
class WaterDripLocation(object):
    """
    滴水线槽定位
    """

    a1: float
    a2: float
    a3: float


class WaterDripInformation(object):
    """
    滴水线槽信息
    """

    water_drip_shape_t = TrapezoidWaterDripShape(a=5, b=10, c=8)  # 梯形滴水线槽数据
    water_drip_shape_c = SemicircleWaterDripShape(a=10, b=8)  # 半圆形滴水线槽数据
    water_drip_location = WaterDripLocation(a1=15, a2=15, a3=20)  # 滴水线槽位置数据


@dataclass
class RoundHeadHangingNailShape(object):
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


class RoundHeadHangingNailInformation(object):
    """
    圆头吊钉数据库
    """

    rounding_head_sets = {}  # 圆头吊钉数据集合
    rounding_head_names = []  # 圆头吊钉名称集合
    # 创建圆钉预埋件DJ-13-120
    single_dj_13 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-13-120",
        abbreviation="DJ",
        capacity=1.3,
        length=120,
        top_diameter=19,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=10,
        middle_height=94,
        bottom_adjacent_height=3,
        bottom_diameter=25,
        bottom_height=10,
        radius=30,
    )
    rounding_head_sets["DJ-13-120"] = single_dj_13
    # 创建圆钉预埋件DJ-20-140
    single_dj_20 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-20-140",
        abbreviation="DJ",
        capacity=2.0,
        length=140,
        top_diameter=26,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=14,
        middle_height=114,
        bottom_adjacent_height=3,
        bottom_diameter=35,
        bottom_height=10,
        radius=37,
    )
    rounding_head_sets["DJ-20-140"] = single_dj_20
    # 创建圆钉预埋件DJ-25-170
    single_dj_25 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-25-170",
        abbreviation="DJ",
        capacity=2.5,
        length=170,
        top_diameter=26,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=14,
        middle_height=144,
        bottom_adjacent_height=3,
        bottom_diameter=35,
        bottom_height=10,
        radius=37,
    )
    rounding_head_sets["DJ-25-170"] = single_dj_25
    # 创建圆钉预埋件DJ-30-210
    single_dj_30 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-30-210",
        abbreviation="DJ",
        capacity=4.0,
        length=210,
        top_diameter=36,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=18,
        middle_height=184,
        bottom_adjacent_height=3,
        bottom_diameter=45,
        bottom_height=10,
        radius=47,
    )
    rounding_head_sets["DJ-30-210"] = single_dj_30
    # 创建圆钉预埋件DJ-50-240
    single_dj_50 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-50-240",
        abbreviation="DJ",
        capacity=5.0,
        length=240,
        top_diameter=36,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=20,
        middle_height=214,
        bottom_adjacent_height=3,
        bottom_diameter=50,
        bottom_height=10,
        radius=47,
    )
    rounding_head_sets["DJ-50-240"] = single_dj_50
    # 创建圆钉预埋件DJ-75-300
    single_dj_75 = RoundHeadHangingNailShape(
        type="圆头吊钉",
        factory="现代营造",
        name="DJ-75-300",
        abbreviation="DJ",
        capacity=7.5,
        length=300,
        top_diameter=46,
        top_height=10,
        top_adjacent_height=3,
        middle_diameter=24,
        middle_height=274,
        bottom_adjacent_height=3,
        bottom_diameter=60,
        bottom_height=10,
        radius=59,
    )
    rounding_head_sets["DJ-75-300"] = single_dj_75
    for key in rounding_head_sets.keys():
        rounding_head_names.append(key)


@dataclass
class LadderBeamAndSlab(object):
    """
    梯梁和梯板形状
    """

    slab_thickness: float  # 板厚度
    slab_length: float  # 板长度
    slab_width: float  # 板宽度
    beam_height: float  # 梁高度
    beam_area_width: float  # 梁部分宽
    beam_width: float  # 梁宽度
    beam_length: float  #


@dataclass
class SingleConnectElementShape(object):
    """
    节点连接件形状数据
    """

    edge_t: float  # 钢筋顶部边距
    edge_m: float  # 螺母距离钢筋底部距离
    nut_thickness: float  # 螺母的厚度
    nut_diameter: float  # 螺母内部直径
    nut_width: float  # 螺母总宽度
    shim_thickness: float  # 垫片厚度
    shim_width: float  # 垫片宽度/边长
    rebar_anchor_diameter: float  # 锚固钢筋直径
    rebar_anchor_a: float  # 锚固钢筋深入段长度
    rebar_anchor_b: float  # 锚固钢筋水平段长度


class ConnectElementShapeInfo(object):
    """
    所有连接预埋件部件形状数据
    """

    connect_info = {}  # 连接形状信息
    bottom_connect_element = SingleConnectElementShape(
        edge_t=22,
        edge_m=14,
        nut_thickness=6.77,
        nut_diameter=16,
        nut_width=28,
        shim_thickness=7.23,
        shim_width=55,
        rebar_anchor_diameter=16,
        rebar_anchor_a=156,
        rebar_anchor_b=260,
    )
    top_connect_element = SingleConnectElementShape(
        edge_t=22,
        edge_m=14,
        nut_thickness=6.77,
        nut_diameter=16,
        nut_width=28,
        shim_thickness=7.23,
        shim_width=55,
        rebar_anchor_diameter=16,
        rebar_anchor_a=156,
        rebar_anchor_b=260,
    )  # TODO 此处的edge_m,edge_t,nut_thickness,shim_thickness与滑动铰支座顶部梯形高度有几何关系
    connect_info["top_shape"] = top_connect_element
    connect_info["bottom_shape"] = bottom_connect_element


class LadderBeamAndSlabInformation(object):
    """
    楼梯梁和板信息
    """

    ladder_info_sets = {}  # 楼梯信息集合
    bottom_plate = LadderBeamAndSlab(
        slab_thickness=120,
        slab_length=1200,
        slab_width=2800,
        beam_height=500,
        beam_area_width=220,
        beam_width=420,
        beam_length=2800,
    )
    top_plate = LadderBeamAndSlab(
        slab_thickness=120,
        slab_length=1200,
        slab_width=2800,
        beam_height=500,
        beam_area_width=220,
        beam_width=420,
        beam_length=2800,
    )
    ladder_info_sets["top"] = top_plate  # 顶部平台板和平台梁尺寸信息
    ladder_info_sets["bottom"] = bottom_plate  # 底部平台板和平台梁尺寸信息


class LadderDoubleJointInformation(object):
    """
    楼梯两侧接缝信息
    """

    left_joint_thick = 20
    top_edge_thick = 15
    top_mid_thick = 25
    bottom_mid_thick = 30
    bottom_edge_thick = 18
    wall_edge_thick = 20  # 楼梯纵向与墙体接缝


@dataclass
class EmbeddedAnchor(object):
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


class EmbeddedAnchorInformation(object):
    """
    预埋锚栓信息
    """

    embedded_anchor_sets = {}
    embedded_anchor_names = []  # 脱模埋件名称列表
    # 创建MS-6-60预埋锚栓
    single_anchor_MS_6 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-6-60",
        abbreviation="MS",
        capacity=0.6,
        m_diameter=20,
        m_length=20,
        anchor_name="M6x1.0",
        e_diameter=6,
        g=16,
        b=5,
        o_diameter=10,
        length=40,
        s_diameter=4,
        l_p=40,
        a=6,
    )
    embedded_anchor_sets["MS-6-60"] = single_anchor_MS_6

    # 创建MS-9-60预埋锚栓
    single_anchor_MS_9 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-9-60",
        abbreviation="MS",
        capacity=0.9,
        m_diameter=22,
        m_length=20,
        anchor_name="M8x1.25",
        e_diameter=8,
        g=16,
        b=5,
        o_diameter=12,
        length=40,
        s_diameter=6,
        l_p=40,
        a=8,
    )
    embedded_anchor_sets["MS-9-60"] = single_anchor_MS_9

    # 创建MS-12-70预埋锚栓
    single_anchor_MS_12 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-12-70",
        abbreviation="MS",
        capacity=1.2,
        m_diameter=26,
        m_length=20,
        anchor_name="M10x1.5",
        e_diameter=10,
        g=22,
        b=5,
        o_diameter=16,
        length=50,
        s_diameter=8,
        l_p=50,
        a=9,
    )
    embedded_anchor_sets["MS-12-70"] = single_anchor_MS_12

    # 创建MS-15-75预埋锚栓
    single_anchor_MS_15 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-15-75",
        abbreviation="MS",
        capacity=1.5,
        m_diameter=28,
        m_length=20,
        anchor_name="M12x1.75",
        e_diameter=12,
        g=25,
        b=5,
        o_diameter=18,
        length=55,
        s_diameter=8,
        l_p=60,
        a=10,
    )
    embedded_anchor_sets["MS-15-75"] = single_anchor_MS_15

    # 创建MS-24-80预埋锚栓
    single_anchor_MS_24 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-24-80",
        abbreviation="MS",
        capacity=2.4,
        m_diameter=30,
        m_length=20,
        anchor_name="M14x2",
        e_diameter=14,
        g=28,
        b=5,
        o_diameter=20,
        length=60,
        s_diameter=10,
        l_p=100,
        a=10,
    )
    embedded_anchor_sets["MS-24-80"] = single_anchor_MS_24

    # 创建MS-36-90预埋锚栓
    single_anchor_MS_36 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-36-90",
        abbreviation="MS",
        capacity=3.6,
        m_diameter=32,
        m_length=20,
        anchor_name="M16x2.0",
        e_diameter=16,
        g=32,
        b=5,
        o_diameter=22,
        length=70,
        s_diameter=10,
        l_p=100,
        a=11,
    )
    embedded_anchor_sets["MS-36-90"] = single_anchor_MS_36

    # 创建MS-60-115预埋锚栓
    single_anchor_MS_60 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-60-115",
        abbreviation="MS",
        capacity=6.0,
        m_diameter=30,
        m_length=35,
        anchor_name="M20x2.5",
        e_diameter=20,
        g=38,
        b=5,
        o_diameter=25,
        length=80,
        s_diameter=12,
        l_p=100,
        a=12,
    )
    embedded_anchor_sets["MS-60-115"] = single_anchor_MS_60

    # 创建MS-75-120预埋锚栓
    single_anchor_MS_75 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-75-120",
        abbreviation="MS",
        capacity=7.5,
        m_diameter=42,
        m_length=20,
        anchor_name="M24x3.0",
        e_diameter=24,
        g=50,
        b=5,
        o_diameter=32,
        length=100,
        s_diameter=12,
        l_p=100,
        a=12,
    )
    embedded_anchor_sets["MS-75-120"] = single_anchor_MS_75

    # 创建MS-120-172预埋锚栓
    single_anchor_MS_120 = EmbeddedAnchor(
        type="预埋锚栓",
        factory="现代营造",
        name="MS-120-172",
        abbreviation="MS",
        capacity=12.0,
        m_diameter=30,
        m_length=52,
        anchor_name="M30x3.5",
        e_diameter=30,
        g=60,
        b=5,
        o_diameter=42,
        length=120,
        s_diameter=22,
        l_p=120,
        a=12,
    )
    embedded_anchor_sets["MS-120-172"] = single_anchor_MS_120

    for key in embedded_anchor_sets.keys():
        embedded_anchor_names.append(key)


@dataclass
class LiftingHook(object):
    """
    吊钩预埋件数据
    """

    type: str  # 类型
    name: str  # 名称
    abbreviation: str  # 编号前缀
    material: str  # 材质
    diameter: float  # 钢筋直径
    top_radius: float  # 顶部半径
    top_height: float  # 顶部高度
    bottom_radius: float  # 底部半径
    bottom_height: float  # 底部高度
    m_height: float  # 弯钩高度
    a1: int  # 企口长边内边长
    b1: int  # 企口长边外边长
    a2: int  # 企口短边内边长
    b2: int  # 企口短边外边长
    d: int  # 企口深度


class LiftingHookInformation(object):
    """
    吊钩信息
    """

    lift_hook_sets = {}  # 吊钩数据集合
    lift_hook_names = []  # 吊钩名称集
    # 创建弯吊钩WDG-6
    single_lift_hook_wdg_6 = LiftingHook(
        type="弯吊钩",
        name="WDG-6",
        abbreviation="WDG",
        material="HPB300",
        diameter=6,
        top_radius=30,
        top_height=80,
        bottom_radius=20,
        bottom_height=300,
        m_height=40,
        a1=80,
        b1=100,
        a2=40,
        b2=60,
        d=20,
    )
    lift_hook_sets["WDG-6"] = single_lift_hook_wdg_6

    # 创建弯吊钩WDG-14
    single_lift_hook_wdg_14 = LiftingHook(
        type="弯吊钩",
        name="WDG-14",
        abbreviation="WDG",
        material="HPB300",
        diameter=14,
        top_radius=40,
        top_height=30,
        bottom_radius=30,
        bottom_height=400,
        m_height=40,
        a1=100,
        b1=120,
        a2=40,
        b2=60,
        d=20,
    )
    lift_hook_sets["WDG-14"] = single_lift_hook_wdg_14
    for key in lift_hook_sets.keys():
        lift_hook_names.append(key)


@dataclass
class RailingShape(object):
    """
    栏杆预埋件数据:name、a、b、c、d、t、fi，注：栏杆埋入企口的深度和边长在此明确
    """

    name: str = "M"
    a: int = 0  # 纵向边长--y方向
    b: int = 0  # 横向边长--x方向
    c: int = 0  # U型钢筋边距--x、y两个方向
    d: int = 0  # U型钢筋竖向轴心距--z方向
    t: int = 0  # 焊板厚度
    fi: int = 0  # 钢筋直径
    depth: int = 15  # 栏杆埋入深度
    length: int = 5  # 延伸底边长度


@dataclass
class RailingLocation(object):
    """
    栏杆预埋件定位信息：a、b
    """

    a: float = 15  # 横向边距
    b: float = 140  # 每阶踏步纵向距离


@dataclass
class RailingEmbeddedRabbet(object):
    """
    栏杆预埋件企口信息：height,extend_width
    """

    height: float = 5  # 企口高度
    extend_width: float = 5  # 企口边缘延伸宽度


class RailingInformation(object):
    """
    栏杆预埋件信息：栏杆几何造型及定位参数
    """

    railing_names = ["M1", "M2", "M3", "M4", "M5", "M6", "M7"]  # 栏杆预埋件
    railing_sets = {}
    # 制作M1栏杆预埋件
    m1_shape = RailingShape(
        "M1", a=90, b=90, c=20, d=100, t=6, fi=8, depth=15, length=5
    )
    railing_sets["M1"] = m1_shape
    # 制作M2栏杆预埋件
    m2_shape = RailingShape(
        "M2", a=100, b=100, c=20, d=100, t=6, fi=8, depth=15, length=5
    )
    railing_sets["M2"] = m2_shape
    # 制作M3栏杆预埋件
    m3_shape = RailingShape(
        "M3", a=100, b=100, c=20, d=120, t=6, fi=10, depth=15, length=5
    )
    railing_sets["M3"] = m3_shape
    # 制作M4栏杆预埋件
    m4_shape = RailingShape(
        "M4", a=110, b=110, c=20, d=120, t=6, fi=10, depth=15, length=5
    )
    railing_sets["M4"] = m4_shape
    # 制作M5栏杆预埋件
    m5_shape = RailingShape(
        "M5", a=120, b=120, c=25, d=140, t=8, fi=12, depth=15, length=5
    )
    railing_sets["M5"] = m5_shape
    # 制作M6栏杆预埋筋
    m6_shape = RailingShape(
        "M6", a=180, b=90, c=30, d=150, t=8, fi=12, depth=15, length=5
    )
    railing_sets["M6"] = m6_shape
    # 制作M7栏杆预埋件
    m7_shape = RailingShape(
        "M7", a=240, b=90, c=30, d=150, t=6, fi=8, depth=15, length=5
    )
    railing_sets["M7"] = m7_shape


@dataclass
class NodeShape(object):
    """
    节点形状尺寸：a、b、b
    """

    a: int
    b: int
    c: int


class NodeInformation(object):
    """
    各节点形状数据库：固定铰节点、滑动铰节点
    """

    fix_node = NodeShape(a=30, b=50, c=20)  # 固定铰形状
    slide_shape = NodeShape(a=30, b=50, c=20)  # 滑动铰形状


# 楼梯几何实体的形成


@dataclass
class BoundingBox(object):
    """
    包围框
    """

    maxp: Point
    minp: Point


@dataclass
class StretchG(object):
    """
    单个几何体的特定格式
    """

    stretch: Point  # 拉伸方向
    length: float  # 拉伸长度
    Points: List[Point]  # 点信息


@dataclass
class SlabG(object):
    """
    楼梯几何体
    """

    top_ear: StretchG = None
    foot_ear: StretchG = None
    body: StretchG = None
