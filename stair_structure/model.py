"""
Date&Time           2022/12/3 11:26
Author              Chengran Xu
"""
from typing import Optional, Dict, Any

from dataclasses import asdict

from converter_dataclass import (
    dataclass_with_converter as dataclass,
    field_with_converter as field,
)
from stair_structure.config import (
    CONCRETE_CONFIG_DICT,
    REBAR_GRADE_CONFIG,
    REBAR_NAME_CONFIG,
    RELATIVE_LIMIT_ZONE,
)


@dataclass(frozen=True)
class ConcreteParameter:
    """
    混凝土详细配置参数获取
    """

    name: str
    f_c: float
    f_t: float
    a_erf: float
    b_ta: float
    f_ck: float
    f_tk: float
    ec: float
    grade: float

    @classmethod
    def by_grade(cls, concrete_grade: int) -> Optional["ConcreteParameter"]:
        """
        根据混凝土强度等级,查找混凝土相关参数
        """
        data = CONCRETE_CONFIG_DICT.get(concrete_grade, None)
        if data is None:
            return data
        else:
            return ConcreteParameter(**data)


@dataclass(frozen=True)
class RelativeLimitZone:
    """
    相对受压区高度
    """

    ksi_b: float

    @classmethod
    def by_rebar_concrete(
        cls, rebar_grade: str, concrete_grade: str
    ) -> Optional["RelativeLimitZone"]:
        """
        根据钢筋强度和查找钢筋配置
        """
        rebar_concrete = rebar_grade + concrete_grade
        data: dict = RELATIVE_LIMIT_ZONE.get(rebar_concrete, None)
        if data is None:
            return data
        else:
            return RelativeLimitZone(**data)


@dataclass(frozen=True)
class SteelParameter:
    """
    钢筋强度相关配置
    """

    f_y: int
    f1_y: int
    d_min: int
    d_max: int
    fyk: int
    fstk: int
    es: int

    @classmethod
    def by_grade(cls, rebar_grade: int) -> Optional["SteelParameter"]:
        """
        根据钢筋强度查找钢筋配置
        """
        data: dict = REBAR_GRADE_CONFIG.get(rebar_grade, None)
        if data is None:
            return data
        else:
            return SteelParameter(**data)


@dataclass
class RebarParameter:
    name: str
    grade: int
    symbol: str
    steel: SteelParameter

    @classmethod
    def by_name(cls, name: str) -> Optional["RebarParameter"]:
        name_config: Optional[dict] = REBAR_NAME_CONFIG.get(name, None)
        if name_config is None:
            return None
        steel = SteelParameter.by_grade(name_config["grade"])  # dict

        return RebarParameter(name=name, steel=steel, **name_config)


@dataclass
class StairID:
    # 楼梯的编号
    project_ID: str
    stair_ID: str


@dataclass
class Material:
    """
    材质相关的设置
    """

    # 钢筋等级
    rebar_name: str
    # 混凝土等级,对应C30 等
    concrete_grade: int = field(default=30)

    def __post_init__(self):
        assert self.concrete_grade in [30, 35, 40, 45, 50], Exception(
            f"混凝土等级异常:{self.concrete_grade}"
        )


@dataclass
class Construction:
    """
    构造相关
    """

    # 保护层厚度,单位毫米
    concrete_cover_thickness: int = field(default=20)
    # 纵顶受力钢筋边距,单位毫米
    longitudinal_top_rebar_distance: int = field(default=20)


@dataclass
class Geometric:
    """
    几何参数,可以用于唯一确定楼梯几何外形
    长度单位均为毫米
    """

    # 楼梯高度
    height: int
    # 梯段板厚度
    thickness: int
    # 梯段板宽度
    width: int
    # 净跨
    clear_span: int
    # 顶端上边长
    top_top_length: int
    # 底端上边长
    bottom_top_length: int
    # 踏步数
    steps_number: int


@dataclass
class LoadData:
    """
    荷载相关设定
    """

    # 可变荷载 kN/m^2
    live_load: float
    # 栏杆荷载 kN/m
    railing_load: float
    # 永久荷载分项系数 无单位
    permanent_load_partial_factor: float
    # 可变荷载分项系数 无单位
    live_load_load_partial_factor: float
    # 准永久值系数 无单位
    quasi_permanent_factor: float
    # 组合值系数 无单位
    combined_factor: float
    # 钢筋混凝土容重 kN/m^3
    reinforced_concrete_bulk_density: float


@dataclass
class LimitSetting:
    # 裂缝限制 mm
    crack: float


@dataclass
class StructuralDesign:
    """
    整个结构计算需要传递的参数
    """

    material: Material = field(converter=Material.converter)
    construction: Construction = field(converter=Construction.converter)
    geometric: Geometric = field(converter=Geometric.converter)
    load_data: LoadData = field(converter=LoadData.converter)
    limit_setting: LimitSetting = field(converter=LimitSetting.converter)

    stair_id: StairID = field(converter=StairID.converter)

    def __post_init__(self):
        if isinstance(self.stair_id, dict):
            self.stair_id = StairID(**self.stair_id)

        if isinstance(self.material, dict):
            self.material = Material(**self.material)

        if isinstance(self.construction, dict):
            self.construction = Construction(**self.construction)

        if isinstance(self.geometric, dict):
            self.geometric = Geometric(**self.geometric)

        if isinstance(self.load_data, dict):
            self.load_data = LoadData(**self.load_data)

        if isinstance(self.limit_setting, dict):
            self.limit_setting = LimitSetting(**self.limit_setting)


@dataclass
class StructuralDesignResult:
    """
    定义结构计算返回的计算结果

    用于对外部使用
    """

    # start 楼梯几何参数
    steps_h: float  # 踏步高度 mm
    steps_b: float  # 踏步宽度 mm
    l0: int  # "计算跨度 mm
    cos: float  # 梯段板与水平方向夹角余弦值
    # 荷载计算
    gkt: float  # 自重 kN/m
    gk: float  # 恒荷标准值 kN/m
    png: float  # Pn(G) 恒荷控制 kN/m
    pnl: float  # Pn(L) 活荷控制 kN/m
    pm: float  # Pm 荷载设计值kN/m
    # 正截面受弯承载力
    m_max: float  # "截面最大弯矩 kN*m

    h0: float  # 受压区高度 /mm
    alpha_s: float  # alpha_s
    ksi: float  # "相对受压区高度变换系数ξ
    ksi_b: float  #
    ksi_status: bool  # "状态 ξ相对于ξb | 界限相对受压区高度变换系数
    p_c_min: float  # 纵向受力钢筋最小配筋率 ρmin
    p_c: float  # "受弯承载力计算配筋率
    # 钢筋计算结果
    as_1: float  # "受弯承载力配筋计算面积  As /mm^2
    as_2: float  # 上部纵筋按构造配筋的计算面积 mm^2
    as_3: float  # 分布钢筋按照单向板的最小配筋率进行计算 /mm^2
    as_1_actual: float  # 受弯承载力配筋实际面积  As /mm^2
    as_2_actual: float  # 上部纵筋按构造配筋的实际面积 mm^2
    as_3_actual: float  # 分布钢筋实际面积 /mm^2
    d_1_actual: float  # 受弯承载力钢筋直径  d /mm
    d_2_actual: float  # 上部纵筋钢筋直径 d /mm
    d_3_actual: float  # 分布钢筋直径  d /mm
    spacing_1_actual: int  # 受弯承载力钢筋间距  spacing /mm
    spacing_2_actual: int  # 上部纵筋钢筋钢筋间距  spacing /mm
    spacing_3_actual: int  # 分布钢筋间距  spacing /mm
    # 挠度
    mq: float  # 永久组合弯矩值Mq kN*m
    sigma_sq: float  # 纵向受拉钢筋的应力 sq N/mm
    a_te: float  # 矩形截面面积 mm^2
    p_te: float  # 矩形截面配筋率 ρte
    fi_i: float  # 钢筋应变不均匀系数φq
    fi: float  # 调整后钢筋应变不均匀系数φq
    fi_w_i: float  # 用于钢筋应变不均匀系数φq
    fi_w: float  # 用于钢筋应变不均匀系数φq
    alpha_e: float  # 钢筋弹性模量与混凝土弹性模量比值
    gama_f: float  # 受压翼缘面积与腹板有效面积比值 γf
    p_t: float  # 纵向受拉钢筋配筋率ρ
    b_s: float  # 受弯构件的短期刚度 Bsq N*mm^2"
    b_l: float  # 受弯构件长期刚度B  N*mm^2
    theta: float  # 考虑荷载长期效应组合对挠度影响增大影响系数 θ
    m_theta: float  # 受压钢筋面积与受拉钢筋面积比值
    deflection_maxk: float  # 受弯构件挠度  mm", blank=False, null=True, default=0)
    deflection_limit: float  # 挠度限值 mm", blank=False, null=True, default=0)
    deflection_status: bool  # 是否满足挠度规范要求 # 挠度验算结果,False 表示未通过
    # 裂缝
    v_i: float  # 是否带肋钢筋决定 Vi
    rebar_n: int  # 单位面积钢筋根数
    c_s: int  # 最外层纵向受拉钢筋外边缘至受拉区底边的距离
    p_te_w: float  # 裂缝验算矩形截面配筋率 ρte
    d_eq: float  # 受拉区纵筋的等效直径 deq mm
    crack_max: float  # 受弯构件挠度  mm"
    crack_status: bool  # 是否满足挠度规范要求


@dataclass
class _StructuralDesignResult(StructuralDesignResult):
    """
    对内使用的数据结构

    """

    # 混凝土材料参数
    concrete_parameter: ConcreteParameter = field(converter=ConcreteParameter.converter)
    # 钢筋材料参数
    rebar_parameter: RebarParameter = field(converter=RebarParameter.converter)

    @classmethod
    def from_out(
        cls, result: StructuralDesignResult, data: StructuralDesign
    ) -> "_StructuralDesignResult":
        rebar = RebarParameter.by_name(data.material.rebar_name)
        concrete = ConcreteParameter.by_grade(data.material.concrete_grade)
        kwargs = asdict(result)
        kwargs["concrete_parameter"] = concrete
        kwargs["rebar_parameter"] = rebar
        return cls(**kwargs)

    def to_out(self) -> StructuralDesignResult:
        kwargs = asdict(self)
        if "concrete_parameter" in kwargs:
            del kwargs["concrete_parameter"]
        if "rebar_parameter" in kwargs:
            del kwargs["rebar_parameter"]
        return StructuralDesignResult(**kwargs)


@dataclass
class CalculationBook:
    """
    约定楼梯计算书能够拥有的参数
    """

    structure_design: StructuralDesign
    structure_design_result: StructuralDesignResult


@dataclass
class _CalculationBook(CalculationBook):
    structure_design: StructuralDesign
    structure_design_result: _StructuralDesignResult

    @classmethod
    def from_out(cls, book: CalculationBook):
        kwargs = {
            "structure_design": book.structure_design,
            "structure_design_result": _StructuralDesignResult.from_out(
                book.structure_design_result, book.structure_design
            ),
        }
        return cls(**kwargs)
