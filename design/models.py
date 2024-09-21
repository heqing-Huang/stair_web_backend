from django.db import models
from django.core.exceptions import ValidationError

from stair_detailed.models import (
    HoleDesignMode,
    JointDesignMode,
    StepSlotDesignMode,
    WaterDripDesignMode,
    HoleType,
    WaterDripLayout,
    WaterDripShape,
    RebarDesignMode,
    LiftingDesignMode,
    DemoldingDesignMode,
    RailDesignMode,
    LiftingType,
    PouringWay,
    DemoldingType,
    RailLayout,
)
from stair_structure.config import REBAR_NAME_CONFIG

# Create your models here.

REBAR_CONFIG_NAME = [[name, name] for name in REBAR_NAME_CONFIG.keys()]

CONCRETE_GRADE = [
    [30, "C30"],
    [35, "C35"],
    [40, "C40"],
    [45, "C45"],
    [50, "C50"],
]

# stair_detail 数据枚举类被修改
HoistDesignMode = LiftingDesignMode
DemoldDesignMode = DemoldingDesignMode
HoistType = LiftingType
DemoldType = DemoldingType


def get_default_remark_name():
    """
    生成默认项目名
    """
    query_set = ModelConstructionData.objects.all()
    if query_set.count() > 0:
        max_id = ModelConstructionData.objects.all().last().id
    else:
        max_id = 0
    return f"ST&JT-{max_id + 1}"

class PreSetModelData(models.Model):
    project_num = models.CharField(
        verbose_name="项目编号",
        null=True,
        blank=True,
        max_length=64,
        default="",
        help_text="下层结构计算需要该参数",
    )
    component_num = models.CharField(
        verbose_name="构件编号",
        null=True,
        blank=True,
        max_length=64,
        default="",
        help_text="下层结构计算需要该参数",
    )

    # 数据别名
    remark_name = models.CharField(
        verbose_name="参数编号",
        null=True,
        blank=False,
        default=get_default_remark_name,
        max_length=64,
    )

    # material 材质,对标 Material
    rebar_name = models.CharField(
        verbose_name="钢筋等级",
        choices=REBAR_CONFIG_NAME,
        default=REBAR_CONFIG_NAME[0][0],
        max_length=32,
    )
    concrete_grade = models.IntegerField(
        verbose_name="混凝土等级",
        choices=CONCRETE_GRADE,
        default=CONCRETE_GRADE[0][0],
    )

    # 构造相关
    protective_layer_thickness = models.IntegerField(
        verbose_name="保护层厚度 c(mm)", default=20
    )
    longitudinal_top_stress_bar_margin = models.IntegerField(
        verbose_name="纵筋合力点边距 as(mm)", default=20
    )

    # 几何参数
    height = models.IntegerField(verbose_name="楼梯高度 H(mm)", null=True)
    thickness = models.IntegerField(verbose_name="梯段板厚度 t(mm)", null=True)
    weight = models.IntegerField(verbose_name="梯段板宽度 B0(mm)", null=True)
    clear_span = models.IntegerField(verbose_name="净跨 Ln(mm)", null=True)
    top_top_length = models.IntegerField(verbose_name="顶端上边长 Lt(mm)", null=True)
    bottom_top_length = models.IntegerField(verbose_name="底端上边长 Lb(mm)", null=True)
    steps_number = models.IntegerField(verbose_name="踏步数 N", null=True)

    # 荷载
    live_load = models.FloatField(verbose_name="可变荷载 qqk(kN / m^2)",
                                  null=True, default=3.5)
    railing_load = models.FloatField(verbose_name="栏杆荷载 gf(kN/m)",
                                     null=True)
    permanent_load_partial_factor = models.FloatField(
        verbose_name="永久荷载分项系数 rG",
        null=True,
        default=1.2,
    )
    live_load_load_partial_factor = models.FloatField(
        verbose_name="可变荷载分项系数 rQ",
        null=True,
        default=1.4,
    )
    quasi_permanent_value_coefficient = models.FloatField(
        verbose_name="准永久值系数 ψq",
        null=True,
        default=0.4,
    )
    combined_value_coefficient = models.FloatField(
        verbose_name="组合值系数",
        null=True,
        default=0.7,
    )
    reinforced_concrete_bulk_density = models.FloatField(
        verbose_name="钢筋混凝土容重 Rc(kN / m^2)",
        null=True,
        default=25.0,
    )

    # 限制设置
    crack = models.FloatField(verbose_name="裂缝限制 (mm)", null=True)

    # 深化设计相关参数
    top_thickness = models.IntegerField(verbose_name="顶端板厚 h1(mm)", default=0)
    bottom_thickness = models.IntegerField(verbose_name="底端板厚 h2(mm)", default=0)
    top_b = models.IntegerField(verbose_name="顶端挑耳宽度 b1(mm)", default=0)
    bottom_b = models.IntegerField(verbose_name="底端挑耳宽度 b2(mm)",
                                   default=0)

    def __str__(self):
        return self.remark_name

    class Meta:
        verbose_name = "预设模型参数"
        verbose_name_plural = verbose_name

class ModelConstructionData(models.Model):
    """
    结构设计阶段需要录入的参数,需要对标stair_structure 模块的 Construction 参数
    """

    project_num = models.CharField(
        verbose_name="项目编号",
        null=True,
        blank=True,
        max_length=64,
        default="",
        help_text="下层结构计算需要该参数",
    )
    component_num = models.CharField(
        verbose_name="构件编号",
        null=True,
        blank=True,
        max_length=64,
        default="",
        help_text="下层结构计算需要该参数",
    )

    # 数据别名
    remark_name = models.CharField(
        verbose_name="参数编号",
        null=True,
        blank=False,
        default=get_default_remark_name,
        max_length=64,
    )

    # 外键楼梯预设参数
    pre_stair = models.ForeignKey(
        PreSetModelData,
        verbose_name="预设参数",
        null=True,
        blank=True,
        on_delete=models.SET_NULL,
    )

    # material 材质,对标 Material
    rebar_name = models.CharField(
        choices=REBAR_CONFIG_NAME,
        verbose_name="钢筋等级",
        max_length=32,
        default=REBAR_CONFIG_NAME[0][0],
    )
    concrete_grade = models.IntegerField(
        verbose_name="混凝土等级",
        choices=CONCRETE_GRADE,
        default=CONCRETE_GRADE[0][0],
    )

    # 构造相关
    protective_layer_thickness = models.IntegerField(
        verbose_name="保护层厚度 c(mm)", default=20
    )
    longitudinal_top_stress_bar_margin = models.IntegerField(
        verbose_name="纵筋合力点边距 as(mm)", default=20
    )

    # 几何参数
    height = models.IntegerField(verbose_name="楼梯高度 H(mm)", null=True)
    thickness = models.IntegerField(verbose_name="梯段板厚度 t(mm)", null=True)
    weight = models.IntegerField(verbose_name="梯段板宽度 B0(mm)", null=True)
    clear_span = models.IntegerField(verbose_name="净跨 Ln(mm)", null=True)
    top_top_length = models.IntegerField(verbose_name="顶端上边长 Lt(mm)",
                                         null=True)
    bottom_top_length = models.IntegerField(verbose_name="底端上边长 Lb(mm)",
                                            null=True)
    steps_number = models.IntegerField(verbose_name="踏步数 N", null=True)

    # 荷载
    live_load = models.FloatField(verbose_name="可变荷载 qqk(kN / m^2)",
                                  null=True, default=3.5)
    railing_load = models.FloatField(verbose_name="栏杆荷载 gf(kN/m)",
                                     null=True)
    permanent_load_partial_factor = models.FloatField(
        verbose_name="永久荷载分项系数 rG",
        null=True,
        default=1.2,
    )
    live_load_load_partial_factor = models.FloatField(
        verbose_name="可变荷载分项系数 rQ",
        null=True,
        default=1.4,
    )
    quasi_permanent_value_coefficient = models.FloatField(
        verbose_name="准永久值系数 ψq",
        null=True,
        default=0.4,
    )
    combined_value_coefficient = models.FloatField(
        verbose_name="组合值系数",
        null=True,
        default=0.7,
    )
    reinforced_concrete_bulk_density = models.FloatField(
        verbose_name="钢筋混凝土容重 Rc(kN / m^2)",
        null=True,
        default=25.0,
    )

    # 限制设置
    crack = models.FloatField(verbose_name="裂缝限制 (mm)", null=True)

    # 挠度系数
    deflection_coefficient_1 = models.FloatField(
        verbose_name="L0<7m",
        null=True,
        default=0,
    )
    deflection_coefficient_2 = models.FloatField(
        verbose_name="7m<=L0<9m",
        null=True,
        default=0,
    )
    deflection_coefficient_3 = models.FloatField(
        verbose_name="7m<=L0",
        null=True,
        default=0,
    )

    class Meta:
        verbose_name = "结构设计参数"
        verbose_name_plural = verbose_name

    def __str__(self):
        return self.remark_name

    def __repr__(self):
        return f"<{self.__class__.__name__}:{self.__str__()}>"


class ModelConstructionResult(models.Model):
    """
    结构计算的结果
    """

    construction = models.ForeignKey(
        to=ModelConstructionData,
        verbose_name="关联参数",
        null=True,
        blank=True,
        on_delete=models.CASCADE,
    )

    # 楼梯几何参数
    steps_h = models.FloatField(verbose_name="踏步高度 mm", null=True)
    steps_b = models.FloatField(verbose_name="踏步宽度 mm", null=True)
    l0 = models.IntegerField(verbose_name="计算跨度 mm", null=True)
    cos = models.FloatField(verbose_name="梯段板与水平方向夹角余弦值",
                            null=True)
    # 荷载
    gkt = models.FloatField(verbose_name="自重 kN/m", null=True)
    gk = models.FloatField(verbose_name="恒荷标准值 kN/m", null=True)
    png = models.FloatField(verbose_name="Pn(G) 恒荷控制 kN/m", null=True)
    pnl = models.FloatField(verbose_name="Pn(L) 活荷控制 kN/m", null=True)
    pm = models.FloatField(verbose_name="Pm 荷载设计值kN/m", null=True)
    # 正截面受弯承载力
    m_max = models.FloatField(verbose_name="截面最大弯矩 kN*m", null=True)
    h0 = models.FloatField(verbose_name="受压区高度 /mm", null=True)
    alpha_s = models.FloatField(verbose_name="alpha_s", null=True)
    ksi = models.FloatField(verbose_name="相对受压区高度变换系数ξ", null=True)
    ksi_b = models.FloatField(
        null=True, blank=True, verbose_name="界限相对受压区高度变换系数",
        help_text="对应下层变动"
    )
    ksi_status = models.BooleanField(
        verbose_name="状态 ξ相对于ξb | 界限相对受压区高度变换系数", null=True,
        default=False
    )
    p_c_min = models.FloatField(verbose_name="纵向受力钢筋最小配筋率 ρmin",
                                null=True)
    p_c = models.FloatField(verbose_name="受弯承载力计算配筋率", null=True)
    as_1 = models.FloatField(verbose_name="受弯承载力配筋计算面积  As /mm^2",
                             null=True)
    as_2 = models.FloatField(verbose_name="上部纵筋按构造配筋的计算面积 mm^2",
                             null=True)
    as_3 = models.FloatField(
        verbose_name="分布钢筋按照单向板的最小配筋率进行计算 /mm^2", null=True)
    as_fact_1 = models.FloatField(
        verbose_name="受弯承载力配筋实际面积  As /mm^2", null=True)
    as_fact_2 = models.FloatField(
        verbose_name="上部纵筋按构造配筋的实际面积 mm^2", null=True)

    as_fact_3 = models.FloatField(verbose_name="分布钢筋实际面积 /mm^2",
                                  null=True)
    d_fact_1 = models.FloatField(verbose_name="受弯承载力钢筋直径  d /mm",
                                 null=True)
    d_fact_2 = models.FloatField(verbose_name="上部纵筋钢筋直径 d /mm",
                                 null=True)
    d_fact_3 = models.FloatField(verbose_name="分布钢筋直径  d /mm", null=True)
    spacing_fact_1 = models.IntegerField(
        verbose_name="受弯承载力钢筋间距  spacing /mm", null=True
    )
    spacing_fact_2 = models.IntegerField(
        verbose_name="上部纵筋钢筋钢筋间距  spacing /mm", null=True
    )
    spacing_fact_3 = models.IntegerField(
        verbose_name="分布钢筋间距  spacing /mm", null=True)
    # 挠度
    mq = models.FloatField(verbose_name="永久组合弯矩值Mq kN*m", null=True)
    sigma_sq = models.FloatField(verbose_name="纵向受拉钢筋的应力 sq N/mm",
                                 null=True)
    a_te = models.FloatField(verbose_name="矩形截面面积 mm^2", null=True)
    p_te = models.FloatField(verbose_name="矩形截面配筋率 ρte", null=True)
    fi_i = models.FloatField(verbose_name="钢筋应变不均匀系数φq", null=True)
    fi = models.FloatField(verbose_name="调整后钢筋应变不均匀系数φq",
                           null=True)
    alpha_e = models.FloatField(
        verbose_name="钢筋弹性模量与混凝土弹性模量比值", null=True)
    gama_f = models.FloatField(
        verbose_name="受压翼缘面积与腹板有效面积比值 γf", null=True)
    p_t = models.FloatField(verbose_name="纵向受拉钢筋配筋率ρ", null=True)
    b_s = models.FloatField(verbose_name="受弯构件的短期刚度 Bsq N*mm^2",
                            null=True)
    b_l = models.FloatField(verbose_name="受弯构件长期刚度B  N*mm^2",
                            null=True)
    theta = models.FloatField(
        verbose_name="考虑荷载长期效应组合对挠度影响增大影响系数 θ", null=True)
    m_theta = models.FloatField(verbose_name="受压钢筋面积与受拉钢筋面积比值",
                                null=True)
    deflection_maxk = models.FloatField(verbose_name="受弯构件挠度", null=True)
    deflection_limit = models.FloatField(verbose_name="挠度限值 mm", null=True)
    deflection_status = models.BooleanField(
        verbose_name="是否满足挠度规范要求",
        null=True,
        default=False,
        help_text="挠度验算结果,False 表示未通过",
    )
    fi_w_i = models.FloatField(verbose_name="钢筋应变不均匀系数φq", null=True)
    fi_w = models.FloatField(verbose_name="钢筋应变不均匀系数φq", null=True)
    # 裂缝
    v_i = models.FloatField(verbose_name="是否带肋钢筋决定 Vi", null=True)
    rebar_n = models.IntegerField(verbose_name="单位面积钢筋根数", null=True)
    c_s = models.IntegerField(
        verbose_name="最外层纵向受拉钢筋外边缘至受拉区底边的距离", null=True)
    p_te_w = models.FloatField(verbose_name="裂缝验算矩形截面配筋率 ρte",
                               null=True)
    d_eq = models.FloatField(verbose_name="受拉区纵筋的等效直径 deq mm",
                             null=True)
    crack_max = models.FloatField(verbose_name="受弯构件挠度  mm", null=True)
    crack_status = models.BooleanField(
        verbose_name="是否满足挠度规范要求", null=True, default=False
    )
    # 调用状态
    success = models.BooleanField(
        verbose_name="计算状态", default=False, help_text="调用计算的状态"
    )
    message = models.CharField(
        verbose_name="错误原因",
        null=True,
        blank=True,
        help_text="错误原因",
        max_length=256,
    )

    class Meta:
        verbose_name = "结构计算结果"
        verbose_name_plural = verbose_name


# DEMOLD_TYPE_CHOICE 配置django 显示部分
DEMOLD_TYPE_CHOICE_SHOW = {
    0: "圆头吊钉",
    1: "锚栓",
}


class DetailData(models.Model):
    """
    深化设计需要参数
    """

    # 外键选择楼梯结构参数
    stair = models.ForeignKey(
        ModelConstructionData,
        verbose_name="结构参数",
        null=True,
        blank=False,
        on_delete=models.SET_NULL,
    )
    # 几何部分,长度单位均为毫米
    width = models.IntegerField(
        verbose_name="梯段板宽度 B0(mm)",
        help_text="此参数需和结构设计端保持一致"
    )
    top_to_length = models.IntegerField(
        verbose_name="顶端上边长 Lt(mm)",
        help_text="此参数需和结构设计端保持一致"
    )
    bottom_top_length = models.IntegerField(
        verbose_name="底端上边长 Lb(mm)",
        help_text="此参数需和结构设计端保持一致"
    )
    top_thickness = models.IntegerField(verbose_name="顶端板厚 h1(mm)")
    bottom_thickness = models.IntegerField(verbose_name="底端板厚 h2(mm)")
    top_b = models.IntegerField(verbose_name="顶端挑耳宽度 b1(mm)", default=0)
    bottom_b = models.IntegerField(verbose_name="底端挑耳宽度 b2(mm)",
                                   default=0)
    # 外观设计
    HOLE_DESIGN_MODE_CHOICE = (
        (HoleDesignMode.AUTOMATIC.value, "自动"),
        (HoleDesignMode.MANUAL.value, "手动"),
    )
    hole_design_mode = models.IntegerField(
        verbose_name="孔洞设计方式",
        choices=HOLE_DESIGN_MODE_CHOICE,
        default=HoleDesignMode.AUTOMATIC.value,
    )

    JOINT_DESIGN_MODE_CHOICE = (
        (JointDesignMode.AUTOMATIC.value, "自动"),
        (JointDesignMode.MANUAL.value, "手动"),
    )
    joint_design_mode = models.IntegerField(
        verbose_name="节点设计方式",
        choices=JOINT_DESIGN_MODE_CHOICE,
        default=JointDesignMode.AUTOMATIC.value,
    )

    STEP_SLOT_DESIGN_MODE_CHOICE = (
        (StepSlotDesignMode.AUTOMATIC.value, "自动"),
        (StepSlotDesignMode.MANUAL.value, "手动"),
    )

    step_slot_design_mode = models.IntegerField(
        verbose_name="防滑槽设计方式",
        choices=STEP_SLOT_DESIGN_MODE_CHOICE,
        default=StepSlotDesignMode.AUTOMATIC.value,
    )

    WATER_DRIP_DESIGN_MODE_CHOICE = (
        (WaterDripDesignMode.MANUAL.value, "手动"),
        (WaterDripDesignMode.NO.value, "No"),
    )
    water_drip_design_mode = models.IntegerField(
        verbose_name="滴水槽设计方式",
        choices=WATER_DRIP_DESIGN_MODE_CHOICE,
        default=WaterDripDesignMode.NO.value,
    )

    def validator_water_drip_design_mode(self):
        water_design_mode = WaterDripDesignMode(self.water_drip_design_mode)
        error_message = "滴水槽设计模式为手动时,截面不可为空"
        if water_design_mode == WaterDripDesignMode.MANUAL:
            if (
                    self.water_drip_semicircle_a is None
                    or self.water_drip_semicircle_b is None
            ) and (
                    self.water_drip_trapezoid_a is None
                    or self.water_drip_trapezoid_b is None
                    or self.water_drip_trapezoid_c is None
            ):
                raise ValidationError(error_message)

    def clean(self):
        super().clean()
        custom_validators = [self.validator_water_drip_design_mode]
        for fun in custom_validators:
            fun()

    # 孔洞 - 顶端
    HOLE_TYPE_CHOICE = (
        (HoleType.FIXED_HINGE.value, "固定铰支座"),
        (HoleType.SLIDING_HINGE.value, "滑动铰支座"),
    )
    top_hole_type = models.IntegerField(
        verbose_name="顶端孔洞类型",
        choices=HOLE_TYPE_CHOICE,
        default=HoleType.FIXED_HINGE.value,
    )
    # 顶端孔洞位置参数
    top_hole_position_a1 = models.IntegerField(
        verbose_name=" a1(mm)", null=True, blank=True
    )
    top_hole_position_a2 = models.IntegerField(
        verbose_name=" a2(mm)", null=True, blank=True
    )
    top_hole_position_b1 = models.IntegerField(
        verbose_name=" a3(mm)", null=True, blank=True
    )
    top_hole_position_b2 = models.IntegerField(
        verbose_name=" a4(mm)", null=True, blank=True
    )

    # 孔洞 - 底端
    bottom_hole_type = models.IntegerField(
        verbose_name="底端孔洞类型",
        choices=HOLE_TYPE_CHOICE,
        default=HoleType.FIXED_HINGE.value,
    )
    # 顶端固定铰支座参数
    top_fix_hinge_c2 = models.IntegerField(
        verbose_name=" c2(mm)", null=True, blank=True
    )
    top_fix_hinge_d2 = models.IntegerField(
        verbose_name=" d2(mm)", null=True, blank=True
    )
    # 顶端滑动铰支座参数
    top_sliding_hinge_c1 = models.IntegerField(
        verbose_name=" c1(mm)", null=True, blank=True
    )
    top_sliding_hinge_d1 = models.IntegerField(
        verbose_name=" d1(mm)", null=True, blank=True
    )
    top_sliding_hinge_e1 = models.IntegerField(
        verbose_name=" e1(mm)", null=True, blank=True
    )
    top_sliding_hinge_f1 = models.IntegerField(
        verbose_name=" f1(mm)", null=True, blank=True
    )
    top_sliding_hinge_h1 = models.IntegerField(
        verbose_name="hinge_h1", null=True, blank=True
    )
    # 底端固定铰支座
    bottom_fix_hinge_c2 = models.IntegerField(
        verbose_name=" c2(mm)", null=True, blank=True
    )
    bottom_fix_hinge_d2 = models.IntegerField(
        verbose_name=" d2(mm)", null=True, blank=True
    )
    # 底端滑动铰支座
    bottom_sliding_hinge_c1 = models.IntegerField(
        verbose_name=" c1(mm)", null=True, blank=True
    )
    bottom_sliding_hinge_d1 = models.IntegerField(
        verbose_name=" d1(mm)", null=True, blank=True
    )
    bottom_sliding_hinge_e1 = models.IntegerField(
        verbose_name=" e1(mm)", null=True, blank=True
    )
    bottom_sliding_hinge_f1 = models.IntegerField(
        verbose_name=" f1(mm)", null=True, blank=True
    )
    bottom_sliding_hinge_h1 = models.IntegerField(
        verbose_name=" h1(mm)", null=True, blank=True
    )
    # 底端孔洞位置参数
    bottom_hole_position_a1 = models.IntegerField(
        verbose_name=" b1(mm)", null=True, blank=True
    )
    bottom_hole_position_a2 = models.IntegerField(
        verbose_name=" b2(mm)", null=True, blank=True
    )
    bottom_hole_position_b1 = models.IntegerField(
        verbose_name=" b3(mm)", null=True, blank=True
    )
    bottom_hole_position_b2 = models.IntegerField(
        verbose_name=" b4(mm)", null=True, blank=True
    )

    # 节点设计
    # 顶部节点外型尺寸
    top_joint_a = models.IntegerField(verbose_name=" a(mm)", null=True,
                                      blank=True)
    top_joint_b = models.IntegerField(verbose_name=" b(mm)", null=True,
                                      blank=True)
    top_joint_c = models.IntegerField(verbose_name=" c(mm)", null=True,
                                      blank=True)
    # 底部节点外型尺寸
    bottom_joint_a = models.IntegerField(verbose_name=" a(mm)", null=True,
                                         blank=True)
    bottom_joint_b = models.IntegerField(verbose_name=" b(mm)", null=True,
                                         blank=True)
    bottom_joint_c = models.IntegerField(verbose_name=" c(mm)", null=True,
                                         blank=True)

    # 防滑槽
    # 防滑槽参数
    step_slot_a = models.IntegerField(verbose_name=" a(mm)", null=True,
                                      blank=True)
    step_slot_b = models.IntegerField(verbose_name=" b(mm)", null=True,
                                      blank=True)
    step_slot_c = models.IntegerField(verbose_name=" c(mm)", null=True,
                                      blank=True)
    step_slot_d = models.IntegerField(verbose_name=" d(mm)", null=True,
                                      blank=True)
    step_slot_e = models.IntegerField(verbose_name=" e(mm)", null=True,
                                      blank=True)
    # 防滑槽位置参数
    step_slot_position_c1 = models.IntegerField(
        verbose_name=" c1(mm)", null=True, blank=True
    )
    step_slot_position_c2 = models.IntegerField(
        verbose_name=" c2(mm)", null=True, blank=True
    )
    step_slot_position_c3 = models.IntegerField(
        verbose_name=" c3(mm)", null=True, blank=True
    )

    # 滴水槽
    WATER_DRIP_LAYOUT_CHOICE = (
        (WaterDripLayout.ONLY_TOP.value, "只上侧"),
        (WaterDripLayout.ONLY_BOTTOM.value, "只下侧"),
        (WaterDripLayout.BOTH.value, "上侧和下侧"),
    )
    water_drip_layout = models.IntegerField(
        verbose_name="滴水槽布局方式",
        choices=WATER_DRIP_LAYOUT_CHOICE,
        default=WaterDripLayout.BOTH.value,
    )
    WATER_DRIP_SHAPE_CHOICE = (
        (WaterDripShape.SEMICIRCLE.value, "半圆"),
        (WaterDripShape.TRAPEZOID.value, "梯形"),
    )
    water_drip_shape = models.IntegerField(
        verbose_name="滴水槽截面类型",
        choices=WATER_DRIP_SHAPE_CHOICE,
        default=WaterDripShape.TRAPEZOID.value,
    )
    # 半圆
    water_drip_semicircle_a = models.FloatField(
        verbose_name=" a(mm)", null=True, blank=True
    )
    water_drip_semicircle_b = models.FloatField(
        verbose_name=" b(mm)", null=True, blank=True
    )
    # 梯形
    water_drip_trapezoid_a = models.FloatField(
        verbose_name=" a(mm)", null=True, blank=True
    )
    water_drip_trapezoid_b = models.FloatField(
        verbose_name=" b(mm)", null=True, blank=True
    )
    water_drip_trapezoid_c = models.FloatField(
        verbose_name=" c(mm)", null=True, blank=True
    )

    # 位置参数
    water_drip_position_a1 = models.FloatField(
        verbose_name=" a1(mm)", null=True, blank=True
    )
    water_drip_position_a2 = models.FloatField(
        verbose_name=" a2(mm)", null=True, blank=True
    )
    water_drip_position_a3 = models.FloatField(
        verbose_name=" a3(mm)", null=True, blank=True
    )

    # 钢筋深化设计参数
    REBA_DESIGN_MODE_CHOICE = (
        (RebarDesignMode.AUTOMATIC.value, "自动"),
        (RebarDesignMode.MANUAL.value, "手动"),
    )
    rebar_design_mode = models.IntegerField(
        verbose_name="钢筋设计方式",
        choices=REBA_DESIGN_MODE_CHOICE,
        default=RebarDesignMode.AUTOMATIC.value,
    )
    # 4号底端边缘纵筋
    bottom_edge_longitudinal_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    bottom_edge_longitudinal_rebar_spacing = models.IntegerField(
        verbose_name="间距S mm", null=True, blank=True
    )
    # 5号顶端边缘纵筋
    top_edge_longitudinal_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    top_edge_longitudinal_rebar_spacing = models.IntegerField(
        verbose_name="间距S mm", null=True, blank=True
    )
    # 6号底端边缘箍筋
    bottom_edge_stirrup_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    bottom_edge_stirrup_spacing = models.IntegerField(
        verbose_name="间距S mm", null=True, blank=True
    )
    # 9号顶端边缘箍筋
    top_edge_stirrup_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    top_edge_stirrup_spacing = models.IntegerField(
        verbose_name="间距S mm", null=True, blank=True
    )

    # 7号 销键加强筋
    hole_reinforce_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    # 8号 吊点加强筋
    hoisting_reinforce_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    # 10号 吊点加强筋
    top_edge_reinforce_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )
    # 11号 吊点加强筋
    bottom_edge_reinforce_rebar_diameter = models.IntegerField(
        verbose_name="直径D mm", null=True, blank=True
    )

    # 预埋件相关
    HOIST_DESIGN_MODE_CHOICE = (
        (HoistDesignMode.AUTOMATIC.value, "自动"),
        (HoistDesignMode.MANUAL.value, "手动"),
    )
    hoist_design_mode = models.IntegerField(
        verbose_name="吊装预埋件设计方式",
        choices=HOIST_DESIGN_MODE_CHOICE,
        default=HoistDesignMode.AUTOMATIC.value,
    )

    DEMOLD_DESIGN_MODE_CHOICE = (
        (DemoldDesignMode.AUTOMATIC.value, "自动"),
        (DemoldDesignMode.MANUAL.value, "手动"),
    )
    demold_design_mode = models.IntegerField(
        verbose_name="脱模预埋件设计方式",
        choices=DEMOLD_DESIGN_MODE_CHOICE,
        default=DemoldDesignMode.AUTOMATIC.value,
    )

    RAIL_DESIGN_MODE_CHOICE = (
        (RailDesignMode.MANUAL.value, "手动"),
        (RailDesignMode.NO.value, "不计算"),
    )
    rail_design_mode = models.IntegerField(
        verbose_name="栏杆预埋件设计方式",
        choices=RAIL_DESIGN_MODE_CHOICE,
        default=RailDesignMode.NO.value,
    )
    HOIST_TYPE_CHOICE = (
        (HoistType.ANCHOR.value, "锚栓"),
        (HoistType.ROUNDING_HEAD.value, "圆头吊钉"),
    )
    hoist_type = models.IntegerField(
        verbose_name="吊装预埋件类型",
        choices=HOIST_TYPE_CHOICE,
        default=HOIST_DESIGN_MODE_CHOICE[0][0],
        null=True,
    )

    # 吊装预埋件定位信息
    hoist_position_a = models.IntegerField(verbose_name=" a(mm)", null=True,
                                           blank=True)
    hoist_position_b = models.IntegerField(verbose_name=" b(mm)", null=True,
                                           blank=True)
    hoist_position_c = models.FloatField(verbose_name=" c(mm)", null=True,
                                         blank=True)
    hoist_position_d = models.FloatField(verbose_name=" d(mm)", null=True,
                                         blank=True)

    # 吊装预埋件规格
    hoist_name = models.CharField(
        verbose_name="吊装预埋件规格", max_length=64, null=True,
        default="DJ-25-170"
    )

    # 脱模方式
    POURING_WAY_CHOICE = (
        (PouringWay.VERTICAL_HORIZONTAL.value, "立式浇筑卧式脱模"),
        (PouringWay.VERTICAL_VERTICAL.value, "立式浇筑立式脱模"),
        (PouringWay.HORIZONTAL_HORIZONTAL.value, "卧式浇筑卧式脱模"),
    )
    pouring_way = models.IntegerField(
        verbose_name="脱模方式",
        choices=POURING_WAY_CHOICE,
        default=PouringWay.VERTICAL_HORIZONTAL.value,
    )

    DEMOLD_TYPE_CHOICE = [
        (tag.value, DEMOLD_TYPE_CHOICE_SHOW[tag.value]) for tag in DemoldType
    ]
    demold_type = models.IntegerField(
        verbose_name="脱模预埋件类型",
        choices=DEMOLD_TYPE_CHOICE,
        default=DemoldType.ANCHOR.value,
    )

    # 脱模位置参数
    demold_position_a = models.IntegerField(
        verbose_name=" a(mm)", null=True, blank=True
    )
    demold_position_b = models.IntegerField(
        verbose_name=" b(mm)", null=True, blank=True
    )
    demold_position_c = models.IntegerField(
        verbose_name=" c(mm)", null=True, blank=True
    )
    demold_position_d = models.IntegerField(
        verbose_name=" d(mm)", null=True, blank=True
    )
    demold_position_t = models.IntegerField(
        verbose_name="预埋件侧面位置参数 t(mm)", null=True, blank=True
    )

    # 脱模件规格
    demold_name = models.CharField(
        verbose_name="脱模预埋件规格", null=True, max_length=64, blank=True
    )

    # 栏杆预埋件
    RAIL_LAYOUT_CHOICE = (
        (RailLayout.ONLY_RIGHT.value, "只右侧"),
        (RailLayout.ONLY_LEFT.value, "只左侧"),
        (RailLayout.BOTH.value, "左+右"),
    )
    rail_layout = models.IntegerField(
        verbose_name="栏杆预埋件布局",
        choices=RAIL_LAYOUT_CHOICE,
        default=RailLayout.ONLY_RIGHT.value,
    )

    # 栏杆预埋件所在的阶数
    rail_number = models.TextField(
        verbose_name="栏杆预埋件所在的阶数", null=True, default="3", blank=True
    )

    # 栏杆位置
    rail_position_a = models.FloatField(verbose_name=" a(mm)", null=True,
                                        blank=True)
    rail_position_b = models.FloatField(verbose_name=" b(mm)", null=True,
                                        blank=True)

    # 栏杆埋件型号
    rail_name = models.CharField(
        verbose_name="栏杆预埋件型号", null=True, max_length=64, default="M1"
    )

    class Meta:
        verbose_name = "深化设计参数"
        verbose_name_plural = verbose_name


class DetailDataCopyChangeWrite(models.Model):
    """
    此表用于存放深化设计期间被修改的参数,但并不做显示
    """

    content = models.JSONField(verbose_name="design parameter")

    class Meta:
        verbose_name = "深化设计-参数修改"
        verbose_name_plural = verbose_name


class ModelDetailedResult(models.Model):
    """
    存放深化设计返回的各种参数
    """

    # 外键楼梯结构参数
    stair = models.ForeignKey(
        ModelConstructionData,
        verbose_name="结构参数",
        null=True,
        blank=False,
        on_delete=models.SET_NULL,
    )
    #  楼梯几何参数
    top_bottom_length = models.FloatField(verbose_name="顶端下边长 mm")
    bottom_bottom_length = models.FloatField(verbose_name="底端下边长 mm")
    l0 = models.IntegerField(verbose_name="计算跨度 mm")
    h0 = models.IntegerField(verbose_name="楼梯总高度 mm")
    v = models.FloatField(verbose_name="计算体积")
    cos = models.FloatField(verbose_name="梯段板与水平方向夹角余弦值")
    sin = models.FloatField(verbose_name="梯段板与水平方向夹角正弦值")
    tan = models.FloatField(verbose_name="梯段板与水平方向夹角正切值")
    # 荷载计算
    gkt = models.FloatField(verbose_name="自重 kN/m")
    gk = models.FloatField(verbose_name="恒荷标准值 kN/m")

    # 吊装预埋件
    gdk = models.FloatField(
        verbose_name="吊装预埋件自重标准值乘以动力系数 kN/m")
    gek = models.FloatField(
        verbose_name="脱模预埋件自重标准值乘以动力系数 kN/m")
    single_capacity_hosit = models.FloatField(verbose_name="受压区高度 /mm")
    hoist_type = models.IntegerField(
        verbose_name="吊装预埋件类型",
        choices=DetailData.HOIST_TYPE_CHOICE,
        default=DetailData.HOIST_TYPE_CHOICE[0][0],
    )
    hoist_name = models.CharField(verbose_name="吊装件型号", max_length=128)

    # AnchorParameter
    hoist_parameter_anchor_parameter_type = models.CharField(
        verbose_name="类型", max_length=64
    )
    hoist_parameter_anchor_parameter_factory = models.CharField(
        verbose_name="系列名", max_length=64
    )
    hoist_parameter_anchor_parameter_name = models.CharField(
        verbose_name="名称", max_length=64
    )
    hoist_parameter_anchor_parameter_abbreviation = models.CharField(
        verbose_name="前缀编号", max_length=64
    )
    hoist_parameter_anchor_parameter_capacity = models.FloatField(
        verbose_name="承载力", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_m_diameter = models.FloatField(
        verbose_name="接口直径", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_m_length = models.FloatField(
        verbose_name="埋入深度", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_anchor_name = models.CharField(
        verbose_name="锚栓螺纹", max_length=64
    )
    hoist_parameter_anchor_parameter_e_diameter = models.FloatField(
        verbose_name="嵌入直径", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_g = models.FloatField(
        verbose_name="嵌入长度", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_b = models.FloatField(
        verbose_name="嵌入箭头长度", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_o_diameter = models.FloatField(
        verbose_name="锚栓直径", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_length = models.FloatField(
        verbose_name="锚栓长度", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_s_diameter = models.FloatField(
        verbose_name="卡槽直径", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_l_p = models.FloatField(
        verbose_name="卡槽长度", null=True, blank=True
    )
    hoist_parameter_anchor_parameter_a = models.FloatField(
        verbose_name="卡槽边距", null=True, blank=True
    )

    # RoundHeadParameter
    hoist_parameter_round_head_parameter_type = models.CharField(
        verbose_name="类型", max_length=64
    )
    hoist_parameter_round_head_parameter_factory = models.CharField(
        verbose_name="系列名", max_length=64
    )
    hoist_parameter_round_head_parameter_name = models.CharField(
        verbose_name="名称", max_length=64
    )
    hoist_parameter_round_head_parameter_abbreviation = models.CharField(
        verbose_name="编号前缀", max_length=64
    )  #
    hoist_parameter_round_head_parameter_capacity = models.FloatField(
        verbose_name="承载力", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_length = models.FloatField(
        verbose_name="吊钉长度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_top_diameter = models.FloatField(
        verbose_name="顶部直径", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_top_height = models.FloatField(
        verbose_name="顶部高度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_top_adjacent_height = models.FloatField(
        verbose_name="顶部连接高度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_middle_diameter = models.FloatField(
        verbose_name="中间直径", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_middle_height = models.FloatField(
        verbose_name="中间高度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_bottom_adjacent_height = models.FloatField(
        verbose_name="底部连接高度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_bottom_diameter = models.FloatField(
        verbose_name="底部半径", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_bottom_height = models.FloatField(
        verbose_name="底部高度", null=True, blank=True
    )
    hoist_parameter_round_head_parameter_radius = models.FloatField(
        verbose_name="埋入半径", null=True, blank=True
    )

    hoist_edge_xa = models.FloatField(verbose_name="hoist_edge_xa")
    hoist_edge_xb = models.FloatField(verbose_name="hoist_edge_xb")
    hoist_edge_xc = models.FloatField(verbose_name="hoist_edge_xc")
    m_hoist_ka = models.FloatField(verbose_name="m_hoist_ka")
    m_hoist_kb = models.FloatField(verbose_name="m_hoist_kb")
    w_hoist_a = models.FloatField(verbose_name="w_hoist_a")
    w_hoist_b = models.FloatField(verbose_name="w_hoist_b")
    sigm_hoist_cka = models.FloatField(verbose_name="sigm_hoist_cka")
    sigm_hoist_ckb = models.FloatField(verbose_name="sigm_hoist_ckb")
    max_sigm = models.FloatField(verbose_name="max_sigm")
    hoist_f_tk = models.FloatField(verbose_name="host_f_tk")
    pouring_way = models.IntegerField(
        verbose_name="浇筑形式",
        choices=DetailData.POURING_WAY_CHOICE,
        default=DetailData.POURING_WAY_CHOICE[0][0],
    )  # 浇筑形式
    q_k1 = models.FloatField(verbose_name="q_k1")
    single_capacity_demold = models.FloatField(
        verbose_name="脱模预埋件单承载")  #
    demold_type = models.IntegerField(
        verbose_name="吊装预埋件类型",
        choices=DetailData.DEMOLD_TYPE_CHOICE,
        default=DetailData.DEMOLD_TYPE_CHOICE[0][0],
    )  #
    demold_name = models.CharField(verbose_name="吊装件型号", max_length=64)

    # AnchorParameter
    demold_parameter_anchor_parameter_type = models.CharField(
        verbose_name="类型", max_length=64
    )
    demold_parameter_anchor_parameter_factory = models.CharField(
        verbose_name="系列名", max_length=64
    )
    demold_parameter_anchor_parameter_name = models.CharField(
        verbose_name="名称", max_length=64
    )
    demold_parameter_anchor_parameter_abbreviation = models.CharField(
        verbose_name="前缀编号", max_length=64
    )
    demold_parameter_anchor_parameter_capacity = models.FloatField(
        verbose_name="承载力", null=True, blank=True
    )
    demold_parameter_anchor_parameter_m_diameter = models.FloatField(
        verbose_name="接口直径", null=True, blank=True
    )
    demold_parameter_anchor_parameter_m_length = models.FloatField(
        verbose_name="埋入深度", null=True, blank=True
    )
    demold_parameter_anchor_parameter_anchor_name = models.CharField(
        verbose_name="锚栓螺纹", max_length=64
    )
    demold_parameter_anchor_parameter_e_diameter = models.FloatField(
        verbose_name="嵌入直径", null=True, blank=True
    )
    demold_parameter_anchor_parameter_g = models.FloatField(
        verbose_name="嵌入长度", null=True, blank=True
    )
    demold_parameter_anchor_parameter_b = models.FloatField(
        verbose_name="嵌入箭头长度", null=True, blank=True
    )
    demold_parameter_anchor_parameter_o_diameter = models.FloatField(
        verbose_name="锚栓直径", null=True, blank=True
    )
    demold_parameter_anchor_parameter_length = models.FloatField(
        verbose_name="锚栓长度", null=True, blank=True
    )
    demold_parameter_anchor_parameter_s_diameter = models.FloatField(
        verbose_name="卡槽直径", null=True, blank=True
    )
    demold_parameter_anchor_parameter_l_p = models.FloatField(
        verbose_name="卡槽长度", null=True, blank=True
    )
    demold_parameter_anchor_parameter_a = models.FloatField(
        verbose_name="卡槽边距", null=True, blank=True
    )

    # RoundHeadParameter
    demold_parameter_round_head_parameter_type = models.CharField(
        verbose_name="类型", max_length=64
    )
    demold_parameter_round_head_parameter_factory = models.CharField(
        verbose_name="系列名", max_length=64
    )
    demold_parameter_round_head_parameter_name = models.CharField(
        verbose_name="名称", max_length=64
    )
    demold_parameter_round_head_parameter_abbreviation = models.CharField(
        verbose_name="编号前缀", max_length=64
    )
    demold_parameter_round_head_parameter_capacity = models.FloatField(
        verbose_name="承载力", null=True, blank=True
    )
    demold_parameter_round_head_parameter_length = models.FloatField(
        verbose_name="吊钉长度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_top_diameter = models.FloatField(
        verbose_name="顶部直径", null=True, blank=True
    )
    demold_parameter_round_head_parameter_top_height = models.FloatField(
        verbose_name="顶部高度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_top_adjacent_height = models.FloatField(
        verbose_name="顶部连接高度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_middle_diameter = models.FloatField(
        verbose_name="中间直径", null=True, blank=True
    )
    demold_parameter_round_head_parameter_middle_height = models.FloatField(
        verbose_name="中间高度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_bottom_adjacent_height = models.FloatField(
        verbose_name="底部连接高度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_bottom_diameter = models.FloatField(
        verbose_name="底部半径", null=True, blank=True
    )
    demold_parameter_round_head_parameter_bottom_height = models.FloatField(
        verbose_name="底部高度", null=True, blank=True
    )
    demold_parameter_round_head_parameter_radius = models.FloatField(
        verbose_name="埋入半径", null=True, blank=True
    )

    # RailParameter
    rail_parameter_name = models.CharField(verbose_name="name", max_length=64)
    rail_parameter_a = models.IntegerField(verbose_name="a", null=True,
                                           blank=True)
    rail_parameter_b = models.IntegerField(verbose_name="b", null=True,
                                           blank=True)
    rail_parameter_c = models.IntegerField(verbose_name="c", null=True,
                                           blank=True)
    rail_parameter_d = models.IntegerField(verbose_name="d", null=True,
                                           blank=True)
    rail_parameter_t = models.IntegerField(verbose_name="t", null=True,
                                           blank=True)
    rail_parameter_fi = models.IntegerField(verbose_name="直径", null=True,
                                            blank=True)
    rail_parameter_depth = models.IntegerField(
        verbose_name="栏杆埋入深度", null=True, blank=True
    )
    rail_parameter_length = models.IntegerField(
        verbose_name="底边长度", null=True, blank=True
    )

    # discussion https://github.com/IBLofCQU/Stair_rebar_layout/discussions/5#discussioncomment-3865559
    # 依赖库的变更https://github.com/IBLofCQU/stair_detailed/blob/499419c4c1e7298b7ef345d9057d7e0a0f774614/stair_detailed/models.py#L862-L862
    # 项目使用实际情况,不支持JsonField
    construction_detailed = models.TextField(
        verbose_name="construction_detailed", null=True, blank=True
    )

    detailed_design = models.ForeignKey(
        DetailDataCopyChangeWrite,
        null=True,
        verbose_name="参数修改",
        help_text="针对stair-detailed 1.0.0 所作改动,仅存储,但不做展示",
        on_delete=models.CASCADE,
    )

    class Meta:
        verbose_name = "深化设计结果"
        verbose_name_plural = verbose_name


class RebarLayoutModel(models.Model):
    """
    存放钢筋排布的数据
    """

    stair = models.ForeignKey(
        ModelConstructionData, verbose_name="所属楼梯",
        on_delete=models.CASCADE
    )
    content = models.JSONField(verbose_name="钢筋数据", null=True, blank=True)

    class Meta:
        verbose_name = "钢筋排布"
        verbose_name_plural = verbose_name


class FileExport(models.Model):
    """
    存放导出的多种模型文件
    """

    stair = models.ForeignKey(
        ModelConstructionData, verbose_name="导出文件",
        on_delete=models.CASCADE
    )
    ifc = models.FileField(
        verbose_name="IFC 模型文件",
        upload_to="%Y/%m/%d",
        null=True,
    )
    bvbs = models.FileField(
        verbose_name="BVBS",
        upload_to="%Y/%m/%d",
        null=True,
    )
    zip_json = models.FileField(
        verbose_name="Rebar Json Zip",
        upload_to="%Y/%m/%d",
        null=True,
    )
    dxf = models.FileField(
        verbose_name="Cad Dxf",
        upload_to="%Y/%m/%d",
        null=True,
    )

    class Meta:
        verbose_name = "楼梯导出模型"
        verbose_name_plural = verbose_name
