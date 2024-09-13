"""
Date&Time           2022/8/3 11:26
Author              HaoLan

"""
import logging
from io import BytesIO
from typing import IO, Tuple, Any
from docxtpl import DocxTemplate

from stair_structure.model import (
    StructuralDesign,
    StructuralDesignResult,
    ConcreteParameter,
    RebarParameter,
)
from stair_detailed.config import (
    ROUNDING_HEAD,
    ANCHOR,
    RAILING,
)  # , ANCHOR   # , RAILING
from stair_detailed.models import (
    RoundHeadParameter,
    DetailedDesign,
    DetailedDesignResult,
    DetailedCalculationBook,
    LiftingPosition,
    DemoldingPosition,
    LiftingDesignMode,
    LiftingType,
    PouringWay,
    StepSlotDesignMode,
    RailDesignMode,
    RailParameter,
    DemoldingDesignMode,
    DemoldingType,
    AnchorParameter,
)
from stair_detailed.tools import get_volume

_logger = logging.getLogger(__name__)


def detailed_design(
    parameter: DetailedDesign,
    structure_design: StructuralDesign,
    structure_design_result: StructuralDesignResult,
) -> DetailedDesignResult:
    """
    该部分需要完成结构计算的逻辑,并返回结构计算对应的数据
    :param parameter:
    :param structure_design_result:
    :param structure_design:
    :return: ConstructionResult_detailed
    """
    assert isinstance(parameter, DetailedDesign), Exception(f"parameter 数据异常")
    assert isinstance(structure_design, StructuralDesign), Exception(
        f"structure_design 数据异常"
    )
    assert isinstance(structure_design_result, StructuralDesignResult), Exception(
        f"structure_design_result 数据异常"
    )

    # 几何信息
    b0: int = parameter.geometric_detailed.width  # 梯段板宽度   即b0， 结构设计输入的值会比b0略大
    ln = structure_design.geometric.clear_span  # 预制楼梯净跨 mm
    steps_h: float = structure_design_result.steps_h  # 踏步高度
    steps_b: float = structure_design_result.steps_b  # 踏步宽度
    height = structure_design.geometric.height  # 楼梯高度
    # 材料信息
    rc = structure_design.load_data.reinforced_concrete_bulk_density  # 混凝土容重
    cos: float = structure_design_result.cos  # 楼梯倾角余弦值
    sin: float = steps_h / ((steps_b**2) + (steps_h**2)) ** 0.5  # 楼梯倾角正弦值
    tan: float = steps_h / steps_b  # 楼梯倾角正切值

    thickness: int = structure_design.geometric.thickness  # 楼梯厚度
    steps_number: int = structure_design.geometric.steps_number  # 踏步数 个
    top_top_length: int = parameter.geometric_detailed.top_top_length  # 顶端上边长
    top_thickness: int = parameter.geometric_detailed.top_thickness  # 顶端板厚
    top_b: int = parameter.geometric_detailed.top_b  # 顶端挑耳宽度
    top_bottom_length: float = (
        top_top_length + (top_thickness / tan) - (steps_h / tan + thickness / sin)
    )  # 顶端下边长

    bottom_top_length: int = parameter.geometric_detailed.bottom_top_length  # 底端上边长
    bottom_thickness: int = parameter.geometric_detailed.bottom_thickness  # 底端板厚
    bottom_b: int = parameter.geometric_detailed.bottom_b  # 底端挑耳宽度
    bottom_bottom_length: float = (
        bottom_top_length - bottom_thickness / tan
    ) + thickness / sin  # 底端下边长
    l_total = ln + top_top_length + bottom_top_length  # 深化设计—楼梯跨度
    h_total = height + bottom_thickness  # 楼梯总高度
    # 吊装和脱模的荷载值
    volume = get_volume(
        steps_number=steps_number,
        thickness=thickness,
        cos=cos,
        steps_h=steps_h,
        steps_b=steps_b,
        b0=b0,
        top_top_length=top_top_length,
        top_thickness=top_thickness,
        top_b=top_b,
        bottom_top_length=bottom_top_length,
        bottom_thickness=bottom_thickness,
        bottom_b=bottom_b,
        top_bottom_length=top_bottom_length,
    )
    g_r = structure_design.load_data.railing_load  # 栏杆荷载标准值
    g_kt = volume * rc  # 自重标准值
    g_k = g_kt + g_r
    # 吊装预埋件的选型和定位
    g_dk = 1.5 * g_kt  # 吊装预埋件自重标准值乘以动力系数
    single_capacity_lifting = g_dk / 3 / 10  # 单个吊装预埋件承载力 /10是将kn转化为t
    # 计算模式
    if (
        parameter.inserts_detailed.lifting_design_mode == LiftingDesignMode.AUTOMATIC
    ):  # 程序自动计算
        # 首先判断吊装预埋件的类型
        # parameter.inserts_detailed.lifting_type 默认为 LiftingType.ROUNDING_HEAD # 吊装预埋件类型
        for rounding_head_dict in ROUNDING_HEAD:
            lifting_capacity_check = ROUNDING_HEAD[rounding_head_dict]["capacity"]
            if single_capacity_lifting <= lifting_capacity_check:
                parameter.inserts_detailed.lifting_name = ROUNDING_HEAD[
                    rounding_head_dict
                ]["name"]
                rounding_head: dict = ROUNDING_HEAD.get(
                    parameter.inserts_detailed.lifting_name, None
                )
                lifting_parameter = RoundHeadParameter(**rounding_head)  # 吊装预埋件参数
                break
        # 吊点验算
        calculate_length = 0.207 * l_total
        if calculate_length > min(
            top_top_length, bottom_top_length
        ):  # 保证按照最大正负弯矩计算的结果不放置在顶端和底端平板上
            steps_location = (
                round((calculate_length - bottom_top_length) / steps_b) + 1
            )  # 第几阶
        else:
            steps_location = 1  # 第1阶
        parameter.inserts_detailed.lifting_position = LiftingPosition(
            a=steps_number - steps_location,
            b=steps_location,
            c=int(0.207 * b0 / 10) * 10,
            d=int(0.207 * b0 / 10) * 10,
        )
    else:
        if parameter.inserts_detailed.lifting_type == LiftingType.ROUNDING_HEAD:
            rounding_head: dict = ROUNDING_HEAD.get(
                parameter.inserts_detailed.lifting_name, None
            )
            lifting_parameter = RoundHeadParameter(**rounding_head)  # 吊装预埋件参数
        else:  # parameter.inserts_detailed.lifting_type == liftingType.ANCHOR:
            anchor: dict = ANCHOR.get(parameter.inserts_detailed.lifting_name, None)
            lifting_parameter = AnchorParameter(**anchor)  # 吊装预埋件参数

    # 验算是否吊点是否发生开裂
    lifting_location_step_a = parameter.inserts_detailed.lifting_position.a
    lifting_location_step_b = parameter.inserts_detailed.lifting_position.b
    # 最不利的边距
    edge_length = max(
        (lifting_location_step_b - 0.5) * steps_b + bottom_top_length,
        (steps_number - lifting_location_step_a - 0.5) * steps_b + top_bottom_length,
    )
    lifting_edge_xa = edge_length - 0.5 * steps_b  # a点的边距
    lifting_edge_xb = edge_length  # b点的边距
    lifting_edge_xc = edge_length + 0.5 * steps_b  # c点的边距
    m_lifting_ka = g_dk * ((lifting_edge_xa / 1000) ** 2)  # a点的弯矩标准值
    m_lifting_kb = g_dk * ((lifting_edge_xb / 1000) ** 2)  # b点的弯矩标准值
    w_lifting_a = b0 * (thickness**2) / 6  # a点的截面抵抗矩
    w_lifting_b = b0 * ((thickness + 0.5 * steps_b * sin) ** 2) / 6
    sigm_lifting_cka = m_lifting_ka * (1000**2) / w_lifting_a  # MPa
    sigm_lifting_ckb = m_lifting_kb * (1000**2) / w_lifting_b  # MPa
    concrete_parameter = ConcreteParameter.by_grade(
        structure_design.material.concrete_grade
    )
    lifting_f_tk = concrete_parameter.f_tk
    max_sigm = max(sigm_lifting_cka, sigm_lifting_ckb)
    # lifting_status_point = max_sigm <= lifting_f_tk

    # 脱模预埋件的选型和定位
    """
    capacity:为深化设计计算出来单脱模预埋件需要的承载力
    根据预埋件的承载力选择相应的预埋件,用于程序自动计算的选择，程序手动输入无需选择预埋件。
    step 1:首先判断脱模预埋件的计算模式：若为手动输入，无需选择，直接将所选择的预埋件信息输出；
            若为自动计算，则需要根据默认吊装预埋件类型根据楼梯承载力较大一点的吊装预埋件，并将该埋件的规格信息输出。
    :return: single_capacity_demolding
    """
    DEMOLDING_A = 1.5  # 脱模吸附力 1.5KN/mm^2
    area_s0 = (
        (b0 + top_b) * top_top_length
        + (b0 + bottom_b) * bottom_top_length
        + ln * b0
        + steps_h * b0 * (steps_number - 1)
        + steps_h * (b0 + top_b)
    ) / 1000000  # 踏步弯折侧面积

    # area_s1 = (top_bottom_length * (b0 + top_b) + bottom_bottom_length * (b0 + bottom_b) + (
    #         (height + top_thickness - bottom_thickness) / sin) * b0) / 1000000  # 踏步平板侧面积
    area_s2 = volume / b0  # 楼梯侧向面积
    g_ek = 1.5 * g_kt  # 依据规范条文，脱模验算时，等效静力荷载标准值不小于构件自重标准值的1.5倍
    if (
        parameter.inserts_detailed.pouring_way == PouringWay.HORIZONTAL_HORIZONTAL
    ):  # 卧式浇筑卧式脱模---吊点仍然放在平板侧
        q_k1 = 1.2 * g_ek + DEMOLDING_A * area_s0
        single_capacity_demolding = max(q_k1, g_ek) / 3 / 10  # 脱模预埋件单钉承载力

    elif (
        parameter.inserts_detailed.pouring_way == PouringWay.VERTICAL_VERTICAL
    ):  # 立式浇筑立式脱模
        q_k1 = 1.2 * g_ek + DEMOLDING_A * area_s2
        single_capacity_demolding = max(q_k1, g_ek) / 2 / 10  # 脱模预埋件单钉承载力
    else:
        # 立式浇筑卧式脱模，侧面2个预埋件翻转
        q_k1 = 1.2 * g_ek + DEMOLDING_A * area_s0
        single_capacity_demolding = max(q_k1, g_ek) / 2 / 10  # 脱模预埋件单钉承载力

    # 计算模式
    if (
        parameter.inserts_detailed.demolding_design_mode
        == DemoldingDesignMode.AUTOMATIC
    ):  # 程序自动计算
        # 首先判断脱模预埋件的类型
        # parameter.inserts_detailed.demold_type 默认为 liftingType.ROUNDING_HEAD # 吊装预埋件类型
        for rounding_head_dict in ROUNDING_HEAD:
            demolding_capacity_check = ROUNDING_HEAD[rounding_head_dict]["capacity"]
            if single_capacity_demolding <= demolding_capacity_check:
                parameter.inserts_detailed.demolding_name = ROUNDING_HEAD[
                    rounding_head_dict
                ]["name"]
                rounding_head: dict = ROUNDING_HEAD.get(
                    parameter.inserts_detailed.demolding_name, None
                )
                demolding_parameter = RoundHeadParameter(**rounding_head)  # 脱模预埋件参数
                break
        xie_length = steps_b / cos
        area_1 = thickness * xie_length
        area_2 = steps_b * steps_h / 2
        centroid = (
            (area_1 * thickness / 2) + (thickness + area_2 / xie_length / 3) * area_2
        ) / (
            area_1 + area_2
        )  # 形心
        # a 脱模预埋件顶端纵向边距 b 脱模预埋件底端纵向边距 c  脱模预埋件左侧横向边距 d 脱模预埋件右侧横向边距 板中心
        parameter.inserts_detailed.demolding_position = DemoldingPosition(
            a=edge_length,
            b=edge_length,
            c=int(0.207 * b0 / 10) * 10,
            d=int(0.207 * b0 / 10) * 10,
            t=int(thickness / 2),
        )  # t=int(centroid))
    else:
        if parameter.inserts_detailed.demolding_type == DemoldingType.ROUNDING_HEAD:
            rounding_head: dict = ROUNDING_HEAD.get(
                parameter.inserts_detailed.demolding_name, None
            )
            demolding_parameter = RoundHeadParameter(**rounding_head)  # 吊装预埋件参数
        else:  # parameter.inserts_detailed.demolding_type == DemoldingType.ANCHOR:
            anchor: dict = ANCHOR.get(parameter.inserts_detailed.demolding_name, None)
            demolding_parameter = AnchorParameter(**anchor)  # 吊装预埋件参数
        # else:  # parameter.inserts_detailed.demold_type == DemoldType.LIFT_HOOK:
        #     lift_hook: dict = LIFT_HOOK.get(parameter.inserts_detailed.lifting_name, None)
        #     demolding_parameter = LiftHookParameter(**lift_hook)  # 吊装预埋件参数

    step_slot_edge = 10  # 防滑槽边距
    if (
        parameter.construction_detailed.step_slot_design_mode
        == StepSlotDesignMode.MANUAL
    ):
        if parameter.inserts_detailed.rail_design_mode == RailDesignMode.MANUAL:
            # 获取栏杆数据
            rail_name = parameter.inserts_detailed.rail_name
            rail: dict = RAILING.get(rail_name, None)
            rail_parameter = RailParameter(**rail)
            rail_b = rail_parameter.b  # 栏杆b方向长度，即与楼梯横向一致
            rail_a = rail_parameter.a  # 栏杆a方向长度，即与楼梯纵向一致
            rail_y = (steps_b - rail_a) / 2  # 栏杆外边缘距离踏步外边缘的纵向距离
            edge_y = (
                parameter.construction_detailed.step_slot.a
                + parameter.construction_detailed.step_slot.b
            ) * 2 + parameter.construction_detailed.step_slot.c
            edge_x = (
                rail_b + parameter.inserts_detailed.rail_position.a + step_slot_edge
            )
            if rail_y >= edge_y:  # 栏杆的纵向边距大于防滑槽在踏步上的纵向边距,无需调整长度
                pass
            else:
                parameter.inserts_detailed.rail_position.a = edge_x  # 调整防滑槽的横向边距
                parameter.inserts_detailed.rail_position.b = edge_x  # 调整防滑槽的横向边距
        else:
            rail_parameter = None
    else:
        if parameter.inserts_detailed.rail_design_mode == RailDesignMode.MANUAL:
            # 获取栏杆数据
            rail_name = parameter.inserts_detailed.rail_name
            rail: dict = RAILING.get(rail_name, None)
            rail_parameter = RailParameter(**rail)
        else:
            rail_parameter = None

    c = DetailedDesignResult(
        detailed_design=parameter,
        top_bottom_length=top_bottom_length,
        bottom_bottom_length=bottom_bottom_length,
        l_total=l_total,
        h_total=h_total,
        v=volume,
        cos=cos,
        sin=sin,
        tan=tan,
        gkt=g_kt,
        gk=g_k,
        gdk=g_dk,
        gek=g_ek,
        single_capacity_lifting=single_capacity_lifting,
        lifting_type=parameter.inserts_detailed.lifting_type,
        lifting_name=parameter.inserts_detailed.lifting_name,
        lifting_parameter=lifting_parameter,
        lifting_edge_xa=lifting_edge_xa,
        lifting_edge_xb=lifting_edge_xb,
        lifting_edge_xc=lifting_edge_xc,
        m_lifting_ka=m_lifting_ka,
        m_lifting_kb=m_lifting_kb,
        w_lifting_a=w_lifting_a,
        w_lifting_b=w_lifting_b,
        sigm_lifting_cka=sigm_lifting_cka,
        sigm_lifting_ckb=sigm_lifting_ckb,
        max_sigm=max_sigm,
        lifting_f_tk=lifting_f_tk,
        pouring_way=parameter.inserts_detailed.pouring_way,
        q_k1=q_k1,
        single_capacity_demolding=single_capacity_demolding,
        demolding_type=parameter.inserts_detailed.demolding_type,
        demolding_name=parameter.inserts_detailed.demolding_name,
        demolding_parameter=demolding_parameter,
        rail_parameter=rail_parameter,
    )
    return c


def to_word_detailed(
    data: DetailedCalculationBook, file: IO[bytes]
) -> Tuple[IO[bytes], Any]:
    """
    基于word模板渲染生成word文件
    :param data:
    :param file:
    :return:
    """
    doc = DocxTemplate(file)
    detailed_result = data.detailed_design_result
    detailed = data.detailed_design_result.detailed_design
    structure_result = data.structural_design_result
    structure_parameter = data.structural_design
    concrete_parameter = ConcreteParameter.by_grade(
        data.structural_design.material.concrete_grade
    )
    rebar_parameter = RebarParameter.by_name(data.structural_design.material.rebar_name)
    context = {
        "detailed_cal_book": {
            "project_ID": structure_parameter.stair_id.project_ID,
            "stair_ID": structure_parameter.stair_id.stair_ID,
            "Ln": structure_parameter.geometric.clear_span,
            "H": structure_parameter.geometric.height,
            "t": structure_parameter.geometric.thickness,
            "steps_number": structure_parameter.geometric.steps_number,
            "b0": detailed.geometric_detailed.width,
            "L_t": detailed.geometric_detailed.top_top_length,
            "L_b": detailed.geometric_detailed.bottom_top_length,
            "h1": detailed.geometric_detailed.top_thickness,
            "h2": detailed.geometric_detailed.bottom_thickness,
            "b1": detailed.geometric_detailed.top_b,
            "b2": detailed.geometric_detailed.bottom_b,
            "q_qk": structure_parameter.load_data.live_load,
            "g_f": structure_parameter.load_data.railing_load,
            "concrete_name": concrete_parameter.name,
            "rc": structure_parameter.load_data.reinforced_concrete_bulk_density,
            "concrete_f_tk": concrete_parameter.f_tk,
            "rebar_name": rebar_parameter.name,
            "steps_h": structure_result.steps_h,
            "steps_b": structure_result.steps_b,
            "l1t": detailed_result.top_bottom_length,
            "l1b": detailed_result.bottom_bottom_length,
            "L0": detailed_result.l_total,
            "v": detailed_result.v,
            "cos": detailed_result.cos,
            "g_kt": detailed_result.gkt,
            "g_k": detailed_result.gk,
            "g_dk": detailed_result.gdk,
            "g_ek": detailed_result.gek,
            "single_capacity_lifting": detailed_result.single_capacity_lifting,
            "lifting_type": detailed_result.lifting_type,
            "lifting_name": detailed_result.lifting_name,
            "lifting_edge_xa": detailed_result.lifting_edge_xa,
            "lifting_edge_xb": detailed_result.lifting_edge_xb,
            "lifting_edge_xc": detailed_result.lifting_edge_xc,
            "m_lifting_ka": detailed_result.m_lifting_ka,
            "m_lifting_kb": detailed_result.m_lifting_kb,
            "w_lifting_a": detailed_result.w_lifting_a,
            "w_lifting_b": detailed_result.w_lifting_b,
            "sigm_lifting_cka": detailed_result.sigm_lifting_cka,
            "sigm_lifting_ckb": detailed_result.sigm_lifting_ckb,
            "max_sigm": detailed_result.max_sigm,
            "lifting_f_tk": detailed_result.lifting_f_tk,
            "pouring_way": detailed_result.pouring_way,
            "q_k1": detailed_result.q_k1,
            "single_capacity_demolding": detailed_result.single_capacity_lifting,
            "demolding_type": detailed_result.lifting_type,
            "demolding_name": detailed_result.lifting_name,
        }
    }
    doc.render(context)  # 转换成字典使用
    bytes_back = BytesIO()
    doc.save(bytes_back)
    bytes_back.seek(0)
    set_of_variables = doc.get_undeclared_template_variables()
    return bytes_back, set_of_variables
