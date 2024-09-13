"""
Date&Time           2022/8/3 11:26
Author              HaoLan

"""
import math
import logging
from io import BytesIO
from dataclasses import dataclass, field
from typing import Optional, IO, Tuple, Any

import numpy as np
from docxtpl import DocxTemplate

from .model import (
    StructuralDesign,
    StructuralDesignResult,
    _StructuralDesignResult,
    ConcreteParameter,
    RebarParameter,
    RelativeLimitZone,
    CalculationBook,
    _CalculationBook,
)

from .tools import find_area_table

_logger = logging.getLogger(__name__)


def structure_cal(parameter: StructuralDesign) -> StructuralDesignResult:
    """
    该部分需要完成结构计算的逻辑,并返回结构计算对应的数据
    :param parameter:
    :return:
    """

    # 输入参数得到的数据
    # 几何信息
    ln = parameter.geometric.clear_span  # 预制楼梯净跨 mm
    total_h = parameter.geometric.height  # 预制楼梯高度 mm
    steps_number = parameter.geometric.steps_number  # 踏步数 个
    t = parameter.geometric.thickness  # 预制楼梯踏步板厚度 mm
    lt = parameter.geometric.top_top_length  # 顶端上边长 mm
    lb = parameter.geometric.bottom_top_length  # 底端上边长 mm
    crack_limit = parameter.limit_setting.crack  # 裂缝限值信息
    # 材料信息
    concrete_grade = parameter.material.concrete_grade  # 混凝土强度等级 无量纲
    concrete_parameter = ConcreteParameter.by_grade(concrete_grade)  # 混凝土参数-类

    concrete_f_c = concrete_parameter.f_c  # 混凝土轴心抗压强度设计值 N/mm^2
    concrete_f_t = concrete_parameter.f_t  # 混凝土轴心抗拉强度设计值 N/mm^2
    concrete_f_tk = concrete_parameter.f_tk  # 混凝土轴心抗拉强度标准值 N/mm^2
    concrete_ec = concrete_parameter.ec  # 混凝土弹性模量 N/mm^2
    concrete_a_ref = concrete_parameter.a_erf  #
    rebar_name = parameter.material.rebar_name  # 钢筋强度等级 无量纲
    rebar_parameter = RebarParameter.by_name(rebar_name)
    rebar_grade = rebar_parameter.grade  # 钢筋等级

    rebar_f_y = rebar_parameter.steel.f_y  # 钢筋抗拉强度设计值 N/mm^2
    rebar_es = rebar_parameter.steel.es  # 钢筋的弹性模量 N/mm^2
    # 构造信息
    concrete_cover = parameter.construction.concrete_cover_thickness  # 保护层厚度 mm
    a_s = parameter.construction.longitudinal_top_rebar_distance  # 踏步板纵筋合力点至近边距离 mm
    # 荷载信息
    q_qk = parameter.load_data.live_load  # 正常使用活荷载 kN/m^2 可变荷载
    railing_load = parameter.load_data.railing_load  # 栏杆荷载 kN/m  0.2
    r_g = parameter.load_data.permanent_load_partial_factor  # 永久荷载分项系数 无量纲 1.3
    r_q = parameter.load_data.live_load_load_partial_factor  # 可变荷载分项系数 无量纲 1.5
    # 准永久值系数 无量纲  参考《建筑结构荷载规范》 GB50009-2012 表5.1.1 民用建筑楼面均布活荷载标准值及其组合值、频遇值和准永久值系数
    fi_q = parameter.load_data.quasi_permanent_factor  # 准永久值系数
    fi_c = parameter.load_data.combined_factor  # 组合值系数 无量纲  查表求得
    rc = parameter.load_data.reinforced_concrete_bulk_density  # 钢筋混凝土容重

    # 约定的数据或通过规范查询的数据

    # 结构设计阶段三类钢筋直径和间距限值
    MIN_DIA_BOTTOM = 10  # mm 下部纵筋最小直径
    MAX_SPACI_BOTTOM = 200  # mm 下部纵筋最大间距
    MIN_DIA_TOP = 8  # mm 上部纵筋最小直径
    MAX_SPACI_TOP = 200  # mm 上部纵筋最大间距
    MIN_DIA_DIS = 8  # mm 中部分布筋最小直径
    MAX_SPACI_DIS = 250  # mm 中部分布筋最大间距
    STATIC_B = 1  # 单位板宽m
    alpha_cr = 1.9  # 构件受力特征系数 查《混规》 无量纲系数
    gama_f = 0  # 受压翼缘面积与腹板有效面积比值 γf ——无翼缘
    # 《混规》7.2.5-1条文规定的两个值
    theta_i = 2  # 考虑荷载长期效应组合对挠度影响增大影响系数上限"
    theta_e = 1.6  # 考虑荷载长期效应组合对挠度影响增大影响系数下限"

    # 计算出的数据，存储并返回

    steps_h = total_h / steps_number  # 踏步高度 mm
    steps_b = ln / (steps_number - 1)  # 踏步宽度 mm
    l0 = ln + lt + lb  # 计算跨度 mm
    cos = steps_b / (steps_b**2 + steps_h**2) ** 0.5  # 踏步余弦值 无量纲
    # 踏步板荷载
    gkt = rc * STATIC_B * (t / cos + steps_h / 2) / 1000  # 自重 kN/m
    gk = gkt + railing_load  # 恒载标准值 kN/m
    png = 1.35 * gk + r_q * fi_c * STATIC_B * q_qk  # 恒荷控制
    pnl = r_g * gk + r_q * STATIC_B * q_qk  # 活荷控制
    pm = max(png, pnl)  # 荷载设计值
    # 正截面受弯承载力计算
    m_max = pm * ((l0 / 1000) ** 2) / 8  # kN*m 跨中最大弯矩
    h0 = t - a_s  # 有效受压区高度  mm
    alpha_s = (
        m_max
        * (10**6)
        / (concrete_a_ref * concrete_f_c * STATIC_B * 1000 * (h0**2))
    )
    mid_as_check = 1 - 2 * alpha_s
    ksi = 1 - mid_as_check**0.5
    ksi_b = RelativeLimitZone.by_rebar_concrete(
        rebar_grade=str(rebar_grade), concrete_grade=str(concrete_grade)
    ).ksi_b
    if mid_as_check > 0:
        ksi_status = True
    else:
        ksi_status = ksi < ksi_b  # 防止发生超筋破坏
    if not ksi_status:
        raise Exception("ksi_status 校验失败")
    # 计算配筋
    #  实配1号钢筋面积 mm^2
    as_1 = concrete_a_ref * concrete_f_c * STATIC_B * 1000 * h0 * ksi / rebar_f_y
    p_c = as_1 / (STATIC_B * 1000 * h0)  # 实际配筋率
    p_c_min = max(0.002, 0.45 * concrete_f_t / rebar_f_y)  # 最小配筋率
    as_2 = p_c_min * STATIC_B * 1000 * t  # 2号钢筋按照构造配筋，即最小配筋面积 mm^2
    as_3 = max(0.15 * as_1, 0.0015 * STATIC_B * 1000 * t)  # 分布钢筋配筋
    #  实配1号钢筋面积 mm^2 直径 mm, 间距 mm
    as_fact_1, d_fact_1, spacing_fact_1 = find_area_table(
        as_1, MIN_DIA_BOTTOM, MAX_SPACI_BOTTOM
    )
    #  实配2号钢筋面积 mm^2 直径 mm, 间距 mm
    as_fact_2, d_fact_2, spacing_fact_2 = find_area_table(
        as_2, MIN_DIA_TOP, MAX_SPACI_TOP
    )
    #  实配3号钢筋面积 mm^2 直径 mm, 间距 mm
    as_fact_3, d_fact_3, spacing_fact_3 = find_area_table(
        as_3, MIN_DIA_DIS, MAX_SPACI_DIS
    )
    # 挠度计算
    mq = (gk * fi_q * r_q) * ((l0 / 1000) ** 2) / 8  # 永久组合弯矩值Mq kN*m
    sigma_sq = mq * (10**6) / (0.87 * h0 * as_fact_1)  # 纵向受拉钢筋应力
    a_te = 0.5 * STATIC_B * 1000 * t  # 矩形截面面积 mm^2
    p_te = as_fact_1 / a_te
    fi_i = 1.1 - 0.65 * concrete_f_tk / (p_te * sigma_sq)  # 受拉钢筋应变不均匀系数
    # 根据《混规》7.1.2-2调整受拉钢筋应变不均匀系数
    fi = fi_i
    if fi < 0.2:
        fi = 0.2
    elif fi > 1.0:
        fi = 1.0
    alpha_e = rebar_es / concrete_ec  # 无量纲
    p_t = as_fact_1 / (STATIC_B * 1000 * h0)  # 纵向受拉钢筋配筋率ρ
    b_s = (
        rebar_es
        * as_fact_1
        * (h0**2)
        / (10**6)
        / (1.15 * fi + 0.2 + 6 * alpha_e * p_t / (1 + 3.5 * gama_f))
    ) / 1000
    m_theta = as_fact_2 / as_fact_1  # 受压钢筋面积与受拉钢筋面积比值
    theta = theta_i - m_theta * (theta_i - theta_e)  # 考虑荷载长期效应组合对挠度影响增大影响系数 θ"
    b_l = b_s / theta  # 受弯构件长期刚度B
    deflection_maxk = (5 * mq * ((l0 / 1000) ** 2) / (48 * b_l)) * 1000  # 单位换算 mm
    # 挠度限值计算
    deflection_limit = l0 / 200
    deflection_status = deflection_maxk < deflection_limit
    if not deflection_status:
        raise Exception("挠度验算不满足规范")

    # 裂缝验算
    if rebar_name != "HPB300":  # 若不为光圆钢筋
        v_i = 1  # 通过是否为带肋钢筋决定 Vi
    else:
        v_i = 0.7  # 查《混规》7.1.2-2表
    # 《混规》7.1.2条文
    if concrete_cover < 20:
        c_s = 20  # 最外层纵向受拉钢筋外边缘至受拉区底边的距离
    elif concrete_cover > 65:
        c_s = 65  # 最外层纵向受拉钢筋外边缘至受拉区底边的距离
    else:
        c_s = concrete_cover
    if p_te < 0.01:
        p_te_w = 0.01
    else:
        p_te_w = p_te
    fi_w_i = 1.1 - 0.65 * concrete_f_tk / (p_te_w * sigma_sq)  # 用于裂缝验算的受拉钢筋应变不均匀系数
    # 根据《混规》7.1.2-2调整受拉钢筋应变不均匀系数
    fi_w = fi_w_i
    if fi_w < 0.2:
        fi_w = 0.2
    elif fi_w > 1.0:
        fi_w = 1.0
    rebar_n = int((STATIC_B * 1000 / spacing_fact_1) + 1)
    d_eq = (rebar_n * (d_fact_1**2)) / (rebar_n * v_i * d_fact_1)  # 受拉区纵向钢筋的等效直径
    crack_max = (
        alpha_cr * fi * sigma_sq * (1.9 * c_s + 0.08 * d_eq / p_te_w) / rebar_es
    )  # 最大裂缝宽度mm
    crack_status = crack_max <= crack_limit
    if not crack_status:
        raise Exception("裂缝验算不满足规范要求")
    result = _StructuralDesignResult(
        steps_h=steps_h,
        steps_b=steps_b,
        l0=l0,
        cos=cos,
        gkt=gkt,
        gk=gk,
        png=png,
        pnl=pnl,
        pm=pm,
        m_max=m_max,
        concrete_parameter=concrete_parameter,
        rebar_parameter=rebar_parameter,
        h0=h0,
        alpha_s=alpha_s,
        ksi=ksi,
        ksi_b=ksi_b,
        ksi_status=ksi_status,
        p_c_min=p_c_min,
        p_c=p_c,
        as_1=as_1,
        as_2=as_2,
        as_3=as_3,
        as_1_actual=as_fact_1,
        as_2_actual=as_fact_2,
        as_3_actual=as_fact_3,
        d_1_actual=d_fact_1,
        d_2_actual=d_fact_2,
        d_3_actual=d_fact_3,
        spacing_1_actual=spacing_fact_1,
        spacing_2_actual=spacing_fact_2,
        spacing_3_actual=spacing_fact_3,
        mq=mq,
        sigma_sq=sigma_sq,
        a_te=a_te,
        p_te=p_te,
        fi_i=fi_i,
        fi=fi,
        fi_w=fi_w,
        fi_w_i=fi_w_i,
        alpha_e=alpha_e,
        gama_f=gama_f,
        p_t=p_t,
        b_s=b_s,
        b_l=b_l,
        theta=theta,
        m_theta=m_theta,
        deflection_maxk=deflection_maxk,
        deflection_limit=deflection_limit,
        deflection_status=deflection_status,
        v_i=v_i,
        rebar_n=rebar_n,
        c_s=c_s,
        p_te_w=p_te_w,
        d_eq=d_eq,
        crack_max=crack_max,
        crack_status=crack_status,
    )
    return result.to_out()


def to_word(data_out: CalculationBook, file: IO[bytes]) -> Tuple[IO[bytes], Any]:
    """
    基于word模板渲染生成word文件
    :param data_out:
    :param file:
    :return:
    """
    data: _CalculationBook = _CalculationBook.from_out(data_out)
    doc = DocxTemplate(file)
    parameter = data.structure_design
    result = data.structure_design_result
    context = {
        "cal_book": {
            "project_ID": parameter.stair_id.project_ID,
            "stair_ID": parameter.stair_id.stair_ID,
            "Ln": parameter.geometric.clear_span,
            "H": parameter.geometric.height,
            "t": parameter.geometric.thickness,
            "steps_number": parameter.geometric.steps_number,
            "L_t": parameter.geometric.top_top_length,
            "L_b": parameter.geometric.bottom_top_length,
            "q_qk": parameter.load_data.live_load,
            "g_f": parameter.load_data.railing_load,
            "r_g": parameter.load_data.permanent_load_partial_factor,
            "r_q": parameter.load_data.live_load_load_partial_factor,
            "fi_q": parameter.load_data.quasi_permanent_factor,
            "fi_c": parameter.load_data.combined_factor,
            "concrete_name": result.concrete_parameter.name,
            "concrete_f_c": result.concrete_parameter.f_c,
            "concrete_f_t": result.concrete_parameter.f_t,
            "rc": parameter.load_data.reinforced_concrete_bulk_density,
            "concrete_f_tk": result.concrete_parameter.f_tk,
            "concrete_ec": result.concrete_parameter.ec,
            "rebar_name": result.rebar_parameter.name,
            "rebar_f_y": result.rebar_parameter.steel.f_y,
            "rebar_es": result.rebar_parameter.steel.es,
            "rebar_symbol": result.rebar_parameter.symbol,
            "concrete_cover": parameter.construction.concrete_cover_thickness,
            "a_s": parameter.construction.longitudinal_top_rebar_distance,
            "steps_h": result.steps_h,
            "steps_b": result.steps_b,
            "L0": result.l0,
            "cos": result.cos,
            "g_kt": result.gkt,
            "g_k": result.gk,
            "pnl": result.pnl,
            "png": result.png,
            "pm": result.pm,
            "m_max": result.m_max,
            "h0": result.h0,
            "alpha_s": result.alpha_s,
            "ksi": result.ksi,
            "ksi_b": result.ksi_b,
            "as_1": result.as_1,
            "as_2": result.as_2,
            "as_3": result.as_3,
            "p_c_min": result.p_c_min,
            "p_c": result.p_c,
            "as_fact_1": result.as_1_actual,
            "as_fact_2": result.as_2_actual,
            "as_fact_3": result.as_3_actual,
            "d_fact_1": result.d_1_actual,
            "d_fact_2": result.d_2_actual,
            "d_fact_3": result.d_3_actual,
            "spacing_fact_1": result.spacing_1_actual,
            "spacing_fact_2": result.spacing_2_actual,
            "spacing_fact_3": result.spacing_3_actual,
            "mq": result.mq,
            "sigma_sq": result.sigma_sq,
            "a_te": result.a_te,
            "p_te": result.p_te,
            "fi_i": result.fi_i,
            "fi": result.fi,
            "fi_w": result.fi_w,
            "fi_w_i": result.fi_w_i,
            "alpha_e": result.alpha_e,
            "gama_f": result.gama_f,
            "p_t": result.p_t,
            "b_s": result.b_s,
            "m_theta": result.m_theta,
            "theta": result.theta,
            "b_l": result.b_l,
            "f_maxk": result.deflection_maxk,
            "f_0": result.deflection_limit,
            "v_i": result.v_i,
            "c_s": result.c_s,
            "p_te_w": result.p_te_w,
            "rebar_n": result.rebar_n,
            "d_eq": result.d_eq,
            "w_max": result.crack_max,
            "w_limit": parameter.limit_setting.crack,
        }
    }
    doc.render(context)  # 转换成字典使用
    bytes_back = BytesIO()
    doc.save(bytes_back)
    bytes_back.seek(0)
    set_of_variables = doc.get_undeclared_template_variables()
    return bytes_back, set_of_variables
