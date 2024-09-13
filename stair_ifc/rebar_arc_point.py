import math

import numpy as np
from typing import List
from dataclasses import dataclass, field

from dc_rebar import Rebar


@dataclass
class Seg(object):
    """
    用于描述一段钢筋是直线还是圆弧
    """

    index_s: List
    seg_type: str = "IfcLineIndex"  # "IfcLineIndex" 表示直线,"IfcArcIndex" 表示圆弧


@dataclass
class IfcRebarData(object):
    radius: float
    points: List[List[float]] = field(default_factory=list)
    seg_s: List[Seg] = field(default_factory=list)


def rebar_arc(rebar: Rebar, rebar_grade=1):
    radius = rebar.radius
    if rebar_grade == 1:
        bending_radius = 1.25 * radius
    elif rebar_grade == 2:
        bending_radius = 2.0 * radius
    elif rebar_grade == 3:
        bending_radius = 2.5 * radius
    else:
        bending_radius = 2.5 * radius

    points = rebar.poly.points
    poly_curve_points = []
    seg_s = []
    if len(rebar.poly.segments) == 0:
        poly_curve_points.append(
            [float(points[0].x), float(points[0].y), float(points[0].z)]
        )
        for i in range(1, len(points) - 1):
            # 定义原本三个点
            point_mid: np.ndarray = np.asarray([points[i].x, points[i].y, points[i].z])
            point_next: np.ndarray = np.asarray(
                [points[i + 1].x, points[i + 1].y, points[i + 1].z]
            )  # 下一个点
            last_point = np.asarray(poly_curve_points[-1])
            # 计算向量
            v_m_2_s: np.ndarray = (last_point - point_mid) / np.linalg.norm(
                last_point - point_mid
            )  # 单位化
            v_m_2_e: np.ndarray = (point_next - point_mid) / np.linalg.norm(
                point_next - point_mid
            )  # 单位化

            # 计算角度cos 和tan 值
            radius_cos = np.dot(v_m_2_s, v_m_2_e)
            if radius_cos < -1 and radius_cos > 1:
                print(radius_cos)

            radius_use = math.acos(radius_cos) / 2
            tan = math.tan(radius_use)
            sin_ = math.sin(radius_use)
            # 计算弯折中心点
            v_2_center = v_m_2_e + v_m_2_s
            v_2_c_normalzise = v_2_center / np.linalg.norm(v_2_center)
            center = point_mid + v_2_c_normalzise * (bending_radius / sin_)  # 中心点
            # 计算圆弧上的三个点
            mid_new_np = center - v_2_c_normalzise * bending_radius
            start_new_np = point_mid + v_m_2_s * (bending_radius / tan)
            end_new_np = point_mid + v_m_2_e * (bending_radius / tan)
            poly_curve_points.append(start_new_np.tolist())
            start_index = len(poly_curve_points) - 1
            poly_curve_points.append(mid_new_np.tolist())
            poly_curve_points.append(end_new_np.tolist())
            # 定义直线段和弯折段
            seg_s.append(
                Seg(index_s=[start_index, start_index + 1], seg_type="IfcLineIndex")
            )
            seg_s.append(
                Seg(
                    index_s=[start_index + 1, start_index + 2, start_index + 3],
                    seg_type="IfcArcIndex",
                )
            )

        poly_curve_points.append(
            [float(points[-1].x), float(points[-1].y), float(points[-1].z)]
        )
        index_use = len(poly_curve_points) - 1
        seg_s.append(Seg(index_s=[index_use, index_use + 1], seg_type="IfcLineIndex"))
    else:
        for p in points:
            poly_curve_points.append([float(p.x), float(p.y), float(p.z)])
        for segment in rebar.poly.segments:
            if len(segment) == 3:
                seg_s.append(Seg(index_s=segment, seg_type="IfcArcIndex"))
            else:
                seg_s.append(Seg(index_s=segment, seg_type="IfcLineIndex"))

    new_rebar = IfcRebarData(
        radius=float(radius), points=poly_curve_points, seg_s=seg_s
    )
    return new_rebar
