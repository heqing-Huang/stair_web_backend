from dataclasses import dataclass
from typing import Optional
import math
import numpy as np
import matplotlib.pyplot as plt

start_distance = 50  # 销键加强筋的起步间距
LENGTH = 300  # 孔洞加强筋平直段长度
RADIUS = 60  # 孔洞加强筋弯曲半径
hole_rebar_objs = []  # fcl模型
hole_a1 = 100  # 顶端右侧孔洞纵向边距
hole_b1 = 300  # 顶端右侧孔洞横向边距
hole_a2 = 100  # 顶端左侧孔洞纵向边距
hole_b2 = 300  # 顶端左侧孔洞横向边距
hole_a3 = 100  # 底端右侧孔洞纵向边距
hole_b3 = 300  # 底端右侧孔洞横向边距
hole_a4 = 100  # 底端左侧孔洞纵向边距
hole_b4 = 300  # 底端左侧孔洞横向边距

width = 2000
l_total = 5000
h_total = 1000
cover = 20
top_thickness = 200
bottom_thickness = 200

top_rebar_x = [hole_b2, width - hole_b1]
top_rebar_y = [l_total - hole_a2, l_total - hole_a1]
top_rebar_z = [h_total - top_thickness + start_distance, h_total - start_distance]
bottom_rebar_x = [hole_b4, width - hole_b3]
bottom_rebar_y = [hole_a4, hole_a3]
bottom_rebar_z = [start_distance, bottom_thickness - start_distance]

top_hole_rebar_fcl = np.array(
    [
        [-RADIUS, -LENGTH, 0],
        [-RADIUS, RADIUS, 0],
        [RADIUS, RADIUS, 0],
        [RADIUS, -LENGTH, 0],
    ]
)
bottom_hole_rebar_fcl = np.array(
    [
        [-RADIUS, LENGTH, 0],
        [-RADIUS, -RADIUS, 0],
        [RADIUS, -RADIUS, 0],
        [RADIUS, LENGTH, 0],
    ]
)
original_direction = np.array([[0, 0, 1]])

from dc_rebar import Rebar, IndexedPolyCurve, Point3D

hole_rebar_BIM = []
top_hole_rebar_BIM = np.array(
    [
        [-RADIUS, -LENGTH, 0],  # 半圆的三个控制点
        [-RADIUS, 0, 0],
        [0, RADIUS, 0],
        [RADIUS, 0, 0],
        [RADIUS, -LENGTH, 0],
    ]
)
bottom_hole_rebar_BIM = np.array(
    [
        [-RADIUS, LENGTH, 0],
        [-RADIUS, 0, 0],
        [0, -RADIUS, 0],
        [RADIUS, 0, 0],
        [RADIUS, LENGTH, 0],
    ]
)
hole_rebar_diameter = 10
for z in top_rebar_z:
    for x, y in zip(top_rebar_x, top_rebar_y):
        rebar_points = (top_hole_rebar_BIM + np.array([x, y, z])).tolist()
        hole_rebar_BIM.append(
            Rebar(
                radius=int(hole_rebar_diameter / 2),
                poly=IndexedPolyCurve(
                    points=rebar_points, segments=[[1, 2], [2, 3, 4], [4, 5]]
                ),
            )
        )
# print(hole_rebar_BIM[0])
