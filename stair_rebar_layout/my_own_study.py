import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
import math

bottom_thickness = 160  # 底端板厚
top_thickness = 160  # 顶端板厚
bottom_top_length = 420  # 底端上边长
top_top_length = 420  # 顶端上边长
top_bottom_length = 210  # 顶端下边长 mm
bottom_bottom_length = 210  # 底端下边长 mm
steps_h = 150  # 踏步高度
steps_b = 300  # 踏步宽度
steps_number = 6  # 踏步数

ln = (steps_number - 1) * steps_b  # 净跨
l_total = 2340  # 计算跨度 mm
h_total = 1070  # 楼梯总高度

# 顶端挑耳宽度
top_b = 80
# 底端挑耳宽度
bottom_b = 0

# 添加底端板三角点
geometry_points = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, float(bottom_thickness)],
    [0.0, float(bottom_top_length), float(bottom_thickness)],
]
# 添加踏步角点
for i in range(steps_number - 1):
    geometry_points.append(
        [
            0.0,
            float(bottom_top_length + i * steps_b),
            float(bottom_thickness + (i + 1) * steps_h),
        ]
    )
    geometry_points.append(
        [
            0.0,
            float(bottom_top_length + (i + 1) * steps_b),
            float(bottom_thickness + (i + 1) * steps_h),
        ]
    )
geometry_points += [
    [0.0, float(bottom_top_length + ln), float(h_total)],  # 添加顶端板右上角点
    [0.0, float(l_total), float(h_total)],
]  # 添加顶端板左上角点

print(geometry_points)  # 15 points


def get_plt():
    x = []
    y = []
    plt.figure()
    for point in geometry_points:
        x.append(-point[1])
        y.append(point[2])
    plt.plot(x, y)
    plt.plot([-2340, -2340, -2130], [1070, 910, 910], c="r", linewidth=1)
    plt.plot([0, -210, -2130], [0, 0, 910], c="r", linewidth=1)
    plt.plot([-210, -2130], [0, 910], c="g", linewidth=3)
    plt.scatter(
        -(l_total - top_bottom_length + bottom_bottom_length) / 2,
        (h_total - top_thickness) / 2,
    )
    plt.plot([-420, -1920], [160, 920], "g", linewidth=3)
    plt.scatter(
        -(l_total - top_top_length + bottom_top_length) / 2,
        (h_total - steps_h + bottom_thickness) / 2,
    )
    plt.savefig("楼梯侧面.png")


from Fcl_models import Agent, Rebar_fcl, Box_fcl, Diagonal_fcl, Cylinder_fcl

width = 1500  # 梯段板宽度
cover = 20  # 保护层厚度（为什么减1？）

print(int((len(geometry_points) - 1) / 2))
cover_objs = []
for i in range(int((len(geometry_points) - 1) / 2)):
    cover_objs.append(
        Box_fcl(
            x=width,
            y=cover,
            z=geometry_points[2 * i + 1][2] - geometry_points[2 * i][2],
            position=np.array(
                [
                    width / 2,
                    (geometry_points[2 * i + 1][1] + cover + geometry_points[2 * i][1])
                    / 2,
                    (geometry_points[2 * i + 1][2] + geometry_points[2 * i][2]) / 2,
                ]
            ),
        )
    )
    cover_objs.append(
        Box_fcl(
            x=width,
            y=geometry_points[2 * i + 2][1] - geometry_points[2 * i + 1][1],
            z=cover,
            position=np.array(
                [
                    width / 2,
                    (geometry_points[2 * i + 2][1] + geometry_points[2 * i + 1][1]) / 2,
                    (
                        geometry_points[2 * i + 2][2]
                        + geometry_points[2 * i + 1][2]
                        - cover
                    )
                    / 2,
                ]
            ),
        )
    )
# print(cover_objs)

# 添加红色水平竖直方向包围盒模型
cover_objs.append(
    Box_fcl(
        x=float(width),
        y=float(cover),
        z=float(top_thickness),
        position=np.array(
            [width / 2, (2 * l_total - cover) / 2, (2 * h_total - top_thickness) / 2]
        ),
    )
)
cover_objs.append(
    Box_fcl(
        x=float(width),
        y=float(top_bottom_length),
        z=float(cover),
        position=np.array(
            [
                width / 2,
                (2 * l_total - top_bottom_length) / 2,
                (2 * h_total - 2 * top_thickness + cover) / 2,
            ]
        ),
    )
)
cover_objs.append(
    Box_fcl(
        x=float(width),
        y=float(bottom_bottom_length),
        z=float(cover),
        position=np.array([width / 2, bottom_bottom_length / 2, cover / 2]),
    )
)
# 添加往楼梯左右侧外部延伸200mm的包围盒模型
cover_objs.append(
    Box_fcl(
        x=float(cover + 200),
        y=float(l_total),
        z=float(h_total),
        position=np.array([(-200 + cover) / 2, l_total / 2, h_total / 2]),
    )
)
cover_objs.append(
    Box_fcl(
        x=float(cover + max(top_b, bottom_b) + 200),
        y=float(l_total),
        z=float(h_total),
        position=np.array(
            [
                (2 * width - cover + (max(top_b, bottom_b)) + 200) / 2,
                l_total / 2,
                h_total / 2,
            ]
        ),
    )
)

print(">---------------------------------------------")
print(f"cover_objs中一共有{len(cover_objs)}个Box_fcl对象")
print(">---------------------------------------------")


def rotation_matrix_from_vectors(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (
        vec2 / np.linalg.norm(vec2)
    ).reshape(3)
    v = np.cross(a, b)
    if any(v):  # 检查a, b向量是否平行
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))
    else:  # 若平行，则旋转矩阵是单位阵
        return np.eye(3)  # cross of all zeros only occurs on identical directions


import cv2

# 添加斜向长方体包围盒Diagonal_fcl
original_direction = np.array([0, 0, 1], float)
final_direction = np.array(
    [0, l_total - top_bottom_length - bottom_bottom_length, h_total - top_thickness],
    float,
)
print("ffffffffff", final_direction)
# t = cv2.Rodrigues(original_direction, final_direction)

transformation = rotation_matrix_from_vectors(original_direction, final_direction)
cover_objs.append(
    Diagonal_fcl(
        x=float(width),
        y=float(cover),
        z=np.sqrt(np.sum(np.square(final_direction))),
        transformation=transformation,
        position=np.array(
            [
                width / 2,
                (l_total - top_bottom_length + bottom_bottom_length) / 2,
                (h_total - top_thickness) / 2,
            ]
        ),
    )
)

# 增加顶端板右侧障碍物空间
original_direction_1 = np.array([0, 0, 1])
final_direction_1 = np.array(
    [
        0,
        l_total - top_top_length - bottom_top_length,
        h_total - steps_h - bottom_thickness,
    ]
)
transformation_1 = rotation_matrix_from_vectors(original_direction_1, final_direction_1)
print(final_direction_1)
cover_objs.append(
    Diagonal_fcl(
        x=float(width),
        y=float(cover),
        z=np.sqrt(np.sum(np.square(final_direction_1))),
        transformation=transformation_1,
        position=np.array(
            [
                width / 2,
                (l_total - top_top_length + bottom_top_length) / 2,
                (h_total - steps_h + bottom_thickness) / 2,
            ]
        ),
    )
)


def get_hole():
    hole_objs = []
    # 顶端孔洞边距
    top_right_long_a1 = 100  # 顶端右侧纵向边距
    top_right_horizon_b1 = 300  # 顶端右侧横向边距
    top_left_long_a2 = 100  # 顶端左侧纵向边距
    top_left_horizon_b2 = 300  # 顶端左侧横向边距
    # 底端孔洞边距
    bottom_right_long_a3 = 100  # 底端右侧纵向边距
    bottom_right_horizon_b3 = 300  # 底端右侧横向边距
    bottom_left_long_a4 = 100  # 底端左侧纵向边距
    bottom_left_horizon_b4 = 300  # 底端左侧横向边距

    top_hole_length = 160
    bottom_hole_length = 160

    top_hole_type = 0  # 固定铰链（1是滑动铰链）
    bottom_hole_type = 0

    # 固定支座参数
    fix_hinge_c2, fix_hinge_d2 = 60, 50
    # 滑动支座参数
    sliding_hinge_c1 = 70
    sliding_hinge_d1 = 55
    sliding_hinge_e1 = 65
    sliding_hinge_f1 = 50
    sliding_hinge_h1 = 50

    if top_hole_type == 0:  # HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
        top_radius = fix_hinge_c2 / 2
    else:
        top_radius = sliding_hinge_c1 / 2
    if bottom_hole_type == 0:  # HoleType.FIXED_HINGE =0 , HoleType.SLIDING_HINGE=1
        bottom_radius = fix_hinge_c2 / 2
    else:
        bottom_radius = sliding_hinge_c1 / 2

    hole_objs.append(
        Cylinder_fcl(
            radius=bottom_radius,
            length=bottom_hole_length,
            position=np.array(
                [
                    bottom_left_horizon_b4,
                    bottom_left_long_a4,
                    0 + bottom_hole_length / 2,
                ]
            ),
        )
    )
    hole_objs.append(
        Cylinder_fcl(
            radius=bottom_radius,
            length=bottom_hole_length,
            position=np.array(
                [
                    width - bottom_right_horizon_b3,
                    bottom_right_long_a3,
                    0 + bottom_hole_length / 2,
                ]
            ),
        )
    )
    hole_objs.append(
        Cylinder_fcl(
            radius=top_radius,
            length=top_hole_length,
            position=np.array(
                [
                    top_left_horizon_b2,
                    l_total - top_left_long_a2,
                    h_total - top_hole_length / 2,
                ]
            ),
        )
    )
    hole_objs.append(
        Cylinder_fcl(
            radius=top_radius,
            length=top_hole_length,
            position=np.array(
                [
                    width - top_right_horizon_b1,
                    l_total - top_right_long_a1,
                    h_total - top_hole_length / 2,
                ]
            ),
        )
    )
    print(hole_objs)
    return hole_objs


@dataclass
class RoundHeadHangingNailShape(object):
    """
    圆头吊钉形状数据
    """

    type: str = "圆头吊钉"  # 类型
    factory: str = "现代营造"  # 系列名
    name: str = "DJ-25-170"  # 名称
    abbreviation: str = "DJ"  # 编号前缀
    capacity: float = 2.5  # 承载力
    length: float = 170  # 吊钉长度
    top_diameter: float = 26  # 顶部直径
    top_height: float = 10  # 顶部高度
    top_adjacent_height: float = 3  # 顶部连接高度
    middle_diameter: float = 14  # 中间直径
    middle_height: float = 144  # 中间高度
    bottom_adjacent_height: float = 3  # 底部连接高度
    bottom_diameter: float = 35  # 底部直径
    bottom_height: float = 10  # 底部高度
    radius: float = 37  # 埋入深度？


def get_lifting():
    lifting_objs = []
    lifting_position_a = 5  # 吊装预埋件顶端踏步阶数
    lifting_position_b = 1  # 吊装预埋件底端踏步阶数
    lifting_position_c = float(400)  # 吊装预埋件左侧横向边距
    lifting_position_d = float(400)  # 吊装预埋件右侧横向边距

    edge_a = bottom_top_length + (lifting_position_a - 0.5) * steps_b  # 吊装预埋件顶端纵向边距
    edge_b = bottom_top_length + (lifting_position_b - 0.5) * steps_b  # 吊装预埋件底端纵向边距
    top_h = bottom_thickness + lifting_position_a * steps_h  # 上部吊装件坐标
    bottom_h = bottom_thickness + lifting_position_b * steps_h  # 底部吊装件坐标
    position_x = [lifting_position_c, float(width - lifting_position_d)]
    position_y = [float(edge_b), float(edge_a)]
    position_z = [float(bottom_h), float(top_h)]

    lifting_type = 0  # 圆头吊钉
    # lifting_type = 1    # 预埋锚栓

    # 4个吊装预埋件孔洞的顶部坐标列表
    lifting_positions = [
        [position_x[0], position_y[0], position_z[0]],  # 左下
        [position_x[0], position_y[1], position_z[1]],  # 左上
        [position_x[1], position_y[0], position_z[0]],  # 右下
        [position_x[1], position_y[1], position_z[1]],
    ]  # 右上
    if lifting_type == 0:  # ROUNDING_HEAD = 0 #    ANCHOR = 1
        rabbet_radius = RoundHeadHangingNailShape.radius  # 顶部半球半径
        top_diameter = RoundHeadHangingNailShape.top_diameter  # 顶部直径
        top_height = (
            RoundHeadHangingNailShape.top_height
            + RoundHeadHangingNailShape.top_adjacent_height
        )  # 顶部总高度
        middle_diameter = RoundHeadHangingNailShape.middle_diameter  # 中部直径
        middle_height = RoundHeadHangingNailShape.middle_height  # 中部高度
        bottom_diameter = RoundHeadHangingNailShape.bottom_diameter  # 底部直径
        bottom_height = (
            RoundHeadHangingNailShape.bottom_height
            + RoundHeadHangingNailShape.bottom_adjacent_height
        )  # 底部高度
        height_0 = (
            rabbet_radius - RoundHeadHangingNailShape.top_height
        ) / 2  # 吊钉的最上端距离台阶面的距离
        for lifting_position in lifting_positions:
            lifting_objs.append(
                Cylinder_fcl(
                    radius=rabbet_radius,
                    length=rabbet_radius,
                    position=np.array(lifting_position)
                    - np.array([0.0, 0.0, rabbet_radius / 2]),
                )
            )
            lifting_objs.append(
                Cylinder_fcl(
                    radius=top_diameter / 2,
                    length=top_height,
                    position=np.array(lifting_position)
                    - np.array([0.0, 0.0, height_0 + top_height / 2]),
                )
            )
            lifting_objs.append(
                Cylinder_fcl(
                    radius=middle_diameter / 2,
                    length=middle_height,
                    position=np.array(lifting_position)
                    - np.array([0.0, 0.0, height_0 + top_height + middle_height / 2]),
                )
            )
            lifting_objs.append(
                Cylinder_fcl(
                    radius=bottom_diameter / 2,
                    length=bottom_height,
                    position=np.array(lifting_position)
                    - np.array(
                        [
                            0.0,
                            0.0,
                            height_0 + top_height + middle_height + bottom_height / 2,
                        ]
                    ),
                )
            )
    print(lifting_objs)
    return lifting_objs


def get_railing():
    """栏杆预埋件"""
    railing_objs = []
    rail_design_mode = 1
    """
    "M2": {"name": "M2", "a": 100, "b": 100, "c": 20, "d": 100, "t": 6, "fi": 8, "depth": 15, "length": 5}
    """
    if rail_design_mode == 1:  # MANUAL = 1    NO = 2
        # 获取栏杆预埋件的数据
        rail_a = 100
        rail_b = 100
        rail_c = 20
        rail_d = 100
        rail_t = 6
        rail_fi = 8  # 直径
        rabbet_depth = float(15)  # 埋入深度
        rabbet_add_d = float(5)  # 底边长度

        rail_number = [1, 3, 6]  # 栏杆所在的阶数
        x_a = 75  # 横向边距
        y_b = 130  # 纵向距离台阶
        # 栏杆布置左右侧
        position_x = []
        rail_layout = 0
        if rail_layout == 0:  # ONLY_RIGHT = 0 ONLY_LEFT = 1   BOTH = 2
            position_x.append(float(width - x_a))  # x向中心点坐标？
        elif rail_layout == 1:
            position_x.append(float(x_a))
        else:
            position_x.append(float(x_a))
            position_x.append(float(width - x_a))
        position_y = []
        for i in rail_number:
            position_y.append(float(bottom_top_length + i * steps_b - y_b))  # y向中心点？
        position_z = []
        for i in rail_number:
            position_z.append(float(bottom_thickness + i * steps_h))
        railing_positions = []
        for i in range(len(position_x)):
            for j in range(len(rail_number)):
                railing_positions.append([position_x[i], position_y[j], position_z[j]])
        print(railing_positions)
        transformation = rotation_matrix_from_vectors(
            np.array([[0, 0, 1]]), np.array([[0, 1, 0]])
        )
        for railing_position in railing_positions:
            railing_objs.append(
                Box_fcl(
                    x=2 * rabbet_add_d + rail_b,
                    y=2 * rabbet_add_d + rail_a,
                    z=rabbet_depth,
                    position=np.array(railing_position)
                    - np.array([0.0, 0.0, rabbet_depth / 2]),
                )
            )
            railing_objs.append(
                Box_fcl(
                    x=rail_b,
                    y=rail_a,
                    z=rail_t,
                    position=np.array(railing_position)
                    - np.array([0.0, 0.0, rabbet_depth + rail_t / 2]),
                )
            )
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_d,
                    transformation=np.eye(3),
                    position=np.array(railing_position)
                    + np.array(
                        [
                            -(rail_b / 2 - rail_c),
                            rail_a / 2 - rail_c,
                            -(rabbet_depth + rail_t + rail_d / 2),
                        ]
                    ),
                )
            )  # 左上
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_a - 2 * rail_c,
                    transformation=transformation,
                    position=np.array(railing_position)
                    + np.array(
                        [
                            -(rail_b / 2 - rail_c),
                            0.0,
                            -(rabbet_depth + rail_t + rail_d + rail_fi / 2),
                        ]
                    ),
                )
            )  # y向短筋
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_d,
                    transformation=np.eye(3),
                    position=np.array(railing_position)
                    + np.array(
                        [
                            -(rail_b / 2 - rail_c),
                            -(rail_a / 2 - rail_c),
                            -(rabbet_depth + rail_t + rail_d / 2),
                        ]
                    ),
                )
            )  # 左下
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_d,
                    transformation=np.eye(3),
                    position=np.array(railing_position)
                    + np.array(
                        [
                            rail_b / 2 - rail_c,
                            rail_a / 2 - rail_c,
                            -(rabbet_depth + rail_t + rail_d / 2),
                        ]
                    ),
                )
            )  # 右上
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_a - 2 * rail_c,
                    transformation=transformation,
                    position=np.array(railing_position)
                    + np.array(
                        [
                            rail_b / 2 - rail_c,
                            0.0,
                            -(rabbet_depth + rail_t + rail_d + rail_fi / 2),
                        ]
                    ),
                )
            )  # y向短筋
            railing_objs.append(
                Rebar_fcl(
                    diameter=rail_fi,
                    length=rail_d,
                    transformation=np.eye(3),
                    position=np.array(railing_position)
                    + np.array(
                        [
                            rail_b / 2 - rail_c,
                            -(rail_a / 2 - rail_c),
                            -(rabbet_depth + rail_t + rail_d / 2),
                        ]
                    ),
                )
            )  # 右下
    # print(railing_objs)
    return railing_objs


# if __name__ == '__main__':
# get_plt()
# get_hole()
# get_lifting()
# get_railing()
