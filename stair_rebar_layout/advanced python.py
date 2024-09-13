from typing import Union
import fcl
import numpy as np
import math


def my(a: Union[str, int]) -> Union[str, int]:
    return a * 2


print(my(21))


# 定义两个形状
box = fcl.Box(1, 1, 1)  # 三边长度1, 1, 1 定义与坐标轴轴对齐的box
sphere = fcl.Sphere(1)  # 半径1 定义Sphere


# 通过形状和位置，生成碰撞对象
box_obj = fcl.CollisionObject(box, fcl.Transform(np.array([0, 0, 0])))
sphere_obj = fcl.CollisionObject(sphere)

# 碰撞检测
request = fcl.CollisionRequest()
result = fcl.CollisionResult()
ret = fcl.collide(box_obj, sphere_obj, request, result)
print("箱子和球是否发生了碰撞", result.is_collision)
# 距离检测
request = fcl.DistanceRequest()
result = fcl.DistanceResult()
ret = fcl.distance(box_obj, sphere_obj, request, result)
print("两者之间最小距离", result.min_distance)
