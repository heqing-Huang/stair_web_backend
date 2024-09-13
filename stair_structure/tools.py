import math

import numpy as np


# 创建并查钢筋面积表
def find_area_table(min_area: float, min_dia: int, max_spacing: int):
    """
    根据计算出来的最小钢筋面积和该类型钢筋的最小直径和最大间距查表
    直径范围：6,8,10,12,14,16,18,20 mm ---0 ,1 ,2 ,3 ,4 ,5 ,6 ,7
    间距范围：70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260 mm
             0,1,2,3,4,5,6,7,8,,9,10,11,12,13,14,15,16,17,18,19
    :param min_area: 最小面积
    :param min_dia: 最小直径
    :param max_spacing: 最大间距
    :return:
    """
    rows = int((max_spacing - 70) / 10) + 1  # 行数 间距
    columns = int((20 - min_dia) / 2) + 1  # 列数 直径
    # 制作单位板宽面积表
    area_table = np.zeros((rows, columns))
    for i in range(rows):
        for j in range(columns):
            area_table[i][j] = (
                (math.pi / 4) * ((j * 2 + min_dia) ** 2) * (1000 / ((i + 7) * 10))
            )
    # print("钢筋面积表:",area_table)
    area_limit = area_table[area_table >= min_area]
    area_limit.sort()
    cur_area = 0
    if len(area_limit) > 0:
        cur_area = area_limit[0]
    index_ = np.where(area_table == cur_area)
    cur_spacing = (index_[0][0] + 7) * 10
    cur_dia = index_[1][0] * 2 + min_dia
    return cur_area, cur_dia, cur_spacing
