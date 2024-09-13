import numpy as np


class PointOffset:
    def __init__(self, bar):
        self.bar = bar

    def featurepoint(self):
        """提取路径点中的特征点"""
        self.bar_simple = [self.bar[0]]
        for i in range(1, len(self.bar) - 1):  # 除去起始点和目标点
            X = self.bar[i - 1]
            Y = self.bar[i]
            Z = self.bar[i + 1]
            XYZ = np.array([X, Y, Z])

            if (
                np.max(np.sum(np.array([0, 0, 0]) == XYZ, axis=1)) != 3
            ):  # 如果相邻三个点中没有[0,0,0]
                if np.linalg.matrix_rank(XYZ) == 2:  # 3点共面，不是特征点，不加入bar_simple
                    self.bar_simple = self.bar_simple
                if np.linalg.matrix_rank(XYZ) == 3:  # 3点不共面，是特征点，将Y加入bar_simple
                    self.bar_simple = self.bar_simple + [Y]
            else:  # 如果相邻三个点中有[0,0,0]
                if np.linalg.matrix_rank(XYZ) == 1:  # 3点共线，不是特征点，不加入bar_simple
                    self.bar_simple = self.bar_simple
                if np.linalg.matrix_rank(XYZ) == 2:  # 3点不共线，共面，是特征点
                    self.bar_simple = self.bar_simple + [Y]
        self.bar_simple = self.bar_simple + [self.bar[-1]]

    def long_picknewpoint(self):
        new_point_1 = self.bar_simple[0]
        dis_x = []
        for i in range(len(self.bar_simple)):
            dis_x = dis_x + [abs(self.bar_simple[i][0] - new_point_1[0])]
        number_lable = np.argmax(dis_x)  # 离起点最远点的索引
        new_point = [[] for _ in range(2)]
        if number_lable == 0:
            new_point[0] = self.bar_simple[0]
            new_point[1] = self.bar_simple[-1]
        else:
            new_point[0] = [
                self.bar_simple[number_lable][0],
                self.bar_simple[0][1],
                self.bar_simple[0][2],
            ]
            new_point[1] = [
                self.bar_simple[number_lable][0],
                self.bar_simple[-1][1],
                self.bar_simple[-1][2],
            ]
        return new_point

    def dis_picknewpoint(self):
        new_point_1 = self.bar_simple[0]
        dis_y = []
        dis_z = []
        for i in range(len(self.bar_simple)):
            dis_y = dis_y + [abs(self.bar_simple[i][1] - new_point_1[1])]
            dis_z = dis_z + [abs(self.bar_simple[i][2] - new_point_1[2])]
        number_label_y = np.argmax(dis_y)
        number_label_z = np.argmax(dis_z)
        new_point = [[] for _ in range(2)]
        if number_label_y == 0 and number_label_z == 0:
            new_point[0] = self.bar_simple[0]
            new_point[1] = self.bar_simple[-1]
        elif number_label_y == 0 and number_label_z != 0:
            new_point[0] = [
                self.bar_simple[0][0],
                self.bar_simple[0][1],
                self.bar_simple[number_label_z][2],
            ]
            new_point[1] = [
                self.bar_simple[-1][0],
                self.bar_simple[-1][1],
                self.bar_simple[number_label_z][2],
            ]
        elif number_label_y != 0 and number_label_z == 0:
            new_point[0] = [
                self.bar_simple[0][0],
                self.bar_simple[number_label_y][1],
                self.bar_simple[0][2],
            ]
            new_point[1] = [
                self.bar_simple[-1][0],
                self.bar_simple[number_label_y][1],
                self.bar_simple[-1][2],
            ]
        elif number_label_y != 0 and number_label_z != 0:
            new_point[0] = [
                self.bar_simple[0][0],
                self.bar_simple[number_label_y][1],
                self.bar_simple[number_label_z][2],
            ]
            new_point[1] = [
                self.bar_simple[-1][0],
                self.bar_simple[number_label_y][1],
                self.bar_simple[number_label_z][2],
            ]
        return new_point


def point_offset_rebar(bar, x, offset_label):
    """
    :param bar: 未调整路径 的一根钢筋
    :param x:  初始坐标
    :param offset_label: 偏移维度
    :return: 调整路径的一个钢筋
    """
    dis = []
    for i in range(len(bar)):
        dis.append([abs(bar[i][offset_label] - x)])
    number_lable = np.argmax(dis)
    new_bar_point = [[] for _ in range(len(bar))]
    for i in range(len(new_bar_point)):
        new_bar_point[i] = bar[i]
        new_bar_point[i][offset_label] = bar[number_lable][offset_label]
    return new_bar_point
