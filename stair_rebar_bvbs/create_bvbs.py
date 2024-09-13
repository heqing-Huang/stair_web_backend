import math
from stair_for_bvbs.models import RebarforBVBS, RebarBVBS, RebarGeoBVBS
import os
import zipfile


class BVBS:
    def __init__(
        self,
        project_number: str = "1",
        schedule_number: str = "1",
        revision: str = "",
        bar_mark: str = "1",
        bar_length: str = "1000",
        bar_quantity: str = "1",
        bar_diameter: str = "12",
        steel_grade: str = "HPB300",
        mandrel_diameter: str = "",
        mesh_type: str = "",
        mesh_width: str = "",
        designer: str = "CR",
        layer: str = "",
        delta: str = "",
        group: str = "",
    ):
        """
        创建bvbs数据
        :param project_number: 项目编号
        :param schedule_number: 钢筋表格编号
        :param revision: 修改版
        :param bar_mark:钢筋序号
        :param bar_length:钢筋长度
        :param bar_quantity:钢筋数量
        :param bar_diameter:钢筋直径
        :param steel_grade:钢筋级别
        :param mandrel_diameter:弯曲直径
        :param mesh_type: 网格类型
        :param mesh_width:网格宽度
        :param designer:设计者
        :param layer:层数
        :param delta:交错钢筋的差值（可选）
        :param group:交错钢筋的组（可选）
        """
        self.bvbs_string = ""
        self.project_number = "j" + project_number
        self.schedule_number = "r" + schedule_number
        self.revision = "i" + revision
        self.bar_mark = "p" + bar_mark
        self.bar_length = "l" + bar_length
        self.bar_quantity = "n" + bar_quantity
        self.bar_weight = "e" + str(
            round(
                7.85 * math.pi * float(bar_diameter) ** 2 / 4 * float(bar_length) / 1e6,
                2,
            )
        )
        self.bar_diameter = "d" + bar_diameter
        self.steel_grade = "g" + steel_grade
        self.mandrel_diameter = "s" + mandrel_diameter
        self.mesh_type = "m" + mesh_type
        self.mesh_width = "b" + mesh_width
        self.designer = "v" + designer
        self.layer = "a" + layer
        self.delta = "t" + delta
        self.group = "c" + group
        self.CRLF = "\n"

    def create_BF2D(
        self, Geometry_block=None, Spacer_block=None, Bar_block=None, Coupler_block=None
    ):
        self.bvbs_string += "BF2D@"
        # 增加H字节
        self.bvbs_string += "H"
        BF2D_H = [
            self.project_number,
            self.schedule_number,
            self.revision,
            self.bar_mark,
            self.bar_length,
            self.bar_quantity,
            self.bar_weight,
            self.bar_diameter,
            self.steel_grade,
            self.mandrel_diameter,
            self.designer,
            self.delta,
            self.group,
        ]
        for H_string in BF2D_H:
            self.bvbs_string += H_string + "@"
        if Geometry_block:
            self.bvbs_string += "G"
            for geo in Geometry_block:
                geo: RebarGeoBVBS
                self.bvbs_string += (
                    "l" + str(geo.length) + "@" + "w" + str(geo.angle) + "@"
                )
        elif Spacer_block:
            pass
        elif Bar_block:
            pass
        elif Coupler_block:
            pass
        else:
            raise Exception("未定义钢筋形状模块")
        # 增加checksum字符串
        checksum_string = self.Checksum()
        self.bvbs_string += checksum_string
        self.bvbs_string += self.CRLF
        return self.bvbs_string

    def Checksum(self):
        checksum_string = "C"
        ascii_sum = 0
        for s in self.bvbs_string:
            ascii_sum += ord(s)
        IP = 96 - ascii_sum % 32
        checksum_string += str(IP)
        checksum_string += "@"
        return checksum_string


def create_bvbs(rebar_for_BVBS: RebarforBVBS):
    ascii_strings = ""
    for attr, value in rebar_for_BVBS.__dict__.items():
        value: RebarBVBS
        bvbs = BVBS(
            project_number=value.project_ID,
            schedule_number=value.stair_ID,
            bar_mark=str(value.mark),
            bar_length=str(value.rebar_length),
            bar_quantity=str(value.rebar_quantity),
            bar_diameter=str(value.rebar_diameter),
            steel_grade=value.rebar_grade,
            mandrel_diameter=str(value.mandrel_diameter),
        )
        ascii_strings += bvbs.create_BF2D(Geometry_block=value.geometric)

    return ascii_strings


# Saves the resulting string to the directory runnning the script.
def save_string_to_file(ascii_strings):
    with open(os.path.join("data", "bvbs_code.abs"), "w") as f:
        try:
            f.write(ascii_strings)
            f.close()
            print(f"BVBS code saved successfully.")
        except Exception:
            f.close()
            print(f"Something went wrong when saving the file. {Exception}")
