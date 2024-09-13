"""
获取各种钢筋的弯箍半径
"""


def rebar_mandrel_radius(radius=5, steel_grade=1):
    """
    根据钢筋的直径和强度等级计算弯箍直径
    :param radius: 钢筋半径
    :param steel_grade: 钢筋强度等级
    :return:
    """
    if steel_grade == 1:
        mandrel_radius = 1.25 * radius * 2
    elif steel_grade == 2:
        mandrel_radius = 2.0 * radius * 2
    elif steel_grade == 3:
        mandrel_radius = 2.5 * radius * 2
    else:
        mandrel_radius = 2.5 * radius * 2
    return int(mandrel_radius)
