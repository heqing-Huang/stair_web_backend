# 计算楼梯体积
def get_volume(
    steps_number,
    thickness,
    cos,
    steps_h,
    steps_b,
    b0,
    top_top_length,
    top_thickness,
    top_b,
    bottom_top_length,
    bottom_thickness,
    bottom_b,
    top_bottom_length,
):
    """
    计算楼梯的体积
    :param steps_number:梯步个数
    :param thickness: 板厚
    :param cos: 余弦
    :param steps_h: 踏步高度
    :param steps_b: 踏步宽度
    :param b0: 中部宽度
    :param top_top_length: 顶端顶部长度
    :param top_thickness: 顶端厚度
    :param top_b: 顶端挑耳
    :param bottom_top_length: 底端底部长度
    :param bottom_thickness: 底部厚度
    :param bottom_b: 底端挑耳
    :param top_bottom_length: 顶端底部长度
    :return: 楼梯体积
    """
    v_mid = (
        (steps_number - 1) * ((2 * thickness / cos + steps_h) / 2) * steps_b * b0
    )  # 中部混凝土体积
    v_top = top_top_length * top_thickness * (b0 + top_b)  # 顶端体积
    v_top_1 = (
        (b0 + top_b)
        * (top_top_length - top_bottom_length)
        * (thickness / cos + steps_h - top_thickness)
        / 2
    )
    v_bottom = bottom_top_length * bottom_thickness * (b0 + bottom_b)
    volume_ = v_mid + v_top + v_top_1 + v_bottom  # 底端体积
    return volume_ / (10**9)
