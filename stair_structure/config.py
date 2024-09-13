"""
Date&Time           2022/8/3 16:00
Author              HaoLan

"""

# 混凝土强度参数配置
CONCRETE_CONFIG = [
    {
        "name": "C30",
        "f_c": 14.3,
        "f_t": 1.43,
        "a_erf": 1.0,
        "b_ta": 0.8,
        "f_ck": 20.1,
        "f_tk": 2.01,
        "ec": 30000.0,
        "grade": 30,
    },
    {
        "name": "C35",
        "f_c": 16.7,
        "f_t": 1.57,
        "a_erf": 1.0,
        "b_ta": 0.8,
        "f_ck": 23.4,
        "f_tk": 2.2,
        "ec": 31500.0,
        "grade": 35,
    },
    {
        "name": "C40",
        "f_c": 19.1,
        "f_t": 1.71,
        "a_erf": 1.0,
        "b_ta": 0.8,
        "f_ck": 26.8,
        "f_tk": 2.39,
        "ec": 32500.0,
        "grade": 40,
    },
    {
        "name": "C45",
        "f_c": 21.2,
        "f_t": 1.8,
        "a_erf": 1.0,
        "b_ta": 0.8,
        "f_ck": 29.6,
        "f_tk": 2.51,
        "ec": 33500.0,
        "grade": 45,
    },
    {
        "name": "C50",
        "f_c": 23.1,
        "f_t": 1.89,
        "a_erf": 1.0,
        "b_ta": 0.8,
        "f_ck": 32.4,
        "f_tk": 2.64,
        "ec": 34500.0,
        "grade": 50,
    },
]
CONCRETE_CONFIG_DICT = {row["grade"]: row for row in CONCRETE_CONFIG}

# 钢筋配置
# 钢筋名字对应的钢筋强度
REBAR_NAME_CONFIG = {
    "HPB300": {"symbol": "d", "grade": 1},
    "HRB335": {"symbol": "D", "grade": 2},
    "HRBF335": {"symbol": "D", "grade": 2},
    "HRB400": {"symbol": "f", "grade": 3},
    "HRBF400": {"symbol": "f", "grade": 3},
    "RRB400": {"symbol": "f", "grade": 3},
    "HRB500": {"symbol": "F", "grade": 4},
    "HRBF500": {"symbol": "F", "grade": 4},
}
REBAR_GRADE_CONFIG = {
    1: {
        "f_y": 270.0,
        "f1_y": 270.0,
        "d_min": 6.0,
        "d_max": 22.0,
        "fyk": 300.0,
        "fstk": 420.0,
        "es": 210000.0,
    },
    2: {
        "f_y": 300.0,
        "f1_y": 300.0,
        "d_min": 6.0,
        "d_max": 50.0,
        "fyk": 335.0,
        "fstk": 455.0,
        "es": 200000.0,
    },
    3: {
        "f_y": 360.0,
        "f1_y": 360.0,
        "d_min": 6.0,
        "d_max": 50.0,
        "fyk": 400.0,
        "fstk": 540.0,
        "es": 200000.0,
    },
    4: {
        "f_y": 435.0,
        "f1_y": 435.0,
        "d_min": 6.0,
        "d_max": 50.0,
        "fyk": 500.0,
        "fstk": 630.0,
        "es": 200000.0,
    },
}
# 相对受压区高度
RELATIVE_LIMIT_ZONE = {
    "130": {"ksi_b": 0.576},
    "135": {"ksi_b": 0.576},
    "140": {"ksi_b": 0.576},
    "145": {"ksi_b": 0.576},
    "155": {"ksi_b": 0.576},
    "230": {"ksi_b": 0.550},
    "235": {"ksi_b": 0.550},
    "240": {"ksi_b": 0.550},
    "245": {"ksi_b": 0.550},
    "255": {"ksi_b": 0.550},
    "330": {"ksi_b": 0.518},
    "335": {"ksi_b": 0.518},
    "340": {"ksi_b": 0.518},
    "345": {"ksi_b": 0.518},
    "355": {"ksi_b": 0.518},
    "430": {"ksi_b": 0.482},
    "435": {"ksi_b": 0.482},
    "440": {"ksi_b": 0.482},
    "445": {"ksi_b": 0.482},
    "455": {"ksi_b": 0.482},
}
