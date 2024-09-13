import numpy as np


def compute_r(st_p, object_p):
    """计算两点距离"""
    delta = object_p - st_p
    r = np.sqrt(np.sum(np.power(delta, 2)))
    return r


def compute_aer(st_p, object_p, r):
    delta = object_p - st_p
    if delta[0] > 0:
        elevation = np.arcsin((delta[2] / r))
        azimuth = np.arctan(delta[1] / delta[0])
    elif delta[0] < 0:
        if delta[1] >= 0:
            elevation = np.arcsin((delta[2] / r))
            azimuth = np.arctan(delta[1] / delta[0]) + np.pi
        elif delta[1] < 0:
            elevation = np.arcsin((delta[2] / r))
            azimuth = np.arctan(delta[1] / delta[0]) - np.pi
    elif delta[0] == 0:
        if delta[1] > 0:
            elevation = np.arcsin((delta[2] / r))
            azimuth = np.pi / 2
        elif delta[1] < 0:
            elevation = np.arcsin((delta[2] / r))
            azimuth = -np.pi / 2
        elif delta[1] == 0:
            azimuth = 0
            if delta[2] == 0:
                elevation = 0
            else:
                elevation = np.arcsin((delta[2] / r))
    return azimuth, elevation


def compute_Attract(qa, azimuth, elevation, r):
    Fatx = qa * r * np.cos(elevation) * np.cos(azimuth)
    Faty = qa * r * np.cos(elevation) * np.sin(azimuth)
    Fatz = qa * r * np.sin(elevation)
    return Fatx, Faty, Fatz
