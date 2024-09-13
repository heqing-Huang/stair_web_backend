"""
Author:zhang chao
Date:
Aim : set up the relative function of  stair solid
"""

from typing import List, Optional, Tuple, Dict
from OCC.Core.gp import gp_Pnt, gp_Ax1, gp_Ax2, gp_Ax3, gp_Dir, gp_XYZ
from OCC.Core.TopoDS import TopoDS_Shape, TopoDS_Wire, TopoDS_Vertex
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeEdge,
)

from OCC.Core.BRep import BRep_Tool

from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape  # 形状与形状之间极值点的距离

from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.HLRAlgo import HLRAlgo_Projector
from OCC.Core.BRepBndLib import brepbndlib_AddOBB
from OCC.Extend.TopologyUtils import TopologyExplorer

from OCC.Core.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape, HLRBRep_PolyAlgo
from OCC.Extend.TopologyUtils import (
    list_of_shapes_to_compound,
    discretize_edge,
    get_sorted_hlr_edges,
    WireExplorer,
    is_wire,
)  # 获取每条边的端点信息

from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods_Vertex, topods_Edge  # 拓扑顶点

import copy

from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core import TopoDS

from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.IFSelect import IFSelect_RetDone


def transform_solid_to_step_data(solid: TopoDS, filename: str) -> None:
    """
    将拓扑形状转化为step文件
    :param solid:
    :param filename:
    :return:
    """
    # initialize the STEP exporter
    step_writer = STEPControl_Writer()
    Interface_Static_SetCVal("write.step.schema", "AP203")

    # transfer shapes and write file
    step_writer.Transfer(solid, STEPControl_AsIs)
    status = step_writer.Write(filename + ".stp")

    if status != IFSelect_RetDone:
        raise AssertionError("load failed")


def make_ax3(origin: List[float], v_z: List[float], v_x: List[float]) -> gp_Ax3:
    """
    创建 gp_Ax3 局部坐标系.
    此处主要用于统一描述遮挡剔除过程中的Ax2 和尝试做点投影的Ax3，为后续手动标注做基础准备
    Args:
        origin:三维坐标点,xyz，用列表表示
        v_z: Z 轴方向,xyz ，用列表表示
        v_x: x 轴方向,xyz ，用列表表示
    Returns:
    """
    # 简单的做数据类型判断
    assert len(origin) == 3, Exception(f"{origin=} 不满足origin xyz 要求")
    assert len(v_z) == 3, Exception(f"{v_z=} 不满足v_z xyz 要求")
    assert len(v_x) == 3, Exception(f"{v_x=} 不满足v_x xyz 要求")

    position = gp_Pnt(*origin)
    direction = gp_Dir(*v_z)
    vx = gp_Dir(*v_x)

    ax_2 = gp_Ax2(position, direction, vx)
    ax_3 = gp_Ax3(ax_2)  # 构造一个对象的参考坐标系

    return ax_3


def rewrite_get_sorted_hlr_edges(
    topods_shape: TopoDS_Shape,
    origin: gp_Pnt,
    project_dir: gp_Dir,
    export_hidden_edges: Optional[bool] = True,
) -> Tuple[List, List]:
    """
    参见 OCC.Extend.TopologyUtils.get_sorted_hlr_edges
    修改传入参数,以满足内部逻辑
    Args:
        topods_shape:
        ax3:局部坐标系
        export_hidden_edges:
    Returns:
    """
    from OCC.Extend.TopologyUtils import get_sorted_hlr_edges

    # 1.加载模型
    hlr = HLRBRep_Algo()  # 获得线段本身
    hlr.Add(topods_shape)
    # 2.设置视点及投影方向、投影平面
    ax2 = gp_Ax2(origin, project_dir)
    projector = HLRAlgo_Projector(ax2)  # 投影函数确定投影平面

    hlr.Projector(projector)  # 设置投影平面
    # 3.计算投影
    hlr.Update()  # 隐藏线移除算法更新
    hlr.Hide()  # 通过该算法计算模型可见性与隐藏线，仅HLRBRep_Algo算法独有
    # 4.提取边
    hlr_shapes = HLRBRep_HLRToShape(hlr)  # 开始投影形状操作

    # 可视化边
    # 可提取边的类型有
    # 垂直于视线方向的边
    visible = []
    visible_sharp_edges_as_compound = hlr_shapes.VCompound()
    if visible_sharp_edges_as_compound:  # 探测尖边
        visible += list(TopologyExplorer(visible_sharp_edges_as_compound).edges())
    # visible_smooth_edges_as_compound = hlr_shapes.Rg1LineVCompound()  # 探测平滑边---实体与实体相连接的接触边
    # if visible_smooth_edges_as_compound:
    #     visible += list(TopologyExplorer(visible_smooth_edges_as_compound).edges())
    # visible_rgnline_as_compound = hlr_shapes.RgNLineVCompound()  # 去掉sewn edges线----圆头吊钉缝边
    # if visible_rgnline_as_compound:
    #     visible += list(TopologyExplorer(visible_rgnline_as_compound).edges())
    visible_contour_edges_as_compound = (
        hlr_shapes.OutLineVCompound()
    )  # 探测垂直轮廓边---形成形状的主要函数
    if visible_contour_edges_as_compound:
        visible += list(TopologyExplorer(visible_contour_edges_as_compound).edges())
    # visible_isoline_edges_as_compound = hlr_shapes.IsoLineVCompound()
    # if visible_isoline_edges_as_compound:
    #    visible += list(TopologyExplorer(visible_isoline_edges_as_compound).edges())

    # 隐藏线的边--平行于视线方向的边
    hidden = []
    if export_hidden_edges:  # 导出隐藏边
        hidden_sharp_edges_as_compound = hlr_shapes.HCompound()
        if hidden_sharp_edges_as_compound:  # 隐藏尖边
            hidden += list(TopologyExplorer(hidden_sharp_edges_as_compound).edges())
        hidden_contour_edges_as_compound = hlr_shapes.OutLineHCompound()  # 隐藏轮廓边
        if hidden_contour_edges_as_compound:
            hidden += list(TopologyExplorer(hidden_contour_edges_as_compound).edges())
    return visible, hidden


def discretize_wire(a_topods_wire: TopoDS_Wire, deflection=0.0005):
    """Returns a set of points"""
    if not is_wire(a_topods_wire):
        raise AssertionError(
            "You must provide a TopoDS_Wire to the discretize_wire function."
        )
    wire_explorer = WireExplorer(a_topods_wire)
    wire_pnts = []
    # loop over ordered edges---边排序
    for edg in wire_explorer.ordered_edges():
        edg_pnts = discretize_edge(edg, deflection)
        wire_pnts += edg_pnts
    return wire_pnts


def compute_project_shape(shape: TopoDS_Shape, origin: gp_Pnt, project_dir: gp_Dir):
    """
    计算一个shape 投影后在平面内的的形状。返回的shape是在这个ax3 下坐标系表示的
    Args:
        shape:
        origin:坐标原点
        project_dir:投影平面方向
    Returns:

    """
    visible, hidden = rewrite_get_sorted_hlr_edges(
        shape, origin, project_dir, export_hidden_edges=False
    )
    make_wire = BRepBuilderAPI_MakeWire()
    points = []
    for edg in visible:
        points_3d = discretize_edge(edg, 0.001)  # 转换成点
        points.append(points_3d)
        for i in range(len(points_3d) - 1):
            point_start = points_3d[i]
            start_pnt = gp_Pnt(*point_start)
            point_end = points_3d[i + 1]
            end_pnt = gp_Pnt(*point_end)
            make_edg_api = BRepBuilderAPI_MakeEdge(start_pnt, end_pnt)
            edg = make_edg_api.Edge()
            make_wire.Add(edg)
    return make_wire.Shape(), points


def compute_project_minimal_distance_pnt(
    shape_1: TopoDS_Shape, shape_2: TopoDS_Shape
) -> List[Tuple[gp_Pnt, gp_Pnt]]:
    """
    shape_1,shape_2 的最近点(所有解决方案）
    Args:
        shape_1:
        shape_2:

    Returns:

    """
    # 关于最近点和最近距离计算,来自于occ examples: minimal_distance.py
    dss = BRepExtrema_DistShapeShape()
    dss.LoadS1(shape_1)
    dss.LoadS2(shape_2)
    dss.Perform()
    assert dss.IsDone(), Exception(f"未成功计算得到最近点：{shape_1},{shape_2}")
    # 得到shape_1,shape_2  上最近点
    data_back: List[Tuple[gp_Pnt, gp_Pnt]] = []
    ns: int = dss.NbSolution()
    for i in range(1, ns + 1):  # start index is 1
        minimal_distance_pnt_1: gp_Pnt = dss.PointOnShape1(i)
        minimal_distance_pnt_2: gp_Pnt = dss.PointOnShape2(i)
        data_back.append((minimal_distance_pnt_1, minimal_distance_pnt_2))
    return data_back


def compute_project_center_to_shape(
    shape_1: TopoDS_Shape, shape_2: TopoDS_Shape
) -> List[Tuple[gp_Pnt, gp_Pnt]]:
    """
    基于compute_project_minimal_distance_pnt

    计算shape2 非轴对齐包围框的中心点到shape1 的最短距离组
    Args:
        shape_1:
        shape_2:

    Returns:

    """
    obb = Bnd_OBB()
    brepbndlib_AddOBB(shape_2, obb, True, True, True)
    center: gp_XYZ = obb.Center()
    pnt: gp_Pnt = gp_Pnt(center)
    shape_2_center_shape = BRepBuilderAPI_MakeVertex(pnt).Shape()
    data_backs: List[Tuple[gp_Pnt, gp_Pnt]] = compute_project_minimal_distance_pnt(
        shape_1, shape_2_center_shape
    )
    return data_backs


def my_BRepAlgoAPI_Fuse(shape1: TopoDS_Shape, shape2: TopoDS_Shape):
    """
    对两实体作布尔并运算
    :param shape1:
    :param shape2:
    :return:
    """
    fuse_solid = BRepAlgoAPI_Fuse(shape1, shape2)
    fuse_solid.SimplifyResult()
    return fuse_solid


def my_BRepAlgoAPI_Cut(shape1: TopoDS_Shape, shape2: TopoDS_Shape):
    """
    对两实体作布尔差运算
    :param shape1:
    :param shape2:
    :return:
    """
    cut_solid = BRepAlgoAPI_Cut(shape1, shape2)
    cut_solid.SimplifyResult()
    return cut_solid


def my_BRepAlgoAPI_Common(shape1: TopoDS_Shape, shape2: TopoDS_Shape):
    """
    对两实体作布尔交运算
    :param shape1:
    :param shape2:
    :return:
    """
    common_solid = BRepAlgoAPI_Common(shape1, shape2)
    common_solid.SimplifyResult()
    return common_solid


def fuse_shape(*args):
    """
    合并多个模型为一个混合模型
    :param args:
    :return:
    """
    if len(args) == 0:
        return None
    fuse_shape = None
    for shape in args:
        if fuse_shape == None:
            fuse_shape = shape
            continue
        fuse_shape = BRepAlgoAPI_Fuse(fuse_shape, shape)
        fuse_shape.SimplifyResult()
        fuse_shape = fuse_shape.Shape()
    return fuse_shape


def rotation_solid(
    solid: TopoDS_Shape, base: List[float], rotation_axis: List[float], angle: float
) -> TopoDS_Shape:
    """
    将实体绕指定轴逆时针旋转一定角度
    :param solid:TopoDS_Shape
    :param base:旋转基点
    :param rotation_axis:旋转轴
    :param angle:旋转角度---弧度制
    :return:
    """
    from OCC.Core.gp import gp_Trsf
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

    transform = gp_Trsf()
    transform.SetRotation(
        gp_Ax1(
            gp_Pnt(base[0], base[1], base[2]),
            gp_Dir(rotation_axis[0], rotation_axis[1], rotation_axis[2]),
        ),
        angle,
    )
    transform_solid = BRepBuilderAPI_Transform(solid, transform).Shape()
    return transform_solid


def move_solid(solid: TopoDS_Shape, vector: List[float]):
    """
    将实体进行平移变换
    :param solid:
    :param vector:
    :return:
    """
    from OCC.Core.gp import gp_Trsf, gp_Vec
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(vector[0], vector[1], vector[2]))
    transform_solid = BRepBuilderAPI_Transform(solid, transform).Shape()
    return transform_solid


def from_edge_to_point(shape: TopAbs_EDGE):
    """
    从几何边到坐标点
    :param shape:拓扑形状
    :return:
    """
    topExp = TopExp_Explorer()
    topExp.Init(shape, TopAbs_EDGE)  # 形状是边的集合
    edge = []  # 边
    while topExp.More():
        edg_ = topods_Edge(topExp.Current())
        edge.append(edg_)
        topExp.Next()
    points = []  # 形成最终点集合
    for ed_num in edge:
        point_3d = discretize_edge(ed_num, 0.001)
        points.append(
            [
                [point_3d[0][0], point_3d[0][1], point_3d[0][2]],
                [point_3d[1][0], point_3d[1][1], point_3d[1][2]],
            ]
        )
    return points


def from_vertex_to_point(point_shape) -> List[List[float]]:
    """
    将形状拓扑顶点的形体为转化为gp_Pnt点
    :param point_shape:
    :return:
    """
    topExp = TopExp_Explorer()
    topExp.Init(point_shape, TopAbs_VERTEX)  # 形状是边的集合
    point = []  # 点
    while topExp.More():
        point_ = topods_Vertex(topExp.Current())
        pnt_ = BRep_Tool.Pnt(point_)  # 拓扑顶点转化为几何点的方法
        # tolerance = BRep_Tool.Tolerance(point_)
        point.append([pnt_.X(), pnt_.Y(), pnt_.Z()])
        topExp.Next()
    return point


def get_U_rein_rebar_vertex(rebar_loc, diam, limit_y):
    """
    获取U型加强筋的交点坐标
    :param rebar_loc: 钢筋位置
    :param diam: 钢筋直径
    :param limit_y: 钢筋位置限制
    :return: 线段格式
    """
    first_point = rebar_loc[0]
    second_point = rebar_loc[1]
    max_y = first_point.y
    min_y = first_point.y
    loc_z = first_point.z
    loc_x = first_point.x
    for num in range(len(rebar_loc)):
        current_point = rebar_loc[num]  # 当前点
        if current_point.y >= max_y:
            max_y = current_point.y
        if current_point.y <= min_y:
            min_y = current_point.y
    length = max_y - min_y + diam / 2
    # 修正圆弧角点
    if min_y < limit_y:
        min_y -= diam / 2
    vertex = [loc_x - diam / 2, min_y, loc_z - diam / 2]
    # 逆时针方向
    point_1 = copy.deepcopy(vertex)
    point_2 = copy.deepcopy(vertex)
    point_2[2] += diam
    point_3 = copy.deepcopy(vertex)
    point_3[1] += length
    point_3[2] += diam
    point_4 = copy.deepcopy(vertex)
    point_4[1] += length
    result = [
        [point_1, point_2],
        [point_2, point_3],
        [point_3, point_4],
        [point_4, point_1],
    ]
    return result
