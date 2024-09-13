"""
# File       : create_ifc.py
# Time       ：2022/11/3 15:41
# Author     ：CR_X
# version    ：python 3.8
# Description：
"""
import os
from typing import Optional
import ifcopenshell
from ifcopenshell.entity_instance import entity_instance

from stair_structure import __version__ as structure_v
from stair_structure.structure_calculation import (
    StructuralDesign,
    StructuralDesignResult,
    RebarParameter,
    ConcreteParameter,
)
from stair_ifc.core import IfcDocument
from stair_detailed.models import DetailedDesign, DetailedDesignResult

from stair_ifc.get_data import StairData
from stair_ifc.rebar_arc_point import rebar_arc
from stair_rebar_layout.models import RebarforBIM


def create_ifc_doc(header_file: Optional[dict] = None, init_doc: Optional[dict] = None):
    ifc_doc = IfcDocument(header_file, init_doc)  # 创建IFC文件的实例
    return ifc_doc


def create_stair_ifc(
    ifc_doc: IfcDocument,
    structure_design: StructuralDesign,
    structure_design_result: StructuralDesignResult,
    detailed_design: DetailedDesign,
    detailed_design_result: DetailedDesignResult,
    rebar_data: RebarforBIM,
):
    if structure_v < "0.1.4":
        concrete_parameter = structure_design_result.concrete_parameter
        rebar_parameter = structure_design_result.rebar_parameter
    else:
        concrete_parameter = ConcreteParameter.by_grade(
            structure_design.material.concrete_grade
        )
        rebar_parameter = RebarParameter.by_name(structure_design.material.rebar_name)

    schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name(ifc_doc._schema.name())
    ifc_ReinforcingBarSurfaceEnum = schema.declaration_by_name(
        "IfcReinforcingBarSurfaceEnum"
    )
    ifc_ReinforcingBarTypeEnum = schema.declaration_by_name("IfcReinforcingBarTypeEnum")
    ReinforcingBarSurface = ifc_ReinforcingBarSurfaceEnum.enumeration_items()  # 钢筋表面类型
    ReinforcingBarType = ifc_ReinforcingBarTypeEnum.enumeration_items()  # 钢筋类型
    concrete_name = concrete_parameter.name
    steel_name = rebar_parameter.name
    # 全局使用的材料
    concrete_material = ifc_doc.ifcfile.createIfcMaterial(
        concrete_name, None, "Concrete"
    )  # 混凝土材质
    steel_material = ifc_doc.ifcfile.createIfcMaterial(
        steel_name, None, "Steel"
    )  # 钢筋材质

    site, site_placement = ifc_doc.create_site()
    building, building_placement = ifc_doc.create_building(site_placement)
    elevation = 0.0
    building_storey, storey_placement = ifc_doc.create_building_storey(
        building_placement, elevation
    )
    # 建立层级关系
    container_building = ifc_doc.create_RelAggregates(
        "Building Container", building, building_storey
    )
    container_site = ifc_doc.create_RelAggregates("Site Container", site, building)

    container_project = ifc_doc.create_RelAggregates(
        "Project Container", ifc_doc.project, site
    )
    owner_history = ifc_doc.owner_history

    # 混凝土主体建模
    stair_position = ifc_doc.create_IfcAxis2Placement3D(origin=[0.0, 0.0, 0.0])
    stair_placement = ifc_doc.create_IfcLocalplacement(storey_placement, stair_position)
    CSG_Solid_list = []
    stair_split_solids = []

    # 楼梯信息获取
    stair_data = StairData(
        structure_design,
        structure_design_result,
        detailed_design,
        detailed_design_result,
    )
    points = stair_data.get_step_points()  # 混凝土主体，需要收尾相连
    top_ear_points = stair_data.get_top_ear()  # 上部挑耳,需要收尾相连
    bottom_ear_points = stair_data.get_bottom_ear()  # 下部挑耳混凝土主体，需要收尾相连

    # 混凝土部分数据
    stair_width = stair_data.get_body_width()  # 拉伸长度
    top_ear_width = stair_data.get_top_ear_width()  # 拉伸长度
    bottom_ear_width = stair_data.get_bottom_ear_width()  # 拉伸长度

    extrude_direction = [0.0, 0.0, 1.0]
    stair_body_position = ifc_doc.create_IfcAxis2Placement3D(
        origin=[0.0, 0.0, 0.0], z=[1.0, 0.0, 0.0], x=[0.0, 1.0, 0.0]
    )
    stair_body = ifc_doc.create_ExtrudedAreaSolid(
        points, stair_body_position, extrude_direction, stair_width
    )
    stair_split_solids.append(stair_body)
    if top_ear_width:
        top_ear_position = ifc_doc.create_IfcAxis2Placement3D(
            origin=[float(stair_width), 0.0, 0.0], z=[1.0, 0.0, 0.0], x=[0.0, 1.0, 0.0]
        )
        stair_top_ear = ifc_doc.create_ExtrudedAreaSolid(
            top_ear_points, top_ear_position, extrude_direction, top_ear_width
        )
        stair_split_solids.append(stair_top_ear)
    if bottom_ear_width:
        bottom_ear_position = ifc_doc.create_IfcAxis2Placement3D(
            origin=[float(stair_width), 0.0, 0.0], z=[1.0, 0.0, 0.0], x=[0.0, 1.0, 0.0]
        )

        stair_bottom_ear = ifc_doc.create_ExtrudedAreaSolid(
            bottom_ear_points, bottom_ear_position, extrude_direction, bottom_ear_width
        )
        stair_split_solids.append(stair_bottom_ear)

    if len(stair_split_solids) > 1:

        csg_handle = ifc_doc.ifcfile.create_entity(
            "IfcBooleanResult", "UNION", stair_split_solids[0], stair_split_solids[1]
        )

        if len(stair_split_solids) > 2:
            for i in range(2, len(stair_split_solids)):
                csg_handle = ifc_doc.ifcfile.create_entity(
                    "IfcBooleanResult", "UNION", csg_handle, stair_split_solids[i]
                )

        solid_back = ifc_doc.ifcfile.create_entity("IfcCsgSolid", csg_handle)

        CSG_Solid_list.append(solid_back)

    else:
        CSG_Solid_list.append(stair_split_solids[0])

    global_geometry_context: entity_instance = ifc_doc.global_geometry_context
    representation = ifc_doc.ifcfile.createIfcShapeRepresentation(
        global_geometry_context, "Body", "SolidModel", CSG_Solid_list
    )
    product_shape = ifc_doc.ifcfile.createIfcProductDefinitionShape(
        None, None, [representation]
    )

    # 创建楼梯
    stair = ifc_doc.ifcfile.createIfcStair(
        ifc_doc.get_global_id(),
        owner_history,
        "预制混凝土楼梯",
        "预制混凝土楼梯",
        None,
        stair_placement,  # 存放位置,局部坐标
        product_shape,  # 几何形状
        None,
    )

    # 洞口信息
    ifc_doc.create_hole(stair, stair_placement, stair_data)

    # 防滑槽
    ifc_doc.create_step_slot(stair, stair_placement, stair_data)

    # 滴水槽
    ifc_doc.create_water_drop(stair, stair_placement, stair_data)

    # 将楼梯绑定到标高处
    ifc_doc.ifcfile.createIfcRelContainedInSpatialStructure(
        ifc_doc.get_global_id(),
        owner_history,
        "Building Storey Container",
        None,
        [stair],
        building_storey,
    )  # 设置在那个楼层

    ifc_doc.ifcfile.createIfcRelAssociatesMaterial(
        ifc_doc.get_global_id(),
        owner_history,
        None,
        None,
        RelatedObjects=[stair],
        RelatingMaterial=concrete_material,
    )

    # 栏杆预埋件
    ifc_doc.create_railing(stair, stair_placement, stair_data)
    # ifc_doc.create_railing(stair, stair_placement, railing_data, building_storey)

    # 吊装预埋件
    ifc_doc.create_lifting(stair, stair_placement, stair_data)
    # ifc_doc.create_lifting(stair, stair_placement, lifting_data, building_storey)

    # 脱模预埋件
    ifc_doc.create_demoulding(stair, stair_placement, stair_data)
    # ifc_doc.create_demoulding(stair, stair_placement, demoulding_data, building_storey)

    # 创建钢筋
    rebars = []
    for attr, value in rebar_data.__dict__.items():
        for rebar in value:
            new_rebar = rebar_arc(rebar, rebar_parameter.grade)

            rebar_instance = ifc_doc.create_rebar(stair_placement, new_rebar)
            rebars.append(rebar_instance)
    ifc_doc.ifcfile.createIfcRelAssociatesMaterial(
        ifc_doc.get_global_id(),
        owner_history,
        None,
        None,
        RelatedObjects=rebars,
        RelatingMaterial=steel_material,
    )
    rel_aggregate = ifc_doc.ifcfile.createIfcRelAggregates(
        ifc_doc.get_global_id(),
        ifc_doc.owner_history,
        "Stair Container",
        None,
        stair,
        rebars,
    )  #

    # entity_instance_write = ifc_doc.ifcfile.createIfcRelContainedInSpatialStructure(
    #     ifc_doc.get_global_id(),
    #     ifc_doc.owner_history,
    #     "Building Storey Container",
    #     None,
    #     rebars,
    #     building_storey)

    # with open(ifc_doc.ifcfile.wrapped_data.header.file_name.name, "w") as f:
    #     f.write(ifc_doc.ifcfile.to_string())
    return ifc_doc


def stair_IFC_creation(
    structure_design: StructuralDesign,
    structure_design_result: StructuralDesignResult,
    detailed_design: DetailedDesign,
    detailed_design_result: DetailedDesignResult,
    rebar_data: RebarforBIM,
):
    """
    绘制楼梯主体并绑定到指定位置,同时绘制开洞
    :param structure_design:
    :param structure_design_result:
    :param detailed_design:
    :param detailed_design_result:
    :param rebar_data:
    :return: ifc
    """
    detailed_design = detailed_design_result.detailed_design
    header_file = {
        "name": "stair.ifc",
        "author": ["Chengran Xu"],
        "organization": ["ZJKJ-CQU"],
        "authorization": "Chengran Xu, ChengranXu@cqu.edu.cn",
    }

    init_doc = {
        "person": {"Identification": "XCR"},
        "application": {"Version": "ifc4x3"},
    }
    # ifc_doc = create_ifc_doc(header_file, init_doc)

    ifc_doc = create_ifc_doc()
    ifc_doc = create_stair_ifc(
        ifc_doc,
        structure_design,
        structure_design_result,
        detailed_design,
        detailed_design_result,
        rebar_data,
    )
    return ifc_doc


def save_stair_ifc_file(ifc_doc):
    """
    保存IFC文件
    :param ifc_doc:
    :return:
    """
    with open(
        os.path.join("data", ifc_doc.ifcfile.wrapped_data.header.file_name.name), "w"
    ) as f:
        f.write(ifc_doc.ifcfile.to_string())
