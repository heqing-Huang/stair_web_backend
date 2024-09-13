"""
# File       : core.py
# Time       ：2022/10/14 17:12
# Author     ：CR_X
# version    ：python 3.8
# Description：通用IFC文件生成, 用于楼梯
"""
import math
import time
import uuid
import warnings
from typing import Union, Optional, List, Dict, Tuple
import hashlib
import logging
import ifcopenshell

# import ifcopenshell.util.pset
from ifcopenshell.file import file
from ifcopenshell.entity_instance import entity_instance
from ifcopenshell import ifcopenshell_wrapper
from ifcopenshell.ifcopenshell_wrapper import schema_definition
from stair_detailed.models import RoundHeadParameter, AnchorParameter, RailParameter
from stair_rebar_layout.models import RebarforBIM
from dc_rebar import Rebar, IndexedPolyCurve
from stair_ifc.get_data import (
    RailingSingleData,
    RoundingHeadSingleData,
    AnchorSingleData,
    StairData,
)
from stair_ifc.rebar_arc_point import IfcRebarData


logger = logging.getLogger(__name__)
Vector = Union[List[float], entity_instance]

"""
file只要开头是六个字母是create，就可以创建一个实体（entity），并调用functools.partial方法
六位以后是IFC数据格式中的实体名（name）
#  例如 createIfcDirection 中 create是判断是否调用functools.partial方法，
"""


class IfcDocument(object):
    """
    封装ifc 相关操作,便于调用
    """

    _ifc_file: file
    _base_x: entity_instance
    _base_y: entity_instance
    _base_z: entity_instance
    _origin: entity_instance
    _global_placement: entity_instance
    _schema: schema_definition
    _person: entity_instance
    _organization: entity_instance
    _application: entity_instance
    _person_and_organization: entity_instance
    _owner_history: entity_instance
    _global_geometry_context: entity_instance
    _project: entity_instance

    def __init__(
        self,
        update_header: Optional[Dict] = None,
        init_document: Optional[Dict] = None,
        schema="IFC4X3",
    ):
        self._ifc_file = file(schema=schema)  # 创建IFC文件instance实例 ifc 格式内存文件,后续操作都需要基于它
        self._origin = self.create_IfcCartesianPoint([0.0, 0.0, 0.0])  # 创建基于IFC的向量表示
        self._base_x = self.create_IfcDirection([1.0, 0.0, 0.0])
        self._base_y = self.create_IfcDirection([0.0, 1.0, 0.0])
        self._base_z = self.create_IfcDirection([0.0, 0.0, 1.0])
        self._global_placement = self.create_IfcAxis2Placement3D(
            self.origin, self.base_z, self.base_x
        )
        self._schema = ifcopenshell_wrapper.schema_by_name(schema)
        if update_header:
            self.update_Header(**update_header)
        else:
            self.update_Header()
        if init_document:
            self.init_document(**init_document)
        else:
            self.init_document()

    def filter_entity_by_attr(
        self, ifc_type: str, attr: str, value: str
    ) -> List[entity_instance]:
        entity_s = []
        for entity in self.ifcfile.by_type(ifc_type):
            if getattr(entity, attr) == value:
                entity_s.append(entity)
        return entity_s

    def update_Header(self, **kwargs):
        """
        更新header文件中的内容，参数输入的示例
        header_file = {'name': 'stair.ifc',
               'author': ['Chengran Xu'],
               'organization': ['ZJKJ-CQU'],
               'authorization': 'Chengran Xu, ChengranXu@cqu.edu.cn'}
        :param kwargs: 字典，其中包含各变量的key
        :return:
        """
        if "name" in kwargs:
            self.ifcfile.wrapped_data.header.file_name.name = kwargs["name"]
        else:
            self.ifcfile.wrapped_data.header.file_name.name = "demo1.ifc"
        if "auther" in kwargs:
            self.ifcfile.wrapped_data.header.file_name.author = kwargs["author"]
        else:
            self.ifcfile.wrapped_data.header.file_name.author = ["Chengran Xu"]
        if "authorization" in kwargs:
            self.ifcfile.wrapped_data.header.file_name.authorization = kwargs[
                "authorization"
            ]
        else:
            self.ifcfile.wrapped_data.header.file_name.authorization = (
                "Chengran Xu, ChengranXu@cqu.edu.cn "
            )
        if "organization" in kwargs:
            self.ifcfile.wrapped_data.header.file_name.organization = kwargs[
                "organization"
            ]
        else:
            self.ifcfile.wrapped_data.header.file_name.organization = ["ZJKJ-CQU"]

    def init_document(self, **kwargs):
        # 创建 person
        if "person" in kwargs:
            person = kwargs["person"]
            self._person = self.create_Person(person)
        else:
            self._person = self.create_Person()
        assert self._person is not None, Exception(f"person_entity is None")

        if "organization" in kwargs:
            organization = kwargs["organization"]
            self._organization = self.create_Organization(organization)
        else:
            self._organization = self.create_Organization()
        assert self._organization is not None, Exception(
            f"_organization_entity is None"
        )

        if "application" in kwargs:
            application = kwargs["application"]
            self._application = self.create_Application(self._organization, application)
        else:
            self._application = self.create_Application(self._organization)
        assert self._application is not None, Exception(f"_application_entity is None")
        self._person_and_organization = self.create_PersonAndOrganization(
            self._person, self._organization
        )

        self._owner_history: entity_instance = self.create_OwnerHistory(
            self._person_and_organization, self._application
        )
        if "project_name" in kwargs:
            project_name = kwargs["project_name"]
            self._project = self.create_project(name=project_name)
        else:
            self._project = self.create_project()

    def create_Unit(self):
        # 设置单位相关
        # 单位相关
        dimensional_exponents: entity_instance = self.ifcfile.create_entity(
            "IfcDimensionalExponents", *[0] * 7
        )
        # 单位采用米
        length_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "LENGTHUNIT", "MILLI", "METRE"
        )
        area_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "AREAUNIT", None, "SQUARE_METRE"
        )
        volume_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "VOLUMEUNIT", None, "CUBIC_METRE"
        )
        time_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "TIMEUNIT", None, "SECOND"
        )
        plane_angle_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "PLANEANGLEUNIT", None, "RADIAN"
        )
        energy_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "ENERGYUNIT", None, "JOULE"
        )
        mass_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "MASSUNIT", "KILO", "GRAM"
        )
        # thermodynamic temperature 热力学温度
        thermodynamic_temperature_unit: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, "THERMODYNAMICTEMPERATUREUNIT", None, "KELVIN"
        )

        # 单位转换,对应角度和弧度的转换单位, 角度被进一步定义为相对于国际单位制单位弧度的换算单位(比例):
        plane_angle_measure: entity_instance = self.ifcfile.create_entity(
            "IfcPlaneAngleMeasure", math.pi / 180
        )

        measure_with_unit: entity_instance = self.ifcfile.create_entity(
            "IfcMeasureWithUnit", plane_angle_measure, plane_angle_unit
        )

        plane_angle_conversion_unit: entity_instance = self.ifcfile.create_entity(
            "IfcConversionBasedUnit",
            dimensional_exponents,
            "PLANEANGLEUNIT",
            "DEGREE",
            measure_with_unit,
        )

        # 比热容

        energy_unit_power: entity_instance = self.ifcfile.create_entity(
            "IfcDerivedUnitElement", energy_unit, 1
        )
        mass_unit_power: entity_instance = self.ifcfile.create_entity(
            "IfcDerivedUnitElement", mass_unit, -1
        )

        thermodynamic_temperature_unit_power: entity_instance = (
            self.ifcfile.create_entity(
                "IfcDerivedUnitElement", thermodynamic_temperature_unit, -1
            )
        )
        specific_heat_capacity_unit: entity_instance = self.ifcfile.create_entity(
            "IfcDerivedUnit",
            [energy_unit_power, mass_unit_power, thermodynamic_temperature_unit_power],
            "SPECIFICHEATCAPACITYUNIT",
            None,
        )

        # 设置单位
        unit_assignment: entity_instance = self.ifcfile.create_entity(
            "IfcUnitAssignment",
            [
                length_unit,
                area_unit,
                volume_unit,
                time_unit,
                plane_angle_conversion_unit,
                specific_heat_capacity_unit,
            ],
        )

        return unit_assignment

    def add_Unit(
        self,
        unit_assignment: Optional[entity_instance] = None,
        add_unit_name: Optional[str] = None,
        add_unit_unit: Optional[str] = None,
    ):
        """
        设置单位，后续添加 IfcSiunit
        :parameter: unit_assignment 已经设置的单位
        :parameter: add_unit_name 增加的单位名称
        :parameter: add_unit_unit 增加的单位的单位
        :return:
        """

        add_unit_entity: entity_instance = self.ifcfile.create_entity(
            "IfcSiunit", None, add_unit_name, None, add_unit_unit
        )
        unit_list = list(unit_assignment.Units)
        unit_list.append(add_unit_entity)
        setattr(unit_assignment, "Units", unit_list)
        return unit_assignment

    def create_geometry_context(self):
        global_placement3d: entity_instance = self.create_IfcAxis2Placement3D()
        global_true_north: entity_instance = self.create_IfcDirection([0.0, 1.0, 0.0])
        # global_true_north: entity_instance = None
        global_geometry_context: entity_instance = self.ifcfile.create_entity(
            "IfcGeometricRepresentationContext",
            None,
            "Model",  # 三维model,二维plan
            3,  # 维度
            1.0e-05,  # 精度
            global_placement3d,
            global_true_north,
        )
        return global_geometry_context

    def create_project(self, name: Optional[str] = None):
        unit_assignment = self.create_Unit()
        self._global_geometry_context = self.create_geometry_context()
        # 定义项目,此时需要给项目传递单位参数,几何空间上下文,以及owner_history
        if name is None:
            name = "Stair"
        ifc_project: entity_instance = self.ifcfile.create_entity(
            "IfcProject",
            self.get_global_id(),
            self.owner_history,
            name,
            None,
            None,
            None,
            None,
            [self._global_geometry_context],
            unit_assignment,
        )
        return ifc_project

    def create_site(self):
        ifc_projects: List[entity_instance] = self.ifcfile.by_type("IfcProject")
        if not ifc_projects:
            raise Exception(f"未成功定义ifcProject")

        site_placement = self.create_IfcLocalplacement(
            relative_to=None, out_placement=self.global_placement
        )
        site = self.ifcfile.createIfcSite(
            self.get_global_id(),
            self.owner_history,
            "Site",
            None,
            None,
            site_placement,
            None,
            None,
            "ELEMENT",
            None,
            None,
            None,
            None,
            None,
        )
        return site, site_placement

    def create_building(self, site_placement: Optional[entity_instance] = None):
        ifc_sites: List[entity_instance] = self.ifcfile.by_type("IfcSite")
        if not ifc_sites:
            raise Exception(f"未成功定义IfcSite")
        building_placement = self.create_IfcLocalplacement(
            site_placement, self.global_placement
        )

        building = self.ifcfile.createIfcBuilding(
            self.get_global_id(),
            self.owner_history,
            "Building",
            None,
            None,
            building_placement,
            None,
            None,
            "ELEMENT",
            None,
            None,
            None,
        )
        return building, building_placement

    def create_building_storey(
        self,
        building_placement: Optional[entity_instance] = None,
        elevation: Optional[float] = 0.0,
    ):
        relative_placement = self.create_IfcAxis2Placement3D(
            origin=[0.0, 0.0, elevation]
        )
        storey_placement = self.create_IfcLocalplacement(
            building_placement, relative_placement
        )

        building_storey = self.ifcfile.createIfcBuildingStorey(
            self.get_global_id(),
            self.owner_history,
            "Storey",
            None,
            None,
            storey_placement,
            None,
            None,
            "ELEMENT",
            elevation,
        )
        return building_storey, storey_placement

    def create_RelAggregates(
        self,
        Name: Optional[str] = None,
        superior_spatial: Optional[entity_instance] = None,
        junior_spatial: Optional[entity_instance] = None,
    ):
        """
        建立容器
        :param Name: "Building Container" "Site Container" "Project Container"
        :param superior_spatial:
        :param junior_spatial:
        :return:
        """
        container = self.ifcfile.createIfcRelAggregates(
            self.get_global_id(),
            self.owner_history,
            Name,
            None,
            superior_spatial,
            [junior_spatial],
        )
        return container

    def create_material_layer(self, concrete_material: Optional[str] = None):

        # 全局使用的材料
        material = self.ifcfile.createIfcMaterial(
            concrete_material, None, "Concrete"
        )  # 楼梯材质
        material_layer = self.ifcfile.createIfcMaterialLayer(material, 0.1, None)
        material_layer_set = self.ifcfile.createIfcMaterialLayerSet(
            [material_layer], None
        )
        material_layer_set_usage = self.ifcfile.createIfcMaterialLayerSetUsage(
            material_layer_set, "AXIS2", "POSITIVE", -0.1
        )
        return material_layer_set_usage

    """
    ＠property 修饰的方法，使得该方法是对象的一个只读属性， 无法重新赋值
    ->描述函数的返回类型
    """

    @property
    def ifcfile(self) -> file:
        return self._ifc_file

    @property
    def origin(self) -> entity_instance:
        return self._origin

    @property
    def base_x(self) -> entity_instance:
        return self._base_x

    @property
    def base_y(self) -> entity_instance:
        return self._base_y

    @property
    def base_z(self) -> entity_instance:
        return self._base_z

    @property
    def global_placement(self) -> entity_instance:
        return self._global_placement

    @property
    def person(self) -> entity_instance:
        return self._person

    @staticmethod
    def get_global_id() -> str:
        """
        生成GlobalId 在整个软件世界中分配全局唯一标识符。
        :return:
        """
        return ifcopenshell.guid.compress(uuid.uuid1().hex)

    @property
    def global_geometry_context(self) -> entity_instance:
        return self._global_geometry_context

    @property
    def organization(self) -> entity_instance:
        return self._organization

    @property
    def application(self) -> entity_instance:
        return self._application

    @property
    def person_and_organization(self) -> entity_instance:
        return self._person_and_organization

    @property
    def owner_history(self) -> entity_instance:
        return self._owner_history

    @property
    def project(self) -> entity_instance:
        return self._project

    def create_IfcLocalplacement(
        self,
        relative_to: Optional[entity_instance] = None,  # LocalPlacement
        out_placement: Optional[entity_instance] = None,
    ) -> entity_instance:  # Axis2Placement
        if out_placement is None:
            out_placement = self.global_placement
        while True:
            ifclocalplacement2: entity_instance = self.ifcfile.createIfcLocalPlacement(
                relative_to, out_placement
            )
            if ifclocalplacement2.get_info().get("id") == 0:
                warnings.warn(f"创建局部坐标系失败,id 为0")
            else:
                break
        return ifclocalplacement2

    def create_IfcCartesianPoint(self, point: List) -> entity_instance:
        """
        create IfcCartesianPoint
        :param point:
        :return:
        """
        while True:
            point_back: entity_instance = self.ifcfile.createIfcCartesianPoint(point)
            if point_back.get_info().get("id") == 0:
                warnings.warn(f"创建点失败,id 为0")
            else:
                break
        return point_back

    def create_IfcDirection(self, vector: List) -> entity_instance:
        """
        create IfcDirection
        :param vector:
        :return:
        """
        while True:
            direction: entity_instance = self.ifcfile.createIfcDirection(vector)
            if direction.get_info().get("id") == 0:
                warnings.warn(f"创建方向向量失败,id为0,将重新创建")
            else:
                break
        return direction

    def create_IfcAxis2Placement3D(
        self, origin: Vector = None, z: Vector = None, x: Vector = None
    ) -> entity_instance:
        """
        创建一个坐标系,满足z轴到x轴右手定则,建立全局或局部坐标系（origin为全局坐标系的坐标）
        通过两个轴创建一个三维坐标系
        创建一个坐标系,z 指向Z 轴,x 指向X 轴，需要满足右手定则
        如果未提供相关参数，默认新建一个并返回全局坐标系
        :param origin:
        :param z:
        :param x:
        :return:
        """
        while True:
            if origin is None and z is None and x is None:
                axis2placement3D: entity_instance = self.global_placement
            else:
                if origin is None:
                    origin = self.origin
                if z is None:
                    z = self.base_z
                if x is None:
                    x = self.base_x
                if not isinstance(origin, entity_instance):
                    origin = self.create_IfcCartesianPoint(origin)
                if not isinstance(z, entity_instance):
                    z = self.create_IfcDirection(z)
                if not isinstance(x, entity_instance):
                    x = self.create_IfcDirection(x)
                axis2placement3D: entity_instance = (
                    self.ifcfile.createIfcAxis2Placement3D(origin, z, x)
                )
            if axis2placement3D.get_info().get("id") == 0:
                warnings.warn(f"坐标系创建id为0")
            else:
                break
        return axis2placement3D

    def create_Person(self, person: Optional[Dict] = None) -> Optional[entity_instance]:
        """
        创建 IfcPerson
        :param person: dic
        :return:
        """

        if person is None:
            person = {"Identification": "Chengran Xu"}
        filter_person_entity_s = self.filter_entity_by_attr(
            "IfcPerson", "Identification", person["Identification"]
        )
        if not filter_person_entity_s:
            ifc_person = self.ifcfile.create_entity("IfcPerson")  # 创建开发人员
        else:
            ifc_person = filter_person_entity_s[0]
        for key, value in person.items():
            setattr(ifc_person, key, value)
        return ifc_person

    def create_Organization(
        self, organization: Optional[Dict] = None
    ) -> Optional[entity_instance]:
        """
        创建组织
        :param organization:
        :return:
        """

        if organization is None:
            organization = {
                "Identification": "ZJKJ-CQU",
                "Name": "中建科技-重庆大学",
                "Description": "中建科技-重庆大学",
            }
        else:
            if "Identification" not in organization:
                organization["Identification"] = "ZJKJ-CQU"
            if "Name" not in organization:
                organization["Name"] = "中建科技-重庆大学"
            if "description" not in organization:
                organization["description"] = "中建科技-重庆大学"
        filter_organization_entity_s = self.filter_entity_by_attr(
            "IfcOrganization", "Identification", organization["Identification"]
        )
        if not filter_organization_entity_s:
            ifc_organization: entity_instance = self.ifcfile.create_entity(
                "IfcOrganization"
            )  # 创建组织
        else:
            ifc_organization = filter_organization_entity_s[0]
        for key, value in organization.items():
            setattr(ifc_organization, key, value)
        return ifc_organization

    def create_Application(
        self,
        organization: Optional[entity_instance] = None,
        application_info: Optional[Dict] = None,
    ):
        if organization is None:
            ifc_organizations: List = self.ifcfile.by_type("IfcOrganization")
            if not ifc_organizations:
                logger.error(f"未找到组织信息,可能无法正确创建IfcApplication")
            else:
                organization = ifc_organizations[0]
        if application_info is None:
            application_info = {
                "Version": "ZJKJ-CQU",
                "ApplicationFullName": "中建科技-重庆大学",
                "ApplicationIdentifier": "中建科技-重庆大学",
            }
        else:
            if "Version" not in application_info:
                application_info["Version"] = ifcopenshell.version
            if "ApplicationFullName" not in application_info:
                application_info["ApplicationFullName"] = "中建科技-重庆大学"
            if "ApplicationIdentifier" not in application_info:
                application_info["ApplicationIdentifier"] = "中建科技-重庆大学"
        filter_application_entity_s = self.filter_entity_by_attr(
            "IfcApplication",
            "ApplicationIdentifier",
            application_info["ApplicationIdentifier"],
        )
        if not filter_application_entity_s:
            ifc_application: entity_instance = self.ifcfile.create_entity(
                "IfcApplication", organization
            )  # 应用
        else:
            ifc_application = filter_application_entity_s[0]
        for key, value in application_info.items():
            setattr(ifc_application, key, value)
        return ifc_application

    def create_PersonAndOrganization(
        self,
        person: Optional[entity_instance] = None,
        organization: Optional[entity_instance] = None,
    ) -> entity_instance:
        if person is None:
            ifc_persons: List = self.ifcfile.by_type("IfcPerson")
            if not ifc_persons:
                logger.error(f"未找到人员信息,可能无法正确创建IfcPersonAndOrganization")
            else:
                person = ifc_persons[0]
        if organization is None:
            ifc_organizations: List = self.ifcfile.by_type("IfcOrganization")
            if not ifc_organizations:
                logger.error(f"未找到组织信息,可能无法正确创建IfcPersonAndOrganization")
            else:
                organization = ifc_organizations[0]

        ifc_person_and_organization: entity_instance = self.ifcfile.create_entity(
            "IfcPersonAndOrganization", person, organization
        )  # 组织和人员合并
        return ifc_person_and_organization

    def create_OwnerHistory(
        self,
        person_and_organization: Optional[entity_instance] = None,
        application: Optional[entity_instance] = None,
    ) -> Optional[entity_instance]:
        owner_history: Optional[entity_instance] = None
        if person_and_organization is None:
            ifc_person_organizations: List = self.ifcfile.by_type(
                "IfcPersonAndOrganization"
            )
            if ifc_person_organizations:
                person_and_organization = ifc_person_organizations[-1]
            else:
                logger.warning(f"项目文件中未能找到IfcPersonAndOrganization")
        if application is None:
            ifc_applications: List[entity_instance] = self.ifcfile.by_type(
                "IfcApplication"
            )
            if ifc_applications:
                application = ifc_applications[-1]
            else:
                logger.warning(f"项目文件中未能找到IfcApplication")
        if person_and_organization and application:
            while True:
                owner_history = self.ifcfile.create_entity(
                    "IfcOwnerHistory",
                    person_and_organization,
                    application,
                    None,
                    "ADDED",
                    None,
                    person_and_organization,
                    application,
                    int(time.time()),
                )
                if owner_history.get_info().get("id") == 0:
                    warnings.warn(
                        f"产生的history为0,{owner_history.get_info()},"
                        f"application:{application.get_info()},"
                        f"person_and_organization:{person_and_organization.get_info()}"
                    )
                else:
                    break
        else:
            logger.warning(f"未能成功寻找到IfcPersonAndOrganization或IfcApplication")
        return owner_history

    def create_ExtrudedAreaSolid(
        self,
        points: List = None,
        ifcaxis2placement: entity_instance = None,
        extrude_direction: List = None,
        extrude_length: float = None,
    ) -> entity_instance:
        # 局部坐标系中的拉伸方向
        ifc_direction = self.ifcfile.create_entity("IfcDirection", extrude_direction)
        ifc_points = []
        for point in points:
            point = self.ifcfile.createIfcCartesianPoint(point)
            ifc_points.append(point)
        poly_line = self.ifcfile.createIfcPolyLine(ifc_points)
        ifcclosedprofile = self.ifcfile.createIfcArbitraryClosedProfileDef(
            "AREA", None, poly_line
        )  # 封闭平面几何
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
        ifc_extrudedareasolid = self.ifcfile.createIfcExtrudedAreaSolid(
            ifcclosedprofile, ifcaxis2placement, ifc_direction, extrude_length
        )
        return ifc_extrudedareasolid

    def create_ExtrudedAreaSolidTapered(
        self,
        section_1: List = None,
        ifcaxis2placement: entity_instance = None,
        extrude_direction: List = None,
        extrude_length: float = None,
        section_2: List = None,
    ) -> entity_instance:

        ifc_direction = self.ifcfile.create_entity("IfcDirection", extrude_direction)
        ifc_section_1 = []
        for point in section_1:
            point = self.ifcfile.createIfcCartesianPoint(point)
            ifc_section_1.append(point)
        poly_line_1 = self.ifcfile.createIfcPolyLine(ifc_section_1)
        ifcclosedprofile_1 = self.ifcfile.createIfcArbitraryClosedProfileDef(
            "AREA", None, poly_line_1
        )  # 封闭平面几何
        ifc_section_2 = []
        for point in section_2:
            point = self.ifcfile.createIfcCartesianPoint(point)
            ifc_section_2.append(point)
        poly_line_2 = self.ifcfile.createIfcPolyLine(ifc_section_2)
        ifcclosedprofile_2 = self.ifcfile.createIfcArbitraryClosedProfileDef(
            "AREA", None, poly_line_2
        )  # 封闭平面几何
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
        ifc_extrudedareasolid = self.ifcfile.createIfcExtrudedAreaSolidTapered(
            ifcclosedprofile_1,
            ifcaxis2placement,
            ifc_direction,
            extrude_length,
            ifcclosedprofile_2,
        )
        return ifc_extrudedareasolid

    def create_SweptDiskSolid(self, rebar: IfcRebarData):
        """
        定义一个通用钢筋的绘制形式
        """
        points_entity_instance = self.ifcfile.create_entity(
            "IfcCartesianPointList3D", rebar.points
        )
        seg_s_entity_instance = []
        for seg in rebar.seg_s:
            seg_s_entity_instance.append(
                self.ifcfile.create_entity(seg.seg_type, seg.index_s)
            )
        ifc_indexed_poly_curve = self.ifcfile.create_entity(
            "IfcIndexedPolyCurve", points_entity_instance, seg_s_entity_instance
        )
        solid = self.ifcfile.create_entity(
            "IfcSweptDiskSolid",
            ifc_indexed_poly_curve,
            float(rebar.radius),
            None,
            None,
            None,
        )  # InnerRadius 空心圆柱内半径

        return solid

    def create_RevolvedAreaSolid(
        self, section: List, solid_placement, Axis1Placement, angle=360
    ):
        """
        定义一个圆台，半横截面是多边形
        """
        ifc_points = []
        for point in section:
            point = self.ifcfile.createIfcCartesianPoint(point)
            ifc_points.append(point)
        poly_line = self.ifcfile.createIfcPolyLine(ifc_points)
        ifcclosedprofile = self.ifcfile.createIfcArbitraryClosedProfileDef(
            "AREA", None, poly_line
        )  # 封闭平面几何
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyzbeam_placement_2
        solid = self.ifcfile.createIfcRevolvedAreaSolid(
            ifcclosedprofile, solid_placement, Axis1Placement, angle
        )
        return solid

    def create_SurfaceCurveSweptAreaSolid(
        self, curve: List, points: List = None, water_drop_shape=None
    ) -> entity_instance:
        curve_points = []
        for p in curve:
            p = self.ifcfile.createIfcCartesianPoint(p)
            curve_points.append(p)
        ifc_curve = self.ifcfile.createIfcPolyLine(curve_points)

        if water_drop_shape.value == 0:  # TRAPEZOID = 0 SEMICIRCLE = 1
            profile_seg_s = [
                self.ifcfile.create_entity("IfcLineIndex", [1, 2]),
                self.ifcfile.create_entity("IfcLineIndex", [2, 3]),
                self.ifcfile.create_entity("IfcLineIndex", [3, 4]),
                self.ifcfile.create_entity("IfcLineIndex", [4, 1]),
            ]
            points_list = self.ifcfile.create_entity("IfcCartesianPointList2D", points)
            profile = self.ifcfile.createIfcIndexedPolyCurve(points_list, profile_seg_s)
            ifc_closedprofile = self.ifcfile.createIfcArbitraryClosedProfileDef(
                "AREA", None, profile
            )  # 封闭平面几何
        else:
            profile_seg_s = [
                self.ifcfile.create_entity("IfcArcIndex", [1, 2, 3]),
                self.ifcfile.create_entity("IfcLineIndex", [3, 1]),
            ]
            points_list = self.ifcfile.create_entity("IfcCartesianPointList2D", points)
            profile = self.ifcfile.createIfcIndexedPolyCurve(points_list, profile_seg_s)

            ifc_closedprofile = self.ifcfile.createIfcArbitraryClosedProfileDef(
                "AREA", None, profile
            )  # 封闭平面几何
        # placement = self.create_IfcAxis2Placement3D(z=[0., 0., -1.], x=[0., 1., 0.])
        ifc_surface = self.ifcfile.createIfcPlane(self.global_placement)
        # 创建截面沿曲线拉伸
        solid = self.ifcfile.createIfcSurfaceCurveSweptAreaSolid(
            ifc_closedprofile, self.global_placement, ifc_curve, None, None, ifc_surface
        )
        return solid

    def create_hole(self, stair, stair_placement, stair_data: StairData):
        hole_positions = [
            stair_data.get_top_hole_position(),
            stair_data.get_bottom_hole_position(),
        ]
        hole_sections = [
            stair_data.get_top_hole_section(),
            stair_data.get_bottom_hole_section(),
        ]
        revolve_point = self.ifcfile.createIfcCartesianPoint([0.0, 0.0, 0.0])
        revolve_direction = self.ifcfile.createIfcDirection([0.0, 1.0, 0.0])
        Axis1Placement = self.ifcfile.create_entity(
            "IfcAxis1Placement", revolve_point, revolve_direction
        )
        opening_solid_placement = self.create_IfcAxis2Placement3D(
            origin=[0.0, 0.0, 0.0], z=[0.0, 1.0, 0.0], x=[-1.0, 0.0, 0.0]
        )
        for i in range(len(hole_positions)):
            for position in hole_positions[i]:
                # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
                opening_solid = self.create_RevolvedAreaSolid(
                    hole_sections[i], opening_solid_placement, Axis1Placement, 400
                )
                # 创建开洞
                opening_representation = self.ifcfile.createIfcShapeRepresentation(
                    self.global_geometry_context, "Body", "SweptSolid", [opening_solid]
                )
                opening_shape = self.ifcfile.createIfcProductDefinitionShape(
                    None, None, [opening_representation]
                )
                opening_coordinate = self.create_IfcAxis2Placement3D(origin=position)
                opening_local_placement = self.create_IfcLocalplacement(
                    relative_to=stair_placement, out_placement=opening_coordinate
                )
                opening_element = self.ifcfile.createIfcOpeningElement(
                    self.get_global_id(),
                    self.owner_history,
                    "Opening",
                    "An awesome opening",
                    None,
                    opening_local_placement,
                    opening_shape,
                    None,
                )
                self.ifcfile.createIfcRelVoidsElement(
                    self.get_global_id(),
                    self.owner_history,
                    None,
                    None,
                    stair,
                    opening_element,
                )

            # # 赋予属性示例
            # quantity_values = [self.ifcfile.createIfcQuantityLength("depth", "洞深", None, 200), ]
            # element_quantity = self.ifcfile.createIfcElementQuantity(self.get_global_id(), self.owner_history,
            #                                                          "BaseQuantities",
            #                                                          None, None,
            #                                                          quantity_values)
            # self.ifcfile.createIfcRelDefinesByProperties(self.get_global_id(), self.owner_history, None, None,
            #                                              [opening_element],
            #                                              element_quantity)

    def create_step_slot(self, stair, stair_placement, stair_data: StairData):
        if stair_data.step_slot_mode.value != 2:
            positions = stair_data.get_step_slot_position()
            slot_positions = stair_data.get_slot_position()
            slot_length = stair_data.get_slot_length()
            slot_section = stair_data.get_slot_section()

            slope_positions = stair_data.get_slope_position()
            slope_sections_1, slope_sections_2 = stair_data.get_slope_section()
            slope_length = stair_data.get_slope_length()
            for i in range(len(positions)):
                for j in range(len(slot_positions)):
                    solids = []
                    slot_direction = [0.0, 0.0, 1.0]
                    slot_placement = self.create_IfcAxis2Placement3D(
                        origin=slot_positions[j], z=[1.0, 0.0, 0.0], x=[0.0, 1.0, 0.0]
                    )
                    step_slot_solid = self.create_ExtrudedAreaSolid(
                        slot_section, slot_placement, slot_direction, slot_length
                    )
                    solids.append(step_slot_solid)

                    for k in range(len(slope_positions)):
                        slope_placement = self.create_IfcAxis2Placement3D(
                            origin=slope_positions[j][k],
                            z=[0.0, 0.0, -1.0],
                            x=[0.0, 1.0, 0.0],
                        )
                        ifc_direction = self.ifcfile.create_entity(
                            "IfcDirection", [0.0, 0.0, 1.0]
                        )

                        ifc_section_1 = []
                        for point in slope_sections_1[k]:
                            point = self.ifcfile.createIfcCartesianPoint(point)
                            ifc_section_1.append(point)
                        ifc_polyline_1 = self.ifcfile.createIfcPolyLine(ifc_section_1)
                        ifc_profile_1 = self.ifcfile.createIfcArbitraryClosedProfileDef(
                            "AREA", None, ifc_polyline_1
                        )

                        ifc_profile_position = self.ifcfile.createIfcCartesianPoint(
                            slope_sections_2[k]
                        )
                        ifc_CTO = (
                            self.ifcfile.createIfcCartesianTransformationOperator2D(
                                None, None, ifc_profile_position, 1e-5
                            )
                        )
                        ifc_profile_2 = self.ifcfile.createIfcDerivedProfileDef(
                            "AREA", None, ifc_profile_1, ifc_CTO, None
                        )
                        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
                        slope_solid = self.ifcfile.createIfcExtrudedAreaSolidTapered(
                            ifc_profile_1,
                            slope_placement,
                            ifc_direction,
                            slope_length,
                            ifc_profile_2,
                        )

                        solids.append(slope_solid)
                    representation = self.ifcfile.createIfcShapeRepresentation(
                        self.global_geometry_context, "Body", "SweptSolid", solids
                    )
                    step_slot_shape = self.ifcfile.createIfcProductDefinitionShape(
                        None, None, [representation]
                    )

                    step_slot_local_placement = self.create_IfcLocalplacement(
                        stair_placement,
                        self.create_IfcAxis2Placement3D(origin=positions[i]),
                    )
                    # 附着到主体上成为开洞
                    step_slot_element = self.ifcfile.createIfcOpeningElement(
                        self.get_global_id(),
                        self.owner_history,
                        "防滑槽",
                        "防滑槽",
                        None,
                        step_slot_local_placement,
                        step_slot_shape,
                        None,
                    )
                    self.ifcfile.createIfcRelVoidsElement(
                        self.get_global_id(),
                        self.owner_history,
                        None,
                        None,
                        stair,
                        step_slot_element,
                    )

    def create_water_drop(self, stair, stair_placement, stair_data: StairData):
        if stair_data.water_drop_mode.value != 2:
            curves = stair_data.get_water_drip_curve()
            sections, section_shape = stair_data.get_water_drop_section()
            for i in range(len(curves)):
                water_drop_solid = self.create_SurfaceCurveSweptAreaSolid(
                    curves[i], sections[i], section_shape
                )
                representation = self.ifcfile.createIfcShapeRepresentation(
                    self.global_geometry_context,
                    "Body",
                    "SweptSolid",
                    [water_drop_solid],
                )
                water_drop_shape = self.ifcfile.createIfcProductDefinitionShape(
                    None, None, [representation]
                )

                water_drop_local_placement = self.create_IfcLocalplacement(
                    stair_placement, self.global_placement
                )
                # 附着到主体上成为开洞
                water_drop_element = self.ifcfile.createIfcOpeningElement(
                    self.get_global_id(),
                    self.owner_history,
                    "滴水槽",
                    "滴水槽",
                    None,
                    water_drop_local_placement,
                    water_drop_shape,
                    None,
                )

                self.ifcfile.createIfcRelVoidsElement(
                    self.get_global_id(),
                    self.owner_history,
                    None,
                    None,
                    stair,
                    water_drop_element,
                )

    def create_railing_embedded_parts(
        self,
        railing_placement,
        rail_parameter: RailParameter,
        stair,
        stair_placement,
        building_storey=None,
    ):
        railing_signle_data = RailingSingleData(rail_parameter)
        (
            rabbet_section_1,
            rabbet_section_2,
            rabbet_depth,
        ) = railing_signle_data.get_rail_embedded_parts_rabbet()
        plate_section, plate_thickness = railing_signle_data.get_single_rail_plate()
        rebars: List[IfcRebarData] = railing_signle_data.get_single_rail_rebar()

        extrude_direction = [0.0, 0.0, 1.0]

        rabbet_placement = self.create_IfcAxis2Placement3D(origin=[0.0, 0.0, 0.0])

        rabbet_solid = self.create_ExtrudedAreaSolidTapered(
            rabbet_section_1,
            rabbet_placement,
            extrude_direction,
            rabbet_depth,
            rabbet_section_2,
        )
        rabbet_solid_representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [rabbet_solid]
        )
        rabbet_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [rabbet_solid_representation]
        )
        rail_local_placement = self.create_IfcLocalplacement(
            stair_placement, railing_placement
        )
        # 附着到主体上成为开洞
        rabbet_element = self.ifcfile.createIfcOpeningElement(
            self.get_global_id(),
            self.owner_history,
            "栏杆预埋件企口",
            "栏杆预埋件企口",
            None,
            rail_local_placement,
            rabbet_shape,
            None,
        )
        self.ifcfile.createIfcRelVoidsElement(
            self.get_global_id(), self.owner_history, None, None, stair, rabbet_element
        )
        extrude_direction = [0.0, 0.0, 1.0]
        # 栏杆预埋件
        solids = []
        plate_placement = self.create_IfcAxis2Placement3D(
            origin=[0.0, 0.0, rabbet_depth]
        )
        plate_solid = self.create_ExtrudedAreaSolid(
            plate_section, plate_placement, extrude_direction, plate_thickness
        )
        solids.append(plate_solid)

        for j in range(len(rebars)):
            rebar_solid = self.create_SweptDiskSolid(rebars[j])
            solids.append(rebar_solid)

        if len(solids) > 1:

            csg_handle = self.ifcfile.create_entity(
                "IfcBooleanResult", "UNION", solids[0], solids[1]
            )

            if len(solids) > 2:
                for i in range(2, len(solids)):
                    csg_handle = self.ifcfile.create_entity(
                        "IfcBooleanResult", "UNION", csg_handle, solids[i]
                    )

            csg_solid = self.ifcfile.create_entity("IfcCsgSolid", csg_handle)

        else:
            csg_solid = solids[0]
        representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [csg_solid]
        )
        rail_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [representation]
        )
        rail = self.ifcfile.create_entity(
            "IfcElementAssembly",
            self.get_global_id(),
            self.owner_history,
            "栏杆埋件",
            None,
            None,
            rail_local_placement,
            rail_shape,
            None,
        )

        # entity_instance_write = self.ifcfile.createIfcRelContainedInSpatialStructure(
        #     self.get_global_id(),
        #     self.owner_history,
        #     "Building Storey Container",
        #     None,
        #     [rail],
        #     building_storey)  #
        rel_aggregate = self.ifcfile.createIfcRelAggregates(
            self.get_global_id(),
            self.owner_history,
            "Stair Container",
            None,
            stair,
            [rail],
        )

    # def create_railing(self, stair, stair_placement, railing_data: RailingEmbeddedPartsData, building_storey):
    def create_railing(self, stair, stair_placement, stair_data: StairData):
        if stair_data.rail_design_mode.value == 1:  # MANUAL = 1  NO = 2
            positions = stair_data.get_rail_embedded_parts_position()
            rail_parameter = stair_data.rail_parameter
            for i in range(len(positions)):
                railing_placement = self.create_IfcAxis2Placement3D(
                    origin=positions[i], z=[0.0, 0.0, -1.0], x=[0.0, 1.0, 0.0]
                )
                # self.create_railing_embedded_parts(railing_placement, rail_parameter, stair, stair_placement,
                #                                    building_storey)
                self.create_railing_embedded_parts(
                    railing_placement, rail_parameter, stair, stair_placement
                )

    def create_rounding_head(
        self,
        lifting_local_placement: entity_instance = None,
        revolve_placement=None,
        revolve_Axis1Placement=None,
        rounding_head_parameter: RoundHeadParameter = None,
        stair: entity_instance = None,
        building_storey=None,
    ):
        single_rounding_head = RoundingHeadSingleData(rounding_head_parameter)
        rabbet_section = single_rounding_head.get_rounding_head_rabbet()
        ring_section = single_rounding_head.get_rounding_head_ring()
        if revolve_Axis1Placement == None:
            revolve_point = self.ifcfile.createIfcCartesianPoint([0.0, 0.0, 0.0])
            revolve_direction = self.ifcfile.createIfcDirection([1.0, 0.0, 0.0])
            revolve_Axis1Placement = self.ifcfile.create_entity(
                "IfcAxis1Placement", revolve_point, revolve_direction
            )
        if revolve_placement == None:
            revolve_placement = self.create_IfcAxis2Placement3D(
                origin=[0.0, 0.0, 0.0], z=[-1.0, 0.0, 0.0], x=[0.0, 0.0, 1.0]
            )

        rabbet_polyline_segs = [
            self.ifcfile.create_entity("IfcLineIndex", [1, 2]),
            self.ifcfile.create_entity("IfcArcIndex", [2, 3, 4]),
            self.ifcfile.create_entity("IfcLineIndex", [4, 1]),
        ]
        rabbet_points_list = self.ifcfile.create_entity(
            "IfcCartesianPointList2D", rabbet_section
        )
        rabbet_polyline = self.ifcfile.createIfcIndexedPolyCurve(
            rabbet_points_list, rabbet_polyline_segs
        )

        rabbet_profile = self.ifcfile.createIfcArbitraryClosedProfileDef(
            "AREA", None, rabbet_polyline
        )  # 封闭平面几何
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
        rabbet_solid = self.ifcfile.createIfcRevolvedAreaSolid(
            rabbet_profile, revolve_placement, revolve_Axis1Placement, 360
        )
        rabbet_solid_representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [rabbet_solid]
        )
        rabbet_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [rabbet_solid_representation]
        )
        # 附着到主体上成为开洞
        rabbet_element = self.ifcfile.createIfcOpeningElement(
            self.get_global_id(),
            self.owner_history,
            "吊装预埋件企口",
            "吊装预埋件企口",
            None,
            lifting_local_placement,
            rabbet_shape,
            None,
        )
        self.ifcfile.createIfcRelVoidsElement(
            self.get_global_id(), self.owner_history, None, None, stair, rabbet_element
        )

        # 圆头吊钉预埋件
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
        ring_solid = self.create_RevolvedAreaSolid(
            ring_section, revolve_placement, revolve_Axis1Placement, 360
        )
        # 创建开洞
        ring_representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [ring_solid]
        )
        ring_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [ring_representation]
        )
        ring = self.ifcfile.create_entity(
            "IfcElementAssembly",
            self.get_global_id(),
            self.owner_history,
            "吊钉埋件",
            None,
            None,
            lifting_local_placement,
            ring_shape,
            None,
        )
        # entity_instance_write = self.ifcfile.createIfcRelContainedInSpatialStructure(
        #     self.get_global_id(),
        #     self.owner_history,
        #     "Building Storey Container",
        #     None,
        #     [ring],
        #     building_storey)  #
        rel_aggregate = self.ifcfile.createIfcRelAggregates(
            self.get_global_id(),
            self.owner_history,
            "Stair Container",
            None,
            stair,
            [ring],
        )

    def create_anchor(
        self,
        lifting_local_placement: entity_instance = None,
        revolve_placement=None,
        revolve_Axis1Placement=None,
        achor_parameter: AnchorParameter = None,
        stair: entity_instance = None,
        building_storey=None,
    ):
        single_anchor = AnchorSingleData(achor_parameter)
        rabbet_section = single_anchor.get_anchor_rabbet()
        anchor_section = single_anchor.get_anchor()
        rebar = single_anchor.get_anchor_rebar()
        rabbet_solid = self.create_RevolvedAreaSolid(
            rabbet_section, revolve_placement, revolve_Axis1Placement, 360
        )
        rabbet_solid_representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [rabbet_solid]
        )
        rabbet_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [rabbet_solid_representation]
        )
        # 附着到主体上成为开洞
        rabbet_element = self.ifcfile.createIfcOpeningElement(
            self.get_global_id(),
            self.owner_history,
            "吊装预埋件企口",
            "吊装预埋件企口",
            None,
            lifting_local_placement,
            rabbet_shape,
            None,
        )
        self.ifcfile.createIfcRelVoidsElement(
            self.get_global_id(), self.owner_history, None, None, stair, rabbet_element
        )
        # 锚栓预埋件
        # 创建拉伸的几何体，局部使用的坐标系是固定的，xyz
        # 创建锚栓
        anchor_solid = self.create_RevolvedAreaSolid(
            anchor_section, revolve_placement, revolve_Axis1Placement, 360
        )
        # 创建锚栓中的小钢筋
        rebar_solid = self.create_SweptDiskSolid(rebar)
        # 合并成一个预埋件
        csg_handle = self.ifcfile.create_entity(
            "IfcBooleanResult", "UNION", anchor_solid, rebar_solid
        )
        csg_solid = self.ifcfile.create_entity("IfcCsgSolid", csg_handle)
        anchor_representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [csg_solid]
        )
        anchor_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [anchor_representation]
        )
        anchor = self.ifcfile.create_entity(
            "IfcElementAssembly",
            self.get_global_id(),
            self.owner_history,
            "锚栓埋件",
            None,
            None,
            lifting_local_placement,
            anchor_shape,
            None,
        )
        # entity_instance_write = self.ifcfile.createIfcRelContainedInSpatialStructure(
        #     self.get_global_id(),
        #     self.owner_history,
        #     "Building Storey Container",
        #     None,
        #     [anchor],
        #     building_storey)  #
        rel_aggregate = self.ifcfile.createIfcRelAggregates(
            self.get_global_id(),
            self.owner_history,
            "Stair Container",
            None,
            stair,
            [anchor],
        )

    # def create_lifting(self, stair, stair_placement, lifting_data: LiftingEmbeddedPartsData, building_storey):
    def create_lifting(self, stair, stair_placement, stair_data: StairData):
        positions = stair_data.get_lifting_embedded_parts_position()

        lifting_parameter = stair_data.lifting_parameter
        revolve_point = self.ifcfile.createIfcCartesianPoint([0.0, 0.0, 0.0])
        revolve_direction = self.ifcfile.createIfcDirection([1.0, 0.0, 0.0])
        revolve_Axis1Placement = self.ifcfile.create_entity(
            "IfcAxis1Placement", revolve_point, revolve_direction
        )
        revolve_placement = self.create_IfcAxis2Placement3D(
            origin=[0.0, 0.0, 0.0], z=[-1.0, 0.0, 0.0], x=[0.0, 0.0, 1.0]
        )

        if stair_data.lifting_type.value == 0:
            for i in range(len(positions)):
                lifting_placement = self.create_IfcAxis2Placement3D(
                    positions[i], z=[0.0, 0.0, -1.0], x=[1.0, 0.0, 0.0]
                )
                lifting_local_placement = self.create_IfcLocalplacement(
                    stair_placement, lifting_placement
                )
                # self.create_rounding_head(lifting_local_placement, revolve_placement,revolve_Axis1Placement,
                #                           lifting_parameter, stair, building_storey)
                self.create_rounding_head(
                    lifting_local_placement,
                    revolve_placement,
                    revolve_Axis1Placement,
                    lifting_parameter,
                    stair,
                )
        else:
            for i in range(len(positions)):
                lifting_placement = self.create_IfcAxis2Placement3D(
                    positions[i], z=[0.0, 0.0, -1.0], x=[1.0, 0.0, 0.0]
                )
                lifting_local_placement = self.create_IfcLocalplacement(
                    stair_placement, lifting_placement
                )
                # self.create_anchor(lifting_local_placement, revolve_placement,revolve_Axis1Placement,
                #                    lifting_parameter, stair, building_storey)
                self.create_anchor(
                    lifting_local_placement,
                    revolve_placement,
                    revolve_Axis1Placement,
                    lifting_parameter,
                    stair,
                )

    # def create_demoulding(self, stair, stair_placement, demoulding_data: DemouldingEmbeddedPartsData, building_storey):
    def create_demoulding(self, stair, stair_placement, stair_data: StairData):

        if stair_data.pouring_way.value != 2:  # 0 立式浇筑卧式脱模，1  # 立式浇筑立式脱模，2  # 卧式浇筑卧式脱模
            (
                positions,
                z_direction,
                x_direction,
            ) = stair_data.get_demoulding_embedded_parts_position()
            demoulding_parameter = stair_data.demolding_parameter
            revolve_point = self.ifcfile.createIfcCartesianPoint([0.0, 0.0, 0.0])
            revolve_direction = self.ifcfile.createIfcDirection([1.0, 0.0, 0.0])
            revolve_Axis1Placement = self.ifcfile.create_entity(
                "IfcAxis1Placement", revolve_point, revolve_direction
            )
            revolve_placement = self.create_IfcAxis2Placement3D(
                origin=[0.0, 0.0, 0.0], z=[-1.0, 0.0, 0.0], x=[0.0, 0.0, 1.0]
            )
            if stair_data.demolding_type.value == 0:  # ROUNDING_HEAD = 0    ANCHOR = 1
                for i in range(len(positions)):
                    demoulding_position = self.create_IfcAxis2Placement3D(
                        positions[i], z=z_direction, x=x_direction
                    )
                    demoulding_local_placement = self.create_IfcLocalplacement(
                        stair_placement, demoulding_position
                    )
                    # self.create_rounding_head(demoulding_local_placement,revolve_placement, revolve_Axis1Placement,
                    #                           demoulding_parameter, stair, building_storey)
                    self.create_rounding_head(
                        demoulding_local_placement,
                        revolve_placement,
                        revolve_Axis1Placement,
                        demoulding_parameter,
                        stair,
                    )
            else:  # stair_data.demolding_type.value == 1:
                for i in range(len(positions)):
                    demoulding_position = self.create_IfcAxis2Placement3D(
                        positions[i], z=z_direction, x=x_direction
                    )
                    demoulding_local_placement = self.create_IfcLocalplacement(
                        stair_placement, demoulding_position
                    )
                    # self.create_anchor(demoulding_local_placement, revolve_placement, revolve_Axis1Placement,
                    #                    demoulding_parameter, stair, building_storey)
                    self.create_anchor(
                        demoulding_local_placement,
                        revolve_placement,
                        revolve_Axis1Placement,
                        demoulding_parameter,
                        stair,
                    )

    def create_rebar(self, stair_placement, rebar: IfcRebarData, name: str = "钢筋"):
        """
        通用的钢筋绘制函数
        :param rebar_data:
        :param name:
        :param iradius:
        :return:
        """

        solid = self.create_SweptDiskSolid(rebar)
        representation = self.ifcfile.createIfcShapeRepresentation(
            self.global_geometry_context, "Body", "SweptSolid", [solid]
        )

        rebar_shape = self.ifcfile.createIfcProductDefinitionShape(
            None, None, [representation]
        )
        local_placement = self.ifcfile.createIfcLocalPlacement(
            stair_placement, self.global_placement
        )
        instance_show = self.ifcfile.create_entity(
            "IfcReinforcingBar",
            self.get_global_id(),
            self.owner_history,
            name,
            None,
            None,
            local_placement,
            rebar_shape,
            None,
        )

        return instance_show
