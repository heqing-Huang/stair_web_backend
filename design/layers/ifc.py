"""
后端程序 和 IFC 的中间层

用于预防底层(stair_ifc) 发生较大接口变动
"""
from abc import ABC, abstractmethod
from typing import Type
from attrs import define, field
import cattrs

from stair_structure.model import StructuralDesign, StructuralDesignResult
from stair_detailed.models import DetailedDesign, DetailedDesignResult
from stair_rebar_layout.models import RebarforBIM
from stair_ifc.create_ifc import stair_IFC_creation


@define
class StairTotalForIfc:
    structure_design: StructuralDesign
    structure_result: StructuralDesignResult
    detailed_design: DetailedDesign
    detailed_result: DetailedDesignResult
    rebars: RebarforBIM


# 后续版本控制
ParameterIfc = StairTotalForIfc


class ActionCreate(ABC):
    @abstractmethod
    def make(self, data: ParameterIfc) -> str:
        """
        约定一个接口,将需要的数据传入,返回生成的ifc 的内容
        Args:
            data:

        Returns:

        """
        pass


class IfcCreateV1(ActionCreate):
    """
    目前真实的,用于创建IFC 的类型实现
    """

    def make(self, data: ParameterIfc) -> str:
        doc = stair_IFC_creation(
            data.structure_design,
            data.structure_result,
            data.detailed_design,
            data.detailed_result,
            data.rebars,
        )
        return doc.ifcfile.to_string()


IfcHandler: Type[ActionCreate] = IfcCreateV1
