import json
import logging
import os
from unittest import skip, skipIf
from datetime import datetime
from dataclasses import asdict

from django.test import TestCase
from django.forms.models import model_to_dict
from django.core.files.base import ContentFile

from stair_detailed import __version__ as detailed_v
from stair_detailed.models import (
    HoleDesignMode,
    JointDesignMode,
    StepSlotDesignMode,
    WaterDripDesignMode,
    HoleType,
    WaterDripLayout,
    WaterDripShape,
    RebarDesignMode,
    LiftingDesignMode,
    DemoldingDesignMode,
    RailDesignMode,
    LiftingType as HoistType,
    PouringWay,
    DemoldingType as DemoldType,
    RailLayout,
    GeometricDetailed,
    RebarDetailed,
    RebarDiamSpac,
    RebarDiam,
    ConstructionDetailed,
    FixedHinge,
    TopHolePosition,
    SlidingHinge,
    BottomHolePosition,
    WaterDripPosition,
    WaterDripTrapezoid,
    InsertsDetailed,
    RailPosition,
    DetailedDesign,
)
from stair_detailed.detailed_design import detailed_design
from stair_structure.model import (
    StructuralDesignResult as StructureResult,
    StairID,
    Material,
    Construction,
    Geometric,
    LoadData,
    LimitSetting,
    StructuralDesign,
)
from stair_structure.structure_calculation import structure_cal
from stair_rebar_layout.Rebar_layout import rebar_layout

from stair_for_bvbs.data_for_bvbs import data_for_bvbs
from stair_rebar_bvbs.create_bvbs import create_bvbs, save_string_to_file
from stair_rebar_bvbs.create_JSON import create_json, save_string_to_json_file

from design import models as db_models
from .models import (
    ModelConstructionData,
    ModelConstructionResult,
    DetailData,
    RebarLayoutModel,
    DetailDataCopyChangeWrite,
)
from .tasks import test
from . import tools
from .tools import (
    call_structural_calculation,
    call_detailed_design_by_model,
    call_design_book,
    call_rebar_layout,
    call_ifc_create,
)
from . import exchange
from design import layers

_logger = logging.getLogger(__name__)

HoistDesignMode = LiftingDesignMode
DemoldDesignMode = DemoldingDesignMode


# Create your tests here.


class TestShowStructure(TestCase):
    def test_show_parameters(self):
        """
        仅供开发期间,输出楼梯结构设计需要的参数,并对比转换
        :return:
        """
        construction = ModelConstructionData()
        parameters = exchange.structure.StructureDataSerializer(
            instance=construction
        ).data
        _logger.info(f"参数为:{parameters}")

    def test_re_serializer(self):
        """
        将结构计算结果存放于数据库中
        :return:
        """
        kwargs = dict(
            # start 楼梯几何参数
            steps_h=20,
            steps_b=30,
            l0=10,
            cos=1,
            gkt=10,
            gk=10,
            png=10,
            pnl=10,
            pm=10,
            # # 正截面受弯承载力
            m_max=10,
            h0=10,
            alpha_s=10,
            ksi=10,
            ksi_status=True,
            p_c_min=10,
            p_c=10,
            as_1=10,
            as_2=10,
            as_3=10,
            # 挠度
            mq=10,
            sigma_sq=10,
            a_te=10,
            p_te=10,
            fi_i=10,
            fi=10,
            alpha_e=10,
            gama_f=10,
            p_t=10,
            b_s=10,
            b_l=10,
            theta=10,
            m_theta=10,
            deflection_maxk=10,
            deflection_limit=10,
            deflection_status=False,
            fi_w_i=10,
            fi_w=10,
            # 裂缝
            v_i=10,
            c_s=10,
            p_te_w=10,
            d_eq=10,
            crack_max=10,
            crack_status=True,
            rebar_n=10,
            # 适配原本下层类属性改动
            ksi_b=10,
            as_1_actual=10,
            as_2_actual=10,
            as_3_actual=10,
            d_1_actual=10,
            d_2_actual=10,
            d_3_actual=10,
            spacing_1_actual=1,
            spacing_2_actual=1,
            spacing_3_actual=1,
        )

        data = ModelConstructionData.objects.create()
        result_serializer = exchange.structure.ResultBothWaySerializer(data=kwargs)
        if result_serializer.is_valid():
            result_serializer.save(construction=data)
        else:
            raise result_serializer.errors
        if not os.path.exists("tmp"):
            os.mkdir("tmp")


class TestDetailed(TestCase):
    def test_logic(self):
        """
        完成:
            - 结构参数输入
            - 结构计算并存储
            - 深化设计参数录入
            - 深化设计结果执行
        Returns:

        """

        structure_parameters = ModelConstructionData.objects.create(
            rebar_name="HRB400",
            concrete_grade=30,
            protective_layer_thickness=20,
            longitudinal_top_stress_bar_margin=25,
            height=3000,
            thickness=210,
            weight=1280,
            clear_span=4420,
            top_top_length=500,
            bottom_top_length=500,
            steps_number=18,
            live_load=3.5,
            railing_load=0.0,
            permanent_load_partial_factor=1.2,
            live_load_load_partial_factor=1.4,
            quasi_permanent_value_coefficient=0.4,
            combined_value_coefficient=0.7,
            reinforced_concrete_bulk_density=25,
            crack=0.3,
        )
        # model instance
        structure_calculation_result: ModelConstructionResult = (
            call_structural_calculation(structure_parameters)
        )
        self.assertIsInstance(
            structure_calculation_result,
            ModelConstructionResult,
            f"结构计算出现错误:{structure_calculation_result}",
        )

        self.assertTrue(
            structure_calculation_result.success,
            f"结构计算出现错误:{structure_calculation_result.message}",
        )

        # 录入深化设计参数
        detail_design_parameter = DetailData.objects.create(
            stair=structure_parameters,
            width=1280,
            top_to_length=500,
            top_thickness=200,
            top_b=0,
            bottom_top_length=500,
            bottom_thickness=200,
            bottom_b=0,
            hole_design_mode=HoleDesignMode.AUTOMATIC.value,
            joint_design_mode=JointDesignMode.MANUAL.value,
            step_slot_design_mode=StepSlotDesignMode.MANUAL.value,
            water_drip_design_mode=WaterDripDesignMode.MANUAL.value,
            top_hole_type=HoleType.FIXED_HINGE.value,
            top_sliding_hinge_c1=70,
            top_sliding_hinge_d1=55,
            top_sliding_hinge_e1=65,
            top_sliding_hinge_f1=50,
            top_sliding_hinge_h1=50,
            top_fix_hinge_c2=60,
            top_fix_hinge_d2=50,
            top_hole_position_a1=100,
            top_hole_position_a2=100,
            top_hole_position_b1=300,
            top_hole_position_b2=300,
            bottom_hole_type=HoleType.FIXED_HINGE.value,
            bottom_sliding_hinge_c1=70,
            bottom_sliding_hinge_d1=55,
            bottom_sliding_hinge_e1=65,
            bottom_sliding_hinge_f1=50,
            bottom_sliding_hinge_h1=50,
            bottom_fix_hinge_c2=60,
            bottom_fix_hinge_d2=50,
            bottom_hole_position_a1=100,
            bottom_hole_position_a2=100,
            bottom_hole_position_b1=300,
            bottom_hole_position_b2=300,
            top_joint_a=30,
            top_joint_b=50,
            top_joint_c=20,
            bottom_joint_a=30,
            bottom_joint_b=50,
            bottom_joint_c=20,
            step_slot_a=9,
            step_slot_b=6,
            step_slot_c=16,
            step_slot_d=8,
            step_slot_e=6,
            step_slot_position_c1=50,
            step_slot_position_c2=50,
            step_slot_position_c3=21,
            # 滴水槽
            water_drip_layout=WaterDripLayout.BOTH.value,
            water_drip_shape=WaterDripShape.TRAPEZOID.value,
            water_drip_semicircle_a=10,
            water_drip_semicircle_b=10,
            water_drip_trapezoid_a=5,
            water_drip_trapezoid_b=10,
            water_drip_trapezoid_c=15,
            water_drip_position_a1=15,
            water_drip_position_a2=15,
            water_drip_position_a3=20,
            rebar_design_mode=RebarDesignMode.AUTOMATIC.value,
            bottom_edge_longitudinal_rebar_diameter=12,
            bottom_edge_longitudinal_rebar_spacing=210,
            top_edge_longitudinal_rebar_diameter=12,
            top_edge_longitudinal_rebar_spacing=210,
            bottom_edge_stirrup_diameter=8,
            bottom_edge_stirrup_spacing=130,
            top_edge_stirrup_diameter=8,
            top_edge_stirrup_spacing=130,
            hole_reinforce_rebar_diameter=10,
            hoisting_reinforce_rebar_diameter=10,
            top_edge_reinforce_rebar_diameter=10,
            bottom_edge_reinforce_rebar_diameter=10,
            hoist_design_mode=HoistDesignMode.AUTOMATIC.value,
            demold_design_mode=DemoldDesignMode.AUTOMATIC.value,
            rail_design_mode=RailDesignMode.MANUAL.value,
            hoist_type=HoistType.ROUNDING_HEAD.value,
            hoist_position_a=2,
            hoist_position_b=10,
            hoist_position_c=300,
            hoist_position_d=300,
            hoist_name="DJ-25-170",
            pouring_way=PouringWay.VERTICAL_HORIZONTAL.value,
            demold_type=DemoldType.ANCHOR.value,
            demold_position_a=600,
            demold_position_b=600,
            demold_position_c=300,
            demold_position_d=300,
            demold_position_t=20,
            demold_name="DJ-25-170",
            rail_layout=RailLayout.BOTH.value,
            rail_number="2 4 6 8",
            rail_name="M1",
            rail_position_a=75,
            rail_position_b=130,
        )

        design_result_instance = call_detailed_design_by_model(detail_design_parameter)
        _logger.debug(f"深化设计计算结果:{model_to_dict(design_result_instance)}")

        # 生成计算书
        book_io = call_design_book(design_result_instance)
        with open(os.path.join("tmp", "design_book.docx"), "wb") as f:
            f.write(book_io.read())

        # 测试序列化结果
        # pprint(SerializerTotalStairsByStructionData(instance=structure_parameters).data)
        start_time = datetime.now()
        rebar_for_bim = call_rebar_layout(design_result_instance)
        end_time = datetime.now()
        _logger.info(f"钢筋排布时间:{end_time - start_time}")
        # 存放钢筋数据
        rebar_result = RebarLayoutModel.objects.create(
            stair=structure_parameters,
            content=asdict(
                rebar_for_bim,
            ),
        )
        exchanged = tools.BeforeFinalCall(rebar_result)
        ifc_call_result = call_ifc_create(exchanged, rebar_result)
        file_exports = db_models.FileExport(
            stair=structure_parameters,
        )
        file_exports.ifc.save(name=f"test.ifc", content=ContentFile(ifc_call_result))
        file_exports.save()
        _logger.info(file_exports)
        # # bvbs 生成
        # rebar_for_BVBS = data_for_bvbs(structure_design, data_back_structure,
        #                                detailed_parameter,
        #                                data_back_detailed)

    @skip("issue 37 调试开关:关闭")
    def test_issue_37_sliding_and_fix_hinge(self):
        """
        该测试用例仅用于调试ISSUE 期间
        用于调试各个序列化过程,期间的git commit 变动无真实意义,在后期将删除跳过该测试
        Returns:

        """
        structure_parameters = ModelConstructionData.objects.create(
            rebar_name="HRB400",
            concrete_grade=30,
            protective_layer_thickness=20,
            longitudinal_top_stress_bar_margin=25,
            height=3000,
            thickness=210,
            weight=1280,
            clear_span=4420,
            top_top_length=500,
            bottom_top_length=500,
            steps_number=18,
            live_load=3.5,
            railing_load=0.0,
            permanent_load_partial_factor=1.2,
            live_load_load_partial_factor=1.4,
            quasi_permanent_value_coefficient=0.4,
            combined_value_coefficient=0.7,
            reinforced_concrete_bulk_density=25,
            crack=0.3,
        )
        # model instance
        structure_calculation_result: ModelConstructionResult = (
            call_structural_calculation(structure_parameters)
        )
        self.assertIsInstance(
            structure_calculation_result,
            ModelConstructionResult,
            f"结构计算出现错误:{structure_calculation_result}",
        )

        self.assertTrue(
            structure_calculation_result.success,
            f"结构计算出现错误:{structure_calculation_result.message}",
        )

        # 录入深化设计参数
        detail_design_parameter = DetailData.objects.create(
            stair=structure_parameters,
            width=1280,
            top_to_length=500,
            top_thickness=200,
            top_b=0,
            bottom_top_length=500,
            bottom_thickness=200,
            bottom_b=0,
            hole_design_mode=HoleDesignMode.MANUAL.value,
            joint_design_mode=JointDesignMode.MANUAL.value,
            step_slot_design_mode=StepSlotDesignMode.MANUAL.value,
            water_drip_design_mode=WaterDripDesignMode.MANUAL.value,
            top_hole_type=HoleType.FIXED_HINGE.value,
            top_sliding_hinge_c1=70,
            top_sliding_hinge_d1=55,
            top_sliding_hinge_e1=65,
            top_sliding_hinge_f1=50,
            top_sliding_hinge_h1=50,
            top_fix_hinge_c2=60,
            top_fix_hinge_d2=50,
            top_hole_position_a1=100,
            top_hole_position_a2=100,
            top_hole_position_b1=300,
            top_hole_position_b2=300,
            bottom_hole_type=HoleType.FIXED_HINGE.value,
            bottom_sliding_hinge_c1=70,
            bottom_sliding_hinge_d1=55,
            bottom_sliding_hinge_e1=65,
            bottom_sliding_hinge_f1=50,
            bottom_sliding_hinge_h1=50,
            bottom_fix_hinge_c2=60,
            bottom_fix_hinge_d2=50,
            bottom_hole_position_a1=100,
            bottom_hole_position_a2=100,
            bottom_hole_position_b1=300,
            bottom_hole_position_b2=300,
            top_joint_a=30,
            top_joint_b=50,
            top_joint_c=20,
            bottom_joint_a=30,
            bottom_joint_b=50,
            bottom_joint_c=20,
            step_slot_a=9,
            step_slot_b=6,
            step_slot_c=16,
            step_slot_d=8,
            step_slot_e=6,
            step_slot_position_c1=50,
            step_slot_position_c2=50,
            step_slot_position_c3=21,
            # 滴水槽
            water_drip_layout=WaterDripLayout.BOTH.value,
            water_drip_shape=WaterDripShape.TRAPEZOID.value,
            water_drip_semicircle_a=10,
            water_drip_semicircle_b=10,
            water_drip_trapezoid_a=5,
            water_drip_trapezoid_b=10,
            water_drip_trapezoid_c=15,
            water_drip_position_a1=15,
            water_drip_position_a2=15,
            water_drip_position_a3=20,
            rebar_design_mode=RebarDesignMode.AUTOMATIC.value,
            bottom_edge_longitudinal_rebar_diameter=12,
            bottom_edge_longitudinal_rebar_spacing=210,
            top_edge_longitudinal_rebar_diameter=12,
            top_edge_longitudinal_rebar_spacing=210,
            bottom_edge_stirrup_diameter=8,
            bottom_edge_stirrup_spacing=130,
            top_edge_stirrup_diameter=8,
            top_edge_stirrup_spacing=130,
            hole_reinforce_rebar_diameter=10,
            hoisting_reinforce_rebar_diameter=10,
            top_edge_reinforce_rebar_diameter=10,
            bottom_edge_reinforce_rebar_diameter=10,
            hoist_design_mode=HoistDesignMode.AUTOMATIC.value,
            demold_design_mode=DemoldDesignMode.AUTOMATIC.value,
            rail_design_mode=RailDesignMode.MANUAL.value,
            hoist_type=HoistType.ROUNDING_HEAD.value,
            hoist_position_a=2,
            hoist_position_b=10,
            hoist_position_c=300,
            hoist_position_d=300,
            hoist_name="DJ-25-170",
            pouring_way=PouringWay.VERTICAL_HORIZONTAL.value,
            demold_type=DemoldType.ANCHOR.value,
            demold_position_a=600,
            demold_position_b=600,
            demold_position_c=300,
            demold_position_d=300,
            demold_position_t=20,
            demold_name="DJ-25-170",
            rail_layout=RailLayout.BOTH.value,
            rail_number="2 4 6 8",
            rail_name="M1",
            rail_position_a=75,
            rail_position_b=130,
        )

        design_result_instance = call_detailed_design_by_model(detail_design_parameter)


class TestCelery(TestCase):
    @skip(f"测试暂时跳过对队列的调用,本地开发环境中rabbit mq 服务异常")
    def test_task_call(self):
        test.apply_async(())


class TestModelToConstruction(TestCase):
    def test_cls_function_model_orm_to_structure_init_and_call(self):
        """
        测试功能函数 是否能够完成model 到
        Returns:

        """
        construction = ModelConstructionData()
        exchange.structure.StructureDataSerializer(instance=construction).data


class TestSerializer(TestCase):
    def test_result_both_way_serializer(self):
        from design.exchange.structure import (
            ResultBothWaySerializer as ConstructionResultBothWaySerializer,
        )

        from design.models import ModelConstructionResult

        construction_data = ModelConstructionData(
            concrete_grade=30,
        )
        result_instance = ModelConstructionResult(
            as_fact_1=1,
            as_fact_2=1,
            as_fact_3=1,
            d_fact_1=1,
            d_fact_2=1,
            d_fact_3=1,
            spacing_fact_1=1,
            spacing_fact_2=1,
            spacing_fact_3=1,
            # 模拟外键限制
            construction=construction_data,
        )

        _logger.info(result_instance)
        serializer = ConstructionResultBothWaySerializer(result_instance)
        _logger.info(serializer.data)
        serializer_for_save = ConstructionResultBothWaySerializer(data=serializer.data)
        if not serializer_for_save.is_valid():
            raise Exception(serializer_for_save.error_messages)
        else:
            result_instance = serializer_for_save.save()
            _logger.info(model_to_dict(result_instance))


class ModelOrmBaseTest(TestCase):
    def test_design_result_copy_change_write_table(self):
        cc_w = DetailDataCopyChangeWrite.objects.create(content={"1": 1})
