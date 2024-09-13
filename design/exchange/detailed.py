import logging
import traceback
from collections import OrderedDict
from dataclasses import asdict, dataclass
from typing import Optional, Dict, Callable

from rest_framework import serializers

from stair_detailed.models import (
    HoleType,
    WaterDripShape,
    RebarDesignMode,
    HoleDesignMode,
    StepSlotDesignMode,
    JointDesignMode,
    DemoldingDesignMode,
    RailDesignMode,
    PouringWay,
    DetailedDesignResult,
    LiftingType,
    DemoldingType,
    WaterDripDesignMode,
)

from design import exchange
from design.exchange.tools import custom_asdict_contain_enum
from design.models import (
    ModelConstructionResult,
    DetailData,
    ModelDetailedResult,
    DetailDataCopyChangeWrite,
)

# import traceback

_logger = logging.getLogger(__name__)

ConstructionResultSerializer = exchange.structure.ResultBothWaySerializer


# 深化设计相关
# stair_detailed 中数据的序列化转换
class _DetailData2TopFixedHingeSerializer(serializers.ModelSerializer):
    """
    处理成为 顶端FixedHinge 对应的key
    """

    fix_hinge_c1 = serializers.IntegerField(source="top_fix_hinge_c1")
    fix_hinge_d1 = serializers.IntegerField(source="top_fix_hinge_d1")
    fix_hinge_e1 = serializers.IntegerField(source="top_fix_hinge_e1")
    fix_hinge_f1 = serializers.IntegerField(source="top_fix_hinge_f1")
    fix_hinge_h1 = serializers.IntegerField(source="top_fix_hinge_h1")

    class Meta:
        model = DetailData
        fields = [
            "fix_hinge_c1",
            "fix_hinge_d1",
            "fix_hinge_e1",
            "fix_hinge_f1",
            "fix_hinge_h1",
        ]


class _DetailData2TopHolePositionSerializer(serializers.ModelSerializer):
    a1 = serializers.IntegerField(source="top_hole_position_a1")
    a2 = serializers.IntegerField(source="top_hole_position_a2")
    b1 = serializers.IntegerField(source="top_hole_position_b1")
    b2 = serializers.IntegerField(source="top_hole_position_b2")

    class Meta:
        model = DetailData
        fields = [
            "a1",
            "a2",
            "b1",
            "b2",
        ]


class _BottomFixedHingeSerializerV2(serializers.ModelSerializer):
    """
    底部固定铰支座，对应下层反转后
    """

    fix_hinge_c2 = serializers.IntegerField(source="bottom_fix_hinge_c2")
    fix_hinge_d2 = serializers.IntegerField(source="bottom_fix_hinge_d2")

    class Meta:
        model = DetailData
        fields = [
            "fix_hinge_c2",
            "fix_hinge_d2",
        ]


class _TopFixedHingeSerializer(serializers.ModelSerializer):
    """
    顶端固定铰支座,对应参数变动后的
    """

    fix_hinge_c2 = serializers.IntegerField(source="top_fix_hinge_c2")
    fix_hinge_d2 = serializers.IntegerField(source="top_fix_hinge_d2")

    class Meta:
        model = DetailData
        fields = [
            "fix_hinge_c2",
            "fix_hinge_d2",
        ]


_SERIALIZER_BOTTOM_FIXED_HINGE = _BottomFixedHingeSerializerV2
_SERIALIZER_TOP_FIXED_HINGE = _TopFixedHingeSerializer


class _BottomSlidingHingeSerializerV2(serializers.ModelSerializer):
    """
    底部滑动铰支座序列化,反转滑动/固定 参数后的序列化
    """

    sliding_hinge_c1 = serializers.IntegerField(source="bottom_sliding_hinge_c1")
    sliding_hinge_d1 = serializers.IntegerField(source="bottom_sliding_hinge_d1")
    sliding_hinge_e1 = serializers.IntegerField(source="bottom_sliding_hinge_e1")
    sliding_hinge_f1 = serializers.IntegerField(source="bottom_sliding_hinge_f1")
    sliding_hinge_h1 = serializers.IntegerField(source="bottom_sliding_hinge_h1")

    class Meta:
        model = DetailData
        fields = [
            "sliding_hinge_c1",
            "sliding_hinge_d1",
            "sliding_hinge_e1",
            "sliding_hinge_f1",
            "sliding_hinge_h1",
        ]


class _TopSlidingHingSerializerV1(serializers.ModelSerializer):
    """
    顶部活动铰支座参数
    """

    sliding_hinge_c1 = serializers.IntegerField(source="top_sliding_hinge_c1")
    sliding_hinge_d1 = serializers.IntegerField(source="top_sliding_hinge_d1")
    sliding_hinge_e1 = serializers.IntegerField(source="top_sliding_hinge_e1")
    sliding_hinge_f1 = serializers.IntegerField(source="top_sliding_hinge_f1")
    sliding_hinge_h1 = serializers.IntegerField(source="top_sliding_hinge_h1")

    class Meta:
        model = DetailData
        fields = [
            "sliding_hinge_c1",
            "sliding_hinge_d1",
            "sliding_hinge_e1",
            "sliding_hinge_f1",
            "sliding_hinge_h1",
        ]


_SERIALIZER_BOTTOM_SLIDING_HINGE = _BottomSlidingHingeSerializerV2
_SERIALIZER_TOP_SLIDING_HINGE = _TopSlidingHingSerializerV1


class _DetailData2BottomHolePositionSerializer(serializers.ModelSerializer):
    a3 = serializers.IntegerField(source="bottom_hole_position_a1")
    a4 = serializers.IntegerField(source="bottom_hole_position_a2")
    b3 = serializers.IntegerField(source="bottom_hole_position_b1")
    b4 = serializers.IntegerField(source="bottom_hole_position_b2")

    class Meta:
        model = DetailData
        fields = [
            "a3",
            "a4",
            "b3",
            "b4",
        ]


class _DetailData2TopJointShapeSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="top_joint_a")
    b = serializers.IntegerField(source="top_joint_b")
    c = serializers.IntegerField(source="top_joint_c")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
            "c",
        ]


class _DetailData2BottomJointShapeSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="bottom_joint_a")
    b = serializers.IntegerField(source="bottom_joint_b")
    c = serializers.IntegerField(source="bottom_joint_c")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
            "c",
        ]


class _DetailData2StepSlotSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="step_slot_a")
    b = serializers.IntegerField(source="step_slot_b")
    c = serializers.IntegerField(source="step_slot_c")
    d = serializers.IntegerField(source="step_slot_d")
    e = serializers.IntegerField(source="step_slot_e")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
            "c",
            "d",
            "e",
        ]


class _DetailData2StepSlotPositionSerializer(serializers.ModelSerializer):
    c1 = serializers.IntegerField(source="step_slot_position_c1")
    c2 = serializers.IntegerField(source="step_slot_position_c2")
    c3 = serializers.IntegerField(source="step_slot_position_c3")

    class Meta:
        model = DetailData
        fields = ["c1", "c2", "c3"]


class _DetailData2WaterDripSemicircleSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="water_drip_semicircle_a")
    b = serializers.IntegerField(source="water_drip_semicircle_b")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
        ]


class _DetailData2WaterDripTrapezoidSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="water_drip_trapezoid_a")
    b = serializers.IntegerField(source="water_drip_trapezoid_b")
    c = serializers.IntegerField(source="water_drip_trapezoid_c")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
            "c",
        ]


class _DetailData2WaterDripPositionSerializer(serializers.ModelSerializer):
    a1 = serializers.FloatField(source="water_drip_position_a1")
    a2 = serializers.FloatField(source="water_drip_position_a2")
    a3 = serializers.FloatField(source="water_drip_position_a3")

    class Meta:
        model = DetailData
        fields = [
            "a1",
            "a2",
            "a3",
        ]


class _DetailData2StructureDetailedSerializer(serializers.ModelSerializer):
    """
    将 orm 转换成结构详情
    """

    top_hole = serializers.SerializerMethodField()

    def get_top_hole(self, obj: DetailData):
        """
        为适应下层变动,增加top hole，与bottom hole 同级
        Args:
            obj:

        Returns:

        """
        if obj.hole_design_mode == HoleDesignMode.MANUAL.value:
            if obj.top_hole_type == HoleType.FIXED_HINGE.value:
                return _SERIALIZER_TOP_FIXED_HINGE(obj).data
            else:
                return _SERIALIZER_TOP_SLIDING_HINGE(obj).data

    top_hole_position = serializers.SerializerMethodField()
    bottom_hole = serializers.SerializerMethodField()
    bottom_hole_position = serializers.SerializerMethodField()

    top_joint = serializers.SerializerMethodField()
    bottom_joint = serializers.SerializerMethodField()
    step_slot = serializers.SerializerMethodField()
    step_slot_position = serializers.SerializerMethodField()

    water_drip = serializers.SerializerMethodField()

    water_drip_position = serializers.SerializerMethodField()

    def get_water_drip_position(self, obj: DetailData):
        return dict(_DetailData2WaterDripPositionSerializer(obj).data)

    def get_water_drip(self, obj: DetailData):
        """
        根据滴水槽设计模式和侧边形状选择,返回拦截后的截面尺寸数据
        Args:
            obj:

        Returns:

        """
        water_design_mode = WaterDripDesignMode(obj.water_drip_design_mode)
        if water_design_mode == WaterDripDesignMode.MANUAL:
            serializer_choose = [
                _DetailData2WaterDripTrapezoidSerializer,
                _DetailData2WaterDripSemicircleSerializer,
            ][obj.water_drip_shape == WaterDripShape.SEMICIRCLE]
            return dict(serializer_choose(obj).data)
        else:
            return None

    def get_step_slot_position(self, obj):
        return _DetailData2StepSlotPositionSerializer(obj).data

    def get_top_hole_position(self, obj: DetailData) -> Optional[Dict]:
        if obj.hole_design_mode == HoleDesignMode.MANUAL.value:
            return _DetailData2TopHolePositionSerializer(obj).data

    def get_bottom_hole(self, obj: DetailData) -> Dict:
        if obj.hole_design_mode == HoleDesignMode.MANUAL.value:
            if obj.bottom_hole_type == HoleType.FIXED_HINGE.value:
                return _SERIALIZER_BOTTOM_FIXED_HINGE(obj).data
            else:
                return _SERIALIZER_BOTTOM_SLIDING_HINGE(obj).data

    def get_bottom_hole_position(self, obj: DetailData) -> Optional[Dict]:
        if obj.hole_design_mode == HoleDesignMode.MANUAL.value:
            return _DetailData2BottomHolePositionSerializer(obj).data

    def get_top_joint(self, obj: DetailData):
        if obj.joint_design_mode == JointDesignMode.MANUAL.value:
            return _DetailData2TopJointShapeSerializer(obj).data

    def get_bottom_joint(self, obj):
        if obj.joint_design_mode == JointDesignMode.MANUAL.value:
            return _DetailData2BottomJointShapeSerializer(obj).data

    def get_step_slot(self, obj):
        if obj.step_slot_design_mode == StepSlotDesignMode.MANUAL.value:
            return _DetailData2StepSlotSerializer(obj).data

    water_drip_layout = serializers.SerializerMethodField()

    def get_water_drip_layout(self, obj):
        if (
            WaterDripDesignMode(obj.water_drip_design_mode)
            == WaterDripDesignMode.MANUAL
        ):
            return obj.water_drip_layout
        else:
            return None

    class Meta:
        fields = [
            "hole_design_mode",
            "joint_design_mode",
            "step_slot_design_mode",
            "water_drip_design_mode",
            "top_hole_type",
            "top_hole",
            "top_hole_position",
            "bottom_hole_type",
            "bottom_hole",
            "bottom_hole_position",
            "top_joint",
            "bottom_joint",
            "step_slot",
            "step_slot_position",
            "water_drip_layout",
            "water_drip_shape",
            "water_drip",
            "water_drip_position",
        ]
        model = DetailData


class _DetailData2GeometricDetailedSerializer(serializers.ModelSerializer):
    # 起初表结构设计命名错误,此处需转换
    top_top_length = serializers.IntegerField(source="top_to_length")

    class Meta:
        model = DetailData
        fields = [
            "width",
            "top_top_length",
            "top_thickness",
            "top_b",
            "bottom_top_length",
            "bottom_thickness",
            "bottom_b",
        ]


class _DetailData2RebarDiamSpaceSerializer(serializers.ModelSerializer):
    def __init__(self, head, *args, **kwargs):
        self.head = head
        super(_DetailData2RebarDiamSpaceSerializer, self).__init__(*args, **kwargs)

    class Meta:
        model = DetailData
        fields = [
            "diameter",
            "spacing",
        ]

    def get_fields(self):
        fields = OrderedDict()
        fields["diameter"] = serializers.IntegerField(source=f"{self.head}_diameter")
        fields["spacing"] = serializers.IntegerField(source=f"{self.head}_spacing")
        return fields


class _DetailData2RebarDiamSerializer(serializers.ModelSerializer):
    head: str

    def __init__(self, head, *args, **kwargs):
        self.head = head
        super(_DetailData2RebarDiamSerializer, self).__init__(*args, **kwargs)

    class Meta:
        model = DetailData
        fields = [
            "diameter",
        ]

    def get_fields(self):
        fields = OrderedDict()
        fields["diameter"] = serializers.IntegerField(source=f"{self.head}_diameter")
        return fields


class _DetailData2RebarDetailedSerializer(serializers.ModelSerializer):
    bottom_edge_longitudinal_rebar = serializers.SerializerMethodField()
    top_edge_longitudinal_rebar = serializers.SerializerMethodField()
    bottom_edge_stirrup = serializers.SerializerMethodField()
    top_edge_stirrup = serializers.SerializerMethodField()
    hole_reinforce_rebar = serializers.SerializerMethodField()
    lifting_reinforce_rebar = serializers.SerializerMethodField()
    top_edge_reinforce_rebar = serializers.SerializerMethodField()
    bottom_edge_reinforce_rebar = serializers.SerializerMethodField()

    def get_bottom_edge_longitudinal_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSpaceSerializer(
                head="bottom_edge_longitudinal_rebar", instance=obj
            ).data

    def get_top_edge_longitudinal_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSpaceSerializer(
                head="top_edge_longitudinal_rebar", instance=obj
            ).data

    def get_bottom_edge_stirrup(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSpaceSerializer(
                head="bottom_edge_stirrup", instance=obj
            ).data

    def get_top_edge_stirrup(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSpaceSerializer(
                head="top_edge_stirrup", instance=obj
            ).data

    def get_hole_reinforce_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSerializer(
                head="top_edge_stirrup", instance=obj
            ).data

    def get_lifting_reinforce_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSerializer(
                head="hoisting_reinforce_rebar", instance=obj
            ).data

    def get_top_edge_reinforce_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSerializer(
                head="top_edge_reinforce_rebar", instance=obj
            ).data

    def get_bottom_edge_reinforce_rebar(self, obj):
        if obj.rebar_design_mode == RebarDesignMode.MANUAL.value:
            return _DetailData2RebarDiamSerializer(
                head="bottom_edge_reinforce_rebar", instance=obj
            ).data

    class Meta:
        model = DetailData
        fields = [
            "rebar_design_mode",
            "bottom_edge_longitudinal_rebar",
            "top_edge_longitudinal_rebar",
            "bottom_edge_stirrup",
            "top_edge_stirrup",
            "hole_reinforce_rebar",
            "lifting_reinforce_rebar",
            "top_edge_reinforce_rebar",
            "bottom_edge_reinforce_rebar",
        ]


class _DetailData2HoistPositionSerializer(serializers.ModelSerializer):
    a = serializers.IntegerField(source="hoist_position_a")
    b = serializers.IntegerField(source="hoist_position_b")
    c = serializers.FloatField(source="hoist_position_c")
    d = serializers.FloatField(source="hoist_position_d")

    class Meta:
        model = DetailData
        fields = ["a", "b", "c", "d"]


class _DetailData2DemoldPositionSerializer(serializers.ModelSerializer):
    a = serializers.FloatField(source="demold_position_a")
    b = serializers.FloatField(source="demold_position_b")
    c = serializers.FloatField(source="demold_position_c")
    d = serializers.FloatField(source="demold_position_d")
    t = serializers.FloatField(source="demold_position_t")

    class Meta:
        model = DetailData
        fields = [
            "a",
            "b",
            "c",
            "d",
            "t",
        ]


class _DetailData2RailPositionSerializer(serializers.ModelSerializer):
    a = serializers.FloatField(source="rail_position_a")
    b = serializers.FloatField(source="rail_position_b")

    class Meta:
        model = DetailData
        fields = ["a", "b"]


class _DetailData2InsertsDetailedSerializer(serializers.ModelSerializer):
    lifting_position = serializers.SerializerMethodField()
    demolding_position = serializers.SerializerMethodField()
    rail_number = serializers.SerializerMethodField()
    rail_position = serializers.SerializerMethodField()
    pouring_way = serializers.SerializerMethodField()

    lifting_design_mode = serializers.IntegerField(source="hoist_design_mode")
    demolding_design_mode = serializers.IntegerField(source="demold_design_mode")

    lifting_type = serializers.IntegerField(source="hoist_type")

    lifting_name = serializers.CharField(source="hoist_name")
    demolding_type = serializers.IntegerField(source="demold_type")

    def get_pouring_way(self, obj: DetailData):
        if obj.demold_design_mode == DemoldingDesignMode.MANUAL.value:
            return obj.pouring_way
        return PouringWay.VERTICAL_HORIZONTAL.value

    def get_rail_position(self, obj: DetailData):
        if obj.rail_design_mode == RailDesignMode.MANUAL.value:
            return _DetailData2RailPositionSerializer(obj).data

    def get_rail_number(self, obj: DetailData):
        try:
            results = list(map(int, obj.rail_number.split(" ")))
        except Exception as e:
            _logger.error(f"{e},data:{obj.rail_number}")
            results = []

        return results

    def get_demolding_position(self, obj: DetailData):
        if obj.demold_design_mode == DemoldingDesignMode.MANUAL.value:
            return _DetailData2DemoldPositionSerializer(obj).data

    def get_lifting_position(self, obj: DetailData):
        if obj.hole_design_mode == HoleDesignMode.MANUAL.value:
            return _DetailData2HoistPositionSerializer(obj).data

    class Meta:
        model = DetailData
        fields = [
            "lifting_design_mode",
            "demolding_design_mode",
            "rail_design_mode",
            "lifting_type",
            "lifting_position",
            "lifting_name",
            "pouring_way",
            "demolding_type",
            "demolding_position",
            "rail_layout",
            "rail_number",
            "rail_position",
            "rail_name",
        ]


class DetailData2ConstructionDetailedSerializer(serializers.ModelSerializer):
    """
    深化设计结果 orm 数据 往dataclass 数据的转换
    """

    construction_detailed = serializers.SerializerMethodField()
    geometric_detailed = serializers.SerializerMethodField()
    rebar_detailed = serializers.SerializerMethodField()
    inserts_detailed = serializers.SerializerMethodField()

    def get_inserts_detailed(self, obj):
        return _DetailData2InsertsDetailedSerializer(instance=obj).data

    def get_geometric_detailed(self, obj):
        return _DetailData2GeometricDetailedSerializer(obj).data

    def get_construction_detailed(self, obj):
        return _DetailData2StructureDetailedSerializer(obj).data

    def get_rebar_detailed(self, obj):
        return _DetailData2RebarDetailedSerializer(obj).data

    class Meta:
        model = DetailData
        fields = [
            "construction_detailed",
            "geometric_detailed",
            "rebar_detailed",
            "inserts_detailed",
        ]


def _dataclass_2_dict_with_head(
    data: dataclass,
    head: Optional[str] = None,
    dict_factory: Callable = custom_asdict_contain_enum,
) -> Dict:
    """
    将dataclass 以一个固定的开头，转换成字典类型
    Args:
        data:
        head:
        dict_factory:

    Returns:

    """
    data_back = {}
    try:
        for key, value in asdict(data, dict_factory=dict_factory).items():
            new_key = key if head is None else f"{head}_{key}"
            data_back[new_key] = value
    except TypeError as e:
        if data is not None:
            _logger.warning(f"{data},{head},{dict_factory},{e}")
            traceback.print_exc()

    return data_back


def detail_result_2_model_result(
    result: DetailedDesignResult, detailed_data: DetailData
) -> ModelDetailedResult:
    """
    完成dataclass 嵌套格式的数据,到数据库表orm 的转换[手动] 默认不保存

    Args:
        detailed_data:
        result:

    Returns:

    """

    if LiftingType(result.lifting_type) == LiftingType.ANCHOR:
        hoist_parameter = _dataclass_2_dict_with_head(
            result.lifting_parameter, "hoist_parameter_anchor_parameter"
        )

    elif LiftingType(result.lifting_type) == LiftingType.ROUNDING_HEAD:
        hoist_parameter = _dataclass_2_dict_with_head(
            result.lifting_parameter, "hoist_parameter_round_head_parameter"
        )
    else:
        raise Exception(f"类型检验失败:{result.lifting_type=}")

    if DemoldingType(result.demolding_type) == DemoldingType.ROUNDING_HEAD:
        demold_parameter = _dataclass_2_dict_with_head(
            result.demolding_parameter, "demold_parameter_round_head_parameter"
        )
    elif DemoldingType(result.demolding_type) == DemoldingType.ANCHOR:
        demold_parameter = _dataclass_2_dict_with_head(
            result.demolding_parameter, "demold_parameter_anchor_parameter"
        )
    else:
        raise Exception(f"类型校验失败:{result.demolding_type=}")
    # 栏杆预埋件参数可能为None

    rail_parameter = _dataclass_2_dict_with_head(
        result.rail_parameter, "rail_parameter"
    )
    content = asdict(
        result.detailed_design, dict_factory=exchange.tools.custom_asdict_contain_enum
    )
    ccw_serializer = SerializerBtnDesignResultCopyWrite(data={"content": content})
    if ccw_serializer.is_valid():
        ccw_instance: DetailDataCopyChangeWrite = ccw_serializer.save()
    else:
        raise ccw_serializer.errors

    result_orm = ModelDetailedResult.objects.update_or_create(
        defaults=dict(
            top_bottom_length=result.top_bottom_length,
            bottom_bottom_length=result.bottom_bottom_length,
            l0=result.l_total,
            h0=result.h_total,
            v=result.v,
            cos=result.cos,
            sin=result.sin,
            tan=result.tan,
            gkt=result.gkt,
            gk=result.gk,
            gdk=result.gdk,
            gek=result.gek,
            single_capacity_hosit=result.single_capacity_lifting,
            hoist_type=result.lifting_type.value,
            hoist_name=result.lifting_name,
            hoist_edge_xa=result.lifting_edge_xa,
            hoist_edge_xb=result.lifting_edge_xb,
            hoist_edge_xc=result.lifting_edge_xc,
            m_hoist_ka=result.m_lifting_ka,
            m_hoist_kb=result.m_lifting_kb,
            w_hoist_a=result.w_lifting_a,
            w_hoist_b=result.w_lifting_b,
            sigm_hoist_cka=result.sigm_lifting_cka,
            sigm_hoist_ckb=result.sigm_lifting_ckb,
            max_sigm=result.max_sigm,
            hoist_f_tk=result.lifting_f_tk,
            pouring_way=result.pouring_way.value,
            q_k1=result.q_k1,
            single_capacity_demold=result.single_capacity_demolding,
            demold_type=result.demolding_type.value,
            demold_name=result.demolding_name,
            detailed_design=ccw_instance,
            **hoist_parameter,
            **demold_parameter,
            **rail_parameter,
        ),
        stair=detailed_data.stair,
    )[0]

    return result_orm


class _SerializerFromModelDetailedResultToAnchorParameter(serializers.ModelSerializer):
    def __init__(self, start, *args, **kwargs):
        self.start = start
        super(_SerializerFromModelDetailedResultToAnchorParameter, self).__init__(
            *args, **kwargs
        )

    def get_fields(self):
        fields = OrderedDict()
        fields["type"] = serializers.CharField(source=f"{self.start}_type")
        fields["factory"] = serializers.CharField(source=f"{self.start}_factory")
        fields["name"] = serializers.CharField(source=f"{self.start}_name")
        fields["abbreviation"] = serializers.CharField(
            source=f"{self.start}_abbreviation"
        )
        fields["capacity"] = serializers.FloatField(source=f"{self.start}_capacity")
        fields["m_diameter"] = serializers.FloatField(source=f"{self.start}_m_diameter")
        fields["m_length"] = serializers.FloatField(source=f"{self.start}_m_length")
        fields["anchor_name"] = serializers.CharField(
            source=f"{self.start}_anchor_name"
        )
        fields["e_diameter"] = serializers.FloatField(source=f"{self.start}_e_diameter")
        fields["g"] = serializers.FloatField(source=f"{self.start}_g")
        fields["b"] = serializers.FloatField(source=f"{self.start}_b")
        fields["o_diameter"] = serializers.FloatField(source=f"{self.start}_o_diameter")
        fields["length"] = serializers.FloatField(source=f"{self.start}_length")
        fields["s_diameter"] = serializers.FloatField(source=f"{self.start}_s_diameter")
        fields["l_p"] = serializers.FloatField(source=f"{self.start}_l_p")
        fields["a"] = serializers.FloatField(source=f"{self.start}_a")

        return fields

    class Meta:
        model = ModelDetailedResult


class _SerializerFormModelDetailedResultToRoundHeadParameter(
    serializers.ModelSerializer
):
    def __init__(self, start, *args, **kwargs):
        self.start = start
        super(_SerializerFormModelDetailedResultToRoundHeadParameter, self).__init__(
            *args, **kwargs
        )

    def get_fields(self):
        fields = OrderedDict()
        fields["type"] = serializers.CharField(source=f"{self.start}_type")
        fields["factory"] = serializers.CharField(source=f"{self.start}_factory")
        fields["name"] = serializers.CharField(source=f"{self.start}_name")
        fields["abbreviation"] = serializers.CharField(
            source=f"{self.start}_abbreviation"
        )
        fields["capacity"] = serializers.FloatField(source=f"{self.start}_capacity")
        fields["length"] = serializers.FloatField(source=f"{self.start}_length")
        fields["top_diameter"] = serializers.FloatField(
            source=f"{self.start}_top_diameter"
        )
        fields["top_height"] = serializers.FloatField(source=f"{self.start}_top_height")
        fields["top_adjacent_height"] = serializers.FloatField(
            source=f"{self.start}_top_adjacent_height"
        )
        fields["middle_diameter"] = serializers.FloatField(
            source=f"{self.start}_middle_diameter"
        )
        fields["middle_height"] = serializers.FloatField(
            source=f"{self.start}_middle_height"
        )
        fields["bottom_adjacent_height"] = serializers.FloatField(
            source=f"{self.start}_bottom_adjacent_height"
        )
        fields["bottom_diameter"] = serializers.FloatField(
            source=f"{self.start}_bottom_diameter"
        )
        fields["bottom_height"] = serializers.FloatField(
            source=f"{self.start}_bottom_height"
        )
        fields["radius"] = serializers.FloatField(source=f"{self.start}_radius")

        return fields

    class Meta:
        model = ModelDetailedResult


class _SerializerFormModelDetailedResultToRailParameter(serializers.ModelSerializer):
    name = serializers.CharField(source="rail_parameter_name")
    a = serializers.IntegerField(source="rail_parameter_a")
    b = serializers.IntegerField(source="rail_parameter_b")
    c = serializers.IntegerField(source="rail_parameter_c")
    d = serializers.IntegerField(source="rail_parameter_d")
    t = serializers.IntegerField(source="rail_parameter_t")
    fi = serializers.IntegerField(source="rail_parameter_fi")
    depth = serializers.IntegerField(source="rail_parameter_depth")
    length = serializers.IntegerField(source="rail_parameter_length")

    class Meta:
        model = ModelDetailedResult
        fields = [
            "name",
            "a",
            "b",
            "c",
            "d",
            "t",
            "fi",
            "depth",
            "length",
        ]


class MDResultToDCResult(serializers.ModelSerializer):
    """
    单向,仅用于导出
    """

    lifting_parameter = serializers.SerializerMethodField()

    def get_lifting_parameter(self, obj: ModelDetailedResult):
        if LiftingType(obj.hoist_type) == LiftingType.ANCHOR:
            return dict(
                _SerializerFromModelDetailedResultToAnchorParameter(
                    instance=obj, start="hoist_parameter_anchor_parameter"
                ).data
            )
        elif LiftingType(obj.hoist_type) == LiftingType.ROUNDING_HEAD:
            return dict(
                _SerializerFormModelDetailedResultToRoundHeadParameter(
                    instance=obj, start="hoist_parameter_round_head_parameter"
                ).data
            )
        else:
            raise Exception(f"hoist_type 类型异常:{obj.hoist_type}")

    demolding_parameter = serializers.SerializerMethodField()

    def get_demolding_parameter(self, obj: ModelDetailedResult):
        if DemoldingType(obj.demold_type) == DemoldingType.ANCHOR:
            return dict(
                _SerializerFromModelDetailedResultToAnchorParameter(
                    instance=obj, start="demold_parameter_anchor_parameter"
                ).data
            )
        elif DemoldingType(obj.demold_type) == DemoldingType.ROUNDING_HEAD:
            return dict(
                _SerializerFormModelDetailedResultToRoundHeadParameter(
                    instance=obj, start="demold_parameter_round_head_parameter"
                ).data
            )
        else:
            raise Exception(f"hoist_type 类型异常:{obj.hoist_type}")

    rail_parameter = serializers.SerializerMethodField()

    def get_rail_parameter(self, obj: ModelDetailedResult):
        return dict(
            _SerializerFormModelDetailedResultToRailParameter(instance=obj).data
        )

    l_total = serializers.FloatField(source="l0")

    h_total = serializers.FloatField(source="h0")
    single_capacity_lifting = serializers.FloatField(source="single_capacity_hosit")
    lifting_type = serializers.IntegerField(source="hoist_type")
    lifting_name = serializers.CharField(source="hoist_name")
    lifting_edge_xa = serializers.FloatField(source="hoist_edge_xa")
    lifting_edge_xb = serializers.FloatField(source="hoist_edge_xb")
    lifting_edge_xc = serializers.FloatField(source="hoist_edge_xc")
    m_lifting_ka = serializers.FloatField(source="m_hoist_ka")
    m_lifting_kb = serializers.FloatField(source="m_hoist_kb")
    w_lifting_a = serializers.FloatField(source="w_hoist_a")
    w_lifting_b = serializers.FloatField(source="w_hoist_b")
    sigm_lifting_cka = serializers.FloatField(source="sigm_hoist_cka")
    sigm_lifting_ckb = serializers.FloatField(source="sigm_hoist_ckb")
    lifting_f_tk = serializers.FloatField(source="hoist_f_tk")
    single_capacity_demolding = serializers.FloatField(source="single_capacity_demold")
    demolding_type = serializers.IntegerField(source="demold_type")
    demolding_name = serializers.CharField(source="demold_name")

    # FIXME(lanhao945): 对相关参数的拦截,如果已经存在了副本,则拦截未能有效
    detailed_design = serializers.JSONField(source="detailed_design.content")

    class Meta:
        model = ModelDetailedResult
        fields = [
            "top_bottom_length",
            "bottom_bottom_length",
            "l_total",
            "h_total",
            "v",
            "cos",
            "sin",
            "tan",
            "gkt",
            "gk",
            "gdk",
            "gek",
            "single_capacity_lifting",
            "lifting_type",
            "lifting_name",
            "lifting_parameter",
            "lifting_edge_xa",
            "lifting_edge_xb",
            "lifting_edge_xc",
            "m_lifting_ka",
            "m_lifting_kb",
            "w_lifting_a",
            "w_lifting_b",
            "sigm_lifting_cka",
            "sigm_lifting_ckb",
            "max_sigm",
            "lifting_f_tk",
            "pouring_way",
            "q_k1",
            "single_capacity_demolding",
            "demolding_type",
            "demolding_name",
            "demolding_parameter",
            "rail_parameter",
            "detailed_design",
        ]


class SerializerFromModelDetailedResultToCalculationBookDetailed(
    serializers.ModelSerializer
):
    """
    为调用计算书配置序列化
    仅用于内部调用 深化设计.计算书生成
    """

    structural_design_result = serializers.SerializerMethodField()

    def get_structural_design_result(self, obj: ModelDetailedResult):
        model_result: ModelConstructionResult = ModelConstructionResult.objects.get(
            construction=obj.stair
        )
        return dict(ConstructionResultSerializer(instance=model_result).data)

    structural_design = serializers.SerializerMethodField()

    def get_structural_design(self, obj: ModelDetailedResult):
        data = dict(exchange.structure.StructureDataSerializer(obj.stair).data)
        return data

    detailed_design_result = serializers.SerializerMethodField()

    def get_detailed_design_result(self, obj: ModelDetailedResult):
        return dict(MDResultToDCResult(instance=obj).data)

    detailed_design = serializers.SerializerMethodField()

    #
    def get_detailed_design(self, obj: ModelDetailedResult):
        detail_data: DetailData = DetailData.objects.get(stair=obj.stair)
        return dict(
            DetailData2ConstructionDetailedSerializer(instance=detail_data).data
        )

    class Meta:
        model = ModelDetailedResult
        fields = [
            "structural_design_result",
            "structural_design",
            "detailed_design_result",
            "detailed_design",
        ]


class SerializerBtnDesignResultCopyWrite(serializers.ModelSerializer):
    """
    完成复制出来的参数的插入和导出——双向
    """

    class Meta:
        model = DetailDataCopyChangeWrite
        fields = ["content"]
