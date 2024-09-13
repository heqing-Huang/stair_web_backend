"""

结构计算相关

"""
import logging

from rest_framework import serializers

from design.models import ModelConstructionData, ModelConstructionResult

_logger = logging.getLogger(__name__)


class _MaterialSerializer(serializers.ModelSerializer):
    """
    用于从 django-orm 转换至下层结构计算的 dataclass对应的key-value参数

    """

    class Meta:
        model = ModelConstructionData
        fields = [
            "rebar_name",
            "concrete_grade",
        ]
        read_only_fields = fields


class _StructureSerializer(serializers.ModelSerializer):
    """
    构造相关词参数序列导出,用于顶层构件结构深化设计时的组合数据
    """

    concrete_cover_thickness = serializers.IntegerField(
        source="protective_layer_thickness"
    )
    longitudinal_top_rebar_distance = serializers.IntegerField(
        source="longitudinal_top_stress_bar_margin"
    )

    class Meta:
        model = ModelConstructionData
        fields = [
            "concrete_cover_thickness",
            "longitudinal_top_rebar_distance",
        ]
        read_only_fields = fields


class _GeometricSerializer(serializers.ModelSerializer):
    width = serializers.IntegerField(source="weight")

    class Meta:
        model = ModelConstructionData
        fields = [
            "height",
            "thickness",
            "width",
            "clear_span",
            "top_top_length",
            "bottom_top_length",
            "steps_number",
        ]
        read_only_fields = fields


class _LoadDataSerializer(serializers.ModelSerializer):
    """
    荷载信息单向序列化
    """

    quasi_permanent_factor = serializers.FloatField(
        source="quasi_permanent_value_coefficient"
    )
    combined_factor = serializers.FloatField(source="combined_value_coefficient")

    class Meta:
        model = ModelConstructionData
        fields = [
            "live_load",
            "railing_load",
            "permanent_load_partial_factor",
            "live_load_load_partial_factor",
            "quasi_permanent_factor",
            "combined_factor",
            "reinforced_concrete_bulk_density",
        ]
        read_only_fields = fields


class _LimitSettingSerializer(serializers.ModelSerializer):
    class Meta:
        model = ModelConstructionData
        fields = [
            "crack",
        ]
        read_only_fields = fields


class _StairIdSerializer(serializers.ModelSerializer):
    """
    该序列化,特用于向下层楼梯结构计算数据作转换,变量名为适应下层变化,不要轻易变动
    """

    project_ID = serializers.CharField(source="project_num")
    stair_ID = serializers.CharField(source="component_num")

    class Meta:
        model = ModelConstructionData
        # 不可修改,对应下层stairID 内部属性的奇葩命名,应该用驼峰命名
        fields = [
            "project_ID",
            "stair_ID",
        ]


class StructureDataSerializer(serializers.ModelSerializer):
    """
    该序列化仅需要完成内部表结构到外层dataclass的转换,不需要考虑反向存储
    """

    material = serializers.SerializerMethodField()

    def get_material(self, obj: ModelConstructionData):
        """
        仅用于key value 的处理,不做强制类型转换

        Args:
            obj:

        Returns:

        """
        material_json = dict(_MaterialSerializer(instance=obj).data)
        return material_json

    stair_id = serializers.SerializerMethodField()

    def get_stair_id(self, obj):
        return dict(_StairIdSerializer(obj).data)

    construction = serializers.SerializerMethodField()

    def get_construction(self, obj: ModelConstructionData):
        return dict(_StructureSerializer(instance=obj).data)

    geometric = serializers.SerializerMethodField()

    def get_geometric(self, obj: ModelConstructionData):
        return dict(_GeometricSerializer(instance=obj).data)

    load_data = serializers.SerializerMethodField()

    def get_load_data(self, obj: ModelConstructionData):
        return dict(_LoadDataSerializer(instance=obj).data)

    limit_setting = serializers.SerializerMethodField()

    def get_limit_setting(self, obj):
        return dict(_LimitSettingSerializer(instance=obj).data)

    class Meta:
        model = ModelConstructionData
        fields = [
            "material",
            "stair_id",
            "construction",
            "geometric",
            "load_data",
            "limit_setting",
        ]


class ResultBothWaySerializer(serializers.ModelSerializer):
    as_1_actual = serializers.FloatField(source="as_fact_1")
    as_2_actual = serializers.FloatField(source="as_fact_2")
    as_3_actual = serializers.FloatField(source="as_fact_3")

    d_1_actual = serializers.FloatField(source="d_fact_1")
    d_2_actual = serializers.FloatField(source="d_fact_2")
    d_3_actual = serializers.FloatField(source="d_fact_3")

    spacing_1_actual = serializers.FloatField(source="spacing_fact_1")
    spacing_2_actual = serializers.FloatField(source="spacing_fact_2")
    spacing_3_actual = serializers.FloatField(source="spacing_fact_3")

    class Meta:
        model = ModelConstructionResult
        fields = [
            # start 楼梯几何参数
            "steps_h",
            "steps_b",
            "l0",
            "cos",
            # 荷载计算
            "gkt",
            "gk",
            "png",
            "pnl",
            "pm",
            # 正截面受弯承载力
            "m_max",
            "h0",
            "alpha_s",
            "ksi",
            "ksi_b",
            "ksi_status",
            "p_c_min",
            "p_c",
            "as_1",
            "as_2",
            "as_3",
            "as_1_actual",
            "as_2_actual",
            "as_3_actual",
            "d_1_actual",
            "d_2_actual",
            "d_3_actual",
            "spacing_1_actual",
            "spacing_2_actual",
            "spacing_3_actual",
            # 挠度
            "mq",
            "sigma_sq",
            "a_te",
            "p_te",
            "fi_i",
            "fi",
            "alpha_e",
            "gama_f",
            "p_t",
            "b_s",
            "b_l",
            "theta",
            "m_theta",
            "deflection_maxk",
            "deflection_limit",
            "deflection_status",
            "fi_w_i",
            "fi_w",
            # 裂缝
            "v_i",
            "rebar_n",
            "c_s",
            "p_te_w",
            "d_eq",
            "crack_max",
            "crack_status",
        ]
