from rest_framework import serializers

from design import models


class Structure(serializers.ModelSerializer):
    class Meta:
        model = models.ModelConstructionData
        fields = [
            "id",
            "project_num",
            "component_num",
            "crack",
            "weight",
            "top_top_length",
            "bottom_top_length"
        ]

class StructurePreSet(serializers.ModelSerializer):
    class Meta:
        model = models.PreSetModelData
        fields = [
            "id",
            "project_num",
            "component_num",
            "remark_name",
            "rebar_name",
            "concrete_grade",
            "protective_layer_thickness",
            "longitudinal_top_stress_bar_margin",
            # 几何参数
            "weight",
            "height",
            "thickness",
            "clear_span",
            "steps_number",
            "top_top_length",
            "bottom_top_length",
            # 荷载
            "live_load",
            "railing_load",
            "permanent_load_partial_factor",
            "live_load_load_partial_factor",
            "quasi_permanent_value_coefficient",
            "combined_value_coefficient",
            "reinforced_concrete_bulk_density",
            # 限制设置
            "crack",
            # 深化设计相关参数
            "top_thickness",
            "bottom_thickness",
            "top_b",
            "bottom_b",
        ]
