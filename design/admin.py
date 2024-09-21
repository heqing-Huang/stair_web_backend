import os
import logging

from django.urls import reverse
from django.contrib import admin
from django.contrib.admin import helpers
from django.utils.safestring import mark_safe
from django.contrib.admin.models import LogEntry
from django.template.loader import render_to_string

# Register your models here.
from . import tasks
from .tools import call_structural_calculation, call_detailed_design_by_model
from . import models
from .models import (
    ModelConstructionData,
    ModelConstructionResult,
    DetailData,
    ModelDetailedResult,
    RebarLayoutModel,

    PreSetModelData,
)

_logger = logging.getLogger(__name__)

# 登录界面标题
admin.site.site_header = '复杂预制构件智能深化设计系统'
# 网页标题
admin.site.site_title = '复杂预制构件智能深化设计系统'
# 后台主页标题
admin.site.index_title = '站点管理'

class NestedFieldSet:
    """
    自定义嵌套分组字段渲染过程
    """

    def __init__(
        self,
        name,
        fieldset,
    ):
        self.name = name
        self.fieldset = fieldset


class ConstructionResultInline(admin.StackedInline):
    max_num = 1
    model = ModelConstructionResult
    readonly_fields = ("count_book_export",)

    def get_fieldsets(self, request, obj=None):
        if obj is not None:
            query_set = self.get_queryset(request)
            if query_set.count() > 0:
                in_line_obj: ModelConstructionResult = query_set.first()
                if in_line_obj.success:
                    return [
                        (
                            "关键参数",
                            {
                                "fields": [
                                    "ksi_status",
                                    "deflection_status",
                                    "crack_status",
                                ]
                            },
                        ),
                        ("文件导出", {"fields": ["count_book_export"]}),
                    ]
                else:
                    return [
                        (
                            "错误提示",
                            {
                                "fields": ["success", "message"],
                            },
                        ),
                    ]

    can_delete = False

    def has_change_permission(self, request, obj=None):
        return False

    @admin.display(description="结构计算书")
    def count_book_export(self, obj):
        if obj is None or obj.id is None:
            return None
        else:
            data = {
                "link_url": reverse("design:structure_book", kwargs={"row_id": obj.id})
            }
            string_html = render_to_string(
                os.path.join("design", "export", "structure_book.html"),
                context=data,
            )
            return string_html


class CustomFieldSetAdmin(admin.ModelAdmin):
    """
    抽象可复用模块
    """

    custom_fieldset = None
    add_form_template = "admin/design/add_form.html"
    change_form_template = add_form_template

    def get_custom_fieldset(self):
        return self.custom_fieldset or ["__all__"]

    def get_fieldsets(self, request, obj=None):
        """
        将原本fieldsets 设置方式,改造成为从custom_fieldsets 中提取
        """
        _return_data = []
        if self.custom_fieldset:
            for row in self.custom_fieldset:
                for column in row:
                    for item in column:
                        _return_data.append(item)
        else:
            return [(None, {"fields": self.get_fields(request, obj)})]
        return _return_data

    def _get_custom_fields(self, context):
        """
        构建自定义的 fieldset 外层组装结构, 通过div浮动实现左右布局
        """
        adminform: helpers.AdminForm = context["adminform"]
        form = adminform.form

        custom_fieldsets_objs = []
        styles = ["left", "right"]
        if self.custom_fieldset:
            for row in self.custom_fieldset:
                mid_row = []
                if len(row) == 1:
                    width = "100%"
                else:
                    width = "50%"

                for index, column in enumerate(row):
                    column_obj = []
                    for name, options in column:
                        fieldset = helpers.Fieldset(
                            form,
                            name,
                            readonly_fields=adminform.readonly_fields,
                            model_admin=adminform.model_admin,
                            **options
                        )
                        column_obj.append(fieldset)
                    mid_row.append(
                        {
                            "width": width,
                            "column": column_obj,
                            "float": styles[index],
                        }
                    )
                custom_fieldsets_objs.append(mid_row)
        return custom_fieldsets_objs

    def render_change_form(
        self, request, context, add=False, change=False, form_url="", obj=None
    ):
        """
        继承add & change 页面,在上下文中Context 中额外提供参数
        Args:
            request:
            context:
            add:
            change:
            form_url:
            obj:

        Returns:

        """

        custom_fieldsets = self._get_custom_fields(context)
        context.update({"custom_fieldsets": custom_fieldsets})
        return super().render_change_form(request, context, add, change, form_url, obj)

class PreCustomFieldSetAdmin(admin.ModelAdmin):
    custom_fieldset = None
    add_form_template = "admin/design/add_preset_model.html"
    change_form_template = add_form_template

    def get_custom_fieldset(self):
        return self.custom_fieldset or ["__all__"]

    def get_fieldsets(self, request, obj=None):
        """
        将原本fieldsets 设置方式,改造成为从custom_fieldsets 中提取
        """
        _return_data = []
        if self.custom_fieldset:
            for row in self.custom_fieldset:
                for column in row:
                    for item in column:
                        _return_data.append(item)
        else:
            return [(None, {"fields": self.get_fields(request, obj)})]
        return _return_data

    def _get_custom_fields(self, context):
        """
        构建自定义的 fieldset 外层组装结构, 通过div浮动实现左右布局
        """
        adminform: helpers.AdminForm = context["adminform"]
        form = adminform.form

        custom_fieldsets_objs = []
        styles = ["left", "right"]
        if self.custom_fieldset:
            for row in self.custom_fieldset:
                mid_row = []
                if len(row) == 1:
                    width = "100%"
                else:
                    width = "50%"

                for index, column in enumerate(row):
                    column_obj = []
                    for name, options in column:
                        fieldset = helpers.Fieldset(
                            form,
                            name,
                            readonly_fields=adminform.readonly_fields,
                            model_admin=adminform.model_admin,
                            **options
                        )
                        column_obj.append(fieldset)
                    mid_row.append(
                        {
                            "width": width,
                            "column": column_obj,
                            "float": styles[index],
                        }
                    )
                custom_fieldsets_objs.append(mid_row)
        return custom_fieldsets_objs

    def render_change_form(
        self, request, context, add=False, change=False, form_url="", obj=None
    ):
        """
        继承add & change 页面,在上下文中Context 中额外提供参数
        Args:
            request:
            context:
            add:
            change:
            form_url:
            obj:

        Returns:

        """

        custom_fieldsets = self._get_custom_fields(context)
        context.update({"custom_fieldsets": custom_fieldsets})
        return super().render_change_form(request, context, add, change, form_url, obj)

@admin.register(ModelConstructionData)
class AdminPageConstructionData(CustomFieldSetAdmin):
    list_display = [
        "remark_name",
    ]
    readonly_fields = ["up_schematic_diagram", "front_schematic_diagram"]

    # 针对两列布局所作

    custom_fieldset = [
        [
            [("预设模型", {"fields": ["pre_stair"]})]
        ],
        # row
        [   # column
            [("项目", {"fields": ["project_num", "component_num"]})],
            [("楼梯", {"fields": ["remark_name"]})],
        ],
        # row
        [
            [("", {"fields": ["front_schematic_diagram"]})],
            [("", {"fields": ["up_schematic_diagram"]})],
        ],
        # row
        [
            [("几何参数", {"fields": ["height",
                                    "weight",
                                    "top_top_length",
                                    "steps_number"]})],
            [(".", {"fields": ["thickness",
                               "clear_span",
                               "bottom_top_length"]})],
        ],
        # row
        [
            [("荷载参数", {"fields": [
                            "live_load",
                            "railing_load",
                            "permanent_load_partial_factor",
                            "live_load_load_partial_factor",
                            "quasi_permanent_value_coefficient",
                            "combined_value_coefficient",
                            "reinforced_concrete_bulk_density",
                        ]
                    },
                )
            ],
            [
                ("材料参数", {"fields": ["rebar_name", "concrete_grade"]}),
                ("构造要求", {"fields": ["protective_layer_thickness",
                                        "longitudinal_top_stress_bar_margin"]}),
                ("限制条件", {"fields": ["crack"]}),
            ],
        ],
    ]

    @admin.display(description="俯视图")
    def up_schematic_diagram(self, _):
        """
        返回编辑页面的示意图部分,无交互意义
        Returns:

        """

        string_html = render_to_string(
            "admin/design/modelconstructiondata/up_schematic_diagram.html"
        )
        return mark_safe(string_html)

    @admin.display(description="正视图")
    def front_schematic_diagram(self, _):
        """
        返回编辑页面的示意图部分,无交互意义
        Returns:

        """

        string_html = render_to_string(
            "admin/design/modelconstructiondata/front_schematic_diagram.html"
        )
        return mark_safe(string_html)

    def get_inlines(self, request, obj):
        if obj is None:
            return []
        if ModelConstructionResult.objects.filter(construction=obj).count() == 0:
            return []

        return super(AdminPageConstructionData, self).get_inlines(request, obj)

    def save_model(self, request, obj, form, change):
        # 首先执行保存
        data_back = super(AdminPageConstructionData, self).save_model(
            request, obj, form, change
        )
        instance: ModelConstructionResult
        if obj:
            call_structural_calculation(obj)
        return data_back


@admin.register(ModelConstructionResult)
class AdminPageConstructionResult(admin.ModelAdmin):
    list_display = [
        "remark_name",
        "success",
        "crack_status",
        "deflection_status",
        "ksi_status",
        "count_book_export",
    ]

    @admin.display(description="导出")
    def count_book_export(self, obj: ModelConstructionResult):
        data = {
            "link_url": reverse("design:structure_book", kwargs={"row_id": obj.id}),
            "status": obj.success,
        }
        string_html = render_to_string(
            "design/export/export_button.html",
            context=data,
        )
        return string_html

    def get_fieldsets(self, request, obj=None):
        obj: ModelConstructionResult
        if obj.success:
            fieldsets = [
                ("楼梯几何参数", {"fields": ["steps_h", "steps_b", "l0", "cos"]}),
                ("荷载", {"fields": ["gkt", "gk", "png", "pnl", "pm"]}),
                (
                    "正截面受弯承载力",
                    {
                        "fields": [
                            "m_max",
                            "h0",
                            "alpha_s",
                            "ksi",
                            "ksi_status",
                            "ksi_b",
                            "p_c_min",
                            "p_c",
                            "as_1",
                            "as_2",
                            "as_3",
                            "as_fact_1",
                            "as_fact_2",
                            "as_fact_3",
                            "d_fact_1",
                            "d_fact_2",
                            "d_fact_3",
                            "spacing_fact_1",
                            "spacing_fact_2",
                            "spacing_fact_3",
                        ]
                    },
                ),
                (
                    "挠度",
                    {
                        "fields": [
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
                        ]
                    },
                ),
                (
                    "裂缝",
                    {
                        "fields": [
                            "v_i",
                            "rebar_n",
                            "c_s",
                            "p_te_w",
                            "d_eq",
                            "crack_max",
                            "crack_status",
                        ]
                    },
                ),
                ("状态控制", {"fields": ["success", "message"]}),
            ]
        else:
            fieldsets = ("状态控制", {"fields": ["success", "message"]})
        return fieldsets

    @admin.display(description="别名")
    def remark_name(self, obj: ModelConstructionResult):
        if obj.construction:
            return obj.construction.remark_name
        else:
            return None

    remark_name.short_description = "别名"


def detail_data_html_image(url, title):
    string_html = render_to_string(
        "admin/design/detaildata/geometric_schematic.html",
        {"image_url": url, "title": title},
    )
    return mark_safe(string_html)


@admin.register(DetailData)
class AdminDetailData(CustomFieldSetAdmin):
    list_display = ["stair"]
    readonly_fields = [
        "side_schematic_diagram",
        "up_schematic_diagram",
        "rebar_side",
        "rebar_up",
        "parts_up",
        "hoisting_1",
        "hoisting_2",
        "demold_show",
        "hanging_nails",
        "lgymj_show",
        "lgdw_show",
        "node_foot_show",
        "holes_show",
        "gdjzz_show",
        "hdjzz_show",
        "node_head_show",
        "fhc_up_show",
        "fhc_side_show",
        "dsc_show",
        "tx_show",
        "circle_show",
    ]
    custom_fieldset = [
        [
            [
                ("", {"fields": ["stair"]}),
            ]
        ],
        [
            [("几何参数", {"fields": ["side_schematic_diagram",
                                      "width",
                                      "top_to_length",
                                      "bottom_top_length"]})],
            [(".", {"fields": ["up_schematic_diagram",
                               "top_thickness",
                               "bottom_thickness",
                               "top_b",
                               "bottom_b",]})],
        ],
        # select 框
        [
            [
                ("钢筋深化设计", {"fields": ["rebar_design_mode"]})
            ],
        ],
        [
            [("示意图", {"fields": ["rebar_side"]})],
            [(".", {"fields": ["rebar_up"]})],
        ],
        [
            [
                (
                    "4号 底端边缘纵筋",
                    {
                        "fields": [
                            "bottom_edge_longitudinal_rebar_diameter",
                            "bottom_edge_longitudinal_rebar_spacing",
                        ]
                    },
                ),
            ],
            [
                (
                    "5号 顶端边缘纵筋",
                    {
                        "fields": [
                            "top_edge_longitudinal_rebar_diameter",
                            "top_edge_longitudinal_rebar_spacing",
                        ]
                    },
                ),
            ],
        ],
        [
            [
                (
                    "6号 底端边缘箍筋",
                    {
                        "fields": [
                            "bottom_edge_stirrup_diameter",
                            "bottom_edge_stirrup_spacing",
                        ]
                    },
                ),
            ],
            [
                (
                    "9号 顶端边缘箍筋",
                    {
                        "fields": [
                            "top_edge_stirrup_diameter",
                            "top_edge_stirrup_spacing",
                        ]
                    },
                ),
            ],
        ],
        [
            [
                ("7号 销键加强筋", {"fields": ["hole_reinforce_rebar_diameter"]}),
            ],
            [
                (
                    "8号 吊点加强筋",
                    {"fields": ["hoisting_reinforce_rebar_diameter"]},
                ),
            ],
        ],
        [
            [
                (
                    "10号 上部边缘加强筋",
                    {"fields": ["top_edge_reinforce_rebar_diameter"]},
                ),
            ],
            [
                (
                    "11号 下部边缘加强筋",
                    {"fields": ["bottom_edge_reinforce_rebar_diameter"]},
                ),
            ],
        ],
        # 吊装预埋件设计相关
        [[("吊装预埋件设计", {"fields": ["hoist_design_mode"]})]],
        [[("", {"fields": ["hoist_type", "hoist_name"]})]],
        [
            [
                (
                    "吊装预埋件位置参数",
                    {
                        "fields": [
                            ("hoist_position_a", "hoist_position_b"),
                            ("hoist_position_c", "hoist_position_d"),
                        ]
                    },
                ),
            ],
            [(".", {"fields": ["parts_up"]})],
        ],
        # 脱模预埋件设计
        [
            [
                (
                    "脱模预埋件设计",
                    {
                        "fields": [
                            "demold_design_mode",
                        ]
                    },
                )
            ]
        ],
        [
            [("", {"fields": ["pouring_way", "demold_type", "demold_name",]})]
        ],
        [
            [
                (
                    "脱模预埋件平面位置参数",
                    {
                        "fields": [
                            ("demold_position_a", "demold_position_b"),
                            ("demold_position_c", "demold_position_d"),
                            "demold_position_t"
                        ]
                    },
                )
            ],
            [(".", {"fields": ["demold_show"]})],
        ],
        # 栏杆预埋件设计
        [
            [("栏杆预埋件设计", {"fields": ["rail_design_mode"]})]
        ],
        [
            [("栏杆预埋件相关参数", {"fields": ["rail_layout", "rail_number", "rail_name"]})],
            [(".", {"fields": ["lgymj_show"]})],
        ],
        [
            [
                (
                    "栏杆预埋件位置参数",
                    {
                        "fields": [
                            "rail_position_a",
                            "rail_position_b",
                        ]
                    },
                ),
            ],
            [(".", {"fields": ["lgdw_show"]})],
        ],
        [
            [
                (
                    "孔洞设计",
                    {
                        "fields": [
                            "hole_design_mode",
                        ]
                    },
                )
            ]
        ],
        [
            [
                (
                    "孔洞位置",
                    {
                        "fields": [
                            (
                                "top_hole_position_a1",
                                "bottom_hole_position_a1",
                            ),
                            (
                                "top_hole_position_a2",
                                "bottom_hole_position_a2",
                            ),
                            (
                                "top_hole_position_b1",
                                "bottom_hole_position_b1",
                            ),
                            (
                                "top_hole_position_b2",
                                "bottom_hole_position_b2",
                            ),
                        ]
                    },
                )
            ],
            [
                (
                    ".",
                    {
                        "fields": [
                            "holes_show",
                        ]
                    },
                )
            ],
        ],
        [
            [
                (
                    "孔洞类型",
                    {
                        "fields": [
                            ("top_hole_type",
                            "bottom_hole_type")
                        ]
                    },
                )
            ]
        ],
        [
            [
                (
                    "固定铰支座孔洞示意图",
                    {
                        "fields": ["gdjzz_show"],
                    },
                )
            ],
            [
                (
                    "滑动铰支座孔洞示意图",
                    {
                        "fields": ["hdjzz_show"],
                    },
                )
            ],
        ],
        [
            [
                (
                    "顶端截面参数-固定铰支座",
                    {
                        "fields": [
                            (
                                "top_fix_hinge_c2",
                                "top_fix_hinge_d2",
                            )
                        ]
                    },
                )
            ],
            [
                (
                    "顶端截面参数-滑动铰支座",
                    {
                        "fields": [
                            (
                                "top_sliding_hinge_c1",
                                "top_sliding_hinge_d1",
                                "top_sliding_hinge_e1",
                            ),
                            ("top_sliding_hinge_f1", "top_sliding_hinge_h1"),
                        ]
                    },
                )
            ],
        ],
        [
            [
                (
                    "底端截面参数-固定铰支座",
                    {
                        "fields": [
                            (
                                "bottom_fix_hinge_c2",
                                "bottom_fix_hinge_d2",
                            )
                        ]
                    },
                )
            ],
            [
                (
                    "底端截面参数-滑动铰支座",
                    {
                        "fields": [
                            (
                                "bottom_sliding_hinge_c1",
                                "bottom_sliding_hinge_d1",
                                "bottom_sliding_hinge_e1",
                            ),
                            (
                                "bottom_sliding_hinge_f1",
                                "bottom_sliding_hinge_h1",
                            ),
                        ]
                    },
                )
            ],
        ],
        # 节点设计
        [
            [
                (
                    "节点设计",
                    {
                        "fields": [
                            "joint_design_mode",
                        ]
                    },
                )
            ]
        ],
        [
            [("节点示意图", {"fields": ["node_foot_show"]})],
            [(".", {"fields": ["node_head_show"]})],
        ],
        [
            [
                (
                    "底端节点外形尺寸",
                    {
                        "fields": [
                            (
                                "bottom_joint_a",
                                "bottom_joint_b",
                                "bottom_joint_c",
                            )
                        ]
                    },
                ),
            ],
            [
                (
                    "顶端节点外形尺寸",
                    {"fields": [("top_joint_a", "top_joint_b", "top_joint_c")]},
                ),
            ],
        ],
        # 防滑槽设计
        [
            [
                (
                    "防滑槽设计",
                    {
                        "fields": [
                            "step_slot_design_mode",
                        ]
                    },
                )
            ]
        ],
        [
            [
                (
                    "防滑槽位置参数",
                    {
                        "fields": [
                            (
                                "step_slot_position_c1",
                                "step_slot_position_c2",
                                "step_slot_position_c3",
                            ),
                        ]
                    },
                )
            ],
            [(".", {"fields": ["fhc_up_show"]})],
        ],
        [
            [
                (
                    "防滑槽形状参数",
                    {
                        "fields": [
                            ("step_slot_a", "step_slot_b", "step_slot_c"),
                            ("step_slot_d", "step_slot_e"),
                        ]
                    },
                )
            ],
            [
                (
                    ".",
                    {
                        "fields": [
                            "fhc_side_show",
                        ]
                    },
                )
            ],
        ],
        # 滴水槽设计
        [
            [
                (
                    "滴水槽设计",
                    {
                        "fields": [
                            "water_drip_design_mode",
                        ]
                    },
                ),
            ]
        ],
        [
            [
                (
                    "滴水槽位置参数",
                    {
                        "fields": [
                            "water_drip_layout",
                            "water_drip_position_a1",
                            "water_drip_position_a2",
                            "water_drip_position_a3",
                        ]
                    },
                )
            ],
            [(".", {"fields": ["dsc_show"]})],
        ],
        [
            [("滴水槽截面参数", {"fields": ["water_drip_shape"]})],
        ],
        [
            [("滴水槽示意图", {"fields": ["tx_show"]})],
            [(".", {"fields": ["circle_show"]})],
        ],
        [
            [
                (
                    "梯形截面参数",
                    {
                        "fields": [
                                "water_drip_trapezoid_a",
                                "water_drip_trapezoid_b",
                                "water_drip_trapezoid_c",
                        ]
                    },
                ),
            ],
            [
                (
                    "半圆截面",
                    {
                        "fields": [
                                "water_drip_semicircle_a",
                                "water_drip_semicircle_b",
                        ]
                    },
                )
            ],
        ],
    ]

    @admin.display(description="")
    def circle_show(self, _):
        return detail_data_html_image("design/img/water/circle.png", "半圆形截面示意图")

    @admin.display(description="")
    def tx_show(self, _):
        return detail_data_html_image("design/img/water/retangle.png", "梯形截面示意图")

    @admin.display(description="")
    def dsc_show(self, _):
        return detail_data_html_image("design/img/water/show.png", "滴水槽布局示意图")

    @admin.display(description="")
    def fhc_up_show(self, _):
        return detail_data_html_image("design/img/anti_skid_groove/up.png", "防滑槽平面示意图")

    @admin.display(description="")
    def fhc_side_show(self, _):
        return detail_data_html_image(
            "design/img/anti_skid_groove/side.png", "防滑槽剖面示意图"
        )

    @admin.display(description="")
    def lgdw_show(self, _):
        return detail_data_html_image("design/img/parts/lgdw_show.png", "栏杆定位平面图")

    @admin.display(description="")
    def gdjzz_show(self, _):
        return detail_data_html_image("design/img/holes/gdjzz.png", "固定铰支座示意图")

    @admin.display(description="")
    def hdjzz_show(self, _):
        return detail_data_html_image("design/img/holes/hdjzz.png", "滑动铰支座示意图")

    @admin.display(description="")
    def holes_show(self, _):
        return detail_data_html_image("design/img/holes/holes.png", "孔洞位置示意图")

    @admin.display(description="")
    def node_foot_show(self, _):
        return detail_data_html_image("design/img/nodes/foot.png", "底端安装节点示意图")

    @admin.display(description="")
    def node_head_show(self, _):
        return detail_data_html_image("design/img/nodes/head.png", "顶端安装节点示意图")

    @admin.display(description="")
    def lgymj_show(self, _):
        return detail_data_html_image("design/img/parts/lgymj_show.png", "栏杆预埋件布局示意图")

    @admin.display(description="")
    def hanging_nails(self, _):
        return detail_data_html_image("design/img/parts/hanging_nails.png", "吊钉侧面示意图")

    def save_model(self, request, obj, form, change):
        result = super(AdminDetailData, self).save_model(request, obj, form, change)
        obj: DetailData
        detailed_result_row = call_detailed_design_by_model(obj)
        tasks.total_back_handle.apply_async((detailed_result_row.id,))
        return result

    @admin.display(description="")
    def demold_show(self, _):
        string_html = render_to_string(
            "admin/design/detaildata/geometric_schematic.html",
            {
                "image_url": "design/img/parts/demold_show.png",
                "title": "脱模预埋件示意图",
            },
        )
        return mark_safe(string_html)

    @admin.display(description="侧面图")
    def side_schematic_diagram(self, _):
        string_html = render_to_string(
            "admin/design/detaildata/geometric_schematic.html",
            {"image_url": "design/img/detail_data/side.png", "title": "侧面图"},
        )
        return mark_safe(string_html)

    @admin.display(description="俯视图")
    def up_schematic_diagram(self, _):
        string_html = render_to_string(
            "admin/design/detaildata/geometric_schematic.html",
            {"image_url": "design/img/detail_data/up.png", "title": "俯视图"},
        )
        return mark_safe(string_html)

    @admin.display(description="钢筋侧面示意图")
    def rebar_side(self, _):
        return detail_data_html_image("design/img/rebars/side.png", "钢筋侧面示意图")

    @admin.display(description="钢筋俯视面示意图")
    def rebar_up(self, _):
        return detail_data_html_image("design/img/rebars/up.png", "钢筋俯视面示意图")

    @admin.display(description="示意图")
    def parts_up(self, _):
        return detail_data_html_image("design/img/parts/up.png", "示意图")

    @admin.display(description="")
    def hoisting_1(self, _):
        return detail_data_html_image("design/img/parts/hoisting_1.png", "预埋吊钉企口示意图")

    @admin.display(description="")
    def hoisting_2(self, _):
        return detail_data_html_image("design/img/parts/hoisting_2.png", "预埋锚栓企口示意图")


@admin.register(ModelDetailedResult)
class AdminPageModelDetailedResult(admin.ModelAdmin):
    list_display = [
        "stair",
        "detailed_design_book_download",
    ]

    @admin.display(description="计算书")
    def detailed_design_book_download(self, obj: ModelDetailedResult):
        data = {"link_url": reverse("design:design_book", kwargs={"row_id": obj.id})}
        string_html = render_to_string("design/export/design_book.html", data)
        return mark_safe(string_html)


@admin.register(RebarLayoutModel)
class RebarLayoutModelAdminPage(admin.ModelAdmin):
    list_display = [
        "stair",
    ]


@admin.register(models.FileExport)
class FileExportAdminPage(admin.ModelAdmin):
    list_display = [
        "stair",
        "ifc",
        "bvbs",
        "zip_json",
        "dxf",
    ]

@admin.register(PreSetModelData)
class PreSetModelDataAdmin(PreCustomFieldSetAdmin):
    list_display = ["remark_name"]
    readonly_fields = ["up_schematic_diagram", "front_schematic_diagram"]
    custom_fieldset = [
        # row
        [  # column
            [("项目", {"fields": ["project_num", "component_num"]})],
            [("楼梯", {"fields": ["remark_name"]})],
        ],
        # row
        [
            [("", {"fields": ["front_schematic_diagram"]})],
            [("", {"fields": ["up_schematic_diagram"]})],
        ],
        # row
        [
            [("几何参数", {"fields": ["height",
                                      "weight",
                                      "top_top_length",
                                      "steps_number"]})],
            [(".", {"fields": ["thickness",
                               "clear_span",
                               "bottom_top_length"]})],
        ],
        # row
        [
            [("荷载参数", {"fields": [
                "live_load",
                "railing_load",
                "permanent_load_partial_factor",
                "live_load_load_partial_factor",
                "quasi_permanent_value_coefficient",
                "combined_value_coefficient",
                "reinforced_concrete_bulk_density"]})
            ],
            [
                ("材料参数", {"fields": ["rebar_name", "concrete_grade"]}),
                ("构造要求", {"fields": ["protective_layer_thickness",
                                         "longitudinal_top_stress_bar_margin"]}),
                ("限制条件", {"fields": ["crack"]}),
            ],
        ],
        [
            [
                ("深化设计参数", {"fields": [
                    "top_thickness",
                    "bottom_thickness",
                    "top_b",
                    "bottom_b",
                ]})
            ]
        ],
    ]

    @admin.display(description="俯视图")
    def up_schematic_diagram(self, _):
        string_html = render_to_string(
            "admin/design/modelconstructiondata/up_schematic_diagram.html"
        )
        return mark_safe(string_html)

    @admin.display(description="正视图")
    def front_schematic_diagram(self, _):
        string_html = render_to_string(
            "admin/design/modelconstructiondata/front_schematic_diagram.html"
        )
        return mark_safe(string_html)
