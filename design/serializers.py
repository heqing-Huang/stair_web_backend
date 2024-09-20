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
