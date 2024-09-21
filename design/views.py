from django.http import HttpResponse
from django.views import View
from rest_framework import pagination
from rest_framework.viewsets import ModelViewSet
from .tools import api_to_word, call_design_book
from design import models, serializers
from .models import ModelConstructionResult, ModelDetailedResult, ModelConstructionData, PreSetModelData


from django.shortcuts import render


# Create your views here.
class StructureDownloadView(View):
    def get(self, request, row_id):
        """

        :param request:
        :param row_id: row_id 为结构计算结果的ID
        :return:
        """
        bytes_io = api_to_word(ModelConstructionResult.objects.get(id=row_id))
        response = HttpResponse(
            bytes_io.read(), content_type="application/docx", charset="utf-8"
        )
        response["Content-Dispositon"] = "attachment; filename=StructureBook.docx"
        response["Access-Control-Allow-Origin"] = "*"
        return response


class ViewDesignDownload(View):
    def get(self, request, row_id):
        bytes_io = call_design_book(ModelDetailedResult.objects.get(id=row_id))
        response = HttpResponse(
            bytes_io.read(), content_type="application/docx", charset="utf-8"
        )
        response["Content-Dispositon"] = "attachment; filename=DetaildDesign.docx"
        response["Access-Control-Allow-Origin"] = "*"
        return response


class DefaultLimitOffsetPagination(pagination.LimitOffsetPagination):
    max_limit = 20
    default_limit = 10


class StructureListAPI(ModelViewSet):
    queryset = models.ModelConstructionData.objects.all()
    pagination_class = DefaultLimitOffsetPagination
    permission_classes = []
    serializer_class = serializers.Structure

    def list(self, request, *args, **kwargs):
        response = super().list(request, *args, **kwargs)
        response.headers.setdefault("Access-Control-Allow-Origin", "*")
        return response

class PreSetModelAPI(ModelViewSet):
    queryset = models.PreSetModelData.objects.all()
    pagination_class = DefaultLimitOffsetPagination
    permission_classes = []
    serializer_class = serializers.StructurePreSet

    def list(self, request, *args, **kwargs):
        response = super().list(request, *args, **kwargs)
        response.headers.setdefault("Access-Control-Allow-Origin", "*")
        return response
