"""
Date&Time           2022/8/5 15:48
Author              HaoLan

"""
from django.urls import path

from design import views
from .views import StructureDownloadView, ViewDesignDownload

app_name = "design"

urlpatterns = [
    path(
        "structure_book/<int:row_id>/StructureBook.docx",
        StructureDownloadView.as_view(),
        name="structure_book",
    ),
    path(
        "design_book/<int:row_id>/DesignBook.docx",
        ViewDesignDownload.as_view(),
        name="design_book",
    ),
    path(
        "structure",
        views.StructureListAPI.as_view({"get": "list"}),
        name="structure_list",
    ),
]
