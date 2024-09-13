"""
This file was generated with the customdashboard management command and
contains the class for the main dashboard.

To activate your index dashboard add the following to your settings.py::
    GRAPPELLI_INDEX_DASHBOARD = 'stair_web_backend.dashboard.CustomIndexDashboard'
"""

from django.utils.translation import gettext_lazy as _
from django.urls import reverse

from grappelli.dashboard import modules, Dashboard, DefaultIndexDashboard
from grappelli.dashboard.utils import get_admin_site_name


class CustomIndexDashboard(Dashboard):
    """
    Custom index dashboard for www.
    """

    def init_with_context(self, context):
        # append a group for "Administration" & "Applications"
        request = context["request"]
        if request.user.is_superuser:
            self.children.append(
                modules.Group(
                    _("超级管理员"),
                    column=1,
                    collapsible=True,
                    children=[
                        modules.AppList(
                            _("Administration"),
                            column=1,
                            collapsible=True,
                            models=("django.contrib.*", "design.models.*"),
                        ),
                    ],
                )
            )

        # append another link list module for "support".
        self.children.append(
            modules.LinkList(
                _("Support"),
                column=2,
                children=[
                    {
                        "title": _("Django Documentation"),
                        "url": "http://docs.djangoproject.com/",
                        "external": True,
                    },
                    {
                        "title": _("Grappelli Documentation"),
                        "url": "http://packages.python.org/django-grappelli/",
                        "external": True,
                    },
                    {
                        "title": _("Grappelli Google-Code"),
                        "url": "http://code.google.com/p/django-grappelli/",
                        "external": True,
                    },
                ],
            )
        )

        # append a recent actions module
        self.children.append(
            modules.RecentActions(
                _("Recent actions"),
                limit=5,
                collapsible=False,
                column=3,
            )
        )
