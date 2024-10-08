import os

from celery import Celery
from django.conf import settings

# set the default Django settings module for the 'celery' program.
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "stairs.settings")
app = Celery()
app.config_from_object("django.conf:settings")
app.autodiscover_tasks(settings.INSTALLED_APPS)
