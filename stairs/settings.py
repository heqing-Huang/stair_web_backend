"""
Django settings for stairs project.

Generated by 'django-admin startproject' using Django 3.2.

For more information on this file, see
https://docs.djangoproject.com/en/3.2/topics/settings/

For the full list of settings and their values, see
https://docs.djangoproject.com/en/3.2/ref/settings/
"""
import os
import warnings
from pathlib import Path

import dotenv

ENV_FILE = ".env"
if not os.path.exists(ENV_FILE):
    warnings.warn(
        f".env 文件未配置, 将载入配置模板 .env.template, 请复制配置模板内容至" f" .env 文件, 并修改对应的设置"
    )
    ENV_FILE = ".env.template"

dotenv.load_dotenv()

# Build paths inside the project like this: BASE_DIR / 'subdir'.
BASE_DIR = Path(__file__).resolve().parent.parent

# Quick-start development settings - unsuitable for production
# See https://docs.djangoproject.com/en/3.2/howto/deployment/checklist/

# SECURITY WARNING: keep the secret key used in production secret!
SECRET_KEY = "django-insecure-n%4!e+x_bw&u01h70unzi&db$$@4&fhj3qvoyz2mj9fl0v)6zi"

# SECURITY WARNING: don't run with debug turned on in production!
DEBUG = True

ALLOWED_HOSTS = []

# Application definition

INSTALLED_APPS = [
    "grappelli",
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "django.contrib.admindocs",
    #
    "whitenoise.runserver_nostatic",
    #
    "rest_framework",
    #
    "design",
]

MIDDLEWARE = [
    "django.middleware.security.SecurityMiddleware",
    "django.contrib.sessions.middleware.SessionMiddleware",
    "django.middleware.common.CommonMiddleware",
    "django.middleware.csrf.CsrfViewMiddleware",
    "django.contrib.auth.middleware.AuthenticationMiddleware",
    "django.contrib.messages.middleware.MessageMiddleware",
    "django.middleware.clickjacking.XFrameOptionsMiddleware",
    "django.contrib.admindocs.middleware.XViewMiddleware",
    #
    "whitenoise.middleware.WhiteNoiseMiddleware",
]

ROOT_URLCONF = "stairs.urls"

TEMPLATES = [
    {
        "BACKEND": "django.template.backends.django.DjangoTemplates",
        "DIRS": [],
        "APP_DIRS": True,
        "OPTIONS": {
            "context_processors": [
                "django.template.context_processors.debug",
                "django.template.context_processors.request",
                "django.contrib.auth.context_processors.auth",
                "django.contrib.messages.context_processors.messages",
            ],
        },
    },
]

WSGI_APPLICATION = "stairs.wsgi.application"

# Database

DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.sqlite3",
        "NAME": os.path.join(BASE_DIR, "db.sqlite3"),
    }
}

# Password validation
# https://docs.djangoproject.com/en/3.2/ref/settings/#auth-password-validators

AUTH_PASSWORD_VALIDATORS = [
    {
        "NAME": "django.contrib.auth.password_validation.UserAttributeSimilarityValidator",
    },
    {
        "NAME": "django.contrib.auth.password_validation.MinimumLengthValidator",
    },
    {
        "NAME": "django.contrib.auth.password_validation.CommonPasswordValidator",
    },
    {
        "NAME": "django.contrib.auth.password_validation.NumericPasswordValidator",
    },
]

# Internationalization
# https://docs.djangoproject.com/en/3.2/topics/i18n/

LANGUAGE_CODE = "zh-hans"

TIME_ZONE = "Asia/Shanghai"

USE_I18N = True

USE_L10N = True

USE_TZ = True

# Static files (CSS, JavaScript, Images)
# https://docs.djangoproject.com/en/3.2/howto/static-files/

STATIC_URL = "/static/"
STATICFILES_DIRS = [os.path.join(BASE_DIR, "static")]
# Default primary key field type
# https://docs.djangoproject.com/en/3.2/ref/settings/#default-auto-field

DEFAULT_AUTO_FIELD = "django.db.models.BigAutoField"

# 日志配置格式
STANDARD_FORMAT = (
    "[%(asctime)s][%(threadName)s:%(thread)d]"
    "[task_id:%(name)s][%(filename)s:%(lineno)d]"
    "[%(levelname)s]:%(message)s"
)
# 配置logging
BASE_LOG_CONFIG = {
    "handlers": ["dev"] if DEBUG else ["default"],
    "level": "DEBUG" if DEBUG else "INFO",
    "propagate": True,
}

LOGGING = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "standard": {
            "format": STANDARD_FORMAT,
        },
        "simple": {
            "format": "[%(levelname)s][%(asctime)s][%(filename)s:%(lineno)d]%(message)s",
        },
    },
    "filters": {},
    "handlers": {
        "default": {  # 默认的部署
            "level": "INFO",
            "class": "logging.StreamHandler",
            "formatter": "standard",
        },
        "dev": {  # 开发期间
            "level": "DEBUG",
            "class": "logging.StreamHandler",
            "formatter": "standard",
        },
    },
    "loggers": {
        "django": {
            "handlers": ["dev"] if DEBUG else ["default", "mail_admins"],
            "level": "WARNING" if DEBUG else "ERROR",
            "propagate": True,
        },
        "design": BASE_LOG_CONFIG,
    },
}
STATIC_ROOT = "statics"

MEDIA_URL = "/media/"
MEDIA_ROOT = "media"

# celery 相关配置
RABBITMQ_HOSTS = os.environ.get("STAIRS_SERVER_RABBITMQ_HOSTS", "127.0.0.1")
RABBITMQ_PORT = os.environ.get("STAIRS_SERVER_RABBITMQ_PORT", "5672")
RABBITMQ_VHOST = os.environ.get("STAIRS_SERVER_RABBITMQ_VHOST", "/")
RABBITMQ_USER = os.environ["STAIRS_SERVER_RABBITMQ_USER"]
RABBITMQ_PWD = os.environ["STAIRS_SERVER_RABBITMQ_PWD"]
# broker_url
BROKER_URL = f"amqp://{RABBITMQ_USER}:{RABBITMQ_PWD}@{RABBITMQ_HOSTS}:{RABBITMQ_PORT}/{RABBITMQ_VHOST}"
CELERYBEAT_SCHEDULER = BROKER_URL
# 强行以 json
CELERY_ACCEPT_CONTENT = ["json"]
CELERY_TASK_SERIALIZER = "json"
CELERY_RESULT_SERIALIZER = "json"
RESULT_SERIALIZER = "json"
# 并发限制
CELERYD_CONCURRENCY = 1
CELERY_WORKER_CONCURRENCY = 1
CELERYD_PREFETCH_MULTIPLIER = 1
CELERYD_MAX_TASKS_PER_CHILD = 1
# 指定exchange 和默认队列
CELERY_DEFAULT_QUEUE = "stair_web_backend"
CELERY_ENABLED = False

# django-grappelli 定制配置
GRAPPELLI_ADMIN_TITLE = "中建科技-楼梯深化设计"
GRAPPELLI_INDEX_DASHBOARD = {  # alternative method
    "django.contrib.admin.site": "stairs.dashboard.BrowseDashboard" ,
}
