# Generated by Django 3.2 on 2023-03-31 06:42

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0014_auto_20230329_1541"),
    ]

    operations = [
        migrations.AlterField(
            model_name="rebarlayoutmodel",
            name="content",
            field=models.JSONField(blank=True, null=True, verbose_name="钢筋数据"),
        ),
    ]
