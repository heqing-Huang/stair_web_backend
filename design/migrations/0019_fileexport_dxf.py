# Generated by Django 3.2 on 2023-04-11 03:26

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0018_auto_20230410_1233"),
    ]

    operations = [
        migrations.AddField(
            model_name="fileexport",
            name="dxf",
            field=models.FileField(
                null=True, upload_to="%Y/%m/%d", verbose_name="Cad Dxf"
            ),
        ),
    ]
