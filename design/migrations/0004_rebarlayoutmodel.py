# Generated by Django 3.2 on 2022-10-14 03:31

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0003_modeldetailedresult_construction_detailed"),
    ]

    operations = [
        migrations.CreateModel(
            name="RebarLayoutModel",
            fields=[
                (
                    "id",
                    models.BigAutoField(
                        auto_created=True,
                        primary_key=True,
                        serialize=False,
                        verbose_name="ID",
                    ),
                ),
                (
                    "content",
                    models.TextField(blank=True, null=True, verbose_name="钢筋数据"),
                ),
                (
                    "stair",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        to="design.modelconstructiondata",
                        verbose_name="所属楼梯",
                    ),
                ),
            ],
            options={
                "verbose_name": "钢筋排布",
                "verbose_name_plural": "钢筋排布",
            },
        ),
    ]
