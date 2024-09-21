# Generated by Django 4.2.16 on 2024-09-20 06:51

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0031_presetmodeldata"),
    ]

    operations = [
        migrations.AddField(
            model_name="modelconstructiondata",
            name="pre_stair",
            field=models.ForeignKey(
                null=True,
                on_delete=django.db.models.deletion.SET_NULL,
                to="design.presetmodeldata",
                verbose_name="预设参数",
            ),
        ),
    ]
